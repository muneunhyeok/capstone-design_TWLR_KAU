// twlr_bridge.ino (v3) - TWLR Teensy 4.1
//   CAN1 (1 Mbps) 하나에: 힙 DM4340P x2 (ID 0x01/0x02, MIT, 500 Hz 홀드) + ODrive 3.6 (노드 3 = 왼쪽 axis0, 노드 4 = 오른쪽 axis1)
//   BNO085 IMU (I2C)
//   밸런싱 제어기 (250 Hz) 가 Teensy 안에서 돈다. Khadas 파이썬은 키보드/로그/게인 설정만.
//   ('B' 를 안 보내면 v2 처럼 힙+IMU 브리지로만 동작 -> 예전 파이썬 USB 바퀴 방식도 그대로 쓸 수 있음)
//
// Teensy -> PC (250 Hz) : A5 5A | payload 104 B | sum8(payload)
//   payload = <uint32 t_us, float f[24], uint8 flags, imu_ok, bal_mode, bal_err>  (104 B)
//   f[0..5]   roll, pitch, yaw, gx(roll율), gy(pitch율), gz   [rad, rad/s]  (센서 X축 회전 = 몸통 pitch, pitch<->gy 짝)
//   f[6..11]  hipL, hipR [deg, h 기준], hipVL, hipVR [deg/s], hipTL, hipTR [N*m]   (dir 적용 = 다리 기준)
//   f[12..14] loopHz, imuUsMax, imuEvHz (1초 진단)
//   f[15..21] th_com [rad], w_com [rad/s], v_meas [m/s], v_cmd [m/s], x [m], th_ref [rad], trim [deg]
//   f[22..23] ODrive 하트비트 수신 Hz, 엔코더 응답 수신 Hz (두 바퀴 합, 정상 ~20 / ~500)
//   flags: bit0 L_en, bit1 R_en, bit2 hold(둘다), bit3 힙fault, bit4 ODrv L 살아있음, bit5 ODrv R 살아있음, bit6 L closed-loop, bit7 R closed-loop
//   bal_mode: 0 IDLE, 1 BALANCE, 2 STOP(에러로 멈춤)      bal_err: 아래 ERR_*
// PC -> Teensy : 5A A5 | <char cmd, float a, float b> | sum8
//   'E' 힙 FREE   'H' 지금 자세로 홀드   'D' 전부 끄기 (힙 + 바퀴)   'K' kp kd   'T' tffL tffR   'W' heartbeat
//   'P' idx val  파라미터 설정 (아래 PRM_*)       'B' a  밸런싱 시작(a=1)/정지(a=0)
//   'V' v_ref[m/s] yaw_ref[rad/s]                 'Z' 지금 자세를 균형점으로 (trim 자동)    'R' trim_deg 직접 설정
// PC heartbeat 가 300 ms 끊기면: tff 0 + 밸런싱 정지(바퀴 IDLE). 힙 홀드는 유지.
//
// ODrive 0.5.1 CANSimple: ID = node<<5 | cmd.  Heartbeat(0x01) 100 ms 자동. 엔코더(0x09)는 RTR 로 요청해야 옴.
// 필요 라이브러리: FlexCAN_T4 (Teensyduino 포함), Adafruit BNO08x
// 배선: BNO085 SDA=18, SCL=19, 3.3V, GND (주소 0x4A, 안 되면 0x4B)

#include <FlexCAN_T4.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_32> can1;   // pins 22 TX / 23 RX
Adafruit_BNO08x bno(-1);
sh2_SensorValue_t sv;

// ---- DM motor limits (모터 레지스터와 같아야 함) ----
const float P_MAX = 12.5f, V_MAX = 10.0f, T_MAX = 28.0f, KP_MAX = 500.0f, KD_MAX = 5.0f;
// ---- hip safety ----
const float    FAULT_TAU = 10.0f, FAULT_ERR_DEG = 10.0f;
const uint32_t FB_TIMEOUT_MS = 50, HB_TIMEOUT_MS = 300;
const int      FAULT_TEMP_C = 70;
const float    TFF_MAX = 9.0f;
// ---- timing ----
const uint32_t PERIOD_US = 2000;    // 500 Hz (힙)
const uint32_t TX_PERIOD_US = 4000; // 250 Hz (PC 송신)
const uint32_t BAL_EVERY = 2;       // 밸런싱/바퀴 = 500/2 = 250 Hz
const float    DEGf = 0.01745329f;

// ================= 힙 (DM) =================
struct Motor {
  uint8_t canId, masterId; float dir;
  bool enabled, holding; float base, tff;
  float pos, vel, tau; uint8_t state, tmos, trotor; uint32_t lastFbMs;
};
Motor M[2] = {
  {0x01, 0x11, +1.0f, false, false, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0x02, 0x12, -1.0f, false, false, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};
float holdKp = 60.0f, holdKd = 2.0f;
bool  fault = false;
uint32_t lastHbMs = 0, cyc = 0, imuUs = 0;
float diagLoopHz = 0, diagImuUsMax = 0, diagImuEvHz = 0; uint32_t winCyc = 0, winImuUsMax = 0, winEv = 0, winT0 = 0;
float qr = 1, qi = 0, qj = 0, qk = 0, gx = 0, gy = 0, gz = 0; uint32_t lastImuMs = 0;

// ================= 바퀴 (ODrive CANSimple) =================
enum { OD_HEARTBEAT = 0x01, OD_SET_STATE = 0x07, OD_GET_ENC = 0x09, OD_SET_MODES = 0x0B, OD_SET_VEL = 0x0D, OD_CLEAR_ERR = 0x18 };
enum { AX_IDLE = 1, AX_CLOSED_LOOP = 8 };
struct Wheel {
  uint8_t node; uint32_t axisErr; uint8_t axState; uint32_t lastHbMs;
  float pos, vel; uint32_t lastEncMs;     // turns, turns/s (ODrive 원래 부호)
  uint32_t lastAnyMs, nHb, nEnc;          // 아무 메시지 마지막 시각, 하트비트/엔코더 수신 횟수 (진단)
};
Wheel W[2] = { {3, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {4, 0, 0, 0, 0, 0, 0, 0, 0, 0} };   // [0]=왼쪽 axis0, [1]=오른쪽 axis1
float diagHbHz = 0, diagEncHz = 0; uint32_t winHb0 = 0, winEnc0 = 0;

// ================= 밸런싱 파라미터 ('P' idx val 로 PC 가 설정. 기본값 = config.py 와 같음) =================
enum {
  PRM_KTH, PRM_KW, PRM_KPV, PRM_KIV, PRM_GAIN, PRM_IMU_SIGN, PRM_IMU_MOUNT, PRM_COM_SIGN, PRM_DPHI_SIGN, PRM_DPHI_AT_H,
  PRM_DIR_L, PRM_DIR_R, PRM_A_MAX, PRM_V_MAX, PRM_FALL_DEG, PRM_START_DEG, PRM_THREF_MAX, PRM_I_MAX, PRM_R_WHEEL, PRM_TRACK,
  PRM_COUNT
};
float prm[PRM_COUNT] = {
  27.9f, 4.65f, 0.078f, 0.036f, 1.0f, +1.0f, 0.0f, +1.0f, -1.0f, 0.0f,
  -1.0f, -1.0f, 6.0f, 1.0f, 30.0f, 8.0f, 8.0f, 0.5f, 0.086f, 0.488f   // 바퀴 방향 -1: 2026-11-22 확인 (앞으로 굴리면 ODrive 값 -)
};
enum { BM_IDLE = 0, BM_BAL = 1, BM_STOP = 2 };
enum { ERR_NONE, ERR_IMU, ERR_HIP, ERR_FALL, ERR_ODRV, ERR_ODRV_NOT_READY, ERR_TILT_START, ERR_PC_LOST, ERR_ODRV_NO_CL, ERR_STOP_CMD };
uint8_t balMode = BM_IDLE, balErr = ERR_NONE;
float trimDeg = 0, vRef = 0, yawRef = 0;
float thCom = 0, wCom = 0, vMeas = 0, vCmd = 0, xMeas = 0, x0 = 0, iV = 0, thRef = 0;
uint32_t balStartMs = 0, balPrevUs = 0;
bool zeroTrimReq = false;

// ---------------- 공통 ----------------
static uint16_t f2u(float x, float a, float b, int bits) { x = constrain(x, a, b); return (uint16_t)((x - a) * (float)((1 << bits) - 1) / (b - a)); }
static float    u2f(uint16_t u, float a, float b, int bits) { return (float)u * (b - a) / (float)((1 << bits) - 1) + a; }
static float relDeg(const Motor& m) { return m.dir * (m.pos - m.base) * 57.29578f; }
static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// ---------------- DM ----------------
static void sendSpecial(const Motor& m, uint8_t code) {
  CAN_message_t msg; msg.id = m.canId; msg.len = 8;
  for (int i = 0; i < 7; i++) msg.buf[i] = 0xFF;
  msg.buf[7] = code; can1.write(msg);
}
static void sendMit(const Motor& m, float p, float v, float kp, float kd, float t) {
  uint16_t pu = f2u(p, -P_MAX, P_MAX, 16), vu = f2u(v, -V_MAX, V_MAX, 12), kpu = f2u(kp, 0, KP_MAX, 12),
           kdu = f2u(kd, 0, KD_MAX, 12), tu = f2u(t, -T_MAX, T_MAX, 12);
  CAN_message_t msg; msg.id = m.canId; msg.len = 8;
  msg.buf[0] = pu >> 8; msg.buf[1] = pu & 0xFF; msg.buf[2] = vu >> 4;
  msg.buf[3] = ((vu & 0x0F) << 4) | (kpu >> 8); msg.buf[4] = kpu & 0xFF;
  msg.buf[5] = kdu >> 4; msg.buf[6] = ((kdu & 0x0F) << 4) | (tu >> 8); msg.buf[7] = tu & 0xFF;
  can1.write(msg);
}

// ---------------- ODrive ----------------
static void odSend(const Wheel& w, uint8_t cmd, const void* data, uint8_t n, bool rtr = false) {
  CAN_message_t msg; msg.id = ((uint32_t)w.node << 5) | cmd; msg.len = 8; msg.flags.remote = rtr;
  memset(msg.buf, 0, 8); if (data && n) memcpy(msg.buf, data, n);
  can1.write(msg);
}
static void odSetState(const Wheel& w, int32_t st) { odSend(w, OD_SET_STATE, &st, 4); }
static void odSetVel(const Wheel& w, float tps)     { float d[2] = {tps, 0.0f}; odSend(w, OD_SET_VEL, d, 8); }
static void odVelMode(const Wheel& w)               { int32_t d[2] = {2, 1}; odSend(w, OD_SET_MODES, d, 8); }  // VELOCITY, PASSTHROUGH
static void odReqEnc(const Wheel& w)                { odSend(w, OD_GET_ENC, nullptr, 0, true); }
static void odClear(const Wheel& w)                 { odSend(w, OD_CLEAR_ERR, nullptr, 0); }
// 살아있음 = 하트비트든 엔코더 응답이든 300 ms 안에 뭐라도 옴 (하트비트가 안 오는 경우가 있어서)
static bool odAlive(const Wheel& w)  { return millis() - w.lastAnyMs < 300; }
static bool odHbOk(const Wheel& w)   { return millis() - w.lastHbMs < 300; }
static bool odEncOk(const Wheel& w)  { return millis() - w.lastEncMs < 50; }
static void wheelsIdle() { for (Wheel& w : W) { odSetVel(w, 0); odSetState(w, AX_IDLE); } }

static void balStop(uint8_t err) {
  if (balMode == BM_BAL || err == ERR_STOP_CMD) wheelsIdle();
  balMode = err ? BM_STOP : BM_IDLE; balErr = err; vCmd = 0; iV = 0; thRef = 0;
}

// ---------------- CAN 수신 ----------------
static void onRx(const CAN_message_t& msg) {
  if (msg.flags.extended) return;
  if (msg.id >= 0x20) {                                     // ODrive (node 3,4)
    uint8_t node = msg.id >> 5, cmd = msg.id & 0x1F;
    for (Wheel& w : W) {
      if (node != w.node) continue;
      if (msg.flags.remote) return;
      w.lastAnyMs = millis();
      if (cmd == OD_HEARTBEAT && msg.len >= 5) {
        memcpy(&w.axisErr, &msg.buf[0], 4); w.axState = msg.buf[4]; w.lastHbMs = millis(); w.nHb++;
      } else if (cmd == OD_GET_ENC && msg.len >= 8) {
        memcpy(&w.pos, &msg.buf[0], 4); memcpy(&w.vel, &msg.buf[4], 4); w.lastEncMs = millis(); w.nEnc++;
      }
      return;
    }
    return;
  }
  if (msg.len < 8) return;                                  // DM 피드백 (0x00~0x1F)
  uint8_t esc = msg.buf[0] & 0x0F;
  for (Motor& m : M) {
    if (esc != m.canId) continue;
    m.state = msg.buf[0] >> 4;
    m.pos = u2f(((uint16_t)msg.buf[1] << 8) | msg.buf[2], -P_MAX, P_MAX, 16);
    m.vel = u2f(((uint16_t)msg.buf[3] << 4) | (msg.buf[4] >> 4), -V_MAX, V_MAX, 12);
    m.tau = u2f(((uint16_t)(msg.buf[4] & 0x0F) << 8) | msg.buf[5], -T_MAX, T_MAX, 12);
    m.tmos = msg.buf[6]; m.trotor = msg.buf[7]; m.lastFbMs = millis();
    return;
  }
}
static void disableAll() { for (Motor& m : M) { sendSpecial(m, 0xFD); m.enabled = m.holding = false; m.tff = 0; } }

// ---------------- IMU ----------------
static void imuEnable() { bno.enableReport(SH2_GYRO_INTEGRATED_RV, 5000); }   // 200 Hz, 자세+각속도 한 리포트
static bool imuFound = false;
// I2C 버스 풀기: Teensy 만 리셋되고 BNO085 는 전원이 유지되면, BNO 가 SDA 를 잡고 있어서 초기화가 실패할 수 있음.
// SCL 을 9번 흔들어서 BNO 가 하던 전송을 끝내게 한 뒤 Wire 를 다시 시작
static void i2cBusClear() {
  pinMode(18, INPUT_PULLUP); pinMode(19, OUTPUT_OPENDRAIN);
  for (int i = 0; i < 9; i++) { digitalWrite(19, LOW); delayMicroseconds(5); digitalWrite(19, HIGH); delayMicroseconds(5); }
  pinMode(19, INPUT_PULLUP); delayMicroseconds(10);
}
static bool imuInit() {
  i2cBusClear();
  Wire.begin(); Wire.setClock(400000);
  if (!bno.begin_I2C(0x4A, &Wire) && !bno.begin_I2C(0x4B, &Wire)) return false;
  imuEnable(); return true;
}
static void imuPoll() {
  if (!imuFound) return;
  if (bno.wasReset()) imuEnable();
  if (bno.getSensorEvent(&sv)) {
    winEv++;
    if (sv.sensorId == SH2_GYRO_INTEGRATED_RV) {
      qr = sv.un.gyroIntegratedRV.real; qi = sv.un.gyroIntegratedRV.i;
      qj = sv.un.gyroIntegratedRV.j;    qk = sv.un.gyroIntegratedRV.k;
      gx = sv.un.gyroIntegratedRV.angVelX; gy = sv.un.gyroIntegratedRV.angVelY; gz = sv.un.gyroIntegratedRV.angVelZ;
      lastImuMs = millis();
    }
  }
}
static bool imuOk() { return imuFound && (millis() - lastImuMs < 50); }
// 몸통 기준 각도/각속도 (센서 X축 회전 = pitch)
static float bodyPitch()     { return -atan2f(2 * (qr * qi + qj * qk), 1 - 2 * (qi * qi + qj * qj)); }
static float bodyRoll()      { return asinf(constrain(2 * (qr * qj - qk * qi), -1.0f, 1.0f)); }
static float bodyYaw()       { return atan2f(2 * (qr * qk + qi * qj), 1 - 2 * (qj * qj + qk * qk)); }
static float bodyPitchRate() { return -gx; }
static float bodyRollRate()  { return gy; }

// ---------------- CoM 모델 (model.py 와 같은 식) ----------------
static float dphiClip(float d) { return clampf(d, -21.6f, 20.5f); }
static float thetaOffDeg(float d)   { d = dphiClip(d); return 6.188f - 0.1781f*d + 0.002189f*d*d - 0.000333f*d*d*d; }
static float thetaOffSlope(float d) { d = dphiClip(d); return -0.1781f + 2*0.002189f*d - 3*0.000333f*d*d; }

// ---------------- 밸런싱 (250 Hz) ----------------
static bool hipOk() { return !fault && M[0].holding && M[1].holding; }

static void balanceStep() {
  uint32_t nowUs = micros();
  float dt = (nowUs - balPrevUs) * 1e-6f; balPrevUs = nowUs;
  dt = clampf(dt, 0.001f, 0.012f);
  const float TPS_PER_MPS = 1.0f / (2.0f * PI * prm[PRM_R_WHEEL]);

  // 상태 추정
  float thBody = prm[PRM_IMU_SIGN] * bodyPitch() - prm[PRM_IMU_MOUNT] * DEGf;
  float wBody  = prm[PRM_IMU_SIGN] * bodyPitchRate();
  float hipRel = M[0].holding && M[1].holding ? 0.5f * (relDeg(M[0]) + relDeg(M[1])) : 0.0f;
  float hipVel = 0.5f * (M[0].dir * M[0].vel + M[1].dir * M[1].vel) * 57.29578f;
  float dphi   = prm[PRM_DPHI_SIGN] * hipRel + prm[PRM_DPHI_AT_H];
  thCom = thBody + prm[PRM_COM_SIGN] * thetaOffDeg(dphi) * DEGf + trimDeg * DEGf;
  wCom  = wBody + prm[PRM_COM_SIGN] * thetaOffSlope(dphi) * prm[PRM_DPHI_SIGN] * hipVel * DEGf;
  float vl = prm[PRM_DIR_L] * W[0].vel / TPS_PER_MPS, vr = prm[PRM_DIR_R] * W[1].vel / TPS_PER_MPS;
  vMeas = 0.5f * (vl + vr);
  xMeas = 0.5f * (prm[PRM_DIR_L] * W[0].pos + prm[PRM_DIR_R] * W[1].pos) / TPS_PER_MPS - x0;

  if (zeroTrimReq) { trimDeg -= thCom / DEGf; thCom = 0; zeroTrimReq = false; }

  // 바퀴 엔코더 요청 (다음 주기에 도착)
  odReqEnc(W[0]); odReqEnc(W[1]);

  if (balMode != BM_BAL) return;

  // 안전
  bool odBad = !odAlive(W[0]) || !odAlive(W[1]) || (odHbOk(W[0]) && W[0].axisErr) || (odHbOk(W[1]) && W[1].axisErr) ||
               !odEncOk(W[0]) || !odEncOk(W[1]);
  // closed-loop 확인은 하트비트가 올 때만 가능 (안 오면 확인 생략 -> 넘어짐 감지가 대신 막음)
  bool notCl = (millis() - balStartMs > 400) &&
               ((odHbOk(W[0]) && W[0].axState != AX_CLOSED_LOOP) || (odHbOk(W[1]) && W[1].axState != AX_CLOSED_LOOP));
  if (!imuOk())                                 { balStop(ERR_IMU);  return; }
  if (fabsf(thCom) > prm[PRM_FALL_DEG] * DEGf)  { balStop(ERR_FALL); return; }
  if (!hipOk())                                 { balStop(ERR_HIP);  return; }
  if (odBad)                                    { balStop(ERR_ODRV); return; }
  if (notCl)                                    { balStop(ERR_ODRV_NO_CL); return; }

  // 두 겹 제어:  th_ref = -(KPV*(v-v_ref) + KIV*∫(v-v_ref)) ,  a = KTH*(th_com - th_ref) + KW*w_com ,  v_cmd += a*dt
  // (기본 게인은 LQR 과 같은 값: KPV = KV/KTH, KIV = KX/KTH)
  float vErr = vMeas - vRef;
  iV = clampf(iV + vErr * dt, -prm[PRM_I_MAX], prm[PRM_I_MAX]);
  float lim = prm[PRM_THREF_MAX] * DEGf;
  thRef = clampf(-(prm[PRM_KPV] * vErr + prm[PRM_KIV] * iV), -lim, lim);
  float a = prm[PRM_GAIN] * (prm[PRM_KTH] * (thCom - thRef) + prm[PRM_KW] * wCom);
  a = clampf(a, -prm[PRM_A_MAX], prm[PRM_A_MAX]);
  vCmd = clampf(vCmd + a * dt, -prm[PRM_V_MAX], prm[PRM_V_MAX]);
  float dv = yawRef * prm[PRM_TRACK] * 0.5f;
  odSetVel(W[0], prm[PRM_DIR_L] * (vCmd - dv) * TPS_PER_MPS);
  odSetVel(W[1], prm[PRM_DIR_R] * (vCmd + dv) * TPS_PER_MPS);
}

static void balStart() {
  if (balMode == BM_BAL) return;
  uint8_t e = ERR_NONE;
  if (!imuOk()) e = ERR_IMU;
  else if (!hipOk()) e = ERR_HIP;
  else if (!odAlive(W[0]) || !odAlive(W[1]) || !odEncOk(W[0]) || !odEncOk(W[1])) e = ERR_ODRV_NOT_READY;
  else if (fabsf(thCom) > prm[PRM_START_DEG] * DEGf) e = ERR_TILT_START;
  if (e) { balMode = BM_STOP; balErr = e; return; }
  for (Wheel& w : W) { odClear(w); w.axisErr = 0; odVelMode(w); odSetVel(w, 0); odSetState(w, AX_CLOSED_LOOP); }
  const float TPS_PER_MPS = 1.0f / (2.0f * PI * prm[PRM_R_WHEEL]);
  x0 = 0.5f * (prm[PRM_DIR_L] * W[0].pos + prm[PRM_DIR_R] * W[1].pos) / TPS_PER_MPS;
  vCmd = 0; iV = 0; thRef = 0; vRef = 0; yawRef = 0;
  balStartMs = millis(); balMode = BM_BAL; balErr = ERR_NONE;
}

// ---------------- PC protocol ----------------
static uint8_t rxb[16]; static int rxn = 0;
static void handleCmd(char c, float a, float b) {
  uint32_t now = millis();
  switch (c) {
    case 'E': for (Motor& m : M) { sendSpecial(m, 0xFC); m.enabled = true; m.holding = false; m.tff = 0; m.lastFbMs = now; } fault = false; break;
    case 'H': for (Motor& m : M) if (m.enabled) { m.base = m.pos; m.holding = true; } break;
    case 'D': balStop(ERR_STOP_CMD); disableAll(); break;
    case 'K': holdKp = constrain(a, 0.0f, 150.0f); holdKd = constrain(b, 0.0f, KD_MAX); break;
    case 'T': M[0].tff = constrain(a, -TFF_MAX, TFF_MAX); M[1].tff = constrain(b, -TFF_MAX, TFF_MAX); break;
    case 'P': { int i = (int)(a + 0.5f); if (i >= 0 && i < PRM_COUNT && isfinite(b)) prm[i] = b; } break;
    case 'B': if (a > 0.5f) balStart(); else balStop(ERR_NONE); break;
    case 'V': vRef = clampf(a, -0.5f, 0.5f); yawRef = clampf(b, -2.0f, 2.0f); break;
    case 'Z': zeroTrimReq = true; break;
    case 'R': if (isfinite(a)) trimDeg = clampf(a, -45.0f, 45.0f); break;
    case 'W': break;
  }
  lastHbMs = now;
}
static void readPc() {
  while (Serial.available()) {
    uint8_t ch = Serial.read();
    if (rxn == 0 && ch != 0x5A) continue;
    if (rxn == 1 && ch != 0xA5) { rxn = (ch == 0x5A); continue; }
    rxb[rxn++] = ch;
    if (rxn == 12) {
      uint8_t s = 0; for (int i = 2; i < 11; i++) s += rxb[i];
      if (s == rxb[11]) { float a, b; memcpy(&a, &rxb[3], 4); memcpy(&b, &rxb[7], 4); handleCmd((char)rxb[2], a, b); }
      rxn = 0;
    }
  }
}
struct __attribute__((packed)) StatePkt { uint32_t t; float f[24]; uint8_t flags, imuOk, balMode, balErr; };
static void sendState() {
  StatePkt p;
  p.t = micros();
  p.f[0] = bodyRoll(); p.f[1] = bodyPitch(); p.f[2] = bodyYaw();
  p.f[3] = bodyRollRate(); p.f[4] = bodyPitchRate(); p.f[5] = gz;
  for (int i = 0; i < 2; i++) {
    const Motor& m = M[i];
    p.f[6 + i]  = m.holding ? relDeg(m) : 0.0f;
    p.f[8 + i]  = m.dir * m.vel * 57.29578f;
    p.f[10 + i] = m.dir * m.tau;
  }
  p.f[12] = diagLoopHz; p.f[13] = diagImuUsMax; p.f[14] = diagImuEvHz;
  p.f[15] = thCom; p.f[16] = wCom; p.f[17] = vMeas; p.f[18] = vCmd; p.f[19] = xMeas; p.f[20] = thRef; p.f[21] = trimDeg;
  p.f[22] = diagHbHz; p.f[23] = diagEncHz;                  // ODrive 하트비트/엔코더 수신 Hz (두 바퀴 합)
  p.flags = (M[0].enabled ? 1 : 0) | (M[1].enabled ? 2 : 0) | ((M[0].holding && M[1].holding) ? 4 : 0) | (fault ? 8 : 0) |
            (odAlive(W[0]) ? 16 : 0) | (odAlive(W[1]) ? 32 : 0) |
            (W[0].axState == AX_CLOSED_LOOP ? 64 : 0) | (W[1].axState == AX_CLOSED_LOOP ? 128 : 0);
  p.imuOk = imuOk() ? 1 : 0; p.balMode = balMode; p.balErr = balErr;
  uint8_t s = 0; const uint8_t* q = (const uint8_t*)&p; for (unsigned i = 0; i < sizeof(p); i++) s += q[i];
  uint8_t hdr[2] = {0xA5, 0x5A};
  if (Serial.availableForWrite() < (int)(sizeof(p) + 3)) return;   // PC 가 안 읽으면 버림 (루프 멈춤 방지)
  Serial.write(hdr, 2); Serial.write(q, sizeof(p)); Serial.write(&s, 1);
}

// ---------------- 힙 안전 ----------------
static void safety() {
  uint32_t now = millis();
  for (Motor& m : M) {
    if (!m.enabled) continue;
    bool bad = (now - m.lastFbMs > FB_TIMEOUT_MS) || (m.state >= 8) || (fabsf(m.tau) > FAULT_TAU) ||
               (m.tmos > FAULT_TEMP_C) || (m.trotor > FAULT_TEMP_C) ||
               (m.holding && fabsf(relDeg(m)) > FAULT_ERR_DEG);
    if (bad) { if (balMode == BM_BAL) balStop(ERR_HIP); disableAll(); fault = true; return; }
  }
  if (now - lastHbMs > HB_TIMEOUT_MS) {                     // PC 끊김
    M[0].tff = M[1].tff = 0;
    if (balMode == BM_BAL) balStop(ERR_PC_LOST);
  }
}

void setup() {
  Serial.begin(115200);
  can1.begin(); can1.setBaudRate(1000000); can1.setMaxMB(16);
  can1.enableFIFO(); can1.enableFIFOInterrupt(); can1.onReceive(onRx);
  imuFound = imuInit();
  balPrevUs = micros();
}

void loop() {
  can1.events();
  readPc();
  static elapsedMicros t, tx;
  if (t < PERIOD_US) return;
  t = 0; cyc++;
  uint32_t t0 = micros();
  imuPoll();
  imuUs = micros() - t0;
  winCyc++; if (imuUs > winImuUsMax) winImuUsMax = imuUs;
  if (millis() - winT0 >= 1000) {
    float sec = (millis() - winT0) / 1000.0f;
    diagLoopHz = winCyc / sec; diagImuUsMax = winImuUsMax; diagImuEvHz = winEv / sec;
    uint32_t hb = W[0].nHb + W[1].nHb, en = W[0].nEnc + W[1].nEnc;
    diagHbHz = (hb - winHb0) / sec; diagEncHz = (en - winEnc0) / sec; winHb0 = hb; winEnc0 = en;
    winCyc = winImuUsMax = winEv = 0; winT0 = millis();
  }
  for (Motor& m : M) {
    if (!m.enabled) continue;
    if (m.holding) sendMit(m, m.base, 0.0f, holdKp, holdKd, m.dir * m.tff);
    else           sendMit(m, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }
  safety();
  // IMU 를 못 찾았으면 1초마다 재시도. 초기화가 수십 ms 걸려서 루프가 멈추므로 힙/밸런싱이 꺼져 있을 때만
  static uint32_t imuRetryMs = 0; static int imuRetries = 0;   // (라이브러리가 begin 마다 메모리를 조금 잡아서 횟수 제한)
  if (!imuFound && imuRetries < 60 && !M[0].enabled && !M[1].enabled && balMode != BM_BAL && millis() - imuRetryMs > 1000) {
    imuRetryMs = millis(); imuRetries++; imuFound = imuInit();
  }
  if (cyc % BAL_EVERY == 0) balanceStep();
  if (tx >= TX_PERIOD_US) { tx = 0; sendState(); }
}
