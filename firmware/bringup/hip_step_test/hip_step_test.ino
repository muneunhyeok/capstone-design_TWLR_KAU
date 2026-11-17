// hip_step_test.ino  -  TWLR hip DM4340P x2 (CAN1) step test
//
// What it does
//   e  : enable selected motors in FREE mode (kp = kd = 0) -> feedback streams, move legs by hand
//   h  : take the CURRENT position as the base pose (software offset, nothing is written to the motor)
//        and hold it with HOLD_KP / HOLD_KD
//   g  : sweep  0 -> +5 -> +10 -> +15 -> +20 -> +15 -> ... -> 0 deg  (leg direction, see dir)
//   n  : same sweep in the negative direction
//   Each step ramps at RAMP_DPS, then dwells DWELL_MS. While sweeping, CSV lines are printed
//   at 100 Hz:  D,t_ms,L_ref,L_pos,L_vel,L_tau,R_ref,R_pos,R_vel,R_tau
//   (deg relative to base, deg/s, N*m - all multiplied by dir so both legs read the same way)
//
// Serial commands (115200, single key, no Enter needed in picocom)
//   0 / 1 / 2  select both (default) / left only / right only
//   e          enable selected (FREE)
//   h          hold selected at current position (= new base, ref 0)
//   g / n      start + / - sweep on the selected, holding motors
//   p          pause sweep (hold where the reference is now)
//   r          ramp reference back to 0 and hold
//   l          toggle CSV logging by hand
//   c          clear motor error (selected, only while disabled)
//   d          disable selected
//   x          disable ALL (emergency)
//   s          status + CAN receive counters
//   f          LOAD mode on/off: gains ramp HOLD 30/2 -> LOAD 80/3 (for lifting the torso)
//   ] / [      gravity feed-forward +0.5 / -0.5 N*m per leg, extension direction (0..FF_MAX),
//              ramped at FF_RATE; only applied in LOAD mode
//   ?          help
//
// v2 (2026-11-16): LOAD mode + feed-forward for the torso lift test. CSV gets an extra 'ff' column.
//
// Safety (any one -> disable ALL)
//   no feedback FB_TIMEOUT_MS, motor error code >= 8, |tau| > FAULT_TAU,
//   |pos - base| > FAULT_RANGE_DEG, |ref - pos| > FAULT_ERR_DEG, temperature > FAULT_TEMP_C
//
// Motor registers (dm_config_tool scan 2026-11-16):
//   L: CAN ID 0x01 / Master 0x11, R: CAN ID 0x02 / Master 0x12, MIT, PMAX 12.5, VMAX 10, TMAX 28

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;   // pins 22 TX / 23 RX

// ---- motor limits: must equal the motor registers ----
const float P_MAX = 12.5f;
const float V_MAX = 10.0f;
const float T_MAX = 28.0f;
const float KP_MAX = 500.0f;
const float KD_MAX = 5.0f;

// ---- hold gains (legs unloaded, wheels free) ----
const float HOLD_KP = 30.0f;    // N*m/rad   (1 N*m error torque = 1.9 deg)
const float HOLD_KD = 2.0f;     // N*m*s/rad

// ---- LOAD mode (torso lift test, wheels on the floor) ----
const float LOAD_KP  = 80.0f;   // N*m/rad
const float LOAD_KD  = 3.0f;
const float KP_RATE  = 50.0f;   // N*m/rad per s  (gain ramp, no torque jump)
const float FF_STEP  = 0.5f;    // N*m per key press
const float FF_MAX   = 8.0f;    // N*m per leg
const float FF_RATE  = 2.0f;    // N*m/s ramp
// + leg direction = FOLD (measured 2026-11-16), so the lift torque is applied in the - direction

// ---- sweep ----
const float    STEP_DEG      = 5.0f;
const float    SWEEP_MAX_DEG = 20.0f;
const float    RAMP_DPS      = 10.0f;   // ramp speed between steps [deg/s]
const uint32_t DWELL_MS      = 1500;    // wait at each step

// ---- safety ----
const float    FAULT_TAU       = 14.0f;  // N*m (rated 9, peak 27)
const float    FAULT_ERR_DEG   = 10.0f;
const float    FAULT_RANGE_DEG = 22.0f;
const uint32_t FB_TIMEOUT_MS   = 50;
const int      FAULT_TEMP_C    = 70;

// ---- timing ----
const uint32_t PERIOD_US = 2000;   // 500 Hz control
const uint32_t LOG_EVERY = 5;      // CSV every 5 cycles = 100 Hz
const uint32_t STATUS_MS = 250;    // status line while not logging

struct Motor {
  const char* name;
  uint8_t  canId;     // command frame ID (MIT mode)
  uint8_t  masterId;  // feedback frame ID
  float    dir;       // +1 / -1 so that + is the same physical leg direction on both sides (MEASURE!)
  bool     selected;
  bool     enabled;
  bool     holding;   // false = FREE
  float    base;      // raw rad at 'h'
  float    refDeg;    // deg relative to base, leg direction
  uint32_t enableMs;
  bool     warnedState;
  // feedback
  float    pos, vel, tau;
  uint8_t  state, tmos, trotor;
  uint32_t lastFbMs;
  uint32_t frames;
};

Motor M[2] = {
  {"L", 0x01, 0x11, +1.0f, true, false, false, 0.0f, 0.0f, 0, false, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0, 0},
  {"R", 0x02, 0x12, -1.0f, true, false, false, 0.0f, 0.0f, 0, false, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0, 0},
};

// sweep state
float    targets[16];
int      nTargets = 0;
int      tIdx = 0;
bool     sweeping = false;
bool     dwelling = false;
uint32_t dwellStart = 0;

// logging
bool     logging = false;
uint32_t logT0 = 0;
uint32_t cycleCount = 0;
uint32_t lastStatusMs = 0;

// LOAD mode state
bool  loadMode = false;
float ffTarget = 0.0f;   // N*m per leg, extension direction
float ffNow    = 0.0f;
float kpNow    = HOLD_KP;
float kdNow    = HOLD_KD;

// CAN diagnostics
uint32_t rxOther = 0;
uint32_t lastOtherId = 0;

// ---------------------------------------------------------------- helpers
static uint16_t f2u(float x, float xmin, float xmax, int bits) {
  if (x < xmin) x = xmin;
  if (x > xmax) x = xmax;
  return (uint16_t)((x - xmin) * (float)((1 << bits) - 1) / (xmax - xmin));
}

static float u2f(uint16_t u, float xmin, float xmax, int bits) {
  return (float)u * (xmax - xmin) / (float)((1 << bits) - 1) + xmin;
}

static float relDeg(const Motor& m) { return m.dir * (m.pos - m.base) * 57.29578f; }

static void sendSpecial(const Motor& m, uint8_t code) {
  CAN_message_t msg;
  msg.id = m.canId;
  msg.len = 8;
  for (int i = 0; i < 7; i++) msg.buf[i] = 0xFF;
  msg.buf[7] = code;          // FC enable, FD disable, FB clear error
  can1.write(msg);
}

static void sendMit(const Motor& m, float p, float v, float kp, float kd, float t) {
  uint16_t pu  = f2u(p,  -P_MAX, P_MAX, 16);
  uint16_t vu  = f2u(v,  -V_MAX, V_MAX, 12);
  uint16_t kpu = f2u(kp, 0.0f,   KP_MAX, 12);
  uint16_t kdu = f2u(kd, 0.0f,   KD_MAX, 12);
  uint16_t tu  = f2u(t,  -T_MAX, T_MAX, 12);
  CAN_message_t msg;
  msg.id = m.canId;
  msg.len = 8;
  msg.buf[0] = pu >> 8;
  msg.buf[1] = pu & 0xFF;
  msg.buf[2] = vu >> 4;
  msg.buf[3] = ((vu & 0x0F) << 4) | (kpu >> 8);
  msg.buf[4] = kpu & 0xFF;
  msg.buf[5] = kdu >> 4;
  msg.buf[6] = ((kdu & 0x0F) << 4) | (tu >> 8);
  msg.buf[7] = tu & 0xFF;
  can1.write(msg);
}

static void onRx(const CAN_message_t& msg) {
  if (msg.len < 8) return;
  if (msg.id != 0x11 && msg.id != 0x12 && msg.id != 0x00) {
    rxOther++;
    lastOtherId = msg.id;
    return;
  }
  uint8_t esc = msg.buf[0] & 0x0F;
  for (Motor& m : M) {
    if (esc != m.canId) continue;
    uint16_t pu = ((uint16_t)msg.buf[1] << 8) | msg.buf[2];
    uint16_t vu = ((uint16_t)msg.buf[3] << 4) | (msg.buf[4] >> 4);
    uint16_t tu = ((uint16_t)(msg.buf[4] & 0x0F) << 8) | msg.buf[5];
    m.state  = msg.buf[0] >> 4;
    m.pos    = u2f(pu, -P_MAX, P_MAX, 16);
    m.vel    = u2f(vu, -V_MAX, V_MAX, 12);
    m.tau    = u2f(tu, -T_MAX, T_MAX, 12);
    m.tmos   = msg.buf[6];
    m.trotor = msg.buf[7];
    m.lastFbMs = millis();
    m.frames++;
    return;
  }
  rxOther++;
  lastOtherId = msg.id;
}

static const char* modeName(const Motor& m) {
  if (!m.enabled) return "OFF ";
  return m.holding ? "HOLD" : "FREE";
}

static void printStatus() {
  for (int i = 0; i < 2; i++) {
    const Motor& m = M[i];
    Serial.printf("%s %s%s st=%u rel=%+6.1f ref=%+5.1f tau=%+5.2f T=%u/%u fb=%lu",
                  m.name, modeName(m), m.selected ? "*" : " ", m.state,
                  m.holding ? relDeg(m) : m.pos * 57.29578f, m.refDeg, m.dir * m.tau,
                  m.tmos, m.trotor, (unsigned long)m.frames);
    Serial.print(i == 0 ? " | " : "");
  }
  Serial.printf(" | %s kp=%.0f ff=%.1f/%.1f | other=%lu\n", loadMode ? "LOAD" : "hold",
                kpNow, ffNow, ffTarget, (unsigned long)rxOther);
}

static void startLog() {
  logging = true;
  logT0 = millis();
  Serial.println("H,t_ms,L_ref,L_pos,L_vel,L_tau,R_ref,R_pos,R_vel,R_tau,ff");
}

static void printLog() {
  Serial.printf("D,%lu", (unsigned long)(millis() - logT0));
  for (const Motor& m : M) {
    Serial.printf(",%.2f,%.2f,%.1f,%.3f", m.refDeg, relDeg(m), m.dir * m.vel * 57.29578f, m.dir * m.tau);
  }
  Serial.printf(",%.2f\n", -ffNow);   // applied feed-forward in leg direction (- = extension)
}

static void disableMotor(Motor& m) {
  sendSpecial(m, 0xFD);
  m.enabled = false;
  m.holding = false;
}

static void disableAll(const char* why) {
  for (Motor& m : M) disableMotor(m);
  sweeping = false;
  dwelling = false;
  loadMode = false;
  ffTarget = 0.0f;
  ffNow = 0.0f;
  kpNow = HOLD_KP;
  kdNow = HOLD_KD;
  if (logging) { logging = false; Serial.println("# LOG END"); }
  Serial.printf("ALL DISABLED: %s\n", why);
}

static void help() {
  Serial.println("# 0/1/2 select | e enable FREE | h hold here (=base) | g/n sweep +/- 5 deg steps to 20");
  Serial.println("# p pause | r ref->0 | l log | c clear err | d disable sel | x disable ALL | s status");
  Serial.println("# f LOAD mode (kp 80) | ] / [ feed-forward +/-0.5 N*m (extension) | + leg dir = FOLD");
}

// ---------------------------------------------------------------- sweep
static bool sweepReady() {
  bool any = false;
  for (const Motor& m : M) {
    if (!m.selected) continue;
    if (!m.enabled || !m.holding) {
      Serial.printf("# %s is not holding - press h first\n", m.name);
      return false;
    }
    any = true;
  }
  return any;
}

static void startTargets(const float* t, int n) {
  for (int i = 0; i < n; i++) targets[i] = t[i];
  nTargets = n;
  tIdx = 0;
  dwelling = false;
  sweeping = true;
}

static void startSweep(float sign) {
  if (!sweepReady()) return;
  float t[16];
  int n = 0;
  int k = (int)(SWEEP_MAX_DEG / STEP_DEG + 0.5f);
  for (int i = 1; i <= k; i++) t[n++] = sign * STEP_DEG * i;
  for (int i = k - 1; i >= 0; i--) t[n++] = sign * STEP_DEG * i;
  startTargets(t, n);
  if (!logging) startLog();
  Serial.printf("# SWEEP %s start (%d targets)\n", sign > 0 ? "+" : "-", n);
}

static void updateSweep() {
  if (!sweeping) return;
  const float dt = PERIOD_US * 1e-6f;
  const float stepMax = RAMP_DPS * dt;
  float target = targets[tIdx];
  bool arrived = true;
  for (Motor& m : M) {
    if (!m.selected || !m.holding) continue;
    float d = target - m.refDeg;
    if (fabsf(d) <= stepMax) {
      m.refDeg = target;
    } else {
      m.refDeg += (d > 0 ? stepMax : -stepMax);
      arrived = false;
    }
  }
  if (!arrived) return;
  uint32_t now = millis();
  if (!dwelling) {
    dwelling = true;
    dwellStart = now;
    Serial.printf("# STEP %+.1f deg\n", target);
  } else if (now - dwellStart >= DWELL_MS) {
    dwelling = false;
    tIdx++;
    if (tIdx >= nTargets) {
      sweeping = false;
      Serial.println("# SWEEP done");
      if (logging) { logging = false; Serial.println("# LOG END"); }
    }
  }
}

// ---------------------------------------------------------------- safety
static void safety() {
  uint32_t now = millis();
  for (Motor& m : M) {
    if (!m.enabled) continue;
    if (now - m.lastFbMs > FB_TIMEOUT_MS) {
      Serial.printf("%s: no feedback for %lu ms (frames %lu)\n", m.name,
                    (unsigned long)(now - m.lastFbMs), (unsigned long)m.frames);
      disableAll("no feedback");
      return;
    }
    if (m.state >= 8) {
      Serial.printf("%s: motor error code 0x%X\n", m.name, m.state);
      disableAll("motor error");
      return;
    }
    if (m.state == 0 && !m.warnedState && now - m.enableMs > 200) {
      Serial.printf("# WARN %s reports DISABLED (enable not accepted?) - LED should be green\n", m.name);
      m.warnedState = true;
    }
    if (fabsf(m.tau) > FAULT_TAU) {
      Serial.printf("%s: torque %.2f N*m > %.1f\n", m.name, m.tau, FAULT_TAU);
      disableAll("torque limit");
      return;
    }
    if (m.tmos > FAULT_TEMP_C || m.trotor > FAULT_TEMP_C) {
      Serial.printf("%s: temperature %u/%u C\n", m.name, m.tmos, m.trotor);
      disableAll("temperature");
      return;
    }
    if (m.holding) {
      float r = relDeg(m);
      if (fabsf(r) > FAULT_RANGE_DEG) {
        Serial.printf("%s: position %+.1f deg outside +-%.0f\n", m.name, r, FAULT_RANGE_DEG);
        disableAll("range");
        return;
      }
      if (fabsf(m.refDeg - r) > FAULT_ERR_DEG) {
        Serial.printf("%s: tracking error %.1f deg\n", m.name, m.refDeg - r);
        disableAll("tracking error");
        return;
      }
    }
  }
}

// ---------------------------------------------------------------- serial
static void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    uint32_t now = millis();
    switch (c) {
      case '0': M[0].selected = true;  M[1].selected = true;  Serial.println("# selected: both");  break;
      case '1': M[0].selected = true;  M[1].selected = false; Serial.println("# selected: L");     break;
      case '2': M[0].selected = false; M[1].selected = true;  Serial.println("# selected: R");     break;
      case 'e':
        for (Motor& m : M) {
          if (!m.selected) continue;
          sendSpecial(m, 0xFC);
          m.enabled = true;
          m.holding = false;
          m.refDeg = 0.0f;
          m.enableMs = now;
          m.lastFbMs = now;
          m.warnedState = false;
        }
        Serial.println("# ENABLE (FREE) sent");
        break;
      case 'h':
        for (Motor& m : M) {
          if (!m.selected) continue;
          if (!m.enabled || m.frames == 0) { Serial.printf("# %s: enable first / no feedback yet\n", m.name); continue; }
          m.base = m.pos;
          m.refDeg = 0.0f;
          m.holding = true;
          Serial.printf("# %s HOLD base = %.3f rad (%.1f deg raw)\n", m.name, m.base, m.base * 57.29578f);
        }
        break;
      case 'g': startSweep(+1.0f); break;
      case 'n': startSweep(-1.0f); break;
      case 'p': sweeping = false; dwelling = false; Serial.println("# PAUSE"); break;
      case 'r': {
        if (!sweepReady()) break;
        float z[1] = {0.0f};
        startTargets(z, 1);
        Serial.println("# RETURN to 0");
        break;
      }
      case 'l':
        if (logging) { logging = false; Serial.println("# LOG END"); } else startLog();
        break;
      case 'c':
        for (Motor& m : M) if (m.selected && !m.enabled) sendSpecial(m, 0xFB);
        Serial.println("# CLEAR ERROR sent (disabled motors only)");
        break;
      case 'd':
        for (Motor& m : M) if (m.selected) disableMotor(m);
        sweeping = false;
        Serial.println("# DISABLE selected");
        break;
      case 'x': disableAll("user x"); break;
      case 's': printStatus(); break;
      case 'f':
        loadMode = !loadMode;
        if (!loadMode) ffTarget = 0.0f;
        Serial.printf("# LOAD mode %s (gains ramp to %.0f/%.1f)\n", loadMode ? "ON" : "OFF",
                      loadMode ? LOAD_KP : HOLD_KP, loadMode ? LOAD_KD : HOLD_KD);
        break;
      case ']':
        if (!loadMode) { Serial.println("# press f first (LOAD mode)"); break; }
        ffTarget = fminf(ffTarget + FF_STEP, FF_MAX);
        Serial.printf("# FF target %.1f N*m per leg\n", ffTarget);
        break;
      case '[':
        ffTarget = fmaxf(ffTarget - FF_STEP, 0.0f);
        Serial.printf("# FF target %.1f N*m per leg\n", ffTarget);
        break;
      case '?': help(); break;
      default: break;
    }
  }
}

// ---------------------------------------------------------------- main
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}

  can1.begin();
  can1.setBaudRate(1000000);
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(onRx);

  Serial.println("# hip_step_test ready");
  help();
}

void loop() {
  can1.events();
  handleSerial();

  static elapsedMicros cyc;
  if (cyc < PERIOD_US) return;
  cyc = 0;
  cycleCount++;

  updateSweep();

  // gain / feed-forward ramps (no torque jumps)
  {
    const float dt = PERIOD_US * 1e-6f;
    float kpT = loadMode ? LOAD_KP : HOLD_KP;
    float kdT = loadMode ? LOAD_KD : HOLD_KD;
    float ffT = loadMode ? ffTarget : 0.0f;
    float dk = KP_RATE * dt, dd = KP_RATE * dt * (LOAD_KD - HOLD_KD) / (LOAD_KP - HOLD_KP), df = FF_RATE * dt;
    kpNow += fmaxf(-dk, fminf(dk, kpT - kpNow));
    kdNow += fmaxf(-dd, fminf(dd, kdT - kdNow));
    ffNow += fmaxf(-df, fminf(df, ffT - ffNow));
  }

  for (Motor& m : M) {
    if (!m.enabled) continue;
    if (m.holding) {
      float pDes = m.base + m.dir * m.refDeg * 0.01745329f;
      float tff  = m.dir * (-ffNow);   // extension = - leg direction
      sendMit(m, pDes, 0.0f, kpNow, kdNow, tff);
    } else {
      sendMit(m, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);   // FREE: zero torque, still returns feedback
    }
  }

  safety();

  uint32_t now = millis();
  if (logging) {
    if (cycleCount % LOG_EVERY == 0) printLog();
  } else if (now - lastStatusMs >= STATUS_MS) {
    lastStatusMs = now;
    bool anyOn = M[0].enabled || M[1].enabled;
    if (anyOn) printStatus();
  }
}
