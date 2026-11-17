// DM-J4340P-2EC register tool for Teensy 4.1 (CAN1: 22 = CTX1, 23 = CRX1, 1 Mbps)
//
// Reads and writes motor registers over CAN (frame ID 0x7FF), so no Damiao debug tool is needed.
//
//   s : scan CAN ID 1..16 and print the registers of every motor that answers
//   L : set the ONE connected motor as LEFT  (CAN ID 1, Master ID 0x11) and save
//   R : set the ONE connected motor as RIGHT (CAN ID 2, Master ID 0x12) and save
//   t : write CAN timeout 2000 (x 50 us = 100 ms) to the ONE connected motor and save
//   j : jog the ONE connected motor to see which one it is (holds position, then:)
//         + / - : move 1 deg   c : back to start   q : stop jog (motor disabled)
//
// L, R and t refuse to run unless exactly one motor answers the scan.
// After L, R or t: power-cycle the motor (24 V off -> on), then press s to check.
//
// Register frame (Damiao protocol, data 8 B, sent to ID 0x7FF):
//   [id_lo, id_hi, op, rid, value (4 B, little endian)]   op: 0x33 read, 0x55 write, 0xAA save
// The reply carries the same op / rid. Save frame layout follows the Damiao SDK (DM_CAN.py);
// the manual does not list it, so success is judged by reading the registers after a power cycle.

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

const uint16_t REG_FRAME_ID = 0x7FF;
const uint8_t OP_READ = 0x33;
const uint8_t OP_WRITE = 0x55;
const uint8_t OP_SAVE = 0xAA;

const uint8_t RID_MST_ID = 7;
const uint8_t RID_ESC_ID = 8;
const uint8_t RID_TIMEOUT = 9;
const uint8_t RID_CTRL_MODE = 10;
const uint8_t RID_PMAX = 21;
const uint8_t RID_VMAX = 22;
const uint8_t RID_TMAX = 23;

// Values the firmware assumes. Must match every motor (checked on scan).
const float EXPECT_PMAX = 12.5f;
const float EXPECT_VMAX = 10.0f;
const float EXPECT_TMAX = 28.0f;
const uint32_t EXPECT_MODE = 1;  // 1 = MIT

const uint16_t LEFT_ID = 0x01;
const uint16_t LEFT_MST = 0x11;
const uint16_t RIGHT_ID = 0x02;
const uint16_t RIGHT_MST = 0x12;
const uint32_t TIMEOUT_VALUE = 2000;  // x 50 us = 100 ms

// ---- jog (identify the connected motor before writing registers) ----
const float P_MAX = 12.5f;            // must equal the motor registers (checked by s)
const float V_MAX = 10.0f;
const float T_MAX = 28.0f;
const float KP_MAX = 500.0f;
const float KD_MAX = 5.0f;
const float DEG = 0.0174533f;
const float JOG_STEP_RAD = 1.0f * DEG;    // one + or - press
const float JOG_LIMIT_RAD = 10.0f * DEG;  // never more than +-10 deg from the start position
const float JOG_RATE = 0.2f;              // rad/s, how fast the target follows a step
const float JOG_KP = 30.0f;               // 1 deg error -> about 0.5 Nm
const float JOG_KD = 1.0f;
const float JOG_MAX_TORQUE = 3.0f;        // Nm, stop above this
const float JOG_MAX_ERROR = 0.3f;         // rad, stop if the shaft cannot follow

bool jogActive = false;
bool jogHaveFeedback = false;
uint16_t jogId = 0;
float jogStart = 0, jogGoal = 0, jogCmd = 0;
float fbPos = 0, fbTorque = 0;
int fbErr = 0;
uint32_t jogEnableMs = 0, jogLastFeedbackMs = 0, jogLastSendUs = 0, jogLastPrintMs = 0;

const uint16_t SCAN_FIRST = 1;
const uint16_t SCAN_LAST = 16;

// ---------------------------------------------------------------- CAN helpers

void drainRx() {
  CAN_message_t msg;
  while (can1.read(msg)) {}
}

void sendRegFrame(uint16_t motorId, uint8_t op, uint8_t rid, uint32_t value) {
  CAN_message_t msg;
  msg.id = REG_FRAME_ID;
  msg.len = 8;
  msg.buf[0] = motorId & 0xFF;
  msg.buf[1] = motorId >> 8;
  msg.buf[2] = op;
  msg.buf[3] = rid;
  memcpy(&msg.buf[4], &value, 4);  // Teensy is little endian
  can1.write(msg);
}

// Waits for a reply with the same op and rid. When checkId is true the reply must also name motorId.
bool waitReply(uint16_t motorId, uint8_t op, uint8_t rid, bool checkId, uint32_t &value) {
  uint32_t start = millis();
  CAN_message_t msg;
  while (millis() - start < 50) {
    if (!can1.read(msg)) continue;
    if (msg.len != 8 || msg.buf[2] != op || msg.buf[3] != rid) continue;
    if (checkId && msg.buf[0] != (motorId & 0xFF)) continue;
    memcpy(&value, &msg.buf[4], 4);
    return true;
  }
  return false;
}

bool readReg(uint16_t motorId, uint8_t rid, uint32_t &value) {
  drainRx();
  sendRegFrame(motorId, OP_READ, rid, 0);
  return waitReply(motorId, OP_READ, rid, true, value);
}

bool writeReg(uint16_t motorId, uint8_t rid, uint32_t value) {
  drainRx();
  sendRegFrame(motorId, OP_WRITE, rid, value);
  uint32_t echo;
  return waitReply(motorId, OP_WRITE, rid, false, echo);
}

void saveParams(uint16_t motorId) {
  drainRx();
  sendRegFrame(motorId, OP_SAVE, 0x01, 0);
  delay(200);
}

void disableMotor(uint16_t motorId) {
  CAN_message_t msg;
  msg.id = motorId;
  msg.len = 8;
  for (int i = 0; i < 7; i++) msg.buf[i] = 0xFF;
  msg.buf[7] = 0xFD;
  for (int i = 0; i < 3; i++) {
    can1.write(msg);
    delay(5);
  }
}

float asFloat(uint32_t raw) {
  float f;
  memcpy(&f, &raw, 4);
  return f;
}

// ---------------------------------------------------------------- scan / print

bool nearlyEqual(float a, float b) { return fabs(a - b) < 0.01f; }

void printRegisters(uint16_t motorId) {
  uint32_t v;
  Serial.print("  Master ID  = ");
  if (readReg(motorId, RID_MST_ID, v)) { Serial.print("0x"); Serial.println(v, HEX); } else Serial.println("(no reply)");
  Serial.print("  CAN ID     = ");
  if (readReg(motorId, RID_ESC_ID, v)) { Serial.print("0x"); Serial.println(v, HEX); } else Serial.println("(no reply)");
  Serial.print("  Timeout    = ");
  if (readReg(motorId, RID_TIMEOUT, v)) { Serial.print(v); Serial.print(" (x50us = "); Serial.print(v * 0.05f, 1); Serial.println(" ms, 0 = off)"); } else Serial.println("(no reply)");

  Serial.print("  Mode       = ");
  if (readReg(motorId, RID_CTRL_MODE, v)) {
    Serial.print(v);
    Serial.println(v == EXPECT_MODE ? "  OK (MIT)" : "  CHECK: firmware expects 1 (MIT)");
  } else Serial.println("(no reply)");

  const uint8_t rids[3] = {RID_PMAX, RID_VMAX, RID_TMAX};
  const char* names[3] = {"  PMAX       = ", "  VMAX       = ", "  TMAX       = "};
  const float expect[3] = {EXPECT_PMAX, EXPECT_VMAX, EXPECT_TMAX};
  for (int i = 0; i < 3; i++) {
    Serial.print(names[i]);
    if (readReg(motorId, rids[i], v)) {
      float f = asFloat(v);
      Serial.print(f, 3);
      if (nearlyEqual(f, expect[i])) Serial.println("  OK");
      else { Serial.print("  MISMATCH: firmware uses "); Serial.println(expect[i], 3); }
    } else Serial.println("(no reply)");
  }
}

// Returns how many motors answered. firstFound gets the lowest ID that answered.
int scan(bool verbose, uint16_t &firstFound) {
  int count = 0;
  firstFound = 0;
  for (uint16_t id = SCAN_FIRST; id <= SCAN_LAST; id++) {
    uint32_t v;
    if (!readReg(id, RID_ESC_ID, v)) continue;
    count++;
    if (firstFound == 0) firstFound = id;
    if (verbose) {
      Serial.print("FOUND motor at CAN ID ");
      Serial.println(id);
      printRegisters(id);
    }
  }
  if (verbose) {
    Serial.print("Scan done: ");
    Serial.print(count);
    Serial.println(count == 1 ? " motor" : " motors");
    if (count == 2) Serial.println("Two motors answered: IDs are already different.");
  }
  return count;
}

bool findSingleMotor(uint16_t &motorId) {
  int count = scan(false, motorId);
  if (count == 1) return true;
  Serial.print("Refused: ");
  Serial.print(count);
  Serial.println(" motors answered. Connect exactly ONE motor to CAN1 and try again.");
  return false;
}

// ---------------------------------------------------------------- configure

void configure(const char* side, uint16_t newId, uint16_t newMst) {
  uint16_t oldId;
  if (!findSingleMotor(oldId)) return;

  Serial.print("Setting motor at CAN ID ");
  Serial.print(oldId);
  Serial.print(" as ");
  Serial.print(side);
  Serial.print(": CAN ID ");
  Serial.print(newId);
  Serial.print(", Master ID 0x");
  Serial.println(newMst, HEX);

  disableMotor(oldId);

  uint32_t v;
  if (!writeReg(oldId, RID_MST_ID, newMst)) Serial.println("  Master ID write: no echo (checking by read)");
  if (readReg(oldId, RID_MST_ID, v) && v == newMst) Serial.println("  Master ID  OK");
  else { Serial.println("  Master ID write FAILED. Nothing saved."); return; }

  uint16_t activeId = oldId;
  if (oldId != newId) {
    if (!writeReg(oldId, RID_ESC_ID, newId)) Serial.println("  CAN ID write: no echo (checking by read)");
    // Some firmware switches to the new ID at once, some only after save + reboot.
    if (readReg(newId, RID_ESC_ID, v) && v == newId) activeId = newId;
    else if (readReg(oldId, RID_ESC_ID, v) && v == newId) activeId = oldId;
    else { Serial.println("  CAN ID write FAILED. Nothing saved. Power-cycle and scan."); return; }
    Serial.print("  CAN ID     OK (motor now answers at ");
    Serial.print(activeId);
    Serial.println(")");
  } else {
    Serial.println("  CAN ID     already correct");
  }

  saveParams(activeId);
  Serial.println("Save sent. Power-cycle the motor (24 V off -> on), then press s.");
  Serial.print("Expect: CAN ID ");
  Serial.print(newId);
  Serial.print(", Master ID 0x");
  Serial.println(newMst, HEX);
}

void writeTimeout() {
  uint16_t motorId;
  if (!findSingleMotor(motorId)) return;
  disableMotor(motorId);
  uint32_t v;
  writeReg(motorId, RID_TIMEOUT, TIMEOUT_VALUE);
  if (readReg(motorId, RID_TIMEOUT, v) && v == TIMEOUT_VALUE) {
    saveParams(motorId);
    Serial.println("Timeout 2000 (100 ms) saved. Power-cycle, then press s.");
  } else {
    Serial.println("Timeout write FAILED. Nothing saved.");
  }
}

// ---------------------------------------------------------------- jog

uint16_t floatToUint(float value, float minValue, float maxValue, int bits) {
  value = constrain(value, minValue, maxValue);
  return (uint16_t)((value - minValue) * ((1UL << bits) - 1) / (maxValue - minValue));
}

float uintToFloat(uint16_t value, float minValue, float maxValue, int bits) {
  return value * (maxValue - minValue) / ((1UL << bits) - 1) + minValue;
}

void sendSpecial(uint16_t motorId, uint8_t lastByte) {
  CAN_message_t msg;
  msg.id = motorId;
  msg.len = 8;
  for (int i = 0; i < 7; i++) msg.buf[i] = 0xFF;
  msg.buf[7] = lastByte;
  can1.write(msg);
}

void sendMit(uint16_t motorId, float pos, float vel, float kp, float kd, float torque) {
  uint16_t p = floatToUint(pos, -P_MAX, P_MAX, 16);
  uint16_t v = floatToUint(vel, -V_MAX, V_MAX, 12);
  uint16_t k = floatToUint(kp, 0.0f, KP_MAX, 12);
  uint16_t d = floatToUint(kd, 0.0f, KD_MAX, 12);
  uint16_t t = floatToUint(torque, -T_MAX, T_MAX, 12);
  CAN_message_t msg;
  msg.id = motorId;
  msg.len = 8;
  msg.buf[0] = p >> 8;
  msg.buf[1] = p & 0xFF;
  msg.buf[2] = v >> 4;
  msg.buf[3] = ((v & 0x0F) << 4) | (k >> 8);
  msg.buf[4] = k & 0xFF;
  msg.buf[5] = d >> 4;
  msg.buf[6] = ((d & 0x0F) << 4) | (t >> 8);
  msg.buf[7] = t & 0xFF;
  can1.write(msg);
}

void stopJog(const char* reason) {
  disableMotor(jogId);
  jogActive = false;
  Serial.print("JOG stopped (");
  Serial.print(reason);
  Serial.println("). Motor disabled. You can press L / R now.");
}

void startJog() {
  uint16_t motorId;
  if (!findSingleMotor(motorId)) return;
  drainRx();
  jogId = motorId;
  jogHaveFeedback = false;
  fbErr = 0;
  sendSpecial(jogId, 0xFC);  // enable
  jogEnableMs = millis();
  jogLastSendUs = micros();
  jogActive = true;
  Serial.print("JOG on motor CAN ID ");
  Serial.print(jogId);
  Serial.println(". Keys: + / - = 1 deg, c = back to start, q = stop");
}

void readJogFeedback() {
  CAN_message_t msg;
  while (can1.read(msg)) {
    if (msg.len != 8 || msg.id == REG_FRAME_ID) continue;
    if ((msg.buf[0] & 0x0F) != (jogId & 0x0F)) continue;
    uint16_t p = (msg.buf[1] << 8) | msg.buf[2];
    uint16_t t = ((msg.buf[4] & 0x0F) << 8) | msg.buf[5];
    fbErr = msg.buf[0] >> 4;
    fbPos = uintToFloat(p, -P_MAX, P_MAX, 16);
    fbTorque = uintToFloat(t, -T_MAX, T_MAX, 12);
    jogLastFeedbackMs = millis();
    if (!jogHaveFeedback) {
      jogHaveFeedback = true;
      if (fabs(fbPos) > 12.0f) { stopJog("position near range limit, power-cycle the motor"); return; }
      jogStart = fbPos;
      jogGoal = fbPos;
      jogCmd = fbPos;
      Serial.println("Holding current position. Press + or -.");
    }
  }
}

void updateJog() {
  readJogFeedback();
  if (!jogActive) return;

  if (!jogHaveFeedback && millis() - jogEnableMs > 1000) { stopJog("no feedback"); return; }
  if (jogHaveFeedback && millis() - jogLastFeedbackMs > 100) { stopJog("feedback lost"); return; }
  if (fbErr >= 8) { stopJog("motor error code"); return; }
  if (fabs(fbTorque) > JOG_MAX_TORQUE) { stopJog("torque limit"); return; }
  if (jogHaveFeedback && fabs(fbPos - jogCmd) > JOG_MAX_ERROR) { stopJog("cannot follow"); return; }

  uint32_t now = micros();
  if (now - jogLastSendUs < 5000) return;  // 200 Hz
  jogLastSendUs = now;

  if (!jogHaveFeedback) {
    sendMit(jogId, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);  // zero torque, just to get feedback
    return;
  }
  float step = JOG_RATE * 0.005f;
  float diff = jogGoal - jogCmd;
  jogCmd = (fabs(diff) <= step) ? jogGoal : jogCmd + (diff > 0.0f ? step : -step);
  sendMit(jogId, jogCmd, 0.0f, JOG_KP, JOG_KD, 0.0f);

  if (millis() - jogLastPrintMs >= 300) {
    jogLastPrintMs = millis();
    Serial.print("  target ");
    Serial.print((jogGoal - jogStart) / DEG, 1);
    Serial.print(" deg | shaft ");
    Serial.print((fbPos - jogStart) / DEG, 1);
    Serial.print(" deg | torque ");
    Serial.print(fbTorque, 2);
    Serial.println(" Nm");
  }
}

void handleJogKey(char c) {
  if (c == '+' || c == '-') {
    float goal = jogGoal + (c == '+' ? JOG_STEP_RAD : -JOG_STEP_RAD);
    jogGoal = constrain(goal, jogStart - JOG_LIMIT_RAD, jogStart + JOG_LIMIT_RAD);
  } else if (c == 'c') {
    jogGoal = jogStart;
  } else if (c == 'q') {
    stopJog("user q");
  } else if (c != '\n' && c != '\r' && c != ' ') {
    Serial.println("In jog: + - c q only. Press q first.");
  }
}

// ---------------------------------------------------------------- main

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  can1.begin();
  can1.setBaudRate(1000000);
  delay(500);
  Serial.println("DM register tool. Commands: s scan, j jog 1 deg, L set left, R set right, t timeout 100 ms");
  Serial.println("For L / R / t connect only ONE motor.");
}

void loop() {
  if (jogActive) {
    updateJog();
    if (Serial.available()) handleJogKey(Serial.read());
    return;
  }
  if (!Serial.available()) return;
  char c = Serial.read();
  uint16_t dummy;
  switch (c) {
    case 'j': startJog(); break;
    case 's': scan(true, dummy); break;
    case 'L': configure("LEFT", LEFT_ID, LEFT_MST); break;
    case 'R': configure("RIGHT", RIGHT_ID, RIGHT_MST); break;
    case 't': writeTimeout(); break;
    default: break;
  }
}
