// DM-J4340P-2EC x 2 test for Teensy 4.1 (CAN1: 22 = CTX1, 23 = CRX1, 1 Mbps, MIT mode)
//
// Both hip motors on one CAN bus. Set their IDs first with dm_config_tool:
//   LEFT  = CAN ID 1, Master ID 0x11
//   RIGHT = CAN ID 2, Master ID 0x12
//
// What changes from dm_single_test:
//   - one Motor struct per motor instead of global variables
//   - feedback is sorted by motor ID (low 4 bits of byte 0), so one receive loop serves both
//   - one 2 ms cycle (500 Hz) sends a frame to every active motor
//   - a fault on either motor disables both (the legs must move together)
//
// Serial commands (115200):
//   0 / 1 / 2   select both (default) / left only / right only
//   e           enable selected (free, zero torque)
//   h           hold selected at current position
//   + / -       move held position by STEP_RAD (same leg direction on both sides, see DIR)
//   c           go back to the position where h was pressed
//   a / b       bench spin + / - for SPIN_HOLD_S (only while BENCH_SPIN_ALLOWED is true)
//   s           stop spin
//   z           set current position as zero (selected, only when disabled or free)
//   d           disable selected
//   x           disable all (emergency)

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// ---- motor limits: must equal the registers read by dm_config_tool ----
const float P_MAX = 12.5f;
const float V_MAX = 10.0f;
const float T_MAX = 28.0f;
const float KP_MAX = 500.0f;
const float KD_MAX = 5.0f;

// ---- loop ----
const uint32_t PERIOD_US = 2000;  // 500 Hz, same as the integrated firmware plan

// ---- hold / step ----
const float HOLD_KP = 5.0f;
const float HOLD_KD = 0.5f;
const float HOLD_RATE = 0.3f;        // rad/s, how fast the held position follows the goal
const float STEP_RAD = 0.10f;        // + / - step (5.7 deg)
const float HOLD_RANGE_RAD = 0.35f;  // max offset from the h position (20 deg, leg limit +20.9 / -21.6 deg)

// ---- bench spin (motors NOT mounted on the legs) ----
const bool BENCH_SPIN_ALLOWED = false; // legs mounted (2026-11-16): continuous spin disabled
const float SPIN_SPEED = 4.0f;         // rad/s (38 rpm). 24 V no-load max 5.45 rad/s
const float SPIN_KD = 1.0f;
const float SPIN_ACCEL = 2.0f;         // rad/s^2
const float SPIN_HOLD_S = 5.0f;

// ---- safety ----
const float MAX_TORQUE = 6.0f;           // Nm, rated 9
const float MAX_POSITION_ERROR = 1.0f;   // rad, in hold
const int MAX_ROTOR_TEMP = 60;           // deg C
const uint32_t FIRST_FEEDBACK_MS = 1000; // after enable
const uint32_t FEEDBACK_TIMEOUT_MS = 50; // after feedback started

enum State { OFF, FREE, HOLD, SPIN, BRAKE };

struct Motor {
  const char* name;
  uint16_t id;     // command frame ID (= CAN ID)
  float dir;       // +1 / -1 so that the same command moves both legs the same way (measure!)
  bool selected;
  State state;

  // feedback (latest value, written by readFeedback)
  bool haveFeedback = false;
  float pos = 0, vel = 0, torque = 0;
  int mosTemp = 0, rotorTemp = 0, err = 0;
  uint32_t lastFeedbackMs = 0;
  uint32_t rxCount = 0;

  // command
  uint32_t lastEnableMs = 0;
  float holdCenter = 0, holdCmd = 0, holdGoal = 0;
  float spinTarget = 0, spinCmd = 0;
  uint32_t spinStartMs = 0;
};

Motor motors[2] = {
  {"L", 0x01, +1.0f, true, OFF},
  {"R", 0x02, -1.0f, true, OFF},  // mirrored mounting assumed; check with + on both legs
};
const int MOTOR_COUNT = 2;

// All types (State, Motor) are defined above the first function on purpose:
// the Arduino builder inserts function prototypes just before the first function,
// so a function that takes Motor must not appear before struct Motor.

const char* stateName(State s) {
  return s == OFF ? "OFF" : s == FREE ? "FREE" : s == HOLD ? "HOLD" : s == SPIN ? "SPIN" : "BRAKE";
}

uint32_t lastCycleUs = 0;
uint32_t lastPrintMs = 0;
uint32_t worstCycleUs = 0;

// ---------------------------------------------------------------- CAN helpers

uint16_t floatToUint(float value, float minValue, float maxValue, int bits) {
  value = constrain(value, minValue, maxValue);
  return (uint16_t)((value - minValue) * ((1UL << bits) - 1) / (maxValue - minValue));
}

float uintToFloat(uint16_t value, float minValue, float maxValue, int bits) {
  return value * (maxValue - minValue) / ((1UL << bits) - 1) + minValue;
}

void sendSpecial(const Motor &m, uint8_t lastByte) {
  CAN_message_t msg;
  msg.id = m.id;
  msg.len = 8;
  for (int i = 0; i < 7; i++) msg.buf[i] = 0xFF;
  msg.buf[7] = lastByte;
  can1.write(msg);
}

void sendMit(const Motor &m, float pos, float vel, float kp, float kd, float torque) {
  uint16_t p = floatToUint(pos, -P_MAX, P_MAX, 16);
  uint16_t v = floatToUint(vel, -V_MAX, V_MAX, 12);
  uint16_t k = floatToUint(kp, 0.0f, KP_MAX, 12);
  uint16_t d = floatToUint(kd, 0.0f, KD_MAX, 12);
  uint16_t t = floatToUint(torque, -T_MAX, T_MAX, 12);

  CAN_message_t msg;
  msg.id = m.id;
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

// ---------------------------------------------------------------- feedback

Motor* motorById(uint8_t id) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if ((motors[i].id & 0x0F) == id) return &motors[i];
  }
  return nullptr;
}

// Reads every waiting frame and stores the latest value per motor. Never waits.
void readFeedback() {
  CAN_message_t msg;
  while (can1.read(msg)) {
    if (msg.len != 8 || msg.id == 0x7FF) continue;
    Motor* m = motorById(msg.buf[0] & 0x0F);
    if (m == nullptr) continue;

    uint16_t p = (msg.buf[1] << 8) | msg.buf[2];
    uint16_t v = (msg.buf[3] << 4) | (msg.buf[4] >> 4);
    uint16_t t = ((msg.buf[4] & 0x0F) << 8) | msg.buf[5];
    m->err = msg.buf[0] >> 4;
    m->pos = uintToFloat(p, -P_MAX, P_MAX, 16);
    m->vel = uintToFloat(v, -V_MAX, V_MAX, 12);
    m->torque = uintToFloat(t, -T_MAX, T_MAX, 12);
    m->mosTemp = msg.buf[6];
    m->rotorTemp = msg.buf[7];
    m->haveFeedback = true;
    m->lastFeedbackMs = millis();
    m->rxCount++;
  }
}

// ---------------------------------------------------------------- state changes

void disableOne(Motor &m) {
  for (int i = 0; i < 3; i++) {
    sendSpecial(m, 0xFD);
    delayMicroseconds(300);
  }
  m.state = OFF;
  m.haveFeedback = false;
}

void disableAll(const char* reason) {
  for (int i = 0; i < MOTOR_COUNT; i++) disableOne(motors[i]);
  Serial.print("ALL DISABLED: ");
  Serial.println(reason);
}

void enableOne(Motor &m) {
  m.haveFeedback = false;
  m.err = 0;
  m.rxCount = 0;
  sendSpecial(m, 0xFC);
  m.state = FREE;
  m.lastEnableMs = millis();
  m.lastFeedbackMs = millis();
}

bool ready(const Motor &m) {
  return m.haveFeedback && m.state != OFF;
}

void startHold(Motor &m) {
  m.holdCenter = m.pos;
  m.holdCmd = m.pos;
  m.holdGoal = m.pos;
  m.state = HOLD;
}

void stepHold(Motor &m, float delta) {
  float goal = m.holdGoal + m.dir * delta;
  goal = constrain(goal, m.holdCenter - HOLD_RANGE_RAD, m.holdCenter + HOLD_RANGE_RAD);
  m.holdGoal = goal;
}

void startSpin(Motor &m, float sign) {
  m.spinTarget = sign * m.dir * SPIN_SPEED;
  m.spinCmd = 0.0f;
  m.spinStartMs = millis();
  m.state = SPIN;
}

// ---------------------------------------------------------------- commands

void selectMotors(bool left, bool right) {
  motors[0].selected = left;
  motors[1].selected = right;
  Serial.print("Selected: ");
  Serial.println(left && right ? "both" : left ? "left" : "right");
}

// Every selected motor must be ready, otherwise the command is refused for all.
bool allSelectedReady(const char* what) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (motors[i].selected && !ready(motors[i])) {
      Serial.print(what);
      Serial.print(" refused: ");
      Serial.print(motors[i].name);
      Serial.println(" has no feedback. Send e first.");
      return false;
    }
  }
  return true;
}

void handleCommand(char c) {
  switch (c) {
    case '0': selectMotors(true, true); return;
    case '1': selectMotors(true, false); return;
    case '2': selectMotors(false, true); return;
    case 'x': disableAll("user x"); return;
    default: break;
  }

  if (c == 'e') {
    for (int i = 0; i < MOTOR_COUNT; i++) if (motors[i].selected) enableOne(motors[i]);
    Serial.println("ENABLE sent to selected (free, zero torque)");
  } else if (c == 'd') {
    for (int i = 0; i < MOTOR_COUNT; i++) if (motors[i].selected) disableOne(motors[i]);
    Serial.println("Selected disabled");
  } else if (c == 'h') {
    if (!allSelectedReady("h")) return;
    for (int i = 0; i < MOTOR_COUNT; i++) {
      Motor &m = motors[i];
      if (!m.selected) continue;
      if (m.state == SPIN) { Serial.println("Spinning. Send s first."); return; }
      if (fabs(m.pos) > 0.9f * P_MAX) { Serial.println("Position out of range. Send z while free, then h."); return; }
    }
    for (int i = 0; i < MOTOR_COUNT; i++) if (motors[i].selected) startHold(motors[i]);
    Serial.println("HOLD selected at current position");
  } else if (c == '+' || c == '-' || c == 'c') {
    for (int i = 0; i < MOTOR_COUNT; i++) {
      Motor &m = motors[i];
      if (!m.selected || m.state != HOLD) continue;
      if (c == 'c') m.holdGoal = m.holdCenter;
      else stepHold(m, c == '+' ? STEP_RAD : -STEP_RAD);
    }
    Serial.println(c == 'c' ? "Back to hold center" : "Step");
  } else if (c == 'a' || c == 'b') {
    if (!BENCH_SPIN_ALLOWED) { Serial.println("Spin disabled (BENCH_SPIN_ALLOWED = false)."); return; }
    if (!allSelectedReady("spin")) return;
    for (int i = 0; i < MOTOR_COUNT; i++) if (motors[i].selected) startSpin(motors[i], c == 'a' ? 1.0f : -1.0f);
    Serial.println("SPIN start (selected)");
  } else if (c == 's') {
    for (int i = 0; i < MOTOR_COUNT; i++) {
      Motor &m = motors[i];
      if (m.selected && m.state == SPIN) { m.spinTarget = 0.0f; m.spinStartMs = 0; }
    }
    Serial.println("SPIN stopping");
  } else if (c == 'z') {
    for (int i = 0; i < MOTOR_COUNT; i++) {
      Motor &m = motors[i];
      if (!m.selected) continue;
      if (m.state != OFF && m.state != FREE) { Serial.println("z only when disabled or free."); return; }
    }
    for (int i = 0; i < MOTOR_COUNT; i++) if (motors[i].selected) sendSpecial(motors[i], 0xFE);
    Serial.println("Zero set (selected)");
  }
}

// ---------------------------------------------------------------- 2 ms cycle

void updateAndSend(Motor &m) {
  const float dt = PERIOD_US * 1e-6f;
  switch (m.state) {
    case OFF:
      return;
    case FREE:
      if (!m.haveFeedback && millis() - m.lastEnableMs >= 100) {
        sendSpecial(m, 0xFC);  // resend enable until feedback arrives
        m.lastEnableMs = millis();
        return;
      }
      sendMit(m, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      return;
    case HOLD: {
      float step = HOLD_RATE * dt;
      float diff = m.holdGoal - m.holdCmd;
      m.holdCmd = (fabs(diff) <= step) ? m.holdGoal : m.holdCmd + (diff > 0.0f ? step : -step);
      sendMit(m, m.holdCmd, 0.0f, HOLD_KP, HOLD_KD, 0.0f);
      return;
    }
    case SPIN: {
      float rampS = SPIN_SPEED / SPIN_ACCEL;
      float elapsed = (millis() - m.spinStartMs) / 1000.0f;
      bool stopping = (m.spinTarget == 0.0f) || (elapsed > rampS + SPIN_HOLD_S);
      float goal = stopping ? 0.0f : m.spinTarget;
      float step = SPIN_ACCEL * dt;
      m.spinCmd = (fabs(goal - m.spinCmd) <= step) ? goal : m.spinCmd + (goal > m.spinCmd ? step : -step);
      sendMit(m, 0.0f, m.spinCmd, 0.0f, SPIN_KD, 0.0f);
      if (stopping && m.spinCmd == 0.0f && fabs(m.vel) < 0.3f) {
        sendSpecial(m, 0xFE);  // position range is only +-12.5 rad: re-zero after a long spin
        m.state = BRAKE;
        Serial.print(m.name);
        Serial.println(" spin done, zero set, braking");
      }
      return;
    }
    case BRAKE:
      sendMit(m, 0.0f, 0.0f, 0.0f, SPIN_KD, 0.0f);
      return;
  }
}

// Returns a reason string if this motor must stop, otherwise nullptr.
const char* checkMotor(const Motor &m) {
  if (m.state == OFF) return nullptr;
  uint32_t limit = m.haveFeedback ? FEEDBACK_TIMEOUT_MS : FIRST_FEEDBACK_MS;
  if (millis() - m.lastFeedbackMs > limit) return "no feedback";
  if (m.err >= 8) return "motor error code";
  if (m.rotorTemp >= MAX_ROTOR_TEMP) return "rotor temperature";
  if (fabs(m.torque) > MAX_TORQUE) return "torque limit";
  if (m.state == HOLD && fabs(m.pos - m.holdCmd) > MAX_POSITION_ERROR) return "position error";
  return nullptr;
}

void checkSafety() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    const char* reason = checkMotor(motors[i]);
    if (reason == nullptr) continue;
    Serial.print(motors[i].name);
    Serial.print(": ");
    Serial.print(reason);
    if (motors[i].err >= 8) { Serial.print(" 0x"); Serial.print(motors[i].err, HEX); }
    Serial.print(" (frames received ");
    Serial.print(motors[i].rxCount);
    Serial.println(")");
    disableAll("fault on one motor");
    return;
  }
}

void printStatus() {
  if (millis() - lastPrintMs < 200) return;
  lastPrintMs = millis();
  bool any = false;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    const Motor &m = motors[i];
    if (m.state == OFF) continue;
    any = true;
    Serial.print(m.name);
    Serial.print(" ");
    Serial.print(stateName(m.state));
    Serial.print(" p=");
    Serial.print(m.pos, 3);
    Serial.print(" v=");
    Serial.print(m.vel, 2);
    Serial.print(" t=");
    Serial.print(m.torque, 2);
    Serial.print(" T=");
    Serial.print(m.rotorTemp);
    Serial.print(" | ");
  }
  if (!any) return;
  Serial.print("cycle max ");
  Serial.print(worstCycleUs);
  Serial.println(" us");
  worstCycleUs = 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  can1.begin();
  can1.setBaudRate(1000000);
  Serial.println("Ready (2 motors). Commands: 0 1 2 | e h + - c a b s z d x");
}

void loop() {
  readFeedback();  // runs every pass, not only every 2 ms

  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r' && c != ' ') handleCommand(c);
  }

  uint32_t now = micros();
  if (now - lastCycleUs >= PERIOD_US) {
    lastCycleUs = now;
    for (int i = 0; i < MOTOR_COUNT; i++) updateAndSend(motors[i]);
    checkSafety();
    uint32_t used = micros() - now;
    if (used > worstCycleUs) worstCycleUs = used;
  }

  printStatus();
}
