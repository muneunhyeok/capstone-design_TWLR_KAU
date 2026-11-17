// DM-J4340P-2EC single motor test for Teensy 4.1
// CAN1 pins: 22 = CTX1, 23 = CRX1. Bitrate 1 Mbps.
// Commands: e enable free, h hold, a spin + (5 s), b spin - (5 s), s stop, d disable

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

const uint16_t CAN_ID = 0x01;

const float P_MAX = 12.5f;
const float V_MAX = 10.0f;
const float T_MAX = 28.0f;
const float KP_MAX = 500.0f;
const float KD_MAX = 5.0f;

const float HOLD_KP = 5.0f;
const float HOLD_KD = 0.5f;

// ---- spin settings (edit here) ----
// Max no-load speed at 24 V is 52 rpm = 5.45 rad/s (manual). 4.0 rad/s = 38 rpm = ~73 % of it.
const float SPIN_SPEED = 4.0f;      // rad/s, target speed. 1 rad/s = 9.55 rpm
const float SPIN_KD = 1.0f;         // torque = KD * (target speed - speed). Must not be 0
const float SPIN_ACCEL = 2.0f;      // rad/s^2, ramp up/down rate
const float SPIN_HOLD_S = 5.0f;     // seconds at constant speed
const float MAX_SPIN_TORQUE = 6.0f; // Nm, stop if |measured torque| exceeds this (rated 9)
const float MAX_POSITION_ERROR = 1.0f;
const int MAX_ROTOR_TEMP = 60;

enum State { STATE_OFF, STATE_FREE, STATE_HOLD, STATE_SPIN, STATE_BRAKE };
State state = STATE_OFF;

float measuredPos = 0.0f;
float measuredVel = 0.0f;
float measuredTorque = 0.0f;
int mosTemp = 0;
int rotorTemp = 0;
int motorErr = 0;
bool haveFeedback = false;

float centerPos = 0.0f;
float commandPos = 0.0f;
float goalPos = 0.0f;

float spinTarget = 0.0f;
float spinCmd = 0.0f;
uint32_t spinStartMs = 0;

uint32_t lastFeedbackMs = 0;
uint32_t lastSendUs = 0;
uint32_t lastPrintMs = 0;
uint32_t lastEnableMs = 0;
uint32_t rxCount = 0;
CAN_message_t lastRx;

uint16_t floatToUint(float value, float minValue, float maxValue, int bits) {
  value = constrain(value, minValue, maxValue);
  float span = maxValue - minValue;
  return (uint16_t)((value - minValue) * ((1UL << bits) - 1) / span);
}

float uintToFloat(uint16_t value, float minValue, float maxValue, int bits) {
  float span = maxValue - minValue;
  return value * span / ((1UL << bits) - 1) + minValue;
}

void sendSpecial(uint8_t lastByte) {
  CAN_message_t msg;
  msg.id = CAN_ID;
  msg.len = 8;
  for (int i = 0; i < 7; i++) msg.buf[i] = 0xFF;
  msg.buf[7] = lastByte;
  can1.write(msg);
}

void sendMit(float pos, float vel, float kp, float kd, float torque) {
  uint16_t p = floatToUint(pos, -P_MAX, P_MAX, 16);
  uint16_t v = floatToUint(vel, -V_MAX, V_MAX, 12);
  uint16_t k = floatToUint(kp, 0.0f, KP_MAX, 12);
  uint16_t d = floatToUint(kd, 0.0f, KD_MAX, 12);
  uint16_t t = floatToUint(torque, -T_MAX, T_MAX, 12);

  CAN_message_t msg;
  msg.id = CAN_ID;
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

void disableMotor(const char* reason) {
  for (int i = 0; i < 3; i++) {
    sendSpecial(0xFD);
    delay(2);
  }
  state = STATE_OFF;
  haveFeedback = false;
  Serial.print("DISABLED: ");
  Serial.println(reason);
}

void readFeedback() {
  CAN_message_t msg;
  while (can1.read(msg)) {
    rxCount++;
    lastRx = msg;
    if (msg.len != 8 || (msg.buf[0] & 0x0F) != (CAN_ID & 0x0F)) continue;

    motorErr = msg.buf[0] >> 4;
    uint16_t p = (msg.buf[1] << 8) | msg.buf[2];
    uint16_t v = (msg.buf[3] << 4) | (msg.buf[4] >> 4);
    uint16_t t = ((msg.buf[4] & 0x0F) << 8) | msg.buf[5];

    measuredPos = uintToFloat(p, -P_MAX, P_MAX, 16);
    measuredVel = uintToFloat(v, -V_MAX, V_MAX, 12);
    measuredTorque = uintToFloat(t, -T_MAX, T_MAX, 12);
    mosTemp = msg.buf[6];
    rotorTemp = msg.buf[7];
    haveFeedback = true;
    lastFeedbackMs = millis();
  }
}

void handleCommand(char command) {
  switch (command) {
    case 'e':
      haveFeedback = false;
      motorErr = 0;
      rxCount = 0;
      sendSpecial(0xFC);
      state = STATE_FREE;
      lastEnableMs = millis();
      lastFeedbackMs = millis();
      Serial.println("ENABLE sent: free mode, zero torque");
      break;
    case 'h':
      if (state == STATE_OFF || !haveFeedback) {
        Serial.println("Send e first and wait for feedback.");
        break;
      }
      if (state == STATE_SPIN) {
        Serial.println("Spinning. Send s to stop first.");
        break;
      }
      if (fabs(measuredPos) > 0.9f * P_MAX) {
        Serial.println("Position out of range. Send d, then e again.");
        break;
      }
      centerPos = measuredPos;
      commandPos = measuredPos;
      goalPos = measuredPos;
      state = STATE_HOLD;
      Serial.println("HOLD: current position");
      break;
    case 'a':
    case 'b':
      if (!haveFeedback || (state != STATE_FREE && state != STATE_HOLD && state != STATE_BRAKE)) {
        Serial.println("Send e (or h) first and wait for feedback.");
        break;
      }
      spinTarget = (command == 'a') ? SPIN_SPEED : -SPIN_SPEED;
      spinCmd = 0.0f;
      spinStartMs = millis();
      state = STATE_SPIN;
      Serial.print("SPIN start: target = ");
      Serial.print(spinTarget, 2);
      Serial.println(" rad/s");
      break;
    case 's':
      if (state == STATE_SPIN) {
        spinTarget = 0.0f;
        spinStartMs = 0;  // force ramp-down phase
        Serial.println("SPIN: stopping");
      }
      break;
    case 'd':
      disableMotor("user command");
      break;
    default:
      break;
  }
}

void checkSafety() {
  if (state == STATE_OFF) return;

  uint32_t feedbackLimit = haveFeedback ? 200 : 1000;
  if (millis() - lastFeedbackMs > feedbackLimit) {
    Serial.print("CAN frames received = ");
    Serial.println(rxCount);
    if (rxCount > 0) {
      Serial.print("Last frame ID = 0x");
      Serial.print(lastRx.id, HEX);
      Serial.print(" data =");
      for (int i = 0; i < lastRx.len; i++) {
        Serial.print(" ");
        Serial.print(lastRx.buf[i], HEX);
      }
      Serial.println();
    }
    disableMotor("no feedback");
    return;
  }
  if (motorErr >= 8) {
    Serial.print("Motor error code = 0x");
    Serial.println(motorErr, HEX);
    disableMotor("motor fault");
    return;
  }
  if (rotorTemp >= MAX_ROTOR_TEMP) {
    disableMotor("rotor temperature limit");
    return;
  }
  if (state == STATE_SPIN && fabs(measuredTorque) > MAX_SPIN_TORQUE) {
    disableMotor("spin torque limit");
    return;
  }
  if (state == STATE_HOLD && fabs(measuredPos - commandPos) > MAX_POSITION_ERROR) {
    disableMotor("position error too large");
  }
}

void sendControl() {
  uint32_t now = micros();
  if (now - lastSendUs < 5000) return;
  lastSendUs = now;

  if (state == STATE_FREE) {
    if (!haveFeedback && millis() - lastEnableMs >= 100) {
      sendSpecial(0xFC);
      lastEnableMs = millis();
      return;
    }
    sendMit(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  } else if (state == STATE_HOLD) {
    float step = 0.3f * 0.005f;
    float difference = goalPos - commandPos;
    if (fabs(difference) <= step) commandPos = goalPos;
    else commandPos += (difference > 0.0f) ? step : -step;
    sendMit(commandPos, 0.0f, HOLD_KP, HOLD_KD, 0.0f);
  } else if (state == STATE_SPIN) {
    // trapezoid: ramp up -> hold SPIN_HOLD_S -> ramp down
    float rampS = fabs(spinTarget == 0.0f ? spinCmd : spinTarget) / SPIN_ACCEL;
    float elapsed = (millis() - spinStartMs) / 1000.0f;
    bool stopping = (spinTarget == 0.0f) || (elapsed > rampS + SPIN_HOLD_S);
    float goal = stopping ? 0.0f : spinTarget;
    float step = SPIN_ACCEL * 0.005f;
    if (fabs(goal - spinCmd) <= step) spinCmd = goal;
    else spinCmd += (goal > spinCmd) ? step : -step;
    sendMit(0.0f, spinCmd, 0.0f, SPIN_KD, 0.0f);
    if (stopping && spinCmd == 0.0f && fabs(measuredVel) < 0.3f) {
      sendSpecial(0xFE);  // set current position as zero (position range is only +-12.5 rad)
      state = STATE_BRAKE;
      Serial.println("SPIN done. Zero set, braking. Send h to hold, a/b to spin again, d to disable.");
    }
  } else if (state == STATE_BRAKE) {
    sendMit(0.0f, 0.0f, 0.0f, SPIN_KD, 0.0f);  // gentle damping, no position target
  }
}

void printStatus() {
  if (millis() - lastPrintMs < 200) return;
  lastPrintMs = millis();
  if (state == STATE_OFF || !haveFeedback) return;

  Serial.print(state == STATE_FREE ? "FREE" : state == STATE_HOLD ? "HOLD" : state == STATE_SPIN ? "SPIN" : "BRAKE");
  Serial.print(" pos=");
  Serial.print(measuredPos, 3);
  Serial.print(" rad vel=");
  Serial.print(measuredVel, 3);
  Serial.print(" rad/s torque=");
  Serial.print(measuredTorque, 3);
  Serial.print(" Nm tMOS=");
  Serial.print(mosTemp);
  Serial.print(" tRotor=");
  Serial.print(rotorTemp);
  Serial.print(" err=0x");
  Serial.println(motorErr, HEX);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  can1.begin();
  can1.setBaudRate(1000000);
  Serial.println("Ready. Commands: e h a b s d");
}

void loop() {
  readFeedback();

  while (Serial.available()) {
    char command = Serial.read();
    if (command != '\n' && command != '\r' && command != ' ') handleCommand(command);
  }

  sendControl();
  checkSafety();
  printStatus();
}
