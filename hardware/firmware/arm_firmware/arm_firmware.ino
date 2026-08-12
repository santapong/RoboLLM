// arm_firmware.ino — arm-fw 2.1: fail-closed 6-DOF arm servo controller.
//
// The Uno is a dumb servo controller; all planning/kinematics lives on the host
// (2 KB RAM can't run micro-ROS — plain serial is the right transport).
//
// v2.1 adds generated per-arm limits, a commissioning-only single-joint
// command, strict invalid-command rejection, and a communication watchdog.
//
// Protocol (115200 baud, one command per line, every motion/state command
// replies with ONE state line):
//   Pi -> Uno:
//     S d0 d1 d2 d3 d4 d5 g   set all targets (calibrated mode only)        -> s
//     C channel deg            move one joint (commissioning mode only)     -> s
//     Q                        query state, no motion                        -> s
//     H                        go to home pose                               -> s
//     E                        enable torque (attach servos)                 -> s
//     X                        e-stop: detach servos (safe, hand-movable)    -> s
//     P                        ping                                          -> # pong arm-fw 2.1
//     L 1 | L 0                onboard LED, smoke test without servos        -> # led
//   Uno -> Pi:
//     s d0..d5 g t_ms          MEASURED angles (deg), gripper, millis()
//     # ...                    comment / banner        ! ...    error
//
// KEY IDEA: we COMMAND servos open-loop but REPORT what the encoders measure.
// Command != measurement — that is what makes recorded demonstrations honest
// (research hypothesis H5). readEncoderDeg() is stubbed to return the slewed
// commanded position until real encoders are wired: that stub IS the
// "commanded state" baseline. Swap it out before recording real datasets.
//
// Wiring: servo POWER from an external 5-6 V supply (NOT the Uno's 5V pin),
// grounds tied together.

#include <Servo.h>
#include <stdlib.h>
#include "arm_config.h"

const uint8_t LED_PIN = 13;

Servo servo[NJOINTS];
Servo gripper;

float current[NJOINTS];  // slewed position actually written to servos (deg)
float target[NJOINTS];   // last commanded target (deg)
float gCurrent = GHOME, gTarget = GHOME;
bool  enabled = false;
int8_t commissioning_channel = -1;
unsigned long last_tick = 0;
unsigned long last_command = 0;

// ------------------------------------------------------------------
// ENCODER READ — replace with the real encoder code per joint.
//   Case 1 analog pot:  map(analogRead(POT_PIN[i]), RAW_MIN, RAW_MAX, JMIN, JMAX)
//   Case 2 AS5600 I2C (via mux): read raw angle register, scale to degrees
//   Case 3 quadrature: counts[i] * DEG_PER_COUNT[i] + offset[i]
// Until then it returns the SLEWED COMMANDED position — the honest label for
// that is "commanded state" (H5 baseline), not a measurement.
// ------------------------------------------------------------------
float readEncoderDeg(uint8_t i) {
  return current[i];              // TODO: real encoder read for joint i
}
float readGripper() {
  return gCurrent;                // TODO: replace if the gripper has feedback
}

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void sendState() {
  Serial.print('s');
  for (uint8_t i = 0; i < NJOINTS; i++) {
    Serial.print(' ');
    Serial.print(readEncoderDeg(i), 2);
  }
  Serial.print(' ');
  Serial.print(readGripper(), 1);
  Serial.print(' ');
  Serial.println(millis());
}

void attachAll() {
  for (uint8_t i = 0; i < NJOINTS; i++) {
    servo[i].attach(SERVO_PIN[i]);
    servo[i].write((int)round(current[i]));
  }
  gripper.attach(GRIPPER_PIN);
  gripper.write((int)round(map((long)gCurrent, 0, 100, 0, 180)));
  enabled = true;
  commissioning_channel = -1;
  last_command = millis();
}

void detachAll() {
  for (uint8_t i = 0; i < NJOINTS; i++) servo[i].detach();
  gripper.detach();
  enabled = false;
  commissioning_channel = -1;
}

void attachCommissionJoint(uint8_t channel) {
  if (!enabled || commissioning_channel != channel) {
    detachAll();
    servo[channel].attach(SERVO_PIN[channel]);
    servo[channel].write((int)round(current[channel]));
    enabled = true;
    commissioning_channel = channel;
  }
  last_command = millis();
}

// Parse "d0 d1 d2 d3 d4 d5 g" (after the 'S').
bool parseFloatToken(char *token, float *result) {
  if (!token || !*token) return false;
  char *end;
  double value = strtod(token, &end);
  if (*end != '\0' || !isfinite(value)) return false;
  *result = (float)value;
  return true;
}

bool parseSet(char *args) {
  float tmp[NJOINTS]; float g;
  char *tok = strtok(args, " ");
  for (uint8_t i = 0; i < NJOINTS; i++) {
    if (!parseFloatToken(tok, &tmp[i])) return false;
    tok = strtok(NULL, " ");
  }
  if (!parseFloatToken(tok, &g) || strtok(NULL, " ") != NULL) return false;
  for (uint8_t i = 0; i < NJOINTS; i++) {
    if (tmp[i] < JMIN[i] || tmp[i] > JMAX[i]) return false;
  }
  if (g < GMIN || g > GMAX) return false;
  for (uint8_t i = 0; i < NJOINTS; i++) target[i] = tmp[i];
  gTarget = g;
  attachCommissionJoint((uint8_t)channel);
  return true;
}

bool parseCommission(char *args) {
  char *channelToken = strtok(args, " ");
  char *degreeToken = strtok(NULL, " ");
  if (!channelToken || !degreeToken || strtok(NULL, " ") != NULL) return false;
  char *end;
  long channel = strtol(channelToken, &end, 10);
  if (*end != '\0' || channel < 0 || channel >= NJOINTS) return false;
  float degrees;
  if (!parseFloatToken(degreeToken, &degrees)) return false;
  if (degrees < JMIN[channel] || degrees > JMAX[channel]) return false;
  target[channel] = degrees;
  if (!enabled) attachAll();
  last_command = millis();
  return true;
}

char buf[96];
uint8_t blen = 0;

void handleLine(char *line) {
  switch (line[0]) {
    case 'S':
      if (!ARM_CALIBRATED) Serial.println("! not_calibrated");
      else if (!parseSet(line + 1)) Serial.println("! bad_or_unsafe_cmd");
      sendState(); break;
    case 'C':
      if (ARM_CALIBRATED) Serial.println("! commissioning_disabled");
      else if (!parseCommission(line + 1)) Serial.println("! bad_or_unsafe_cmd");
      sendState(); break;
    case 'Q':
      if (line[1] != '\0') Serial.println("! bad_cmd");
      else last_command = millis();
      sendState(); break;
    case 'H':
      if (line[1] != '\0') Serial.println("! bad_cmd");
      else if (!ARM_CALIBRATED) Serial.println("! not_calibrated");
      else {
        for (uint8_t i = 0; i < NJOINTS; i++) target[i] = HOME_DEG[i];
        gTarget = GHOME;
        if (!enabled) attachAll();
        last_command = millis();
      }
      sendState(); break;
    case 'E':
      if (line[1] != '\0') Serial.println("! bad_cmd");
      else if (!ARM_CALIBRATED) Serial.println("! not_calibrated");
      else { attachAll(); last_command = millis(); }
      sendState(); break;
    case 'X':
      if (line[1] != '\0') Serial.println("! bad_cmd");
      else detachAll();
      sendState(); break;
    case 'P':
      if (line[1] == '\0') Serial.println("# pong arm-fw 2.1");
      else Serial.println("! bad_cmd");
      break;
    case 'L':
      if (line[1] == ' ' && (line[2] == '0' || line[2] == '1') && line[3] == '\0') {
        digitalWrite(LED_PIN, line[2] == '1' ? HIGH : LOW);
        Serial.println("# led");
      } else Serial.println("! bad_cmd");
      break;
    case '\0': break;
    default:  Serial.println("! bad_cmd"); break;
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  for (uint8_t i = 0; i < NJOINTS; i++) { current[i] = HOME_DEG[i]; target[i] = HOME_DEG[i]; }
  Serial.begin(115200);
  for (uint8_t i = 0; i < 3; i++) {           // boot blink: firmware alive
    digitalWrite(LED_PIN, HIGH); delay(80);
    digitalWrite(LED_PIN, LOW);  delay(80);
  }
  // start relaxed & safe; the host sends 'E' / 'S' / 'H' to energize
  Serial.println("# ready arm-fw 2.1");
  if (!ARM_CALIBRATED) Serial.println("# commissioning_lock active");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      buf[blen] = '\0';
      if (blen > 0) handleLine(buf);
      blen = 0;
    } else if (blen < sizeof(buf) - 1) {
      buf[blen++] = c;
    } else {
      blen = 0;
      Serial.println("! overflow");
    }
  }

  unsigned long now = millis();
  if (enabled && now - last_command > COMMAND_TIMEOUT_MS) {
    detachAll();
    Serial.println("# command_timeout torque_off");
  }
  if (now - last_tick >= TICK_MS) {
    last_tick = now;
    for (uint8_t i = 0; i < NJOINTS; i++) {
      float d = clampf(target[i] - current[i], -MAX_STEP_DEG[i], MAX_STEP_DEG[i]);
      current[i] += d;
      if (enabled) servo[i].write((int)round(current[i]));
    }
    float dg = clampf(gTarget - gCurrent, -GMAX_STEP, GMAX_STEP);
    gCurrent += dg;
    if (enabled) gripper.write((int)round(map((long)gCurrent, 0, 100, 0, 180)));
  }
}
