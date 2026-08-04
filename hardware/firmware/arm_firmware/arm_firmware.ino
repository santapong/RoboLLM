// arm_firmware.ino — arm-fw 2.0: 6-DOF servo arm + gripper with MEASURED state.
//
// The Uno is a dumb servo controller; all planning/kinematics lives on the host
// (2 KB RAM can't run micro-ROS — plain serial is the right transport).
//
// v2 merges the Phase A research protocol (measured encoder state, batch set,
// timestamps, e-stop) with v1's slew-rate limiting and smoke tests.
//
// Protocol (115200 baud, one command per line, every motion/state command
// replies with ONE state line):
//   Pi -> Uno:
//     S d0 d1 d2 d3 d4 d5 g   set all joint targets (deg) + gripper (0-100) -> s
//     Q                        query state, no motion                        -> s
//     H                        go to home pose                               -> s
//     E                        enable torque (attach servos)                 -> s
//     X                        e-stop: detach servos (safe, hand-movable)    -> s
//     P                        ping                                          -> # pong arm-fw 2.0
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

const uint8_t NJOINTS = 6;
const uint8_t SERVO_PIN[NJOINTS] = {3, 5, 6, 9, 10, 11}; // PWM-capable pins
const uint8_t GRIPPER_PIN = 4;
const uint8_t LED_PIN = 13;

// Per-joint safety limits (deg) — tighten once the arm's real limits are known.
const float JMIN[NJOINTS] = {  0,   0,   0,   0,   0,   0};
const float JMAX[NJOINTS] = {180, 180, 180, 180, 180, 180};
const float GMIN = 0, GMAX = 100;
const float HOME_DEG[NJOINTS] = {90, 90, 90, 90, 90, 90};

const uint16_t TICK_MS = 20;     // 50 Hz slew update
const float MAX_STEP_DEG = 2.0;  // per tick == 100 deg/s

Servo servo[NJOINTS];
Servo gripper;

float current[NJOINTS];  // slewed position actually written to servos (deg)
float target[NJOINTS];   // last commanded target (deg)
float gCurrent = 0, gTarget = 0;
bool  enabled = false;
unsigned long last_tick = 0;

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
    servo[i].write((int)round(current[i]));
    servo[i].attach(SERVO_PIN[i]);
  }
  gripper.write((int)round(map((long)gCurrent, 0, 100, 0, 180)));
  gripper.attach(GRIPPER_PIN);
  enabled = true;
}

void detachAll() {
  for (uint8_t i = 0; i < NJOINTS; i++) servo[i].detach();
  gripper.detach();
  enabled = false;
}

// Parse "d0 d1 d2 d3 d4 d5 g" (after the 'S').
bool parseSet(char *args) {
  float tmp[NJOINTS]; float g;
  char *tok = strtok(args, " ");
  for (uint8_t i = 0; i < NJOINTS; i++) {
    if (!tok) return false;
    tmp[i] = atof(tok);
    tok = strtok(NULL, " ");
  }
  if (!tok) return false;
  g = atof(tok);

  bool oor = false;
  for (uint8_t i = 0; i < NJOINTS; i++) {
    float c = clampf(tmp[i], JMIN[i], JMAX[i]);
    if (c != tmp[i]) oor = true;
    target[i] = c;
  }
  float gc = clampf(g, GMIN, GMAX);
  if (gc != g) oor = true;
  gTarget = gc;

  if (!enabled) attachAll();
  if (oor) Serial.println("! out_of_range");
  return true;
}

char buf[96];
uint8_t blen = 0;

void handleLine(char *line) {
  switch (line[0]) {
    case 'S': if (!parseSet(line + 1)) Serial.println("! bad_cmd"); sendState(); break;
    case 'Q': sendState(); break;
    case 'H':
      for (uint8_t i = 0; i < NJOINTS; i++) target[i] = HOME_DEG[i];
      gTarget = 0;
      if (!enabled) attachAll();
      sendState(); break;
    case 'E': attachAll();  sendState(); break;
    case 'X': detachAll();  sendState(); break;
    case 'P': Serial.println("# pong arm-fw 2.0"); break;
    case 'L':
      digitalWrite(LED_PIN, (line[1] == ' ' && line[2] == '1') ? HIGH : LOW);
      Serial.println("# led"); break;
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
  Serial.println("# ready arm-fw 2.0");
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
  if (now - last_tick >= TICK_MS) {
    last_tick = now;
    for (uint8_t i = 0; i < NJOINTS; i++) {
      float d = clampf(target[i] - current[i], -MAX_STEP_DEG, MAX_STEP_DEG);
      current[i] += d;
      if (enabled) servo[i].write((int)round(current[i]));
    }
    float dg = clampf(gTarget - gCurrent, -MAX_STEP_DEG * 2, MAX_STEP_DEG * 2);
    gCurrent += dg;
    if (enabled) gripper.write((int)round(map((long)gCurrent, 0, 100, 0, 180)));
  }
}
