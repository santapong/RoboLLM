// arm_firmware.ino — DIY robot arm firmware for Arduino Uno R3
//
// The Uno is a dumb servo controller: it holds up to 6 hobby servos and obeys
// a tiny line-based serial protocol (115200 baud). All planning/kinematics
// lives on the host (Raspberry Pi 5 / laptop) — the Uno's 2 KB RAM can't run
// micro-ROS, so plain serial is the right transport.
//
// Protocol (one command per line, replies end with \n):
//   PING              -> PONG arm-fw 1.0
//   E 1 | E 0         -> attach/detach all servos (0 = torque off)  -> OK
//   S <ch> <deg>      -> move servo <ch> (0-5) to <deg> (clamped)   -> OK
//   G                 -> A <deg0> <deg1> <deg2> <deg3> <deg4> <deg5>
//   LED 1 | LED 0     -> onboard LED, smoke test without servos     -> OK
//   Unknown input     -> ERR <what>
//
// Movement is slew-rate limited (MAX_STEP_DEG every TICK_MS) so a big command
// doesn't slam the arm at full servo speed.
//
// Wiring: servo signal pins below; servo POWER must come from an external
// 5-6 V supply (NOT the Uno's 5V pin), with grounds tied together.

#include <Servo.h>

const uint8_t N_SERVOS = 6;
const uint8_t SERVO_PINS[N_SERVOS] = {3, 5, 6, 9, 10, 11}; // PWM-capable pins
const uint8_t LED_PIN = 13;

// Per-channel safe range — tighten these once you know your arm's limits.
const uint8_t MIN_DEG[N_SERVOS] = {0, 0, 0, 0, 0, 0};
const uint8_t MAX_DEG[N_SERVOS] = {180, 180, 180, 180, 180, 180};
const uint8_t HOME_DEG = 90;

const uint16_t TICK_MS = 20;      // 50 Hz update
const uint8_t MAX_STEP_DEG = 2;   // max degrees per tick (=100 deg/s)

Servo servos[N_SERVOS];
float current_deg[N_SERVOS];
float target_deg[N_SERVOS];
bool enabled = false;
unsigned long last_tick = 0;

char line[32];
uint8_t line_len = 0;

void attach_all() {
  for (uint8_t i = 0; i < N_SERVOS; i++) {
    servos[i].write((int)current_deg[i]);
    servos[i].attach(SERVO_PINS[i]);
  }
  enabled = true;
}

void detach_all() {
  for (uint8_t i = 0; i < N_SERVOS; i++) servos[i].detach();
  enabled = false;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  for (uint8_t i = 0; i < N_SERVOS; i++) {
    current_deg[i] = HOME_DEG;
    target_deg[i] = HOME_DEG;
  }
  Serial.begin(115200);
  // boot blink: 3 fast flashes = firmware alive
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(80);
    digitalWrite(LED_PIN, LOW);  delay(80);
  }
  Serial.println(F("READY arm-fw 1.0"));
}

void handle_line() {
  line[line_len] = '\0';
  if (strcmp(line, "PING") == 0) {
    Serial.println(F("PONG arm-fw 1.0"));
  } else if (strcmp(line, "G") == 0) {
    Serial.print(F("A"));
    for (uint8_t i = 0; i < N_SERVOS; i++) {
      Serial.print(' '); Serial.print((int)current_deg[i]);
    }
    Serial.println();
  } else if (strncmp(line, "E ", 2) == 0) {
    if (line[2] == '1') attach_all(); else detach_all();
    Serial.println(F("OK"));
  } else if (strncmp(line, "LED ", 4) == 0) {
    digitalWrite(LED_PIN, line[4] == '1' ? HIGH : LOW);
    Serial.println(F("OK"));
  } else if (strncmp(line, "S ", 2) == 0) {
    int ch = -1, deg = -1;
    if (sscanf(line + 2, "%d %d", &ch, &deg) == 2 && ch >= 0 && ch < N_SERVOS) {
      if (deg < MIN_DEG[ch]) deg = MIN_DEG[ch];
      if (deg > MAX_DEG[ch]) deg = MAX_DEG[ch];
      target_deg[ch] = deg;
      Serial.println(F("OK"));
    } else {
      Serial.println(F("ERR S <ch 0-5> <deg 0-180>"));
    }
  } else if (line_len > 0) {
    Serial.print(F("ERR unknown: ")); Serial.println(line);
  }
  line_len = 0;
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') handle_line();
    else if (line_len < sizeof(line) - 1) line[line_len++] = c;
    else line_len = 0; // overflow: drop the line
  }

  unsigned long now = millis();
  if (now - last_tick >= TICK_MS) {
    last_tick = now;
    for (uint8_t i = 0; i < N_SERVOS; i++) {
      float d = target_deg[i] - current_deg[i];
      if (d > MAX_STEP_DEG) d = MAX_STEP_DEG;
      if (d < -MAX_STEP_DEG) d = -MAX_STEP_DEG;
      current_deg[i] += d;
      if (enabled) servos[i].write((int)current_deg[i]);
    }
  }
}
