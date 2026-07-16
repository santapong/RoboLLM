/*
 * Panda arm servo firmware — what the Arduino actually receives.
 *
 * THE KEY IDEA
 * ------------
 * The Arduino does NOT do kinematics. The PC (ROS2) computes IK and
 * trajectories, then streams TARGET JOINT ANGLES over serial. This board
 * just maps each angle to a servo PWM signal. That's the whole job.
 *
 * SERIAL PROTOCOL (115200 baud, one ASCII line per command, ~30 Hz)
 * -----------------------------------------------------------------
 *   PC -> Arduino:   S,<j1>,<j2>,<j3>,<j4>,<j5>,<j6>,<j7>,<grip>\n
 *       All values are SERVO DEGREES 0..180. The PC already converted
 *       each joint from radians to the servo's range:
 *           servo_deg = (q_rad - joint_lower) / (joint_upper - joint_lower) * 180
 *       <grip> is the gripper servo: 0 = fully open, 60 = closed.
 *
 *   Arduino -> PC:   ACK,<seq>,<millis>\n     (confirms each command)
 *
 * A real 7-DOF arm this size would use serial-bus servos (Dynamixel) or
 * stepper drivers, but the information exchanged is identical: target
 * joint positions go down, acknowledgements/feedback come back.
 */
#include <Servo.h>

const int NUM_JOINTS = 7;
const int SERVO_PINS[NUM_JOINTS] = {3, 5, 6, 9, 10, 11, 12};
const int GRIPPER_PIN = 13;

// Limit how fast a servo may move (deg per 20 ms tick) so a bad command
// can't slam the arm at full speed.
const float MAX_STEP_DEG = 2.0;

Servo joints[NUM_JOINTS];
Servo gripper;
float target_deg[NUM_JOINTS + 1];   // last commanded targets
float current_deg[NUM_JOINTS + 1];  // where the servos actually are
unsigned long seq = 0;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_JOINTS; i++) {
    joints[i].attach(SERVO_PINS[i]);
    current_deg[i] = target_deg[i] = 90.0;  // mid-range on power-up
    joints[i].write(90);
  }
  gripper.attach(GRIPPER_PIN);
  current_deg[NUM_JOINTS] = target_deg[NUM_JOINTS] = 0.0;  // open
  gripper.write(0);
  Serial.println("READY,panda_arm_servo,v1");
}

void parseCommand(char *line) {
  // Expected: S,<j1>,...,<j7>,<grip>
  char *tok = strtok(line, ",");
  if (tok == NULL || tok[0] != 'S') return;
  for (int i = 0; i <= NUM_JOINTS; i++) {
    tok = strtok(NULL, ",");
    if (tok == NULL) return;                    // malformed -> ignore line
    target_deg[i] = constrain(atof(tok), 0.0, 180.0);
  }
  seq++;
  Serial.print("ACK,");
  Serial.print(seq);
  Serial.print(",");
  Serial.println(millis());
}

void loop() {
  // 1. Read complete lines from serial
  static char buf[96];
  static int len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buf[len] = '\0';
      parseCommand(buf);
      len = 0;
    } else if (len < 95) {
      buf[len++] = c;
    }
  }

  // 2. Every 20 ms, slew each servo toward its target (rate limiting)
  static unsigned long last = 0;
  if (millis() - last >= 20) {
    last = millis();
    for (int i = 0; i <= NUM_JOINTS; i++) {
      float d = target_deg[i] - current_deg[i];
      d = constrain(d, -MAX_STEP_DEG, MAX_STEP_DEG);
      current_deg[i] += d;
      if (i < NUM_JOINTS) joints[i].write((int)current_deg[i]);
      else gripper.write((int)current_deg[i]);
    }
  }
}
