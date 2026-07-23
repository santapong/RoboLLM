"""uno_link.py — drive the project's REAL DIY arm from the panda demos.

The panda demos normally stream `S,d1,…,d7,grip` fire-and-forget lines at
50 Hz (see arduino/panda_arm_servo.ino). The project's actual hardware
(../../hardware: Uno R3 + arm_firmware) speaks a REQUEST-REPLY protocol
instead, via ArmSerial:

    PING            -> PONG arm-fw 1.0
    E <0|1>         -> OK            (servo torque on/off)
    S <ch> <deg>    -> OK            (ONE joint at a time, int degrees)
    G               -> A d0 d1 ... d5

This adapter makes the demos speak that protocol, so demo 03/04/05 can move
the real arm (or hardware/sim_uno.py) instead of the educational simulator.

RETARGETING 7 DOF -> 6 CHANNELS
-------------------------------
The DIY arm has 6 servos, the Panda has 7 joints + gripper. We map the five
most important joints and the gripper, dropping the two redundant wrist/elbow
rolls (a common "retargeting" trick — the arm still reaches the same x/y/z,
just without the extra null-space freedom):

    uno ch 0 = panda_joint1 (base yaw)      ch 3 = panda_joint6 (wrist pitch)
    uno ch 1 = panda_joint2 (shoulder)      ch 4 = panda_joint7 (wrist roll)
    uno ch 2 = panda_joint4 (elbow)         ch 5 = gripper (0=open, 60=closed)

Because request-reply BLOCKS while waiting for each `OK`, commands are sent
from a background thread (20 Hz, changed channels only) so the GUI never
stalls — the same reason real drivers are asynchronous.

Usage: run any demo with
    PANDA_PROTOCOL=uno ARM_PORT=/dev/pts/N ros2 launch ...   # sim_uno.py
    PANDA_PROTOCOL=uno ros2 launch ...                       # real Uno (auto-detect)
"""
import os
import sys
import threading
import time

_HW_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', '..', 'hardware')
sys.path.insert(0, _HW_DIR)
from arm_serial import ArmSerial  # noqa: E402  (the project's real driver)

# panda joint index driven by each uno channel 0..4; channel 5 is the gripper
CHANNEL_JOINTS = [0, 1, 3, 5, 6]
GRIP_OPEN_M, GRIP_CLOSED_M = 0.035, 0.02   # finger joint value -> servo deg


class UnoLink:
    """Same interface as the demos' SerialLink (send_joints / read_rx / error),
    but speaking the hardware/ request-reply protocol via ArmSerial."""

    def __init__(self, port=None):
        self.error = None
        self._lock = threading.Lock()
        self._desired = None            # latest wanted [6] servo degrees
        self._log = []
        try:
            self.arm = ArmSerial(port=port)
            pong = self.arm.ping()
            self.arm.enable(True)
            with self._lock:
                self._log += [f'RX  {pong}', 'TX  E 1   RX  OK (torque on)']
            threading.Thread(target=self._worker, daemon=True).start()
        except Exception as e:
            self.arm = None
            self.error = str(e)

    # ---- demo-facing interface (called at 50 Hz from the GUI tick) ----
    def send_joints(self, kin, q, finger):
        servo = [(q[j] - kin.lower[j]) / (kin.upper[j] - kin.lower[j]) * 180.0
                 for j in CHANNEL_JOINTS]
        grip = (GRIP_OPEN_M - finger) / (GRIP_OPEN_M - GRIP_CLOSED_M) * 60.0
        servo.append(max(0.0, min(180.0, grip)))
        with self._lock:
            self._desired = servo
        return 'UNO ' + ' '.join(f'{v:.0f}' for v in servo)

    def read_rx(self):
        with self._lock:
            out, self._log = self._log, []
        return out

    # ---- background sender (20 Hz, changed channels only) ----
    def _worker(self):
        last_sent = [None] * 6
        ticks = 0
        while self.arm is not None:
            time.sleep(0.05)
            with self._lock:
                desired = self._desired
            if desired is None:
                continue
            try:
                for ch, deg in enumerate(desired):
                    d = int(round(deg))
                    if last_sent[ch] != d:
                        self.arm.set_joint(ch, d)      # 'S ch d' -> 'OK'
                        last_sent[ch] = d
                        with self._lock:
                            self._log.append(f'TX  S {ch} {d}   RX  OK')
                ticks += 1
                if ticks % 20 == 0:                     # 1 Hz position poll
                    joints = self.arm.get_joints()      # 'G' -> 'A ...'
                    with self._lock:
                        self._log.append('RX  A ' + ' '.join(map(str, joints)))
            except Exception as e:
                self.error = f'serial lost: {e}'
                return


if __name__ == '__main__':
    # Self-test against hardware/sim_uno.py (or the real Uno):
    #   python3 hardware/sim_uno.py            # note the /dev/pts/N it prints
    #   ARM_PORT=/dev/pts/N python3 examples/panda_arm/uno_link.py
    import numpy as np
    from ament_index_python.packages import get_package_share_directory
    from panda_kinematics import PandaKinematics

    urdf = os.path.join(get_package_share_directory(
        'moveit_resources_panda_description'), 'urdf', 'panda.urdf')
    kin = PandaKinematics(open(urdf).read())
    link = UnoLink()
    if link.error:
        print('FAILED to connect:', link.error)
        sys.exit(1)

    ready = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    print('sending ready pose ->', link.send_joints(kin, ready, 0.035))
    time.sleep(4)                       # let the fw slew (100 deg/s)
    for line in link.read_rx():
        print(' ', line)
    got = link.arm.get_joints()
    want = [int(round((ready[j] - kin.lower[j]) /
                      (kin.upper[j] - kin.lower[j]) * 180)) for j in CHANNEL_JOINTS]
    errs = [abs(g - w) for g, w in zip(got[:5], want)]
    print(f'target {want} + grip 0  |  arm reports {got}  |  errs {errs}')
    print('SELF-TEST', 'PASSED' if max(errs) <= 2 else 'FAILED')
