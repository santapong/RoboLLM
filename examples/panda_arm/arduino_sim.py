#!/usr/bin/env python3
"""Virtual Arduino: emulates arduino/panda_arm_servo.ino on a fake serial port.

Creates a pty (pseudo-terminal) pair and symlinks the PC-facing end to
/tmp/ttyPANDA — so any program that would talk to a real board at
/dev/ttyUSB0 can talk to /tmp/ttyPANDA instead, using the identical
protocol. When you get a real Arduino, flash the .ino and change the port;
nothing else in the system changes.

Implements the same behavior as the firmware:
  * parses  S,<j1..j7>,<grip>\n  (servo degrees 0-180)
  * replies ACK,<seq>,<millis>\n
  * rate-limits servo motion (2 deg / 20 ms) like the real loop() does
  * logs what the "servos" are doing
"""
import os
import pty
import select
import time

PORT_LINK = '/tmp/ttyPANDA'
NUM = 8  # 7 joints + gripper
MAX_STEP = 2.0  # deg per 20 ms tick


def main():
    master, slave = pty.openpty()
    if os.path.islink(PORT_LINK) or os.path.exists(PORT_LINK):
        os.remove(PORT_LINK)
    os.symlink(os.ttyname(slave), PORT_LINK)
    print(f'[arduino-sim] virtual board on {PORT_LINK} '
          f'(-> {os.ttyname(slave)})', flush=True)

    target = [90.0] * 7 + [0.0]
    current = list(target)
    seq = 0
    buf = b''
    t0 = time.time()
    last_tick = t0
    last_log = t0

    os.write(master, b'READY,panda_arm_servo,v1\n')
    while True:
        r, _, _ = select.select([master], [], [], 0.02)
        if r:
            try:
                data = os.read(master, 256)
            except OSError:
                break
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                parts = line.decode(errors='ignore').strip().split(',')
                if len(parts) == NUM + 1 and parts[0] == 'S':
                    try:
                        vals = [max(0.0, min(180.0, float(v))) for v in parts[1:]]
                    except ValueError:
                        continue
                    target = vals
                    seq += 1
                    ms = int((time.time() - t0) * 1000)
                    os.write(master, f'ACK,{seq},{ms}\n'.encode())

        now = time.time()
        if now - last_tick >= 0.02:  # 20 ms servo update, like loop()
            last_tick = now
            for i in range(NUM):
                d = max(-MAX_STEP, min(MAX_STEP, target[i] - current[i]))
                current[i] += d

        if now - last_log >= 1.0:  # once a second, report servo state
            last_log = now
            joints = '  '.join(f'{v:6.1f}' for v in current[:7])
            print(f'[arduino-sim] servos(deg): {joints}  grip: {current[7]:5.1f}'
                  f'   (cmds received: {seq})', flush=True)


if __name__ == '__main__':
    main()
