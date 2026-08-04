#!/usr/bin/env python3
"""sim_uno.py — fake Arduino: emulates arm_firmware.ino (arm-fw 2.0) on a pty.

Lets you develop/test the WHOLE arm stack (arm_serial.py, arm_bridge_node.py,
camera_logger.py, acceptance_test.py) with no hardware plugged in. Speaks the
identical v2 protocol — S/Q/H/E/X/P/L with measured-state `s` replies and
millis timestamps — including the 100 deg/s slew limit, so motion takes real
time and measured != commanded is visible mid-move, exactly like the bench.

    python3 sim_uno.py            # prints the port, e.g. /dev/pts/3
    ARM_PORT=/dev/pts/3 python3 arm_serial.py ping
"""
import os
import pty
import threading
import time
import tty

N = 6
HOME = [90.0] * N
current = HOME[:]
target = HOME[:]
g_current = 0.0
g_target = 0.0
enabled = False
MAX_STEP = 2.0   # deg per 20 ms tick == 100 deg/s, same as firmware
T0 = time.monotonic()


def millis() -> int:
    return int((time.monotonic() - T0) * 1000)


def slew_loop():
    global g_current
    while True:
        for i in range(N):
            d = max(-MAX_STEP, min(MAX_STEP, target[i] - current[i]))
            current[i] += d
        g_current += max(-2 * MAX_STEP, min(2 * MAX_STEP, g_target - g_current))
        time.sleep(0.02)


def state_line() -> str:
    joints = " ".join(f"{v:.2f}" for v in current)
    return f"s {joints} {g_current:.1f} {millis()}"


def handle(line: str) -> str:
    global enabled, g_target
    if not line:
        return ""
    c, _, args = line.partition(" ")
    if c == "S":
        try:
            vals = [float(v) for v in args.split()]
            if len(vals) != N + 1:
                raise ValueError
            for i in range(N):
                target[i] = max(0.0, min(180.0, vals[i]))
            g_target = max(0.0, min(100.0, vals[N]))
            enabled = True
            return state_line()
        except ValueError:
            return "! bad_cmd\n" + state_line()
    if c == "Q":
        return state_line()
    if c == "H":
        target[:] = HOME
        g_target = 0.0
        enabled = True
        return state_line()
    if c == "E":
        enabled = True
        return state_line()
    if c == "X":
        enabled = False
        return state_line()
    if c == "P":
        return "# pong arm-fw 2.0 (SIM)"
    if c == "L":
        return "# led"
    return "! bad_cmd"


def main():
    master, slave = pty.openpty()
    tty.setraw(slave)  # no echo/line-editing — else the sim reads its own replies
    port = os.ttyname(slave)
    print(f"SIM UNO on {port}   (Ctrl-C to stop)", flush=True)
    threading.Thread(target=slew_loop, daemon=True).start()
    os.write(master, b"# ready arm-fw 2.0 (SIM)\n")
    buf = b""
    while True:
        buf += os.read(master, 64)
        while b"\n" in buf or b"\r" in buf:
            buf = buf.replace(b"\r", b"\n")
            line, _, buf = buf.partition(b"\n")
            reply = handle(line.decode(errors="replace").strip())
            if reply:
                os.write(master, reply.encode() + b"\n")
                print(f"  {line.decode()!r:32} -> {reply.splitlines()[-1]}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
