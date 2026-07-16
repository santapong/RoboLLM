#!/usr/bin/env python3
"""sim_uno.py — fake Arduino: emulates arm_firmware.ino on a virtual serial port.

Lets you develop/test the WHOLE arm stack (arm_serial.py, arm_bridge_node.py,
future MCP tools) with no hardware plugged in. It speaks the identical
protocol, including the 100 deg/s slew-rate limit, so motion takes real time.

    python3 sim_uno.py            # prints the port, e.g. /dev/pts/3
    ARM_PORT=/dev/pts/3 python3 arm_serial.py ping
"""
import os
import pty
import threading
import time
import tty

N = 6
current = [90.0] * N
target = [90.0] * N
enabled = False
MAX_STEP = 2.0   # deg per 20 ms tick == 100 deg/s, same as firmware


def slew_loop():
    while True:
        for i in range(N):
            d = max(-MAX_STEP, min(MAX_STEP, target[i] - current[i]))
            current[i] += d
        time.sleep(0.02)


def handle(line: str) -> str:
    global enabled
    if line == "PING":
        return "PONG arm-fw 1.0 (SIM)"
    if line == "G":
        return "A " + " ".join(str(int(v)) for v in current)
    if line.startswith("E "):
        enabled = line[2] == "1"
        return "OK"
    if line.startswith("LED "):
        return "OK"
    if line.startswith("S "):
        try:
            ch, deg = (int(v) for v in line[2:].split())
            if not 0 <= ch < N:
                raise ValueError
            target[ch] = max(0, min(180, deg))
            return "OK"
        except ValueError:
            return "ERR S <ch 0-5> <deg 0-180>"
    return f"ERR unknown: {line}" if line else ""


def main():
    master, slave = pty.openpty()
    tty.setraw(slave)  # no echo/line-editing — else the sim reads its own replies
    port = os.ttyname(slave)
    print(f"SIM UNO on {port}   (Ctrl-C to stop)", flush=True)
    threading.Thread(target=slew_loop, daemon=True).start()
    os.write(master, b"READY arm-fw 1.0 (SIM)\n")
    buf = b""
    while True:
        buf += os.read(master, 64)
        while b"\n" in buf or b"\r" in buf:
            buf = buf.replace(b"\r", b"\n")
            line, _, buf = buf.partition(b"\n")
            reply = handle(line.decode(errors="replace").strip())
            if reply:
                os.write(master, reply.encode() + b"\n")
                print(f"  {line.decode()!r:24} -> {reply}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
