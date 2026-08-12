#!/usr/bin/env python3
"""Fake Arduino for arm-fw 2.1, including limits and the watchdog.

For normal simulated motion, select the explicit simulation profile:

    ARM_CONFIG=ros2/robo_arm_driver/config/joints.sim.yaml python3 hardware/sim_uno.py
"""
from __future__ import annotations

import os
from pathlib import Path
import pty
import sys
import threading
import time
import tty

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2" / "robo_arm_driver"))

from robo_arm_driver.config import ConfigError, NJOINTS, load_arm_config  # noqa: E402

CONFIG = load_arm_config()
TICK_S = 1.0 / CONFIG.control_rate_hz
current = [joint.home_deg for joint in CONFIG.joints]
target = current[:]
g_current = CONFIG.gripper.open_percent
g_target = g_current
enabled = False
last_command = time.monotonic()
T0 = time.monotonic()


def millis() -> int:
    return int((time.monotonic() - T0) * 1000)


def watchdog_tick(now: float | None = None) -> bool:
    """Apply the communication watchdog; return true when it trips."""
    global enabled
    now = time.monotonic() if now is None else now
    if enabled and (now - last_command) * 1000.0 > CONFIG.command_timeout_ms:
        enabled = False
        return True
    return False


def slew_loop() -> None:
    global enabled, g_current
    while True:
        watchdog_tick()
        for index, joint in enumerate(CONFIG.joints):
            max_step = joint.max_velocity_deg_s * TICK_S
            delta = max(-max_step, min(max_step, target[index] - current[index]))
            current[index] += delta
        grip_step = CONFIG.gripper.max_velocity_percent_s * TICK_S
        g_current += max(-grip_step, min(grip_step, g_target - g_current))
        time.sleep(TICK_S)


def state_line() -> str:
    joints = " ".join(f"{value:.2f}" for value in current)
    return f"s {joints} {g_current:.1f} {millis()}"


def _error(reason: str) -> str:
    return f"! {reason}\n{state_line()}"


def handle(line: str) -> str:
    global enabled, g_target, last_command
    if not line:
        return ""
    command, _, args = line.partition(" ")
    if command == "S":
        if not CONFIG.calibrated:
            return _error("not_calibrated")
        try:
            values = [float(value) for value in args.split()]
            if len(values) != NJOINTS + 1:
                raise ValueError
            for joint, value in zip(CONFIG.joints, values[:NJOINTS]):
                joint.require_raw(value)
            grip_min = min(CONFIG.gripper.open_percent, CONFIG.gripper.closed_percent)
            grip_max = max(CONFIG.gripper.open_percent, CONFIG.gripper.closed_percent)
            if not grip_min <= values[-1] <= grip_max:
                raise ConfigError("gripper outside configured range")
        except (ValueError, ConfigError):
            return _error("bad_or_unsafe_cmd")
        target[:] = values[:NJOINTS]
        g_target = values[-1]
        enabled = True
        last_command = time.monotonic()
        return state_line()
    if command == "C":
        if CONFIG.calibrated:
            return _error("commissioning_disabled")
        try:
            fields = args.split()
            if len(fields) != 2:
                raise ValueError
            channel = int(fields[0])
            if not 0 <= channel < NJOINTS:
                raise ValueError
            target[channel] = CONFIG.joints[channel].require_raw(float(fields[1]))
        except (ValueError, ConfigError):
            return _error("bad_or_unsafe_cmd")
        enabled = True
        last_command = time.monotonic()
        return state_line()
    if command == "Q" and not args:
        last_command = time.monotonic()
        return state_line()
    if command == "H" and not args:
        if not CONFIG.calibrated:
            return _error("not_calibrated")
        target[:] = [joint.home_deg for joint in CONFIG.joints]
        g_target = CONFIG.gripper.open_percent
        enabled = True
        last_command = time.monotonic()
        return state_line()
    if command == "E" and not args:
        if not CONFIG.calibrated:
            return _error("not_calibrated")
        enabled = True
        last_command = time.monotonic()
        return state_line()
    if command == "X" and not args:
        enabled = False
        return state_line()
    if command == "P" and not args:
        return "# pong arm-fw 2.1 (SIM)"
    if command == "L" and args in {"0", "1"}:
        return "# led"
    return "! bad_cmd"


def main() -> None:
    master, slave = pty.openpty()
    tty.setraw(slave)
    port = os.ttyname(slave)
    print(f"SIM UNO on {port}   (Ctrl-C to stop)", flush=True)
    print(f"config={CONFIG.source} calibrated={CONFIG.calibrated}", flush=True)
    threading.Thread(target=slew_loop, daemon=True).start()
    os.write(master, b"# ready arm-fw 2.1 (SIM)\n")
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
