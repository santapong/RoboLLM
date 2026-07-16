#!/usr/bin/env python3
"""Host-side driver for the DIY arm's Arduino Uno (see firmware/arm_firmware).

Works on the laptop AND the Raspberry Pi 5 — anything with pyserial.

    from arm_serial import ArmSerial, find_arduino_port
    arm = ArmSerial()          # auto-detects the port
    arm.ping()                 # -> "PONG arm-fw 1.0"
    arm.enable(True)           # attach servos (torque on)
    arm.set_joint(0, 120)      # servo 0 -> 120 deg (fw slew-limits the move)
    arm.get_joints()           # -> [120, 90, 90, 90, 90, 90]
    arm.enable(False)          # torque off

CLI:  python3 arm_serial.py ping | joints | set <ch> <deg> | led <0|1>
"""
from __future__ import annotations

import os
import sys
import time

import serial
from serial.tools import list_ports

# USB VID:PID pairs seen on Unos: genuine (2341/2A03) and CH340/CP210x clones.
ARDUINO_IDS = {(0x2341, None), (0x2A03, None), (0x1A86, None), (0x10C4, None)}


def find_arduino_port() -> str | None:
    """Return the device path of the first Arduino-looking serial port."""
    for p in list_ports.comports():
        if p.vid is None:
            continue
        if any(p.vid == vid for vid, _ in ARDUINO_IDS):
            return p.device
    # fallback: any ttyACM/ttyUSB
    for p in list_ports.comports():
        if "ttyACM" in p.device or "ttyUSB" in p.device:
            return p.device
    return None


class ArmSerial:
    def __init__(self, port: str | None = None, baud: int = 115200,
                 boot_wait_s: float = 2.5):
        # ARM_PORT env overrides auto-detect (e.g. a sim_uno.py /dev/pts/N)
        port = port or os.environ.get("ARM_PORT") or find_arduino_port()
        if port is None:
            raise RuntimeError(
                "No Arduino found. Plug the Uno in via USB and check "
                "`ls /dev/ttyACM*` (you must be in the 'dialout' group).")
        self.ser = serial.Serial(port, baud, timeout=1.0)
        # Opening the port toggles DTR -> the Uno RESETS. Wait for reboot.
        time.sleep(boot_wait_s)
        self.ser.reset_input_buffer()
        self.port = port

    def _cmd(self, text: str) -> str:
        self.ser.write((text + "\n").encode())
        reply = self.ser.readline().decode(errors="replace").strip()
        # skip the boot banner if it raced with us
        while reply.startswith("READY"):
            reply = self.ser.readline().decode(errors="replace").strip()
        return reply

    def ping(self) -> str:
        reply = self._cmd("PING")
        if not reply.startswith("PONG"):
            raise RuntimeError(f"bad PING reply: {reply!r}")
        return reply

    def enable(self, on: bool) -> None:
        self._cmd(f"E {1 if on else 0}")

    def led(self, on: bool) -> None:
        self._cmd(f"LED {1 if on else 0}")

    def set_joint(self, ch: int, deg: float) -> None:
        reply = self._cmd(f"S {int(ch)} {int(round(deg))}")
        if reply != "OK":
            raise RuntimeError(f"set_joint failed: {reply!r}")

    def get_joints(self) -> list[int]:
        reply = self._cmd("G")
        if not reply.startswith("A "):
            raise RuntimeError(f"bad G reply: {reply!r}")
        return [int(v) for v in reply.split()[1:]]

    def close(self) -> None:
        self.ser.close()


def main() -> int:
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return 2
    arm = ArmSerial()
    print(f"# connected on {arm.port}")
    if args[0] == "ping":
        print(arm.ping())
    elif args[0] == "joints":
        print(arm.get_joints())
    elif args[0] == "set" and len(args) == 3:
        arm.enable(True)
        arm.set_joint(int(args[1]), float(args[2]))
        time.sleep(1.5)
        print(arm.get_joints())
    elif args[0] == "led" and len(args) == 2:
        arm.led(args[1] == "1")
        print("OK")
    else:
        print(__doc__)
        return 2
    arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
