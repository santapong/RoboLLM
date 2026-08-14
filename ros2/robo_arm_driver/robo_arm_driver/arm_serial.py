#!/usr/bin/env python3
"""Host-side API for arm-fw 2.1.

The public API uses logical radians and a normalized gripper value. Only this
module converts those actions to raw servo degrees/percent on the serial wire.
"""
from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from typing import Sequence

import serial
from serial.tools import list_ports

from .config import ArmConfig, ConfigError, NJOINTS, load_arm_config

ARDUINO_IDS = {0x2341, 0x2A03, 0x1A86, 0x10C4}


class FirmwareError(RuntimeError):
    """Firmware rejected a command."""


def find_arduino_port() -> str | None:
    for port in list_ports.comports():
        if port.vid in ARDUINO_IDS:
            return port.device
    for port in list_ports.comports():
        if "ttyACM" in port.device or "ttyUSB" in port.device:
            return port.device
    return None


@dataclass(frozen=True)
class ArmState:
    q: list[float]
    gripper: float
    t_arduino_ms: int
    t_host: float


class ArmSerial:
    def __init__(
        self,
        port: str | None = None,
        baud: int = 115200,
        boot_wait_s: float = 2.5,
        config: ArmConfig | None = None,
        config_path: str | os.PathLike[str] | None = None,
    ):
        self.config = config or load_arm_config(config_path)
        port = port or os.environ.get("ARM_PORT") or find_arduino_port()
        if port is None:
            raise RuntimeError(
                "No Arduino found. Plug the Mega in via USB and check "
                "`ls /dev/ttyACM*` (you must be in the dialout group).")
        self.ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(boot_wait_s)  # opening the USB serial port resets the board
        self.ser.reset_input_buffer()
        self.port = port

    def _cmd_state(self, line: str) -> ArmState:
        self.ser.write((line + "\n").encode())
        deadline = time.time() + 2.0
        error: str | None = None
        while time.time() < deadline:
            raw = self.ser.readline().decode(errors="replace").strip()
            if not raw or raw.startswith("#"):
                continue
            if raw.startswith("!"):
                error = raw
                continue
            if raw.startswith("s "):
                state = self._parse_state(raw)
                if error:
                    raise FirmwareError(f"firmware rejected {line!r}: {error}")
                return state
        if error:
            raise FirmwareError(f"firmware rejected {line!r}: {error}")
        raise TimeoutError(f"no state reply to {line!r}")

    def _parse_state(self, raw: str) -> ArmState:
        parts = raw.split()
        if len(parts) != NJOINTS + 3:
            raise ValueError(f"malformed state: {raw!r}")
        raw_deg = [float(value) for value in parts[1:1 + NJOINTS]]
        gripper_wire = float(parts[1 + NJOINTS])
        return ArmState(
            q=[joint.raw_to_rad(value) for joint, value in zip(self.config.joints, raw_deg)],
            gripper=self.config.gripper.wire_to_logical(gripper_wire),
            t_arduino_ms=int(parts[2 + NJOINTS]),
            t_host=time.time(),
        )

    def get_state(self) -> ArmState:
        return self._cmd_state("Q")

    def set_action(self, q: Sequence[float], gripper: float = 0.0) -> ArmState:
        self.config.require_calibrated()
        if len(q) != NJOINTS:
            raise ConfigError(f"expected {NJOINTS} joint targets, got {len(q)}")
        raw = []
        for joint, radians in zip(self.config.joints, q):
            joint.require_rad(radians)
            raw.append(f"{joint.rad_to_raw(radians):.2f}")
        grip = self.config.gripper.logical_to_wire(gripper)
        return self._cmd_state("S " + " ".join(raw) + f" {grip:.1f}")

    def commission_joint(self, channel: int, raw_deg: float) -> ArmState:
        """Move one channel inside its narrow raw commissioning window."""
        if not 0 <= channel < NJOINTS:
            raise ConfigError(f"joint channel must be in [0, {NJOINTS - 1}]")
        raw_deg = self.config.joints[channel].require_raw(raw_deg)
        return self._cmd_state(f"C {channel} {raw_deg:.2f}")

    def home(self) -> ArmState:
        self.config.require_calibrated()
        return self._cmd_state("H")

    def enable(self) -> ArmState:
        self.config.require_calibrated()
        return self._cmd_state("E")

    def relax(self) -> ArmState:
        return self._cmd_state("X")

    def ping(self) -> str:
        self.ser.write(b"P\n")
        deadline = time.time() + 2.0
        while time.time() < deadline:
            raw = self.ser.readline().decode(errors="replace").strip()
            if "pong" in raw:
                return raw.lstrip("# ")
        raise RuntimeError("no pong reply")

    def led(self, on: bool) -> None:
        self.ser.write(f"L {1 if on else 0}\n".encode())
        self.ser.readline()

    def close(self) -> None:
        try:
            self.relax()
        except Exception:
            pass
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()


def main() -> int:
    args = sys.argv[1:]
    if not args:
        print(
            "usage: arm_serial.py ping | state | commission <ch> <raw-deg> | "
            "home | relax | led <0|1> | config")
        return 2
    arm = ArmSerial()
    print(f"# connected on {arm.port}; config={arm.config.source}")
    try:
        if args[0] == "ping":
            print(arm.ping())
        elif args[0] == "state":
            state = arm.get_state()
            print("q (rad):", [round(v, 3) for v in state.q],
                  "gripper:", round(state.gripper, 2), "t_ms:", state.t_arduino_ms)
        elif args[0] == "commission" and len(args) == 3:
            state = arm.commission_joint(int(args[1]), float(args[2]))
            print("q (rad):", [round(v, 3) for v in state.q])
        elif args[0] == "home":
            print("homed:", [round(v, 3) for v in arm.home().q])
        elif args[0] == "relax":
            arm.relax()
            print("relaxed (torque off)")
        elif args[0] == "led" and len(args) == 2:
            arm.led(args[1] == "1")
            print("OK")
        elif args[0] == "config":
            print(f"calibrated={arm.config.calibrated} state_source={arm.config.state_source}")
            for joint in arm.config.joints:
                print(f"{joint.channel}: {joint.name} raw=[{joint.min_deg}, {joint.max_deg}] "
                      f"home={joint.home_deg} pin={joint.servo_pin}")
        else:
            return 2
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
