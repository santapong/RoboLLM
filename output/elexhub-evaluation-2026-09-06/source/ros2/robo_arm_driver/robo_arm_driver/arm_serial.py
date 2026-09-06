#!/usr/bin/env python3
"""Host-side API for arm-fw 2.1.

The public API uses logical radians and a normalized gripper value. Only this
module converts those actions to raw servo degrees/percent on the serial wire.
"""
from __future__ import annotations

import os
import math
import sys
import time
from dataclasses import dataclass
from typing import Sequence

import serial
from serial.tools import list_ports

from .config import ArmConfig, ConfigError, load_arm_config

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
    state_source: str = "commanded"
    enabled: bool | None = None
    fault: str = "none"


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
        self._faulted = False
        # A simulated profile may only talk to a PTY. Never probe USB with it.
        explicit_port = port or os.environ.get("ARM_PORT")
        if self.config.schema_version == 2:
            is_pty = bool(explicit_port and os.path.realpath(explicit_port).startswith("/dev/pts/"))
            if self.config.simulation and not is_pty:
                raise ConfigError("simulation profile requires an explicit /dev/pts/ port")
            if not is_pty and not (self.config.calibrated or self.config.commissioning_allowed):
                raise ConfigError("physical Elexhub connection blocked: hardware evidence is incomplete")
        port = port or os.environ.get("ARM_PORT") or find_arduino_port()
        if port is None:
            raise RuntimeError(
                "No Arduino found. Plug the Mega in via USB and check "
                "`ls /dev/ttyACM*` (you must be in the dialout group).")
        self.ser = serial.Serial(port, baud, timeout=1.0)
        time.sleep(boot_wait_s)  # opening the USB serial port resets the board
        self.ser.reset_input_buffer()
        self.port = port
        if self.config.schema_version == 2:
            try:
                banner = self.ping()
                expected = {"joints": str(self.config.njoints), "backend": "pca9685",
                            "profile": self.config.profile_hash,
                            "runtime": "simulation" if self.config.simulation else "hardware"}
                fields = dict(part.split("=", 1) for part in banner.split() if "=" in part)
                if not banner.startswith("pong arm-fw 3.0 ") or any(fields.get(k) != v for k, v in expected.items()):
                    raise FirmwareError("firmware capability/profile mismatch; motion refused")
            except Exception:
                self.ser.close()
                raise

    def _cmd_state(self, line: str) -> ArmState:
        if self._faulted and line.split()[0] in {"S", "H", "C", "K"}:
            raise FirmwareError("fault latched; explicit enable is required after inspection")
        try:
            self.ser.write((line + "\n").encode())
        except Exception:
            self._faulted = self.config.schema_version == 2
            raise
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
                try:
                    state = self._parse_state(raw)
                except (ValueError, IndexError):
                    self._faulted = self.config.schema_version == 2
                    raise
                if error:
                    if self.config.schema_version == 2:
                        self._faulted = True
                    raise FirmwareError(f"firmware rejected {line!r}: {error}")
                if state.fault != "none":
                    self._faulted = True
                return state
        if error:
            self._faulted = self.config.schema_version == 2
            raise FirmwareError(f"firmware rejected {line!r}: {error}")
        self._faulted = self.config.schema_version == 2
        raise TimeoutError(f"no state reply to {line!r}")

    def _parse_state(self, raw: str) -> ArmState:
        parts = raw.split()
        if len(parts) != self.config.njoints + (5 if self.config.schema_version == 2 else 3):
            raise ValueError(f"malformed state: {raw!r}")
        raw_deg = [float(value) for value in parts[1:1 + self.config.njoints]]
        gripper_wire = float(parts[1 + self.config.njoints])
        if not all(math.isfinite(v) for v in raw_deg + [gripper_wire]):
            raise ValueError("non-finite controller state")
        for joint, value in zip(self.config.joints, raw_deg):
            joint.require_raw(value)
        if not 0 <= gripper_wire <= 100:
            raise ValueError("invalid gripper controller state")
        if self.config.schema_version == 2 and parts[-2] not in {"0", "1"}:
            raise ValueError("invalid enabled controller state")
        return ArmState(
            q=[joint.raw_to_rad(value) for joint, value in zip(self.config.joints, raw_deg)],
            gripper=self.config.gripper.wire_to_logical(gripper_wire),
            t_arduino_ms=int(parts[2 + self.config.njoints]),
            t_host=time.time(),
            state_source=self.config.state_source,
            enabled=parts[-2] == "1" if self.config.schema_version == 2 else None,
            fault=parts[-1] if self.config.schema_version == 2 else "none",
        )

    def get_state(self) -> ArmState:
        return self._cmd_state("Q")

    def set_action(self, q: Sequence[float], gripper: float = 0.0) -> ArmState:
        self.config.require_calibrated()
        if len(q) != self.config.njoints:
            raise ConfigError(f"expected {self.config.njoints} joint targets, got {len(q)}")
        raw = []
        for joint, radians in zip(self.config.joints, q):
            joint.require_rad(radians)
            raw.append(f"{joint.rad_to_raw(radians):.2f}")
        grip = self.config.gripper.logical_to_wire(gripper)
        return self._cmd_state("S " + " ".join(raw) + f" {grip:.1f}")

    def commission_joint(self, channel: int, raw_deg: float) -> ArmState:
        """Move one channel inside its narrow raw commissioning window."""
        self.config.require_commissioning()
        if not 0 <= channel < self.config.njoints:
            raise ConfigError(f"joint channel must be in [0, {self.config.njoints - 1}]")
        raw_deg = self.config.joints[channel].require_raw(raw_deg)
        return self._cmd_state(f"C {channel} {raw_deg:.2f}")

    def commission_gripper(self, value: float) -> ArmState:
        self.config.require_commissioning()
        if self.config.schema_version != 2:
            raise ConfigError("gripper commissioning requires protocol 3.0")
        wire = self.config.gripper.logical_to_wire(value)
        return self._cmd_state(f"C {self.config.njoints} {wire:.2f}")

    def hold(self) -> ArmState:
        """Explicit healthy-control heartbeat; queries never prolong motion."""
        return self._cmd_state("K" if self.config.schema_version == 2 else "Q")

    def home(self) -> ArmState:
        self.config.require_calibrated()
        return self._cmd_state("H")

    def enable(self) -> ArmState:
        self.config.require_calibrated()
        state = self._cmd_state("E")
        self._faulted = state.fault != "none"
        return state

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
            print("PWM disabled; servo power is controlled by the physical cutoff")
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
