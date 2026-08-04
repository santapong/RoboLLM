#!/usr/bin/env python3
"""Host-side driver for the DIY arm's Uno (arm-fw 2.0, see firmware/arm_firmware).

Exposes the robot API every later phase uses — mirroring the VLA I/O contract
(state in, position targets out):

    from arm_serial import ArmSerial
    arm = ArmSerial()               # auto-detects the port (or ARM_PORT env)
    st = arm.get_state()            # MEASURED joint angles (radians) + gripper
    arm.set_action(st.q, 0.5)      # command targets (radians) + gripper 0..1
    arm.home(); arm.relax()

Angles are RADIANS in Python (ML convention); the wire is DEGREES. All unit
conversion + per-joint calibration lives HERE, in one place (JointCalib).
The `s` reply is always the MEASURED value — with the encoder stub in the
firmware it equals the slewed commanded position (the H5 baseline); with real
encoders it is the truth. Either way, mid-motion state != target.

CLI:  python3 arm_serial.py ping | state | set <ch> <deg> | home | relax | led <0|1>
"""
from __future__ import annotations

import math
import os
import sys
import time
from dataclasses import dataclass, field

import serial
from serial.tools import list_ports

NJOINTS = 6

# USB VID:PID pairs seen on Unos: genuine (2341/2A03) and CH340/CP210x clones.
ARDUINO_IDS = {0x2341, 0x2A03, 0x1A86, 0x10C4}


def find_arduino_port() -> str | None:
    """Return the device path of the first Arduino-looking serial port."""
    for p in list_ports.comports():
        if p.vid in ARDUINO_IDS:
            return p.device
    for p in list_ports.comports():
        if "ttyACM" in p.device or "ttyUSB" in p.device:
            return p.device
    return None


@dataclass
class JointCalib:
    """Per-joint calibration: wire is degrees, Python is radians.

    offset_deg shifts the raw firmware angle so the logical zero lines up;
    sign flips a joint whose angle increases the 'wrong' way. Fill in during
    the bench calibration step.
    """
    offset_deg: float = 90.0        # firmware home 90 deg == logical 0 rad
    sign: float = 1.0
    min_rad: float = -math.pi / 2
    max_rad: float = math.pi / 2


@dataclass
class ArmState:
    q: list[float]          # 6 joint angles, radians (measured)
    gripper: float          # 0..1 (0 = open, 1 = closed)
    t_arduino_ms: int       # firmware millis() timestamp
    t_host: float           # host time.time() when the line was received


class ArmSerial:
    def __init__(self, port: str | None = None, baud: int = 115200,
                 boot_wait_s: float = 2.5,
                 calib: list[JointCalib] | None = None):
        # ARM_PORT env overrides auto-detect (e.g. a sim_uno.py /dev/pts/N)
        port = port or os.environ.get("ARM_PORT") or find_arduino_port()
        if port is None:
            raise RuntimeError(
                "No Arduino found. Plug the Uno in via USB and check "
                "`ls /dev/ttyACM*` (you must be in the 'dialout' group).")
        self.calib = calib or [JointCalib() for _ in range(NJOINTS)]
        self.ser = serial.Serial(port, baud, timeout=1.0)
        # Opening the port toggles DTR -> a real Uno RESETS. Wait for reboot.
        time.sleep(boot_wait_s)
        self.ser.reset_input_buffer()
        self.port = port

    # ---- low-level line I/O ----
    def _cmd_state(self, line: str) -> ArmState:
        """Send one command, return the single `s` state reply.
        Skips `#` comments, logs `!` errors."""
        self.ser.write((line + "\n").encode())
        deadline = time.time() + 2.0
        while time.time() < deadline:
            raw = self.ser.readline().decode(errors="replace").strip()
            if not raw or raw.startswith("#"):
                continue
            if raw.startswith("!"):
                print(f"[arm] firmware error: {raw}", file=sys.stderr)
                continue
            if raw.startswith("s "):
                return self._parse_state(raw)
        raise TimeoutError(f"no state reply to {line!r}")

    def _parse_state(self, raw: str) -> ArmState:
        parts = raw.split()
        if len(parts) != NJOINTS + 3:
            raise ValueError(f"malformed state: {raw!r}")
        deg = [float(x) for x in parts[1:1 + NJOINTS]]
        g_wire = float(parts[1 + NJOINTS])
        t_ms = int(parts[2 + NJOINTS])
        q = [self._deg_to_rad(i, deg[i]) for i in range(NJOINTS)]
        return ArmState(q=q, gripper=g_wire / 100.0,
                        t_arduino_ms=t_ms, t_host=time.time())

    # ---- unit conversion + calibration (the ONE place) ----
    def _deg_to_rad(self, i: int, deg: float) -> float:
        c = self.calib[i]
        return c.sign * math.radians(deg - c.offset_deg)

    def _rad_to_deg(self, i: int, rad: float) -> float:
        c = self.calib[i]
        return math.degrees(c.sign * rad) + c.offset_deg

    # ---- public API (the contract everything else uses) ----
    def get_state(self) -> ArmState:
        """Read MEASURED joint angles (radians) without moving."""
        return self._cmd_state("Q")

    def set_action(self, q, gripper: float = 0.0) -> ArmState:
        """Command target joint angles (radians) + gripper (0..1).
        Returns the measured state immediately after (still mid-motion)."""
        if len(q) != NJOINTS:
            raise ValueError(f"expected {NJOINTS} joint targets, got {len(q)}")
        qd = []
        for i in range(NJOINTS):
            c = self.calib[i]
            qi = max(c.min_rad, min(c.max_rad, q[i]))   # soft clamp, friendly
            qd.append(f"{self._rad_to_deg(i, qi):.2f}")
        gw = f"{max(0.0, min(1.0, gripper)) * 100.0:.1f}"
        return self._cmd_state("S " + " ".join(qd) + " " + gw)

    def home(self) -> ArmState:
        return self._cmd_state("H")

    def enable(self) -> ArmState:
        return self._cmd_state("E")

    def relax(self) -> ArmState:
        """E-stop: de-energize servos (safe; hand-movable if backdrivable)."""
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
        self.ser.readline()   # consume the "# led" ack

    def close(self) -> None:
        try:
            self.relax()
        except Exception:
            pass
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def main() -> int:
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return 2
    arm = ArmSerial()
    print(f"# connected on {arm.port}")
    if args[0] == "ping":
        print(arm.ping())
    elif args[0] == "state":
        st = arm.get_state()
        print("q (rad):", [round(v, 3) for v in st.q],
              "gripper:", round(st.gripper, 2), "t_ms:", st.t_arduino_ms)
    elif args[0] == "set" and len(args) == 3:
        ch, deg = int(args[1]), float(args[2])
        st = arm.get_state()
        q = list(st.q)
        q[ch] = arm._deg_to_rad(ch, deg)
        arm.set_action(q, st.gripper)
        time.sleep(1.5)
        print("q (rad):", [round(v, 3) for v in arm.get_state().q])
    elif args[0] == "home":
        print("homed:", [round(v, 3) for v in arm.home().q])
    elif args[0] == "relax":
        arm.relax()
        print("relaxed (torque off)")
    elif args[0] == "led" and len(args) == 2:
        arm.led(args[1] == "1")
        print("OK")
    else:
        print(__doc__)
        arm.close()
        return 2
    arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
