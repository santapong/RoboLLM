"""Load and validate the one canonical arm configuration.

Raw hobby-servo values are degrees. Public robot joint values are radians.
Keeping that conversion here prevents ROS, learning, or planning code from
ever handling PWM-facing coordinates directly.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

NJOINTS = 6


class ConfigError(ValueError):
    """The arm configuration is missing, unsafe, or internally inconsistent."""


@dataclass(frozen=True)
class JointConfig:
    name: str
    channel: int
    servo_pin: int
    offset_deg: float
    sign: float
    min_deg: float
    max_deg: float
    home_deg: float
    max_velocity_deg_s: float

    @property
    def min_rad(self) -> float:
        return min(self.raw_to_rad(self.min_deg), self.raw_to_rad(self.max_deg))

    @property
    def max_rad(self) -> float:
        return max(self.raw_to_rad(self.min_deg), self.raw_to_rad(self.max_deg))

    @property
    def max_velocity_rad_s(self) -> float:
        return math.radians(self.max_velocity_deg_s)

    def raw_to_rad(self, raw_deg: float) -> float:
        return self.sign * math.radians(raw_deg - self.offset_deg)

    def rad_to_raw(self, radians: float) -> float:
        return math.degrees(self.sign * radians) + self.offset_deg

    def require_raw(self, raw_deg: float) -> float:
        raw_deg = float(raw_deg)
        if not math.isfinite(raw_deg):
            raise ConfigError(f"{self.name}: non-finite raw angle")
        if not self.min_deg <= raw_deg <= self.max_deg:
            raise ConfigError(
                f"{self.name}: {raw_deg:.2f} deg outside configured "
                f"[{self.min_deg:.2f}, {self.max_deg:.2f}]")
        return raw_deg

    def require_rad(self, radians: float) -> float:
        radians = float(radians)
        if not math.isfinite(radians):
            raise ConfigError(f"{self.name}: non-finite joint angle")
        if not self.min_rad <= radians <= self.max_rad:
            raise ConfigError(
                f"{self.name}: {radians:.4f} rad outside configured "
                f"[{self.min_rad:.4f}, {self.max_rad:.4f}]")
        return radians


@dataclass(frozen=True)
class GripperConfig:
    servo_pin: int
    open_percent: float
    closed_percent: float
    max_velocity_percent_s: float

    def logical_to_wire(self, value: float) -> float:
        value = float(value)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ConfigError(f"gripper: expected a finite value in [0, 1], got {value!r}")
        return self.open_percent + value * (self.closed_percent - self.open_percent)

    def wire_to_logical(self, percent: float) -> float:
        span = self.closed_percent - self.open_percent
        return max(0.0, min(1.0, (float(percent) - self.open_percent) / span))


@dataclass(frozen=True)
class ArmConfig:
    source: Path
    calibrated: bool
    state_source: str
    command_timeout_ms: int
    control_rate_hz: float
    joints: tuple[JointConfig, ...]
    gripper: GripperConfig

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(j.name for j in self.joints)

    def require_calibrated(self) -> None:
        if not self.calibrated:
            raise ConfigError(
                "arm is not calibrated; complete docs/physical-arm/"
                "HARDWARE_WORKSHEET.md, update joints.yaml, regenerate the "
                "firmware config, and set calibrated: true")


def default_config_path() -> Path:
    override = os.environ.get("ARM_CONFIG")
    if override:
        return Path(override).expanduser().resolve()

    source_tree = Path(__file__).resolve().parents[1] / "config" / "joints.yaml"
    if source_tree.is_file():
        return source_tree

    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("robo_arm_driver")) / "config" / "joints.yaml"
    except (ImportError, LookupError) as exc:
        raise ConfigError("cannot locate joints.yaml; set ARM_CONFIG explicitly") from exc


def _number(item: dict[str, Any], key: str, context: str) -> float:
    value = item.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{context}.{key} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ConfigError(f"{context}.{key} must be finite")
    return value


def load_arm_config(path: str | os.PathLike[str] | None = None) -> ArmConfig:
    source = Path(path).expanduser().resolve() if path else default_config_path()
    try:
        raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ConfigError(f"cannot read arm config {source}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ConfigError(f"invalid YAML in {source}: {exc}") from exc

    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise ConfigError(f"{source}: schema_version must be 1")
    if not isinstance(raw.get("calibrated"), bool):
        raise ConfigError(f"{source}: calibrated must be true or false")
    state_source = raw.get("state_source")
    if state_source not in {"commanded", "measured"}:
        raise ConfigError(f"{source}: state_source must be commanded or measured")

    timeout = raw.get("command_timeout_ms")
    if isinstance(timeout, bool) or not isinstance(timeout, int) or not 100 <= timeout <= 5000:
        raise ConfigError(f"{source}: command_timeout_ms must be an integer in [100, 5000]")
    rate = _number(raw, "control_rate_hz", str(source))
    if not 1.0 <= rate <= 100.0:
        raise ConfigError(f"{source}: control_rate_hz must be in [1, 100]")

    items = raw.get("joints")
    if not isinstance(items, list) or len(items) != NJOINTS:
        raise ConfigError(f"{source}: exactly {NJOINTS} joints are required")
    joints: list[JointConfig] = []
    for index, item in enumerate(items):
        context = f"joints[{index}]"
        if not isinstance(item, dict):
            raise ConfigError(f"{context} must be a mapping")
        name = item.get("name")
        channel = item.get("channel")
        servo_pin = item.get("servo_pin")
        if not isinstance(name, str) or not name:
            raise ConfigError(f"{context}.name must be a non-empty string")
        if isinstance(channel, bool) or not isinstance(channel, int):
            raise ConfigError(f"{context}.channel must be an integer")
        if isinstance(servo_pin, bool) or not isinstance(servo_pin, int):
            raise ConfigError(f"{context}.servo_pin must be an integer")
        sign = _number(item, "sign", context)
        if sign not in {-1.0, 1.0}:
            raise ConfigError(f"{context}.sign must be -1 or 1")
        joint = JointConfig(
            name=name,
            channel=channel,
            servo_pin=servo_pin,
            offset_deg=_number(item, "offset_deg", context),
            sign=sign,
            min_deg=_number(item, "min_deg", context),
            max_deg=_number(item, "max_deg", context),
            home_deg=_number(item, "home_deg", context),
            max_velocity_deg_s=_number(item, "max_velocity_deg_s", context),
        )
        if not 0.0 <= joint.min_deg < joint.max_deg <= 180.0:
            raise ConfigError(f"{context}: require 0 <= min_deg < max_deg <= 180")
        if not 0.0 <= joint.offset_deg <= 180.0:
            raise ConfigError(f"{context}: offset_deg must be in [0, 180]")
        if not joint.min_deg <= joint.home_deg <= joint.max_deg:
            raise ConfigError(f"{context}: home_deg must be inside the joint limits")
        if not 0.0 < joint.max_velocity_deg_s <= 180.0:
            raise ConfigError(f"{context}: max_velocity_deg_s must be in (0, 180]")
        if not 2 <= joint.servo_pin <= 12:
            raise ConfigError(
                f"{context}: servo_pin must be in [2, 12] for the current wiring profile"
            )
        joints.append(joint)

    if sorted(j.channel for j in joints) != list(range(NJOINTS)):
        raise ConfigError(f"{source}: joint channels must be unique values 0..{NJOINTS - 1}")
    if len({j.name for j in joints}) != NJOINTS:
        raise ConfigError(f"{source}: joint names must be unique")
    if len({j.servo_pin for j in joints}) != NJOINTS:
        raise ConfigError(f"{source}: servo pins must be unique")
    joints.sort(key=lambda joint: joint.channel)

    grip_raw = raw.get("gripper")
    if not isinstance(grip_raw, dict):
        raise ConfigError(f"{source}: gripper must be a mapping")
    grip_pin = grip_raw.get("servo_pin")
    if isinstance(grip_pin, bool) or not isinstance(grip_pin, int):
        raise ConfigError(f"{source}: gripper.servo_pin must be an integer")
    if not 2 <= grip_pin <= 12:
        raise ConfigError(f"{source}: gripper.servo_pin must be in [2, 12]")
    gripper = GripperConfig(
        servo_pin=grip_pin,
        open_percent=_number(grip_raw, "open_percent", "gripper"),
        closed_percent=_number(grip_raw, "closed_percent", "gripper"),
        max_velocity_percent_s=_number(grip_raw, "max_velocity_percent_s", "gripper"),
    )
    if not (0.0 <= gripper.open_percent <= 100.0 and
            0.0 <= gripper.closed_percent <= 100.0):
        raise ConfigError(f"{source}: gripper endpoints must be in [0, 100]")
    if gripper.open_percent == gripper.closed_percent:
        raise ConfigError(f"{source}: gripper endpoints must differ")
    if gripper.max_velocity_percent_s <= 0.0:
        raise ConfigError(f"{source}: gripper max velocity must be positive")
    if gripper.servo_pin in {j.servo_pin for j in joints}:
        raise ConfigError(f"{source}: gripper servo pin duplicates a joint pin")

    return ArmConfig(
        source=source,
        calibrated=raw["calibrated"],
        state_source=state_source,
        command_timeout_ms=timeout,
        control_rate_hz=rate,
        joints=tuple(joints),
        gripper=gripper,
    )
