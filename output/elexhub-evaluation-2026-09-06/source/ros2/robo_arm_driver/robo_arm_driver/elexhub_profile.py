"""Version 2: a measured physical profile and an explicit provisional simulator.

No numerical preview value is evidence that a physical arm is calibrated.
This module deliberately depends only on the driver's existing dependencies.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from .config import ArmConfig, ConfigError, GripperConfig, JointConfig, _number


def profile_digest(raw: dict) -> str:
    return hashlib.sha256(
        json.dumps(raw, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()[:24]


def hardware_gaps(raw: dict, *, commissioning: bool = False) -> list[str]:
    """Checklist data must exist before firmware can enable even one output."""
    gaps = []
    evidence = raw.get("evidence", {})
    keys = [
        "topology",
        "power_review",
        "ground_and_polarity",
        "cutoff",
        "oe_default_disabled",
    ]
    if not commissioning:
        keys += [
            "joint_calibration",
            "home_repetitions",
            "watchdog",
            "geometry",
            "camera_to_base",
        ]
    for key in keys:
        if not isinstance(evidence.get(key), str) or not evidence[key].strip():
            gaps.append(f"evidence.{key}")
    power = raw.get("power", {})
    required = [
        "battery_min_v",
        "battery_full_v",
        "battery_discharge_a",
        "bms_continuous_a",
        "regulator_input_min_v",
        "regulator_input_max_v",
        "servo_rail_v",
        "regulator_continuous_a",
        "regulator_peak_a",
        "fuse_a",
        "wire_rating_a",
        "cutoff_rating_a",
    ]
    valid = {}
    for key in required:
        value = power.get(key)
        if (
            isinstance(value, bool)
            or not isinstance(value, (float, int))
            or not math.isfinite(value)
            or value <= 0
        ):
            gaps.append(f"power.{key}")
        else:
            valid[key] = float(value)
    if len(valid) == len(required):
        p = valid
        if (
            not p["regulator_input_min_v"]
            <= p["battery_min_v"]
            < p["battery_full_v"]
            <= p["regulator_input_max_v"]
        ):
            gaps.append("power.regulator_input_range")
        if p["fuse_a"] > min(
            p["wire_rating_a"],
            p["cutoff_rating_a"],
            p["bms_continuous_a"],
            p["battery_discharge_a"],
        ):
            gaps.append("power.fuse_exceeds_upstream_ratings")
        if p["regulator_peak_a"] < p["regulator_continuous_a"]:
            gaps.append("power.regulator_peak_a")
    total_stall = 0.0
    for i, servo in enumerate(raw.get("joints", []) + [raw.get("gripper", {})]):
        spec = servo.get("measured", {})
        for key in ["model", "datasheet", "calibration_record"]:
            if not isinstance(spec.get(key), str) or not spec[key].strip():
                gaps.append(f"actuator[{i}].{key}")
        nums = {}
        for key in [
            "voltage_min_v",
            "voltage_max_v",
            "stall_current_a",
            "pulse_min_us",
            "pulse_max_us",
        ]:
            value = spec.get(key)
            if (
                isinstance(value, bool)
                or not isinstance(value, (float, int))
                or not math.isfinite(value)
                or value <= 0
            ):
                gaps.append(f"actuator[{i}].{key}")
            else:
                nums[key] = value
        if len(nums) == 5:
            total_stall += nums["stall_current_a"]
            if not 400 <= nums["pulse_min_us"] < nums["pulse_max_us"] <= 2600:
                gaps.append(f"actuator[{i}].pulse_range")
            if (
                "servo_rail_v" in valid
                and not nums["voltage_min_v"]
                <= valid["servo_rail_v"]
                <= nums["voltage_max_v"]
            ):
                gaps.append(f"actuator[{i}].voltage_incompatible")
        if servo.get("calibration_verified") is not True and not commissioning:
            gaps.append(f"actuator[{i}].calibration_verified")
        if commissioning:
            lo, hi = servo.get("commission_min"), servo.get("commission_max")
            home = servo.get("home_deg", servo.get("open_percent"))
            if (
                any(
                    isinstance(v, bool)
                    or not isinstance(v, (float, int))
                    or not math.isfinite(v)
                    for v in [lo, hi, home]
                )
                or not 0 <= lo < hi <= (180 if i < len(raw.get("joints", [])) else 100)
                or hi - lo > 4
                or not lo <= home <= hi
            ):
                gaps.append(f"actuator[{i}].narrow_commissioning_window")
    # Deliberately conservative supply gate: do not rely on guessed load diversity.
    if (
        "regulator_continuous_a" in valid
        and valid["regulator_continuous_a"] < total_stall
    ):
        gaps.append("power.regulator_continuous_a_below_total_stall")
    if raw.get("geometry", {}).get("status") != "measured" and not commissioning:
        gaps.append("geometry.status")
    return sorted(set(gaps))


def load_v2(raw: dict, source: Path) -> ArmConfig:
    if raw.get("actuator_backend") != "pca9685":
        raise ConfigError("v2 actuator_backend must be pca9685")
    if raw.get("runtime") not in {"hardware", "simulation"}:
        raise ConfigError("v2 runtime must be hardware or simulation")
    for flag in ["calibrated", "commissioning_allowed"]:
        if not isinstance(raw.get(flag), bool):
            raise ConfigError(f"{flag} must be boolean")
    if raw.get("state_source") != "commanded":
        raise ConfigError(
            "PCA9685 profile state_source must be commanded; vision is a separate observation"
        )
    simulation = raw["runtime"] == "simulation"
    if not simulation and (raw["calibrated"] or raw["commissioning_allowed"]):
        gaps = hardware_gaps(raw, commissioning=not raw["calibrated"])
        if gaps:
            raise ConfigError("hardware evidence missing: " + ", ".join(gaps))
    items = raw.get("joints")
    if not isinstance(items, list) or not 1 <= len(items) <= 15:
        raise ConfigError("v2 requires 1..15 arm joints plus a gripper")
    joints = []
    for index, item in enumerate(items):
        if (
            not isinstance(item, dict)
            or not isinstance(item.get("name"), str)
            or not item["name"]
        ):
            raise ConfigError(f"joint {index}: missing name")
        ch = item.get("pwm_channel")
        if isinstance(ch, bool) or not isinstance(ch, int) or not 0 <= ch < 16:
            raise ConfigError("pwm_channel must be an integer in 0..15")
        measured = item.get("measured", {})
        j = JointConfig(
            name=item["name"],
            channel=index,
            servo_pin=None,
            pwm_channel=ch,
            pulse_min_us=measured.get("pulse_min_us"),
            pulse_max_us=measured.get("pulse_max_us"),
            **{
                k: _number(item, k, item["name"])
                for k in [
                    "offset_deg",
                    "sign",
                    "min_deg",
                    "max_deg",
                    "home_deg",
                    "max_velocity_deg_s",
                ]
            },
        )
        if j.sign not in {-1, 1} or not 0 <= j.offset_deg <= 180:
            raise ConfigError("invalid joint sign/offset")
        if (
            not 0 <= j.min_deg < j.max_deg <= 180
            or not j.min_deg <= j.home_deg <= j.max_deg
        ):
            raise ConfigError("invalid joint limits/home")
        if not 0 < j.max_velocity_deg_s <= 180:
            raise ConfigError("invalid joint velocity")
        joints.append(j)
    g = raw.get("gripper", {})
    ch = g.get("pwm_channel")
    if isinstance(ch, bool) or not isinstance(ch, int) or not 0 <= ch < 16:
        raise ConfigError("gripper pwm_channel must be in 0..15")
    measured = g.get("measured", {})
    gripper = GripperConfig(
        servo_pin=None,
        pwm_channel=ch,
        pulse_min_us=measured.get("pulse_min_us"),
        pulse_max_us=measured.get("pulse_max_us"),
        **{
            k: _number(g, k, "gripper")
            for k in ["open_percent", "closed_percent", "max_velocity_percent_s"]
        },
    )
    if (
        not 0 <= gripper.open_percent <= 100
        or not 0 <= gripper.closed_percent <= 100
        or gripper.open_percent == gripper.closed_percent
    ):
        raise ConfigError("invalid gripper endpoints")
    if not 0 < gripper.max_velocity_percent_s <= 200:
        raise ConfigError("invalid gripper speed")
    channels = [j.pwm_channel for j in joints] + [ch]
    if len(channels) != len(set(channels)) or len({j.name for j in joints}) != len(
        joints
    ):
        raise ConfigError("duplicate names or PCA9685 channels")
    timeout = raw.get("command_timeout_ms")
    rate = _number(raw, "control_rate_hz", "profile")
    if (
        isinstance(timeout, bool)
        or not isinstance(timeout, int)
        or not 100 <= timeout <= 5000
        or not 1 <= rate <= 100
    ):
        raise ConfigError("invalid control rate/timeout")
    pca = raw.get("pca9685", {})
    if pca.get("address") != 0x40 or pca.get("oe_pin") != 22 or pca.get("pwm_hz") != 50:
        raise ConfigError("this wiring revision requires address 0x40, OE D22, 50 Hz")
    return ArmConfig(
        source=source,
        calibrated=raw["calibrated"],
        state_source="commanded",
        command_timeout_ms=timeout,
        control_rate_hz=rate,
        joints=tuple(joints),
        gripper=gripper,
        schema_version=2,
        actuator_backend="pca9685",
        simulation=simulation,
        commissioning_allowed=raw["commissioning_allowed"],
        profile_hash=profile_digest(raw),
        profile_data=raw,
    )
