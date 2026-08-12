"""End-to-end serial contract test against the pseudo-terminal Uno."""
from __future__ import annotations

import os
import importlib.util
from pathlib import Path
import subprocess
import sys
import time

import pytest

pytest.importorskip("serial", reason="pyserial is required for the serial integration test")

from robo_arm_driver.arm_serial import ArmSerial, FirmwareError
from robo_arm_driver.config import ConfigError

ROOT = Path(__file__).resolve().parents[1]
SIM_CONFIG = ROOT / "ros2" / "robo_arm_driver" / "config" / "joints.sim.yaml"
PHYSICAL_CONFIG = ROOT / "ros2" / "robo_arm_driver" / "config" / "joints.yaml"


def _start_sim(config_path: Path):
    env = os.environ.copy()
    env["ARM_CONFIG"] = str(config_path)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    process = subprocess.Popen(
        [sys.executable, str(ROOT / "hardware" / "sim_uno.py")],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        env=env,
    )
    banner = process.stdout.readline().strip()
    assert banner.startswith("SIM UNO on "), banner
    port = banner.split()[3]
    process.stdout.readline()
    return process, port


def test_driver_and_simulated_firmware_contract():
    process, port = _start_sim(SIM_CONFIG)
    try:
        with ArmSerial(port=port, boot_wait_s=0.05, config_path=SIM_CONFIG) as arm:
            assert "arm-fw 2.1" in arm.ping()
            before = arm.get_state()
            arm.set_action([0.1, 0, 0, 0, 0, 0], gripper=0.25)
            time.sleep(0.2)
            after = arm.get_state()
            assert after.q[0] > before.q[0]

            with pytest.raises(ConfigError, match="outside configured"):
                arm.set_action([2.0, 0, 0, 0, 0, 0])
            with pytest.raises(FirmwareError, match="commissioning_disabled"):
                arm._cmd_state("C 0 90")
    finally:
        process.terminate()
        process.wait(timeout=5)


def test_commissioning_profile_allows_only_narrow_single_joint_motion():
    process, port = _start_sim(PHYSICAL_CONFIG)
    try:
        with ArmSerial(port=port, boot_wait_s=0.05, config_path=PHYSICAL_CONFIG) as arm:
            before = arm.get_state()
            arm.commission_joint(0, 91.0)
            time.sleep(0.1)
            after = arm.get_state()
            assert after.q[0] > before.q[0]
            assert after.q[1:] == pytest.approx(before.q[1:])
            with pytest.raises(ConfigError, match="outside configured"):
                arm.commission_joint(0, 120.0)
            with pytest.raises(ConfigError, match="not calibrated"):
                arm.set_action([0.0] * 6)
            with pytest.raises(FirmwareError, match="not_calibrated"):
                arm._cmd_state("S 90 90 90 90 90 90 50")
    finally:
        process.terminate()
        process.wait(timeout=5)


def test_simulated_watchdog_deenergizes_after_timeout(monkeypatch):
    monkeypatch.setenv("ARM_CONFIG", str(SIM_CONFIG))
    spec = importlib.util.spec_from_file_location("sim_uno_watchdog_test", ROOT / "hardware" / "sim_uno.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.enabled = True
    module.last_command = 10.0
    before_timeout = 10.0 + (module.CONFIG.command_timeout_ms - 1) / 1000.0
    after_timeout = 10.0 + (module.CONFIG.command_timeout_ms + 1) / 1000.0
    assert module.watchdog_tick(before_timeout) is False
    assert module.enabled is True
    assert module.watchdog_tick(after_timeout) is True
    assert module.enabled is False
