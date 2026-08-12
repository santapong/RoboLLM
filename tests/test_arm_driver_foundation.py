"""Fast, hardware-free tests for the v0.2 arm safety boundary."""
from pathlib import Path

import pytest

from robo_arm_driver.config import ConfigError, load_arm_config
from robo_arm_driver.safety import TrajectorySampler, validate_trajectory

ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "ros2" / "robo_arm_driver" / "config"


def test_physical_profile_is_fail_closed_and_not_generic_0_180():
    config = load_arm_config(CONFIG_DIR / "joints.yaml")
    assert config.calibrated is False
    assert config.state_source == "commanded"
    assert all(joint.min_deg > 0.0 and joint.max_deg < 180.0 for joint in config.joints)
    with pytest.raises(ConfigError, match="not calibrated"):
        config.require_calibrated()


def test_simulation_profile_is_explicitly_motion_ready():
    config = load_arm_config(CONFIG_DIR / "joints.sim.yaml")
    config.require_calibrated()
    assert config.joint_names == tuple(f"joint{i}" for i in range(1, 7))
    assert config.joints[0].raw_to_rad(90.0) == pytest.approx(0.0)
    assert config.joints[0].rad_to_raw(0.0) == pytest.approx(90.0)


def test_trajectory_reorders_names_and_sampler_interpolates():
    config = load_arm_config(CONFIG_DIR / "joints.sim.yaml")
    names = list(reversed(config.joint_names))
    source_positions = [[0.6, 0.5, 0.4, 0.3, 0.2, 0.1]]
    points = validate_trajectory(names, source_positions, [1.0], [0.0] * 6, config)
    assert points[0].positions == pytest.approx((0.1, 0.2, 0.3, 0.4, 0.5, 0.6))

    sampler = TrajectorySampler([0.0] * 6, points, start_s=10.0)
    halfway, done = sampler.sample(10.5)
    assert halfway == pytest.approx((0.05, 0.1, 0.15, 0.2, 0.25, 0.3))
    assert done is False
    final, done = sampler.sample(11.1)
    assert final == pytest.approx(points[0].positions)
    assert done is True


@pytest.mark.parametrize(
    ("positions", "times", "match"),
    [
        ([[2.0, 0, 0, 0, 0, 0]], [2.0], "outside configured"),
        ([[1.0, 0, 0, 0, 0, 0]], [0.1], "above configured"),
        ([[0.0] * 6], [0.0], "strictly increasing"),
    ],
)
def test_trajectory_rejects_limit_speed_and_time_violations(positions, times, match):
    config = load_arm_config(CONFIG_DIR / "joints.sim.yaml")
    with pytest.raises(ConfigError, match=match):
        validate_trajectory(config.joint_names, positions, times, [0.0] * 6, config)


def test_trajectory_requires_exact_joint_set():
    config = load_arm_config(CONFIG_DIR / "joints.sim.yaml")
    with pytest.raises(ConfigError, match="exactly once"):
        validate_trajectory(
            ["joint1"] * 6,
            [[0.0] * 6],
            [1.0],
            [0.0] * 6,
            config,
        )
