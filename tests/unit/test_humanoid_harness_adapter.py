import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "examples/humanoid_harness"))
from harness import camera_target, slew_target


def test_stale_lost_invalid_observation_holds():
    low, high = np.array([0.16, -0.5, 0.9]), np.array([0.43, -0.13, 1.35])
    good = {"running": True, "tracked": True, "target": [0.1, 0, 0.3]}
    assert camera_target(good, 0.1, low, high) is not None
    for packet, age in [({**good, "tracked": False}, 0), ({**good, "running": False}, 0),
                        ({**good, "target": [float("nan"), 0, 0]}, 0),
                        ({**good, "target": ["bad", 0, 0]}, 0),
                        (good, -1), (good, 0.6), (good, float("nan"))]:
        assert camera_target(packet, age, low, high) is None


def test_mapping_has_fixed_depth_and_clamped_image_axes():
    low, high = np.array([0.16, -0.5, 0.9]), np.array([0.43, -0.13, 1.35])
    packet = {"running": True, "tracked": True, "target": [20, 100, -20]}
    target = camera_target(packet, 0, low, high)
    assert np.allclose(target, [0.295, -0.5, 0.9])


def test_target_slew_is_a_vector_bound():
    value = slew_target([0, 0, 0], [1, 1, 1])
    assert np.linalg.norm(value) == pytest.approx(0.012)
    with pytest.raises(ValueError):
        slew_target([0, 0, 0], [float("inf"), 1, 1])
