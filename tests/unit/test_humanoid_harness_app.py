"""Mode-transition regressions using MuJoCo without a viewer or camera device."""
import queue
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("mujoco")
pytest.importorskip("viser")
pytest.importorskip("mjviser")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "humanoid_harness"))
from app import HumanoidApp
from fetch_model import DEFAULT_ASSETS
from simulation import WORKSPACE, HumanoidSimulation

pytestmark = pytest.mark.skipif(not (DEFAULT_ASSETS / "provenance.json").exists(), reason="Run fetch_model.py to download pinned G1 assets")


class FakeCamera:
    """Lifecycle counter only: never opens a process or video device."""

    def __init__(self):
        self.starts = self.stops = 0

    def start(self):
        self.starts += 1

    def stop(self):
        self.stops += 1


@pytest.fixture
def app():
    # Bypass __init__: it intentionally opens a local Viser server. Exercise
    # the real event handler with the real simulator and plain GUI values.
    instance = HumanoidApp.__new__(HumanoidApp)
    instance.sim = HumanoidSimulation(width=320, height=240)
    instance.commands = queue.SimpleQueue()
    instance.mode, instance.arm = "hold", "right"
    instance.last_target = np.array(WORKSPACE["right"]["default"])
    instance.pending_target = instance.last_target.copy()
    instance.holding_camera = False
    instance.camera_error = None
    instance.camera = FakeCamera()
    instance.gizmo = SimpleNamespace(position=instance.last_target.copy(), visible=False)
    instance.manual = SimpleNamespace(value=False)
    instance.arm_control = SimpleNamespace(value="Right")
    instance.camera_start = SimpleNamespace(disabled=False)
    instance.camera_stop = SimpleNamespace(disabled=True)
    instance.camera_image = SimpleNamespace(image=None)
    instance.history = []
    yield instance
    instance.sim.close()


def test_selecting_arm_while_held_does_not_resume_reaching(app):
    before = app.sim.data.qpos.copy()
    app.commands.put(("arm", "left"))
    command = app._events()
    assert app.mode == "hold"
    state = app.sim.step(command)
    for _ in range(24):
        state = app.sim.step()
    assert state["arm"] == "left"
    assert state["mode"] == "hold"
    # Gravity causes a small servo deflection, but changing the selected arm
    # must not begin the ~0.6-radian reaching movement observed before the fix.
    assert np.max(np.abs(app.sim.data.qpos - before)) < 0.1
    assert app.camera.starts == 0


def test_arm_then_manual_in_one_tick_uses_selected_arm_workspace(app):
    assert app.sim.state()["target"][1] < 0  # Old right-arm marker.
    app.commands.put(("arm", "left"))
    app.commands.put(("manual", True))
    command = app._events()
    assert app.mode == "reach"
    target = np.asarray(command["target"])
    assert np.all(target >= WORKSPACE["left"]["min"])
    assert np.all(target <= WORKSPACE["left"]["max"])
    np.testing.assert_array_equal(app.last_target, target)
    state = app.sim.step(command)
    assert state["arm"] == "left"
    assert not state["safety"]["target_clamped"]


def test_arm_then_camera_start_in_one_tick_holds_selected_arm(app):
    left_hand = np.asarray(app.sim.state()["hand_positions"]["left"])
    app.commands.put(("arm", "left"))
    app.commands.put(("camera_start", None))
    command = app._events()
    assert app.mode == "camera"
    assert app.holding_camera
    assert app.camera.starts == 1
    np.testing.assert_allclose(app.last_target, np.clip(left_hand, WORKSPACE["left"]["min"], WORKSPACE["left"]["max"]))
    state = app.sim.step(command)
    assert state["mode"] == "hold"
    assert state["arm"] == "left"
    assert not state["reached"]
