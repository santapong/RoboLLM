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
import app as app_module
from app import HumanoidApp, InitialViewFit, camera_aspect, view_pose
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
    instance.workspace = instance.sim.workspace
    instance.model_id = instance.sim.model_id
    instance.models = {entry["id"]: entry for entry in app_module.list_models()}
    instance.model_error = None
    instance.current_view = "3/4"
    instance.view_control = SimpleNamespace(value="3/4")
    instance._scene_generation = 1
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
    instance.model_control = SimpleNamespace(value=instance.models[instance.model_id]["label"])
    instance.model_status = SimpleNamespace(content="")
    instance.model_notes = SimpleNamespace(content="")
    instance.reference_folder = SimpleNamespace(visible=False)
    instance.reference_image = SimpleNamespace(image=None)
    instance.server = SimpleNamespace(get_clients=dict)
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


def test_view_presets_show_front_side_rear_and_three_quarter():
    center, distance = [0, 0, 0.8], 2.0
    front, look = view_pose("Front", center, distance)
    side, _ = view_pose("Side", center, distance)
    rear, _ = view_pose("Rear", center, distance)
    oblique, _ = view_pose("3/4", center, distance)
    assert front[0] > 0 and front[1] == 0
    assert side[0] == 0 and side[1] < 0
    assert rear[0] < 0 and rear[1] == 0
    assert oblique[0] > 0 and oblique[1] < 0
    assert look == tuple(center)
    with pytest.raises(ValueError):
        view_pose("Front", center, float("nan"))


def test_view_event_changes_clients_without_moving_robot(app):
    client = SimpleNamespace(camera=SimpleNamespace(aspect=1.5))
    app.server.get_clients = lambda: {1: client}
    app.commands.put(("view", "Rear"))
    assert app._events() is None
    assert app.current_view == "Rear"
    assert client.camera.position[0] < app.sim.view_center[0]
    assert app.mode == "hold" and app.camera.starts == 0


def test_view_selector_starts_synchronized_and_programmatic_changes_do_not_loop(app):
    class ViewControl:
        def __init__(self, options):
            self._value, self.options = options[0], options
            self.callbacks = []

        @property
        def value(self):
            return self._value

        @value.setter
        def value(self, value):
            self._value = value
            for callback in self.callbacks:
                callback(SimpleNamespace(client_id=None, target=self))

        def on_click(self, callback):
            self.callbacks.append(callback)

        def click(self, value):
            if value == self.value:
                return  # Reproduce the initially selected Front click failure.
            self._value = value
            for callback in self.callbacks:
                callback(SimpleNamespace(client_id=1, target=self))

    app.server.gui = SimpleNamespace(add_button_group=lambda _, options, **__: ViewControl(options))
    app._make_view_control()
    assert app.view_control.value == app.current_view == "3/4"
    assert app.view_control.options == app_module.VIEWS
    app.view_control.click("Front")
    app._events()
    assert app.current_view == app.view_control.value == "Front"
    assert app.commands.empty()
    app._set_view("3/4")
    assert app.current_view == app.view_control.value == "3/4"
    assert app.commands.empty()


def test_narrow_client_view_fits_arm_width_and_invalid_aspect_uses_base(app):
    wide = SimpleNamespace(camera=SimpleNamespace(aspect=1.5))
    narrow = SimpleNamespace(camera=SimpleNamespace(aspect=0.35))
    invalid = SimpleNamespace(camera=SimpleNamespace(aspect=float("nan")))
    app.server.get_clients = lambda: {1: wide, 2: narrow, 3: invalid}
    app._set_view("Front")
    base = app.sim.view_distance
    center = app.sim.view_center
    assert wide.camera.position[0] - center[0] == pytest.approx(base)
    assert invalid.camera.position == wide.camera.position
    narrow_distance = narrow.camera.position[0] - center[0]
    width = max(0.8, app.workspace["left"]["max"][1] - app.workspace["right"]["min"][1]) + 0.25
    assert narrow_distance > base
    assert 2 * narrow_distance * np.tan(0.4) * narrow.camera.aspect >= width
    assert wide.camera.look_at == narrow.camera.look_at == tuple(center)


def test_initial_view_fit_corrects_once_then_leaves_orbit_and_zoom_alone():
    fit = InitialViewFit(1.5, now=10)
    assert not fit.observe(None, now=10.1)
    assert not fit.observe(1.5, now=10.2)  # Orbit update, same viewport.
    assert fit.observe(0.35, now=10.4)  # Sidebar establishes the real canvas.
    assert not fit.observe(1.5, now=10.5)
    assert not fit.observe(0.25, now=11)
    expired = InitialViewFit(1.5, now=10)
    assert not expired.observe(0.35, now=13.1)
    assert expired.finished
    unknown = InitialViewFit(None, now=10)
    assert unknown.observe(0.35, now=10.1)


def test_viewport_callback_queues_only_one_correction_and_waits_for_initial_data(app):
    class ClientCamera:
        def __init__(self):
            self.ready = False
            self.ratio = 1.5
            self.callbacks = []

        @property
        def aspect(self):
            assert self.ready
            return self.ratio

        def on_update(self, callback):
            self.callbacks.append(callback)

        def receive(self, aspect):
            self.ready, self.ratio = True, aspect
            for callback in self.callbacks:
                callback(self)

    client = SimpleNamespace(client_id=1, camera=ClientCamera())
    app.server.get_clients = lambda: {1: client}
    assert camera_aspect(client.camera) is None
    app._initialize_client_view(client)
    app._events()  # Must not read uninitialized camera.aspect without a guard.
    base_position = client.camera.position
    client.camera.receive(0.35)
    app._events()
    assert client.camera.position[0] > base_position[0]
    corrected_position = client.camera.position
    client.camera.position = (9, 8, 7)  # User-controlled orbit / zoom.
    client.camera.receive(0.35)
    client.camera.receive(0.4)
    assert app.commands.empty()
    assert client.camera.position == (9, 8, 7)
    assert corrected_position != client.camera.position


def test_model_change_stops_camera_and_uses_new_workspace(app, monkeypatch):
    class NewSimulation:
        def __init__(self, **_):
            self.model_id, self.model_label = "test_model", "Tall test robot"
            self.workspace = {arm: {name: (np.asarray(value) + [0, 0, 0.6]).tolist()
                                   for name, value in box.items()} for arm, box in WORKSPACE.items()}
            self.view_center, self.view_distance = [0, 0, 1.2], 3.0
            self.model = SimpleNamespace(nq=19)
            self.closed = False

        def close(self):
            self.closed = True

    old = app.sim
    closed = []
    old_close = old.close
    old.close = lambda: (closed.append(True), old_close())
    app.models["test_model"] = {"id": "test_model", "label": "Tall test robot", "available": True}
    monkeypatch.setattr(app_module, "HumanoidSimulation", NewSimulation)
    rebuilt = []
    monkeypatch.setattr(app, "_build_scene", lambda: (rebuilt.append(app.sim), setattr(app, "_scene_generation", 2)))
    app.mode, app.arm = "camera", "left"
    app.history = [np.ones(3)]
    app.commands.put(("model", "test_model"))
    command = app._events()
    assert command == {"mode": "hold", "arm": "right"}
    assert app.mode == "hold" and app.camera.stops == 1 and app.camera.starts == 0
    assert closed and len(rebuilt) == 1
    assert app.model_id == "test_model" and app.arm == "right" and app.history == []
    np.testing.assert_array_equal(app.pending_target, app.workspace["right"]["default"])
    assert app.pending_target[2] > WORKSPACE["right"]["max"][2]
    app.mode = "reach"
    expected = app.pending_target.copy()
    app.commands.put(("target", {"generation": 1, "position": [0, 0, 0]}))
    app._events()
    np.testing.assert_array_equal(app.pending_target, expected)


def test_failed_model_selection_retains_old_model_and_holds(app, monkeypatch):
    old = app.sim
    app.models["missing"] = {"id": "missing", "label": "Unavailable model", "available": False}
    monkeypatch.setattr(app, "_build_scene", lambda: pytest.fail("Missing assets must not rebuild the scene"))
    app.commands.put(("model", "missing"))
    command = app._events()
    assert command == {"mode": "hold", "arm": "right"}
    assert app.sim is old and app.model_id == "g1"
    assert "not available locally" in app.model_error
    assert app.model_control.value == app.models["g1"]["label"]
    assert app.camera.stops == 1 and app.camera.starts == 0


def test_concept_reference_shown_only_when_selected_local_image_exists(app, tmp_path):
    from PIL import Image

    image = tmp_path / "reference.png"
    Image.new("RGB", (30, 20), "cyan").save(image)
    app.models[app.model_id] = {"id": app.model_id, "label": "Test concept", "available": True,
                                "source_kind": "original-procedural", "reference_image": str(image)}
    app._model_details()
    assert app.reference_folder.visible
    assert app.reference_image.image.shape == (20, 30, 3)
    assert "Original procedural concept" in app.model_status.content
    app.models[app.model_id].pop("reference_image")
    app._model_details()
    assert not app.reference_folder.visible


def test_scene_rebuild_removes_old_handles_and_gizmo_callback(app, monkeypatch):
    class Gizmo:
        def __init__(self, **kwargs):
            self.position = kwargs["position"]
            self.callbacks = []

        def on_update(self, callback):
            self.callbacks.append(callback)

        def remove_update_callback(self, _):
            self.callbacks.clear()

    class SceneApi:
        def __init__(self):
            self.handles = [object()]  # Previous-model meshes and floor.
            self.resets = 0

        def reset(self):
            self.handles.clear()
            self.resets += 1

        def add_transform_controls(self, _, **kwargs):
            handle = Gizmo(**kwargs)
            self.handles.append(handle)
            return handle

        def add_line_segments(self, *_, **__):
            handle = SimpleNamespace()
            self.handles.append(handle)
            return handle

    class Scene:
        def __init__(self, server, *_args, **_kwargs):
            server.scene.handles.append(self)
            self.refresh_handler = None

        def set_refresh_handler(self, handler):
            self.refresh_handler = handler

        def update_from_mjdata(self, _):
            pass

    app.server.scene = SceneApi()
    app.scene = Scene(app.server)
    app.gizmo = Gizmo(position=[0, 0, 0])
    app.gizmo.on_update(lambda _: None)
    previous = app.gizmo
    monkeypatch.setattr(app_module, "ViserMujocoScene", Scene)
    app._build_scene()
    assert not previous.callbacks and len(app.gizmo.callbacks) == 1
    assert len(app.server.scene.handles) == 3
    previous = app.gizmo
    app._build_scene()
    assert not previous.callbacks and len(app.gizmo.callbacks) == 1
    assert app.server.scene.resets == 2 and len(app.server.scene.handles) == 3
    app.gizmo.callbacks[0](None)
    kind, payload = app.commands.get()
    assert kind == "target" and payload["generation"] == app._scene_generation
