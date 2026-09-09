"""Interactive local humanoid experiments: MuJoCo + Viser + optional webcam."""

from __future__ import annotations

import argparse
import io
import json
import os
import queue
import signal
import threading
import time
from pathlib import Path

import numpy as np
import viser
from camera_bridge import CameraBridge
from harness import camera_target, slew_target
from mjviser import ViserMujocoScene
from PIL import Image, ImageDraw
from simulation import HumanoidSimulation, list_models

HERE = Path(__file__).resolve().parent
VIEWS = ("Front", "Side", "Rear", "3/4")


def camera_aspect(camera):
    """Viser has no aspect value until its first viewport message arrives."""
    try:
        aspect = getattr(camera, "aspect", None)
    except (AssertionError, ZeroDivisionError):
        return None
    return aspect if isinstance(aspect, (float, int)) and np.isfinite(aspect) and aspect > 0 else None


class InitialViewFit:
    """Permit one startup viewport correction; never track later orbit/zoom."""

    def __init__(self, aspect, now):
        self.aspect, self.deadline = aspect, now + 3.0
        self.finished = False
        self.lock = threading.Lock()

    def observe(self, aspect, now):
        with self.lock:
            if self.finished:
                return False
            if now > self.deadline:
                self.finished = True
                return False
            if aspect is None:
                return False
            if self.aspect is None or abs(aspect - self.aspect) > 0.01:
                self.finished = True
                return True
            return False


def view_pose(name, center, distance):
    """Same framing for different robots; world +X is the robot's front."""
    directions = {"Front": (1, 0, 0.05), "Side": (0, -1, 0.05),
                  "Rear": (-1, 0, 0.05), "3/4": (0.707, -0.707, 0.20)}
    if name not in directions:
        raise ValueError(f"Unknown view: {name}")
    point = np.asarray(center, dtype=float)
    if point.shape != (3,) or not np.all(np.isfinite(point)) or not np.isfinite(distance) or distance <= 0:
        raise ValueError("View framing must be finite with positive distance")
    return tuple(point + float(distance) * np.array(directions[name])), tuple(point)


def camera_placeholder():
    image = Image.new("RGB", (480, 270), "#1c2635")
    draw = ImageDraw.Draw(image)
    draw.text((125, 108), "CAMERA OFF", fill="#dce7f4", font_size=28)
    draw.text((87, 153), "Start camera to follow your hand", fill="#adbccd", font_size=19)
    return np.asarray(image)


class HumanoidApp:
    def __init__(self, args):
        self.args = args
        self.commands = queue.SimpleQueue()
        self.stop = threading.Event()
        self.mode, self.arm = "demo", "right"
        self.models = {entry["id"]: entry for entry in list_models()}
        self.sim = HumanoidSimulation(model_name=getattr(args, "model", "g1"))
        self.workspace = self.sim.workspace
        self.model_id = self.sim.model_id
        self.model_error = None
        self.current_view = "3/4"
        self._scene_generation = 0
        self.pending_target = np.array(self.workspace[self.arm]["default"])
        self.last_target = self.pending_target.copy()
        self.holding_camera = False
        self.camera_error = None
        self.last_client_time = time.monotonic()
        self.camera = CameraBridge(args.camera_python, args.camera, args.hand_model)
        self.server = viser.ViserServer(host="127.0.0.1", port=args.port, label="RoboLLM · Humanoid Lab")
        self.server.gui.configure_theme(dark_mode=True, control_width="large", show_logo=False,
                                        show_share_button=False, brand_color=(97, 169, 245))
        self.server.gui.main_panel.dock_right()
        self.server.scene.set_up_direction("+z")
        self.history = []
        self.scene = None
        self._build_scene()
        self._make_controls()

        @self.server.on_client_connect
        def connected(client):
            self._initialize_client_view(client)

    def _initialize_client_view(self, client):
        initial_fit = InitialViewFit(camera_aspect(client.camera), time.monotonic())
        self.commands.put(("client_view", client.client_id))

        @client.camera.on_update
        def viewport_ready(camera):
            # The sidebar can resize the canvas after on_client_connect. Viser
            # has no callback-removal API here, so this one becomes inert after
            # a single correction or its short initialization deadline.
            if initial_fit.observe(camera_aspect(camera), time.monotonic()):
                self.commands.put(("client_view", client.client_id))

    def _build_scene(self):
        """Replace every visual handle, including grids not owned by MjViser.

        ViserMujocoScene has no destroy API in the installed version. Viser's
        public scene.reset() removes registered nodes and their callbacks.
        Our GUI and client-connect callback are registered only once.
        """
        if self.scene is not None:
            self.scene.set_refresh_handler(None)
            self.gizmo.remove_update_callback("all")
        self.server.scene.reset()
        self._scene_generation += 1
        self.scene = ViserMujocoScene(self.server, self.sim.model, num_envs=1)
        self.scene.camera_tracking_enabled = False
        self.scene.update_from_mjdata(self.sim.data)
        self.gizmo = self.server.scene.add_transform_controls("/target-control", scale=0.15, disable_rotations=True,
                                                            position=self.last_target, visible=False)
        self.trail = self.server.scene.add_line_segments("/hand-trail", np.zeros((1, 2, 3), dtype=np.float32), (84, 219, 187), thickness=3, thickness_units="screen")
        generation, gizmo = self._scene_generation, self.gizmo

        @gizmo.on_update
        def dragged(_):
            self.commands.put(("target", {"generation": generation, "position": list(gizmo.position)}))

    def _set_view(self, name, client_id=None):
        if name not in VIEWS:
            return
        self.current_view = name
        if self.view_control.value != name:
            self.view_control.value = name
        clients = self.server.get_clients()
        selected = clients.values() if client_id is None else [clients[client_id]] if client_id in clients else []
        width = max(0.8, self.workspace["left"]["max"][1] - self.workspace["right"]["min"][1]) + 0.25
        for client in selected:
            distance = self.sim.view_distance
            aspect = camera_aspect(client.camera)
            if aspect is not None:
                distance = max(distance, width / (2 * np.tan(0.40) * aspect))
            position, look_at = view_pose(name, self.sim.view_center, distance)
            client.camera.position = position
            client.camera.look_at = look_at
            client.camera.up_direction = (0, 0, 1)
            client.camera.fov = 0.80

    def _make_view_control(self):
        # The API has no initial_value argument for button groups. Its initial
        # value is the first option; restore the display order after creation.
        initial_options = (self.current_view, *(name for name in VIEWS if name != self.current_view))
        self.view_control = self.server.gui.add_button_group("View", initial_options,
                                                            hint="Front, right side, rear, or three-quarter view")
        self.view_control.options = VIEWS

        @self.view_control.on_click
        def choose_view(event):
            # Viser also invokes this synchronously for programmatic assignments.
            # Only human clicks should enqueue a new view command.
            if event.client_id is not None:
                self.commands.put(("view", event.target.value))

    def _model_details(self):
        entry = self.models[self.model_id]
        kind = "Original procedural concept" if "original" in entry.get("source_kind", "") else "Public robot model"
        self.model_status.content = self.model_error or f"**{entry['label']}** · {kind}\n\n{entry.get('description', '')}"
        self.model_notes.content = ("**Perception:** local hand landmarks.\n\n**Controller:** inverse kinematics and position servos.\n\n"
                                   "**Harness:** tracking freshness, hold, workspace and target-speed limits.\n\n"
                                   f"**Simulation:** MuJoCo physics, {self.sim.model.nq} joints with a supported pelvis.\n\n"
                                   "This example demonstrates upper-body control. Walking and learned VLM/WAM control are separate experiments.")
        self.reference_folder.visible = False
        source = entry.get("reference_image")
        if source and Path(source).is_file():
            try:
                with Image.open(source) as reference:
                    reference = reference.convert("RGB")
                    reference.thumbnail((1600, 1000))
                    self.reference_image.image = np.asarray(reference)
                self.reference_folder.visible = True
            except (OSError, ValueError):
                self.model_status.content += "\n\nThe local design reference could not be opened."

    def _switch_model(self, model_id):
        if model_id == self.model_id:
            return False
        label = self.models.get(model_id, {}).get("label", model_id)
        self.model_status.content = f"Loading **{label}**…"
        self.model_control.disabled = True
        self._camera_off()
        self.mode, self.holding_camera = "hold", True
        self.manual.value = False
        self.gizmo.visible = False
        self.sim.step({"mode": "hold", "arm": self.arm})
        old_sim = self.sim
        new_sim = None
        try:
            if model_id not in self.models or not self.models[model_id].get("available", False):
                raise ValueError("The selected model is not available locally")
            new_sim = HumanoidSimulation(model_name=model_id)
            self.sim = new_sim
            self.workspace = new_sim.workspace
            self.last_target = np.asarray(self.workspace["right"]["default"])
            self._build_scene()
        except (OSError, RuntimeError, ValueError) as exc:
            if new_sim is not None:
                new_sim.close()
                self.sim = old_sim
                self.workspace = old_sim.workspace
                self.last_target = np.asarray(self.workspace[self.arm]["default"])
                self._build_scene()
            self.model_error = f"Could not load model: {exc}"
            self.model_control.value = self.models[self.model_id]["label"]
            self.model_control.disabled = False
            self._model_details()
            return False
        old_sim.close()
        self.model_id = new_sim.model_id
        self.model_error = None
        self.arm = "right"
        self.arm_control.value = "Right"
        self.history = []
        self.pending_target = self.last_target.copy()
        self.model_control.value = self.models[self.model_id]["label"]
        self.model_control.disabled = False
        self._model_details()
        self._set_view(self.current_view)
        return True

    def _make_controls(self):
        self.server.gui.add_html('<div style="padding:4px 0 12px"><div style="font-size:11px;letter-spacing:2px;color:#9cacbf">ROBOTICS WORKBENCH</div><div style="font-size:27px;font-weight:700;margin-top:7px">Humanoid Lab</div><div style="color:#c0ccdc;margin-top:8px;line-height:1.55">Explore humanoid designs and guide their hands.<br>Watch the controller and tracking harness respond.</div></div>')
        self.state_card = self.server.gui.add_markdown("**Starting the scripted example…**")
        with self.server.gui.add_folder("Choose a humanoid"):
            labels = [entry["label"] for entry in self.models.values() if entry.get("available")]
            self.model_control = self.server.gui.add_dropdown("Model", labels, initial_value=self.models[self.model_id]["label"])
            label_to_id = {entry["label"]: entry["id"] for entry in self.models.values()}
            self.model_control.on_update(lambda _: self.commands.put(("model", label_to_id[self.model_control.value])))
            self.model_status = self.server.gui.add_markdown("")
            self._make_view_control()
        self.reference_folder = self.server.gui.add_folder("Design reference", visible=False, expand_by_default=False)
        with self.reference_folder:
            self.reference_image = self.server.gui.add_image(np.zeros((1, 1, 3), dtype=np.uint8), label="Generated design turnaround", format="jpeg")
            self.server.gui.add_markdown("Generated concept views guide the constructed 3D approximation. The simulation uses explicit joints and simplified collision geometry.")
        with self.server.gui.add_folder("Try the robot"):
            example = self.server.gui.add_button("Run reaching example", icon=viser.Icon.PLAYER_PLAY, color="blue")
            hold = self.server.gui.add_button("Hold robot", icon=viser.Icon.PLAYER_PAUSE, color="orange")
            reset = self.server.gui.add_button("Reset to home", icon=viser.Icon.ROTATE_CLOCKWISE)
            self.arm_control = self.server.gui.add_dropdown("Arm", ["Right", "Left"], initial_value="Right")
            wave = self.server.gui.add_button("Wave", icon=viser.Icon.HAND_STOP)
            self.manual = self.server.gui.add_checkbox("Drag the orange target", initial_value=False)
            example.on_click(lambda _: self.commands.put(("example", None)))
            hold.on_click(lambda _: self.commands.put(("hold", None)))
            reset.on_click(lambda _: self.commands.put(("reset", None)))
            wave.on_click(lambda _: self.commands.put(("wave", None)))
            self.arm_control.on_update(lambda _: self.commands.put(("arm", self.arm_control.value.lower())))
            self.manual.on_update(lambda _: self.commands.put(("manual", self.manual.value)))
        with self.server.gui.add_folder("Use your camera"):
            self.server.gui.add_markdown("Raise **one open hand**. Move it sideways or up/down to guide the robot hand. Depth stays fixed.")
            self.camera_start = self.server.gui.add_button("Start camera", icon=viser.Icon.CAMERA, color="blue")
            self.camera_stop = self.server.gui.add_button("Stop camera", icon=viser.Icon.CAMERA_OFF, disabled=True)
            self.camera_image = self.server.gui.add_image(camera_placeholder(), label="Local camera preview", format="jpeg")
            self.camera_status = self.server.gui.add_markdown("Camera off · nothing is recorded.")
            self.camera_start.on_click(lambda _: self.commands.put(("camera_start", None)))
            self.camera_stop.on_click(lambda _: self.commands.put(("camera_stop", None)))
        with self.server.gui.add_folder("What is running", expand_by_default=False):
            self.model_notes = self.server.gui.add_markdown("")
            self.server.gui.add_markdown("Drag the scene to orbit · scroll to zoom. Camera frames stay in memory and are sent only to this local viewer.")
        with self.server.gui.add_folder("Session", expand_by_default=False):
            shutdown = self.server.gui.add_button("Shut down demo", icon=viser.Icon.POWER)
            shutdown.on_click(lambda _: self.stop.set())
        self._model_details()

    def _camera_off(self):
        self.camera.stop()
        self.camera_error = None
        self.camera_start.disabled, self.camera_stop.disabled = False, True
        self.camera_image.image = camera_placeholder()

    def _events(self):
        command = None
        while not self.commands.empty():
            kind, value = self.commands.get()
            if kind == "model":
                if value != self.model_id:
                    self._switch_model(value)
                    command = {"mode": "hold", "arm": self.arm}
            elif kind == "view":
                self._set_view(value)
            elif kind == "client_view":
                self.last_client_time = time.monotonic()
                self._set_view(self.current_view, value)
            elif kind == "example":
                self._camera_off()
                self.mode = "demo"
                self.manual.value = False
                command = {"mode": "demo", "arm": self.arm}
            elif kind in ("hold", "camera_stop"):
                self._camera_off()
                self.manual.value = False
                self.gizmo.visible = False
                self.mode = "hold"
                command = {"mode": "hold", "arm": self.arm}
            elif kind == "reset":
                self._camera_off()
                self.sim.reset()
                self.manual.value = False
                self.gizmo.visible = False
                self.mode, self.arm = "hold", "right"
                self.arm_control.value = "Right"
                self.history = []
                self.last_target = np.array(self.workspace["right"]["default"])
                self.pending_target = self.last_target.copy()
                command = {"mode": "hold", "arm": self.arm}
            elif kind == "wave":
                self._camera_off()
                self.mode = "pose"
                self.manual.value = False
                command = {"mode": "pose", "pose": "wave", "arm": self.arm}
            elif kind == "arm":
                if value == self.arm:
                    continue
                self.arm = value
                self.last_target = np.array(self.workspace[self.arm]["default"])
                self.pending_target = self.last_target.copy()
                self.history = []
                if self.mode in ("hold", "camera"):
                    command = {"mode": "hold", "arm": self.arm}
                    self.holding_camera = True
                elif self.mode == "demo":
                    command = {"mode": "demo", "arm": self.arm}
                elif self.mode == "pose":
                    command = {"mode": "pose", "pose": "wave", "arm": self.arm}
                else:
                    command = {"mode": "reach", "arm": self.arm, "target": self.last_target.tolist()}
                self.gizmo.position = self.last_target
            elif kind == "manual":
                self.gizmo.visible = bool(value)
                if value:
                    self._camera_off()
                    self.mode = "reach"
                    state = self.sim.state()
                    target = state["target"] if state["arm"] == self.arm else self.workspace[self.arm]["default"]
                    self.pending_target = np.clip(target, self.workspace[self.arm]["min"], self.workspace[self.arm]["max"])
                    self.last_target = self.pending_target.copy()
                    self.gizmo.position = self.pending_target
                    command = {"mode": "reach", "arm": self.arm, "target": self.pending_target.tolist()}
            elif kind == "target":
                if self.mode == "reach":
                    if value["generation"] != self._scene_generation:
                        continue
                    self.pending_target = np.clip(value["position"], self.workspace[self.arm]["min"], self.workspace[self.arm]["max"])
            elif kind == "camera_start":
                try:
                    self.camera_error = None
                    self.camera.start()
                    self.camera_start.disabled, self.camera_stop.disabled = True, False
                    self.manual.value = False
                    self.mode, self.holding_camera = "camera", True
                    self.last_target = np.clip(self.sim.state()["hand_positions"][self.arm], self.workspace[self.arm]["min"], self.workspace[self.arm]["max"])
                    command = {"mode": "hold", "arm": self.arm}
                except (OSError, RuntimeError, ValueError) as exc:
                    self.camera_error = f"Camera could not start: {exc}"
        return command

    def run(self):
        self.sim.step({"mode": "demo"}, dt=0.04)
        start = last_ui = last_status = time.monotonic()
        frame_count = 0
        status_path = HERE / ".runtime" / "status.json"
        status_path.parent.mkdir(exist_ok=True)
        print(f"HUMANOID_DEMO http://127.0.0.1:{self.args.port} pid={os.getpid()}", flush=True)
        try:
            while not self.stop.is_set():
                tick = time.monotonic()
                command = self._events()
                camera, jpeg, age = self.camera.snapshot()
                if self.server.get_clients():
                    self.last_client_time = tick
                elif tick - self.last_client_time > 5 and camera.get("running"):
                    self._camera_off()
                    self.mode = "hold"
                    command = {"mode": "hold", "arm": self.arm}
                if self.mode == "camera":
                    target = camera_target(camera, age, self.workspace[self.arm]["min"], self.workspace[self.arm]["max"])
                    if target is None:
                        if not self.holding_camera:
                            command = {"mode": "hold", "arm": self.arm}
                        self.holding_camera = True
                    else:
                        self.last_target = slew_target(self.last_target, target)
                        command = {"mode": "reach", "arm": self.arm, "target": self.last_target.tolist()}
                        self.holding_camera = False
                elif self.mode == "reach":
                    self.last_target = slew_target(self.last_target, self.pending_target)
                    command = {"mode": "reach", "arm": self.arm, "target": self.last_target.tolist()}
                state = self.sim.step(command, dt=0.04)
                self.scene.update_from_mjdata(self.sim.data)
                hand = np.asarray(state["hand_position"])
                self.history.append(hand)
                self.history = self.history[-100:]
                if len(self.history) > 1:
                    self.trail.points = np.stack([self.history[:-1], self.history[1:]], axis=1).astype(np.float32)
                frame_count += 1
                if tick - last_ui >= 0.25:
                    last_ui = tick
                    safety = state["safety"]
                    label = "Tracking lost · HOLD" if self.mode == "camera" and self.holding_camera else {"demo": "Scripted reaching", "pose": "Wave example", "camera": "Following your hand", "reach": "Manual target", "hold": "Robot holding"}[self.mode]
                    limit = "Within joint limits" if safety["joint_limit_violation_rad"] < 1e-5 and safety["finite"] and not safety["fault"] else safety["fault"] or "Check joint state"
                    if self.mode == "hold" or (self.mode == "camera" and self.holding_camera):
                        detail = "Position servos holding"
                    elif self.mode == "pose":
                        detail = "Joint-space pose"
                    else:
                        detail = f"**Hand → target:** {state['error_m']*100:.1f} cm"
                    self.state_card.content = f"### {label}\n\n{detail} · **{state['arm'].title()} arm**\n\n{limit} · **{frame_count/max(tick-start,0.01):.0f} Hz** viewer"
                    self.camera_start.disabled = bool(camera.get("running"))
                    self.camera_stop.disabled = not self.camera_start.disabled
                    self.camera_status.content = f"**{camera.get('backend', 'Camera off')}**\n\n{self.camera_error or camera.get('message', '')}\n\nPreview only · no recording"
                    if jpeg and camera.get("running"):
                        self.camera_image.image = np.asarray(Image.open(io.BytesIO(jpeg)).convert("RGB"))
                if tick - last_status >= 1:
                    last_status = tick
                    # Current operational metadata only; never serialize camera JPEGs.
                    metadata = {"pid": os.getpid(), "port": self.args.port, "mode": self.mode, "model_id": self.model_id,
                                "view": self.current_view, "state": state,
                                "camera": {k: camera.get(k) for k in ("running", "tracked", "backend", "message")}, "camera_age_s": age,
                                "harness_hold": self.holding_camera, "wall_s": tick-start, "viewer_hz": frame_count/max(tick-start,0.01)}
                    temporary = status_path.with_suffix(".tmp")
                    temporary.write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n")
                    temporary.replace(status_path)
                if self.args.duration and tick-start >= self.args.duration:
                    break
                self.stop.wait(max(0.0, 0.04 - (time.monotonic() - tick)))
        finally:
            self.camera.stop()
            self.sim.close()
            self.server.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8846)
    parser.add_argument("--model", default="g1", help="registered humanoid model id (default: g1)")
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--camera-python", default=os.environ.get("ROBOLLM_CAMERA_PYTHON", "/mnt/data/RoboLLM-elexhub/.venv/bin/python"))
    parser.add_argument("--hand-model", default=os.environ.get("ROBOLLM_HAND_MODEL", "/mnt/data/RoboLLM-elexhub/.cache/perception/hand_landmarker.task"))
    parser.add_argument("--duration", type=float, default=None, help="optional bounded smoke duration in seconds")
    args = parser.parse_args()
    app = HumanoidApp(args)
    signal.signal(signal.SIGTERM, lambda *_: app.stop.set())
    signal.signal(signal.SIGINT, lambda *_: app.stop.set())
    app.run()


if __name__ == "__main__":
    main()
