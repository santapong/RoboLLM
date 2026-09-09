"""Interactive local Unitree G1 experiment: MuJoCo + Viser + optional webcam."""

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
from simulation import WORKSPACE, HumanoidSimulation

HERE = Path(__file__).resolve().parent


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
        self.pending_target = np.array(WORKSPACE[self.arm]["default"])
        self.last_target = self.pending_target.copy()
        self.holding_camera = False
        self.camera_error = None
        self.last_client_time = time.monotonic()
        self.camera = CameraBridge(args.camera_python, args.camera, args.hand_model)
        self.sim = HumanoidSimulation()
        self.server = viser.ViserServer(host="127.0.0.1", port=args.port, label="RoboLLM · Humanoid Lab")
        self.server.gui.configure_theme(dark_mode=True, control_width="large", show_logo=False,
                                        show_share_button=False, brand_color=(97, 169, 245))
        self.server.gui.main_panel.dock_right()
        self.server.scene.set_up_direction("+z")
        self.scene = ViserMujocoScene(self.server, self.sim.model, num_envs=1)
        self.scene.camera_tracking_enabled = False
        self.scene.update_from_mjdata(self.sim.data)
        self.gizmo = self.server.scene.add_transform_controls("/target-control", scale=0.15, disable_rotations=True,
                                                            position=self.last_target, visible=False)
        self.trail = self.server.scene.add_line_segments("/hand-trail", np.zeros((1, 2, 3), dtype=np.float32), (84, 219, 187), thickness=3, thickness_units="screen")
        self.history = []
        self._make_controls()

        @self.server.on_client_connect
        def connected(client):
            client.camera.position = (1.50, -1.50, 1.30)
            client.camera.look_at = (0.05, 0.0, 0.85)
            client.camera.up_direction = (0, 0, 1)
            client.camera.fov = 0.80
            self.last_client_time = time.monotonic()

        @self.gizmo.on_update
        def dragged(_):
            self.commands.put(("target", list(self.gizmo.position)))

    def _make_controls(self):
        self.server.gui.add_html('<div style="padding:4px 0 12px"><div style="font-size:11px;letter-spacing:2px;color:#9cacbf">ROBOTICS WORKBENCH</div><div style="font-size:27px;font-weight:700;margin-top:7px">Humanoid Lab</div><div style="color:#c0ccdc;margin-top:8px;line-height:1.55">Move a simulated G1 hand toward a target.<br>Watch the controller and tracking harness respond.</div></div>')
        self.state_card = self.server.gui.add_markdown("**Starting the scripted example…**")
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
            self.server.gui.add_markdown("**Perception:** local hand landmarks.\n\n**Controller:** inverse kinematics and position servos.\n\n**Harness:** tracking freshness, hold, workspace and target-speed limits.\n\n**Simulation:** MuJoCo physics, 29-joint G1 with a supported pelvis.\n\nThis example demonstrates upper-body control. Walking and learned VLM/WAM control are separate experiments.")
            self.server.gui.add_markdown("Drag the scene to orbit · scroll to zoom. Camera frames stay in memory and are sent only to this local viewer.")
        with self.server.gui.add_folder("Session", expand_by_default=False):
            shutdown = self.server.gui.add_button("Shut down demo", icon=viser.Icon.POWER)
            shutdown.on_click(lambda _: self.stop.set())

    def _camera_off(self):
        self.camera.stop()
        self.camera_error = None
        self.camera_start.disabled, self.camera_stop.disabled = False, True
        self.camera_image.image = camera_placeholder()

    def _events(self):
        command = None
        while not self.commands.empty():
            kind, value = self.commands.get()
            if kind == "example":
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
                self.last_target = np.array(WORKSPACE["right"]["default"])
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
                self.last_target = np.array(WORKSPACE[self.arm]["default"])
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
                    target = state["target"] if state["arm"] == self.arm else WORKSPACE[self.arm]["default"]
                    self.pending_target = np.clip(target, WORKSPACE[self.arm]["min"], WORKSPACE[self.arm]["max"])
                    self.last_target = self.pending_target.copy()
                    self.gizmo.position = self.pending_target
                    command = {"mode": "reach", "arm": self.arm, "target": self.pending_target.tolist()}
            elif kind == "target":
                if self.mode == "reach":
                    self.pending_target = np.clip(value, WORKSPACE[self.arm]["min"], WORKSPACE[self.arm]["max"])
            elif kind == "camera_start":
                try:
                    self.camera_error = None
                    self.camera.start()
                    self.camera_start.disabled, self.camera_stop.disabled = True, False
                    self.manual.value = False
                    self.mode, self.holding_camera = "camera", True
                    self.last_target = np.clip(self.sim.state()["hand_positions"][self.arm], WORKSPACE[self.arm]["min"], WORKSPACE[self.arm]["max"])
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
                    target = camera_target(camera, age, WORKSPACE[self.arm]["min"], WORKSPACE[self.arm]["max"])
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
                    metadata = {"pid": os.getpid(), "port": self.args.port, "mode": self.mode, "state": state,
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
