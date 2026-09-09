"""Supported Unitree G1 arm reaching in MuJoCo; classical IK, no learned policy.

Run a bounded headless example:
  MUJOCO_GL=egl python examples/humanoid_harness/simulation.py --output /tmp/g1-demo
Model download is explicit: run fetch_model.py once before constructing a sim.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
import numpy as np
from PIL import Image

try:
    from .fetch_model import DEFAULT_ASSETS, REPOSITORY, REVISION
except ImportError:
    from fetch_model import DEFAULT_ASSETS, REPOSITORY, REVISION

WORKSPACE = {
    "right": {"min": [0.16, -0.50, 0.90], "max": [0.43, -0.13, 1.35], "default": [0.30, -0.26, 1.10]},
    "left": {"min": [0.16, 0.13, 0.90], "max": [0.43, 0.50, 1.35], "default": [0.30, 0.26, 1.10]},
}
ARM_JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")


def _model_xml(asset_dir: Path, width: int, height: int) -> str:
    manifest_path = asset_dir / "provenance.json"
    if not manifest_path.exists():
        raise FileNotFoundError("Pinned G1 model missing. Run examples/humanoid_harness/fetch_model.py first.")
    provenance = json.loads(manifest_path.read_text())
    if provenance.get("revision") != REVISION or provenance.get("repository") != REPOSITORY:
        raise ValueError("Model provenance does not match the pinned Menagerie source")
    for item in provenance["files"]:
        source = asset_dir / item["path"]
        if not source.resolve().is_relative_to(asset_dir.resolve()):
            raise ValueError("Invalid model manifest path")
        if hashlib.sha256(source.read_bytes()).hexdigest() != item["sha256"]:
            raise ValueError(f"Model file modified: {item['path']}")
    root = ET.parse(asset_dir / "g1.xml").getroot()
    root.find("compiler").set("meshdir", str((asset_dir / "assets").resolve()))
    root.find("option").set("timestep", "0.002")
    world = root.find("worldbody")
    pelvis = world.find("body[@name='pelvis']")
    pelvis.remove(pelvis.find("freejoint"))
    root.remove(root.find("keyframe"))
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth=str(width), offheight=str(height))
    ET.SubElement(visual, "headlight", ambient="0.45 0.45 0.5", diffuse="0.65 0.65 0.65")
    ET.SubElement(visual, "rgba", haze="0.06 0.08 0.13 1")
    assets = root.find("asset")
    ET.SubElement(assets, "texture", name="floor_grid", type="2d", builtin="checker", width="256", height="256", rgb1="0.12 0.16 0.22", rgb2="0.16 0.20 0.26")
    ET.SubElement(assets, "material", name="floor_mat", texture="floor_grid", texrepeat="6 6", reflectance="0.12")
    ET.SubElement(world, "geom", name="floor", type="plane", size="3 3 0.1", material="floor_mat")
    ET.SubElement(world, "geom", name="support_base", type="cylinder", pos="-0.15 0 0.025", size="0.20 0.025", rgba="0.1 0.55 0.63 1", contype="0", conaffinity="0")
    ET.SubElement(world, "geom", name="pelvis_support", type="capsule", fromto="-0.15 0 0.05 -0.15 0 0.80", size="0.032", rgba="0.1 0.55 0.63 1", contype="0", conaffinity="0")
    ET.SubElement(world, "geom", name="pelvis_bracket", type="capsule", fromto="-0.15 0 0.79 0 0 0.79", size="0.022", rgba="0.1 0.55 0.63 1", contype="0", conaffinity="0")
    target = ET.SubElement(world, "body", name="reach_target", mocap="true", pos="0.30 -0.26 1.10")
    ET.SubElement(target, "geom", name="target_sphere", type="sphere", size="0.026", rgba="1 0.42 0.12 0.85", contype="0", conaffinity="0")
    for arm in WORKSPACE:
        wrist = root.find(f".//body[@name='{arm}_wrist_yaw_link']")
        ET.SubElement(wrist, "site", name=f"{arm}_hand_tip", pos="0.105 0 0", size="0.012", rgba="0.15 0.9 0.7 1", group="0")
    return ET.tostring(root, encoding="unicode")


class HumanoidSimulation:
    """Single-thread-owned MuJoCo simulator; expose model/data to local viewers.

    Commands: {mode: reach, arm: right|left, target: [x,y,z]},
    {mode: pose, pose: home|wave|arms_up, arm: right|left},
    {mode: demo}, or {mode: hold}. Positions are world-frame metres (x forward,
    y robot-left, z up). step(None) retains the previous command. The workspace
    box is a UI limit, not a guarantee that every target is reachable.
    """

    def __init__(self, asset_dir: str | Path | None = None, width: int = 960, height: int = 720):
        self.asset_dir = Path(asset_dir) if asset_dir else DEFAULT_ASSETS
        self.width, self.height = int(width), int(height)
        self.model = mujoco.MjModel.from_xml_string(_model_xml(self.asset_dir, self.width, self.height))
        self.data = mujoco.MjData(self.model)
        self._ik_data = mujoco.MjData(self.model)
        self._renderer = None
        self._joint_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(self.model.njnt)]
        self._qadr = self.model.jnt_qposadr.copy()
        self._dadr = self.model.jnt_dofadr.copy()
        self._limits = self.model.jnt_range.copy()
        self._safe_lo = self._limits[:, 0] + 0.025
        self._safe_hi = self._limits[:, 1] - 0.025
        self._act_joint = self.model.actuator_trnid[:, 0]
        self._arms = {arm: np.array([self._joint_names.index(f"{arm}_{part}_joint") for part in ARM_JOINTS]) for arm in WORKSPACE}
        self._sites = {arm: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"{arm}_hand_tip") for arm in WORKSPACE}
        self._target_mocap = self.model.body_mocapid[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "reach_target")]
        self._home = np.zeros(self.model.nq)
        for arm, sign in (("left", 1), ("right", -1)):
            self._home[self._qadr[self._arms[arm][0]]] = -0.10
            self._home[self._qadr[self._arms[arm][1]]] = sign * 0.20
            self._home[self._qadr[self._arms[arm][3]]] = 0.60
        self.max_joint_speed = 1.1
        self.reset()

    def reset(self) -> dict:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self._home
        self._goal = self._home.copy()
        self.data.ctrl[:] = self._goal[self._qadr[self._act_joint]]
        self.arm, self.mode, self.pose = "right", "hold", "home"
        self.target = np.array(WORKSPACE[self.arm]["default"], dtype=float)
        self._target_clamped = False
        self._fault = None
        self._update_marker()
        mujoco.mj_forward(self.model, self.data)
        return self.state()

    def _update_marker(self):
        self.data.mocap_pos[self._target_mocap] = self.target

    def _accept(self, command: dict):
        if not isinstance(command, dict):
            raise TypeError("Command must be an object")
        mode = command.get("mode", self.mode)
        arm = command.get("arm", self.arm)
        pose = command.get("pose", self.pose)
        if mode not in ("reach", "pose", "demo", "hold"):
            raise ValueError("Unknown control mode")
        if arm not in WORKSPACE:
            raise ValueError("arm must be right or left")
        if pose not in ("home", "wave", "arms_up"):
            raise ValueError("Unknown pose")
        target = np.asarray(command.get("target", self.target if arm == self.arm else WORKSPACE[arm]["default"]), dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("Target must contain three finite metre coordinates")
        bounded = np.clip(target, WORKSPACE[arm]["min"], WORKSPACE[arm]["max"])
        self._target_clamped = bool(np.any(bounded != target))
        self.target, self.arm, self.pose = bounded, arm, pose
        if mode == "hold" and self.mode != "hold":
            self._goal = self.data.qpos.copy()
        self.mode = mode

    def _ik(self):
        """Damped least-squares position-only IK on a separate kinematic state."""
        joint_ids = self._arms[self.arm]
        qa, da = self._qadr[joint_ids], self._dadr[joint_ids]
        self._ik_data.qpos[:] = self._goal
        jac = np.zeros((3, self.model.nv))
        for _ in range(16):
            mujoco.mj_kinematics(self.model, self._ik_data)
            mujoco.mj_comPos(self.model, self._ik_data)
            error = self.target - self._ik_data.site_xpos[self._sites[self.arm]]
            if np.linalg.norm(error) < 0.0005:
                break
            mujoco.mj_jacSite(self.model, self._ik_data, jac, None, self._sites[self.arm])
            local = jac[:, da]
            delta = local.T @ np.linalg.solve(local @ local.T + 0.015 ** 2 * np.eye(3), error)
            self._ik_data.qpos[qa] = np.clip(self._ik_data.qpos[qa] + np.clip(delta, -0.18, 0.18), self._safe_lo[joint_ids], self._safe_hi[joint_ids])
        self._goal[qa] = self._ik_data.qpos[qa]

    def _pose_goal(self):
        self._goal[:] = self._home
        if self.pose == "wave":
            ids = self._arms[self.arm]
            sign = 1 if self.arm == "left" else -1
            self._goal[self._qadr[ids[:4]]] = [-0.85, sign * 0.65, sign * -0.25, -0.70]
            self._goal[self._qadr[ids[6]]] = 0.35 * math.sin(self.data.time * 3)
        elif self.pose == "arms_up":
            for arm, ids in self._arms.items():
                sign = 1 if arm == "left" else -1
                self._goal[self._qadr[ids[:4]]] = [-1.85, sign * 0.45, 0, 0.15]

    def step(self, command: dict | None = None, dt: float = 0.04) -> dict:
        if not math.isfinite(dt) or not 0 < dt <= 0.2:
            raise ValueError("dt must be finite and in (0, 0.2] seconds")
        if command is not None:
            self._accept(command)
        if self.mode == "demo":
            sign = 1 if self.arm == "left" else -1
            phase = self.data.time * 0.6
            self.target = np.array([0.30 + 0.04 * math.sin(phase), sign * (0.28 + 0.065 * math.sin(phase * 0.8)), 1.12 + 0.11 * math.sin(phase * 0.65)])
            self._ik()
        elif self.mode == "reach":
            self._ik()
        elif self.mode == "pose":
            self._pose_goal()
        self._goal[self._qadr] = np.clip(self._goal[self._qadr], self._safe_lo, self._safe_hi)
        self._update_marker()
        desired = self._goal[self._qadr[self._act_joint]]
        max_delta = self.max_joint_speed * self.model.opt.timestep
        for _ in range(max(1, round(dt / self.model.opt.timestep))):
            self.data.ctrl[:] += np.clip(desired - self.data.ctrl, -max_delta, max_delta)
            mujoco.mj_step(self.model, self.data)
            if not np.all(np.isfinite(self.data.qpos)) or np.max(np.abs(self.data.qvel)) > 100:
                self.reset()
                self._fault = "Numerical instability: simulation reset and held"
                break
        mujoco.mj_forward(self.model, self.data)
        return self.state()

    def state(self) -> dict:
        hand = self.data.site_xpos[self._sites[self.arm]].copy()
        q = self.data.qpos[self._qadr]
        violation = np.maximum(self._limits[:, 0] - q, q - self._limits[:, 1]).max()
        error = float(np.linalg.norm(hand - self.target))
        return {
            "model": "Unitree G1 (29 actuated joints)", "source_revision": REVISION,
            "scope": "MuJoCo simulation; supported pelvis; classical IK and position control; no learned VLM/WAM or walking",
            "time": float(self.data.time), "mode": self.mode, "arm": self.arm, "pose": self.pose,
            "target": self.target.tolist(), "hand_position": hand.tolist(), "hand_positions": {arm: self.data.site_xpos[site].tolist() for arm, site in self._sites.items()},
            "error_m": error, "reached": bool(error < 0.035 and self.mode in ("reach", "demo")),
            "joints": {name: float(value) for name, value in zip(self._joint_names, q)},
            "workspace": WORKSPACE, "max_joint_speed_rad_s": self.max_joint_speed,
            "safety": {"fixed_base": True, "target_clamped": self._target_clamped, "joint_limit_violation_rad": max(0.0, float(violation)), "finite": bool(np.all(np.isfinite(q))), "fault": self._fault},
        }

    def render(self) -> np.ndarray:
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [0.03, 0, 0.86]
        camera.distance, camera.azimuth, camera.elevation = 2.6, 145, -13
        options = mujoco.MjvOption()
        options.geomgroup[3] = 0
        self._renderer.update_scene(self.data, camera=camera, scene_option=options)
        return self._renderer.render().copy()

    def render_jpeg(self) -> bytes:
        stream = io.BytesIO()
        Image.fromarray(self.render()).save(stream, "JPEG", quality=88)
        return stream.getvalue()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


def run_example(output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    sim = HumanoidSimulation(width=960, height=720)
    started = time.monotonic()
    results = []
    try:
        for arm, target in (("right", [0.30, -0.26, 1.10]), ("right", [0.34, -0.32, 1.22]), ("left", [0.30, 0.26, 1.10])):
            sim.reset()
            initial = sim.step({"mode": "reach", "arm": arm, "target": target})
            for _ in range(124):
                final = sim.step()
            results.append({"arm": arm, "target": target, "initial_error_m": initial["error_m"], "final_error_m": final["error_m"], "reached": final["reached"], "safety": final["safety"]})
            Image.fromarray(sim.render()).save(output / f"reach-{len(results)}.png")
        report = {"experiment": "supported G1 scripted reaching; no learned policy", "model_revision": REVISION, "mujoco_version": mujoco.__version__, "numpy_version": np.__version__, "timestep": sim.model.opt.timestep, "controller": "position-only damped-least-squares IK, rate-limited position servos", "wall_seconds": time.monotonic() - started, "episodes": results}
        (output / "simulation-results.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        return report
    finally:
        sim.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "artifacts")
    run_example(parser.parse_args().output)
