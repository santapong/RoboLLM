"""Supported humanoid arm reaching in MuJoCo; classical IK, no learned policy.

Run a bounded headless example:
  MUJOCO_GL=egl python examples/humanoid_harness/simulation.py --output /tmp/g1-demo
Model download is explicit: run fetch_model.py once before constructing a sim.
"""
from __future__ import annotations

import argparse
import copy
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
    from .fetch_model import OFFICIAL_MODELS, REPOSITORY, REVISION, model_assets
except ImportError:
    from fetch_model import OFFICIAL_MODELS, REPOSITORY, REVISION, model_assets

WORKSPACE = {
    "right": {"min": [0.16, -0.50, 0.90], "max": [0.43, -0.13, 1.35], "default": [0.30, -0.26, 1.10]},
    "left": {"min": [0.16, 0.13, 0.90], "max": [0.43, 0.50, 1.35], "default": [0.30, 0.26, 1.10]},
}
ARM_JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
MODELS_DIRECTORY = Path(__file__).resolve().parent / "models"
MODEL_PROFILES = {
    "g1": {"workspace": WORKSPACE, "view_center": [0.03, 0, 0.86], "view_distance": 2.6,
           "description": "Unitree G1 · 29 joints, seven-joint arms; supported pelvis"},
    "h1": {"workspace": {
        "right": {"min": [0.24, -0.62, 1.15], "max": [0.55, -0.18, 1.68], "default": [0.40, -0.32, 1.45]},
        "left": {"min": [0.24, 0.18, 1.15], "max": [0.55, 0.62, 1.68], "default": [0.40, 0.32, 1.45]},
    }, "view_center": [0.03, 0, 1.00], "view_distance": 3.25,
        "description": "Unitree H1 · 19 joints, four-joint arms; supported pelvis"},
}


def list_models() -> list[dict]:
    """List pinned upstream robots and local models/*/model.json concepts.

    A custom descriptor declares id, label, description, source_kind,
    source_revision, workspace, optional reference_image/view_center/view_distance.
    model.xml is robot-only; the common decorator adds the supported scene.
    """
    models = []
    for name, specification in OFFICIAL_MODELS.items():
        directory = model_assets(name)
        models.append({"id": name, "label": specification["label"], **copy.deepcopy(MODEL_PROFILES[name]),
                       "available": (directory / "provenance.json").is_file() and (directory / specification["xml"]).is_file(),
                       "source_kind": "upstream-robot", "source_revision": REVISION,
                       "model_path": str(directory / specification["xml"]), "reference_image": None})
    for path in sorted(MODELS_DIRECTORY.glob("*/model.json")):
        metadata = json.loads(path.read_text())
        if metadata.get("id") != path.parent.name or metadata["id"] in OFFICIAL_MODELS:
            raise ValueError(f"Custom model id must match its directory and not replace an official model: {path}")
        if metadata.get("source_kind") != "original-procedural":
            raise ValueError(f"Custom concepts must identify their estimated procedural provenance: {path}")
        reference = metadata.get("reference_image")
        models.append({**metadata, "available": (path.parent / "model.xml").is_file(),
                       "model_path": str(path.parent / "model.xml"),
                       "reference_image": str((path.parent / reference).resolve()) if reference else None})
    return models


def _workspace(value: dict) -> dict:
    result = copy.deepcopy(value)
    for arm in ("left", "right"):
        try:
            low, high, default = (np.asarray(result[arm][key], dtype=float) for key in ("min", "max", "default"))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("Each arm needs finite min/max/default workspace coordinates") from exc
        if any(item.shape != (3,) for item in (low, high, default)) or not np.isfinite([low, high, default]).all():
            raise ValueError("Workspace coordinates must be finite XYZ vectors")
        if np.any(low >= high) or np.any(default < low) or np.any(default > high):
            raise ValueError("Workspace defaults must lie inside increasing min/max bounds")
        result[arm] = {key: item.tolist() for key, item in zip(("min", "max", "default"), (low, high, default))}
    return result


def _verified_root(asset_dir: Path, model_name: str) -> ET.Element:
    manifest_path = asset_dir / "provenance.json"
    if not manifest_path.exists():
        raise FileNotFoundError(f"Pinned {model_name.upper()} model missing. Run examples/humanoid_harness/fetch_model.py --model {model_name} first.")
    provenance = json.loads(manifest_path.read_text())
    if provenance.get("revision") != REVISION or provenance.get("repository") != REPOSITORY:
        raise ValueError("Model provenance does not match the pinned Menagerie source")
    if provenance.get("model_name", "g1") != model_name:
        raise ValueError("Model assets belong to a different humanoid")
    for item in provenance["files"]:
        source = asset_dir / item["path"]
        if not source.resolve().is_relative_to(asset_dir.resolve()):
            raise ValueError("Invalid model manifest path")
        if hashlib.sha256(source.read_bytes()).hexdigest() != item["sha256"]:
            raise ValueError(f"Model file modified: {item['path']}")
    root = ET.parse(asset_dir / OFFICIAL_MODELS[model_name]["xml"]).getroot()
    root.find("compiler").set("meshdir", str((asset_dir / "assets").resolve()))
    return root


def _model_xml(asset_dir: Path, width: int, height: int, model_name: str = "g1", *,
               custom_model_path: Path | None = None, workspace: dict | None = None) -> str:
    if custom_model_path is None:
        root = _verified_root(asset_dir, model_name)
    else:
        root = ET.parse(custom_model_path).getroot()
        compiler = root.find("compiler")
        if compiler is None:
            compiler = ET.SubElement(root, "compiler", angle="radian")
        for directory in ("meshdir", "texturedir"):
            compiler.set(directory, str((custom_model_path.parent / compiler.get(directory, ".")).resolve()))
        if root.find("include") is not None:
            raise ValueError("Custom humanoid XML must be standalone; expand includes first")
    workspace = workspace or WORKSPACE
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option", integrator="implicitfast")
    option.set("timestep", "0.002")
    world = root.find("worldbody")
    pelvis = world.find("body[@name='pelvis']") if world is not None else None
    if pelvis is None:
        raise ValueError("Humanoid model needs a top-level pelvis body")
    freejoint = pelvis.find("freejoint")
    if freejoint is not None:
        pelvis.remove(freejoint)
    keyframe = root.find("keyframe")
    if keyframe is not None:
        root.remove(keyframe)
    if model_name == "h1" and custom_model_path is None:
        # H1 upstream supplies torque motors. Adapt them to the same position
        # servo contract while retaining each original maximum motor torque.
        joints = {joint.get("name"): joint for joint in root.findall(".//joint[@name]")}
        for actuator in root.find("actuator"):
            force_range = actuator.attrib.pop("ctrlrange")
            actuator.tag = "position"
            actuator.set("kp", "500")
            actuator.set("dampratio", "1")
            actuator.set("ctrlrange", joints[actuator.get("joint")].get("range"))
            actuator.set("forcerange", force_range)
            actuator.set("forcelimited", "true")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth=str(width), offheight=str(height))
    ET.SubElement(visual, "headlight", ambient="0.45 0.45 0.5", diffuse="0.65 0.65 0.65")
    ET.SubElement(visual, "rgba", haze="0.06 0.08 0.13 1")
    assets = root.find("asset")
    if assets is None:
        assets = ET.SubElement(root, "asset")
    ET.SubElement(assets, "texture", name="floor_grid", type="2d", builtin="checker", width="256", height="256", rgb1="0.12 0.16 0.22", rgb2="0.16 0.20 0.26")
    ET.SubElement(assets, "material", name="floor_mat", texture="floor_grid", texrepeat="6 6", reflectance="0.12")
    ET.SubElement(world, "geom", name="floor", type="plane", size="3 3 0.1", material="floor_mat", contype="1", conaffinity="1", group="0")
    ET.SubElement(world, "geom", name="support_base", type="cylinder", pos="-0.15 0 0.025", size="0.20 0.025", rgba="0.1 0.55 0.63 1", contype="0", conaffinity="0")
    pelvis_z = float(pelvis.get("pos", "0 0 0").split()[2])
    stand_top, bracket_z = (0.80, 0.79) if model_name == "g1" else (pelvis_z + 0.01, pelvis_z)
    ET.SubElement(world, "geom", name="pelvis_support", type="capsule", fromto=f"-0.15 0 0.05 -0.15 0 {stand_top}", size="0.032", rgba="0.1 0.55 0.63 1", contype="0", conaffinity="0")
    ET.SubElement(world, "geom", name="pelvis_bracket", type="capsule", fromto=f"-0.15 0 {bracket_z} 0 0 {bracket_z}", size="0.022", rgba="0.1 0.55 0.63 1", contype="0", conaffinity="0")
    target = ET.SubElement(world, "body", name="reach_target", mocap="true", pos=" ".join(map(str, workspace["right"]["default"])))
    ET.SubElement(target, "geom", name="target_sphere", type="sphere", size="0.026", rgba="1 0.42 0.12 0.85", contype="0", conaffinity="0")
    for arm in workspace:
        if root.find(f".//site[@name='{arm}_hand_tip']") is not None:
            continue
        link, position = ("elbow", "0.28 0 -0.015") if model_name == "h1" else ("wrist_yaw", "0.105 0 0")
        hand_body = root.find(f".//body[@name='{arm}_{link}_link']")
        if hand_body is None:
            raise ValueError(f"Custom humanoid needs {arm}_hand_tip site or {arm}_{link}_link body")
        ET.SubElement(hand_body, "site", name=f"{arm}_hand_tip", pos=position, size="0.012", rgba="0.15 0.9 0.7 1", group="0")
    return ET.tostring(root, encoding="unicode")


class HumanoidSimulation:
    """Single-thread-owned MuJoCo simulator; expose model/data to local viewers.

    Commands: {mode: reach, arm: right|left, target: [x,y,z]},
    {mode: pose, pose: home|wave|arms_up, arm: right|left},
    {mode: demo}, or {mode: hold}. Positions are world-frame metres (x forward,
    y robot-left, z up). step(None) retains the previous command. The workspace
    box is a UI limit, not a guarantee that every target is reachable.
    """

    def __init__(self, asset_dir: str | Path | None = None, width: int = 960, height: int = 720,
                 model_name: str = "g1", *, custom_model_path: str | Path | None = None,
                 custom_metadata: dict | None = None):
        if custom_model_path is not None:
            custom_model_path = Path(custom_model_path).resolve()
            metadata = {"id": "custom", "label": "Original custom humanoid", "source_kind": "original-procedural",
                        "workspace": WORKSPACE, **(custom_metadata or {})}
            if metadata["id"] in OFFICIAL_MODELS or metadata["source_kind"] != "original-procedural":
                raise ValueError("Custom humanoids cannot claim an upstream robot's identity")
            model_name = metadata["id"]
        else:
            metadata = next((item for item in list_models() if item["id"] == model_name), None)
            if metadata is None:
                raise ValueError(f"Unknown humanoid model: {model_name}")
            if model_name not in OFFICIAL_MODELS:
                custom_model_path = Path(metadata["model_path"]).resolve()
        self.model_id, self.model_label = model_name, metadata["label"]
        self.source_kind = metadata["source_kind"]
        self.source_revision = metadata.get("source_revision", "local-custom")
        self.workspace = _workspace(metadata.get("workspace", WORKSPACE))
        self.view_center = list(metadata.get("view_center", [0.03, 0, 0.86]))
        self.view_distance = float(metadata.get("view_distance", 2.6))
        if np.asarray(self.view_center).shape != (3,) or not np.isfinite(self.view_center).all() or not math.isfinite(self.view_distance) or self.view_distance <= 0:
            raise ValueError("Model camera needs a finite XYZ center and positive distance")
        self.asset_dir = Path(asset_dir) if asset_dir else (custom_model_path.parent if custom_model_path else model_assets(model_name))
        self.width, self.height = int(width), int(height)
        model_xml = _model_xml(self.asset_dir, self.width, self.height, model_name,
                               custom_model_path=custom_model_path, workspace=self.workspace)
        self.model_sha256 = hashlib.sha256(model_xml.encode()).hexdigest()
        self.model = mujoco.MjModel.from_xml_string(model_xml)
        if self.model.nq != self.model.nv or self.model.nq != self.model.njnt or self.model.nu != self.model.njnt:
            raise ValueError("Supported humanoids require a fixed pelvis, one hinge DOF per joint, and one position actuator per joint")
        if not np.all(self.model.jnt_type == mujoco.mjtJoint.mjJNT_HINGE) or not np.all(self.model.jnt_limited):
            raise ValueError("Supported humanoids require finite limited hinge joints")
        if not np.all(np.isfinite(self.model.jnt_range)) or np.any(np.diff(self.model.jnt_range, axis=1) <= 0.05):
            raise ValueError("Joint ranges must be finite and leave room for a 0.025-radian safety margin")
        transmissions = self.model.actuator_trnid[:, 0]
        if not np.array_equal(np.sort(transmissions), np.arange(self.model.njnt)):
            raise ValueError("Every joint must have exactly one actuator")
        if not np.all(self.model.actuator_gainprm[:, 0] > 0) or not np.allclose(self.model.actuator_biasprm[:, 1], -self.model.actuator_gainprm[:, 0]):
            raise ValueError("The reaching controller requires position servo actuators")
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
        arm_parts, suffix = (ARM_JOINTS[:4], "") if model_name == "h1" else (ARM_JOINTS, "_joint")
        try:
            self._arms = {arm: np.array([self._joint_names.index(f"{arm}_{part}{suffix}") for part in arm_parts]) for arm in self.workspace}
        except ValueError as exc:
            raise ValueError("Custom humanoid needs G1-style shoulder/elbow/wrist joint names") from exc
        self._sites = {arm: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"{arm}_hand_tip") for arm in self.workspace}
        self._target_mocap = self.model.body_mocapid[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "reach_target")]
        self._home = np.zeros(self.model.nq)
        for arm, sign in (("left", 1), ("right", -1)):
            self._home[self._qadr[self._arms[arm][0]]] = -0.10
            self._home[self._qadr[self._arms[arm][1]]] = sign * 0.20
            self._home[self._qadr[self._arms[arm][3]]] = 0.60
        for name, value in metadata.get("home", {}).items():
            if name not in self._joint_names or not math.isfinite(value):
                raise ValueError("Home pose must reference existing joints with finite angles")
            self._home[self._qadr[self._joint_names.index(name)]] = value
        if np.any(self._home[self._qadr] < self._safe_lo) or np.any(self._home[self._qadr] > self._safe_hi):
            raise ValueError("Home pose must stay inside joint safety margins")
        self.max_joint_speed = 1.1
        self.reset()

    def reset(self) -> dict:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self._home
        self._goal = self._home.copy()
        self.data.ctrl[:] = self._goal[self._qadr[self._act_joint]]
        self.arm, self.mode, self.pose = "right", "hold", "home"
        self.target = np.array(self.workspace[self.arm]["default"], dtype=float)
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
        if arm not in self.workspace:
            raise ValueError("arm must be right or left")
        if pose not in ("home", "wave", "arms_up"):
            raise ValueError("Unknown pose")
        target = np.asarray(command.get("target", self.target if arm == self.arm else self.workspace[arm]["default"]), dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("Target must contain three finite metre coordinates")
        bounded = np.clip(target, self.workspace[arm]["min"], self.workspace[arm]["max"])
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
            if len(ids) > 6:
                self._goal[self._qadr[ids[6]]] = 0.35 * math.sin(self.data.time * 3)
            else:
                self._goal[self._qadr[ids[2]]] += 0.22 * math.sin(self.data.time * 3)
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
            default = self.workspace[self.arm]["default"]
            self.target = np.array([default[0] + 0.04 * math.sin(phase), sign * (abs(default[1]) + 0.02 + 0.065 * math.sin(phase * 0.8)), default[2] + 0.02 + 0.11 * math.sin(phase * 0.65)])
            self.target = np.clip(self.target, self.workspace[self.arm]["min"], self.workspace[self.arm]["max"])
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
            "model": f"{self.model_label} ({self.model.nu} actuated joints)", "model_id": self.model_id,
            "source_kind": self.source_kind, "source_revision": self.source_revision, "model_sha256": self.model_sha256,
            "scope": "MuJoCo simulation; supported pelvis; classical IK and position control; no learned VLM/WAM or walking",
            "time": float(self.data.time), "mode": self.mode, "arm": self.arm, "pose": self.pose,
            "target": self.target.tolist(), "hand_position": hand.tolist(), "hand_positions": {arm: self.data.site_xpos[site].tolist() for arm, site in self._sites.items()},
            "error_m": error, "reached": bool(error < 0.035 and self.mode in ("reach", "demo")),
            "joints": {name: float(value) for name, value in zip(self._joint_names, q)},
            "workspace": self.workspace, "max_joint_speed_rad_s": self.max_joint_speed,
            "safety": {"fixed_base": True, "target_clamped": self._target_clamped, "joint_limit_violation_rad": max(0.0, float(violation)), "finite": bool(np.all(np.isfinite(q))), "fault": self._fault},
        }

    def render(self) -> np.ndarray:
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = self.view_center
        camera.distance, camera.azimuth, camera.elevation = self.view_distance, 145, -13
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


def run_example(output: Path, model_name: str = "g1") -> dict:
    output.mkdir(parents=True, exist_ok=True)
    sim = HumanoidSimulation(width=960, height=720, model_name=model_name)
    started = time.monotonic()
    results = []
    try:
        high_target = np.clip(np.asarray(sim.workspace["right"]["default"]) + [0.04, -0.06, 0.12],
                              sim.workspace["right"]["min"], sim.workspace["right"]["max"]).tolist()
        targets = (("right", sim.workspace["right"]["default"]), ("right", high_target), ("left", sim.workspace["left"]["default"]))
        for arm, target in targets:
            sim.reset()
            initial = sim.step({"mode": "reach", "arm": arm, "target": target})
            for _ in range(124):
                final = sim.step()
            results.append({"arm": arm, "target": target, "initial_error_m": initial["error_m"], "final_error_m": final["error_m"], "reached": final["reached"], "safety": final["safety"]})
            Image.fromarray(sim.render()).save(output / f"reach-{len(results)}.png")
        report = {"experiment": f"supported {sim.model_label} scripted reaching; no learned policy", "model_id": sim.model_id,
                  "source_kind": sim.source_kind, "model_revision": sim.source_revision, "model_sha256": sim.model_sha256,
                  "mujoco_version": mujoco.__version__, "numpy_version": np.__version__, "timestep": sim.model.opt.timestep,
                  "controller": "position-only damped-least-squares IK, rate-limited position servos", "wall_seconds": time.monotonic() - started, "episodes": results}
        (output / "simulation-results.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        return report
    finally:
        sim.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "artifacts")
    parser.add_argument("--model", default="g1", help="registered humanoid id (g1, h1, or local custom concept)")
    args = parser.parse_args()
    run_example(args.output, model_name=args.model)
