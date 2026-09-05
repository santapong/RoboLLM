"""The bed environment: UR5e + 2F-85, red target, EE-delta actions, B1's observation contract.

Observation keys are exactly B1's three (`examples/mujoco/reaching.py`):
`observation.images.front` (224×224×3), `observation.state` (14: EE xyz, EE quat
wxyz, gripper opening, six joint angles), `observation.camera_lag_ms` (1). The
target never enters the observation; the camera is the only goal input.

Every action passes the safety wrapper (S1–S5) before execution; S6–S7 are
measured after. Success = EE within 3 cm of the target for 5 consecutive frames
(B1's constants). Progress ∈ {0, 0.5, 1}: 0.5 if the EE ever came within 2×
threshold (rule R8).
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from expert import MinkController  # noqa: E402
from families import EpisodeSpec, relocated_target  # noqa: E402
from safety import Decision, EpisodeSafety, SafetyWrapper, robot_geom_ids, self_collision_measurable  # noqa: E402
from scene import build_scene  # noqa: E402

FPS = 20
MAX_FRAMES = 100
SUCCESS_DISTANCE_M = 0.03
SUCCESS_FRAMES = 5
PROGRESS_HALF_DISTANCE_M = 2 * SUCCESS_DISTANCE_M
INSTRUCTION = "touch the red target"
VARIATIONS = ("nominal", "camera_shift", "camera_shift_far", "lighting", "target_relocation")
CAMERA_SHIFT_M = np.array([0.15, 0.10, 0.0])  # the camera_shift variation: translate, do not re-aim
CAMERA_SHIFT_FAR_M = np.array([0.30, 0.20, 0.0])  # twice as far: outside any ±0.2 m training jitter
STATE_NAMES = ["ee_x", "ee_y", "ee_z", "ee_qw", "ee_qx", "ee_qy", "ee_qz", "gripper", "q1", "q2", "q3", "q4", "q5", "q6"]
ACTION_NAMES = ["dx", "dy", "dz", "droll", "dpitch", "dyaw", "gripper"]
GRIPPER_CTRL_MAX = 255.0


class SuccessDetector:
    def __init__(self, threshold_m: float = SUCCESS_DISTANCE_M, consecutive_frames: int = SUCCESS_FRAMES):
        self.threshold_m, self.consecutive_frames = threshold_m, consecutive_frames
        self.streak = 0

    def reset(self) -> None:
        self.streak = 0

    def update(self, error_m: float) -> bool:
        self.streak = self.streak + 1 if error_m <= self.threshold_m else 0
        return self.streak >= self.consecutive_frames


@dataclass
class StepResult:
    observation: dict[str, np.ndarray]
    error_m: float
    success: bool
    progress: float
    decision: Decision
    executed: np.ndarray


def dataset_features(height: int = 224, width: int = 224) -> dict[str, dict]:
    return {
        "observation.images.front": {"dtype": "video", "shape": (height, width, 3), "names": ["height", "width", "channels"]},
        "observation.state": {"dtype": "float32", "shape": (14,), "names": STATE_NAMES},
        "action": {"dtype": "float32", "shape": (7,), "names": ACTION_NAMES},
        "action.executed": {"dtype": "float32", "shape": (7,), "names": ACTION_NAMES},
        "observation.noise_sigma": {"dtype": "float32", "shape": (1,), "names": ["noise_sigma"]},
        "observation.camera_lag_ms": {"dtype": "float32", "shape": (1,), "names": ["camera_lag_ms"]},
    }


class BedEnv:
    def __init__(self, render: bool = True, height: int = 224, width: int = 224):
        self.model = build_scene.load_model()
        self.data = mujoco.MjData(self.model)
        physics_hz = 1.0 / self.model.opt.timestep
        self.substeps = int(round(physics_hz / FPS))
        self.render_enabled = render
        self.height, self.width = height, width
        self.renderer = mujoco.Renderer(self.model, height=height, width=width) if render else None
        self.camera = build_scene.CAMERA_NAME
        self.cam_id = self.model.camera(self.camera).id
        self._cam_pos0 = self.model.cam_pos[self.cam_id].copy()
        self._cam_quat0 = self.model.cam_quat[self.cam_id].copy()
        self.cam_lookat = np.asarray(build_scene.CAMERA_LOOKAT, dtype=float).copy()
        self._light_diffuse0 = self.model.light_diffuse.copy()
        self.controller = MinkController(self.model)
        self.wrapper = SafetyWrapper()
        self.robot_geoms = robot_geom_ids(self.model)
        self.floor_geom = self.model.geom("floor").id
        self.s6_measurable = self_collision_measurable(self.model, self.robot_geoms)
        self.target_mocap = self.model.body(build_scene.TARGET_NAME).mocapid[0]
        self.ee_site = self.model.site(build_scene.EE_SITE).id
        self.arm_qpos = self.controller.arm_qpos
        self.arm_dof = self.controller.arm_dof
        self.arm_act = np.array([self.model.actuator(a).id for a in build_scene.ARM_ACTUATORS])
        self.grip_act = self.model.actuator(build_scene.GRIPPER_ACTUATOR).id
        self.detector = SuccessDetector()
        self.spec: EpisodeSpec | None = None
        self.safety = EpisodeSafety()
        self.min_error_m = np.inf
        self.frame = 0
        self.gripper_cmd = 0.0

    # ----- state helpers -----
    @property
    def end_effector(self) -> np.ndarray:
        return self.data.site_xpos[self.ee_site].copy()

    @property
    def ee_rot(self) -> np.ndarray:
        return self.data.site_xmat[self.ee_site].reshape(3, 3).copy()

    @property
    def commanded_ee(self) -> tuple[np.ndarray, np.ndarray]:
        """(pos, rot) of the controller's commanded pose; the pose deltas integrate on."""
        return self.controller.ee_pose()

    @property
    def target(self) -> np.ndarray:
        return self.data.mocap_pos[self.target_mocap].copy()

    @property
    def error_m(self) -> float:
        return float(np.linalg.norm(self.target - self.end_effector))

    def _gripper_opening(self) -> float:
        return float(np.clip(self.data.ctrl[self.grip_act] / GRIPPER_CTRL_MAX, 0.0, 1.0))

    def state(self) -> np.ndarray:
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, self.ee_rot.reshape(-1))
        return np.concatenate([self.end_effector, quat, [self._gripper_opening()], self.data.qpos[self.arm_qpos]]).astype(np.float32)

    def observation(self, render: bool = True) -> dict[str, np.ndarray]:
        """B1's observation dict. `render=False` skips the camera (image None) — the evaluator asks for a
        frame only when the policy is about to be queried; success and error are state-based."""
        if not render:
            image = None
        elif self.renderer is not None:
            self.renderer.update_scene(self.data, camera=self.camera)
            image = self.renderer.render().copy()
        else:
            image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        return {
            "observation.images.front": image,
            "observation.state": self.state(),
            "observation.camera_lag_ms": np.zeros(1, dtype=np.float32),
        }

    def set_camera_azimuth(self, azimuth_deg: float) -> None:
        """Rotate the fixed camera about the world z axis through the scene's look-at point (distance and
        elevation kept) and re-aim it at that point; 0 restores the nominal pose. Recording-time
        viewpoint diversity (recipe v3, ±20°, Cai et al. 2603.26757); the frozen suite stays at 0."""
        if abs(float(azimuth_deg)) < 1e-12:
            self.model.cam_pos[self.cam_id] = self._cam_pos0
            self.model.cam_quat[self.cam_id] = self._cam_quat0
            return
        a = np.deg2rad(float(azimuth_deg))
        c, s = np.cos(a), np.sin(a)
        rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        pos = self.cam_lookat + rz @ (self._cam_pos0 - self.cam_lookat)
        xy = build_scene._lookat_xyaxes(pos, self.cam_lookat)
        x, y = xy[:3], xy[3:]
        z = np.cross(x, y)
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, np.stack([x, y, z], axis=1).reshape(-1))
        self.model.cam_pos[self.cam_id] = pos
        self.model.cam_quat[self.cam_id] = quat

    def set_camera_pose(self, azimuth_deg: float = 0.0, translation_m=(0.0, 0.0, 0.0)) -> None:
        """Azimuth about the look-at point (re-aimed), then a plain translation of the camera position with the
        orientation kept — the perturbation family of the camera_shift variation. Recipe v4 jitters both."""
        self.set_camera_azimuth(azimuth_deg)
        t = np.asarray(translation_m, dtype=float).reshape(3)
        if np.any(np.abs(t) > 1e-12):
            self.model.cam_pos[self.cam_id] = self.model.cam_pos[self.cam_id] + t

    # ----- episode control -----
    def reset(self, spec: EpisodeSpec, variation: str = "nominal", camera_azimuth_deg: float = 0.0, camera_translation_m=(0.0, 0.0, 0.0)) -> dict[str, np.ndarray]:
        if variation not in VARIATIONS:
            raise ValueError(f"unknown variation {variation!r}; choose from {VARIATIONS}")
        self.spec = spec
        mujoco.mj_resetData(self.model, self.data)
        self.set_camera_pose(camera_azimuth_deg, camera_translation_m)
        self.model.light_diffuse[:] = self._light_diffuse0
        if variation == "camera_shift":
            self.model.cam_pos[self.cam_id] = self.model.cam_pos[self.cam_id] + CAMERA_SHIFT_M
        elif variation == "camera_shift_far":
            self.model.cam_pos[self.cam_id] = self.model.cam_pos[self.cam_id] + CAMERA_SHIFT_FAR_M
        elif variation == "lighting":
            self.model.light_diffuse[:] = self._light_diffuse0 * 0.45
        target = relocated_target(spec) if variation == "target_relocation" else np.asarray(spec.target)
        self.data.mocap_pos[self.target_mocap] = target
        build_scene.set_home(self.model, self.data)
        self.data.qpos[self.arm_qpos] = np.asarray(spec.initial_q)
        self.data.ctrl[self.arm_act] = np.asarray(spec.initial_q)
        self.data.qvel[:] = 0.0
        self.gripper_cmd = 0.0
        self.data.ctrl[self.grip_act] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.controller.sync(self.data)
        self.detector.reset()
        self.safety = EpisodeSafety(self_collision_measurable=self.s6_measurable)
        self.min_error_m = self.error_m
        self.frame = 0
        return self.observation()

    def _apply(self, arm_targets: np.ndarray, gripper_cmd: float) -> None:
        self.data.ctrl[self.arm_act] = arm_targets
        self.data.ctrl[self.grip_act] = float(np.clip(gripper_cmd, 0.0, 1.0)) * GRIPPER_CTRL_MAX
        for _ in range(self.substeps):
            # Bias-force compensation on the arm joints (gravity + Coriolis), as the real
            # UR5e controller does internally. Without it the Menagerie position servos
            # (kp 2000 / 500) sag ~0.017 rad and a delta command re-planned from the sagged
            # pose creeps at <1 mm per step (measured 3 Sep 2026, SDD §7).
            self.data.qfrc_applied[self.arm_dof] = self.data.qfrc_bias[self.arm_dof]
            mujoco.mj_step(self.model, self.data)

    def step(self, action: np.ndarray, render: bool = True) -> StepResult:
        if self.spec is None:
            raise RuntimeError("call reset() first")
        ee_before = self.end_effector
        decision = self.wrapper.check(action, ee_before)
        executed = np.zeros(7, dtype=np.float32)
        if decision.ok:
            # Deltas integrate on the controller's *commanded* pose (teleop convention);
            # the servo tracks it with lag and the observation reports the actual pose.
            arm_targets, residual = self.controller.solve_delta(decision.action.astype(np.float64))
            ik = self.wrapper.check_ik(residual)
            if ik.ok:
                self.gripper_cmd = float(np.clip(self.gripper_cmd + 0.1 * decision.action[6], 0.0, 1.0))
                self._apply(arm_targets, self.gripper_cmd)
                executed = decision.action
            else:
                decision = Decision(False, ik.code, ik.depth)
                self.safety.note(ik.code, ik.depth, rejected=True)
                self._apply(self.data.ctrl[self.arm_act].copy(), self.gripper_cmd)
        else:
            self.safety.note(decision.code, decision.depth, rejected=True)
            self._apply(self.data.ctrl[self.arm_act].copy(), self.gripper_cmd)  # hold
        for code, depth in SafetyWrapper.measure(self.model, self.data, self.robot_geoms, self.floor_geom):
            if code == "S6_self_collision" and not self.s6_measurable:
                continue
            self.safety.note(code, depth, rejected=False)
        self.frame += 1
        error = self.error_m
        self.min_error_m = min(self.min_error_m, error)
        success = self.detector.update(error)
        progress = 1.0 if success else (0.5 if self.min_error_m <= PROGRESS_HALF_DISTANCE_M else 0.0)
        return StepResult(self.observation(render), error, success, progress, decision, executed)

    def close(self) -> None:
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None


def run_episode(env: BedEnv, expert, spec: EpisodeSpec, variation: str = "nominal", max_frames: int = MAX_FRAMES) -> dict:
    """Roll one episode with a scripted expert; returns the episode summary row."""
    env.reset(spec, variation)
    expert.reset(spec)
    success = False
    frames = 0
    for _ in range(max_frames):
        cmd_pos, cmd_rot = env.commanded_ee
        out = expert.act(cmd_pos, cmd_rot, env.target)
        result = env.step(out.executed)
        frames += 1
        if result.success:
            success = True
            break
    return {
        "seed": spec.seed,
        "family": spec.family,
        "cell": spec.cell,
        "variation": variation,
        "success": success,
        "progress": 1.0 if success else (0.5 if env.min_error_m <= PROGRESS_HALF_DISTANCE_M else 0.0),
        "frames": frames,
        "final_error_m": round(env.error_m, 5),
        "min_error_m": round(float(env.min_error_m), 5),
        "safe": env.safety.safe,
        "rejections": dict(env.safety.rejections),
        "measured": dict(env.safety.measured),
        "worst_depth": round(env.safety.worst_depth, 4),
    }
