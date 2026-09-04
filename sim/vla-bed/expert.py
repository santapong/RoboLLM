"""End-effector-space controller and scripted experts (SDD §7, design rules R2–R4).

`MinkController` turns a clamped EE delta (base frame) into joint position
targets for the UR5e through mink differential IK. `OracleExpert` moves the EE
straight toward the target while holding the home (tool-down) orientation.
`NoisyExpert` follows DART / Zhang et al.: it *executes* a noisy action but
*labels* the frame with the clean expert action (both are returned).

`headroom` < 1 caps the clean label below the safety limits (recipe v3: 0.7 ×), so a
policy's regression spread around the label stays inside S2/S3 instead of being
rejected one-sidedly — the v2 labels sat exactly on the cap (audit, 4 Sep 2026).
The executed (noisy) action is still clipped at the full limits.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import mink
import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scene import build_scene  # noqa: E402

XYZ_STEP_LIMIT_M = 0.010  # per 20 Hz control step (= 0.20 m/s), SDD §6.2 S2
RPY_STEP_LIMIT_RAD = 0.05  # per control step, SDD §6.2 S3
IK_RESIDUAL_LIMIT_M = 0.02  # SDD §6.2 S5
EXPERT_NOISE_SEED_OFFSET = 424_242  # B1's value
DEFAULT_NOISE_FRACTION = 0.5  # sigma = fraction × per-step limit (measured, not literature)
ACTION_DIM = 7
STEP_LIMITS = np.array([XYZ_STEP_LIMIT_M] * 3 + [RPY_STEP_LIMIT_RAD] * 3 + [1.0])
IK_ITERATIONS = 8
IK_DT = 0.05


def _rotvec_between(rot_current: np.ndarray, rot_target: np.ndarray) -> np.ndarray:
    """Rotation vector that takes rot_current to rot_target, in the base frame."""
    rel = mink.SO3.from_matrix(rot_target @ rot_current.T)
    return rel.log()


class MinkController:
    """Differential IK on the unmodified Menagerie model; gripper joints are frozen."""

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.configuration = mink.Configuration(model)
        self.ee_task = mink.FrameTask(
            frame_name=build_scene.EE_SITE, frame_type="site", position_cost=1.0, orientation_cost=0.2, lm_damping=1e-6
        )
        self.posture_task = mink.PostureTask(model, cost=1e-3)
        self.limits = [
            mink.ConfigurationLimit(model),
            mink.VelocityLimit(model, {name: np.pi for name in build_scene.ARM_JOINTS}),
        ]
        self.arm_qpos = np.array([model.joint(j).qposadr[0] for j in build_scene.ARM_JOINTS])
        self.arm_dof = np.array([model.joint(j).dofadr[0] for j in build_scene.ARM_JOINTS])
        self.gripper_dof = np.array([i for i in range(model.nv) if i not in set(self.arm_dof.tolist())])
        self.home_rot = self._rot_at(build_scene.HOME_QPOS)
        self.solver = "daqp"

    def _rot_at(self, arm_q: np.ndarray) -> np.ndarray:
        q = self.configuration.q.copy()
        q[self.arm_qpos] = arm_q
        self.configuration.update(q)
        return self.configuration.get_transform_frame_to_world(build_scene.EE_SITE, "site").as_matrix()[:3, :3].copy()

    def sync(self, data: mujoco.MjData) -> None:
        self.configuration.update(data.qpos.copy())
        self.posture_task.set_target(self.configuration.q)

    def ee_pose(self) -> tuple[np.ndarray, np.ndarray]:
        T = self.configuration.get_transform_frame_to_world(build_scene.EE_SITE, "site").as_matrix()
        return T[:3, 3].copy(), T[:3, :3].copy()

    def solve_delta(self, delta: np.ndarray, iterations: int = IK_ITERATIONS) -> tuple[np.ndarray, float]:
        """Return (arm joint targets, position residual in metres) for one EE delta."""
        pos, rot = self.ee_pose()
        target_pos = pos + delta[:3]
        target_rot = mink.SO3.exp(delta[3:6]).as_matrix() @ rot
        self.ee_task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(target_rot), target_pos))
        for _ in range(iterations):
            vel = mink.solve_ik(self.configuration, [self.ee_task, self.posture_task], IK_DT, self.solver, limits=self.limits)
            vel[self.gripper_dof] = 0.0
            self.configuration.integrate_inplace(vel, IK_DT)
        new_pos, _ = self.ee_pose()
        residual = float(np.linalg.norm(new_pos - target_pos))
        return self.configuration.q[self.arm_qpos].copy(), residual

    def settle_residual(self, target_pos: np.ndarray, iterations: int = 200) -> float:
        """Reachability probe from home: drive the EE to target_pos, return the residual."""
        q = self.configuration.q.copy()
        q[self.arm_qpos] = build_scene.HOME_QPOS
        self.configuration.update(q)
        self.posture_task.set_target(self.configuration.q)
        self.ee_task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(self.home_rot), np.asarray(target_pos, dtype=float)))
        for _ in range(iterations):
            vel = mink.solve_ik(self.configuration, [self.ee_task, self.posture_task], IK_DT, self.solver, limits=self.limits)
            vel[self.gripper_dof] = 0.0
            self.configuration.integrate_inplace(vel, IK_DT)
        pos, _ = self.ee_pose()
        return float(np.linalg.norm(pos - np.asarray(target_pos)))


def clip_action(action: np.ndarray) -> np.ndarray:
    return np.clip(np.asarray(action, dtype=np.float64), -STEP_LIMITS, STEP_LIMITS)


@dataclass
class ExpertOutput:
    clean: np.ndarray  # the label recorded as `action`
    executed: np.ndarray  # what is applied (`action.executed`)


class OracleExpert:
    name = "oracle"

    def __init__(self, home_rot: np.ndarray, headroom: float = 1.0):
        if not 0.0 < headroom <= 1.0:
            raise ValueError("headroom must be in (0, 1]")
        self.home_rot = home_rot
        self.headroom = float(headroom)
        self.cap = STEP_LIMITS.copy()
        self.cap[:6] *= self.headroom

    def reset(self, spec) -> None:  # noqa: ARG002 — interface parity with NoisyExpert
        return None

    def clean_action(self, ee_pos: np.ndarray, ee_rot: np.ndarray, target: np.ndarray) -> np.ndarray:
        action = np.zeros(ACTION_DIM)
        action[:3] = target - ee_pos
        action[3:6] = _rotvec_between(ee_rot, self.home_rot)
        return np.clip(action, -self.cap, self.cap).astype(np.float32)

    def act(self, ee_pos, ee_rot, target) -> ExpertOutput:
        clean = self.clean_action(ee_pos, ee_rot, target)
        return ExpertOutput(clean=clean, executed=clean.copy())


class NoisyExpert(OracleExpert):
    """Executes clean + N(0, (fraction·limit)²) on the six motion dims; labels stay clean."""

    name = "noisy"

    def __init__(self, home_rot: np.ndarray, noise_fraction: float = DEFAULT_NOISE_FRACTION, headroom: float = 1.0):
        super().__init__(home_rot, headroom)
        if noise_fraction < 0:
            raise ValueError("noise_fraction must be non-negative")
        self.noise_fraction = float(noise_fraction)
        self.sigma = self.noise_fraction * STEP_LIMITS[:6]
        self._rng = np.random.default_rng(0)

    def reset(self, spec) -> None:
        self._rng = np.random.default_rng(spec.seed + EXPERT_NOISE_SEED_OFFSET)

    def act(self, ee_pos, ee_rot, target) -> ExpertOutput:
        clean = self.clean_action(ee_pos, ee_rot, target)
        executed = clean.astype(np.float64).copy()
        executed[:6] += self._rng.normal(0.0, self.sigma)
        # The wrapper would reject an out-of-limit action; noise is truncated at the limit (SDD §7).
        return ExpertOutput(clean=clean, executed=clip_action(executed).astype(np.float32))


EXPERTS = {"oracle": OracleExpert, "noisy": NoisyExpert}


def make_expert(name: str, home_rot: np.ndarray, noise_fraction: float = DEFAULT_NOISE_FRACTION, headroom: float = 1.0):
    if name == "oracle":
        return OracleExpert(home_rot, headroom)
    if name == "noisy":
        return NoisyExpert(home_rot, noise_fraction, headroom)
    raise ValueError(f"unknown expert {name!r}; choose from {sorted(EXPERTS)}")
