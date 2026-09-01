#!/usr/bin/env python3
"""Deterministic MuJoCo red-target reaching task used by B1 preparation.

This module intentionally has no LeRobot or torch dependency.  Dataset writers,
learned-policy adapters, and evaluators all share this frozen task boundary.
"""

from __future__ import annotations

import os
from collections.abc import Iterable
from dataclasses import asdict, dataclass

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from arm_dataset import JOINT_NAMES
from arm_dataset import MODEL_XML as A3_MODEL_XML

INSTRUCTION = "touch the red target"
FPS = 20
MAX_FRAMES = 100
SUCCESS_DISTANCE_M = 0.03
SUCCESS_FRAMES = 5
ACTION_SLEW = np.asarray([0.10] * 6 + [0.08], dtype=np.float64)

# Data-collection expert noise (see NoisyExpert). These affect the recorded
# trajectory only; the task contract above is unchanged.
# 1.75 is the tuned value: 100% expert success and no episode truncated at the
# frame cap over 100 seeds, at 2.6x the oracle's trajectory length. 2.0 reaches
# 48-frame episodes but truncates 2%, which would clone failed demonstrations.
DEFAULT_EXPERT_NOISE = 1.75
EXPERT_NOISE_SEED_OFFSET = 424_242
# Interior margin held away from the actuator bounds, matching make_episode_spec.
BOUNDS_MARGIN = 0.02

# Targets are derived from these reachable arm postures, then receive small
# seeded Cartesian jitter.  The gripper remains half-open in every family.
GOAL_FAMILIES: dict[str, np.ndarray] = {
    "front_high": np.asarray([0.00, -0.38, 0.48, 0.00, -0.18, 0.00, 0.50]),
    "front_low": np.asarray([0.00, 0.18, 0.62, 0.00, -0.12, 0.00, 0.50]),
    "left": np.asarray([0.55, -0.20, 0.52, 0.00, -0.16, 0.00, 0.50]),
    "right": np.asarray([-0.55, -0.20, 0.52, 0.00, -0.16, 0.00, 0.50]),
    "near": np.asarray([0.12, 0.36, 0.70, 0.00, -0.28, 0.00, 0.50]),
}
FAMILY_NAMES = tuple(GOAL_FAMILIES)

# The A3 model remains byte-for-byte available in arm_dataset.py.  B1 extends
# it only with a red target, an end-effector site, and a visual occluder.
REACHING_MODEL_XML = (
    A3_MODEL_XML.replace(
        '<position kp="45" kv="5"',
        '<position kp="180" kv="18"',
    )
    .replace(
        '<light pos="1 -1 2"/>',
        '<light name="key" pos="1 -1 2" diffuse="1 1 1"/>'
        '<body name="red_target" mocap="true" pos="0 0 0.5">'
        '<geom name="target_geom" type="sphere" size="0.035" '
        'rgba="0.90 0.02 0.02 1" contype="0" conaffinity="0"/>'
        "</body>"
        '<body name="occluder" mocap="true" pos="0 0 -2">'
        '<geom name="occluder_geom" type="box" size="0.10 0.02 0.12" '
        'rgba="0.10 0.10 0.12 0" contype="0" conaffinity="0"/>'
        "</body>",
    )
    .replace(
        '<geom type="box" size="0.06 0.025 0.025"\n'
        '                          rgba="0.95 0.55 0.15 1"/>',
        '<geom type="box" size="0.06 0.025 0.025"\n'
        '                          rgba="0.95 0.55 0.15 1"/>'
        '<site name="end_effector" pos="0 0 0.035" size="0.012" '
        'rgba="0.2 1 0.2 1"/>',
    )
)


@dataclass(frozen=True)
class EpisodeSpec:
    """Everything needed to reproduce one episode (including privileged goal)."""

    seed: int
    split: str
    family: str
    initial_state: tuple[float, ...]
    target: tuple[float, float, float]
    goal_state: tuple[float, ...]

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass
class StepResult:
    observation: dict[str, np.ndarray]
    error_m: float
    success: bool


class SuccessDetector:
    """Require the distance threshold for consecutive control frames."""

    def __init__(
        self,
        threshold_m: float = SUCCESS_DISTANCE_M,
        consecutive_frames: int = SUCCESS_FRAMES,
    ) -> None:
        self.threshold_m = threshold_m
        self.consecutive_frames = consecutive_frames
        self.streak = 0

    def update(self, error_m: float) -> bool:
        self.streak = self.streak + 1 if error_m <= self.threshold_m else 0
        return self.streak >= self.consecutive_frames


def build_reaching_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(REACHING_MODEL_XML)


def actuator_bounds(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]:
    return model.actuator_ctrlrange[:, 0].copy(), model.actuator_ctrlrange[:, 1].copy()


def clamp_action(model: mujoco.MjModel, action: np.ndarray) -> np.ndarray:
    low, high = actuator_bounds(model)
    return np.clip(np.asarray(action, dtype=np.float64), low, high)


def slew_limit(desired: np.ndarray, previous: np.ndarray) -> np.ndarray:
    desired = np.asarray(desired, dtype=np.float64)
    previous = np.asarray(previous, dtype=np.float64)
    if desired.shape != (7,) or previous.shape != (7,):
        raise ValueError("desired and previous actions must each have shape (7,)")
    return previous + np.clip(desired - previous, -ACTION_SLEW, ACTION_SLEW)


def _site_position_for_qpos(model: mujoco.MjModel, qpos: np.ndarray) -> np.ndarray:
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
    return data.site_xpos[site_id].copy()


def _settled_site_for_ctrl(model: mujoco.MjModel, control: np.ndarray) -> np.ndarray:
    """Compute the gravity-loaded position reached by a constant safe command."""
    data = mujoco.MjData(model)
    data.qpos[:] = control
    data.ctrl[:] = control
    for _ in range(1_500):  # three simulated seconds at the 2 ms physics rate
        mujoco.mj_step(model, data)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
    return data.site_xpos[site_id].copy()


def make_episode_spec(seed: int, family: str, split: str = "train") -> EpisodeSpec:
    if family not in GOAL_FAMILIES:
        raise ValueError(f"unknown goal family {family!r}; choose from {FAMILY_NAMES}")
    if split not in {"train", "evaluation"}:
        raise ValueError("split must be 'train' or 'evaluation'")

    rng = np.random.default_rng(seed)
    model = build_reaching_model()
    low, high = actuator_bounds(model)
    initial = np.asarray([0.0, -0.05, 0.10, 0.0, -0.05, 0.0, 0.50])
    initial += rng.uniform(-0.06, 0.06, size=7)
    initial[6] = np.clip(initial[6], 0.35, 0.65)
    initial = np.clip(initial, low + 0.02, high - 0.02)

    reference_q = GOAL_FAMILIES[family].copy()
    reference_q[:6] += rng.uniform(-0.025, 0.025, size=6)
    reference_q = np.clip(reference_q, low + 0.02, high - 0.02)
    # Joint jitter produces a seeded spatial target jitter while retaining an
    # exact, privileged expert solution. Neither value is a model observation.
    target = _settled_site_for_ctrl(model, reference_q)

    return EpisodeSpec(
        seed=int(seed),
        split=split,
        family=family,
        initial_state=tuple(float(x) for x in initial),
        target=tuple(float(x) for x in target),
        goal_state=tuple(float(x) for x in reference_q),
    )


def episode_specs(
    count: int,
    seed: int,
    split: str,
    families: Iterable[str] = FAMILY_NAMES,
) -> list[EpisodeSpec]:
    """Return balanced, deterministic specs with split-isolated random seeds."""
    if count <= 0:
        raise ValueError("count must be positive")
    family_list = tuple(families)
    if not family_list:
        raise ValueError("at least one family is required")
    offset = 0 if split == "train" else 1_000_000_000
    return [
        make_episode_spec(
            seed + offset + index, family_list[index % len(family_list)], split
        )
        for index in range(count)
    ]


class ReachingEnv:
    """20 Hz position-control task with optional visual evaluation variations."""

    def __init__(
        self, height: int = 240, width: int = 320, render: bool = True
    ) -> None:
        self.model = build_reaching_model()
        self.data = mujoco.MjData(self.model)
        self.height = height
        self.width = width
        self.substeps = round((1.0 / self.model.opt.timestep) / FPS)
        self.renderer = (
            mujoco.Renderer(self.model, height=height, width=width) if render else None
        )
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [0.0, 0.0, 0.48]
        self.camera.distance = 1.45
        self.camera.azimuth = 135
        self.camera.elevation = -18
        self.detector = SuccessDetector()
        self.last_action = np.zeros(7, dtype=np.float64)
        self.spec: EpisodeSpec | None = None
        self._site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "end_effector"
        )
        self._target_mocap_id = self.model.body_mocapid[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "red_target")
        ]
        self._occluder_mocap_id = self.model.body_mocapid[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "occluder")
        ]
        self._occluder_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "occluder_geom"
        )
        self._key_light_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_LIGHT, "key"
        )

    def reset(
        self, spec: EpisodeSpec, variation: str = "nominal"
    ) -> dict[str, np.ndarray]:
        mujoco.mj_resetData(self.model, self.data)
        self.model.light_diffuse[self._key_light_id] = [1.0, 1.0, 1.0]
        self.model.geom_rgba[self._occluder_geom_id, 3] = 0.0
        self.camera.azimuth = 135
        self.camera.elevation = -18
        target = np.asarray(spec.target, dtype=np.float64).copy()
        if variation == "camera_shift":
            self.camera.azimuth = 150
            self.camera.elevation = -12
        elif variation == "lighting":
            self.model.light_diffuse[self._key_light_id] = [0.42, 0.50, 0.65]
        elif variation == "occlusion":
            self.model.geom_rgba[self._occluder_geom_id, 3] = 0.82
            self.data.mocap_pos[self._occluder_mocap_id] = target + [0.08, -0.08, 0.02]
        elif variation == "target_relocation":
            rng = np.random.default_rng(spec.seed + 77_777)
            relocated_goal = np.asarray(spec.goal_state, dtype=np.float64).copy()
            relocated_goal[:3] += rng.uniform(-0.04, 0.04, size=3)
            relocated_goal = clamp_action(self.model, relocated_goal)
            target = _settled_site_for_ctrl(self.model, relocated_goal)
        elif variation != "nominal":
            raise ValueError(f"unknown variation {variation!r}")

        self.data.qpos[:] = spec.initial_state
        self.data.ctrl[:] = spec.initial_state
        self.data.mocap_pos[self._target_mocap_id] = target
        self.data.mocap_quat[self._target_mocap_id] = [1.0, 0.0, 0.0, 0.0]
        self.last_action = np.asarray(spec.initial_state, dtype=np.float64).copy()
        self.detector = SuccessDetector()
        self.spec = EpisodeSpec(
            seed=spec.seed,
            split=spec.split,
            family=spec.family,
            initial_state=spec.initial_state,
            target=tuple(float(x) for x in target),
            goal_state=tuple(
                float(x)
                for x in (
                    relocated_goal
                    if variation == "target_relocation"
                    else np.asarray(spec.goal_state)
                )
            ),
        )
        mujoco.mj_forward(self.model, self.data)
        return self.observation()

    @property
    def end_effector(self) -> np.ndarray:
        return self.data.site_xpos[self._site_id].copy()

    @property
    def target(self) -> np.ndarray:
        return self.data.mocap_pos[self._target_mocap_id].copy()

    @property
    def error_m(self) -> float:
        return float(np.linalg.norm(self.target - self.end_effector))

    def observation(self) -> dict[str, np.ndarray]:
        state = np.asarray(self.data.qpos, dtype=np.float32).copy()
        if self.renderer is None:
            image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        else:
            self.renderer.update_scene(self.data, camera=self.camera)
            image = self.renderer.render().copy()
        # Deliberately no target coordinates: the camera is the only goal input.
        return {
            "observation.images.front": image,
            "observation.state": state,
            "observation.camera_lag_ms": np.zeros(1, dtype=np.float32),
        }

    def step(self, action: np.ndarray) -> StepResult:
        action = np.asarray(action, dtype=np.float64)
        if action.shape != (7,) or not np.isfinite(action).all():
            raise ValueError(
                "ReachingEnv accepts only finite shape-(7,) validated actions"
            )
        low, high = actuator_bounds(self.model)
        if np.any(action < low) or np.any(action > high):
            raise ValueError("action exceeds actuator bounds")
        # Dataset/policy actions are float32, so allow only representation noise.
        if np.any(np.abs(action - self.last_action) > ACTION_SLEW + 1e-6):
            raise ValueError("action exceeds per-tick slew limit")
        self.data.ctrl[:] = action
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        self.last_action = action.copy()
        error = self.error_m
        return StepResult(self.observation(), error, self.detector.update(error))

    def close(self) -> None:
        if self.renderer is not None:
            self.renderer.close()


class OracleExpert:
    """Privileged reachable-posture expert used only to generate/prove data."""

    def reset(self, spec: EpisodeSpec) -> None:
        """No-op: the oracle is stateless and fully determined by the spec."""

    def action(self, env: ReachingEnv) -> np.ndarray:
        if env.spec is None:
            raise RuntimeError("reset the environment before requesting an action")
        desired = np.asarray(env.spec.goal_state, dtype=np.float64)
        desired = clamp_action(env.model, desired)
        return slew_limit(desired, env.last_action).astype(np.float32)


class NoisyExpert:
    """Same privileged goal, deliberately imperfect path.

    The oracle drives straight down the slew limit, so every state it records
    already lies on the optimal path and a policy cloned from it has never seen
    a recovery.  This expert perturbs the goal posture with seeded noise scaled
    by the *remaining* per-joint distance, so the arm wanders while it is far
    away and quiets down as it arrives — the episode still settles inside the
    5-frame success window, but the dataset gains off-optimal states paired
    with the corrective action that fixes them.

    The task boundary is untouched: the emitted command is clamped to actuator
    bounds and slew-limited exactly like the oracle's, so `ReachingEnv.step`
    validates it identically.
    """

    def __init__(self, scale: float = DEFAULT_EXPERT_NOISE) -> None:
        if scale < 0.0:
            raise ValueError("expert noise scale must be non-negative")
        self.scale = float(scale)
        self._rng = np.random.default_rng(0)

    def reset(self, spec: EpisodeSpec) -> None:
        # Offset keeps this stream independent of the spec's own jitter draws.
        self._rng = np.random.default_rng(spec.seed + EXPERT_NOISE_SEED_OFFSET)

    def action(self, env: ReachingEnv) -> np.ndarray:
        if env.spec is None:
            raise RuntimeError("reset the environment before requesting an action")
        desired = np.asarray(env.spec.goal_state, dtype=np.float64)
        sigma = self.scale * np.abs(desired - env.last_action)
        desired = desired + self._rng.normal(0.0, np.maximum(sigma, 0.0))
        # Hold the same interior margin make_episode_spec uses. Clamping hard to
        # the actuator bound lets the float32 cast below round a hair outside it,
        # and ReachingEnv.step checks bounds without tolerance.
        low, high = actuator_bounds(env.model)
        desired = np.clip(desired, low + BOUNDS_MARGIN, high - BOUNDS_MARGIN)
        return slew_limit(desired, env.last_action).astype(np.float32)


EXPERTS = {"oracle": OracleExpert, "noisy": NoisyExpert}


def make_expert(name: str = "oracle", scale: float = DEFAULT_EXPERT_NOISE):
    """Build a data-collection expert by name."""
    if name not in EXPERTS:
        raise ValueError(f"unknown expert {name!r}; choose from {sorted(EXPERTS)}")
    return NoisyExpert(scale) if name == "noisy" else OracleExpert()


def run_oracle_episode(
    spec: EpisodeSpec, max_frames: int = MAX_FRAMES, expert: object | None = None
) -> dict[str, object]:
    env = ReachingEnv(render=False)
    expert = OracleExpert() if expert is None else expert
    try:
        env.reset(spec)
        expert.reset(spec)
        min_error = env.error_m
        success = False
        frames = 0
        for frames in range(1, max_frames + 1):
            result = env.step(expert.action(env))
            min_error = min(min_error, result.error_m)
            if result.success:
                success = True
                break
        return {
            "success": success,
            "frames": frames,
            "final_error_m": env.error_m,
            "min_error_m": min_error,
        }
    finally:
        env.close()


__all__ = [
    "ACTION_SLEW",
    "DEFAULT_EXPERT_NOISE",
    "EXPERTS",
    "FAMILY_NAMES",
    "FPS",
    "GOAL_FAMILIES",
    "INSTRUCTION",
    "JOINT_NAMES",
    "MAX_FRAMES",
    "SUCCESS_DISTANCE_M",
    "SUCCESS_FRAMES",
    "EpisodeSpec",
    "NoisyExpert",
    "OracleExpert",
    "ReachingEnv",
    "SuccessDetector",
    "actuator_bounds",
    "build_reaching_model",
    "episode_specs",
    "make_episode_spec",
    "make_expert",
    "run_oracle_episode",
    "slew_limit",
]
