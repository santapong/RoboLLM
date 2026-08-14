#!/usr/bin/env python3
"""Record a scripted 6-DOF arm episode directly as a LeRobot v3 dataset."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Any

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)] + ["gripper"]
MODEL_XML = """
<mujoco model="robollm_diy_arm">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>
  <visual><global offwidth="320" offheight="240"/></visual>
  <default>
    <joint type="hinge" damping="1.2" armature="0.03" limited="true"
           range="-1.4 1.4"/>
    <geom type="capsule" size="0.035" density="500" rgba="0.15 0.55 0.85 1"/>
    <position kp="45" kv="5" ctrllimited="true" ctrlrange="-1.4 1.4"/>
  </default>
  <worldbody>
    <light pos="1 -1 2"/>
    <geom type="plane" size="2 2 0.05" rgba="0.18 0.20 0.24 1"/>
    <body name="base" pos="0 0 0.08">
      <geom type="cylinder" size="0.11 0.08" rgba="0.25 0.28 0.32 1"/>
      <body name="link1" pos="0 0 0.08">
        <joint name="joint1" axis="0 0 1"/>
        <geom fromto="0 0 0 0 0 0.22"/>
        <body name="link2" pos="0 0 0.22">
          <joint name="joint2" axis="0 1 0" range="-1.1 1.1"/>
          <geom fromto="0 0 0 0 0 0.24"/>
          <body name="link3" pos="0 0 0.24">
            <joint name="joint3" axis="0 1 0" range="-1.2 1.2"/>
            <geom fromto="0 0 0 0 0 0.22"/>
            <body name="link4" pos="0 0 0.22">
              <joint name="joint4" axis="1 0 0"/>
              <geom fromto="0 0 0 0 0 0.14" size="0.03"/>
              <body name="link5" pos="0 0 0.14">
                <joint name="joint5" axis="0 1 0" range="-1.1 1.1"/>
                <geom fromto="0 0 0 0 0 0.11" size="0.028"/>
                <body name="link6" pos="0 0 0.11">
                  <joint name="joint6" axis="0 0 1"/>
                  <geom fromto="0 0 0 0 0 0.09" size="0.025"/>
                  <body name="gripper" pos="0 0 0.09">
                    <joint name="gripper" type="slide" axis="1 0 0"
                           range="0 1" damping="2"/>
                    <geom type="box" size="0.06 0.025 0.025"
                          rgba="0.95 0.55 0.15 1"/>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position joint="joint1"/>
    <position joint="joint2" ctrlrange="-1.1 1.1"/>
    <position joint="joint3" ctrlrange="-1.2 1.2"/>
    <position joint="joint4"/>
    <position joint="joint5" ctrlrange="-1.1 1.1"/>
    <position joint="joint6"/>
    <position joint="gripper" kp="25" kv="3" ctrlrange="0 1"/>
  </actuator>
</mujoco>
"""


def build_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(MODEL_XML)


def scripted_action(frame: int, frames: int, episode: int = 0) -> np.ndarray:
    """Smooth deterministic target that excites every axis without hitting limits."""
    if frames <= 1:
        raise ValueError("frames must be greater than one")
    phase = 2.0 * np.pi * frame / (frames - 1) + episode * 0.2
    return np.asarray(
        [
            0.55 * np.sin(phase),
            0.45 * np.sin(phase + 0.4),
            0.50 * np.sin(2.0 * phase),
            0.35 * np.cos(phase),
            0.30 * np.sin(phase - 0.6),
            0.40 * np.cos(2.0 * phase),
            0.50 + 0.45 * np.sin(phase),
        ],
        dtype=np.float32,
    )


def dataset_features(height: int, width: int) -> dict[str, dict]:
    return {
        "observation.images.front": {
            "dtype": "video",
            "shape": (height, width, 3),
            "names": ["height", "width", "channels"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (7,),
            "names": JOINT_NAMES,
        },
        "action": {
            "dtype": "float32",
            "shape": (7,),
            "names": JOINT_NAMES,
        },
        "observation.camera_lag_ms": {
            "dtype": "float32",
            "shape": (1,),
            "names": ["camera_lag_ms"],
        },
    }


def _dataset_type(dataset_class: Any | None = None):
    if dataset_class is not None:
        return dataset_class
    try:
        from lerobot.datasets import LeRobotDataset
    except ImportError as exc:
        raise RuntimeError(
            "Install requirements/lerobot.txt in .venv-lerobot before recording"
        ) from exc
    return LeRobotDataset


def _steps_per_frame(model: mujoco.MjModel, fps: int) -> int:
    physics_hz = round(1.0 / model.opt.timestep)
    if fps <= 0 or physics_hz % fps:
        raise ValueError(f"fps must divide the {physics_hz} Hz physics rate")
    return physics_hz // fps


def run_policy(frames: int = 100, fps: int = 20) -> dict[str, float]:
    """Run the scripted policy headlessly without rendering or dataset packages."""
    model = build_model()
    data = mujoco.MjData(model)
    substeps = _steps_per_frame(model, fps)
    errors = []
    for frame in range(frames):
        action = scripted_action(frame, frames)
        data.ctrl[:] = action
        for _ in range(substeps):
            mujoco.mj_step(model, data)
        errors.append(np.square(data.qpos - action))
    return {
        "frames": float(frames),
        "sim_seconds": float(data.time),
        "tracking_rmse_rad": float(np.sqrt(np.mean(errors))),
    }


def record_dataset(
    root: str | Path,
    repo_id: str,
    task: str,
    episodes: int = 1,
    frames: int = 100,
    fps: int = 20,
    height: int = 240,
    width: int = 320,
    dataset_class: Any | None = None,
) -> dict[str, float]:
    if episodes <= 0 or frames <= 1:
        raise ValueError("episodes must be positive and frames must be greater than one")
    model = build_model()
    data = mujoco.MjData(model)
    substeps = _steps_per_frame(model, fps)
    dataset = _dataset_type(dataset_class).create(
        repo_id=repo_id,
        root=Path(root),
        fps=fps,
        robot_type="robollm_diy_arm_sim",
        features=dataset_features(height, width),
        use_videos=True,
    )
    renderer = mujoco.Renderer(model, height=height, width=width)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.0, 0.0, 0.48]
    camera.distance = 1.45
    camera.azimuth = 135
    camera.elevation = -18
    errors = []

    try:
        for episode in range(episodes):
            mujoco.mj_resetData(model, data)
            for frame in range(frames):
                action = scripted_action(frame, frames, episode)
                data.ctrl[:] = action
                for _ in range(substeps):
                    mujoco.mj_step(model, data)
                renderer.update_scene(data, camera=camera)
                image = renderer.render().copy()
                state = np.asarray(data.qpos, dtype=np.float32).copy()
                errors.append(np.square(state - action))
                dataset.add_frame(
                    {
                        "observation.images.front": image,
                        "observation.state": state,
                        "action": action,
                        "observation.camera_lag_ms": np.zeros(1, dtype=np.float32),
                        "task": task,
                    }
                )
            dataset.save_episode()
    except Exception:
        dataset.clear_episode_buffer()
        raise
    finally:
        renderer.close()
        dataset.finalize()

    return {
        "episodes": float(episodes),
        "frames": float(episodes * frames),
        "tracking_rmse_rad": float(np.sqrt(np.mean(errors))),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default="datasets/robollm-mujoco")
    parser.add_argument("--repo-id", default="local/robollm-mujoco")
    parser.add_argument("--task", default="move every arm joint smoothly")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()

    if args.validate_only:
        metrics = run_policy(frames=args.frames, fps=args.fps)
    else:
        metrics = record_dataset(
            root=args.root,
            repo_id=args.repo_id,
            task=args.task,
            episodes=args.episodes,
            frames=args.frames,
            fps=args.fps,
        )
    print(" ".join(f"{key}={value:.4f}" for key, value in metrics.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
