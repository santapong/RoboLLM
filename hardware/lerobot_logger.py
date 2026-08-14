#!/usr/bin/env python3
"""Record one DIY-arm demonstration as a LeRobot v3 dataset.

This intentionally stays small: one front camera, one 7-value state vector
(six joints plus gripper), one matching action vector, and one task label.
For kinesthetic teaching the arm is relaxed and the observed position is the
position action demonstrated by the operator.

LeRobot 0.6 requires NumPy 2 while ROS 2 Jazzy in this repository requires
NumPy 1.26. Run this file from the separate ``.venv-lerobot`` environment
described in ``hardware/README.md``.
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from arm_serial import ArmSerial
from camera_logger import Camera


def lerobot_features(image_shape: tuple[int, ...], joint_names: list[str]) -> dict[str, dict]:
    """Return the minimal LeRobot v3 schema used by this arm."""
    if len(image_shape) != 3 or image_shape[2] != 3:
        raise ValueError(f"expected an HxWx3 camera image, got {image_shape}")
    axes = [*joint_names, "gripper"]
    return {
        "observation.images.front": {
            "dtype": "video",
            "shape": image_shape,
            "names": ["height", "width", "channels"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (len(axes),),
            "names": axes,
        },
        "action": {
            "dtype": "float32",
            "shape": (len(axes),),
            "names": axes,
        },
        "observation.camera_lag_ms": {
            "dtype": "float32",
            "shape": (1,),
            "names": ["camera_lag_ms"],
        },
    }


class LeRobotRecorder:
    """Thin adapter from the existing arm/camera APIs to LeRobotDataset."""

    def __init__(
        self,
        arm: ArmSerial,
        camera: Camera,
        root: str | Path,
        repo_id: str,
        task: str,
        fps: int = 15,
        dataset_class: Any | None = None,
    ):
        if fps <= 0:
            raise ValueError("fps must be positive")
        if not task.strip():
            raise ValueError("task must not be empty")
        self.arm = arm
        self.camera = camera
        self.root = Path(root)
        self.repo_id = repo_id
        self.task = task.strip()
        self.fps = fps
        self._dataset_class = dataset_class

    def _dataset_type(self):
        if self._dataset_class is not None:
            return self._dataset_class
        try:
            from lerobot.datasets import LeRobotDataset
        except ImportError as exc:
            raise RuntimeError(
                "LeRobot is not installed. Create .venv-lerobot and install "
                "requirements/lerobot.txt; do not install it in the ROS venv."
            ) from exc
        return LeRobotDataset

    def record_episode(self, steps: int) -> Path:
        if steps <= 0:
            raise ValueError("steps must be positive")

        image_bgr, t_cam = self.camera.grab()
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        joint_names = [joint.name for joint in self.arm.config.joints]
        dataset = self._dataset_type().create(
            repo_id=self.repo_id,
            root=self.root,
            fps=self.fps,
            robot_type="robollm_diy_arm",
            features=lerobot_features(image_rgb.shape, joint_names),
            use_videos=True,
        )

        period = 1.0 / self.fps
        try:
            for step in range(steps):
                started = time.monotonic()
                if step:
                    image_bgr, t_cam = self.camera.grab()
                    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

                state = self.arm.get_state()
                position = np.asarray([*state.q, state.gripper], dtype=np.float32)
                dataset.add_frame(
                    {
                        "observation.images.front": image_rgb,
                        "observation.state": position,
                        "action": position.copy(),
                        "observation.camera_lag_ms": np.asarray(
                            [(state.t_host - t_cam) * 1000.0], dtype=np.float32
                        ),
                        "task": self.task,
                    }
                )
                remaining = period - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)
            dataset.save_episode()
        except Exception:
            dataset.clear_episode_buffer()
            raise
        finally:
            dataset.finalize()

        return self.root


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default="datasets/robollm-arm")
    parser.add_argument("--repo-id", default="local/robollm-arm")
    parser.add_argument("--task", default="pick up the block")
    parser.add_argument("--steps", type=int, default=150)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument(
        "--allow-commanded-state",
        action="store_true",
        help="allow a simulation-only dataset when encoders are not installed",
    )
    args = parser.parse_args()

    with ArmSerial() as arm:
        if arm.config.state_source != "measured" and not args.allow_commanded_state:
            raise RuntimeError(
                "state_source is not measured; install/configure encoders first, "
                "or use --allow-commanded-state only to test the simulation pipeline"
            )
        arm.relax()
        camera = Camera(index=args.camera)
        try:
            recorder = LeRobotRecorder(
                arm=arm,
                camera=camera,
                root=args.root,
                repo_id=args.repo_id,
                task=args.task,
                fps=args.fps,
            )
            output = recorder.record_episode(args.steps)
            print(f"[lerobot] wrote {args.steps} frames to {output}")
        finally:
            camera.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
