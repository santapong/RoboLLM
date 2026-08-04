"""
camera_logger.py — synchronized (image, state, action) capture.

This is the substrate for demonstration data. The single most important
property: each saved frame pairs a camera image with the arm state read
at (as close as possible to) the SAME instant, plus the action that was
commanded. Get the timestamps right here and Phase B is easy.

Sync strategy (simple + good enough at 10-30 Hz):
  1. grab the newest camera frame
  2. immediately read arm state
  3. record host timestamps for BOTH; store the pair
The residual offset (a few ms of USB latency) is logged so you can audit
it. For tighter sync later, timestamp on the capture thread — noted, not
needed to start.

Depends on: opencv-python (cv2), numpy, and arm_serial.ArmSerial.
    pip install opencv-python numpy
Output layout (one folder per episode):
    episodes/ep_0000/
        frames/000000.jpg, 000001.jpg, ...
        record.jsonl        # one line per step: t, q, gripper, action, frame
        meta.json           # fps, camera info, task label
"""
from __future__ import annotations
import json
import os
import time
from dataclasses import dataclass, asdict
from typing import Optional, Sequence

import cv2
import numpy as np

from arm_serial import ArmSerial, ArmState


@dataclass
class Frame:
    step: int
    t_host: float            # host time of the state read
    t_cam: float             # host time the image was grabbed
    cam_lag_ms: float        # t_host - t_cam, for auditing sync quality
    q: list                  # measured joint angles (rad)
    gripper: float           # measured gripper 0..1
    action_q: Optional[list] # commanded joint targets this step (rad) or None
    action_gripper: Optional[float]
    frame_path: str


class Camera:
    def __init__(self, index: int = 0, width: int = 640, height: int = 480):
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.cap.isOpened():
            raise RuntimeError(f"camera index {index} did not open")
        # warm up (first frames are often dark/garbage)
        for _ in range(5):
            self.cap.read()

    def grab(self):
        """Return (image_bgr, t_host). Drains buffer to get the NEWEST frame."""
        # a single read() may return a stale buffered frame; grab twice
        self.cap.grab()
        ok, img = self.cap.read()
        t = time.time()
        if not ok:
            raise RuntimeError("camera read failed")
        return img, t

    def release(self):
        self.cap.release()


class EpisodeRecorder:
    def __init__(self, arm: ArmSerial, cam: Camera, out_root: str = "episodes",
                 task_label: str = "task"):
        self.arm = arm
        self.cam = cam
        self.out_root = out_root
        self.task_label = task_label

    def _new_episode_dir(self) -> str:
        os.makedirs(self.out_root, exist_ok=True)
        n = len([d for d in os.listdir(self.out_root) if d.startswith("ep_")])
        d = os.path.join(self.out_root, f"ep_{n:04d}")
        os.makedirs(os.path.join(d, "frames"), exist_ok=True)
        return d

    def record_step(self, ep_dir: str, step: int,
                    action_q: Optional[Sequence[float]] = None,
                    action_gripper: Optional[float] = None) -> Frame:
        """Capture one synchronized (image, state) pair and log it."""
        img, t_cam = self.cam.grab()
        st: ArmState = self.arm.get_state()      # read state right after the image
        frame_rel = os.path.join("frames", f"{step:06d}.jpg")
        cv2.imwrite(os.path.join(ep_dir, frame_rel), img)
        fr = Frame(
            step=step, t_host=st.t_host, t_cam=t_cam,
            cam_lag_ms=(st.t_host - t_cam) * 1000.0,
            q=[round(x, 5) for x in st.q], gripper=round(st.gripper, 4),
            action_q=[round(x, 5) for x in action_q] if action_q is not None else None,
            action_gripper=action_gripper, frame_path=frame_rel,
        )
        with open(os.path.join(ep_dir, "record.jsonl"), "a") as f:
            f.write(json.dumps(asdict(fr)) + "\n")
        return fr

    def record_episode(self, steps: int, hz: float = 15.0,
                       action_source=None) -> str:
        """Record one episode of `steps` frames at `hz`.

        action_source(step, state) -> (q, gripper) or None
          - For TELEOPERATION: return the operator's commanded target each step
            (and this method will send it to the arm).
          - For KINESTHETIC teaching (arm relaxed, moved by hand): pass None;
            we only record measured state, action == the next measured state.
        """
        ep = self._new_episode_dir()
        period = 1.0 / hz
        lags = []
        for step in range(steps):
            t0 = time.time()
            action_q = action_gripper = None
            if action_source is not None:
                st = self.arm.get_state()
                out = action_source(step, st)
                if out is not None:
                    action_q, action_gripper = out
                    self.arm.set_action(action_q, action_gripper)
            fr = self.record_step(ep, step, action_q, action_gripper)
            lags.append(fr.cam_lag_ms)
            # maintain loop rate
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)
        meta = {"fps": hz, "steps": steps, "task": self.task_label,
                "cam_lag_ms_mean": float(np.mean(lags)),
                "cam_lag_ms_max": float(np.max(lags))}
        with open(os.path.join(ep, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        print(f"[rec] {ep}: {steps} frames @ {hz}Hz, "
              f"cam lag mean={meta['cam_lag_ms_mean']:.1f}ms max={meta['cam_lag_ms_max']:.1f}ms")
        return ep


if __name__ == "__main__":
    # Kinesthetic demo (arm relaxed, you move it by hand):
    
    with ArmSerial() as arm:   # ARM_PORT env or autodetect
        arm.relax()                       # so you can move it by hand
        cam = Camera(index=0)
        try:
            rec = EpisodeRecorder(arm, cam, task_label="pick_block")
            rec.record_episode(steps=150, hz=15.0, action_source=None)
        finally:
            cam.release()
