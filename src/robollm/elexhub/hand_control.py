"""Relative left-hand control of the local simulator, never physical hardware."""

from __future__ import annotations

import math
import threading
import time

import numpy as np

from .simulation import MotionRejected, SimArm

SAMPLE_MAX_AGE_S = 0.35
HEARTBEAT_LEASE_S = 1.5
CONTROL_INTERVAL_S = 0.05
MAX_TARGET_SPEED_M_S = 0.05


def accepted_sample(value, now):
    """Validate the camera contract independently at the control boundary."""
    if not isinstance(value, dict) or value.get("handedness") != "Left":
        return None
    try:
        stamp = float(value["timestamp"])
        wrist = np.asarray(value["wrist"], dtype=float)
        scale, pinch, confidence = (
            float(value[key]) for key in ["palm_scale", "pinch", "confidence"]
        )
    except (KeyError, TypeError, ValueError):
        return None
    if (
        not all(math.isfinite(v) for v in [stamp, scale, pinch, confidence])
        or not 0 <= now - stamp <= SAMPLE_MAX_AGE_S
        or wrist.shape != (2,)
        or not np.isfinite(wrist).all()
        or np.any(wrist < 0)
        or np.any(wrist > 1)
        or not 0.001 <= scale <= 2
        or not 0 <= pinch <= 1
        or not 0.65 <= confidence <= 1
    ):
        return None
    return {
        "timestamp": stamp,
        "wrist": wrist.copy(),
        "palm_scale": scale,
        "pinch": pinch,
    }


def relative_target(sample, neutral, neutral_tcp, geometry):
    """Map image-relative controls to a small bounded simulation workspace.

    Palm size is a control signal, not measured camera depth. A fixed image
    translation maps to the same direction in the mirrored preview and the
    existing overview camera (+Y is screen-right and +Z is up).
    """
    reach = sum(link["length_m"] for link in geometry["links"][1:])
    reach += 0.75 * geometry["gripper"]["finger_length_m"]
    delta_image = sample["wrist"] - neutral["wrist"]
    delta = np.array(
        [
            0.17 * reach * math.log(sample["palm_scale"] / neutral["palm_scale"]),
            0.55 * reach * delta_image[0],
            -0.55 * reach * delta_image[1],
        ]
    )
    delta = np.clip(
        delta,
        -reach * np.array([0.11, 0.21, 0.21]),
        reach * np.array([0.11, 0.21, 0.21]),
    )
    target = np.asarray(neutral_tcp, float) + delta
    shoulder = np.asarray(geometry["base_origin_m"], float) + np.asarray(
        geometry["links"][1]["offset_m"], float
    )
    # Include the neutral point, so acquisition never commands a new position.
    lower = np.minimum(neutral_tcp, [0.08, -0.28 * reach, 0.06])
    upper = np.maximum(
        neutral_tcp, [0.8 * reach, 0.28 * reach, shoulder[2] + 0.55 * reach]
    )
    target = np.clip(target, lower, upper)
    radius = max(
        float(np.linalg.norm(np.asarray(neutral_tcp) - shoulder)), 0.88 * reach
    )
    offset = target - shoulder
    target = shoulder + offset * min(
        1, radius / max(float(np.linalg.norm(offset)), 1e-12)
    )
    return target


class HandMirrorController:
    """A paced latest-sample worker; start and reacquisition use a neutral pose."""

    def __init__(self, arm, sample_callable, *, clock=time.monotonic):
        if not isinstance(arm, SimArm) or not arm.config.simulation:
            raise MotionRejected("left-hand mirroring requires the local SimArm")
        if not callable(sample_callable):
            raise TypeError("a latest-hand-sample callable is required")
        self.arm, self.sample_callable, self.clock = arm, sample_callable, clock
        self.lock = threading.RLock()
        self.thread = None
        self.stopped = threading.Event()
        self.running = False
        self.state = "stopped"
        self.error = None
        self.target = None
        self.neutral = self.neutral_tcp = None
        self.last_heartbeat = 0.0
        self.generation = 0

    def status(self):
        with self.lock:
            value = {
                "running": self.running,
                "state": self.state,
                "target_m": None if self.target is None else self.target.tolist(),
                "error": self.error,
                "can_recenter": self.running and self.neutral is not None,
            }
        with self.arm.lock:
            value.update(tcp_m=self.arm.tcp().tolist(), gripper=float(self.arm.gripper))
        return value

    def start(self):
        with self.lock:
            if self.running:
                raise MotionRejected("left-hand mirroring is already running")
            if self.thread is not None and self.thread.is_alive():
                raise MotionRejected("wait for the previous hand controller to stop")
            self.generation += 1
            self.running = True
            self.state, self.error = "waiting_for_left", None
            self.neutral = self.neutral_tcp = None
            self.last_heartbeat = self.clock()
            self.stopped = threading.Event()
        self.arm.hold()
        with self.arm.lock:
            self.arm.aborted = False
        current = self.arm.tcp()
        with self.lock:
            self.target = current
            self.thread = threading.Thread(
                target=self._run, daemon=True, name="elexhub-hand-control"
            )
            self.thread.start()
        return self.status()

    def heartbeat(self):
        with self.lock:
            if self.running:
                self.last_heartbeat = self.clock()
        return self.status()

    def stop(self):
        with self.lock:
            owned = self.running
            self.running = False
            self.state = "stopped"
            self.generation += 1
            self.stopped.set()
            self.neutral = self.neutral_tcp = None
            thread = self.thread
        if owned:
            self.arm.hold()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=0.3)
        return self.status()

    def close(self):
        return self.stop()

    def recenter(self):
        with self.lock:
            if not self.running:
                raise MotionRejected("start left-hand mirroring before recentering")
            self.generation += 1
            self.neutral = self.neutral_tcp = None
            self.state, self.error = "waiting_for_left", None
        self.arm.hold()
        current = self.arm.tcp()
        with self.lock:
            self.target = current
        return self.status()

    def _guard(self, generation, captured_at):
        with self.lock:
            now = self.clock()
            if (
                not self.running
                or self.generation != generation
                or now - self.last_heartbeat > HEARTBEAT_LEASE_S
                or not 0 <= now - captured_at <= SAMPLE_MAX_AGE_S
            ):
                raise MotionRejected("hand control sample or session expired")

    def _tick(self):
        now = self.clock()
        with self.lock:
            if not self.running:
                return
            generation = self.generation
            if now - self.last_heartbeat > HEARTBEAT_LEASE_S:
                self.running = False
                self.state = "stopped"
                self.error = "control connection lost; start mirroring again"
                self.stopped.set()
                expired = True
            else:
                expired = False
        if expired:
            self.arm.hold()
            return
        sample = accepted_sample(self.sample_callable(), self.clock())
        if sample is None:
            with self.lock:
                if not self.running or self.generation != generation:
                    return
                acquired = self.neutral is not None
                self.neutral = self.neutral_tcp = None
                self.state = (
                    "holding"
                    if acquired or self.state != "waiting_for_left"
                    else "waiting_for_left"
                )
                self.error = None
            self.arm.hold()
            current = self.arm.tcp()
            with self.lock:
                self.target = current
            return
        current = self.arm.tcp()
        with self.lock:
            if not self.running or self.generation != generation:
                return
            if self.neutral is None:
                self.neutral = sample
                self.neutral_tcp = current
                self.target = self.neutral_tcp.copy()
                self.state, self.error = "tracking", None
                return
            desired = relative_target(
                sample, self.neutral, self.neutral_tcp, self.arm.geometry
            )
            # Never accumulate targets far ahead of an arm limited by joint speed.
            delta = desired - current
            distance = float(np.linalg.norm(delta))
            target = current + delta * min(
                1, MAX_TARGET_SPEED_M_S * CONTROL_INTERVAL_S / max(distance, 1e-12)
            )
        try:
            self.arm.teleop_step(
                target,
                1 - sample["pinch"],
                CONTROL_INTERVAL_S,
                guard=lambda: self._guard(generation, sample["timestamp"]),
            )
        except (ValueError, RuntimeError) as exc:
            with self.lock:
                if not self.running or self.generation != generation:
                    return
                self.state, self.error = "rejected", str(exc)
            self.arm.hold()
            current = self.arm.tcp()
            with self.lock:
                self.target = current
        else:
            with self.lock:
                if self.running and self.generation == generation:
                    self.target = target
                    self.state, self.error = "tracking", None

    def _run(self):
        try:
            while not self.stopped.is_set():
                started = time.monotonic()
                self._tick()
                self.stopped.wait(
                    max(0, CONTROL_INTERVAL_S - (time.monotonic() - started))
                )
        except Exception as exc:  # noqa: BLE001 - fail closed on worker faults
            with self.lock:
                self.running = False
                self.state, self.error = "stopped", str(exc)
                self.stopped.set()
            self.arm.hold()
