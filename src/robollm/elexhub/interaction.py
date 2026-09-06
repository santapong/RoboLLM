"""Small deictic command vocabulary, explicit preview, and observable outcomes."""

from __future__ import annotations

import hashlib
import json
import math
import re
import time
from dataclasses import asdict, dataclass

import numpy as np

from .model import mat_to_base
from .simulation import MotionRejected


class IntentError(ValueError):
    pass


@dataclass(frozen=True)
class Intent:
    action: str
    object_id: str | None
    destination_mat_m: tuple[float, float]
    calibration_id: str
    instruction: str


def parse_instruction(
    text, destination, *, object_id="red_box_0", calibration_id="simulation"
):
    if not isinstance(text, str) or len(text) > 160:
        raise IntentError("use a short reach or move instruction")
    normalized = " ".join(re.findall(r"[a-z]+", text.lower()))
    if normalized in {"reach here", "reach there", "go here", "point here"}:
        action = "reach"
        object_id = None
    elif normalized in {
        "move this red box here",
        "move the red box here",
        "move this here",
        "move red box here",
        "put this here",
        "put the red box here",
    }:
        action = "pick_place"
    else:
        raise IntentError('Supported: "reach here" or "move this red box here"')
    try:
        xy = tuple(float(v) for v in destination)
    except (TypeError, ValueError):
        raise IntentError("select a destination on the mat") from None
    if (
        len(xy) != 2
        or not all(math.isfinite(v) for v in xy)
        or not 0 <= xy[0] <= 0.18
        or not 0 <= xy[1] <= 0.12
    ):
        raise IntentError("destination must be inside the 180 x 120 mm grid")
    if action == "pick_place" and not object_id:
        raise IntentError("select one object first")
    return Intent(action, object_id, xy, calibration_id, text)


def select_pointed_object(objects, xy, max_distance_m=0.035):
    candidates = sorted(
        (float(np.linalg.norm(np.asarray(o["position_mat_m"])[:2] - xy)), o["id"])
        for o in objects
    )
    if not candidates or candidates[0][0] > max_distance_m:
        raise IntentError("point closer to the object")
    if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < 0.015:
        raise IntentError("selection ambiguous; separate the objects or point closer")
    return candidates[0][1]


def require_fresh(observation, *, now=None, calibration_id=None, require_tool=False):
    now = time.monotonic() if now is None else now
    stamp = observation.get("captured_at_monotonic")
    if (
        not isinstance(stamp, (float, int))
        or not math.isfinite(stamp)
        or not 0 <= now - stamp <= 0.30
    ):
        raise MotionRejected("camera observation is stale")
    if not observation.get("reference_visible"):
        raise MotionRejected("workspace reference is missing")
    if (
        calibration_id is not None
        and observation.get("calibration_id") != calibration_id
    ):
        raise MotionRejected("calibration changed since preview")
    if observation.get("hand_in_workspace", True):
        raise MotionRejected("clear your hands from the workspace")
    if require_tool and observation.get("tcp_mat_m") is None:
        raise MotionRejected("tool marker is not visible")


def waypoints(config, intent, source_base=None):
    destination = mat_to_base(config, intent.destination_mat_m, 0.080)
    if intent.action == "reach":
        return [("REACH", destination, False)]
    if source_base is None:
        raise IntentError("the selected object is no longer visible")
    source = np.asarray(source_base, float)
    if source.shape != (3,) or not np.isfinite(source).all():
        raise IntentError("invalid object position")
    clearance = 0.050
    grasp = max(0.012, float(source[2]))
    return [
        ("APPROACH", np.r_[source[:2], clearance], True),
        ("DESCEND", np.r_[source[:2], grasp], True),
        ("GRASP", None, True),
        ("LIFT", np.r_[source[:2], clearance], True),
        ("TRANSFER", np.r_[destination[:2], clearance], True),
        ("LOWER", np.r_[destination[:2], 0.026], True),
        ("RELEASE", None, True),
        ("RETREAT", np.r_[destination[:2], clearance], True),
        ("VERIFY", None, False),
    ]


class TaskExecutor:
    def __init__(self, arm, *, observe=None, record=None, mode="sim"):
        if mode not in {"sim", "shadow", "hardware"}:
            raise ValueError("unknown execution mode")
        self.arm, self.observe, self.record, self.mode = arm, observe, record, mode
        self.stage = "IDLE"
        self.previewed = None
        self.busy = False
        self.history = []

    def emit(self, kind, **fields):
        entry = {
            "kind": kind,
            "stage": self.stage,
            "time_monotonic": time.monotonic(),
            "mode": self.mode,
            **fields,
        }
        self.history.append(entry)
        if self.record:
            self.record(entry)

    def preview(self, intent, source_base=None):
        if self.busy:
            raise IntentError("a task is already executing")
        if self.mode in {"hardware", "shadow"}:
            observation = self.observe()
            # Hands may be present while selecting; execution has its own clear-workspace gate.
            require_fresh(
                {**observation, "hand_in_workspace": False},
                calibration_id=intent.calibration_id,
            )
        if hasattr(self.arm, "sync_preview"):
            self.arm.sync_preview()
        if source_base is None and intent.action == "pick_place":
            source_base = self.arm.object_position()
        points = waypoints(self.arm.config, intent, source_base)
        start = self.arm.last_command.copy()
        solved = []
        for stage, target, downward in points:
            if target is None:
                continue
            q = self.arm.solve(
                target,
                start=start,
                downward=downward,
                allow_grasp=stage
                in {"DESCEND", "LIFT", "TRANSFER", "LOWER", "RETREAT"},
            )
            self.arm.check_path(
                q,
                start=start,
                allow_grasp=stage
                in {"DESCEND", "LIFT", "TRANSFER", "LOWER", "RETREAT"},
            )
            solved.append(
                {"stage": stage, "target_base_m": target.tolist(), "q_rad": q.tolist()}
            )
            start = q
        payload = {
            "intent": asdict(intent),
            "source_base_m": None
            if source_base is None
            else np.asarray(source_base).tolist(),
            "profile_hash": self.arm.config.profile_hash,
            "start_q_rad": self.arm.last_command.tolist(),
            "trajectory": solved,
            "created_at": time.monotonic(),
        }
        token = hashlib.sha256(
            json.dumps(payload, sort_keys=True).encode()
        ).hexdigest()[:24]
        self.previewed = (token, intent, points, payload)
        self.stage = "PREVIEW"
        self.emit("preview", **payload, preview_id=token)
        return {**payload, "preview_id": token, "mode": self.mode}

    def execute(self, preview_id, *, workspace_clear=False, condition="C"):
        if self.busy:
            raise IntentError("a task is already executing")
        if not self.previewed or preview_id != self.previewed[0]:
            raise IntentError("preview is missing or changed")
        if not workspace_clear:
            raise IntentError("confirm the workspace is clear")
        token, intent, points, payload = self.previewed
        if time.monotonic() - payload["created_at"] > 30:
            raise IntentError("preview expired; create a new preview")
        if condition not in {"A", "B", "C"}:
            raise IntentError("unknown control condition")
        if self.mode == "shadow":
            raise IntentError("shadow mode never executes motion")
        if hasattr(self.arm, "sync_preview"):
            self.arm.sync_preview()
        if np.max(np.abs(self.arm.last_command - payload["start_q_rad"])) > 0.02:
            raise IntentError("arm starting state changed; create a new preview")
        self.previewed = None
        self.busy = True
        self.arm.aborted = False
        lifted = False

        def guard():
            if self.arm.aborted:
                raise MotionRejected("operator aborted the task")
            if self.mode == "hardware":
                require_fresh(
                    self.observe(),
                    calibration_id=intent.calibration_id,
                    require_tool=condition == "C",
                )

        try:
            guard()
            if (
                intent.action == "pick_place"
                and np.linalg.norm(
                    self.arm.object_position() - payload["source_base_m"]
                )
                > 0.005
            ):
                raise MotionRejected("object moved since preview")
            for stage, target, downward in points:
                self.stage = stage
                self.emit("stage", preview_id=token)
                guard()
                if stage == "GRASP":
                    self.arm.set_gripper(0.70, guard=guard)
                elif stage == "RELEASE":
                    self.arm.set_gripper(0, guard=guard)
                elif target is not None:
                    self.arm.reach(
                        target,
                        condition,
                        downward=downward,
                        allow_grasp=stage
                        in {"DESCEND", "LIFT", "TRANSFER", "LOWER", "RETREAT"},
                        guard=guard,
                        callback=lambda o: self.emit("observation", **o),
                    )
                    if stage == "LIFT":
                        lifted = self.arm.object_position()[2] > 0.035
                        if not lifted:
                            raise MotionRejected("grasp failed: object did not lift")
            dest = mat_to_base(
                self.arm.config,
                intent.destination_mat_m,
                self.arm.geometry["object"]["edge_m"] / 2,
            )
            errors = []
            success = True
            for tcp, obj in self.arm.verification_samples(
                condition=condition, guard=guard
            ):
                if intent.action == "reach":
                    error = float(np.linalg.norm(tcp - points[0][1]))
                    success = success and error <= 0.015
                else:
                    error = float(np.linalg.norm(obj[:2] - dest[:2]))
                    success = (
                        success and lifted and error <= 0.020 and 0 < obj[2] < 0.025
                    )
                errors.append(error)
            error = max(errors)
            self.stage = "COMPLETE" if success else "FAILED"
            result = {
                "success": bool(success),
                "error_m": error,
                "lift_verified": bool(lifted),
                "condition": condition,
                "verification_dwell_s": 0.5,
                "observation_source": "simulator"
                if self.mode != "hardware"
                else "webcam",
                "physical_evidence": self.mode == "hardware",
            }
            self.emit("result", **result)
            return result
        except Exception as exc:
            self.arm.abort()
            self.stage = "ABORTED"
            self.emit("failure", reason=str(exc))
            raise
        finally:
            self.busy = False

    def abort(self):
        self.arm.abort()
        self.previewed = None
        self.stage = "ABORTED"
        self.emit("operator_abort")
