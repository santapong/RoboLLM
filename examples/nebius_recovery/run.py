"""CPU simulation: reach a workpiece, notice displacement, recover once or hold.

The observation is simulator telemetry, not visual perception. The NVIDIA model
selects a symbolic action; local code owns all coordinates, IK and motor control.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import platform
import random
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path


SCENARIOS = ("nominal", "displaced", "baseline", "stale", "out_of_bounds")
MAX_AGE_S = 0.5
MAX_CALLS = 2
MAX_RECOVERIES = 1
REACH_TOLERANCE_M = 0.035
DT = 0.04


class GuardError(ValueError):
    """A requested motion was rejected before reaching the controller."""


def validate_motion(decision: dict, observation: dict, *, current_version: int,
                    recoveries: int) -> list[float] | None:
    """Validate the symbolic decision against an immutable, paused-world snapshot.

    ``hold`` is always motion-free. The scene may be unsafe or unavailable when
    holding. No clamp is allowed here: unreachable metadata is rejected whole.
    """
    from .provider import validate_decision

    validate_decision(decision)
    if decision["action"] == "hold":
        return None
    safety = observation.get("state", {}).get("safety", {})
    violation = safety.get("joint_limit_violation_rad")
    if (safety.get("finite") is not True or safety.get("fault") is not None
            or isinstance(violation, bool) or not isinstance(violation, (int, float))
            or not math.isfinite(violation) or violation > 0.001):
        raise GuardError("Dynamics telemetry is unsafe")
    scene = observation["scene"]
    if scene.get("target_id") != "workpiece":
        raise GuardError("Unknown observed target")
    if type(scene.get("version")) is not int or scene["version"] != current_version:
        raise GuardError("Scene version changed or is invalid")
    age = scene.get("age_s")
    if isinstance(age, bool) or not isinstance(age, (int, float)) or not math.isfinite(age) or not 0 <= age <= MAX_AGE_S:
        raise GuardError("Observation is stale or has invalid age")
    target = scene.get("position")
    if not isinstance(target, list) or len(target) != 3 or any(
        isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in target
    ):
        raise GuardError("Observed target needs three finite coordinates")
    workspace = observation["workspace"]
    if any(not lo <= value <= hi for value, lo, hi in zip(target, workspace["min"], workspace["max"])):
        raise GuardError("Observed target is outside the allowed workspace")
    if observation["event"] == "target_moved" and decision["action"] != "reobserve":
        raise GuardError("A moved target requires an explicit reobserve decision")
    if decision["action"] == "reobserve":
        if observation["event"] != "target_moved" or recoveries >= MAX_RECOVERIES:
            raise GuardError("Recovery is unavailable or exhausted")
        if observation.get("retries_remaining", 0) <= 0:
            raise GuardError("Recovery budget exhausted")
    return [float(x) for x in target]


def _distance(a, b) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def _git_revision() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=Path(__file__).resolve().parents[2], text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _source_hash() -> str:
    digest = hashlib.sha256()
    model = Path(__file__).resolve().parents[1] / "humanoid_harness" / "models" / "concept01"
    root = Path(__file__).resolve().parents[2]
    paths = [Path(__file__), Path(__file__).with_name("provider.py"),
             model.parents[1] / "simulation.py", model / "model.xml", model / "model.json",
             *sorted((model / "meshes").glob("*.obj"))]
    for path in paths:
        digest.update(str(path.resolve().relative_to(root)).encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _source_dirty() -> bool | None:
    try:
        return bool(subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=normal"], cwd=Path(__file__).resolve().parents[2], text=True).strip())
    except (OSError, subprocess.CalledProcessError):
        return None


def _render_frame(sim, observed: list[float], panel: dict, path: Path) -> None:
    """Render true MuJoCo state plus a green observed-target marker and telemetry."""
    from PIL import Image, ImageDraw, ImageFont

    # Both green observation and orange command markers are real scene bodies.
    image = Image.fromarray(sim.render())
    canvas = Image.new("RGB", (image.width + 340, image.height), "#0b1321")
    canvas.paste(image, (0, 0))
    draw = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 17)
        title_font = ImageFont.truetype("DejaVuSans.ttf", 22)
    except OSError:
        font = title_font = ImageFont.load_default()
    x = image.width + 20
    draw.text((x, 22), "RoboLLM / Recovery", fill="#dce7f8", font=title_font)
    lines = [
        "SUPPORTED HUMANOID / MuJoCo", "", f"Scenario: {panel['scenario']}",
        f"Provider: {panel['provider']}", f"Sim time: {panel['time']:.2f} s",
        f"Controller: {panel['mode']}", f"Event: {panel['event']}",
        f"Decision: {panel['decision']}", "", f"Observed error: {panel['error_m']*1000:.1f} mm",
        f"Target version: {panel['version']}", f"Model calls: {panel['calls']} / {MAX_CALLS}",
        f"Recoveries: {panel['recoveries']} / 1", "", "Green = observed workpiece",
        "Orange = commanded target", "", "Telemetry observation",
        "Classical IK + joint dynamics", "Fixed pelvis / no grasping",
        "No visual perception / walking", "Self-collision disabled",
    ]
    if panel["provider"] == "fixture":
        lines.extend(["", "OFFLINE FIXTURE", "No cloud inference in this run"])
    for index, line in enumerate(lines):
        color = "#f5bd63" if "FIXTURE" in line else "#c2cedd"
        draw.text((x, 70 + index * 25), line, fill=color, font=font)
    canvas.save(path)


def _scene_simulation(sim_factory, output: Path):
    """Copy the original local model, adding one non-colliding observed body."""
    directory = Path(__file__).resolve().parents[1] / "humanoid_harness" / "models" / "concept01"
    root = ET.parse(directory / "model.xml").getroot()
    root.find("compiler").set("meshdir", str((directory / "meshes").resolve()))
    body = ET.SubElement(root.find("worldbody"), "body", name="observed_workpiece", mocap="true", pos="0.32 -0.31 1.13")
    ET.SubElement(body, "geom", name="observed_workpiece_marker", type="sphere", size="0.032", rgba="0.1 0.95 0.5 0.75", contype="0", conaffinity="0", group="0")
    runtime_model = output / "runtime-scene.xml"
    runtime_model.write_text(ET.tostring(root, encoding="unicode") + "\n")
    metadata = json.loads((directory / "model.json").read_text())
    return sim_factory(model_name="concept01", custom_model_path=runtime_model,
                       custom_metadata=metadata, width=960, height=720)


class SceneSensor:
    """Versioned simulator observation, read from the actual MuJoCo target body.

    ``stale`` explicitly simulates a sensor whose acquisition time never updates.
    The target injector owns version changes; it is not a learned perception model.
    """

    def __init__(self, sim, position: list[float], *, stale: bool = False):
        self.sim = sim
        self.version = 1
        self.stale = stale
        self._mocap = int(sim.model.body("observed_workpiece").mocapid[0])
        self.move(position, increment=False)

    def move(self, position: list[float], *, increment: bool = True):
        import mujoco
        self.sim.data.mocap_pos[self._mocap] = position
        if increment:
            self.version += 1
        mujoco.mj_forward(self.sim.model, self.sim.data)

    def observe(self) -> dict:
        return {"target_id": "workpiece", "position": self.sim.data.body("observed_workpiece").xpos.tolist(),
                "version": self.version, "age_s": 2.0 if self.stale else 0.0,
                "sample_sim_time_s": float(self.sim.data.time),
                "source": "MuJoCo observed_workpiece body telemetry"}


def run_scenario(scenario: str, provider, output: str | Path, *, seed: int = 0,
                 render: bool = False, steps: int = 250, sim_factory=None) -> dict:
    """Execute one bounded experiment, always persisting outcome and decisions.

    During a model request the simulation is held and time does not advance.
    This is explicit simulation snapshot freshness, not a claim about physical
    sensors remaining fresh across network latency.
    """
    if scenario not in SCENARIOS:
        raise ValueError(f"Unknown scenario: {scenario}")
    if steps < 150:
        raise ValueError("At least six simulated seconds are needed for evaluation")
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    import numpy as np
    if sim_factory is None:
        from examples.humanoid_harness.simulation import HumanoidSimulation
        sim_factory = HumanoidSimulation
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    frames = output / "frames"
    if render:
        frames.mkdir(exist_ok=True)
        if any(frames.iterdir()):
            raise ValueError("Use an empty output frames directory to prevent stale evidence")
    rng = random.Random(seed)
    sim = _scene_simulation(sim_factory, output)
    events: list[dict] = []
    trajectory: list[dict] = []
    step_safety: list[dict] = []
    evidence: list[dict] = []
    calls = recoveries = reach_commands = 0
    last_event, last_decision = "initial", "hold"
    errors: list[str] = []
    frame_number = 0
    initial_state = sim.state()
    workspace = copy.deepcopy(sim.workspace["right"])
    original_target = list(workspace["default"])
    # Held-out seeds vary the target slightly while preserving the same test.
    original_target[0] += rng.uniform(-0.008, 0.008)
    original_target[2] += rng.uniform(-0.008, 0.008)
    observed_target = original_target.copy()
    if scenario == "out_of_bounds":
        observed_target[0] = workspace["max"][0] + 0.20
    sensor = SceneSensor(sim, observed_target, stale=scenario == "stale")
    commanded_version = 0
    monitored_version = 1
    if scenario in ("stale", "out_of_bounds"):
        last_event = scenario
    started = time.monotonic()
    created_at = datetime.now(timezone.utc).isoformat()

    def event(kind: str, **fields):
        events.append({"sim_time_s": sim.state()["time"], "kind": kind, **fields})

    def advance(command=None):
        state = sim.step(command)
        step_safety.append(copy.deepcopy(state["safety"]))
        return state

    def decide(event_name: str) -> None:
        nonlocal calls, recoveries, reach_commands, last_decision, commanded_version
        # Force hold before transport; blocking provider work cannot advance
        # physics or mutate the scene. No credentials are put into observations.
        held = advance({"mode": "hold"})
        snapshot = {"event": event_name, "scene": sensor.observe(),
                    "state": held, "workspace": workspace,
                    "retries_remaining": MAX_RECOVERIES - recoveries,
                    "world_paused_during_inference": True}
        if calls >= MAX_CALLS:
            errors.append("Model call budget exhausted")
            event("guard_rejected", reason=errors[-1])
            return
        calls += 1
        try:
            decision, audit = provider.decide(copy.deepcopy(snapshot))
            evidence.append(audit)
            # Reacquire actual scene telemetry after transport. In this single
            # owner simulation no physics steps occur during the blocking call.
            # Confirm identity/version/position before accepting a model choice.
            fresh_scene = sensor.observe()
            if (fresh_scene["version"] != snapshot["scene"]["version"]
                    or fresh_scene["position"] != snapshot["scene"]["position"]):
                raise GuardError("Scene changed during inference")
            confirmation = {**snapshot, "scene": fresh_scene, "state": sim.state()}
            target = validate_motion(decision, confirmation, current_version=sensor.version, recoveries=recoveries)
            last_decision = decision["action"]
            event("decision", observation=snapshot, decision=decision, evidence=audit)
            if target is None:
                advance({"mode": "hold"})
                return
            if decision["action"] == "reobserve":
                # Simulated re-observation confirms the current scene version;
                # this is not an image detector or new external sensor reading.
                recoveries += 1
                event("reobservation_confirmed", scene=fresh_scene)
            advance({"mode": "reach", "arm": "right", "target": target})
            reach_commands += 1
            commanded_version = sensor.version
            event("reach_issued", target=target, scene_version=sensor.version)
        except Exception as exc:
            # Do not serialize raw provider exceptions: they may include a
            # response body or transport headers. Error class is enough here.
            errors.append(f"Decision rejected: {type(exc).__name__}")
            last_decision = "rejected / hold"
            advance({"mode": "hold"})
            event("decision_failed", error_type=type(exc).__name__, action="hold")

    try:
        decide(last_event)
        moved = False
        for index in range(steps):
            state = advance()
            if scenario in ("displaced", "baseline") and not moved and state["time"] >= 1.2:
                observed_target = [original_target[0] + 0.035, original_target[1] - 0.065, original_target[2] + 0.09]
                sensor.move(observed_target)
                moved = True
                event("target_displaced", observed_target=observed_target, old_commanded_target=state["target"], displacement_m=_distance(observed_target, original_target))
            # Independent execution monitor reads the world after injection.
            # New scene versions plus a >4cm goal mismatch invalidate the plan.
            sensed = sensor.observe()
            mismatch = _distance(sensed["position"], state["target"])
            if sensed["version"] != monitored_version and sensed["version"] != commanded_version and mismatch > 0.04:
                monitored_version = sensed["version"]
                last_event = "target_moved"
                event("execution_monitor_target_mismatch", mismatch_m=mismatch,
                      observed_scene_version=sensed["version"], commanded_scene_version=commanded_version)
                if scenario != "baseline":
                    decide("target_moved")
                    state = sim.state()
                else:
                    event("baseline_ignores_displacement")
            safety = state["safety"]
            error = _distance(state["hand_position"], sensed["position"])
            row = {"time_s": state["time"], "hand_position": state["hand_position"],
                   "observed_target": sensed["position"], "commanded_target": state["target"],
                   "observed_error_m": error, "mode": state["mode"], "scene_version": sensor.version,
                   "frame_index": frame_number if render and index % 2 == 0 else None,
                   "joint_limit_violation_rad": safety["joint_limit_violation_rad"],
                   "finite": safety["finite"], "fault": safety["fault"]}
            trajectory.append(row)
            if not safety["finite"] or safety["fault"] or safety["joint_limit_violation_rad"] > 0.001:
                errors.append("Dynamics safety monitor tripped")
                advance({"mode": "hold"})
                event("dynamics_monitor_hold")
                break
            if render and index % 2 == 0:
                _render_frame(sim, sensed["position"], {"scenario": scenario,
                    "provider": getattr(provider, "label", "unknown"), "time": state["time"],
                    "mode": state["mode"], "event": last_event, "decision": last_decision,
                    "error_m": error, "version": sensor.version, "calls": calls, "recoveries": recoveries},
                    frames / f"frame-{frame_number:05d}.png")
                frame_number += 1
        final = sim.state()
        final_error = _distance(final["hand_position"], sensor.observe()["position"])
        finite = all(row["finite"] for row in step_safety)
        max_violation = max((row["joint_limit_violation_rad"] for row in step_safety), default=float("inf"))
        no_fault = all(row["fault"] is None for row in step_safety)
        dynamics_ok = finite and no_fault and max_violation <= 0.001
        held_safely = final["mode"] == "hold" and reach_commands == 0 and dynamics_ok
        terminal_max_error = max((row["observed_error_m"] for row in trajectory[-10:]), default=float("inf"))
        task_success = terminal_max_error < REACH_TOLERANCE_M and final["mode"] == "reach" and dynamics_ok and not errors
        if scenario == "baseline":
            acceptance_pass = not task_success and final_error > REACH_TOLERANCE_M and dynamics_ok and not errors
        elif scenario in ("stale", "out_of_bounds"):
            acceptance_pass = held_safely and not errors
        else:
            acceptance_pass = task_success and (scenario != "displaced" or recoveries == 1)
        report = {"experiment": "supported humanoid execution monitoring and bounded recovery",
            "created_at_utc": created_at, "scenario": scenario, "seed": seed,
            "provider": getattr(provider, "label", "unknown"),
            "live_nebius_inference_verified": bool(evidence) and all(item.get("provider") == "nebius-live" for item in evidence) and not errors,
            "source_commit": _git_revision(), "source_sha256": _source_hash(),
            "source_tree_dirty": _source_dirty(),
            "python_version": platform.python_version(), "model_id": initial_state["model_id"],
            "mujoco_version": mujoco.__version__, "numpy_version": np.__version__,
            "model_sha256": initial_state["model_sha256"], "simulation_scope": initial_state["scope"],
            "limitations": ["supported pelvis; no balance or walking", "simulator telemetry; no image perception", "classical IK; no VLA/WAM policy", "no grasping", "estimated physical parameters; self-collision disabled", "simulation paused during model request; no real-time physical freshness claim"],
            "success_conditions": {"reach_tolerance_m": REACH_TOLERANCE_M, "max_joint_violation_rad": 0.001,
                "max_model_calls": MAX_CALLS, "max_recoveries": MAX_RECOVERIES, "max_scene_age_s": MAX_AGE_S},
            "task_success": task_success, "safe_hold": held_safely, "acceptance_pass": acceptance_pass,
            "final_observed_error_m": final_error, "final_mode": final["mode"],
            "terminal_max_observed_error_m": terminal_max_error, "terminal_window_samples": min(10, len(trajectory)),
            "max_joint_limit_violation_rad": max_violation, "all_finite": finite, "no_dynamics_fault": no_fault,
            "model_calls": calls, "recoveries": recoveries, "reach_commands": reach_commands,
            "errors": errors, "model_evidence": evidence, "simulated_seconds": final["time"],
            "wall_seconds": time.monotonic() - started, "rendered_frames": frame_number,
            "frame_rate": 1 / (2 * DT), "frame_note": "Physics motion at stated simulation frame rate; provider request wall time is excluded"}
        report["paused_world_during_inference"] = True
        report["safety_sampling"] = "Controller step endpoints (40 ms), not continuous-time collision certification"
        report["safety_samples"] = len(step_safety)
        (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        (output / "events.jsonl").write_text("".join(json.dumps(row, allow_nan=False) + "\n" for row in events))
        (output / "trajectory.jsonl").write_text("".join(json.dumps(row, allow_nan=False) + "\n" for row in trajectory))
        return report
    finally:
        sim.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--provider", choices=("fixture", "nebius"), required=True)
    parser.add_argument("--scenario", choices=SCENARIOS, default="displaced")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path, default=Path("artifacts/nebius/displaced"))
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--steps", type=int, default=250)
    args = parser.parse_args()
    from .provider import FixtureProvider, NebiusProvider, ProviderError
    try:
        provider = FixtureProvider() if args.provider == "fixture" else NebiusProvider()
    except ProviderError as exc:
        print(f"Cannot start live provider: {exc}. No simulation or fallback was run.", file=sys.stderr)
        return 2
    report = run_scenario(args.scenario, provider, args.output, seed=args.seed, render=args.render, steps=args.steps)
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["acceptance_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
