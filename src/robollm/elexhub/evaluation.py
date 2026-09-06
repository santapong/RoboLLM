"""Frozen paired CPU trials. Simulated results never become physical evidence."""

from __future__ import annotations

import csv
import hashlib
import importlib.metadata
import json
import math
import platform
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np

from .model import mat_to_base
from .simulation import SimArm


def digest(data):
    return hashlib.sha256(
        json.dumps(data, sort_keys=True, allow_nan=False).encode()
    ).hexdigest()


def frozen_suite(count=100, seed=20260906):
    if not 1 <= count <= 10000:
        raise ValueError("trial count must be 1..10000")
    rng = np.random.default_rng(seed)
    # All conditions use exactly these goals, including unreachable ones.
    trials = [
        {
            "seed": seed + i,
            "target_mat_m": [
                float(rng.uniform(0.025, 0.155)),
                float(rng.uniform(0.045, 0.105)),
            ],
            "height_m": float(rng.uniform(0.065, 0.105)),
        }
        for i in range(count)
    ]
    return {
        "schema_version": 1,
        "seed": seed,
        "trials": trials,
        "success_threshold_m": 0.015,
        "dwell_s": 0.5,
        "conditions": ["A", "B", "C"],
        "C_observation": "simulator TCP (oracle); no camera noise or hardware evidence",
    }


def wilson(success, total):
    if not total:
        return [None, None]
    z = 1.959963984540054
    p = success / total
    den = 1 + z * z / total
    mid = (p + z * z / (2 * total)) / den
    half = z * math.sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / den
    return [max(0, mid - half), min(1, mid + half)]


def summarize(rows):
    output = {}
    for condition in "ABC":
        selected = [r for r in rows if r["condition"] == condition]
        errors = [r["error_m"] for r in selected if r["error_m"] is not None]
        count = len(selected)
        success = sum(r["success"] for r in selected)
        output[condition] = {
            "n": count,
            "successes": success,
            "success_rate": success / count if count else None,
            "wilson_95": wilson(success, count),
            "failure_count": count - success,
            "median_error_m_completed_only": float(np.median(errors))
            if errors
            else None,
            "p90_error_m_completed_only": float(np.quantile(errors, 0.9))
            if errors
            else None,
            "aborted": sum(r["failure_reason"] is not None for r in selected),
        }
    paired = {}
    by = {(r["seed"], r["condition"]): r for r in rows}
    seeds = sorted({r["seed"] for r in rows})
    for first, second in [("A", "B"), ("B", "C"), ("A", "C")]:
        differences = np.asarray(
            [
                int(by[s, second]["success"]) - int(by[s, first]["success"])
                for s in seeds
                if (s, first) in by and (s, second) in by
            ]
        )
        if len(differences):
            rng = np.random.default_rng(913)
            boot = np.mean(rng.choice(differences, (5000, len(differences))), axis=1)
            paired[f"{second}-{first}"] = {
                "paired_n": len(differences),
                "success_rate_difference": float(differences.mean()),
                "paired_bootstrap_95": np.quantile(boot, [0.025, 0.975]).tolist(),
            }
    return {
        "conditions": output,
        "paired": paired,
        "physical_evidence": False,
        "interpretation": "Internal simulation validation with provisional dimensions and actuator parameters. C uses oracle TCP feedback.",
    }


def evaluate(config, directory, suite, *, progress=print):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    if (directory / "trials.jsonl").exists():
        raise ValueError(
            "result directory already contains trials; choose a new directory"
        )

    def git_value(*args):
        try:
            return subprocess.check_output(
                ["git", *args], text=True, stderr=subprocess.DEVNULL
            ).strip()
        except (FileNotFoundError, subprocess.CalledProcessError):
            return None

    manifest = {
        "suite": suite,
        "suite_sha256": digest(suite),
        "profile_hash": config.profile_hash,
        "profile": config.profile_data,
        "python": platform.python_version(),
        "versions": {
            p: importlib.metadata.version(p)
            for p in ["numpy", "mujoco", "mink", "opencv-contrib-python"]
        },
        "git_commit": git_value("rev-parse", "HEAD"),
        "git_working_tree_dirty": bool(git_value("status", "--porcelain")),
        "domain": "provisional_simulation",
        "physical_evidence": False,
    }
    root = Path(__file__).resolve().parents[3]
    snapshots = {}
    sources = sorted((root / "src/robollm/elexhub").glob("*.py")) + sorted(
        (root / "ros2/robo_arm_driver/robo_arm_driver").glob("*.py")
    )
    for source in sources:
        relative = source.relative_to(root)
        target = directory / "source" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        snapshots[str(relative)] = hashlib.sha256(source.read_bytes()).hexdigest()
    manifest["source_sha256"] = snapshots
    (directory / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    arm = SimArm(config, directory / "model")
    rows = []
    try:
        with (directory / "trials.jsonl").open("w") as stream:
            for trial in suite["trials"]:
                # Rotate execution order without changing any scene/seed pairing.
                offset = trial["seed"] % 3
                order = list("ABC")
                order = order[offset:] + order[:offset]
                for condition in order:
                    arm.reset()
                    target = mat_to_base(
                        config, trial["target_mat_m"], trial["height_m"]
                    )
                    start = time.monotonic()
                    error = None
                    reason = None
                    success = False
                    dwell_errors = []

                    def record(obs, dwell_errors=dwell_errors, target=target):
                        dwell_errors.append(
                            (
                                obs["time_sim_s"],
                                float(
                                    np.linalg.norm(
                                        np.asarray(obs["tcp_position_m"]) - target
                                    )
                                ),
                            )
                        )

                    try:
                        arm.reach(target, condition, callback=record)
                        # Explicitly sample a held goal for 0.5 simulation seconds.
                        samples = []
                        for _ in range(
                            round(suite["dwell_s"] / arm.model.opt.timestep)
                        ):
                            arm._command(arm.last_command, compensate=condition != "A")
                            if arm.collisions():
                                raise ValueError("collision during verification dwell")
                            samples.append(float(np.linalg.norm(arm.tcp() - target)))
                        error = float(np.linalg.norm(arm.tcp() - target))
                        success = max(samples) <= suite["success_threshold_m"]
                    except Exception as exc:  # noqa: BLE001 - preserve every failed trial
                        reason = f"{type(exc).__name__}: {exc}"
                    row = {
                        **trial,
                        "condition": condition,
                        "target_base_m": target.tolist(),
                        "success": bool(success),
                        "error_m": error,
                        "failure_reason": reason,
                        "runtime_s": time.monotonic() - start,
                        "physical_evidence": False,
                        "observation_source": "simulator",
                        "profile_hash": config.profile_hash,
                    }
                    stream.write(json.dumps(row, allow_nan=False) + "\n")
                    stream.flush()
                    rows.append(row)
                if len(rows) % 30 == 0:
                    progress(
                        f"{len(rows)}/{len(suite['trials']) * 3} paired trials recorded",
                        flush=True,
                    )
    finally:
        arm.close()
    summary = summarize(rows)
    (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    lines = [
        "# Elexhub synthetic comparison",
        "",
        summary["interpretation"],
        "",
        "Every rejection remains a failed trial. Error quantiles cover completed trials only.",
        "",
        "|Condition|Success / attempted|Wilson 95% interval|Median completed error (mm)|Aborted|",
        "|---|---:|---:|---:|---:|",
    ]
    for c, v in summary["conditions"].items():
        lo, hi = v["wilson_95"]
        err = v["median_error_m_completed_only"]
        lines.append(
            f"|{c}|{v['successes']} / {v['n']}|{lo:.3f}–{hi:.3f}|{err * 1000:.2f}|{v['aborted']}|"
            if err is not None
            else f"|{c}|{v['successes']} / {v['n']}|{lo:.3f}–{hi:.3f}|unavailable|{v['aborted']}|"
        )
    lines += [
        "",
        "A: nominal servo model. B: provisional gravity compensation. C: B plus bounded oracle TCP correction.",
        "",
        "These are software checks, not validation of the Elexhub hardware or an RGB feedback experiment.",
    ]
    (directory / "REPORT.md").write_text("\n".join(lines) + "\n")
    return summary


def physical_templates(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    rows = []
    # 12 fixed targets x 3 repetitions x 3 conditions, with balanced ordering.
    for target in range(12):
        xy = [0.04 + 0.035 * (target % 4), 0.055 + 0.025 * (target // 4)]
        for repetition in range(3):
            for c in "ABC"[repetition:] + "ABC"[:repetition]:
                rows.append(
                    {
                        "trial_id": f"reach-{target:02d}-{repetition}-{c}",
                        "task": "reach",
                        "condition": c,
                        "target_x_m": xy[0],
                        "target_y_m": xy[1],
                        "height_m": 0.08,
                    }
                )
    rows += [
        {"trial_id": f"intent-{i:02d}", "task": "intent", "condition": "interface"}
        for i in range(30)
    ]
    rows += [
        {"trial_id": f"pickplace-{i:02d}", "task": "pick_place", "condition": "C"}
        for i in range(20)
    ]
    fields = [
        "trial_id",
        "task",
        "condition",
        "target_x_m",
        "target_y_m",
        "height_m",
        "profile_hash",
        "calibration_id",
        "success",
        "error_m",
        "failure_reason",
        "recording_path",
        "instruction",
        "expected_object_id",
        "selected_object_id",
        "selected_target_x_m",
        "selected_target_y_m",
        "selection_latency_s",
        "clarification_count",
        "time_to_success_s",
        "abort_stage",
    ]
    path = directory / "physical-trials-template.csv"
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    (directory / "physical-protocol.json").write_text(
        json.dumps(
            {
                "status": "unexecuted",
                "physical_evidence": False,
                "reach_attempts": 108,
                "intent_attempts": 30,
                "pick_place_attempts": 20,
                "prerequisites": [
                    "verified hardware profile",
                    "power/cutoff and watchdog checks",
                    "camera registration",
                    "condition B/C actuator identification",
                    "same goals and starting poses across A/B/C",
                ],
                "metrics": {
                    "reach": "3D TCP <=15 mm continuously for 0.5 s, camera measured",
                    "intent": "source/destination agreement before motion, including clarification rate",
                    "pick_place": "verified lift >35 mm and released object XY <=20 mm, stable for 0.5 s",
                },
            },
            indent=2,
        )
        + "\n"
    )
    return path
