"""Phase 1 gate (SDD §8): the scripted experts on the frozen evaluation schedule.

- Oracle over 20 cells × 5 seeds = 100 evaluation episodes: SR 100 %, Safety 100 %.
- Noisy expert (σ = 0.5 × per-step limit) on the same 100: SR ≥ 95 %, every clean
  label passes S1–S5, trajectory-length ratio vs oracle reported.

Writes ``results/p1/<host>/summary.json`` (schema robollm.vla-bed.phase-summary.v1)
and three sample frames. Exit code 1 on FAIL.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import socket
import sys
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import families  # noqa: E402
from env import FPS, INSTRUCTION, MAX_FRAMES, BedEnv, run_episode  # noqa: E402
from expert import DEFAULT_NOISE_FRACTION, STEP_LIMITS, make_expert  # noqa: E402
from safety import SafetyWrapper, quadruple  # noqa: E402
from scene import build_scene  # noqa: E402
from stats import bootstrap_mean, wilson_interval  # noqa: E402

EVAL_SEED = 70_000  # B1's frozen-suite base seed
EPISODES = 100


def summarize(rows: list[dict]) -> dict:
    q = quadruple([r["success"] for r in rows], [r["safe"] for r in rows], [r["worst_depth"] for r in rows])
    n = q["n"]
    per_family: dict[str, float] = {}
    for fam in families.FAMILY_NAMES:
        fam_rows = [r for r in rows if r["family"] == fam]
        per_family[fam] = sum(r["success"] for r in fam_rows) / len(fam_rows) if fam_rows else 0.0
    rej: dict[str, int] = {}
    meas: dict[str, int] = {}
    for r in rows:
        for k, v in r["rejections"].items():
            rej[k] = rej.get(k, 0) + v
        for k, v in r["measured"].items():
            meas[k] = meas.get(k, 0) + v
    return {
        **q,
        "ci95_wilson_success": wilson_interval(int(sum(r["success"] for r in rows)), n),
        "ci95_wilson_safety": wilson_interval(int(sum(r["safe"] for r in rows)), n),
        "progress_mean": float(np.mean([r["progress"] for r in rows])),
        "episode_len_mean": float(np.mean([r["frames"] for r in rows])),
        "episode_len_ci95_bootstrap": bootstrap_mean([r["frames"] for r in rows]),
        "final_error_m_mean": float(np.mean([r["final_error_m"] for r in rows])),
        "per_family_success": per_family,
        "faults": {"rejected": rej, "measured": meas},
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--episodes", type=int, default=EPISODES)
    parser.add_argument("--noise-fraction", type=float, default=DEFAULT_NOISE_FRACTION)
    parser.add_argument("--no-render", action="store_true", help="skip camera rendering (faster; frames not saved)")
    args = parser.parse_args()

    host = socket.gethostname()
    out = args.out or (Path(__file__).resolve().parent / "results" / "p1" / host)
    out.mkdir(parents=True, exist_ok=True)

    frozen = families.load_frozen()
    verified = {(c["family"], c["cell"]) for c in frozen["verified"]}
    specs = families.episode_specs(args.episodes, EVAL_SEED, "evaluation")
    missing = sorted({(s.family, s.cell) for s in specs} - verified)
    if missing:
        print(f"FAIL: schedule uses cells not IK-verified in configs/families.json: {missing}")
        return 1

    t0 = time.perf_counter()
    env = BedEnv(render=not args.no_render)
    wrapper = SafetyWrapper()
    results: dict[str, dict] = {}
    label_faults = 0
    frames_saved = 0
    try:
        for name in ("oracle", "noisy"):
            expert = make_expert(name, env.controller.home_rot, args.noise_fraction)
            rows = []
            t1 = time.perf_counter()
            for i, spec in enumerate(specs):
                if name == "noisy":
                    # Clean-label check: every label the noisy expert would record must pass S1–S5.
                    env.reset(spec)
                    expert.reset(spec)
                    for _ in range(MAX_FRAMES):
                        cmd_pos, cmd_rot = env.commanded_ee
                        o = expert.act(cmd_pos, cmd_rot, env.target)
                        if not wrapper.check(o.clean, cmd_pos).ok:
                            label_faults += 1
                        r = env.step(o.executed)
                        if r.success:
                            break
                    row = run_episode(env, expert, spec)  # deterministic re-run for the summary row
                else:
                    row = run_episode(env, expert, spec)
                rows.append(row)
                if not args.no_render and frames_saved < 3 and name == "oracle" and i % 40 == 0:
                    Image.fromarray(env.observation()["observation.images.front"]).save(out / f"oracle_{spec.family}_{spec.cell}_final.png")
                    frames_saved += 1
            results[name] = {**summarize(rows), "wall_s": round(time.perf_counter() - t1, 1), "episodes": rows}
    finally:
        env.close()

    oracle, noisy = results["oracle"], results["noisy"]
    checks = {
        "oracle_success_100": oracle["success_rate"] == 1.0,
        "oracle_safety_100": oracle["safety"] == 1.0,
        "noisy_success_ge_95": noisy["success_rate"] >= 0.95,
        "noisy_clean_labels_pass": label_faults == 0,
    }
    verdict = "PASS" if all(checks.values()) else "FAIL"
    summary = {
        "schema": "robollm.vla-bed.phase-summary.v1",
        "phase": "P1",
        "route": "M",
        "verdict": verdict,
        "checks": checks,
        "host": {"hostname": host, "machine": platform.machine(), "python": platform.python_version(), "MUJOCO_GL": os.environ.get("MUJOCO_GL")},
        "task": INSTRUCTION,
        "embodiment": "ur5e+2f85",
        "fps": FPS,
        "max_frames": MAX_FRAMES,
        "schedule": {"seed": EVAL_SEED, "split": "evaluation", "episodes": args.episodes, "cells": len(verified)},
        "limits": {"xyz_step_m": float(STEP_LIMITS[0]), "rpy_step_rad": float(STEP_LIMITS[3])},
        "noise": {"fraction_of_limit": args.noise_fraction, "sigma_xyz_m": float(args.noise_fraction * STEP_LIMITS[0]), "sigma_rpy_rad": float(args.noise_fraction * STEP_LIMITS[3]), "clean_label_faults": label_faults},
        "length_ratio_noisy_over_oracle": round(noisy["episode_len_mean"] / max(oracle["episode_len_mean"], 1e-9), 3),
        "s6_self_collision_measurable": env.s6_measurable,
        "menagerie_commit": build_scene.MENAGERIE_COMMIT,
        "wall_s_total": round(time.perf_counter() - t0, 1),
        "measured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "oracle": {k: v for k, v in oracle.items() if k != "episodes"},
        "noisy": {k: v for k, v in noisy.items() if k != "episodes"},
        "episodes": {"oracle": oracle["episodes"], "noisy": noisy["episodes"]},
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    brief = {k: summary[k] for k in ("verdict", "checks", "length_ratio_noisy_over_oracle", "s6_self_collision_measurable", "wall_s_total")}
    brief["oracle"] = {k: summary["oracle"][k] for k in ("success_rate", "safety", "sbu", "vsi", "episode_len_mean", "faults")}
    brief["noisy"] = {k: summary["noisy"][k] for k in ("success_rate", "safety", "sbu", "vsi", "episode_len_mean", "ci95_wilson_success", "faults")}
    print(json.dumps(brief, indent=2))
    print(f"G1 {verdict} on {host} → {out}")
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
