"""Verify a completed local pilot and archive its portable raw evidence."""

import argparse
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from experiment import VARIANTS, paired_success, sha256, write_json


def summarize(run: Path, output: Path) -> None:
    record_file = run / "record.json"
    record = json.loads(record_file.read_text())
    if record["status"] != "complete" or record["closed_loop"]["status"] != "complete":
        raise ValueError("only a completed offline and closed-loop run can be archived")
    if not record["closed_loop"]["controls_pass"]:
        raise ValueError("oracle/hold controls failed; do not interpret this run")
    expected = record["closed_loop"]["seeds"]
    if len(expected) != len(set(expected)):
        raise ValueError("duplicate evaluation seeds")
    records = {}
    for label, summary in record["closed_loop"]["aggregate"].items():
        file = run / (label.replace("+", "_") + ".json")
        data = json.loads(file.read_text())
        rows = data["episodes"]
        if [r["seed"] for r in rows] != expected or data["label"] != label:
            raise ValueError(f"episode identity mismatch: {label}")
        if summary != data["aggregate"]:
            raise ValueError(f"aggregate mismatch: {label}")
        if summary["n"] != len(rows) or not np.isclose(summary["success_rate"], np.mean([r["success"] for r in rows])):
            raise ValueError(f"success denominator mismatch: {label}")
        if not np.isclose(summary["safety"], np.mean([r["safe"] for r in rows])):
            raise ValueError(f"safety denominator mismatch: {label}")
        if summary["joint_success_and_safe"] != sum(r["success"] and r["safe"] for r in rows):
            raise ValueError(f"joint outcome mismatch: {label}")
        records[label] = data
    for info in record["offline"]:
        if sha256(run / info["checkpoint"]) != info["checkpoint_sha256"]:
            raise ValueError("checkpoint hash mismatch")
    if sha256(run / "normalizer.npz") != record["normalizer_sha256"]:
        raise ValueError("normalizer hash mismatch")
    for variant in VARIANTS[1:]:
        recalculated = paired_success(records["bc"]["episodes"], records[variant]["episodes"])
        if recalculated != record["closed_loop"]["paired"][f"bc_vs_{variant}"]:
            raise ValueError("paired comparison mismatch")

    output.mkdir(parents=True, exist_ok=False)
    files = [record_file] + [run / (label.replace("+", "_") + ".json") for label in records]
    for file in files:
        shutil.copyfile(file, output / file.name)
    write_json(output / "verification.json", {
        "verified_at": datetime.now(timezone.utc).isoformat(),
        "source_run": str(run.resolve()), "record_sha256": sha256(record_file),
        "archived_sha256": {p.name: sha256(p) for p in files},
        "checks": ["complete stages", "oracle/hold controls", "unique equal episode seeds",
                   "raw/aggregate agreement", "checkpoint hashes", "normalizer hash", "paired comparisons"],
        "checkpoint_files": "retained in local ignored run directory; hashes archived in record.json",
    })
    means = {v: [r["development"]["episode_mean"] for r in record["offline"] if r["variant"] == v] for v in VARIANTS}
    by_seed = {(r["variant"], r["seed"]): r["development"]["episode_mean"] for r in record["offline"]}
    seeds = record["arguments"]["train_seeds"]
    aggregate = record["closed_loop"]["aggregate"]
    n = len(expected)
    better_action = all(by_seed["aux_future", s]["action_xyz_mae_mm"] < by_seed["bc", s]["action_xyz_mae_mm"] for s in seeds)
    beats_shuffle = all(by_seed["aux_future", s]["action_xyz_mae_mm"] < by_seed["shuffled_future", s]["action_xyz_mae_mm"] for s in seeds)
    better_future = all(by_seed["aux_future", s]["future_xyz_mae_mm"] < min(by_seed["aux_future", s]["persistence_xyz_mae_mm"], by_seed["shuffled_future", s]["future_xyz_mae_mm"]) for s in seeds)
    joint_delta = aggregate["aux_future"]["joint_success_and_safe"] - aggregate["bc"]["joint_success_and_safe"]
    def measured(label):
        return sum(aggregate[label]["faults"]["measured"].values())
    gate_pass = better_action and beats_shuffle and better_future and joint_delta > 0 and measured("aux_future") <= measured("bc")
    verdict = "met the local exploration gate" if gate_pass else "did not meet the local exploration gate"
    lines = [
        "# CPU harness and predictive-policy pilot — results", "",
        f"The pilot **{verdict}**. Interpret offline forecast/action errors separately from task completion.", "",
        f"Run started {record['created_at']}; source `{record['source_commit']}` (clean at launch: {not record['source_dirty']}). Completed on CPU in **{record['wall_s']:.1f} seconds**. No cloud job, API model call or physical robot was used.", "",
        "## Offline development evaluation", "",
        f"Episode means on 100 recorded development episodes. {len(seeds)} training seeds; this table averages their separate episode means, not independent scenes across seeds.", "",
        "| Policy | Action XYZ MAE (mm), mean across seeds | Future XYZ MAE (mm) |",
        "|---|---:|---:|",
    ]
    for variant, rows in means.items():
        action = np.mean([r["action_xyz_mae_mm"] for r in rows])
        future = f"{np.mean([r['future_xyz_mae_mm'] for r in rows]):.3f}" if variant != "bc" else "not trained"
        lines.append(f"| {variant} | {action:.3f} | {future} |")
    persistence = means["bc"][0]["persistence_xyz_mae_mm"]
    lines += [
        "", f"Predicting zero position change gives **{persistence:.3f} mm** future XYZ MAE. Future prediction is conditioned on the recorded executed command. The action learner imitates the separate clean expert label.", "",
        "## Closed-loop development pilot", "",
        f"Training seed {record['arguments']['rollout_seed']}; {n} matched {record['closed_loop']['variation']} episodes, one observation/action per step, unchanged 3 cm/five-frame success rule and safety checks.", "",
        "| Arm | Successful | Safe | Both successful and safe | Harness hold frames |",
        "|---|---:|---:|---:|---:|",
    ]
    for label, a in aggregate.items():
        lines.append(f"| {label} | {round(a['success_rate'] * n)}/{n} | {round(a['safety'] * n)}/{n} | {a['joint_success_and_safe']}/{n} | {a['harness_holds']} |")
    lines += ["", "| Paired comparison | Success change | Improved / regressed episodes | 95% episode bootstrap interval |",
              "|---|---:|---:|---|"]
    for label, a in record["closed_loop"]["paired"].items():
        lo, hi = a["ci95_episode_bootstrap"]
        lines.append(f"| {label} | {a['success_delta']:+.0%} | {a['improved']} / {a['regressed']} | [{lo:+.0%}, {hi:+.0%}] |")
    lines += [
        "", "## Interpretation and limits", "",
        f"Auxiliary forecasting reduced action error versus BC across every training seed: **{better_action}**. It beat shuffled supervision on action error across every seed: **{beats_shuffle}**. It beat shuffled forecasts and persistence across every seed: **{better_future}**. Its closed-loop joint endpoint changed by **{joint_delta:+d}/{n} episodes** versus BC.", "",
        "Visibility hold is a bounded intervention that can stall when re-observation cannot reveal the target; it is not recovery planning. Its holds, success and safety must be read together.", "",
        f"The miniature model has {record['offline'][0]['parameters']:,} parameters and handcrafted target-color features. Its task prior, model, training budget and action horizon differ from SmolVLA v6. Do not rank these scores against v6's full-suite result or describe them as WAM/VLM reproduction. There is one closed-loop training initialization and a small, repeatedly used development suite; paired intervals do not include all training or scene uncertainty.", "",
        "Retain v6 as the existing VLA baseline. A passing pilot would only motivate a larger controlled study. Full-model predictive training and VLM-directed harness recovery remain separate proposals requiring fresh evaluation data and explicit compute budgets.", "",
        "## Evidence and reproduction", "",
        "- [Frozen protocol and paper reading](../README.md)",
        "- [Complete run record and per-episode offline errors](record.json)",
        "- [Archive verification and SHA256 values](verification.json)",
        "- Per-arm JSON files beside this report retain all closed-loop episode rows.",
        f"- {len(record['offline'])} model state dictionaries and normalization values remain in the ignored artifact directory named in the verification record.", "",
        "Archive checks recomputed counts and paired results from raw rows and verified all saved model hashes. See the protocol and repository validation record for tests run in this session.", "",
    ]
    (output / "RESULTS.md").write_text("\n".join(lines))
    print(output / "RESULTS.md")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    summarize(args.run, args.output)
