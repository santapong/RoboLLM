#!/usr/bin/env python3
"""Config-driven SmolVLA fine-tune on the bed (SDD §8 P4).

Runs LeRobot's own ``lerobot-train`` in-process so the bed can (a) apply a
train-time action-label representation (``labels.py``: gripper_frame /
chunk_delta) to every sampled window and (b) patch the dataset statistics the
normaliser reads, before the trainer sees them. Baseline runs use the stored
labels untouched.

    python sim/vla-bed/gpu/train.py --run baseline --mode cpu-smoke [--execute]   # 2 steps on CPU, zero spend
    python sim/vla-bed/gpu/train.py --run baseline --mode smoke --execute          # 10 steps on the GPU: it/s, VRAM
    python sim/vla-bed/gpu/train.py --run baseline --mode full --execute           # the priced run

Overrides for borrowed hosts (Kaggle, RunPod): --dataset-root, --output-root, --steps,
--batch-size, --save-freq, --max-hours (stop training cleanly after a wall-clock budget;
the last saved checkpoint is still evaluated by the notebook), --vlm-dtype float16 (cast
the *frozen* VLM after load for GPUs without native bfloat16 — T4, P100).

Without --execute the resolved lerobot-train command is printed and nothing runs.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import sys
import time
from pathlib import Path

import numpy as np

BED_DIR = Path(__file__).resolve().parents[1]
ROOT = BED_DIR.parents[1]
CONFIG_FILE = BED_DIR / "gpu" / "config.json"
sys.path.insert(0, str(BED_DIR))
import labels  # noqa: E402
import resources  # noqa: E402

MODES = ("cpu-smoke", "smoke", "full")


def load_config(path: Path = CONFIG_FILE) -> dict:
    return json.loads(path.read_text())


def get_run(config: dict, name: str) -> dict:
    for run in config["runs"]:
        if run["name"] == name:
            return run
    raise SystemExit(f"unknown run {name!r}; choose from {[r['name'] for r in config['runs']]}")


def output_dir(config: dict, run: dict, mode: str, output_root: Path | None = None) -> Path:
    return (Path(output_root) if output_root else ROOT / config["output_dir"]) / run["name"] / mode


def build_argv(config: dict, run: dict, mode: str, out: Path, overrides: dict | None = None) -> list[str]:
    ov = overrides or {}
    ds = config["dataset"]
    steps = ov.get("steps") or {"cpu-smoke": config["cpu_smoke_steps"], "smoke": config["smoke_steps"], "full": run["steps"]}[mode]
    batch = ov.get("batch_size") or (config["batch_size"] if mode == "full" else 2)
    save_freq = ov.get("save_freq") or (steps if mode != "full" else run["save_freq"])
    dataset_root = Path(ov["dataset_root"]) if ov.get("dataset_root") else ROOT / ds["train_root"]
    device = "cpu" if mode == "cpu-smoke" else "cuda"
    argv = [
        "lerobot-train",
        f"--policy.path={config['model']}",
        f"--dataset.repo_id={ds['train_repo_id']}",
        f"--dataset.root={dataset_root}",
        f"--rename_map={json.dumps(config['rename_map'])}",
        f"--batch_size={batch}",
        f"--num_workers={0 if mode == 'cpu-smoke' else config['num_workers']}",
        f"--steps={steps}",
        f"--save_freq={save_freq}",
        "--save_checkpoint=true",
        f"--output_dir={out}",
        f"--job_name=robollm_vla_bed_{run['name']}_{mode}",
        f"--seed={config['seed']}",
        f"--policy.chunk_size={config['chunk_size']}",
        f"--policy.n_action_steps={config['n_action_steps']}",
        f"--policy.device={device}",
        "--policy.push_to_hub=false",
        "--save_checkpoint_to_hub=false",
        "--wandb.enable=false",
    ]
    for key, value in run.get("policy_overrides", {}).items():
        argv.append(f"--policy.{key}={json.dumps(value) if isinstance(value, bool) else value}")
    return argv


# ----- label representation wrapper -----


class LabelledDataset:
    """Proxy over a LeRobotDataset that rewrites each window's `action` into the run's representation."""

    def __init__(self, inner, representation: str, chunk_size: int, std_floor: float):
        self._inner = inner
        self.representation = representation
        self.chunk_size = chunk_size
        self._patch_stats(std_floor)

    def __getattr__(self, name):  # everything the trainer reads (meta, episodes, num_frames, …)
        return getattr(self._inner, name)

    def __len__(self):
        return len(self._inner)

    def __getitem__(self, idx):
        item = self._inner[idx]
        if self.representation == "identity":
            return item
        import torch

        a = item["action"]
        a_np = a.detach().cpu().numpy().astype(np.float64)
        s = np.asarray(item["observation.state"]).reshape(-1)[-14:]
        t = labels.transform(a_np, self.representation, s)
        item["action"] = torch.as_tensor(t, dtype=a.dtype)
        return item

    def _patch_stats(self, std_floor: float) -> None:
        stats = self._inner.meta.stats
        if self.representation != "identity":
            stats["action"] = self._transformed_action_stats()
        for key in ("action", "observation.state"):
            if key in stats and "std" in stats[key]:
                std = np.asarray(stats[key]["std"], dtype=np.float32).copy()
                floored = std < std_floor
                std[floored] = 1.0
                stats[key]["std"] = std
                stats[key]["std_floored_channels"] = np.flatnonzero(floored)

    def _transformed_action_stats(self) -> dict:
        """Stats over every training window in the run's representation (windows padded by repeating the last frame, as LeRobot does)."""
        hf = self._inner.hf_dataset
        actions = np.asarray(hf["action"], dtype=np.float64)
        states = np.asarray(hf["observation.state"], dtype=np.float64)
        episodes = np.asarray(hf["episode_index"])
        windows = []
        for ep in np.unique(episodes):
            idx = np.flatnonzero(episodes == ep)
            a, s = actions[idx], states[idx]
            for t in range(len(idx)):
                w = np.arange(t, t + self.chunk_size)
                w = np.minimum(w, len(idx) - 1)
                windows.append(labels.transform(a[w], self.representation, s[t]))
        return labels.window_stats(np.stack(windows))


class TimeBudgetExceeded(RuntimeError):
    pass


class StepClock:
    """Wraps LeRobot's update_policy: counts steps, measures steps/s after warm-up, enforces --max-hours."""

    def __init__(self, original, max_hours: float | None, warmup_steps: int = 2):
        self.original = original
        self.max_hours = max_hours
        self.warmup = warmup_steps
        self.steps = 0
        self.t_start = time.perf_counter()
        self.t_measure = None

    def __call__(self, *args, **kwargs):
        result = self.original(*args, **kwargs)
        self.steps += 1
        if self.steps == self.warmup:
            self.t_measure = time.perf_counter()
        if self.max_hours is not None and (time.perf_counter() - self.t_start) > self.max_hours * 3600:
            raise TimeBudgetExceeded(f"wall-clock budget of {self.max_hours} h reached after {self.steps} steps")
        return result

    @property
    def steps_per_s(self) -> float | None:
        if self.t_measure is None or self.steps <= self.warmup:
            return None
        return (self.steps - self.warmup) / (time.perf_counter() - self.t_measure)


def cast_vlm(policy, dtype_name: str):
    """Cast the frozen VLM (vision encoder + language model) after load; the action expert stays float32."""
    import torch

    vlm = policy.model.vlm_with_expert.vlm
    vlm.to(getattr(torch, dtype_name))
    return policy


def gpu_info() -> dict:
    try:
        import torch

        if torch.cuda.is_available():
            props = torch.cuda.get_device_properties(0)
            return {"gpu": props.name, "vram_gb": round(props.total_memory / 1e9, 1), "bf16_native": bool(torch.cuda.is_bf16_supported()), "peak_vram_gb": round(torch.cuda.max_memory_allocated() / 1e9, 2)}
    except Exception:  # noqa: BLE001
        pass
    return {"gpu": None}


def run_training(config: dict, run: dict, mode: str, out: Path, overrides: dict | None = None) -> int:
    from lerobot.scripts import lerobot_train as lt

    ov = overrides or {}
    original = lt.make_train_eval_datasets
    representation = run["representation"]

    def patched(cfg):
        train_ds, eval_ds = original(cfg)
        train_ds = LabelledDataset(train_ds, representation, config["chunk_size"], config["std_floor"])
        if eval_ds is not None:
            eval_ds = LabelledDataset(eval_ds, representation, config["chunk_size"], config["std_floor"])
        return train_ds, eval_ds

    lt.make_train_eval_datasets = patched
    if ov.get("vlm_dtype") and ov["vlm_dtype"] != "bfloat16":
        original_make_policy = lt.make_policy

        def patched_make_policy(*a, **k):
            return cast_vlm(original_make_policy(*a, **k), ov["vlm_dtype"])

        lt.make_policy = patched_make_policy
    clock = StepClock(lt.update_policy, ov.get("max_hours"))
    lt.update_policy = clock
    sys.argv = build_argv(config, run, mode, out, ov)
    os.environ.update({"WANDB_MODE": "disabled", "HF_HUB_DISABLE_TELEMETRY": "1"})
    t0 = time.perf_counter()
    status = "completed"
    try:
        lt.main()
    except TimeBudgetExceeded as exc:
        status = f"stopped: {exc}"
        print(status)
    record = {
        "schema": "robollm.vla-bed.training-run.v1",
        "run": run["name"],
        "mode": mode,
        "representation": representation,
        "status": status,
        "steps_done": clock.steps,
        "steps_per_s": round(clock.steps_per_s, 4) if clock.steps_per_s else None,
        "vlm_dtype": ov.get("vlm_dtype") or "bfloat16",
        "overrides": {k: (str(v) if isinstance(v, Path) else v) for k, v in ov.items() if v is not None},
        **gpu_info(),
        "argv": sys.argv,
        "wall_s": round(time.perf_counter() - t0, 1),
        "resources": resources.snapshot(),
        "measured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    out.mkdir(parents=True, exist_ok=True)
    (out / "run_record.json").write_text(json.dumps(record, indent=2) + "\n")
    print(f"training record → {out / 'run_record.json'} ({record['wall_s']} s, {record['steps_done']} steps, {record['steps_per_s']} steps/s, peak RSS {record['resources']['peak_rss_mb']} MB)")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", required=True)
    parser.add_argument("--mode", choices=MODES, required=True)
    parser.add_argument("--config", type=Path, default=CONFIG_FILE)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dataset-root", type=Path, default=None, help="LeRobot dataset root (e.g. /kaggle/input/vla-bed-v2/v2/train)")
    parser.add_argument("--output-root", type=Path, default=None, help="where <run>/<mode>/ is written (e.g. /kaggle/working/artifacts)")
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--batch-size", type=int, default=None)
    parser.add_argument("--save-freq", type=int, default=None)
    parser.add_argument("--max-hours", type=float, default=None)
    parser.add_argument("--vlm-dtype", choices=("bfloat16", "float16"), default=None)
    args = parser.parse_args()
    config = load_config(args.config)
    run = get_run(config, args.run)
    out = output_dir(config, run, args.mode, args.output_root)
    overrides = {"dataset_root": args.dataset_root, "steps": args.steps, "batch_size": args.batch_size, "save_freq": args.save_freq, "max_hours": args.max_hours, "vlm_dtype": args.vlm_dtype}
    argv = build_argv(config, run, args.mode, out, overrides)
    print(("EXECUTE: " if args.execute else "DRY-RUN: ") + shlex.join(argv))
    if not args.execute:
        return 0
    if not run.get("enabled", True) and args.mode == "full":
        raise SystemExit(f"run {run['name']} is disabled in {args.config}")
    if out.exists() and any(out.iterdir()):
        raise SystemExit(f"output path is not empty: {out}")
    return run_training(config, run, args.mode, out, overrides)


if __name__ == "__main__":
    raise SystemExit(main())
