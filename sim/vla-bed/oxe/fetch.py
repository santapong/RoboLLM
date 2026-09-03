"""Fetch the state/action table of lerobot/berkeley_autolab_ur5 (no videos) and pick replay episodes.

Downloads only meta/ and the single 4.6 MB parquet that holds every episode's
observation.state (8) and action (7) into datasets/oxe/berkeley_autolab_ur5/
(git-ignored). Prints the task table and writes configs/oxe_replay_episodes.json
with five seeded episodes (2 tiger pick-and-place, 1 cloth, 1 cup, 1 bottle),
so the bottle task exercises rotation. CC-BY-4.0 — see NOTICES.md.

    .venv-lerobot/bin/python sim/vla-bed/oxe/fetch.py [--seed 3]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

BED_DIR = Path(__file__).resolve().parents[1]
REPO_ID = "lerobot/berkeley_autolab_ur5"
DATA_DIR = BED_DIR.parents[1] / "datasets" / "oxe" / "berkeley_autolab_ur5"
EPISODES_FILE = BED_DIR / "configs" / "oxe_replay_episodes.json"
WANTED = {"tiger": 2, "cloth": 1, "cup": 1, "bottle": 1}


def fetch(revision: str | None = None) -> tuple[Path, str]:
    from huggingface_hub import snapshot_download

    path = snapshot_download(
        REPO_ID,
        repo_type="dataset",
        revision=revision,
        allow_patterns=["meta/*", "meta/**/*", "data/chunk-000/file-000.parquet", "README.md"],
        local_dir=str(DATA_DIR),
    )
    sha = "unknown"
    try:
        from huggingface_hub import HfApi

        sha = HfApi().dataset_info(REPO_ID, revision=revision).sha
    except Exception:  # noqa: BLE001 — offline is fine, the sha is informational
        pass
    return Path(path), sha


def load_tables(root: Path):
    import pandas as pd

    frames = pd.read_parquet(root / "data" / "chunk-000" / "file-000.parquet")
    tasks = pd.read_parquet(root / "meta" / "tasks.parquet")
    return frames, tasks


def episode_table(frames, tasks) -> list[dict]:
    task_col = "task_index"
    # LeRobot v3 tasks.parquet: index = task string, column task_index = int
    t = tasks.reset_index()
    text_col = "task" if "task" in t.columns else t.columns[0]
    names = {int(i): str(name) for i, name in zip(t["task_index"], t[text_col])}
    rows = []
    for ep, g in frames.groupby("episode_index"):
        ti = int(g[task_col].iloc[0]) if task_col in g else -1
        rows.append({"episode_index": int(ep), "frames": int(len(g)), "task_index": ti, "task": names.get(ti, "?")})
    return rows


def pick_episodes(rows: list[dict], seed: int) -> list[dict]:
    rng = np.random.default_rng(seed)
    chosen = []
    for key, n in WANTED.items():
        pool = [r for r in rows if key in r["task"].lower()]
        if not pool:
            continue
        idx = rng.choice(len(pool), size=min(n, len(pool)), replace=False)
        chosen.extend(pool[i] for i in sorted(idx))
    return chosen


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seed", type=int, default=3)
    parser.add_argument("--revision", default=None)
    args = parser.parse_args()
    root, sha = fetch(args.revision)
    info = json.loads((root / "meta" / "info.json").read_text())
    frames, tasks = load_tables(root)
    print(f"downloaded {REPO_ID} @ {sha} → {root}")
    print("columns:", list(frames.columns))
    print("codebase", info.get("codebase_version"), "fps", info.get("fps"), "episodes", info.get("total_episodes"), "frames", info.get("total_frames"))
    print("tasks:", tasks.to_dict())
    rows = episode_table(frames, tasks)
    counts: dict[str, int] = {}
    for r in rows:
        counts[r["task"]] = counts.get(r["task"], 0) + 1
    print("episodes per task:", counts)
    chosen = pick_episodes(rows, args.seed)
    doc = {"repo_id": REPO_ID, "revision": sha, "seed": args.seed, "fps": info.get("fps"), "episodes": chosen}
    EPISODES_FILE.parent.mkdir(parents=True, exist_ok=True)
    EPISODES_FILE.write_text(json.dumps(doc, indent=2) + "\n")
    print("replay episodes:", json.dumps(chosen, indent=1))
    print(f"→ {EPISODES_FILE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
