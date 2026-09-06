"""Record or validate one dataset split of the bed (SDD §8/P2).

    .venv-lerobot/bin/python sim/vla-bed/record.py --recipe v2 --split train
    .venv-lerobot/bin/python sim/vla-bed/record.py --recipe v2 --validate

Datasets land under datasets/vla-bed/<recipe>/<split> (git-ignored); the
manifest is datasets/vla-bed/<recipe>/manifest.json. Runs on the workstation
(headless EGL) in .venv-lerobot; see requirements-record.txt.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dataset as ds  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--recipe", choices=sorted(n for n, r in ds.RECIPES.items() if r.expert != "dagger"), required=True)  # v7 is made by dagger.py
    parser.add_argument("--split", choices=("train", "evaluation"), default="train")
    parser.add_argument("--episodes", type=int, default=None, help="override the recipe's episode count")
    parser.add_argument("--output-root", type=Path, default=ds.DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--validate", action="store_true", help="validate + summarize the recipe's manifest instead of recording")
    parser.add_argument("--no-decode", action="store_true", help="skip video decoding during validation")
    args = parser.parse_args()

    manifest = args.output_root / args.recipe / "manifest.json"
    if args.validate:
        report = ds.validate_dataset(manifest, decode_video=not args.no_decode)
        summary = ds.summarize(manifest) if report["valid"] else {}
        print(json.dumps({"validation": report, "summary": summary}, indent=2))
        return 0 if report["valid"] else 1
    result = ds.record_split(args.recipe, args.split, output_root=args.output_root, manifest_path=manifest, episodes=args.episodes)
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
