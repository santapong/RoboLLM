"""Hardware-free checks for the dry-run-first B1 GPU handoff."""

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PREFLIGHT_PATH = ROOT / "scripts" / "b1_gpu" / "preflight.py"


def _load_preflight_module():
    spec = importlib.util.spec_from_file_location("b1_preflight", PREFLIGHT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_training_configuration_is_frozen_local_only_and_compatible():
    module = _load_preflight_module()
    config = module.load_config(ROOT / "training" / "b1_smolvla.json")
    assert config["model"] == "lerobot/smolvla_base"
    assert config["steps"] == 20_000
    assert config["checkpoint_steps"] == [5_000, 10_000, 20_000]
    assert config["wandb"] is False
    assert config["push_to_hub"] is False
    assert config["save_checkpoint_to_hub"] is False


def test_smoke_and_full_training_default_to_dry_run():
    for mode in ("smoke", "full"):
        process = subprocess.run(
            [sys.executable, str(ROOT / "scripts" / "b1_gpu" / "train.py"), mode],
            check=True,
            capture_output=True,
            text=True,
        )
        assert process.stdout.startswith("DRY-RUN: lerobot-train ")
        assert "--wandb.enable=false" in process.stdout
        assert "--policy.push_to_hub=false" in process.stdout
        assert "--policy.device=cuda" in process.stdout


def test_checkpoint_selector_considers_only_frozen_steps(tmp_path):
    results = tmp_path / "results"
    for step, success in ((5000, 0.5), (10000, 0.8), (20000, 0.7), (15000, 1.0)):
        directory = results / f"{step:06d}"
        directory.mkdir(parents=True)
        (directory / "nominal.json").write_text(
            json.dumps(
                {
                    "success_rate": success,
                    "end_effector_error_m": {"final_mean": 1.0 - success},
                }
            ),
            encoding="utf-8",
        )
    process = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts" / "b1_gpu" / "select_checkpoint.py"),
            "--results-root",
            str(results),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    result = json.loads(process.stdout.removeprefix("DRY-RUN:"))
    assert result["selected"]["step"] == 10_000
    assert {row["step"] for row in result["candidates"]} == {5_000, 10_000, 20_000}
