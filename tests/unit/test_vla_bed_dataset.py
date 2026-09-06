"""UR5e VLA sim bed — recorder, manifest, validator and summary (sim/vla-bed/dataset.py).

Uses a fake LeRobot dataset so lerobot/torch are not needed; needs mink + Menagerie.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

BED = Path(__file__).resolve().parents[2] / "sim" / "vla-bed"
sys.path.insert(0, str(BED))

pytest.importorskip("mink")
if not (BED / "assets" / "mujoco_menagerie" / "universal_robots_ur5e" / "scene.xml").exists():
    pytest.skip("Menagerie checkout missing (run scripts/pi_setup.sh)", allow_module_level=True)

import dataset as ds  # noqa: E402


class FakeDataset:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.frames: list[dict] = []
        self.buffer: list[dict] = []
        self.episodes = 0
        self.finalized = False

    def add_frame(self, frame):
        self.buffer.append(frame)

    def save_episode(self):
        for j, f in enumerate(self.buffer):
            self.frames.append({**f, "episode_index": self.episodes, "frame_index": j, "timestamp": j / self.kwargs["fps"]})
        self.buffer = []
        self.episodes += 1

    def clear_episode_buffer(self):
        self.buffer = []

    def finalize(self):
        self.finalized = True

    def __len__(self):
        return len(self.frames)

    def __getitem__(self, i):
        return self.frames[i]


class FakeLeRobotDataset:
    instances: dict[str, FakeDataset] = {}

    @classmethod
    def create(cls, **kwargs):
        inst = FakeDataset(**kwargs)
        cls.instances[kwargs["repo_id"]] = inst
        return inst


@pytest.fixture(scope="module")
def recorded(tmp_path_factory):
    root = tmp_path_factory.mktemp("vla-bed-datasets")
    FakeLeRobotDataset.instances.clear()
    result = ds.record_split("v2", "train", output_root=root, episodes=2, dataset_class=FakeLeRobotDataset)
    return root, result


def loader(repo_id, root):
    return FakeLeRobotDataset.instances[repo_id]


def test_record_split_schedule_features_and_manifest(recorded):
    root, result = recorded
    assert result["episodes"] == 2 and result["successes"] == 2
    fake = FakeLeRobotDataset.instances["local/robollm-vla-bed-v2-train"]
    assert fake.finalized and fake.episodes == 2
    assert set(fake.kwargs["features"]) == set(ds.FEATURE_KEYS)
    first = fake.frames[0]
    assert set(ds.FEATURE_KEYS) <= set(first) and first["task"] == "touch the red target"
    assert first["observation.state"].shape == (14,) and first["action"].shape == (7,) and first["action.executed"].shape == (7,)
    sigmas = {f["episode_index"]: float(f["observation.noise_sigma"][0]) for f in fake.frames}
    assert sigmas == {0: 0.0, 1: 0.5}  # episode 0 clean, episode 1 at σ = 0.5 × limit
    manifest = json.loads((root / "v2" / "manifest.json").read_text())
    assert manifest["schema"] == ds.MANIFEST_SCHEMA and manifest["recipe"] == "v2"
    rows = manifest["splits"]["train"]["episodes"]
    assert [r["noise_sigma"] for r in rows] == [0.0, 0.5]
    assert all(r["safe"] and r["success"] for r in rows)
    assert all(set(r) >= {"seed", "family", "cell", "target", "frame_count", "progress", "final_error_m"} for r in rows)


def test_noisy_episode_executed_differs_from_label_but_clean_is_reused(recorded):
    fake = FakeLeRobotDataset.instances["local/robollm-vla-bed-v2-train"]
    clean_ep = [f for f in fake.frames if f["episode_index"] == 0]
    noisy_ep = [f for f in fake.frames if f["episode_index"] == 1]
    assert all(np.allclose(f["action"], f["action.executed"]) for f in clean_ep)
    assert any(not np.allclose(f["action"], f["action.executed"]) for f in noisy_ep)


def test_validator_and_summary_pass_then_catch_corruption(recorded):
    root, _ = recorded
    manifest = root / "v2" / "manifest.json"
    report = ds.validate_dataset(manifest, decode_video=True, dataset_loader=loader)
    assert report["valid"], report["errors"]
    assert report["clean_label_faults"] == 0 and report["executed_action_faults"] == 0
    summary = ds.summarize(manifest, dataset_loader=loader)
    split = summary["splits"]["train"]
    assert split["episodes"] == 2 and "20" in split["chunk_padding_percent"]
    assert set(split["by_sigma"]) == {"0.0", "0.5"}
    assert sum(split["by_sigma"]["0.5"]["distance_to_target_hist"]["counts"]) == split["by_sigma"]["0.5"]["frames"]
    # corrupt one label beyond the per-step limit → clean-label fault
    fake = FakeLeRobotDataset.instances["local/robollm-vla-bed-v2-train"]
    original = fake.frames[3]["action"].copy()
    fake.frames[3]["action"] = original + np.array([0.05, 0, 0, 0, 0, 0, 0], dtype=np.float32)
    bad = ds.validate_dataset(manifest, decode_video=False, dataset_loader=loader)
    assert not bad["valid"] and bad["clean_label_faults"] == 1
    fake.frames[3]["action"] = original
    # wrong state shape
    saved = fake.frames[0]["observation.state"]
    fake.frames[0]["observation.state"] = saved[:7]
    bad = ds.validate_dataset(manifest, decode_video=False, dataset_loader=loader)
    assert not bad["valid"] and any("bad vector shape" in e for e in bad["errors"])
    fake.frames[0]["observation.state"] = saved


def test_recipes_and_padding():
    assert ds.RECIPES["v1"].sigma(0) == 0.0 and ds.RECIPES["v2"].sigma(0) == 0.0 and ds.RECIPES["v2"].sigma(1) == 0.5
    assert ds.RECIPES["v2b"].sigma(7) == 0.25
    assert ds.chunk_padding_percent([20, 20], 20) == 0.0
    assert ds.chunk_padding_percent([10], 20) == 50.0


def test_v3_recipe_headroom_and_camera_jitter(tmp_path):
    FakeLeRobotDataset.instances.clear()
    result = ds.record_split("v3", "train", output_root=tmp_path, episodes=3, dataset_class=FakeLeRobotDataset)
    manifest = json.loads(Path(result["manifest"]).read_text())
    assert manifest["headroom"] == 0.7 and manifest["camera_jitter_deg"] == 20.0
    rows = manifest["splits"]["train"]["episodes"]
    az = [r["camera_azimuth_deg"] for r in rows]
    assert all(-20.0 <= a <= 20.0 for a in az) and len(set(az)) == 3
    assert az == [round(ds.camera_azimuth(ds.RECIPES["v3"], type("S", (), {"seed": r["seed"]})(), "train"), 4) for r in rows]
    assert ds.camera_azimuth(ds.RECIPES["v3"], type("S", (), {"seed": 1})(), "evaluation") == 0.0
    frames = FakeLeRobotDataset.instances[f"local/robollm-vla-bed-v3-train"].frames
    labels = np.stack([f["action"] for f in frames])
    executed = np.stack([f["action.executed"] for f in frames])
    assert np.max(np.abs(labels[:, :3])) <= 0.7 * 0.010 + 1e-6
    assert np.max(np.abs(executed[:, :3])) <= 0.010 + 1e-6
    assert result["success_rate"] == 1.0


def test_v4_recipe_adds_camera_translation_on_train_only(tmp_path):
    FakeLeRobotDataset.instances.clear()
    result = ds.record_split("v4", "train", output_root=tmp_path, episodes=3, dataset_class=FakeLeRobotDataset)
    manifest = json.loads(Path(result["manifest"]).read_text())
    assert manifest["camera_translate_m"] == 0.2 and manifest["camera_jitter_deg"] == 20.0 and manifest["headroom"] == 0.7
    rows = manifest["splits"]["train"]["episodes"]
    tr = [r["camera_translation_m"] for r in rows]
    assert all(len(t) == 3 and abs(t[0]) <= 0.2 and abs(t[1]) <= 0.2 and abs(t[2]) <= 0.05 for t in tr) and len({tuple(t) for t in tr}) == 3
    assert ds.camera_translation(ds.RECIPES["v4"], type("S", (), {"seed": 1})(), "evaluation") == (0.0, 0.0, 0.0)
    assert ds.camera_translation(ds.RECIPES["v3"], type("S", (), {"seed": 1})(), "train") == (0.0, 0.0, 0.0)
    assert result["success_rate"] == 1.0


def test_v5_recipes_reduce_noise_and_keep_the_frozen_evaluation_split():
    for name, noise in (("v5a", 0.25), ("v5b", 0.0)):
        r = ds.RECIPES[name]
        assert (r.noise_fraction, r.headroom, r.camera_jitter_deg, r.camera_translate_m) == (noise, 0.7, 20.0, 0.20)
        assert r.sigma(1) == noise and r.sigma(0) == 0.0
        # the frozen suite: same base seed and episode count as v2 → identical evaluation seeds/targets
        assert (r.base_seed, r.evaluation) == (ds.RECIPES["v2"].base_seed, ds.RECIPES["v2"].evaluation)
