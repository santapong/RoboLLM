"""Unit test for sim/vla-bed/dagger.py — the relabel loop with a stub policy and the fake dataset writer (needs MuJoCo + mink)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

BED = Path(__file__).resolve().parents[2] / "sim" / "vla-bed"
sys.path.insert(0, str(BED))
sys.path.insert(0, str(Path(__file__).resolve().parent))
dg = pytest.importorskip("dagger")
from test_vla_bed_dataset import FakeLeRobotDataset  # noqa: E402


class HalfOracle:
    """A stub policy: the oracle's action scaled by 0.5 (misses like a weak policy, so the labels differ from the executed actions)."""

    def __init__(self, env):
        from expert import make_expert
        self.expert = make_expert("oracle", env.controller.home_rot, 0.0, 0.7)

    def reset(self, spec):
        self.expert.reset(spec)

    def act(self, env, observation):
        p, r = env.commanded_ee
        a = self.expert.act(p, r, env.target).clean * 0.5
        return np.repeat(a.reshape(1, 7), 20, axis=0)


def test_relabel_writes_oracle_labels_and_policy_executions(tmp_path):
    pytest.importorskip("mink")
    import os
    os.environ.setdefault("MUJOCO_GL", "egl")
    from env import BedEnv
    FakeLeRobotDataset.instances.clear()
    env = BedEnv(render=False)
    try:
        res = dg.relabel("v5a", "baseline", "stub", episodes=2, output_root=tmp_path, dataset_class=FakeLeRobotDataset, policy=HalfOracle(env), env=env)
    finally:
        env.close()
    fake = FakeLeRobotDataset.instances["local/robollm-vla-bed-v7-train"]
    assert res["episodes"] == 2 and fake.finalized and fake.episodes == 2 and res["frames"] == len(fake.frames) > 2
    f = fake.frames[0]
    assert f["action"].shape == (7,) and f["action.executed"].shape == (7,) and f["observation.noise_sigma"][0] == 0.0
    assert np.max(np.abs(f["action"][:3])) <= 0.007 + 1e-9  # the oracle label carries the base recipe's headroom cap
    assert np.allclose(f["action.executed"][:3], 0.5 * f["action"][:3], atol=1e-6)  # what the stub policy actually did
    m = json.loads((tmp_path / "v7" / "manifest.json").read_text())
    assert m["recipe"] == "v7" and m["base_recipe"] == "v5a" and m["relabel"]["labeller"] == "oracle" and m["splits"]["train"]["base_seed"] == dg.DAGGER_BASE_SEED
    seeds = {r["seed"] for r in m["splits"]["train"]["episodes"]}
    import families
    assert seeds.isdisjoint({s.seed for s in families.episode_specs(400, 10_000, "train")}) and seeds.isdisjoint({s.seed for s in families.episode_specs(100, 10_000, "evaluation")})
