"""Unit tests for sim_server.py + remote_env.py — the msgpack wire format round-trips, and the in-process handler
drives a real BedEnv through RemoteEnv exactly like the local env (needs MuJoCo + mink, no sockets)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

BED = Path(__file__).resolve().parents[2] / "sim" / "vla-bed"
sys.path.insert(0, str(BED))
ss = pytest.importorskip("sim_server")
msgpack = pytest.importorskip("msgpack")


def test_obs_round_trip_through_msgpack():
    obs = {"observation.images.front": np.arange(224 * 224 * 3, dtype=np.uint8).reshape(224, 224, 3), "observation.state": np.arange(14, dtype=np.float32), "observation.camera_lag_ms": np.zeros(1, dtype=np.float32)}
    wire = msgpack.unpackb(msgpack.packb({"obs": ss.encode_obs(obs)}, use_bin_type=True), raw=False)
    back = ss.decode_obs(wire["obs"])
    assert back["observation.images.front"].dtype == np.uint8 and back["observation.images.front"].shape == (224, 224, 3)
    assert np.array_equal(back["observation.images.front"], obs["observation.images.front"])
    assert np.array_equal(back["observation.state"], obs["observation.state"])
    assert ss.decode_obs(ss.encode_obs({"observation.images.front": None, "observation.state": np.zeros(14)}))["observation.images.front"] is None
    assert ss.decode_obs(None) is None


def test_handler_rejects_bad_requests_without_an_env():
    h = ss.SimServerHandler(env=object())
    assert h.handle({"cmd": "nope"})["error"] == "bad_request"
    assert "error" in h.handle({"cmd": "step", "action": [0.0] * 7})  # object() has no step → caught, never raises
    assert h.handle("garbage")["error"] == "bad_request"
    assert h.handle({"cmd": "close"}) == {"ok": True} and h.closed


@pytest.fixture(scope="module")
def bed():
    pytest.importorskip("mink")
    from env import BedEnv
    env = BedEnv(render=False)
    yield env
    env.close()


def _spec():
    from families import load_frozen, episode_specs
    return episode_specs(1, 10_000, "evaluation")[0]


def test_remote_env_matches_local_env_step_for_step(bed):
    """The same seeded episode through RemoteEnv(handler) and through BedEnv directly gives identical rows."""
    from env import BedEnv
    from remote_env import RemoteEnv
    from expert import make_expert
    spec = _spec()
    handler = ss.SimServerHandler(env=bed)
    remote = RemoteEnv(handler.handle, render=False)
    assert remote.controller.home_rot.shape == (3, 3) and remote.info["protocol"] == ss.PROTOCOL
    local = BedEnv(render=False)
    try:
        oracle_r = make_expert("oracle", remote.controller.home_rot)
        oracle_l = make_expert("oracle", local.controller.home_rot)
        remote.reset(spec, "nominal"); local.reset(spec, "nominal")
        assert np.isclose(remote.error_m, local.error_m) and np.isclose(remote.min_error_m, local.min_error_m)
        for _ in range(30):
            pr, rr = remote.commanded_ee
            pl, rl = local.commanded_ee
            assert np.allclose(pr, pl) and np.allclose(rr, rl) and np.allclose(remote.target, local.target)
            a_r = oracle_r.act(pr, rr, remote.target).executed
            a_l = oracle_l.act(pl, rl, local.target).executed
            assert np.allclose(a_r, a_l)
            sr = remote.step(a_r, render=False); sl = local.step(a_l, render=False)
            assert sr.decision.ok == sl.decision.ok and sr.success == sl.success and np.isclose(sr.error_m, sl.error_m, atol=1e-9)
            assert np.allclose(sr.executed, sl.executed) and sr.observation["observation.images.front"] is None
            if sr.success:
                break
        assert remote.safety.safe == local.safety.safe and remote.safety.rejections == dict(local.safety.rejections)
        assert remote.frame == local.frame and remote.requests > 30
        obs = remote.observation(render=False)
        assert obs["observation.state"].shape == (14,) and np.allclose(obs["observation.state"], local.observation(render=False)["observation.state"])
        with pytest.raises(Exception):
            remote.step(np.zeros(3))
    finally:
        local.close()
