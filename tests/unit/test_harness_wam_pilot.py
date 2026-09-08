"""Scientific validity checks for the isolated CPU pilot."""

import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("torch")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim/vla-bed/experiments/harness_wam"))
import experiment as exp


def test_temporal_pairs_exclude_terminal_frames_and_reject_gaps():
    assert exp.transition_indices(np.array([0, 0, 0, 1, 1]), np.array([0, 1, 2, 0, 1])).tolist() == [0, 1, 3]
    with pytest.raises(ValueError, match="nonconsecutive"):
        exp.transition_indices(np.array([0, 0]), np.array([0, 2]))


def test_camera_presence_and_centroid_do_not_confuse_black_with_center():
    image = np.zeros((11, 11, 3), dtype=np.uint8)
    assert not exp.color_features(image).any()
    image[5, 5] = [255, 0, 0]
    f = exp.color_features(image)
    assert f[0] == 1 and f[1] == 0 and f[2] == 0 and f[3] > 0
    image[5, 5] = [255, 255, 255]
    assert not exp.color_features(image).any()


def test_policy_and_harness_have_no_access_to_privileged_env_or_spec():
    class Forbidden:
        def __getattribute__(self, name):
            raise AssertionError("privileged simulator access")

    model = exp.TinyPolicy().eval()
    norm = exp.Normalizer(np.zeros(22, dtype=np.float32), np.ones(22, dtype=np.float32), np.ones(22, dtype=np.float32))
    obs = {"observation.state": np.zeros(14, dtype=np.float32),
           **{f"observation.images.{c}": np.zeros((11, 11, 3), dtype=np.uint8) for c in exp.CAMERAS}}
    policy = exp.LearnedAdapter(model, norm, harness=True)
    policy.reset(Forbidden())
    assert not policy.act(Forbidden(), obs).any() and policy.holds == 1
    obs["observation.images.wrist"][5, 5] = [255, 0, 0]
    command = policy.act(Forbidden(), obs)
    assert command.shape == (1, 7) and np.all(np.abs(command) <= exp.ACTION_SCALE)
    assert policy.holds == 1  # either camera is sufficient
    policy.reset(Forbidden())
    assert policy.holds == 0


def test_paired_comparison_rejects_silent_intersection_and_duplicates():
    with pytest.raises(ValueError, match="equal unique"):
        exp.paired_success([{"seed": 1, "success": True}], [{"seed": 2, "success": True}])
    with pytest.raises(ValueError, match="equal unique"):
        exp.paired_success([{"seed": 1, "success": True}] * 2, [{"seed": 1, "success": True}])


def test_normalizer_uses_only_supplied_fit_data_and_stays_finite():
    fit = exp.Transitions(np.ones((3, 22), dtype=np.float32), np.zeros((3, 7)), np.zeros((3, 7)),
                          np.zeros((3, 22), dtype=np.float32), np.zeros(3))
    norm = exp.Normalizer.fit(fit)
    assert np.allclose(norm.mean, 1)
    assert np.isfinite(norm.transform(fit.x)).all() and (norm.delta_std > 0).all()


def test_auxiliary_supervision_trains_encoder_and_keeps_action_executed_separate():
    exp.torch.manual_seed(10)
    model = exp.TinyPolicy()
    x = exp.torch.randn(4, 22)
    a0, f0 = model(x, exp.torch.zeros(4, 7))
    a1, f1 = model(x, exp.torch.ones(4, 7))
    assert exp.torch.equal(a0, a1)  # observed executed action cannot leak into BC prediction
    assert not exp.torch.equal(f0, f1)
    f0.square().mean().backward()
    assert model.encoder[0].weight.grad.abs().sum() > 0


def test_checkpoint_reload_preserves_predictions(tmp_path):
    model = exp.TinyPolicy().eval()
    path = tmp_path / "weights.pt"
    exp.torch.save(model.state_dict(), path)
    restored = exp.TinyPolicy().eval()
    restored.load_state_dict(exp.torch.load(path, weights_only=True))
    x = exp.torch.ones((1, 22))
    action = exp.torch.zeros((1, 7))
    assert all(exp.torch.equal(a, b) for a, b in zip(model(x, action), restored(x, action)))
