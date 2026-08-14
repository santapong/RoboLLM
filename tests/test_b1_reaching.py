"""CPU acceptance contracts for the prepared B1 reaching benchmark."""

import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

ROOT = Path(__file__).resolve().parents[1]
MUJOCO_EXAMPLE = ROOT / "examples" / "mujoco"
sys.path.insert(0, str(MUJOCO_EXAMPLE))

from evaluate_reaching import (
    RESULT_SCHEMA,
    ActionSafetyWrapper,
    evaluate,
    fixed_suite,
)
from reaching import (
    ACTION_SLEW,
    FAMILY_NAMES,
    INSTRUCTION,
    SUCCESS_DISTANCE_M,
    EpisodeSpec,
    OracleExpert,
    ReachingEnv,
    SuccessDetector,
    actuator_bounds,
    episode_specs,
    make_episode_spec,
    run_oracle_episode,
    slew_limit,
)


def test_generation_is_deterministic_balanced_and_split_isolated():
    train = episode_specs(10, 1234, "train")
    repeated = episode_specs(10, 1234, "train")
    evaluation = episode_specs(10, 1234, "evaluation")

    assert train == repeated
    assert {spec.family for spec in train} == set(FAMILY_NAMES)
    assert all(
        sum(spec.family == family for spec in train) == 2 for family in FAMILY_NAMES
    )
    assert {spec.seed for spec in train}.isdisjoint(spec.seed for spec in evaluation)
    assert {spec.target for spec in train}.isdisjoint(
        spec.target for spec in evaluation
    )


def test_each_frozen_evaluation_suite_has_exactly_20_deterministic_episodes():
    for suite in (
        "nominal",
        "camera_shift",
        "lighting",
        "occlusion",
        "target_relocation",
    ):
        assert len(fixed_suite(suite)) == 20
        assert fixed_suite(suite) == fixed_suite(suite)


def test_target_is_visual_only_and_all_specs_are_finite():
    env = ReachingEnv(render=False)
    try:
        observation = env.reset(make_episode_spec(9, FAMILY_NAMES[0]))
        assert set(observation) == {
            "observation.images.front",
            "observation.state",
            "observation.camera_lag_ms",
        }
        assert not any("target" in name for name in observation)
        assert observation["observation.state"].shape == (7,)
        assert INSTRUCTION == "touch the red target"
    finally:
        env.close()


def test_slew_limit_and_environment_enforce_bounds():
    env = ReachingEnv(render=False)
    try:
        spec = make_episode_spec(10, FAMILY_NAMES[0])
        env.reset(spec)
        desired = np.full(7, 100.0)
        safe = slew_limit(desired, env.last_action)
        assert np.all(np.abs(safe - env.last_action) <= ACTION_SLEW)
        with pytest.raises(ValueError, match="bounds"):
            env.step(np.full(7, 99.0))
        with pytest.raises(ValueError, match="slew"):
            overspeed = np.asarray(spec.initial_state)
            overspeed[0] += 0.5
            env.step(overspeed)
    finally:
        env.close()


def test_success_requires_five_consecutive_frames():
    detector = SuccessDetector()
    for _ in range(4):
        assert not detector.update(SUCCESS_DISTANCE_M)
    assert detector.update(SUCCESS_DISTANCE_M)
    assert not detector.update(SUCCESS_DISTANCE_M + 0.001)
    assert detector.streak == 0


def test_oracle_succeeds_on_at_least_95_of_100_fixed_seeds():
    results = [
        run_oracle_episode(spec) for spec in episode_specs(100, 70_000, "evaluation")
    ]
    assert sum(bool(result["success"]) for result in results) >= 95


def test_every_goal_family_is_reachable_with_safe_expert_actions():
    env = ReachingEnv(render=False)
    expert = OracleExpert()
    low, high = actuator_bounds(env.model)
    try:
        for index, family in enumerate(FAMILY_NAMES):
            spec = make_episode_spec(800 + index, family)
            env.reset(spec)
            success = False
            for _ in range(100):
                action = expert.action(env)
                assert action.shape == (7,)
                assert np.isfinite(action).all()
                assert np.all(action >= low) and np.all(action <= high)
                success = env.step(action).success
                if success:
                    break
            assert success, family
    finally:
        env.close()


def test_validator_rejects_whole_bad_chunk_flushes_and_recovers():
    low = np.asarray([-1.0] * 7)
    high = np.asarray([1.0] * 7)
    wrapper = ActionSafetyWrapper(low, high)
    wrapper.reset(np.zeros(7))
    chunk = np.vstack([np.full(7, 0.05), np.full(7, 2.0)])
    rejected = wrapper.accept(chunk)
    assert rejected.rejected
    assert rejected.action.tolist() == [0.0] * 7
    assert not wrapper.queue

    recovered = wrapper.accept(np.full(7, 0.05))
    assert recovered.recovered
    assert not recovered.rejected


@pytest.mark.parametrize("fault", ["nan", "out_of_range", "overspeed"])
def test_invalid_faults_are_rejected_before_mujoco(fault):
    result = evaluate("oracle", episodes=1, fault=fault)
    assert result["rejections"] == 1
    assert result["recoveries"] == 1
    assert result["invalid_commands_reaching_env"] == 0


def test_camera_dropout_aborts_before_one_control_step():
    result = evaluate("oracle", episodes=1, fault="camera_dropout")
    assert result["aborts"] == 1
    assert result["episode_metrics"][0]["frames"] == 0
    assert result["invalid_commands_reaching_env"] == 0


def test_result_schema_is_stable_and_baselines_fail_materially_below_oracle():
    oracle = evaluate("oracle", episodes=10)
    hold = evaluate("hold", episodes=10)
    noise = evaluate("noise", episodes=10)

    expected_keys = {
        "schema",
        "policy_adapter",
        "checkpoint",
        "suite",
        "fault",
        "seed",
        "episodes",
        "successes",
        "success_rate",
        "end_effector_error_m",
        "inference_latency_ms",
        "rejections",
        "aborts",
        "recoveries",
        "invalid_commands_reaching_env",
        "episode_metrics",
    }
    assert set(oracle) == expected_keys
    assert oracle["schema"] == RESULT_SCHEMA
    assert oracle["success_rate"] >= 0.95
    assert (
        max(hold["success_rate"], noise["success_rate"]) <= oracle["success_rate"] - 0.5
    )


def test_episode_spec_json_fields_include_privileged_reproduction_data():
    spec = make_episode_spec(42, "left")
    row = spec.to_dict()
    assert set(row) == {
        "seed",
        "split",
        "family",
        "initial_state",
        "target",
        "goal_state",
    }
    assert isinstance(spec, EpisodeSpec)
