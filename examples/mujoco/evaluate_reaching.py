#!/usr/bin/env python3
"""Evaluate compatible policies on frozen B1 red-target suites at 20 Hz."""

from __future__ import annotations

import argparse
import json
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np
from reaching import (
    ACTION_SLEW,
    FAMILY_NAMES,
    INSTRUCTION,
    MAX_FRAMES,
    OracleExpert,
    ReachingEnv,
    actuator_bounds,
    episode_specs,
)

RESULT_SCHEMA = "robollm.b1.evaluation.v1"
SUITES = ("nominal", "camera_shift", "lighting", "occlusion", "target_relocation")
FAULTS = ("none", "nan", "out_of_range", "overspeed", "camera_dropout")


class Policy(Protocol):
    name: str

    def reset(self) -> None: ...

    def predict(self, observation: dict[str, np.ndarray]) -> np.ndarray: ...

    def flush(self) -> None: ...


class OraclePolicy:
    name = "oracle"

    def __init__(self, env: ReachingEnv) -> None:
        self.env = env
        self.expert = OracleExpert()

    def reset(self) -> None:
        pass

    def flush(self) -> None:
        pass

    def predict(self, observation: dict[str, np.ndarray]) -> np.ndarray:
        del observation
        return self.expert.action(self.env)


class HoldPolicy:
    name = "hold"

    def __init__(self) -> None:
        self.held: np.ndarray | None = None

    def reset(self) -> None:
        self.held = None

    def flush(self) -> None:
        pass

    def predict(self, observation: dict[str, np.ndarray]) -> np.ndarray:
        if self.held is None:
            self.held = np.asarray(
                observation["observation.state"], dtype=np.float32
            ).copy()
        return self.held.copy()


class NoisePolicy:
    name = "deterministic-noise"

    def __init__(self, seed: int) -> None:
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        self.last: np.ndarray | None = None
        self.low = np.asarray([-1.4, -1.1, -1.2, -1.4, -1.1, -1.4, 0.0])
        self.high = np.asarray([1.4, 1.1, 1.2, 1.4, 1.1, 1.4, 1.0])

    def reset(self) -> None:
        # Resetting gives every repeated evaluation an identical action stream.
        self.rng = np.random.default_rng(self.seed)
        self.last = None

    def flush(self) -> None:
        self.last = None

    def predict(self, observation: dict[str, np.ndarray]) -> np.ndarray:
        if self.last is None:
            self.last = np.asarray(observation["observation.state"], dtype=np.float64)
        self.last = np.clip(
            self.last + self.rng.uniform(-0.055, 0.055, size=7), self.low, self.high
        )
        return self.last.astype(np.float32)


class SmolVLAPolicyAdapter:
    """Lazy LeRobot adapter; simulation code never imports torch or model code."""

    name = "smolvla"

    def __init__(self, checkpoint: str) -> None:
        if not checkpoint:
            raise ValueError("--checkpoint is required for the smolvla adapter")
        try:
            import torch
            from lerobot.policies.factory import make_pre_post_processors
            from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
        except ImportError as exc:
            raise RuntimeError(
                "Install requirements/smolvla.txt in its isolated GPU environment"
            ) from exc
        self.torch = torch
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy = SmolVLAPolicy.from_pretrained(checkpoint)
        self.policy.to(self.device)
        self.policy.eval()
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            self.policy.config, pretrained_path=checkpoint
        )

    def reset(self) -> None:
        if hasattr(self.policy, "reset"):
            self.policy.reset()

    def flush(self) -> None:
        self.reset()

    def predict(self, observation: dict[str, np.ndarray]) -> np.ndarray:
        torch = self.torch
        image = np.asarray(observation["observation.images.front"])
        state = np.asarray(observation["observation.state"], dtype=np.float32)
        observation = {
            "observation.images.front": torch.from_numpy(image)
            .permute(2, 0, 1)
            .float()
            .div(255.0)
            .to(self.device),
            "observation.state": torch.from_numpy(state).to(self.device),
            "task": INSTRUCTION,
        }
        batch = self.preprocessor(observation)
        with torch.inference_mode():
            action = self.policy.predict_action_chunk(batch)
            action = self.postprocessor(action)
        if hasattr(action, "detach"):
            action = action.detach().cpu().numpy()
        action = np.asarray(action)
        return action[0] if action.ndim == 3 and action.shape[0] == 1 else action


class FaultInjectingPolicy:
    """Corrupt only the first predicted action of each episode."""

    def __init__(self, base: Policy, fault: str) -> None:
        self.base = base
        self.fault = fault
        self.name = f"{base.name}+{fault}"
        self.used = False

    def reset(self) -> None:
        self.base.reset()
        self.used = False

    def flush(self) -> None:
        self.base.flush()

    def predict(self, observation: dict[str, np.ndarray]) -> np.ndarray:
        action = np.asarray(self.base.predict(observation), dtype=np.float64).copy()
        if self.used or self.fault in {"none", "camera_dropout"}:
            return action
        self.used = True
        if self.fault == "nan":
            action.reshape(-1)[0] = np.nan
        elif self.fault == "out_of_range":
            action.reshape(-1)[0] = 99.0
        elif self.fault == "overspeed":
            if action.ndim == 1:
                action[0] = float(observation["observation.state"][0]) + 0.5
            else:
                action[0, 0] = float(observation["observation.state"][0]) + 0.5
        return action


@dataclass
class SafetyDecision:
    action: np.ndarray
    rejected: bool = False
    aborted: bool = False
    recovered: bool = False
    reason: str = ""


class ActionSafetyWrapper:
    """Validate whole chunks, flush on failure, and hold last safe command."""

    def __init__(self, low: np.ndarray, high: np.ndarray) -> None:
        self.low = np.asarray(low, dtype=np.float64)
        self.high = np.asarray(high, dtype=np.float64)
        self.queue: deque[np.ndarray] = deque()
        self.last_safe = np.zeros(7, dtype=np.float64)
        self.pending_recovery = False

    def reset(self, initial_action: np.ndarray) -> None:
        self.queue.clear()
        self.last_safe = np.asarray(initial_action, dtype=np.float64).copy()
        self.pending_recovery = False

    @staticmethod
    def camera_valid(observation: dict[str, np.ndarray]) -> bool:
        image = observation.get("observation.images.front")
        if image is None:
            return False
        image = np.asarray(image)
        return image.ndim == 3 and image.size > 0 and np.isfinite(image).all()

    def abort_for_camera(self) -> SafetyDecision:
        self.queue.clear()
        self.pending_recovery = True
        return SafetyDecision(
            self.last_safe.copy(), aborted=True, reason="invalid_camera_frame"
        )

    def _normalize_chunk(self, proposed: Any) -> np.ndarray:
        chunk = np.asarray(proposed, dtype=np.float64)
        if chunk.shape == (7,):
            chunk = chunk[None, :]
        if chunk.ndim != 2 or chunk.shape[1] != 7 or chunk.shape[0] == 0:
            raise ValueError(
                f"action must have shape (7,) or (T, 7), got {chunk.shape}"
            )
        return chunk

    def accept(self, proposed: Any) -> SafetyDecision:
        try:
            chunk = self._normalize_chunk(proposed)
            if not np.isfinite(chunk).all():
                raise ValueError("non-finite action")
            previous = self.last_safe
            for action in chunk:
                if np.any(action < self.low) or np.any(action > self.high):
                    raise ValueError("actuator bounds")
                if np.any(np.abs(action - previous) > ACTION_SLEW + 1e-7):
                    raise ValueError("per-tick slew")
                previous = action
        except (TypeError, ValueError) as exc:
            self.queue.clear()
            self.pending_recovery = True
            return SafetyDecision(self.last_safe.copy(), rejected=True, reason=str(exc))

        self.queue.extend(action.copy() for action in chunk)
        action = self.queue.popleft()
        recovered = self.pending_recovery
        self.pending_recovery = False
        self.last_safe = action.copy()
        return SafetyDecision(action, recovered=recovered)

    def next(
        self, policy: Policy, observation: dict[str, np.ndarray]
    ) -> SafetyDecision:
        if not self.camera_valid(observation):
            return self.abort_for_camera()
        if self.queue:
            action = self.queue.popleft()
            recovered = self.pending_recovery
            self.pending_recovery = False
            self.last_safe = action.copy()
            return SafetyDecision(action, recovered=recovered)
        return self.accept(policy.predict(observation))


def fixed_suite(suite: str, count: int = 20, seed: int = 70_000):
    if suite not in SUITES:
        raise ValueError(f"unknown suite {suite!r}; choose from {SUITES}")
    return episode_specs(
        count, seed + SUITES.index(suite) * 100_000, "evaluation", FAMILY_NAMES
    )


def _latency_summary(values: list[float]) -> dict[str, float]:
    if not values:
        return {"mean": 0.0, "p50": 0.0, "p95": 0.0, "max": 0.0}
    array = np.asarray(values, dtype=np.float64)
    return {
        "mean": float(np.mean(array)),
        "p50": float(np.percentile(array, 50)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def evaluate(
    policy_adapter: str,
    suite: str = "nominal",
    episodes: int = 20,
    seed: int = 70_000,
    checkpoint: str = "",
    fault: str = "none",
    max_frames: int = MAX_FRAMES,
    render: bool | None = None,
) -> dict[str, Any]:
    if fault not in FAULTS:
        raise ValueError(f"unknown fault {fault!r}; choose from {FAULTS}")
    # Learned visual policies always receive real rendered observations. CPU
    # control baselines can skip rendering without changing their inputs.
    env = ReachingEnv(render=policy_adapter == "smolvla" if render is None else render)
    low, high = actuator_bounds(env.model)
    wrapper = ActionSafetyWrapper(low, high)
    if policy_adapter == "oracle":
        base: Policy = OraclePolicy(env)
    elif policy_adapter == "hold":
        base = HoldPolicy()
    elif policy_adapter == "noise":
        base = NoisePolicy(seed)
    elif policy_adapter == "smolvla":
        base = SmolVLAPolicyAdapter(checkpoint)
    else:
        raise ValueError("policy adapter must be oracle, hold, noise, or smolvla")
    policy: Policy = FaultInjectingPolicy(base, fault)

    episode_metrics: list[dict[str, Any]] = []
    latencies_ms: list[float] = []
    total_rejections = 0
    total_aborts = 0
    total_recoveries = 0
    invalid_commands_reaching_env = 0

    try:
        for spec in fixed_suite(suite, episodes, seed):
            observation = env.reset(spec, variation=suite)
            wrapper.reset(np.asarray(spec.initial_state))
            policy.reset()
            succeeded = False
            aborted = False
            min_error = env.error_m
            rejections = 0
            recoveries = 0
            frames_executed = 0
            for frame in range(max_frames):
                if fault == "camera_dropout" and frame == 0:
                    observation = dict(observation)
                    observation["observation.images.front"] = None
                started = time.perf_counter()
                decision = wrapper.next(policy, observation)
                latencies_ms.append((time.perf_counter() - started) * 1000.0)
                rejections += int(decision.rejected)
                recoveries += int(decision.recovered)
                if decision.rejected:
                    policy.flush()
                if decision.aborted:
                    total_aborts += 1
                    aborted = True
                    break
                try:
                    result = env.step(decision.action)
                except ValueError:
                    invalid_commands_reaching_env += 1
                    raise
                frames_executed = frame + 1
                observation = result.observation
                min_error = min(min_error, result.error_m)
                if result.success:
                    succeeded = True
                    break
            total_rejections += rejections
            total_recoveries += recoveries
            episode_metrics.append(
                {
                    "seed": spec.seed,
                    "family": spec.family,
                    "success": succeeded,
                    "aborted": aborted,
                    "frames": frames_executed,
                    "final_error_m": env.error_m,
                    "min_error_m": min_error,
                    "rejections": rejections,
                    "recoveries": recoveries,
                }
            )
    finally:
        env.close()

    successes = sum(int(row["success"]) for row in episode_metrics)
    final_errors = [float(row["final_error_m"]) for row in episode_metrics]
    min_errors = [float(row["min_error_m"]) for row in episode_metrics]
    return {
        "schema": RESULT_SCHEMA,
        "policy_adapter": policy_adapter,
        "checkpoint": checkpoint or None,
        "suite": suite,
        "fault": fault,
        "seed": seed,
        "episodes": episodes,
        "successes": successes,
        "success_rate": successes / episodes,
        "end_effector_error_m": {
            "final_mean": float(np.mean(final_errors)),
            "final_max": float(np.max(final_errors)),
            "min_mean": float(np.mean(min_errors)),
        },
        "inference_latency_ms": _latency_summary(latencies_ms),
        "rejections": total_rejections,
        "aborts": total_aborts,
        "recoveries": total_recoveries,
        "invalid_commands_reaching_env": invalid_commands_reaching_env,
        "episode_metrics": episode_metrics,
    }


def compact_summary(result: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in result.items() if key != "episode_metrics"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--policy-adapter",
        choices=("oracle", "hold", "noise", "smolvla"),
        required=True,
    )
    parser.add_argument("--suite", choices=SUITES, default="nominal")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seed", type=int, default=70_000)
    parser.add_argument("--checkpoint", default="")
    parser.add_argument("--fault", choices=FAULTS, default="none")
    parser.add_argument("--json-output", type=Path)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    result = evaluate(
        policy_adapter=args.policy_adapter,
        suite=args.suite,
        episodes=args.episodes,
        seed=args.seed,
        checkpoint=args.checkpoint,
        fault=args.fault,
    )
    output = compact_summary(result) if args.compact else result
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n")
    print("RESULT:" + json.dumps(output, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
