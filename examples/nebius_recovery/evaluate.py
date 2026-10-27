"""Run a declared five-scenario, three-seed recovery evaluation."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .provider import FixtureProvider, NebiusProvider, ProviderError
from .run import SCENARIOS, run_scenario


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--provider', choices=('fixture', 'nebius'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        provider = FixtureProvider() if args.provider == 'fixture' else NebiusProvider()
    except ProviderError as exc:
        parser.error(f'{exc}. No simulation or fixture fallback was run')
    if (args.output / 'summary.json').exists():
        parser.error('Choose a new output directory to preserve the previous evidence')
    results = []
    for scenario in SCENARIOS:
        for seed in (0, 1, 2):
            report = run_scenario(scenario, provider, args.output / scenario / str(seed), seed=seed)
            results.append({key: report[key] for key in (
                'scenario', 'seed', 'provider', 'task_success', 'safe_hold',
                'acceptance_pass', 'final_observed_error_m',
                'max_joint_limit_violation_rad', 'model_calls', 'recoveries',
                'live_nebius_inference_verified', 'source_commit', 'source_sha256')})
    summary = {
        'provider': args.provider, 'seeds': [0, 1, 2],
        'evaluation': 'All five scenarios, declared before execution; no seed selection',
        'note': 'Fixture is a deterministic reference, not NVIDIA inference. Baseline comparison tests monitoring/recovery, not LLM superiority.',
        'runs': results,
        'all_acceptance_pass': all(row['acceptance_pass'] for row in results),
        'total_decision_calls': sum(row['model_calls'] for row in results),
    }
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return 0 if summary['all_acceptance_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
