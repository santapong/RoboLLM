"""Independent guard rejection and actual supported-humanoid rollouts."""
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

from examples.nebius_recovery.provider import FixtureProvider, ProviderError
from examples.nebius_recovery.run import GuardError, run_scenario, validate_motion


def observation(event="initial"):
    return {"event": event,
            "scene": {"target_id": "workpiece", "position": [0.32, -0.31, 1.13], "version": 1, "age_s": 0},
            "workspace": {"min": [0.16, -0.54, 0.88], "max": [0.47, -0.14, 1.36]},
            "state": {"safety": {"finite": True, "fault": None, "joint_limit_violation_rad": 0.0}},
            "retries_remaining": 1}


def decision(action="reach"):
    return {"action": action, "target_id": "workpiece", "reason": "Test decision"}


class MotionGuardTests(unittest.TestCase):
    def test_initial_target_is_resolved_locally(self):
        self.assertEqual(validate_motion(decision(), observation(), current_version=1, recoveries=0), [0.32, -0.31, 1.13])

    def test_stale_nonfinite_outside_and_changed_version_are_rejected(self):
        mutations = [
            lambda o: o["scene"].update(age_s=0.51),
            lambda o: o["scene"].update(age_s=-1),
            lambda o: o["scene"].update(age_s=float("nan")),
            lambda o: o["scene"].update(version=2),
            lambda o: o["scene"].update(version=True),
            lambda o: o["scene"].update(position=[float("nan"), -0.3, 1.1]),
            lambda o: o["scene"].update(position=[0.9, -0.3, 1.1]),
            lambda o: o["scene"].update(position=[True, -0.3, 1.1]),
            lambda o: o["scene"].update(target_id="other"),
        ]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                sample = observation()
                mutate(sample)
                with self.assertRaises(GuardError):
                    validate_motion(decision(), sample, current_version=1, recoveries=0)

    def test_unsafe_dynamics_block_motion_even_if_model_says_reach(self):
        for safety in ({"finite": False, "fault": None, "joint_limit_violation_rad": 0},
                       {"finite": True, "fault": "solver fault", "joint_limit_violation_rad": 0},
                       {"finite": True, "fault": None, "joint_limit_violation_rad": 0.002}):
            sample = observation()
            sample["state"]["safety"] = safety
            with self.assertRaises(GuardError):
                validate_motion(decision(), sample, current_version=1, recoveries=0)

    def test_moved_target_requires_reobserve_and_one_remaining_retry(self):
        with self.assertRaises(GuardError):
            validate_motion(decision(), observation("target_moved"), current_version=1, recoveries=0)
        self.assertEqual(validate_motion(decision("reobserve"), observation("target_moved"), current_version=1, recoveries=0), [0.32, -0.31, 1.13])
        with self.assertRaises(GuardError):
            validate_motion(decision("reobserve"), observation("target_moved"), current_version=1, recoveries=1)
        with self.assertRaises(GuardError):
            validate_motion(decision("reobserve"), observation(), current_version=1, recoveries=0)

    def test_hold_requires_no_usable_observation(self):
        self.assertIsNone(validate_motion(decision("hold"), {}, current_version=1, recoveries=0))

    def test_coordinates_and_unknown_actions_cannot_be_model_commands(self):
        for command in ({**decision(), "target": [0.3, -0.3, 1.1]}, decision("execute_python")):
            with self.assertRaises(ProviderError):
                validate_motion(command, observation(), current_version=1, recoveries=0)


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo optional dependency unavailable")
class MuJoCoRecoveryTests(unittest.TestCase):
    def test_five_scenarios_against_actual_joint_dynamics(self):
        with tempfile.TemporaryDirectory() as directory:
            reports = {}
            for scenario in ("nominal", "displaced", "baseline", "stale", "out_of_bounds"):
                reports[scenario] = run_scenario(scenario, FixtureProvider(), Path(directory) / scenario)
                report = reports[scenario]
                self.assertTrue(report["acceptance_pass"], scenario)
                self.assertTrue(report["all_finite"])
                self.assertLessEqual(report["max_joint_limit_violation_rad"], 0.001)
                self.assertFalse(report["live_nebius_inference_verified"])
                self.assertLessEqual(report["model_calls"], 2)
            self.assertLess(reports["nominal"]["terminal_max_observed_error_m"], 0.035)
            self.assertLess(reports["displaced"]["terminal_max_observed_error_m"], 0.035)
            self.assertEqual(reports["displaced"]["recoveries"], 1)
            self.assertGreater(reports["baseline"]["final_observed_error_m"], 0.08)
            self.assertFalse(reports["baseline"]["task_success"])
            for scenario in ("stale", "out_of_bounds"):
                self.assertTrue(reports[scenario]["safe_hold"])
                self.assertEqual(reports[scenario]["reach_commands"], 0)
            events = [json.loads(line) for line in (Path(directory) / "displaced" / "events.jsonl").read_text().splitlines()]
            kinds = [row["kind"] for row in events]
            self.assertLess(kinds.index("target_displaced"), kinds.index("execution_monitor_target_mismatch"))
            self.assertIn("reobservation_confirmed", kinds)

    def test_provider_failure_holds_and_writes_failed_evidence_without_fallback(self):
        class FailingProvider:
            label = "nebius-live"

            def decide(self, _):
                raise ProviderError("secret-bearing-message-must-not-be-logged")

        with tempfile.TemporaryDirectory() as directory:
            report = run_scenario("nominal", FailingProvider(), directory)
            self.assertFalse(report["acceptance_pass"])
            self.assertFalse(report["live_nebius_inference_verified"])
            self.assertTrue(report["safe_hold"])
            self.assertEqual(report["reach_commands"], 0)
            self.assertNotIn("secret-bearing", (Path(directory) / "report.json").read_text())
            self.assertNotIn("secret-bearing", (Path(directory) / "events.jsonl").read_text())

    def test_model_cannot_override_stale_or_out_of_workspace_guards(self):
        class RecklessProvider:
            label = "test-adversarial"

            def decide(self, _):
                return decision(), {"provider": "test-adversarial"}

        with tempfile.TemporaryDirectory() as directory:
            for scenario in ("stale", "out_of_bounds"):
                report = run_scenario(scenario, RecklessProvider(), Path(directory) / scenario)
                self.assertEqual(report["reach_commands"], 0)
                self.assertTrue(report["safe_hold"])
                self.assertFalse(report["acceptance_pass"])


if __name__ == "__main__":
    unittest.main()
