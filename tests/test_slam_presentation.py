import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "run_slam_presentation.py"
SPEC = importlib.util.spec_from_file_location("run_slam_presentation", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
presentation = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = presentation
SPEC.loader.exec_module(presentation)


def healthy_status(**overrides):
    status = {
        "moving": False,
        "motion_cancelled": False,
        "scan_seen": True,
        "scan_age_s": 0.02,
        "sectors": {
            "front": 0.60,
            "front_left": 0.55,
            "front_right": 0.40,
            "left": 0.80,
            "right": 0.30,
        },
        "slam": {
            "active": True,
            "pose_valid": True,
            "pose_age_s": 0.04,
            "map_age_s": 0.20,
        },
    }
    status.update(overrides)
    return status


class FakeClient:
    def __init__(self, coverage_result=None):
        self.calls = []
        self.current_status = healthy_status()
        self.coverage_result = coverage_result

    def stop(self):
        self.calls.append(("stop", None))
        self.current_status["moving"] = False
        return {"ok": True, "status": self.current_status}

    def status(self):
        self.calls.append(("status", None))
        return self.current_status

    def coverage_segment(self, duration_s):
        self.calls.append(("coverage", duration_s))
        if self.coverage_result is not None:
            return self.coverage_result
        return {"ok": True, "reason": "max_duration", "elapsed_s": duration_s, "status": self.current_status}

    def trick(self, name):
        self.calls.append(("trick", name))
        return {"ok": True, "trick": {"name": name}}

    def mark_corner(self):
        self.calls.append(("mark", None))
        return {"ok": True}


class PresentationTests(unittest.TestCase):
    def test_stationary_validation_rejects_invalid_slam(self):
        status = healthy_status()
        status["slam"]["pose_valid"] = False
        with self.assertRaisesRegex(presentation.PresentationAbort, "pose is invalid"):
            presentation.validate_stationary_state(status)

    def test_corner_candidate_requires_right_wall_and_left_turn_clearance(self):
        candidate = presentation.corner_mark_candidate(healthy_status())
        self.assertIsNotNone(candidate)
        self.assertEqual(candidate["turn"], "left")
        self.assertEqual(candidate["target_front_m"], 0.20)

        blocked = healthy_status()
        blocked["sectors"]["left"] = 0.30
        self.assertIsNone(presentation.corner_mark_candidate(blocked))

    def test_stopped_prey_sequence_resets_and_stops(self):
        client = FakeClient()
        result = presentation.stopped_gesture(client, "prey")
        self.assertEqual([call for call in client.calls if call[0] == "trick"], [("trick", "prey"), ("trick", "reset")])
        self.assertGreaterEqual([call[0] for call in client.calls].count("stop"), 2)
        self.assertEqual(result["gesture"]["trick"]["name"], "prey")

    def test_bounded_routine_finishes_with_stop(self):
        client = FakeClient()
        ticks = iter([0, 1, 2, 8, 12, 18, 24, 30, 31, 32, 33, 34, 35, 36, 37, 38])
        with patch.object(presentation.time, "monotonic", side_effect=lambda: next(ticks)):
            report = presentation.run_presentation(
                client,
                duration_s=30,
                segment_s=10,
                prey_interval_s=100,
                pee_interval_s=100,
                enable_prey=False,
                enable_pee=False,
            )

        self.assertTrue(report["ok"])
        self.assertGreaterEqual(report["segments"], 1)
        self.assertEqual(client.calls[-2][0], "stop")
        self.assertEqual(client.calls[-1][0], "status")

    def test_manual_stop_result_aborts_and_final_stops(self):
        cancelled = {
            "ok": False,
            "reason": "motion_cancelled",
            "elapsed_s": 0.2,
            "status": healthy_status(motion_cancelled=True),
        }
        client = FakeClient(coverage_result=cancelled)
        ticks = iter([0, 1, 2, 3, 4, 5, 6, 7])
        with patch.object(presentation.time, "monotonic", side_effect=lambda: next(ticks)):
            with self.assertRaisesRegex(presentation.PresentationAbort, "manual stop latched"):
                presentation.run_presentation(
                    client,
                    duration_s=30,
                    segment_s=10,
                    prey_interval_s=100,
                    pee_interval_s=100,
                )

        self.assertEqual(client.calls[-1][0], "stop")


if __name__ == "__main__":
    unittest.main()
