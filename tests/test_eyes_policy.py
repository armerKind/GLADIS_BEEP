import json
import unittest
from pathlib import Path

from beep_eyes.policy import evaluate_proposal


FIXTURE = Path(__file__).parent / "fixtures" / "eyes_response.json"


def safe_status():
    return {
        "moving": False,
        "motion_lease_id": None,
        "scan_age_s": 0.05,
        "sectors": {"front": 1.0, "front_left": 1.0, "front_right": 1.0, "left": 1.0, "right": 1.0, "rear": 1.0},
        "slam": {"usable": True},
    }


class EyesPolicyTests(unittest.TestCase):
    def response(self):
        return json.loads(FIXTURE.read_text())

    def test_shadow_mode_never_marks_steps_eligible(self):
        decision = evaluate_proposal(self.response(), safe_status(), mode="shadow")
        self.assertFalse(decision["dispatch_performed"])
        self.assertTrue(all(not item["eligible"] for item in decision["steps"]))

    def test_stationary_mode_allows_speech_and_observation(self):
        decision = evaluate_proposal(self.response(), safe_status(), mode="stationary")
        self.assertEqual([item["eligible"] for item in decision["steps"]], [True, True])

    def test_stationary_mode_rejects_movement(self):
        response = self.response()
        response["scene"]["hazards"] = []
        response["plan"]["steps"] = [{"skill": "advance", "distance_m": 0.2, "reason": "approach"}]
        decision = evaluate_proposal(response, safe_status(), mode="stationary")
        self.assertEqual(decision["steps"][0]["reason"], "movement_not_enabled")

    def test_supervised_mode_rejects_movement_near_visual_glass(self):
        response = self.response()
        response["plan"]["steps"] = [{"skill": "orient", "direction": "right", "degrees": 10, "reason": "look"}]
        decision = evaluate_proposal(response, safe_status(), mode="supervised")
        self.assertEqual(decision["steps"][0]["reason"], "visual_hazard")

    def test_missing_lidar_rejects_physical_step(self):
        response = self.response()
        response["scene"]["hazards"] = []
        response["plan"]["steps"] = [{"skill": "gesture", "name": "pray", "reason": "greet"}]
        status = safe_status()
        status["sectors"].pop("rear")
        decision = evaluate_proposal(response, status, mode="stationary")
        self.assertEqual(decision["steps"][0]["reason"], "sector_missing:rear")

    def test_low_confidence_rejects_action(self):
        response = self.response()
        response["confidence"] = 0.2
        decision = evaluate_proposal(response, safe_status(), mode="stationary")
        self.assertEqual(decision["steps"][0]["reason"], "low_confidence")


if __name__ == "__main__":
    unittest.main()
