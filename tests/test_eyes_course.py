import copy
import json
import unittest
from pathlib import Path

from beep_eyes.course import recommend_course
from beep_eyes.schema import validate_perception_response

FIXTURE = Path(__file__).parent / "fixtures" / "eyes_response.json"


def response():
    return json.loads(FIXTURE.read_text())


def status():
    return {"moving": False, "motion_lease_id": None, "scan_age_s": 0.05,
            "slam": {"usable": True},
            "sectors": {"front": 1.0, "front_left": 1.0, "front_right": 1.0,
                        "left": 1.0, "right": 1.0, "rear": 1.0}}


class CameraCourseTests(unittest.TestCase):
    def plan(self, skill, **kwargs):
        r = response()
        r["scene"]["hazards"] = []
        r["plan"]["steps"] = [{"skill": skill, "reason": "panel evidence", **kwargs}]
        return validate_perception_response(r)

    def test_visual_advance_is_only_candidate_when_all_forward_sectors_clear(self):
        recommendation = recommend_course(self.plan("advance", distance_m=0.2), status())
        self.assertEqual(recommendation["recommendation"], "forward_candidate")
        self.assertFalse(recommendation["dispatch_performed"])
        self.assertNotIn("command", recommendation)

    def test_range_gate_blocks_advance(self):
        state = status()
        state["sectors"]["front_left"] = 0.30
        recommendation = recommend_course(self.plan("advance", distance_m=0.2), state)
        self.assertEqual(recommendation["recommendation"], "hold_and_reobserve")
        self.assertEqual(recommendation["reason"], "forward_range_gate_failed")

    def test_panel_central_hazard_blocks_advance_even_with_clear_lidar(self):
        r = self.plan("advance", distance_m=0.2)
        r["scene"]["hazards"] = [{"type": "glass", "description": "glass ahead", "position": "center", "confidence": 0.9}]
        recommendation = recommend_course(validate_perception_response(r), status())
        self.assertEqual(recommendation["recommendation"], "hold_and_reobserve")
        self.assertEqual(recommendation["reason"], "central_visual_hazard")

    def test_central_person_recommends_clear_side_replan_not_terminal_hold(self):
        r = self.plan("advance", distance_m=0.1)
        r["scene"]["hazards"] = [
            {"type": "person", "description": "person ahead", "position": "center", "confidence": 0.95},
            {"type": "obstacle", "description": "chair legs", "position": "right", "confidence": 0.9},
        ]
        state = status()
        state["sectors"].update(left=0.8, front_left=0.7, right=0.8, front_right=0.7)
        recommendation = recommend_course(validate_perception_response(r), state)
        self.assertEqual(recommendation["recommendation"], "replan_around_dynamic_hazard")
        self.assertEqual(recommendation["direction"], "left")
        self.assertFalse(recommendation["dispatch_performed"])

    def test_panel_orient_advises_look_only_not_translation(self):
        recommendation = recommend_course(self.plan("orient", direction="left", degrees=10), status())
        self.assertEqual(recommendation["recommendation"], "look_left")
        self.assertNotIn("duration_s", recommendation)

    def test_stale_sensors_busy_or_invalid_slam_fail_closed(self):
        plan = self.plan("advance", distance_m=0.2)
        for mutate in (
            lambda s: s.update(scan_age_s=1.0),
            lambda s: s.update(moving=True),
            lambda s: s.update(slam={"usable": False}),
            lambda s: s["sectors"].pop("rear"),
        ):
            state = status()
            mutate(state)
            self.assertEqual(recommend_course(plan, state)["recommendation"], "hold_and_reobserve")

    def test_low_confidence_and_escalation_cannot_recommend_motion(self):
        plan = self.plan("advance", distance_m=0.2)
        for field, value in (("confidence", 0.2), ("escalate", True)):
            candidate = copy.deepcopy(plan)
            candidate[field] = value
            if field == "escalate":
                candidate["escalation_reason"] = "uncertain scene"
            self.assertEqual(recommend_course(candidate, status())["recommendation"], "hold_and_reobserve")

    def test_reverse_requires_rear_and_both_sides(self):
        plan = self.plan("retreat", distance_m=0.1)
        state = status()
        state["sectors"]["rear"] = 0.3
        self.assertEqual(recommend_course(plan, state)["recommendation"], "hold_and_reobserve")
        state["sectors"]["rear"] = 0.8
        self.assertEqual(recommend_course(plan, state)["recommendation"], "reverse_candidate")

    def test_live_owned_motion_can_receive_advisory_but_no_dispatch(self):
        state = status()
        state.update(moving=True, motion_lease_id="lease-1")
        course = recommend_course(self.plan("advance", distance_m=0.2), state)
        self.assertEqual(course["recommendation"], "forward_candidate")
        self.assertFalse(course["dispatch_performed"])

    def test_changed_mission_invalidates_old_panel_recommendation(self):
        course = recommend_course(self.plan("advance", distance_m=0.2), status(), mission_valid=False)
        self.assertEqual(course["recommendation"], "hold_and_reobserve")
        self.assertEqual(course["reason"], "mission_identity_or_state_changed")


if __name__ == "__main__":
    unittest.main()
