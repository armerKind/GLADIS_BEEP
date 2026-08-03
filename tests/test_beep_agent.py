import copy
import json
import unittest
from pathlib import Path

from beep_agent import EmbodiedExecutive, EmbodiedGoal, GoalArbiter, SkillCall, SkillValidationError, WorldModel, validate_skill_call

FIXTURE = Path(__file__).parent / "fixtures" / "box_far_response.json"


class WorldModelTests(unittest.TestCase):
    def response(self):
        return json.loads(FIXTURE.read_text())

    def test_tracks_same_box_across_changed_packet_ids(self):
        world = WorldModel()
        first = self.response()
        world.update(first, observed_at=1.0, packet_id="p1")
        track_id = world.snapshot()["tracks"][0]["track_id"]
        second = self.response()
        second["scene"]["objects"][0]["id"] = "provider-renamed-box"
        second["attention"]["target_id"] = "provider-renamed-box"
        second["scene"]["objects"][0]["description"] = "The same white box bearing a dark quadruped silhouette."
        snapshot = world.update(second, observed_at=2.0, packet_id="p2")
        self.assertEqual(len(snapshot["tracks"]), 1)
        self.assertEqual(snapshot["tracks"][0]["track_id"], track_id)
        self.assertEqual(snapshot["tracks"][0]["seen_count"], 2)
        self.assertEqual(snapshot["attention_track_id"], track_id)

    def test_rejects_duplicate_packet(self):
        world = WorldModel()
        world.update(self.response(), observed_at=1.0, packet_id="p1")
        with self.assertRaisesRegex(ValueError, "duplicate"):
            world.update(self.response(), observed_at=2.0, packet_id="p1")


class ExecutiveTests(unittest.TestCase):
    def setUp(self):
        self.world = WorldModel()
        self.goal = EmbodiedGoal("investigate_and_mark", "white box", marking_requested=True)
        self.executive = EmbodiedExecutive(self.world, self.goal)
        self.response = json.loads(FIXTURE.read_text())

    def update(self, packet_id, proximity, features, source_id="box-local-1"):
        response = copy.deepcopy(self.response)
        response["scene"]["objects"][0].update(id=source_id, proximity=proximity, visible_features=features)
        response["attention"]["target_id"] = source_id
        self.world.update(response, observed_at=float(self.world.sequence + 1), packet_id=packet_id)

    def test_self_directed_box_mission_progression(self):
        self.update("far", "far", ["front_face"])
        approach = self.executive.decide(rollout_mode="shadow")
        self.assertEqual(approach.skill_call.name, "approach_target")
        self.assertEqual(approach.skill_call.arguments["max_step_m"], 0.15)

        self.update("near-front", "near", ["front_face"], source_id="box-renamed")
        inspect_left = self.executive.decide(rollout_mode="shadow")
        inspect_right = self.executive.decide(rollout_mode="shadow")
        self.assertEqual(inspect_left.skill_call.arguments["strategy"], "left_view")
        self.assertEqual(inspect_right.skill_call.arguments["strategy"], "right_view")

        self.update("corner", "near", ["front_face", "right_outward_corner"], source_id="box-third-id")
        shadow_mark = self.executive.decide(rollout_mode="shadow")
        live_mark = self.executive.decide(rollout_mode="supervised")
        self.assertEqual(shadow_mark.skill_call.name, "mark_target")
        self.assertTrue(shadow_mark.skill_call.arguments["dry_run"])
        self.assertFalse(live_mark.skill_call.arguments["dry_run"])

    def test_unknown_world_observes_without_outsourcing_goal(self):
        decision = self.executive.decide()
        self.assertEqual(decision.skill_call.name, "observe")
        self.assertEqual(decision.rationale, "acquire_initial_world_state")


class SkillTests(unittest.TestCase):
    def test_rejects_raw_motor_function(self):
        with self.assertRaisesRegex(SkillValidationError, "unknown"):
            validate_skill_call(SkillCall("set_motor_register", {"value": 100}))

    def test_rejects_unbounded_approach(self):
        with self.assertRaisesRegex(SkillValidationError, "within"):
            validate_skill_call(SkillCall("approach_target", {
                "target_id": "object-0001", "max_step_m": 1.0,
                "stop_distance_m": 0.4, "reason": "Too enthusiastic.",
            }))


class GoalArbiterTests(unittest.TestCase):
    def test_curiosity_selects_and_pursues_object_without_operator_mission(self):
        world = WorldModel()
        world.update(json.loads(FIXTURE.read_text()), observed_at=1.0, packet_id="box")
        selection = GoalArbiter().select(world)
        decision = EmbodiedExecutive(world, selection.goal).decide()
        self.assertEqual(selection.drive, "curiosity")
        self.assertEqual(selection.goal.name, "investigate")
        self.assertEqual(decision.skill_call.name, "approach_target")

        response = json.loads(FIXTURE.read_text())
        response["scene"]["objects"][0].update(proximity="near", visible_features=["right_outward_corner"])
        world.update(response, observed_at=2.0, packet_id="corner")
        marking = GoalArbiter().select(world)
        self.assertEqual(marking.goal.name, "investigate_and_mark")
        self.assertTrue(marking.goal.marking_requested)

    def test_engaged_person_preempts_object_curiosity(self):
        world = WorldModel()
        response = json.loads((Path(__file__).parent / "fixtures" / "eyes_response.json").read_text())
        world.update(response, observed_at=1.0, packet_id="social")
        selection = GoalArbiter().select(world)
        decision = EmbodiedExecutive(world, selection.goal).decide()
        self.assertEqual(selection.drive, "social")
        self.assertEqual(decision.skill_call.name, "speak")


if __name__ == "__main__":
    unittest.main()
