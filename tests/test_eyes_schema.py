import json
import unittest
from pathlib import Path

from beep_eyes.schema import PerceptionValidationError, normalize_gemini_response, validate_perception_response


FIXTURE = Path(__file__).parent / "fixtures" / "eyes_response.json"


class EyesSchemaTests(unittest.TestCase):
    def response(self):
        return json.loads(FIXTURE.read_text())

    def test_valid_fixture(self):
        result = validate_perception_response(self.response())
        self.assertEqual(result["plan"]["steps"][0]["skill"], "speak")

    def test_rejects_attention_target_missing_from_entities(self):
        result = self.response()
        result["attention"] = {"target_type": "person", "target_id": "missing", "reason": "Engage."}
        with self.assertRaisesRegex(PerceptionValidationError, "does not reference"):
            validate_perception_response(result)

    def test_rejects_duplicate_entity_ids(self):
        result = self.response()
        duplicate = dict(result["scene"]["objects"][0])
        duplicate["id"] = result["scene"]["people"][0]["id"]
        result["scene"]["objects"].append(duplicate)
        with self.assertRaisesRegex(PerceptionValidationError, "duplicate"):
            validate_perception_response(result)

    def test_rejects_raw_or_unknown_motor_skill(self):
        result = self.response()
        result["plan"]["steps"] = [{"skill": "set_velocity", "reason": "faster"}]
        with self.assertRaisesRegex(PerceptionValidationError, "not whitelisted"):
            validate_perception_response(result)

    def test_rejects_out_of_bounds_heading(self):
        result = self.response()
        result["plan"]["steps"] = [{"skill": "orient", "direction": "left", "degrees": 90, "reason": "person"}]
        with self.assertRaisesRegex(PerceptionValidationError, "within"):
            validate_perception_response(result)

    def test_rejects_more_than_three_steps(self):
        result = self.response()
        result["plan"]["steps"] = [{"skill": "observe", "reason": "wait"}] * 4
        with self.assertRaisesRegex(PerceptionValidationError, "at most 3"):
            validate_perception_response(result)

    def test_rejects_unknown_fields(self):
        result = self.response()
        result["raw_command"] = "forward 100"
        with self.assertRaisesRegex(PerceptionValidationError, "unknown keys"):
            validate_perception_response(result)

    def test_escalation_requires_reason(self):
        result = self.response()
        result["escalate"] = True
        with self.assertRaisesRegex(PerceptionValidationError, "required"):
            validate_perception_response(result)

    def test_normalizes_compact_gemini_wire_format(self):
        wire = {
            "schema_version": "1.0", "scene_summary": "A quiet room.", "changes": [],
            "entities": [], "hazards": [], "attention_type": "none", "attention_id": "",
            "attention_reason": "No target.", "goal": "Continue observing.",
            "steps": [{"skill": "observe", "reason": "Wait for change.", "argument": "", "amount": 0}],
            "confidence": 0.8, "uncertainty": [], "escalate": False, "escalation_reason": "",
        }
        result = normalize_gemini_response(wire)
        self.assertIsNone(result["attention"]["target_id"])
        self.assertEqual(result["plan"]["steps"][0], {"skill": "observe", "reason": "Wait for change."})


if __name__ == "__main__":
    unittest.main()
