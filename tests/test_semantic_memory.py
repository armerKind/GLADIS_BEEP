"""Offline cross-boot semantic bank; no bridge or robot required."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from beep_agent import AgentSession, SemanticMemory, StrategicOwner
from beep_audio.social import SocialTranscript
from beep_agent import fsutil


class SemanticMemoryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "persistent" / "memory_bank.json"
        self.bank = SemanticMemory(self.path)

    def test_reload_across_boots_without_restoring_boot_scoped_world(self):
        self.bank.record_person("owner-max", name="Max", visual_traits=["blue sweater"],
                                last_seen_at=120, last_place="living_room",
                                known_facts=["likes robots"], recent_topics=["BEEP"])
        self.bank.record_place("living_room", label="living_room",
                               map_place_ids=["boot-a:place-0001"],
                               visual_landmarks=["grey sofa"], notes=["charging spot"],
                               last_visited_at=121)
        self.bank.record_conversation(person_id="owner-max", place_label="living_room",
                                      heard_text="Hello", spoken_reply="Hello, Max",
                                      topics=["greeting"], timestamp=122)
        restored = SemanticMemory(self.path)
        fresh = AgentSession(robot_id="BEEP", mission_id="mission-new", boot_id="boot-new",
                             memory=restored)
        self.assertEqual(fresh.world.sequence, 0)
        found = restored.find_person("MAX")
        assert found is not None
        self.assertEqual(found["last_seen_at"], 120)
        self.assertEqual(restored.places["living_room"]["map_place_ids"],
                         ["boot-a:place-0001"])
        self.assertEqual(restored.conversations[0]["spoken_reply"], "Hello, Max")
        context = StrategicOwner(fresh).build_context()
        self.assertIn("Max", context["semantic_memory"])
        self.assertIn("living_room", context["semantic_memory"])
        self.assertNotIn("semantic_memory", fresh.to_dict())
        self.assertIsNone(AgentSession.from_dict(fresh.to_dict(), expected_boot_id="boot-new").memory)

    def test_incremental_updates_are_bounded_and_find_trait_ambiguous(self):
        self.bank.record_person("p1", visual_traits=["red shirt"], last_seen_at=20)
        self.bank.record_person("p1", name="Alex", visual_traits=["short hair", "red shirt"],
                                last_seen_at=19, known_facts=["plays music"])
        self.bank.record_person("p2", name="Sam", visual_traits=["red shirt"],
                                last_seen_at=21)
        self.assertEqual(self.bank.people["p1"]["last_seen_at"], 20)
        self.assertEqual(self.bank.people["p1"]["visual_traits"], ["red shirt", "short hair"])
        self.assertIsNone(self.bank.find_person("red shirt"))
        found = self.bank.find_person("alex")
        assert found is not None
        self.assertEqual(found["person_id"], "p1")
        self.assertIsNone(self.bank.find_person("nonexistent"))
        self.bank.record_place("hallway", label="Hallway", visual_landmarks=["rug"],
                               last_visited_at=22)
        self.bank.record_place("hallway", label="Hallway", notes=["north side"],
                               last_visited_at=18)
        self.assertEqual(self.bank.places["hallway"]["last_visited_at"], 22)
        self.assertEqual(self.bank.places["hallway"]["visual_landmarks"], ["rug"])
        self.assertEqual(self.bank.places["hallway"]["notes"], ["north side"])

    def test_conversation_rolling_cap_and_social_transcript_hook(self):
        for i in range(105):
            self.bank.record_conversation(heard_text=f"hello {i}", timestamp=i + 1)
        transcript = SocialTranscript("utterance-1", "How are you?", "session-1", "boot-a", None, None)
        with self.assertRaises(ValueError):
            transcript.remember_exchange(self.bank, spoken_reply="")
        transcript.remember_exchange(self.bank, spoken_reply="Operational.",
                                     person_id="p1", topics=("greeting",), timestamp=106)
        restored = SemanticMemory(self.path)
        self.assertEqual(len(restored.conversations), 100)
        self.assertEqual(restored.conversations[0]["heard_text"], "hello 6")
        self.assertEqual(restored.conversations[-1]["spoken_reply"], "Operational.")
        self.assertEqual(restored.conversations[-1]["topics"], ["greeting"])

    def test_anonymous_eviction_does_not_erase_named_person(self):
        self.bank.record_person("known", name="Max", last_seen_at=1)
        for i in range(70):
            self.bank.record_person(f"anonymous-{i}", last_seen_at=i + 2)
        self.assertEqual(len(self.bank.people), 64)
        self.assertIn("known", SemanticMemory(self.path).people)

    def test_prompt_budget_and_corrupt_file_fail_closed(self):
        self.bank.record_person("p1", name="Max", visual_traits=["blue jacket"],
                                known_facts=["likes tea"], recent_topics=["robotics"])
        self.bank.record_place("kitchen", label="Kitchen", notes=["tea cabinet"])
        text = self.bank.prompt_summary()
        self.assertIn("Person Max", text)
        self.assertIn("blue jacket", text)
        self.assertIn("Place Kitchen", text)
        self.assertLessEqual(len(self.bank.prompt_summary(18)), 18)
        with self.assertRaises(ValueError):
            self.bank.prompt_summary(-1)
        self.path.write_text('{"schema_version": 1, "people": []}')
        with self.assertRaises(ValueError):
            SemanticMemory(self.path)

    def test_named_profile_precedes_new_anonymous_sightings_in_small_prompt(self):
        self.bank.record_person("known", name="Max", known_facts=["likes tea"], last_seen_at=1)
        for index in range(10):
            self.bank.record_person(f"anon:{index}", visual_traits=["visitor"],
                                    last_seen_at=index + 100)
        summary = self.bank.prompt_summary(100)
        self.assertIn("Person Max", summary)
        self.assertLess(summary.index("Person Max"), summary.index("Person anon:"))

    def test_atomic_failure_preserves_previous_file_and_no_temp_files(self):
        self.bank.record_person("p1", name="Max")
        previous = self.path.read_bytes()
        with patch.object(fsutil.os, "replace", side_effect=OSError("disk fault")):
            with self.assertRaises(OSError):
                self.bank.record_person("p1", visual_traits=["new coat"])
        self.assertEqual(self.path.read_bytes(), previous)
        self.assertEqual(list(self.path.parent.glob("*.tmp")), [])

    def test_confident_observation_is_session_scoped_and_place_label_explicit(self):
        response = {"confidence": .85, "scene": {
            "people": [{"id": "p1", "description": "blue sweater", "confidence": .9}],
            "objects": [{"description": "grey sofa", "confidence": .95}]}}
        self.bank.record_observation(response, session_id="session-a", observed_at=40,
                                     place_label="living_room")
        self.bank.record_observation(response, session_id="session-b", observed_at=41)
        self.assertEqual(len(self.bank.people), 2)
        self.assertEqual(self.bank.places["living_room"]["visual_landmarks"], ["grey sofa"])
        self.assertEqual(self.bank.people["anon:session-a:p1"]["last_place"], "living_room")
        self.assertIsNone(self.bank.people["anon:session-b:p1"]["last_place"])
        response["confidence"] = .6
        self.bank.record_observation(response, session_id="session-c", observed_at=42,
                                     place_label="kitchen")
        self.assertNotIn("kitchen", self.bank.places)
        self.assertEqual(len(self.bank.people), 2)


if __name__ == "__main__":
    unittest.main()
