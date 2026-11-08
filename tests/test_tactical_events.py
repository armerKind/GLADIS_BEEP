"""Phase B tactical attention is advisory; no bridge or motor authority."""
import unittest

from beep_agent.tactical import TacticalEvent, TacticalGate, TacticalValidationError
from beep_agent.session import AgentSession


def event(name="one", *, source="visual_salience", suggestion="request_yield", at=10.0,
          mission_id=None, lease_id=None, revision=0):
    return TacticalEvent.from_dict({
        "schema_version": 1, "event_id": name, "session_id": "s", "boot_id": "b",
        "map_epoch": 1, "map_revision": revision, "mission_id": mission_id,
        "lease_id": lease_id, "source": source, "acquired_at_robot": at,
        "ttl_ms": 2000, "confidence": .8, "suggestion": suggestion,
        "bearing_hint_rad": None, "evidence_ids": ["frame-1"],
    })


class TacticalContractTests(unittest.TestCase):
    def setUp(self):
        self.session = AgentSession(robot_id="beep", mission_id=None, boot_id="b", session_id="s")
        self.gate = TacticalGate(self.session)

    def offer(self, e, **kwargs):
        return self.gate.ingest(e, now_robot=10.1, map_epoch=1, map_revision=0,
                                locomotion_active=True, **kwargs)

    def test_schema_roundtrip_and_invalid_event_rejected(self):
        e = event()
        self.assertEqual(TacticalEvent.from_dict(e.to_dict()), e)
        for change in ({"ttl_ms": 0}, {"confidence": float("nan")},
                       {"source": "motor_command"}, {"bearing_hint_rad": 9},
                       {"evidence_ids": ["dup", "dup"]}):
            with self.subTest(change=change), self.assertRaises(TacticalValidationError):
                TacticalEvent.from_dict(dict(e.to_dict(), **change))
        from dataclasses import replace
        with self.assertRaises(TacticalValidationError):
            self.offer(replace(e, confidence=2.0))

    def test_gait_pitch_visual_noise_cannot_yield_or_emit_cue(self):
        decision = self.offer(event(), corroborated_visual=False)
        self.assertEqual(decision.suggestion, "attention_candidate")
        self.assertEqual(decision.reason, "egomotion_ambiguous")
        self.assertFalse(decision.motor_authority)
        duplicate = self.offer(event(), corroborated_visual=False)
        self.assertFalse(duplicate.accepted)
        self.assertEqual(duplicate.reason, "duplicate")
        corroborated = self.offer(event("person", at=10.02), corroborated_visual=True)
        self.assertEqual(corroborated.suggestion, "request_yield")
        self.assertFalse(corroborated.motor_authority)

    def test_footsteps_do_not_ack_but_quiet_human_speech_can(self):
        noise = self.offer(event("footstep", source="vad_onset", suggestion="ack_cue"))
        self.assertEqual(noise.suggestion, "attention_candidate")
        speech = self.offer(event("human", source="vad_onset", suggestion="ack_cue", at=10.01),
                            corroborated_speech=True, mixer_allowed=True)
        self.assertEqual(speech.suggestion, "ack_cue")
        rapid = self.offer(event("human2", source="vad_onset", suggestion="ack_cue", at=10.02),
                           corroborated_speech=True, mixer_allowed=True)
        self.assertEqual(rapid.suggestion, "attention_candidate")
        self.assertEqual(rapid.reason, "cue_rate_limited")

    def test_delayed_stale_or_wrong_owner_event_does_not_mutate_gate(self):
        self.assertFalse(self.gate.ingest(event(at=1.0), now_robot=10.1,
                         map_epoch=1, map_revision=0, locomotion_active=True).accepted)
        self.session.bind_motion(mission_id="m", lease_id="l", boot_id="b")
        self.assertFalse(self.offer(event("old", at=10.02)).accepted)
        self.assertTrue(self.offer(event("current", mission_id="m", lease_id="l", at=10.02),
                                   corroborated_visual=True).accepted)
        self.session.release_motion(mission_id="m", lease_id="l")
        self.assertFalse(self.offer(event("late", mission_id="m", lease_id="l", at=10.02)).accepted)
        self.assertFalse(self.offer(event("wrong-revision", revision=1, at=10.02)).accepted)

    def test_unverified_hazard_and_keyword_never_become_voice_stop(self):
        unverified = self.offer(event("haz", source="local_hazard"))
        self.assertEqual(unverified.suggestion, "attention_candidate")
        verified = self.offer(event("haz2", source="local_hazard", at=10.01), verified_hazard=True)
        self.assertEqual(verified.suggestion, "request_yield")
        self.assertFalse(verified.motor_authority)
        keyword = self.offer(event("stop", source="speech_keyword", at=10.02))
        self.assertEqual(keyword.suggestion, "notify_strategy")
        self.assertEqual(keyword.reason, "separate_stop_required")


if __name__ == "__main__":
    unittest.main()
