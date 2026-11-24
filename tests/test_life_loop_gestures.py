"""Pure host-only gesture proposal policy; no bridge transport."""
import unittest

from beep_agent.gesture_policy import GesturePolicy, INITIAL_AUTO_GESTURES
from beep_agent.gesture_policy import GestureIntent, GestureAdmission, GestureAdmissionError


class GesturePolicyTests(unittest.TestCase):
    def test_immutable_allowlist_and_context_filter(self):
        self.assertEqual(INITIAL_AUTO_GESTURES, frozenset({'wave_hand', 'pray', 'stretch'}))
        p = GesturePolicy()
        self.assertEqual(p.consider('social_greeting', event_id='e1', entity_id='p1',
                                    observed_at=9, now=10, reason='hello').name, 'wave_hand')
        for context in ('cheeky_dog_idle', 'mark_target', 'floor_turn', 'crawl',
                        'lie_down', 'squat', 'pee'):
            with self.subTest(context=context):
                self.assertFalse(GesturePolicy().consider(context, event_id='e2',
                    entity_id='p1', observed_at=9, now=10, reason='idle').allowed)
        with self.assertRaises(AttributeError):
            INITIAL_AUTO_GESTURES.add('pee')

    def test_fresh_context_and_person_evidence(self):
        p = GesturePolicy()
        self.assertEqual(p.consider('social_greeting', event_id='x', entity_id=None,
            observed_at=9, now=10, reason='hello').reason, 'entity_required')
        self.assertEqual(p.consider('social_greeting', event_id='x', entity_id='p',
            observed_at=1, now=10, reason='hello').reason, 'stale_evidence')
        self.assertEqual(p.consider('after_rest', event_id='r', entity_id=None,
            observed_at=9, now=10, reason='stretch').name, 'stretch')

    def test_budget_cooldown_dedup_and_turn_suppression(self):
        p = GesturePolicy(cooldown_s=10, max_per_session=2)
        self.assertTrue(p.consider('social_greeting', event_id='a', entity_id='p',
            observed_at=10, now=10, reason='hello', conversation_turn_id='t1').allowed)
        self.assertEqual(p.consider('social_greeting', event_id='a', entity_id='p',
            observed_at=10, now=11, reason='hello', conversation_turn_id='t1').reason, 'duplicate_event')
        self.assertEqual(p.consider('social_greeting', event_id='b', entity_id='p',
            observed_at=21, now=21, reason='hello', conversation_turn_id='t1').reason, 'same_conversation_turn')
        self.assertEqual(p.consider('begging_attention', event_id='c', entity_id='p',
            observed_at=12, now=12, reason='please', conversation_turn_id='t2').reason, 'cooldown')
        self.assertTrue(p.consider('begging_attention', event_id='d', entity_id='p',
            observed_at=22, now=22, reason='please', conversation_turn_id='t2').allowed)
        self.assertEqual(p.consider('after_rest', event_id='e', entity_id=None,
            observed_at=40, now=40, reason='stretch').reason, 'session_budget')
        self.assertEqual(p.used, 2)

    def test_snapshot_restart_keeps_dedup_and_budget(self):
        p = GesturePolicy(cooldown_s=10, max_per_session=2)
        p.consider('after_rest', event_id='a', entity_id=None, observed_at=10, now=10, reason='stretch')
        restored = GesturePolicy.from_dict(p.to_dict())
        self.assertEqual(restored.consider('after_rest', event_id='a', entity_id=None,
            observed_at=11, now=11, reason='stretch').reason, 'duplicate_event')
        self.assertEqual(restored.consider('after_rest', event_id='b', entity_id=None,
            observed_at=11, now=11, reason='stretch').reason, 'cooldown')
        self.assertTrue(restored.consider('after_rest', event_id='c', entity_id=None,
            observed_at=20, now=20, reason='stretch').allowed)


class GestureAdmissionTests(unittest.TestCase):
    def intent(self, **changes):
        fields = dict(decision_id='d1', session_id='s1', boot_id='boot1',
            name='wave_hand', event_id='frame1', observed_at_robot=10.,
            issued_at_robot=10.5, expires_at_robot=13., map_epoch=1,
            map_revision=2, released_mission_id='mission1',
            released_lease_id='lease1', release_id='release1',
            rollout_mode='supervised')
        fields.update(changes)
        return GestureIntent(**fields)

    def readback(self, **changes):
        status = dict(boot_id='boot1', robot_monotonic_s=11., moving=False,
            motion_lease_id=None, posture='standing', posture_uncertain=False,
            fall_suspected=False, scan_age_s=.1,
            slam={'map_epoch':1, 'map_revision':2, 'map_revision_status':'verified'})
        status.update(changes)
        return {'mission':{'id':'mission1','lease_id':'lease1',
                           'release_id':'release1','state':'completed'}, 'status':status}

    def test_exact_release_and_stale_or_changed_owner(self):
        a = GestureAdmission(session_id='s1', boot_id='boot1',
            released_mission_id='mission1', released_lease_id='lease1', release_id='release1')
        a.check(self.intent(), self.readback(), physical_posture_confirmed=True)
        for intent, state in (
            (self.intent(session_id='other'), self.readback()),
            (self.intent(released_lease_id='other'), self.readback()),
            (self.intent(), self.readback(motion_lease_id='new-lease')),
            (self.intent(), self.readback(boot_id='boot2')),
            (self.intent(), self.readback(slam={'map_epoch':1,'map_revision':3,'map_revision_status':'verified'})),
            (self.intent(), self.readback(robot_monotonic_s=14)),
            (self.intent(), self.readback(scan_age_s=2)),
            (self.intent(), self.readback(posture_uncertain=True)),
            (self.intent(), self.readback(fall_suspected=True)),
        ):
            with self.subTest(intent=intent, state=state):
                with self.assertRaises(GestureAdmissionError):
                    a.check(intent, state, physical_posture_confirmed=True)
        with self.assertRaises(GestureAdmissionError):
            a.check(self.intent(), self.readback(), physical_posture_confirmed=False)

    def test_claim_once_and_ambiguous_posture_latch(self):
        a = GestureAdmission(session_id='s1', boot_id='boot1',
            released_mission_id='mission1', released_lease_id='lease1', release_id='release1')
        a.check(self.intent(), self.readback(), physical_posture_confirmed=True)
        a.claim(self.intent())
        with self.assertRaises(GestureAdmissionError): a.claim(self.intent())
        a.mark_uncertain('lost_action_receipt')
        with self.assertRaises(GestureAdmissionError):
            a.check(self.intent(decision_id='d2'), self.readback(), physical_posture_confirmed=True)
        restored = GestureAdmission.from_dict(a.to_dict())
        with self.assertRaises(GestureAdmissionError):
            restored.check(self.intent(decision_id='d2'), self.readback(), physical_posture_confirmed=True)


if __name__ == '__main__':
    unittest.main()
