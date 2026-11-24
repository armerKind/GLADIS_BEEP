"""Pure host-only gesture proposal policy; no bridge transport."""
import unittest

from beep_agent.gesture_policy import GesturePolicy, INITIAL_AUTO_GESTURES


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


if __name__ == '__main__':
    unittest.main()
