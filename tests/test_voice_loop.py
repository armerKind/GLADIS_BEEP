"""End-to-end offline social loop with real VAD/handoff and injected I/O."""
import struct
import tempfile
from pathlib import Path
import unittest

from beep_agent.memory import SemanticMemory
from beep_audio.capture import AudioFrame
from beep_audio.echo import BargeInGate, FrameAlignedEcho
from beep_audio.social import SocialHandoff
from beep_audio.vad import HysteresisVad
from beep_audio.voice_loop import VoiceLoop


def frame(index: int, level: int) -> AudioFrame:
    return AudioFrame('session', 'boot', index, 1. + index * .02,
                      'measured_capture', struct.pack('<320h', *([level] * 320)))


class VoiceLoopTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'memory_bank.json'
        self.bank = SemanticMemory(self.path)
        self.owner = [None, None, True]
        self.clock = [1.10]
        self.calls = []
        gate = BargeInGate(FrameAlignedEcho('session', 'boot', delay_s=0,
                                           echo_gain=0),
                           HysteresisVad(onset_rms=500, offset_rms=200,
                                         onset_frames=2, offset_frames=2))
        self.loop = VoiceLoop(gate, SocialHandoff('session', 'boot'), self.bank,
            transcribe=lambda req: self._asr(req),
            compose_reply=lambda transcript, memory: self._reply(transcript, memory),
            synthesize=lambda text: self._tts(text), play=lambda audio: self._play(audio),
            now_robot=lambda: self.clock[0],
            read_authority=lambda: tuple(self.owner), person_id='owner-max')
        self.next_text = 'My name is Max.'
        self.play_ok = True

    def _asr(self, req):
        self.calls.append(('asr', len(req.pcm_s16le)))
        return self.next_text

    def _reply(self, transcript, context):
        self.calls.append(('reply', None if transcript is None else transcript.text, context))
        return 'Noted. Your persistence is now measurable.'

    def _tts(self, text):
        self.calls.append(('tts', text))
        return b'fake-audio'

    def _play(self, audio):
        self.calls.append(('play', audio))
        return self.play_ok

    def speak(self, start):
        outputs = [self.loop.process(frame(start + i, level))
                   for i, level in enumerate((800, 800, 0, 0))]
        return outputs[-1]

    def test_introduce_name_room_fact_and_reload_across_boot(self):
        result = self.speak(0)
        self.assertEqual(result.state, 'replied')
        self.assertEqual([call[0] for call in self.calls], ['asr', 'reply', 'tts', 'play'])
        self.assertEqual(self.bank.people['owner-max']['name'], 'Max')
        self.assertEqual(self.bank.conversations[-1]['spoken_reply'], result.reply)
        self.clock[0] = 1.30
        self.next_text = 'This room is the living room.'
        room = self.speak(4)
        self.assertEqual(room.state, 'replied')
        self.assertIn('living_room', self.bank.places)
        self.clock[0] = 1.45
        self.next_text = 'Remember that I like chamomile tea.'
        self.assertEqual(self.speak(8).state, 'replied')
        fresh = SemanticMemory(self.path)
        self.assertIn('I like chamomile tea', fresh.people['owner-max']['known_facts'])
        self.assertEqual([c['topics'][0] for c in fresh.conversations],
                         ['introduction', 'place', 'facts'])
        self.assertIn('Max', self.calls[-3][2].memory_summary)  # Memory entered reply context.
        self.assertEqual(len(fresh.conversations), 3)

    def test_no_consent_never_calls_cloud_or_writes(self):
        self.owner[2] = False
        for i in range(4):
            self.loop.process(frame(i, 800 if i < 2 else 0))
        self.assertEqual(self.calls, [])
        self.assertFalse(self.path.exists())

    def test_conflicting_spoken_name_does_not_rename_known_person(self):
        self.bank.record_person('owner-max', name='Max')
        self.next_text = 'My name is Alice.'
        self.assertEqual(self.speak(0).state, 'replied')
        fresh = SemanticMemory(self.path)
        self.assertEqual(fresh.people['owner-max']['name'], 'Max')
        self.assertEqual(fresh.people['self-intro:alice']['name'], 'Alice')

    def test_owner_change_during_asr_drops_transcript(self):
        def swapped(req):
            self.owner[0], self.owner[1] = 'other-mission', 'other-lease'
            return 'My name is Max.'
        self.loop.transcribe = swapped
        result = self.speak(0)
        self.assertEqual(result.state, 'stale_or_unconsented')
        self.assertFalse(self.path.exists())
        self.assertEqual(self.calls, [])

    def test_playback_failure_does_not_write_memory(self):
        self.play_ok = False
        self.assertEqual(self.speak(0).state, 'playback_unconfirmed')
        self.assertFalse(self.path.exists())

    def test_owner_change_during_playback_does_not_write_memory(self):
        def changed(_audio):
            self.owner[2] = False
            return True
        self.loop.play = changed
        self.assertEqual(self.speak(0).state, 'owner_changed_after_playback')
        self.assertFalse(self.path.exists())

    def test_asr_latency_expires_the_one_shot_receipt(self):
        def slow(_request):
            self.clock[0] = 20.0
            return 'My name is Max.'
        self.loop.transcribe = slow
        self.assertEqual(self.speak(0).state, 'stale_or_unconsented')
        self.assertFalse(self.path.exists())

    def test_proactive_greeting_gaps_visual_context_and_cooldown(self):
        first = self.loop.initiate_turn(visual_context='A person is near the sofa.')
        self.assertEqual(first.state, 'initiated')
        self.assertFalse(first.memory_written)
        self.assertFalse(self.path.exists())
        context = self.calls[0][2]
        self.assertIsNone(self.calls[0][1])
        self.assertEqual(context.trigger, 'session_start')
        self.assertEqual(context.visual_context, 'A person is near the sofa.')
        self.assertIn('speaker_name_unknown', context.gaps)
        self.assertIn('room_label_unknown', context.gaps)
        self.assertEqual(self.loop.history, ({'role': 'assistant', 'content': first.reply},))
        self.assertEqual(self.loop.initiate_turn('person_noticed').state, 'cooldown')
        self.assertEqual(len(self.calls), 3)
        self.clock[0] += 31
        self.assertEqual(self.loop.initiate_turn('person_noticed').state, 'initiated')
        self.assertEqual(self.calls[3][2].history[0]['content'], first.reply)

    def test_no_consent_or_changed_owner_blocks_proactive_audio_and_history(self):
        self.owner[2] = False
        self.assertEqual(self.loop.initiate_turn().state, 'no_consent')
        self.assertEqual(self.calls, [])
        self.owner[2] = True
        def changed(_transcript, _context):
            self.owner[0], self.owner[1] = 'another', 'lease'
            return 'Hello.'
        self.loop.compose_reply = changed
        self.assertEqual(self.loop.initiate_turn().state, 'owner_changed_before_tts')
        self.assertEqual(self.loop.history, ())
        self.assertFalse(self.path.exists())

    def test_unconfirmed_proactive_playback_does_not_start_cooldown(self):
        self.loop.play = lambda _audio: False
        self.assertEqual(self.loop.initiate_turn().state, 'playback_unconfirmed')
        self.assertEqual(self.loop.history, ())
        self.loop.play = lambda _audio: True
        self.assertEqual(self.loop.initiate_turn().state, 'initiated')

    def test_previously_known_place_does_not_claim_current_room(self):
        self.bank.record_place('old-kitchen', label='kitchen')
        self.loop.initiate_turn()
        self.assertIn('room_label_unknown', self.calls[0][2].gaps)

    def test_speech_turns_keep_only_last_eight_role_messages(self):
        greeting = self.loop.initiate_turn()
        self.assertEqual(greeting.state, 'initiated')
        for n in range(6):
            self.clock[0] = 1.20 + n * .1
            self.next_text = f'Hello again {n}.'
            self.assertEqual(self.speak(n * 4).state, 'replied')
        self.assertEqual(len(self.loop.history), 8)
        self.assertEqual([item['role'] for item in self.loop.history],
                         ['user', 'assistant'] * 4)
        self.assertEqual(self.loop.history[-2]['content'], 'Hello again 5.')
        contexts = [call[2] for call in self.calls if call[0] == 'reply']
        self.assertEqual(len(contexts[-1].history), 8)
        self.assertEqual(contexts[-1].history[-2]['content'], 'Hello again 4.')

    def test_structured_natural_memory_and_follow_up_gaps(self):
        responses = iter((
            {'reply': 'Nico. Where are we?', 'memory_updates': {'name': 'Nico', 'topics': ['introductions']}},
            {'reply': 'Kitchen noted.', 'memory_updates': {'room': 'kitchen', 'topics': ['kitchen']}},
            {'reply': 'Coffee. Excellent choice.', 'memory_updates': {'facts': ['drinking coffee'], 'topics': ['coffee']}}
        ))
        self.loop.compose_reply = lambda transcript, context: responses.__next__()
        for index, text in enumerate(("Hey, I'm Nico.", "We're in the kitchen.",
                                      'Just drinking coffee.')):
            self.next_text = text
            self.clock[0] = 1.10 + index * .1
            self.assertEqual(self.speak(index * 4).state, 'replied')
        self.assertEqual(self.bank.people['owner-max']['name'], 'Nico')
        self.assertIn('kitchen', self.bank.places)
        self.assertIn('drinking coffee', self.bank.people['owner-max']['known_facts'])
        self.assertEqual(self.bank.people['owner-max']['recent_topics'][-1], 'coffee')
        self.clock[0] += 31
        self.loop.compose_reply = self._reply
        self.loop.initiate_turn('person_noticed')
        context = self.calls[-3][2]
        self.assertIn('follow_up_topic:coffee', context.gaps)
        self.assertNotIn('speaker_name_unknown', context.gaps)
        self.assertNotIn('room_label_unknown', context.gaps)

    def test_unsupported_structured_claim_falls_back_to_speech_regex(self):
        self.next_text = "Hey, I'm Nico."
        self.loop.compose_reply = lambda transcript, context: {
            'reply': 'Hello.', 'memory_updates': {'name': 'SomeoneElse', 'facts': ['owns a yacht']}}
        self.assertEqual(self.speak(0).state, 'replied')
        self.assertEqual(self.bank.people['owner-max']['name'], 'Nico')
        self.assertEqual(self.bank.people['owner-max']['known_facts'], [])

    def test_im_drinking_is_not_a_person_name(self):
        self.next_text = "I'm drinking coffee."
        self.assertEqual(self.speak(0).state, 'replied')
        self.assertIsNone(self.bank.people['owner-max']['name'])

    def test_extraction_callback_override_and_invalid_visual_bound(self):
        self.next_text = 'Just drinking coffee.'
        self.loop.extract_memory = lambda transcript, context: {
            'facts': ['drinking coffee'], 'topics': ['coffee']}
        self.assertEqual(self.speak(0).state, 'replied')
        self.assertIn('drinking coffee', self.bank.people['owner-max']['known_facts'])
        from beep_audio.capture import AudioContractError
        self.clock[0] += 31
        with self.assertRaises(AudioContractError):
            self.loop.initiate_turn(visual_context='x' * 241)


if __name__ == '__main__':
    unittest.main()
