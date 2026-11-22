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

    def _reply(self, transcript, memory):
        self.calls.append(('reply', transcript.text, memory))
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
        self.assertIn('Max', self.calls[-3][2])  # Memory entered reply context.
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


if __name__ == '__main__':
    unittest.main()
