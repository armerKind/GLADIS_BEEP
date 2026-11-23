"""Host-only tests of Telegram-text -> BEEP speaker -> durable semantic memory."""
import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch

from beep_agent.memory import SemanticMemory
from beep_audio.capture import AudioContractError
from scripts import run_social_voice as social
from scripts import social_relay as host_relay


class RelayTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'memory_bank.json'

    def args(self, *rest):
        return social.parse_args(['--memory-file', str(self.path), *rest])

    def test_read_context_json_is_read_only_and_keeps_room_unknown(self):
        bank = SemanticMemory(self.path)
        bank.record_person('max', name='Max', recent_topics=['music'])
        out = io.StringIO()
        with patch.object(social, 'parse_args', return_value=self.args('--read-context', '--person-id', 'max')):
            with redirect_stdout(out):
                social.main()
        context = json.loads(out.getvalue())
        self.assertIn('Max', context['memory_summary'])
        self.assertIn('follow_up_topic:music', context['gaps'])
        self.assertIn('room_label_unknown', context['gaps'])
        self.assertEqual(len(SemanticMemory(self.path).conversations), 0)

    def test_spoken_turn_commits_only_after_confirmed_playback(self):
        args = self.args('--speak', '--heard-text', "I'm Max. I like music.",
                         '--reply', 'Music. A promising topic for a creature with speakers.',
                         '--person-id', 'max', '--memory-updates-json',
                         json.dumps({'name':'Max', 'facts':['I like music'], 'topics':['music']}))
        events = []
        def play(audio):
            self.assertEqual(audio, b'mp3')
            self.assertFalse(self.path.exists())
            events.append('played')
            return True
        with redirect_stdout(io.StringIO()):
            result = social.speak_turn(SemanticMemory(self.path), args,
                synthesize=lambda text: b'mp3', play=play,
                authority=lambda: (None, None, True))
        self.assertEqual(events, ['played'])
        self.assertTrue(result.memory_written)
        bank = SemanticMemory(self.path)
        self.assertEqual(bank.people['max']['name'], 'Max')
        self.assertEqual(bank.people['max']['known_facts'], ['I like music'])
        self.assertEqual(len(bank.conversations), 1)
        self.assertEqual(bank.conversations[0]['spoken_reply'], args.reply)
        self.assertEqual(bank.conversations[0]['heard_text'], args.heard_text)

    def test_failed_playback_or_authority_never_creates_memory(self):
        args = self.args('--speak', '--heard-text', 'Hello.', '--reply', 'Hello back.')
        for authority, play in ((lambda: (None, None, True), lambda _: False),
                                (lambda: (None, None, False), lambda _: True)):
            with redirect_stdout(io.StringIO()), self.assertRaises(SystemExit):
                social.speak_turn(SemanticMemory(self.path), args,
                    synthesize=lambda _: b'audio', play=play, authority=authority)
            self.assertFalse(self.path.exists())

    def test_owner_change_after_playback_does_not_commit(self):
        args = self.args('--speak', '--heard-text', 'Hello.', '--reply', 'Hello back.')
        checks = iter([(None, None, True)] * 3 + [(None, 'new-lease', True)])
        with redirect_stdout(io.StringIO()), self.assertRaises(SystemExit):
            social.speak_turn(SemanticMemory(self.path), args,
                synthesize=lambda _: b'audio', play=lambda _: True,
                authority=lambda: next(checks))
        self.assertFalse(self.path.exists())

    def test_unsupported_memory_fields_rejected_before_synthesis(self):
        args = self.args('--speak', '--heard-text', 'Hello.', '--reply', 'Hello back.',
                         '--memory-updates-json', '{"name":"Max"}')
        with self.assertRaises(AudioContractError):
            social.speak_turn(SemanticMemory(self.path), args,
                synthesize=lambda _: self.fail('must not synthesize'),
                play=lambda _: self.fail('must not play'),
                authority=lambda: (None, None, True))
        self.assertFalse(self.path.exists())
        args.memory_updates_json = '{"topics":["invented submarines"]}'
        with self.assertRaises(AudioContractError):
            social.speak_turn(SemanticMemory(self.path), args,
                synthesize=lambda _: self.fail('must not synthesize'),
                play=lambda _: self.fail('must not play'),
                authority=lambda: (None, None, True))
        self.assertFalse(self.path.exists())

    def test_speaker_filter_and_headphones_card_mocked(self):
        listing = 'card 2: Headphones [bcm2835 Headphones], device 0: bcm2835 Headphones\n'
        class Result:
            returncode = 0
            stdout = listing
        with patch.object(social.subprocess, 'run', return_value=Result()) as run:
            self.assertTrue(social.analog_playback(b'mp3', 4.5))
        self.assertEqual(run.call_args_list[1].args[0][1:4], ['-c', '2', 'set'])
        self.assertIn('volume=4.500,alimiter=limit=0.95:level=0', run.call_args_list[2].args[0])
        self.assertEqual(run.call_args_list[3].args[0][-1], 'plughw:2,0')

    def test_host_relays_audio_and_verifies_cleanup_without_chat_api(self):
        class Response:
            status_code = 200
            def __init__(self, content=None): self.content = content
            def raise_for_status(self): pass
            def json(self): return {'content': self.content}
        class Session:
            def __init__(self): self.upload = None; self.reads = 0
            def put(self, url, headers, json, timeout):
                self.upload = json['content']
                self.asserted_url = url
                return Response()
            def get(self, url, timeout):
                self.reads += 1
                if self.reads == 1: return Response(self.upload)
                result = Response(); result.status_code = 404
                return result
            def delete(self, *args, **kwargs):
                raise AssertionError('unlink should not need delete-to-trash fallback')
        session = Session()
        args = SimpleNamespace(read_context=False, heard_text='Hello', reply='Hello back',
            memory_updates_json=None, person_id=None, speaker_gain=4.0)
        outputs = AsyncMock(side_effect=[{'state':'replied','memory_written':True},
                                         {'removed':True}])
        with (patch.object(host_relay, 'login', return_value=(session, 'xsrf')),
              patch.object(host_relay, 'edge_sonia', return_value=b'fake-mp3'),
              patch.object(host_relay, 'execute', outputs)):
            result = host_relay.relay(args)
        self.assertTrue(result['memory_written'])
        self.assertIn('relay-', session.asserted_url)
        self.assertEqual(session.reads, 2)


if __name__ == '__main__':
    unittest.main()
