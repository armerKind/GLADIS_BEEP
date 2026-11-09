"""Synthetic independent-clock and social handoff regressions; not live AEC."""
import math
import struct
import unittest

from beep_audio.capture import AudioFrame, AudioContractError
from beep_audio.echo import RenderFrame, BargeDecision, BargeInGate
from beep_audio.adaptive import AdaptiveEcho
from beep_audio.social import SocialHandoff, TranscriptResult
from beep_audio.vad import HysteresisVad


def wave(t):
    return int(1700 * math.sin(2 * math.pi * (230 * t + 17 * t * t)))


def pcm(fn):
    return struct.pack('<320h', *(max(-32768, min(32767, fn(i))) for i in range(320)))


def capture(index, stamp, samples, quality='measured_capture'):
    return AudioFrame('s', 'b', index, stamp, quality, samples)


def render(index, stamp, samples, confirmed=True):
    return RenderFrame('s', 'b', index, stamp, samples, confirmed)


class AdaptiveTests(unittest.TestCase):
    def test_exhausted_render_history_does_not_reuse_a_newer_reference(self):
        echo = AdaptiveEcho('s', 'b', delay_s=.04, echo_gain=.5)
        for index in range(140):
            echo.submit_render(render(index, index * .02, pcm(lambda k: 1000)))
        self.assertLessEqual(len(echo._renders), 128)
        for index in range(12):
            result = echo.process(capture(index, index * .02 + .04,
                                          pcm(lambda k: 500)))
            self.assertIsNone(result)
        self.assertIsNone(echo.process(capture(12, .28, pcm(lambda k: 500))))

    def test_both_capture_and_render_clocks_can_drift_independently(self):
        echo = AdaptiveEcho('s', 'b', delay_s=.04, echo_gain=.5, max_ppm=300)
        errors = []
        for index in range(85):
            rtime = index * .02 * (1 - 120 / 1e6)
            echo.submit_render(render(index, rtime,
                        pcm(lambda k: wave(rtime + k * .02 * (1 - 120 / 1e6) / 320))))
            if index >= 2:
                ctime = (index - 2) * .02 * (1 + 170 / 1e6) + .04
                recorded = pcm(lambda k: round(.5 * wave(
                    ctime - .04 + k * .02 * (1 + 170 / 1e6) / 320)))
                residual = echo.process(capture(index - 2, ctime, recorded))
                if index < 13:
                    self.assertIsNone(residual)  # Bounded clock-regression warmup.
                    continue
                assert residual is not None
                if index > 32:
                    errors.append(max(abs(x) for x in struct.unpack('<320h', residual)))
        self.assertLess(max(errors), 110)

    def test_positive_and_negative_ppm_resampling_with_near_end_preserved(self):
        for ppm in (-180, 180):
            with self.subTest(ppm=ppm):
                echo = AdaptiveEcho('s', 'b', delay_s=.04, echo_gain=.5,
                                    max_ppm=300, max_jitter_s=.0005)
                residuals = []
                for i in range(85):
                    render_stamp = i * .02 * (1 + ppm / 1e6)
                    render_pcm = pcm(lambda k: wave(render_stamp + k * .02 * (1 + ppm / 1e6) / 320))
                    echo.submit_render(render(i, render_stamp, render_pcm))
                    if i < 2:
                        continue
                    stamp = (i - 2) * .02 + .04
                    speech = 500 if i >= 50 else 0
                    mic = pcm(lambda k: round(.5 * wave(stamp - .04 + k / 16000)) + speech)
                    residual = echo.process(capture(i - 2, stamp, mic))
                    if i < 13:
                        self.assertIsNone(residual)
                        continue
                    assert residual is not None, (ppm, i)
                    if i > 32:
                        samples = struct.unpack('<320h', residual)
                        residuals.append(sum(abs(v - speech) for v in samples) / 320)
                self.assertLess(max(residuals), 110)

    def test_render_jitter_is_smoothed_but_jump_and_unconfirmed_frame_fail_closed(self):
        echo = AdaptiveEcho('s', 'b', delay_s=.04, echo_gain=.5,
                            max_ppm=300, max_jitter_s=.0006)
        for i in range(60):
            jitter = 0. if i == 0 else (.00015 if i % 2 else -.00015)
            echo.submit_render(render(i, i * .02 + jitter, pcm(lambda k: 500)))
            if i >= 2:
                result = echo.process(capture(i - 2, i * .02, pcm(lambda k: 250)))
                if i >= 13:
                    self.assertIsNotNone(result)
                else:
                    self.assertIsNone(result)
        echo.submit_render(render(60, 1.23, pcm(lambda k: 500)))
        self.assertIsNone(echo.process(capture(58, 1.20, pcm(lambda k: 250))))
        with self.assertRaises(AudioContractError):
            echo.submit_render(render(61, 1.24, pcm(lambda k: 500), confirmed=False))

    def test_excess_drift_unknown_capture_and_route_reset_fail_closed(self):
        echo = AdaptiveEcho('s', 'b', delay_s=.04, echo_gain=.5, max_ppm=300)
        for i in range(55):
            echo.submit_render(render(i, i * .02002, pcm(lambda k: 500)))
            if i >= 2:
                output = echo.process(capture(i - 2, i * .02,
                                              pcm(lambda k: 250)))
        self.assertIsNone(output)  # 1000 ppm exceeds explicit correction budget.
        echo.reset_route()
        self.assertIsNone(echo.process(capture(53, 1.10, pcm(lambda k: 250))))
        echo.submit_render(render(55, 1.12, pcm(lambda k: 500)))
        self.assertIsNone(echo.process(capture(54, 1.14, pcm(lambda k: 250),
                                               quality='ingest_approximate')))


class SocialTests(unittest.TestCase):
    def test_synthetic_echo_barge_cut_to_exact_owner_transcription_handoff(self):
        echo = AdaptiveEcho('s', 'b', delay_s=.04, echo_gain=.5)
        gate = BargeInGate(echo, HysteresisVad(onset_rms=300, offset_rms=150,
                                               onset_frames=2, offset_frames=2))
        handoff = SocialHandoff('s', 'b')
        playback = True
        requests = []
        for index in range(18):
            if playback:
                echo.submit_render(render(index, index * .02, pcm(lambda k: 1000)))
            sample = capture(index, .04 + index * .02,
                             pcm(lambda k: (1500 if index >= 13 else 500) if playback
                                 else (1000 if index == 15 else 0)))
            decision = gate.observe(sample, playback_active=playback)
            action = handoff.observe(sample, decision, mission_id='m', lease_id='l',
                                     upload_consented=True)
            if action.cut_tts:
                playback = False  # Fake mixer; no actual speaker call.
            if action.transcription is not None:
                requests.append(action.transcription)
        self.assertEqual(len(requests), 1)
        self.assertEqual(requests[0].pcm_s16le[:640], pcm(lambda k: 1000))
        self.assertEqual(requests[0].utterance_id, 'b:14')
        self.assertFalse(requests[0].motor_authority)

    def test_tts_cut_then_utterance_end_yields_one_consented_request(self):
        handoff = SocialHandoff('s', 'b', max_frames=4)
        def decision(event=None, cut=False):
            return BargeDecision(cut, False, event, 'synthetic', near_end_pcm_s16le=pcm(lambda k: 200))
        onset = handoff.observe(capture(0, 1., pcm(lambda k: 1000)),
                                decision('vad_onset', True), mission_id='m',
                                lease_id='l', upload_consented=True)
        self.assertTrue(onset.cut_tts)
        self.assertIsNone(onset.transcription)
        handoff.observe(capture(1, 1.02, pcm(lambda k: 1000)), decision(),
                        mission_id='m', lease_id='l', upload_consented=True)
        ended = handoff.observe(capture(2, 1.04, pcm(lambda k: 0)),
                                decision('vad_end'), mission_id='m', lease_id='l',
                                upload_consented=True)
        self.assertFalse(ended.cut_tts)
        assert ended.transcription is not None
        self.assertEqual(len(ended.transcription.pcm_s16le), 1920)
        self.assertEqual(ended.transcription.pcm_s16le[:640], pcm(lambda k: 200))
        self.assertTrue(all(not hasattr(value, 'pcm_s16le')
                            for value in handoff._pending.values()))
        self.assertEqual((ended.transcription.mission_id, ended.transcription.lease_id), ('m', 'l'))
        self.assertFalse(ended.transcription.motor_authority)
        self.assertIsNone(handoff.observe(capture(3, 1.06, pcm(lambda k: 0)),
                                          decision(), mission_id='m', lease_id='l',
                                          upload_consented=True).transcription)

    def test_abort_consent_revocation_owner_change_and_cap_never_upload(self):
        def decision(event=None):
            return BargeDecision(False, False, event, 'synthetic', near_end_pcm_s16le=pcm(lambda k: 200))
        for terminal in ('vad_aborted', 'consent', 'owner', 'limit'):
            with self.subTest(terminal=terminal):
                handoff = SocialHandoff('s', 'b', max_frames=2)
                handoff.observe(capture(0, 1., pcm(lambda k: 1000)), decision('vad_onset'),
                                mission_id='m', lease_id='l', upload_consented=True)
                if terminal == 'vad_aborted':
                    event, consent, owner = 'vad_aborted', True, 'l'
                elif terminal == 'consent':
                    event, consent, owner = 'vad_end', False, 'l'
                elif terminal == 'owner':
                    event, consent, owner = 'vad_end', True, 'replacement'
                else:
                    handoff.observe(capture(1, 1.02, pcm(lambda k: 1000)), decision(),
                                    mission_id='m', lease_id='l', upload_consented=True)
                    event, consent, owner = None, True, 'l'
                out = handoff.observe(capture(2 if terminal == 'limit' else 1,
                                              1.04 if terminal == 'limit' else 1.02,
                                              pcm(lambda k: 1000)), decision(event),
                                      mission_id='m', lease_id=owner,
                                      upload_consented=consent)
                self.assertIsNone(out.transcription)
                self.assertTrue(out.utterance_aborted)

    def test_unmeasured_capture_and_invalid_decision_cannot_be_uploaded(self):
        handoff = SocialHandoff('s', 'b')
        with self.assertRaises(AudioContractError):
            handoff.observe(capture(0, 0., pcm(lambda k: 10), 'ingest_approximate'),
                            BargeDecision(False, False, 'vad_onset', 'synthetic',
                                          near_end_pcm_s16le=pcm(lambda k: 10)),
                            mission_id=None, lease_id=None, upload_consented=True)
        with self.assertRaises(AudioContractError):
            handoff.observe(capture(0, 0., pcm(lambda k: 10)),
                            BargeDecision(False, False, 'vad_onset', 'synthetic', True),
                            mission_id=None, lease_id=None, upload_consented=True)

    def test_missing_near_end_pcm_aborts_instead_of_uploading_raw_tts_echo(self):
        handoff = SocialHandoff('s', 'b')
        result = handoff.observe(capture(0, .04, pcm(lambda k: 1500)),
                                 BargeDecision(True, False, 'vad_onset', 'synthetic'),
                                 mission_id=None, lease_id=None, upload_consented=True)
        self.assertTrue(result.utterance_aborted)
        self.assertIsNone(result.transcription)

    def test_transcript_return_is_exact_once_fresh_and_advisory_only(self):
        handoff = SocialHandoff('s', 'b')
        sample = pcm(lambda k: 800)
        onset = BargeDecision(True, False, 'vad_onset', 'synthetic',
                              near_end_pcm_s16le=sample)
        end = BargeDecision(False, False, 'vad_end', 'synthetic',
                            near_end_pcm_s16le=sample)
        handoff.observe(capture(0, 1., sample), onset, mission_id='m', lease_id='l',
                        upload_consented=True)
        request = handoff.observe(capture(1, 1.02, sample), end, mission_id='m',
                                  lease_id='l', upload_consented=True).transcription
        assert request is not None
        result = TranscriptResult(request.utterance_id, 's', 'b', 'm', 'l',
                                  'Stop please', 1.5)
        self.assertIsNone(handoff.accept_transcript(result, mission_id='m',
                          lease_id='replacement', now_robot=1.6, upload_consented=True))
        # A wrong owner cannot be repaired by later reusing a stale result.
        self.assertIsNone(handoff.accept_transcript(result, mission_id='m',
                          lease_id='l', now_robot=1.7, upload_consented=True))

        handoff = SocialHandoff('s', 'b')
        handoff.observe(capture(0, 1., sample), onset, mission_id='m', lease_id='l',
                        upload_consented=True)
        req = handoff.observe(capture(1, 1.02, sample), end, mission_id='m',
                              lease_id='l', upload_consented=True).transcription
        assert req is not None
        result = TranscriptResult(req.utterance_id, 's', 'b', 'm', 'l', 'Stop please', 1.5)
        accepted = handoff.accept_transcript(result, mission_id='m', lease_id='l',
                                             now_robot=1.6, upload_consented=True)
        assert accepted is not None
        self.assertEqual(accepted.text, 'Stop please')
        self.assertFalse(accepted.motor_authority)
        self.assertIsNone(handoff.accept_transcript(result, mission_id='m', lease_id='l',
                                                    now_robot=1.7, upload_consented=True))

    def test_transcript_expiry_and_consent_revocation(self):
        for consent, now in ((False, 1.5), (True, 12.)):
            handoff = SocialHandoff('s', 'b')
            sound = pcm(lambda k: 500)
            for index, event in ((0, 'vad_onset'), (1, 'vad_end')):
                action = handoff.observe(capture(index, 1. + index * .02, sound),
                    BargeDecision(False, False, event, 'synthetic',
                                  near_end_pcm_s16le=sound), mission_id=None,
                    lease_id=None, upload_consented=True)
            req = action.transcription
            assert req is not None
            result = TranscriptResult(req.utterance_id, 's', 'b', None, None,
                                      'hello', 1.3)
            self.assertIsNone(handoff.accept_transcript(result, mission_id=None,
                              lease_id=None, now_robot=now, upload_consented=consent))

    def test_intervening_owner_change_invalidates_pending_asr_receipt(self):
        handoff = SocialHandoff('s', 'b')
        sound = pcm(lambda k: 500)
        for index, event in ((0, 'vad_onset'), (1, 'vad_end')):
            action = handoff.observe(capture(index, 1. + index * .02, sound),
                BargeDecision(False, False, event, 'synthetic',
                              near_end_pcm_s16le=sound), mission_id='m',
                lease_id='l', upload_consented=True)
        req = action.transcription
        assert req is not None
        handoff.observe(capture(2, 1.04, sound),
                        BargeDecision(False, False, None, 'synthetic'),
                        mission_id='replacement', lease_id='new', upload_consented=True)
        result = TranscriptResult(req.utterance_id, 's', 'b', 'm', 'l', 'hello', 1.5)
        self.assertIsNone(handoff.accept_transcript(result, mission_id='m', lease_id='l',
                                                    now_robot=1.6, upload_consented=True))


if __name__ == '__main__':
    unittest.main()
