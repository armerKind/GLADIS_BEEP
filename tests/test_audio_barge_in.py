"""Offline Phase D audio contract tests; synthetic PCM is not a Pi AEC benchmark."""
import math
import struct
from io import BytesIO
import unittest

from beep_audio.capture import AudioFrame, BoundedCaptureRing, AudioContractError, PcmCapture
from beep_audio.echo import RenderFrame, FrameAlignedEcho, BargeInGate
from beep_audio.vad import HysteresisVad


def pcm(amplitude, *, phase=0):
    return struct.pack('<320h', *(int(amplitude * math.sin(2 * math.pi * (i + phase) / 32))
                                    for i in range(320)))


def mix(a, b):
    x, y = struct.unpack('<320h', a), struct.unpack('<320h', b)
    return struct.pack('<320h', *(max(-32768, min(32767, p + q)) for p, q in zip(x, y)))


def frame(index, sound, *, time=None, quality='measured_capture'):
    return AudioFrame('s', 'b', index, index * .02 if time is None else time,
                      quality, sound)


def rendered(index, sound, *, time=None, confirmed=True):
    return RenderFrame('s', 'b', index, index * .02 if time is None else time,
                       sound, confirmed)


class AudioTests(unittest.TestCase):
    def test_injected_pcm_capture_is_bounded_and_does_not_invent_exposure_time(self):
        source = BytesIO(pcm(100) * 3 + b'partial')
        stamp = iter([1., 1.02, 1.04])
        capture = PcmCapture(source, lambda: next(stamp), 's', 'b', capacity=2)
        for index in range(3):
            sample = capture.read_one()
            self.assertEqual(sample.sequence, index)
            self.assertEqual(sample.timestamp_quality, 'ingest_approximate')
        self.assertEqual([x.sequence for x in capture.ring.snapshot()], [1, 2])
        with self.assertRaises(AudioContractError):
            capture.read_one()

    def test_bounded_capture_validates_identity_order_and_pcm(self):
        ring = BoundedCaptureRing('s', 'b', capacity=2)
        for index in range(3):
            ring.append(frame(index, pcm(500)))
        self.assertEqual([x.sequence for x in ring.snapshot()], [1, 2])
        with self.assertRaises(AudioContractError):
            ring.append(frame(2, pcm(500)))
        with self.assertRaises(AudioContractError):
            ring.append(AudioFrame('s', 'other', 3, .06, 'measured_capture', pcm(500)))
        with self.assertRaises(AudioContractError):
            frame(3, b'bad')

    def test_vad_hysteresis_and_gait_only_is_not_speech_recognition(self):
        vad = HysteresisVad(onset_rms=400, offset_rms=200, onset_frames=2,
                            offset_frames=2)
        self.assertEqual(vad.observe(pcm(100)), None)
        self.assertEqual(vad.observe(pcm(1000)), None)
        self.assertEqual(vad.observe(pcm(1000)), 'onset')
        self.assertIsNone(vad.observe(pcm(100)))
        self.assertEqual(vad.observe(pcm(100)), 'end')
        self.assertFalse(vad.active)

    def test_playback_echo_only_cannot_barge_but_double_talk_can(self):
        echo = FrameAlignedEcho('s', 'b', delay_s=.04, echo_gain=.5)
        gate = BargeInGate(echo, HysteresisVad(onset_rms=350, offset_rms=200,
                                               onset_frames=2, offset_frames=2))
        speech = pcm(1800, phase=8)
        for index in range(4):
            voice = pcm(1200)
            echo.submit_render(rendered(index, voice))
            recording = pcm(600) if index < 2 else mix(pcm(600), speech)
            result = gate.observe(frame(index, recording, time=.04 + index * .02),
                                  playback_active=True)
            self.assertFalse(result.motor_authority)
            if index < 3:
                self.assertFalse(result.cut_tts)
        self.assertTrue(result.cut_tts)
        self.assertEqual(result.reason, 'near_end_vad_onset')
        self.assertEqual(result.audio_event, 'vad_onset')

    def test_unknown_reference_drift_and_ingest_stamp_degrade_to_short_chunks(self):
        echo = FrameAlignedEcho('s', 'b', delay_s=.04, echo_gain=.5)
        gate = BargeInGate(echo, HysteresisVad(onset_rms=300, offset_rms=150,
                                               onset_frames=2, offset_frames=2))
        echo.submit_render(rendered(0, pcm(1000)))
        for sample in (frame(0, pcm(2000), time=.04, quality='ingest_approximate'),
                       frame(1, pcm(2000), time=.061),
                       frame(2, pcm(2000), time=.09)):
            result = gate.observe(sample, playback_active=True)
            self.assertFalse(result.cut_tts)
            self.assertTrue(result.short_tts_chunks)
        self.assertEqual(result.reason, 'echo_reference_unavailable')

    def test_unconfirmed_render_and_identity_mismatch_never_certify_aec(self):
        echo = FrameAlignedEcho('s', 'b', delay_s=.04, echo_gain=.5)
        with self.assertRaises(AudioContractError):
            echo.submit_render(rendered(0, pcm(1000), confirmed=False))
        with self.assertRaises(AudioContractError):
            echo.submit_render(RenderFrame('s', 'other', 0, 0., pcm(1000), True))
        gate = BargeInGate(echo, HysteresisVad(onset_rms=300, offset_rms=150,
                                               onset_frames=2, offset_frames=2))
        self.assertFalse(gate.observe(frame(0, pcm(2000), time=.04),
                                      playback_active=True).cut_tts)
        self.assertFalse(gate.observe(frame(1, pcm(2000), time=.06),
                                      playback_active=True).cut_tts)

    def test_accumulated_clock_drift_fails_closed_after_alignment_window(self):
        echo = FrameAlignedEcho('s', 'b', delay_s=.04, echo_gain=.5)
        gate = BargeInGate(echo, HysteresisVad(onset_rms=300, offset_rms=150,
                                               onset_frames=2, offset_frames=2))
        for index in range(8):
            echo.submit_render(rendered(index, pcm(1000)))
            observed = gate.observe(frame(index, pcm(1500),
                                          time=.04 + index * .0205),
                                    playback_active=True)
            if index >= 5:
                self.assertTrue(observed.short_tts_chunks)
                self.assertFalse(observed.cut_tts)
                self.assertEqual(observed.reason, 'echo_reference_unavailable')

    def test_no_playback_vad_stays_live_without_echo_reference(self):
        gate = BargeInGate(FrameAlignedEcho('s', 'b', delay_s=.04, echo_gain=.5),
                           HysteresisVad(onset_rms=300, offset_rms=150,
                                         onset_frames=2, offset_frames=2))
        self.assertFalse(gate.observe(frame(0, pcm(1000)), playback_active=False).cut_tts)
        second = gate.observe(frame(1, pcm(1000)), playback_active=False)
        self.assertEqual(second.audio_event, 'vad_onset')
        self.assertFalse(second.cut_tts)
        self.assertFalse(second.motor_authority)
        self.assertEqual(gate.observe(frame(2, pcm(0)), playback_active=False).audio_event, None)
        self.assertEqual(gate.observe(frame(3, pcm(0)), playback_active=False).audio_event,
                         'vad_end')

    def test_capture_gap_resets_partial_onset_and_playback_transition(self):
        gate = BargeInGate(FrameAlignedEcho('s', 'b', delay_s=.04, echo_gain=.5),
                           HysteresisVad(onset_rms=300, offset_rms=150,
                                         onset_frames=2, offset_frames=2))
        self.assertIsNone(gate.observe(frame(0, pcm(1000)), playback_active=False).audio_event)
        self.assertIsNone(gate.observe(frame(2, pcm(1000)), playback_active=False).audio_event)
        self.assertEqual(gate.observe(frame(3, pcm(1000)),
                                      playback_active=False).audio_event, 'vad_onset')
        self.assertTrue(gate.observe(frame(4, pcm(1000)),
                                     playback_active=True).short_tts_chunks)


if __name__ == '__main__':
    unittest.main()
