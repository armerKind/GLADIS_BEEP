"""Phase D offline echo/reference and barge-in contract.

FrameAlignedEcho is a deliberately simple, fixed-delay *synthetic linear echo*
model. It is NOT a production AEC or a stop-command recognizer. Replace it with
measured SpeexDSP/WebRTC AEC before enabling live TTS barge-in.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import isfinite
import struct

from .capture import AudioContractError, AudioFrame, _identity, _pcm
from .vad import HysteresisVad


@dataclass(frozen=True)
class RenderFrame:
    session_id: str
    boot_id: str
    sequence: int
    rendered_at_robot: float
    pcm_s16le: bytes
    actually_rendered: bool

    def __post_init__(self):
        if (not _identity(self.session_id) or not _identity(self.boot_id)
                or type(self.sequence) is not int or self.sequence < 0
                or isinstance(self.rendered_at_robot, bool)
                or not isinstance(self.rendered_at_robot, (int, float))
                or not isfinite(self.rendered_at_robot) or self.rendered_at_robot < 0
                or not _pcm(self.pcm_s16le)
                or type(self.actually_rendered) is not bool):
            raise AudioContractError('invalid rendered reference frame')


class FrameAlignedEcho:
    """Exact-identity alignment and fixed-gain synthetic subtraction; no drift repair."""

    def __init__(self, session_id: str, boot_id: str, *, delay_s: float,
                 echo_gain: float, tolerance_s: float = .002):
        if (not _identity(session_id) or not _identity(boot_id)
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not isfinite(v) for v in (delay_s, echo_gain, tolerance_s))
                or not 0 <= delay_s <= .5 or not 0 <= echo_gain <= 2
                or not 0 < tolerance_s <= .005):
            raise AudioContractError('invalid synthetic echo configuration')
        self.session_id, self.boot_id = session_id, boot_id
        self.delay_s, self.echo_gain, self.tolerance_s = delay_s, echo_gain, tolerance_s
        self._renders: deque[RenderFrame] = deque(maxlen=100)
        self._render_seq = self._capture_seq = -1
        self._render_stamp = self._capture_stamp = -1.0

    def submit_render(self, frame: RenderFrame) -> None:
        if (not isinstance(frame, RenderFrame)
                or (frame.session_id, frame.boot_id) != (self.session_id, self.boot_id)
                or not frame.actually_rendered or frame.sequence <= self._render_seq
                or frame.rendered_at_robot <= self._render_stamp):
            raise AudioContractError('unconfirmed, stale or wrong-owner render reference')
        self._renders.append(frame)
        self._render_seq, self._render_stamp = frame.sequence, frame.rendered_at_robot

    def process(self, capture: AudioFrame) -> bytes | None:
        if (not isinstance(capture, AudioFrame)
                or (capture.session_id, capture.boot_id) != (self.session_id, self.boot_id)
                or capture.sequence <= self._capture_seq
                or capture.acquired_at_robot <= self._capture_stamp):
            raise AudioContractError('stale or wrong-owner capture for echo processing')
        self._capture_seq = capture.sequence
        self._capture_stamp = capture.acquired_at_robot
        if capture.timestamp_quality != 'measured_capture':
            return None
        expected_render_time = capture.acquired_at_robot - self.delay_s
        while self._renders and self._renders[0].rendered_at_robot < expected_render_time - self.tolerance_s:
            self._renders.popleft()
        if (not self._renders or abs(self._renders[0].rendered_at_robot - expected_render_time)
                > self.tolerance_s):
            return None  # Unknown alignment/drift/underrun cannot certify near-end speech.
        rendered = self._renders.popleft()
        mic = struct.unpack('<320h', capture.pcm_s16le)
        ref = struct.unpack('<320h', rendered.pcm_s16le)
        return struct.pack('<320h', *(max(-32768, min(32767, round(m - self.echo_gain * r)))
                                      for m, r in zip(mic, ref)))


@dataclass(frozen=True)
class BargeDecision:
    cut_tts: bool
    short_tts_chunks: bool
    audio_event: str | None
    reason: str
    motor_authority: bool = False


class BargeInGate:
    """Never closes the mic during TTS. No reference means no asserted barge-in."""

    def __init__(self, echo: FrameAlignedEcho, vad: HysteresisVad):
        self.echo, self.vad = echo, vad
        self._last_capture_seq = -1
        self._last_capture_stamp = -1.0
        self._was_playback: bool | None = None

    def observe(self, capture: AudioFrame, *, playback_active: bool) -> BargeDecision:
        if (not isinstance(capture, AudioFrame)
                or (capture.session_id, capture.boot_id) !=
                   (self.echo.session_id, self.echo.boot_id)
                or capture.sequence <= self._last_capture_seq
                or capture.acquired_at_robot <= self._last_capture_stamp
                or type(playback_active) is not bool):
            raise AudioContractError('invalid barge-in observation')
        # Dropped capture frames invalidate a consecutive VAD onset.
        if ((self._last_capture_seq >= 0 and capture.sequence != self._last_capture_seq + 1)
                or (self._was_playback is not None and playback_active != self._was_playback)):
            self.vad.reset()
        self._last_capture_seq = capture.sequence
        self._last_capture_stamp = capture.acquired_at_robot
        self._was_playback = playback_active
        if not playback_active:
            event = self.vad.observe(capture.pcm_s16le)
            return BargeDecision(False, False, None if event is None else 'vad_' + event,
                                 'mic_vad_only')
        residual = self.echo.process(capture)
        if residual is None:
            self.vad.reset()
            return BargeDecision(False, True, None, 'echo_reference_unavailable')
        event = self.vad.observe(residual)
        return BargeDecision(event == 'onset', False,
                             None if event is None else 'vad_' + event,
                             'near_end_vad_onset' if event == 'onset' else 'aligned_reference')
