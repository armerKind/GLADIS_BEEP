"""Offline social-interruption/ASR handoff, without playback or cloud side effects."""
from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass, replace
from math import isfinite

from .capture import AudioContractError, AudioFrame, _identity, _pcm
from .echo import BargeDecision


@dataclass(frozen=True)
class TranscriptionRequest:
    utterance_id: str
    session_id: str
    boot_id: str
    mission_id: str | None
    lease_id: str | None
    first_acquired_at_robot: float
    last_acquired_at_robot: float
    pcm_s16le: bytes
    timestamp_quality: str = 'measured_capture'
    motor_authority: bool = False


@dataclass(frozen=True)
class TranscriptResult:
    utterance_id: str
    session_id: str
    boot_id: str
    mission_id: str | None
    lease_id: str | None
    text: str
    completed_at_robot: float

    def __post_init__(self):
        if (not all(_identity(v) for v in
                    (self.utterance_id, self.session_id, self.boot_id))
                or (self.mission_id is None) != (self.lease_id is None)
                or any(v is not None and not _identity(v) for v in
                       (self.mission_id, self.lease_id))
                or not isinstance(self.text, str) or not self.text.strip()
                or len(self.text) > 1000 or type(self.completed_at_robot) not in (int, float)
                or not isfinite(self.completed_at_robot) or self.completed_at_robot < 0):
            raise AudioContractError('invalid ASR result')


@dataclass(frozen=True)
class SocialTranscript:
    utterance_id: str
    text: str
    session_id: str
    boot_id: str
    mission_id: str | None
    lease_id: str | None
    motor_authority: bool = False


@dataclass(frozen=True)
class _PendingReceipt:
    session_id: str
    boot_id: str
    mission_id: str | None
    lease_id: str | None
    last_acquired_at_robot: float


@dataclass(frozen=True)
class SocialAction:
    cut_tts: bool
    short_tts_chunks: bool
    transcription: TranscriptionRequest | None
    utterance_aborted: bool
    motor_authority: bool = False


class SocialHandoff:
    """Bound one consented utterance to exact ownership and bounded raw PCM.

    An output request is a local handoff, NOT an upload or a speech transcript.
    """

    def __init__(self, session_id: str, boot_id: str, *, max_frames: int = 250):
        if (not _identity(session_id) or not _identity(boot_id)
                or type(max_frames) is not int or not 1 <= max_frames <= 500):
            raise AudioContractError('invalid social handoff configuration')
        self.session_id, self.boot_id, self.max_frames = session_id, boot_id, max_frames
        self._frames: list[AudioFrame] = []
        self._owner: tuple[str | None, str | None] | None = None
        # Retain metadata only. The request carries PCM out to its consumer;
        # waiting for a transcript must not hold raw audio indefinitely.
        self._pending: OrderedDict[str, _PendingReceipt] = OrderedDict()
        self._last_seq = -1
        self._last_stamp = -1.

    def _clear(self) -> None:
        self._frames.clear()
        self._owner = None

    def observe(self, frame: AudioFrame, decision: BargeDecision, *,
                mission_id: str | None, lease_id: str | None,
                upload_consented: bool) -> SocialAction:
        if (not isinstance(frame, AudioFrame) or
                (frame.session_id, frame.boot_id) != (self.session_id, self.boot_id)
                or frame.sequence <= self._last_seq
                or frame.acquired_at_robot <= self._last_stamp
                or not isinstance(decision, BargeDecision) or decision.motor_authority
                or type(decision.cut_tts) is not bool
                or type(decision.short_tts_chunks) is not bool
                or decision.audio_event not in
                   (None, 'vad_onset', 'vad_end', 'vad_aborted')
                or type(upload_consented) is not bool
                or (mission_id is None) != (lease_id is None)
                or any(v is not None and not _identity(v) for v in (mission_id, lease_id))):
            raise AudioContractError('invalid social/ASR handoff identity or decision')
        cut, chunks = decision.cut_tts, decision.short_tts_chunks
        gap = (self._last_seq >= 0 and (frame.sequence != self._last_seq + 1 or not
               .015 <= frame.acquired_at_robot - self._last_stamp <= .025))
        self._last_seq, self._last_stamp = frame.sequence, frame.acquired_at_robot
        if not upload_consented or any(
                (receipt.mission_id, receipt.lease_id) != (mission_id, lease_id)
                for receipt in self._pending.values()):
            self._pending.clear()
        if self._frames and (gap or self._owner != (mission_id, lease_id)
                             or not upload_consented
                             or frame.timestamp_quality != 'measured_capture'):
            self._clear()
            return SocialAction(cut, chunks, None, True)
        if decision.audio_event == 'vad_aborted':
            aborted = bool(self._frames)
            self._clear()
            return SocialAction(cut, chunks, None, aborted)
        # Never hand a raw mic/TTS mixture to ASR. The barge gate must provide
        # its near-end residual, or the request is explicitly abandoned.
        if upload_consented and (self._frames or decision.audio_event == 'vad_onset') and not _pcm(decision.near_end_pcm_s16le):
            self._clear()
            return SocialAction(cut, chunks, None, True)
        if decision.audio_event == 'vad_onset':
            if self._frames:
                self._clear()
                return SocialAction(cut, chunks, None, True)
            if upload_consented:
                if frame.timestamp_quality != 'measured_capture':
                    raise AudioContractError('unmeasured capture cannot form transcription request')
                self._owner = (mission_id, lease_id)
                self._frames.append(replace(frame, pcm_s16le=decision.near_end_pcm_s16le))
            return SocialAction(cut, chunks, None, False)
        if not self._frames:
            return SocialAction(cut, chunks, None, False)
        if len(self._frames) >= self.max_frames:
            self._clear()
            return SocialAction(cut, chunks, None, True)
        self._frames.append(replace(frame, pcm_s16le=decision.near_end_pcm_s16le))
        if decision.audio_event != 'vad_end':
            return SocialAction(cut, chunks, None, False)
        first = self._frames[0]
        request = TranscriptionRequest(
            f'{self.boot_id}:{first.sequence}', self.session_id, self.boot_id,
            mission_id, lease_id, first.acquired_at_robot, frame.acquired_at_robot,
            b''.join(part.pcm_s16le for part in self._frames))
        self._clear()
        self._pending[request.utterance_id] = _PendingReceipt(
            self.session_id, self.boot_id, mission_id, lease_id,
            frame.acquired_at_robot)
        if len(self._pending) > 8:
            self._pending.popitem(last=False)
        return SocialAction(cut, chunks, request, False)

    def accept_transcript(self, result: TranscriptResult, *,
                          mission_id: str | None, lease_id: str | None,
                          now_robot: float, upload_consented: bool) -> SocialTranscript | None:
        """Consume an exact, timely ASR reply once; never execute text as a command."""
        if (not isinstance(result, TranscriptResult)
                or (mission_id is None) != (lease_id is None)
                or any(v is not None and not _identity(v) for v in (mission_id, lease_id))
                or type(upload_consented) is not bool
                or type(now_robot) not in (int, float) or not isfinite(now_robot)
                or now_robot < 0):
            raise AudioContractError('invalid transcript return context')
        if not upload_consented:
            self._pending.clear()
            return None
        request = self._pending.get(result.utterance_id)
        if request is None:
            return None
        owner = (mission_id, lease_id)
        if ((result.session_id, result.boot_id, result.mission_id, result.lease_id) !=
            (request.session_id, request.boot_id, request.mission_id, request.lease_id)
                or owner != (request.mission_id, request.lease_id)
                or (result.session_id, result.boot_id) != (self.session_id, self.boot_id)
                or not request.last_acquired_at_robot <= result.completed_at_robot <= now_robot
                or now_robot > request.last_acquired_at_robot + 10):
            self._pending.pop(result.utterance_id, None)
            return None
        self._pending.pop(result.utterance_id)
        return SocialTranscript(result.utterance_id, result.text.strip(),
                                self.session_id, self.boot_id, mission_id, lease_id)
