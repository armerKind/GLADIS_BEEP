"""Offline, bounded 16 kHz mono PCM frame contract. No ALSA device opened."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import isfinite
from typing import BinaryIO, Callable


class AudioContractError(ValueError):
    pass


def _identity(value: str) -> bool:
    return isinstance(value, str) and 0 < len(value) <= 256 and bool(value.strip())


def _pcm(value: bytes | None) -> bool:
    return isinstance(value, bytes) and len(value) == 640  # 20 ms, 16 kHz, signed LE mono


@dataclass(frozen=True)
class AudioFrame:
    session_id: str
    boot_id: str
    sequence: int
    acquired_at_robot: float
    timestamp_quality: str
    pcm_s16le: bytes

    def __post_init__(self):
        if (not _identity(self.session_id) or not _identity(self.boot_id)
                or type(self.sequence) is not int or self.sequence < 0
                or isinstance(self.acquired_at_robot, bool)
                or not isinstance(self.acquired_at_robot, (int, float))
                or not isfinite(self.acquired_at_robot) or self.acquired_at_robot < 0
                or self.timestamp_quality not in {'measured_capture', 'ingest_approximate'}
                or not _pcm(self.pcm_s16le)):
            raise AudioContractError('invalid 20 ms capture frame')


class BoundedCaptureRing:
    """Keep a fixed number of ordered frames; ingestion time is not exposure time."""

    def __init__(self, session_id: str, boot_id: str, *, capacity: int = 250):
        if (not _identity(session_id) or not _identity(boot_id)
                or type(capacity) is not int or not 1 <= capacity <= 1500):
            raise AudioContractError('invalid capture ring configuration')
        self.session_id, self.boot_id = session_id, boot_id
        self._frames: deque[AudioFrame] = deque(maxlen=capacity)
        self._last_sequence = -1
        self._last_stamp = -1.0

    def append(self, frame: AudioFrame) -> None:
        if (not isinstance(frame, AudioFrame)
                or (frame.session_id, frame.boot_id) != (self.session_id, self.boot_id)
                or frame.sequence <= self._last_sequence
                or frame.acquired_at_robot <= self._last_stamp):
            raise AudioContractError('capture identity, order or clock mismatch')
        self._frames.append(frame)
        self._last_sequence, self._last_stamp = frame.sequence, frame.acquired_at_robot

    def snapshot(self) -> tuple[AudioFrame, ...]:
        return tuple(self._frames)


class PcmCapture:
    """Read one complete frame from an injected PCM source, never infer exposure time."""

    def __init__(self, source: BinaryIO, clock: Callable[[], float],
                 session_id: str, boot_id: str, *, capacity: int = 250):
        if not callable(getattr(source, 'read', None)) or not callable(clock):
            raise AudioContractError('PCM source and monotonic clock required')
        self.source, self.clock = source, clock
        self.ring = BoundedCaptureRing(session_id, boot_id, capacity=capacity)
        self._next_sequence = 0

    def read_one(self) -> AudioFrame:
        data = self.source.read(640)
        if not _pcm(data):
            raise AudioContractError('capture EOF or partial PCM frame')
        frame = AudioFrame(self.ring.session_id, self.ring.boot_id,
                           self._next_sequence, self.clock(), 'ingest_approximate', data)
        self.ring.append(frame)
        self._next_sequence += 1
        return frame
