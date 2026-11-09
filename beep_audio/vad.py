"""Offline energy VAD with hysteresis; not speech or stop-command recognition."""
from __future__ import annotations

from math import isfinite, sqrt
import struct

from .capture import AudioContractError, _pcm


class HysteresisVad:
    """RMS in signed-16-bit sample units; thresholds require real gait calibration."""

    def __init__(self, *, onset_rms: float, offset_rms: float,
                 onset_frames: int = 2, offset_frames: int = 3):
        if (any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                not isfinite(v) for v in (onset_rms, offset_rms))
                or not 0 < offset_rms < onset_rms <= 32767
                or type(onset_frames) is not int or not 1 <= onset_frames <= 50
                or type(offset_frames) is not int or not 1 <= offset_frames <= 50):
            raise AudioContractError('invalid VAD hysteresis')
        self.onset_rms, self.offset_rms = onset_rms, offset_rms
        self.onset_frames, self.offset_frames = onset_frames, offset_frames
        self.active = False
        self._above = self._below = 0

    def reset(self) -> None:
        self.active = False
        self._above = self._below = 0

    def clear_partial(self) -> None:
        """Switch capture mode without losing an active utterance edge."""
        self._above = self._below = 0

    def abort(self) -> bool:
        """Return whether an open utterance needs a terminal aborted event."""
        was_active = self.active
        self.reset()
        return was_active

    def observe(self, pcm_s16le: bytes) -> str | None:
        if not _pcm(pcm_s16le):
            raise AudioContractError('invalid VAD PCM')
        samples = struct.unpack('<320h', pcm_s16le)
        rms = sqrt(sum(sample * sample for sample in samples) / len(samples))
        if not self.active:
            self._above = self._above + 1 if rms >= self.onset_rms else 0
            if self._above >= self.onset_frames:
                self.active = True
                self._above = self._below = 0
                return 'onset'
        else:
            self._below = self._below + 1 if rms < self.offset_rms else 0
            if self._below >= self.offset_frames:
                self.active = False
                self._above = self._below = 0
                return 'end'
        return None
