"""Offline bounded independent-clock alignment with linear PCM resampling.

Timestamp regression is a *synthetic reference alignment* demonstration, not a
room AEC, drift-control interface to SpeexDSP/WebRTC, or a measured BEEP delay.
"""
from __future__ import annotations

from collections import deque
from math import isfinite
import struct

from .capture import AudioContractError, AudioFrame, _identity
from .echo import RenderFrame


def _period(frames, stamp_attr: str):
    """Least-squares frame period, smoothing zero-mean callback timestamp jitter."""
    first_seq = frames[0].sequence
    xs = [frame.sequence - first_seq for frame in frames]
    ys = [getattr(frame, stamp_attr) for frame in frames]
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    denom = sum((x - mx) ** 2 for x in xs)
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / denom
    return slope, my - slope * mx, first_seq


class AdaptiveEcho:
    """Bounded timestamp-regressed reference resampling + synthetic fixed-gain echo.

    Caller must supply *actually rendered* PCM and measured common-clock stamps.
    No acoustic-path adaptation or near-end classification is performed here.
    """

    def __init__(self, session_id: str, boot_id: str, *, delay_s: float,
                 echo_gain: float, max_ppm: float = 300,
                 max_jitter_s: float = .001):
        if (not _identity(session_id) or not _identity(boot_id)
                or any(type(x) not in (int, float) or not isfinite(x)
                       for x in (delay_s, echo_gain, max_ppm, max_jitter_s))
                or not 0 <= delay_s <= .5 or not 0 <= echo_gain <= 2
                or not 1 <= max_ppm <= 1000 or not 0 < max_jitter_s <= .002):
            raise AudioContractError('invalid adaptive reference configuration')
        self.session_id, self.boot_id = session_id, boot_id
        self.delay_s, self.echo_gain = delay_s, echo_gain
        self.max_ppm, self.max_jitter_s = max_ppm, max_jitter_s
        self._renders: deque[RenderFrame] = deque(maxlen=128)
        self._captures: deque[AudioFrame] = deque(maxlen=64)
        self._last_render_seq = self._last_capture_seq = -1
        self._last_render_stamp = self._last_capture_stamp = -1.
        self._healthy = True

    def reset_route(self) -> None:
        """Discard references and regression on a route change/underrun.

        Sequence and clock fences remain: the caller cannot replay old frames.
        """
        self._renders.clear()
        self._captures.clear()
        self._healthy = True

    def submit_render(self, frame: RenderFrame) -> None:
        if (not isinstance(frame, RenderFrame) or not frame.actually_rendered
                or (frame.session_id, frame.boot_id) != (self.session_id, self.boot_id)
                or frame.sequence <= self._last_render_seq
                or frame.rendered_at_robot <= self._last_render_stamp):
            raise AudioContractError('stale, unconfirmed or wrong-owner render')
        if self._renders:
            previous = self._renders[-1]
            if (frame.sequence != previous.sequence + 1 or
                    abs(frame.rendered_at_robot - previous.rendered_at_robot - .02)
                    > 2 * self.max_jitter_s + .02 * self.max_ppm / 1e6):
                # A playback pause starts a new regression segment. Keep the
                # monotonic fences, but never mix turns or erase this new frame.
                self._renders.clear()
                self._captures.clear()
                self._healthy = True
        self._renders.append(frame)
        self._last_render_seq, self._last_render_stamp = frame.sequence, frame.rendered_at_robot

    def process(self, capture: AudioFrame) -> bytes | None:
        if (not isinstance(capture, AudioFrame)
                or (capture.session_id, capture.boot_id) != (self.session_id, self.boot_id)
                or capture.sequence <= self._last_capture_seq
                or capture.acquired_at_robot <= self._last_capture_stamp):
            raise AudioContractError('stale or wrong-owner capture')
        previous_seq, previous_stamp = self._last_capture_seq, self._last_capture_stamp
        self._last_capture_seq, self._last_capture_stamp = capture.sequence, capture.acquired_at_robot
        if capture.timestamp_quality != 'measured_capture':
            self.reset_route()
            return None
        if previous_seq >= 0 and (capture.sequence != previous_seq + 1 or not
                .015 <= capture.acquired_at_robot - previous_stamp <= .025):
            # The newly submitted render frames may already belong to turn 2.
            # Discard only the stale capture regression, not that render segment.
            self._captures.clear()
        self._captures.append(capture)
        if not self._healthy or len(self._renders) < 12 or len(self._captures) < 12:
            return None  # Period fit not yet stable enough to assert near-end audio.
        rframes = list(self._renders)[-64:]
        render_period, render_origin, render_first = _period(rframes, 'rendered_at_robot')
        # The cap clock may drift independently. Both estimates must stay bounded.
        cframes = list(self._captures)
        cap_period, cap_origin, cap_first = _period(cframes, 'acquired_at_robot')
        if (any(abs(f.rendered_at_robot -
                           (render_origin + (f.sequence-render_first)*render_period))
                       > self.max_jitter_s for f in rframes)
                or any(abs(f.acquired_at_robot -
                           (cap_origin + (f.sequence-cap_first)*cap_period))
                       > self.max_jitter_s for f in cframes)):
            self._healthy = False
            return None
        if (abs(render_period / .02 - 1) * 1e6 > self.max_ppm
                or abs(cap_period / .02 - 1) * 1e6 > self.max_ppm):
            # A short fit can mistake bounded callback jitter for clock drift.
            # Withhold residuals until the fit settles; latch a sustained fault.
            if len(cframes) >= 32 and len(rframes) >= 32:
                self._healthy = False
            return None
        render_map = {f.sequence: f for f in self._renders}
        values = {seq: struct.unpack('<320h', f.pcm_s16le) for seq, f in render_map.items()}
        mic = struct.unpack('<320h', capture.pcm_s16le)
        out = []
        cap_start = cap_origin + (capture.sequence - cap_first) * cap_period
        for index, sample in enumerate(mic):
            time = cap_start + index * cap_period / 320 - self.delay_s
            position = (time - render_origin) / render_period * 320 + render_first * 320
            if render_first * 320 - 1e-6 <= position < render_first * 320:
                position = render_first * 320
            seq = int(position // 320)
            offset = position - seq * 320
            # Permit minute floating-point boundary errors, never unknown samples.
            if seq not in values or offset < -1e-5 or offset > 319 + 1e-5:
                if seq not in values or not (319 < offset < 320 and seq + 1 in values):
                    self._healthy = False  # Reference underrun; require explicit route reacquisition.
                    return None
            if offset < 0:
                offset = 0.
            low = int(offset)
            alpha = offset - low
            if alpha < 1e-9:
                alpha = 0.  # Floating boundary is not a missing next frame.
            if low >= 320:
                return None
            first = values[seq][low]
            if low == 319 and alpha > 0:
                if seq + 1 not in values:
                    self._healthy = False
                    return None
                second = values[seq + 1][0]
            else:
                second = values[seq][min(low + 1, 319)]
            echo = first + alpha * (second - first)
            out.append(max(-32768, min(32767, round(sample - self.echo_gain * echo))))
        return struct.pack('<320h', *out)
