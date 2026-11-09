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


def _period(frames, *, min_frames=12):
    """Least-squares frame period, smoothing zero-mean callback timestamp jitter."""
    if len(frames) < min_frames:
        return .02, frames[0].rendered_at_robot if frames else 0., frames[0].sequence if frames else 0
    first_seq = frames[0].sequence
    xs = [frame.sequence - first_seq for frame in frames]
    ys = [frame.rendered_at_robot for frame in frames]
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
                self._healthy = False  # Explicit reacquisition required.
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
        if (capture.timestamp_quality != 'measured_capture'
                or (previous_seq >= 0 and (capture.sequence != previous_seq + 1
                    or not .015 <= capture.acquired_at_robot - previous_stamp <= .025))):
            self.reset_route()
            return None
        self._captures.append(capture)
        if not self._healthy or len(self._renders) < 12 or len(self._captures) < 12:
            return None  # Period fit not yet stable enough to assert near-end audio.
        render_period, render_origin, render_first = _period(list(self._renders)[-64:])
        # The cap clock may drift independently. Both estimates must stay bounded.
        cframes = list(self._captures)
        # Same fields as render frames, without pretending acquisition was render.
        cap_x = [f.sequence - cframes[0].sequence for f in cframes]
        cap_y = [f.acquired_at_robot for f in cframes]
        mx, my = sum(cap_x) / len(cap_x), sum(cap_y) / len(cap_y)
        cap_period = sum((x-mx)*(y-my) for x, y in zip(cap_x, cap_y)) / sum((x-mx)**2 for x in cap_x)
        if (abs(render_period / .02 - 1) * 1e6 > self.max_ppm
                or abs(cap_period / .02 - 1) * 1e6 > self.max_ppm):
            self._healthy = False
            return None
        render_map = {f.sequence: f for f in self._renders}
        values = {seq: struct.unpack('<320h', f.pcm_s16le) for seq, f in render_map.items()}
        mic = struct.unpack('<320h', capture.pcm_s16le)
        out = []
        for index, sample in enumerate(mic):
            time = capture.acquired_at_robot + index * cap_period / 320 - self.delay_s
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
