"""Continuous movement-frame ring buffer for BEEP temporal vision.

Capture runs independently of semantic inference. A caller may build a chronological
4, 6, or 9-frame panel while new frames continue entering the ring. Individual
blurred frames are retained as uncertainty rather than turning perception into a
reason to stop locomotion.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Any

from .contact_sheet import (CapturedFrame, build_navigation_contact_sheet, fetch_jpeg, fetch_json,
                            frame_quality, summarize_navigation_context)

PANEL_COUNTS = frozenset({4, 6, 9})


@dataclass(frozen=True)
class MovingFrameSample:
    frame: CapturedFrame
    bridge_status: dict[str, Any]

    def metadata(self, previous: CapturedFrame | None = None) -> dict[str, Any]:
        status = self.bridge_status
        return {
            **self.frame.metadata(),
            "quality": frame_quality(self.frame, previous),
            "motion": {
                "moving": bool(status.get("moving")),
                "motion_lease_id": status.get("motion_lease_id"),
                "last_command": status.get("last_command"),
                "scan_age_s": status.get("scan_age_s"),
            },
            "pose": status.get("pose"),
            "localization": {
                "pose_valid": (status.get("slam") or {}).get("pose_valid"),
                "usable": (status.get("slam") or {}).get("usable"),
                "pose_age_s": (status.get("slam") or {}).get("pose_age_s"),
                "map_age_s": (status.get("slam") or {}).get("map_age_s"),
                "pose_guard_reason": (status.get("slam") or {}).get("pose_guard_reason"),
            },
        }


class MovingFrameRing:
    """Thread-safe bounded chronological evidence buffer."""

    def __init__(self, max_frames: int = 18) -> None:
        if max_frames < 9 or max_frames > 90:
            raise ValueError("max_frames must be within [9, 90]")
        self._frames: deque[MovingFrameSample] = deque(maxlen=max_frames)
        self._condition = threading.Condition()

    def add(self, sample: MovingFrameSample) -> None:
        with self._condition:
            if self._frames and sample.frame.captured_at <= self._frames[-1].frame.captured_at:
                raise ValueError("moving frame timestamps must be strictly chronological")
            self._frames.append(sample)
            self._condition.notify_all()

    def latest(self, count: int = 9) -> list[MovingFrameSample]:
        if count not in PANEL_COUNTS:
            raise ValueError("panel count must be 4, 6, or 9")
        with self._condition:
            if len(self._frames) < count:
                raise RuntimeError(f"moving frame ring has {len(self._frames)} frames; {count} required")
            return list(self._frames)[-count:]

    def wait_for(self, count: int = 9, timeout_s: float = 10.0) -> bool:
        if count not in PANEL_COUNTS:
            raise ValueError("panel count must be 4, 6, or 9")
        deadline = time.monotonic() + float(timeout_s)
        with self._condition:
            while len(self._frames) < count:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._condition.wait(remaining)
            return True

    def __len__(self) -> int:
        with self._condition:
            return len(self._frames)


class ContinuousMovingCapture:
    """Background bridge-camera capture that never acquires or alters motion."""

    def __init__(self, base_url: str, *, fps: float = 3.0, max_frames: int = 18,
                 timeout_s: float = 4.0) -> None:
        if fps < 0.5 or fps > 6.0:
            raise ValueError("fps must be within [0.5, 6.0]")
        self.base_url = base_url.rstrip("/")
        self.interval_s = 1.0 / float(fps)
        self.timeout_s = float(timeout_s)
        self.ring = MovingFrameRing(max_frames)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._sequence = 0
        self._captured = 0
        self._errors = 0
        self._last_error: str | None = None

    def start(self) -> "ContinuousMovingCapture":
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("moving capture is already running")
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="beep-moving-eyes", daemon=True)
            self._thread.start()
        return self

    def stop(self, timeout_s: float | None = None) -> None:
        self._stop.set()
        thread = self._thread
        # One capture may be inside three consecutive bounded HTTP requests.
        join_timeout = max(5.0, 3 * self.timeout_s + 1.0) if timeout_s is None else float(timeout_s)
        if thread is not None:
            thread.join(join_timeout)
        if thread is not None and thread.is_alive():
            raise RuntimeError("moving capture thread did not stop")

    def _run(self) -> None:
        next_capture = time.monotonic()
        while not self._stop.is_set():
            try:
                try:
                    before = fetch_json(self.base_url, "/status", self.timeout_s)
                except Exception:
                    before = {}
                captured_at = time.time()
                # Conservative request-start timestamp, never response completion.
                jpeg = fetch_jpeg(self.base_url, self.timeout_s)
                try:
                    status = fetch_json(self.base_url, "/status", self.timeout_s)
                except Exception as exc:
                    status = {"status_error": f"{type(exc).__name__}: {exc}"}
                # A post-capture lease alone could belong to a replacement owner.
                # A missing or changing owner makes this frame advisory only.
                before_lease = before.get("motion_lease_id") if isinstance(before, dict) else None
                after_lease = status.get("motion_lease_id")
                if not before_lease or before_lease != after_lease:
                    status = dict(status, motion_lease_id=None, capture_ownership_unverified=True)
                with self._lock:
                    frame_id = f"m{self._sequence:06d}"
                    self._sequence += 1
                self.ring.add(MovingFrameSample(CapturedFrame(frame_id, captured_at, jpeg), status))
                with self._lock:
                    self._captured += 1
                    self._last_error = None
            except Exception as exc:
                with self._lock:
                    self._errors += 1
                    self._last_error = f"{type(exc).__name__}: {exc}"
            next_capture += self.interval_s
            wait_s = max(0.0, next_capture - time.monotonic())
            if wait_s == 0.0:
                next_capture = time.monotonic()
            self._stop.wait(wait_s)

    def stats(self) -> dict[str, Any]:
        with self._lock:
            return {
                "captured": self._captured,
                "errors": self._errors,
                "last_error": self._last_error,
                "buffered": len(self.ring),
            }

    def temporal_window(self, count: int = 9, *, slam_map: dict[str, Any] | None = None) -> tuple[list[CapturedFrame], bytes, dict[str, Any]]:
        samples = self.ring.latest(count)
        frames = [sample.frame for sample in samples]
        metadata = []
        for index, sample in enumerate(samples):
            metadata.append(sample.metadata(frames[index - 1] if index else None))
        latest_status = samples[-1].bridge_status
        frame_statuses = [sample.bridge_status for sample in samples]
        packet = {
            "schema_version": "1.1",
            "capture_mode": "moving_temporal_ring",
            "captured_at": frames[-1].captured_at,
            "panel_count": count,
            "frame_span_s": round(frames[-1].captured_at - frames[0].captured_at, 3),
            "frames": metadata,
            "bridge_status": latest_status,
            "navigation_context": summarize_navigation_context(latest_status, slam_map),
            "target_search": {
                "target": "green baseball cap",
                "visual_cues": ["broad crown", "projecting curved brim", "fabric seams or panels"],
                "false_positives": ["green robot component", "green patch", "green cloth", "green furniture"],
                "confirmation_rule": "distinct from robot, matching cap structure across changed views",
            },
            "capture_stats": self.stats(),
            "evidence_policy": {
                "chronological": True,
                "motion_blur_tolerated": True,
                "missing_single_frame_is_not_a_stop_reason": True,
                "map_pose_heading_are_visualized_in_contact_sheet": True,
            },
        }
        columns = 2 if count == 4 else 3
        panel = build_navigation_contact_sheet(
            frames, frame_statuses=frame_statuses, current_status=latest_status,
            slam_map=slam_map, columns=columns)
        return frames, panel, packet
