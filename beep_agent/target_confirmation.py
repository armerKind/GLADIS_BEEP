"""Conservative, body-agnostic confirmation of a requested visual target.

This module never performs I/O or dispatches motion. A result is evidence for a
session-level goal transition, not permission to approach or move toward it.
"""
from __future__ import annotations

import math
import re
from typing import Any

_WORDS = re.compile(r"[a-z0-9]+")
_IGNORED = {"a", "an", "the", "my", "on", "in", "at", "with", "of", "to", "find", "look", "for"}


def _tokens(value: str) -> set[str]:
    return set(_WORDS.findall(value.lower())) - _IGNORED


class TargetConfirmation:
    """Require two compatible sightings from different camera poses/windows."""

    def __init__(self, target: str, *, min_translation_m: float = 0.20,
                 min_rotation_rad: float = 0.35, max_gap_s: float = 30.0) -> None:
        self.target = target.strip()
        self.tokens = _tokens(self.target)
        if not self.tokens:
            raise ValueError("target must contain a distinctive object description")
        self.min_translation_m = min_translation_m
        self.min_rotation_rad = min_rotation_rad
        self.max_gap_s = max_gap_s
        self._pending: dict[str, Any] | None = None
        self.confirmed: dict[str, Any] | None = None

    def observe(self, response: dict, packet: dict, *, track_id: str | None) -> dict | None:
        if self.confirmed is not None:
            return self.confirmed
        attention = response.get("attention") or {}
        objects = (response.get("scene") or {}).get("objects") or []
        uncertainty = response.get("uncertainty") or []
        frames = packet.get("frames") or []
        bridge = packet.get("bridge_status") or {}
        lease = bridge.get("motion_lease_id")
        if (not track_id or attention.get("target_type") != "object"
                or not lease or not frames or response.get("confidence", 0) < 0.8
                or any(self.tokens & _tokens(str(item)) for item in uncertainty)):
            self._pending = None
            return None
        frame = frames[-1]
        localization = frame.get("localization") or {}
        pose = frame.get("pose") or {}
        if (any((part.get("motion") or {}).get("motion_lease_id") != lease for part in frames)
                or localization.get("pose_valid") is not True
                or any(not isinstance(pose.get(axis), (int, float))
                       or not math.isfinite(pose[axis]) for axis in ("x", "y", "yaw"))):
            self._pending = None
            return None
        object_id = attention.get("target_id")
        matched = [item for item in objects if item.get("id") == object_id
                   and item.get("confidence", 0) >= 0.85
                   and self.tokens <= _tokens(str(item.get("description", "")))]
        if len(matched) != 1:
            self._pending = None
            return None
        observed_at = frame.get("captured_at")
        frame_id = frame.get("frame_id")
        completed_at = (packet.get("inference") or {}).get("completed_at")
        if (not isinstance(observed_at, (int, float)) or not frame_id
                or not isinstance(completed_at, (int, float))
                or not 0 <= completed_at - observed_at <= self.max_gap_s):
            self._pending = None
            return None
        current = {"frame_id": frame_id, "captured_at": observed_at,
                   "track_id": track_id, "lease_id": lease,
                   "description": matched[0]["description"],
                   "confidence": matched[0]["confidence"],
                   "position": matched[0]["position"],
                   "pose": {axis: pose[axis] for axis in ("x", "y", "yaw")}}
        previous = self._pending
        if previous is not None:
            gap = observed_at - previous["captured_at"]
            if (previous["track_id"] == track_id and previous["lease_id"] == lease
                    and previous["frame_id"] != frame_id and 0 < gap <= self.max_gap_s):
                a, b = previous["pose"], current["pose"]
                translation = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
                rotation = abs(math.atan2(math.sin(b["yaw"] - a["yaw"]),
                                              math.cos(b["yaw"] - a["yaw"])))
                if translation >= self.min_translation_m or rotation >= self.min_rotation_rad:
                    self.confirmed = {"target": self.target, "track_id": track_id,
                                      "evidence": [previous, current],
                                      "viewpoint_change_m": round(translation, 3),
                                      "viewpoint_change_rad": round(rotation, 3)}
                    return self.confirmed
            elif gap <= 0:
                # A repeated or older camera window cannot become a new first sighting.
                return None
        self._pending = current
        return None
