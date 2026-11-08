"""Conservative bridge-owned revision of optimized submap placements.

A Cartographer SubmapList is evidence of placements, not an explicit optimizer
counter. The observer fails closed on gaps/disappearance and never treats a new
scan or a growing-submap version as a global correction.
"""
from __future__ import annotations

import math
import time
from typing import Mapping


class MapRevisionTracker:
    def __init__(self, *, position_epsilon_m: float = 0.02, yaw_epsilon_rad: float = 0.02,
                 max_age_s: float = 2.0, clock=time.monotonic):
        if position_epsilon_m <= 0 or yaw_epsilon_rad <= 0 or max_age_s <= 0:
            raise ValueError("revision thresholds must be positive")
        self.position_epsilon_m = position_epsilon_m
        self.yaw_epsilon_rad = yaw_epsilon_rad
        self.max_age_s = max_age_s
        self._clock = clock
        self._last_seen: float | None = None
        self.boot_id: str | None = None
        self.map_epoch: int | None = None
        self.revision: int | None = None
        self._poses: dict[str, tuple[float, float, float]] = {}
        self._unknown = True
        self._requires_new_epoch = False

    def status(self) -> dict:
        fresh = (self._last_seen is not None and
                 0 <= self._clock() - self._last_seen <= self.max_age_s)
        return {"boot_id": self.boot_id, "map_epoch": self.map_epoch,
                "map_revision": self.revision if not self._unknown and fresh else None,
                "status": "unknown" if self._unknown or not fresh else "verified"}

    def observe(self, submaps: Mapping[str, tuple[float, float, float, int]], *,
                boot_id: str, map_epoch: int, healthy: bool = False) -> tuple[int | None, str]:
        if not boot_id or isinstance(map_epoch, bool) or not isinstance(map_epoch, int) or map_epoch < 0:
            raise ValueError("invalid map identity")
        if (self.boot_id, self.map_epoch) != (boot_id, map_epoch):
            self.boot_id, self.map_epoch = boot_id, map_epoch
            self.revision, self._poses, self._unknown = None, {}, True
            self._requires_new_epoch = False
            self._last_seen = None
        if not healthy or not submaps:
            self._unknown = True
            self._last_seen = None
            return None, "unknown"
        poses = {}
        for key, value in submaps.items():
            if (not isinstance(key, str) or not key or len(value) != 4
                    or any(isinstance(n, bool) or not isinstance(n, (int, float))
                           or not math.isfinite(n) for n in value)
                    or isinstance(value[3], bool) or not isinstance(value[3], int)
                    or value[3] < 0):
                self._unknown = True
                self._last_seen = None
                return None, "unknown"
            poses[key] = (float(value[0]), float(value[1]), float(value[2]))
        if self._requires_new_epoch:
            return None, "unknown"
        if self.revision is None:
            self.revision, self._poses, self._unknown = 0, poses, False
            self._last_seen = self._clock()
            return 0, "verified"
        if not self._poses.keys() <= poses.keys():
            self._requires_new_epoch = True
            self._unknown = True
            self._last_seen = None
            return None, "unknown"
        if (self._unknown or self._last_seen is None or
                not 0 <= self._clock() - self._last_seen <= self.max_age_s):
            # A gap may hide an optimization; never reuse pre-gap metric state.
            self.revision += 1
            self._poses = poses
            self._unknown = False
            self._last_seen = self._clock()
            return self.revision, "verified"
        corrected = any(
            math.hypot(poses[key][0] - xyz[0], poses[key][1] - xyz[1]) >= self.position_epsilon_m
            or abs(math.atan2(math.sin(poses[key][2] - xyz[2]),
                              math.cos(poses[key][2] - xyz[2]))) >= self.yaw_epsilon_rad
            for key, xyz in self._poses.items()
        )
        if corrected:
            self.revision += 1
            self._poses = poses
        else:
            # New submaps enter the baseline, but a growing submap version alone
            # never increments the optimization revision.
            for key, pose in poses.items():
                self._poses.setdefault(key, pose)
        self._last_seen = self._clock()
        return self.revision, "verified"
