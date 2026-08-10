#!/usr/bin/env python3
"""Probe whether BEEP's current SLAM stack can be preserved for preparation.

This is deliberately narrower than the final fair-readiness gate. It decides only
whether restarting LiDAR/Cartographer is necessary; clearance and stationary
stability are still validated by ``prepare_fair_run.sh`` afterward.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from typing import Any
import urllib.request


def _fresh(value: Any, maximum_s: float) -> bool:
    try:
        age = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(age) and 0.0 <= age <= maximum_s


def stopped_and_unleased(status: dict[str, Any]) -> bool:
    """Return whether bridge state proves no motion owner remains."""
    return status.get("moving") is False and status.get("motion_lease_id") is None


def preparation_status_ready(status: dict[str, Any]) -> bool:
    """Return whether the active sensor/SLAM stack should be preserved."""
    slam = status.get("slam") or {}
    return all(
        (
            stopped_and_unleased(status),
            _fresh(status.get("scan_age_s"), 0.5),
            slam.get("usable") is True,
            _fresh(slam.get("map_age_s"), 2.5),
            _fresh(slam.get("pose_age_s"), 0.5),
        )
    )


def fetch_status(base_url: str, timeout_s: float) -> dict[str, Any]:
    with urllib.request.urlopen(base_url.rstrip("/") + "/status", timeout=timeout_s) as response:
        payload = json.load(response)
    if not isinstance(payload, dict):
        raise ValueError("bridge status must be a JSON object")
    return payload


def probe_bridge_ready(
    base_url: str,
    timeout_s: float = 3.0,
    sample_count: int = 3,
    interval_s: float = 0.15,
) -> tuple[bool, dict[str, Any] | None, str | None]:
    """Require several consecutive healthy samples before preserving SLAM."""
    latest = None
    try:
        for index in range(max(1, sample_count)):
            latest = fetch_status(base_url, timeout_s)
            if not preparation_status_ready(latest):
                return False, latest, "bridge_or_slam_not_ready"
            if index + 1 < sample_count:
                time.sleep(max(0.0, interval_s))
    except Exception as exc:
        return False, latest, f"status_unavailable:{type(exc).__name__}"
    return True, latest, None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bridge-url", default="http://192.168.8.88:8766")
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--interval", type=float, default=0.15)
    parser.add_argument("--status-json", help="evaluate one supplied status object; intended for tests/replay")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.status_json is not None:
        status = json.loads(args.status_json)
        ready = isinstance(status, dict) and preparation_status_ready(status)
        reason = None if ready else "bridge_or_slam_not_ready"
    else:
        ready, status, reason = probe_bridge_ready(
            args.bridge_url,
            timeout_s=max(0.1, args.timeout),
            sample_count=max(1, args.samples),
            interval_s=max(0.0, args.interval),
        )
    slam = (status or {}).get("slam") or {}
    print(
        json.dumps(
            {
                "ready": ready,
                "reason": reason,
                "moving": (status or {}).get("moving"),
                "lease": (status or {}).get("motion_lease_id"),
                "scan_age_s": (status or {}).get("scan_age_s"),
                "map_age_s": slam.get("map_age_s"),
                "pose_age_s": slam.get("pose_age_s"),
            },
            sort_keys=True,
        )
    )
    return 0 if ready else 1


if __name__ == "__main__":
    raise SystemExit(main())
