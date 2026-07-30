#!/usr/bin/env python3
"""Run BEEP's bounded SLAM presentation routine.

The coordinator intentionally lives off-robot. Each navigation request is a short
lease enforced by the bridge, and every transition is stopped and revalidated.
Camera and microphone data are never requested.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from typing import Any

DEFAULT_BASE_URL = "http://192.168.8.88:8766"
HARD_MIN_CLEARANCE_M = 0.15
TARGET_MARK_CLEARANCE_M = 0.20


class PresentationAbort(RuntimeError):
    pass


@dataclass
class BridgeClient:
    base_url: str

    def get(self, path: str, params: dict[str, Any] | None = None, timeout: float = 5.0) -> dict[str, Any]:
        query = urllib.parse.urlencode(params or {})
        url = self.base_url.rstrip("/") + path + (("?" + query) if query else "")
        request = urllib.request.Request(url, headers={"User-Agent": "beep-slam-presentation/1.0"})
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                return json.load(response)
        except (OSError, urllib.error.URLError, json.JSONDecodeError) as exc:
            raise PresentationAbort(f"bridge request failed for {path}: {exc}") from exc

    def status(self) -> dict[str, Any]:
        return self.get("/status")

    def stop(self) -> dict[str, Any]:
        return self.get("/stop", timeout=5.0)

    def coverage_segment(self, duration_s: float) -> dict[str, Any]:
        duration_s = max(5.0, min(float(duration_s), 60.0))
        return self.get(
            "/coverage_explore",
            {
                "max_duration": f"{duration_s:.2f}",
                "min_duration": f"{duration_s:.2f}",
                "coverage_window": f"{max(10.0, duration_s):.2f}",
                "min_growth_cells": "1",
                "save": "0",
            },
            timeout=duration_s + 12.0,
        )

    def trick(self, name: str) -> dict[str, Any]:
        return self.get("/action", {"name": name, "wait": "1"}, timeout=12.0)

    def mark_corner(self) -> dict[str, Any]:
        return self.get(
            "/mark_object",
            {
                "target_front": f"{TARGET_MARK_CLEARANCE_M:.2f}",
                "max_duration": "5.0",
                "turn": "left",
            },
            timeout=30.0,
        )


def emit(event: str, **payload: Any) -> None:
    print(json.dumps({"time": round(time.time(), 3), "event": event, **payload}, sort_keys=True), flush=True)


def validate_stationary_state(status: dict[str, Any], hard_min_m: float = HARD_MIN_CLEARANCE_M) -> None:
    if status.get("moving"):
        raise PresentationAbort("bridge reports moving during stationary validation")
    if not status.get("scan_seen") or status.get("scan_age_s") is None or float(status["scan_age_s"]) > 0.60:
        raise PresentationAbort("LiDAR is missing or stale")

    slam = status.get("slam") or {}
    if not slam.get("active"):
        raise PresentationAbort("SLAM is inactive")
    if slam.get("usable") is False:
        raise PresentationAbort(f"SLAM is unusable: {slam.get('usable_reason') or 'unknown reason'}")
    if slam.get("pose_valid") is not True:
        raise PresentationAbort("guarded SLAM pose is invalid")
    if slam.get("pose_age_s") is None or float(slam["pose_age_s"]) > 1.0:
        raise PresentationAbort("SLAM pose is stale")
    if slam.get("map_age_s") is None or float(slam["map_age_s"]) > 2.5:
        raise PresentationAbort("SLAM map is stale")

    sectors = status.get("sectors") or {}
    for name in ("front", "front_left", "front_right", "left", "right"):
        value = sectors.get(name)
        if value is None:
            raise PresentationAbort(f"LiDAR sector {name} is missing")
        if float(value) < float(hard_min_m):
            raise PresentationAbort(f"hard clearance violated in {name}: {float(value):.3f}m")


def corner_mark_candidate(status: dict[str, Any]) -> dict[str, Any] | None:
    """Recognize a corner where a left turn leaves the front wall by the right leg.

    The right side must already contain the adjacent corner wall, while the left
    side must have enough room for an in-place left turn. This deliberately skips
    mirror-image corners rather than improvising a risky 270-degree maneuver.
    """
    sectors = status.get("sectors") or {}
    try:
        front = float(sectors["front"])
        right = float(sectors["right"])
        left = float(sectors["left"])
        front_left = float(sectors["front_left"])
        front_right = float(sectors["front_right"])
    except (KeyError, TypeError, ValueError):
        return None

    if not (TARGET_MARK_CLEARANCE_M <= front <= 0.70):
        return None
    if not (HARD_MIN_CLEARANCE_M <= right <= 0.45):
        return None
    if left < 0.45 or front_left < 0.32 or front_right < HARD_MIN_CLEARANCE_M:
        return None
    return {
        "turn": "left",
        "target_front_m": TARGET_MARK_CLEARANCE_M,
        "front_m": round(front, 3),
        "right_wall_m": round(right, 3),
        "left_clearance_m": round(left, 3),
    }


def stopped_gesture(client: BridgeClient, name: str) -> dict[str, Any]:
    status = client.stop().get("status") or client.status()
    validate_stationary_state(status)
    result = client.trick(name)
    if not result.get("ok"):
        raise PresentationAbort(f"gesture {name} failed: {result.get('reason') or result.get('error')}")
    reset = client.trick("reset")
    if not reset.get("ok"):
        raise PresentationAbort(f"neutral reset after {name} failed")
    status = client.stop().get("status") or client.status()
    validate_stationary_state(status)
    return {"gesture": result, "reset": reset, "status": status}


def run_presentation(
    client: BridgeClient,
    duration_s: float,
    segment_s: float,
    prey_interval_s: float,
    pee_interval_s: float,
    enable_prey: bool = True,
    enable_pee: bool = True,
) -> dict[str, Any]:
    duration_s = max(30.0, min(float(duration_s), 600.0))
    segment_s = max(10.0, min(float(segment_s), 60.0))
    started = time.monotonic()
    deadline = started + duration_s
    next_prey = started + max(30.0, float(prey_interval_s))
    next_pee = started + max(45.0, float(pee_interval_s))
    segments = gestures = marks = 0
    reason = "duration_complete"

    initial = client.stop().get("status") or client.status()
    validate_stationary_state(initial)
    emit("presentation_started", duration_s=duration_s, segment_s=segment_s)

    try:
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining < 5.0:
                break
            lease_s = min(segment_s, remaining)
            emit("navigation_lease_started", lease_s=round(lease_s, 2), segment=segments + 1)
            result = client.coverage_segment(lease_s)
            segments += 1
            emit(
                "navigation_lease_finished",
                segment=segments,
                ok=bool(result.get("ok")),
                reason=result.get("reason"),
                elapsed_s=result.get("elapsed_s"),
            )
            if result.get("reason") == "motion_cancelled" or (result.get("status") or {}).get("motion_cancelled"):
                reason = "motion_cancelled"
                raise PresentationAbort("manual stop latched during navigation")
            if not result.get("ok"):
                reason = "navigation_failed"
                raise PresentationAbort(f"navigation lease failed: {result.get('reason')}")

            stationary = client.stop().get("status") or client.status()
            validate_stationary_state(stationary)
            now = time.monotonic()

            candidate = corner_mark_candidate(stationary) if enable_pee and now >= next_pee else None
            if candidate is not None:
                emit("corner_mark_started", geometry=candidate)
                marked = client.mark_corner()
                if not marked.get("ok"):
                    raise PresentationAbort(f"corner marking failed: {marked.get('reason')}")
                marks += 1
                next_pee = time.monotonic() + max(90.0, float(pee_interval_s))
                stationary = client.stop().get("status") or client.status()
                validate_stationary_state(stationary)
                emit("corner_mark_finished", marks=marks)
            elif enable_prey and now >= next_prey:
                emit("prey_started")
                stopped_gesture(client, "prey")
                gestures += 1
                next_prey = time.monotonic() + max(60.0, float(prey_interval_s))
                emit("prey_finished", gestures=gestures)
    except PresentationAbort:
        raise
    finally:
        try:
            client.stop()
        except Exception as stop_exc:  # final report must preserve stop transport failure
            emit("final_stop_failed", error=repr(stop_exc))

    final_status = client.status()
    validate_stationary_state(final_status)
    report = {
        "ok": True,
        "reason": reason,
        "elapsed_s": round(time.monotonic() - started, 2),
        "segments": segments,
        "prey_gestures": gestures,
        "corner_marks": marks,
        "final_status": final_status,
    }
    emit("presentation_finished", **{key: value for key, value in report.items() if key != "final_status"})
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default=DEFAULT_BASE_URL)
    parser.add_argument("--duration", type=float, default=600.0, help="total routine duration, clamped to 30..600 seconds")
    parser.add_argument("--segment", type=float, default=45.0, help="guarded navigation lease, clamped to 10..60 seconds")
    parser.add_argument("--prey-interval", type=float, default=150.0)
    parser.add_argument("--pee-interval", type=float, default=210.0)
    parser.add_argument("--no-prey", action="store_true")
    parser.add_argument("--enable-pee", action="store_true", help="opt in to experimental corner marking; disabled by default")
    parser.add_argument("--armed", action="store_true", help="required for physical execution")
    parser.add_argument("--dry-run", action="store_true", help="validate current state and print the plan without movement")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    client = BridgeClient(args.base_url)
    stop_requested = {"value": False}

    def on_signal(signum: int, _frame: Any) -> None:
        stop_requested["value"] = True
        emit("signal_received", signal=signum)
        try:
            client.stop()
        finally:
            raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        status = client.stop().get("status") or client.status()
        validate_stationary_state(status)
        if args.dry_run:
            plan = {
                "ok": True,
                "dry_run": True,
                "duration_s": max(30.0, min(args.duration, 600.0)),
                "segment_s": max(10.0, min(args.segment, 60.0)),
                "prey": not args.no_prey,
                "pee": args.enable_pee,
                "hard_min_clearance_m": HARD_MIN_CLEARANCE_M,
                "mark_target_clearance_m": TARGET_MARK_CLEARANCE_M,
                "corner_candidate_now": corner_mark_candidate(status),
                "camera_used": False,
                "microphone_used": False,
            }
            print(json.dumps(plan, indent=2, sort_keys=True))
            return 0
        if not args.armed:
            raise PresentationAbort("physical execution requires --armed")
        report = run_presentation(
            client,
            duration_s=args.duration,
            segment_s=args.segment,
            prey_interval_s=args.prey_interval,
            pee_interval_s=args.pee_interval,
            enable_prey=not args.no_prey,
            enable_pee=args.enable_pee,
        )
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    except KeyboardInterrupt:
        emit("presentation_aborted", reason="operator_interrupt")
        return 130
    except PresentationAbort as exc:
        emit("presentation_aborted", reason=str(exc), signal_stop=stop_requested["value"])
        try:
            client.stop()
        except Exception as stop_exc:
            emit("abort_stop_failed", error=repr(stop_exc))
        return 2


if __name__ == "__main__":
    sys.exit(main())
