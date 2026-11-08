#!/usr/bin/env python3
"""Start one map-guided onboard mission with parallel shadow camera perception.

LiDAR/SLAM and the bridge own gait and obstacle handling. Gemini is never in the
motor loop; the moving-eyes child observes continuously while inference runs.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

# Direct invocation from any working directory must find the sibling packages.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from beep_agent.body import BridgeBodyAdapter


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://192.168.8.88:8766")
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--panel", type=int, choices=(4, 6, 9), default=9)
    parser.add_argument("--fps", type=float, default=3.0)
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--inference-interval", type=float, default=2.0)
    parser.add_argument("--target", default="scene")
    parser.add_argument("--mock-response", type=Path, help="Offline/test perception only")
    parser.add_argument("--max-inferences", type=int, default=0)
    parser.add_argument("--output-root", type=Path, default=Path("captures/eyes/moving"))
    return parser.parse_args()


def run(args, *, adapter=None, runner=None):
    if not 5 <= args.duration <= 600:
        raise ValueError("duration must be within [5, 600]")
    adapter = adapter or BridgeBodyAdapter(args.base_url, args.timeout)
    runner = runner or subprocess.run
    preflight = adapter.mission()
    status = preflight["status"]
    existing = preflight.get("mission")
    if (existing and existing.get("state") in {"starting", "running"}) or status.get("moving") or status.get("motion_lease_id"):
        raise RuntimeError("another mission or motor lease owns BEEP")
    slam = status.get("slam") or {}
    age = status.get("scan_age_s")
    if not isinstance(age, (int, float)) or age < 0 or age > 0.6 or slam.get("usable") is not True:
        raise RuntimeError("map-guided mission needs fresh scan and usable SLAM")

    # The cleanup is armed before we parse the response: an accepted request
    # whose reply is malformed must not be left intentionally running.
    started = False
    mission_id = lease_id = None
    child_result = None
    outcome = {}
    try:
        response = adapter._json("/mission/start", {
            "mode": "coverage", "duration_s": args.duration,
            "min_duration": args.duration, "save": False,
        })
        started = response.get("accepted") is True
        if not started:
            return {"accepted": False, "reason": response.get("reason") or response.get("error")}
        mission = response.get("mission") or {}
        mission_id, lease_id = mission.get("id"), mission.get("lease_id")
        if not mission_id or not lease_id:
            raise RuntimeError("accepted mission lacks exact ID/lease; onboard watchdog retains deadline")
        command = [
            sys.executable, str(Path(__file__).with_name("run_moving_eyes.py")),
            "--base-url", args.base_url, "--duration", str(args.duration),
            "--panel", str(args.panel), "--fps", str(args.fps),
            "--timeout", str(args.timeout), "--inference-interval", str(args.inference_interval),
            "--target", args.target, "--output-root", str(args.output_root),
            "--mode", "shadow",
        ]
        if args.mock_response:
            command += ["--mock-response", str(args.mock_response)]
        if args.max_inferences:
            command += ["--max-inferences", str(args.max_inferences)]
        child_result = runner(command, capture_output=True, text=True, timeout=args.duration + args.timeout + 15)
        outcome.update({"accepted": True, "mission_id": mission_id, "lease_id": lease_id,
                        "eyes_exit_code": child_result.returncode,
                        "eyes_output": child_result.stdout[-3000:],
                        "eyes_error": child_result.stderr[-1000:]})
        return outcome
    finally:
        if started:
            try:
                current = adapter.mission()
                active = current.get("mission") or {}
                if (mission_id and lease_id and active.get("id") == mission_id
                        and active.get("lease_id") == lease_id
                        and active.get("state") in {"starting", "running"}):
                    adapter._json("/mission/cancel", {"mission_id": mission_id, "lease_id": lease_id})
                    # Cancellation is asynchronous; read back exact-owner release.
                    for _ in range(12):
                        time.sleep(0.25)
                        current = adapter.mission()
                        if current["status"].get("motion_lease_id") != lease_id:
                            break
                if current["status"].get("motion_lease_id") == lease_id:
                    raise RuntimeError("mission cleanup unverified: original lease still active")
                final_mission = current.get("mission") or {}
                outcome["final_mission_state"] = (final_mission.get("state")
                                                  if final_mission.get("id") == mission_id else "replaced")
                outcome["original_lease_released"] = True
            except Exception as exc:
                # Do not hide an earlier model error or invent a successful stop.
                print(json.dumps({"cleanup_unverified": str(exc), "mission_id": mission_id,
                                  "lease_id": lease_id}), file=sys.stderr, flush=True)
                raise


def main():
    result = run(parse_args())
    print(json.dumps(result, sort_keys=True))
    return 0 if result.get("accepted") and result.get("eyes_exit_code") == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
