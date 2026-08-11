#!/usr/bin/env python3
"""
BEEP bridge v0.3: fast HTTP nervous system for Yahboom DogZilla S2.

Goals:
- No browser/Jupyter in the control loop.
- Low-latency compact status.
- App-port 6000 movement wrapper with bounded commands and stop bursts.
- ROS2 /scan LiDAR sectors.
- Camera frame capture from MJPEG port 6500.
- Adaptive forward approach with stall detection and optional small reorientation.
"""
from __future__ import annotations

import hmac
import json
import math
import os
import random
import socket
import sys
import threading
import time
import urllib.request
from pathlib import Path
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, str(Path(__file__).resolve().parent))
from frontier_planner import (  # noqa: E402
    OccupancyGrid as FrontierGrid,
    astar_path,
    choose_natural_motion,
    find_frontier_plan,
    inflate_obstacles,
    nearest_free_cell,
)

HOST = os.environ.get("BEEP_APP_HOST", "192.168.8.88")
APP_PORT = int(os.environ.get("BEEP_APP_PORT", "6000"))
CAMERA_URL = os.environ.get("BEEP_CAMERA_URL", f"http://{HOST}:6500/video_feed")
HTTP_BIND = os.environ.get("BEEP_BRIDGE_BIND", "0.0.0.0")
HTTP_PORT = int(os.environ.get("BEEP_BRIDGE_PORT", "8766"))
MAX_MOVE_S = float(os.environ.get("BEEP_MAX_MOVE_S", "5.0"))
FORWARD_UNTIL_MAX_S = float(os.environ.get("BEEP_FORWARD_UNTIL_MAX_S", "12.0"))
FRONT_STOP_M = float(os.environ.get("BEEP_FRONT_STOP_M", "0.35"))
SIDE_STOP_M = float(os.environ.get("BEEP_SIDE_STOP_M", "0.22"))
SCAN_STALE_S = float(os.environ.get("BEEP_SCAN_STALE_S", "0.60"))
HARD_CLEARANCE_M = max(0.18, float(os.environ.get("BEEP_HARD_CLEARANCE_M", "0.18")))
FOOTPRINT_FRONT_M = max(0.55, float(os.environ.get("BEEP_FOOTPRINT_FRONT_M", "0.55")))
FOOTPRINT_DIAGONAL_M = max(0.42, float(os.environ.get("BEEP_FOOTPRINT_DIAGONAL_M", "0.42")))
FOOTPRINT_SIDE_M = max(0.35, float(os.environ.get("BEEP_FOOTPRINT_SIDE_M", "0.35")))
FOOTPRINT_REAR_M = max(0.45, float(os.environ.get("BEEP_FOOTPRINT_REAR_M", "0.45")))
FORWARD_CORRIDOR_M = max(0.65, float(os.environ.get("BEEP_FORWARD_CORRIDOR_M", "0.65")))
TURN_SWEEP_M = max(0.42, float(os.environ.get("BEEP_TURN_SWEEP_M", "0.42")))
TURN_START_CLEARANCE_M = max(TURN_SWEEP_M + 0.03,
                             float(os.environ.get("BEEP_TURN_START_CLEARANCE_M", "0.45")))
ESCAPE_CLEARANCE_GAIN_M = max(0.08, float(os.environ.get("BEEP_ESCAPE_CLEARANCE_GAIN_M", "0.08")))
BACKOFF_MAX_FRONT_LOSS_M = min(0.02, max(0.005, float(os.environ.get("BEEP_BACKOFF_MAX_FRONT_LOSS_M", "0.02"))))
BACKOFF_MAX_HEADING_DRIFT_RAD = math.radians(
    min(5.0, max(1.0, float(os.environ.get("BEEP_BACKOFF_MAX_HEADING_DRIFT_DEG", "5.0"))))
)
REVERSE_ESCAPE_MAX_S = min(1.2, max(
    0.30, float(os.environ.get("BEEP_REVERSE_ESCAPE_MAX_S", "1.2"))))
REVERSE_CORRECTION_SEGMENT_S = min(0.60, max(
    0.30, float(os.environ.get("BEEP_REVERSE_CORRECTION_SEGMENT_S", "0.60"))))
REVERSE_CORRECTION_ATTEMPTS = min(3, max(
    1, int(os.environ.get("BEEP_REVERSE_CORRECTION_ATTEMPTS", "3"))))
REVERSE_CORRECTION_STEP = min(10, max(
    5, int(os.environ.get("BEEP_REVERSE_CORRECTION_STEP", "8"))))
ROBOT_FOOTPRINT_RADIUS_M = max(0.35, float(os.environ.get("BEEP_ROBOT_FOOTPRINT_RADIUS_M", "0.35")))
TURN_PROGRESS_WINDOW_S = max(0.50, float(os.environ.get("BEEP_TURN_PROGRESS_WINDOW_S", "0.75")))
TURN_PROGRESS_MIN_RAD = math.radians(max(5.0, float(os.environ.get("BEEP_TURN_PROGRESS_MIN_DEG", "10.0"))))
OBSERVER_STOP_TOKEN = os.environ.get("BEEP_OBSERVER_STOP_TOKEN", "")
OBSERVER_STOP_MIN_INTERVAL_S = max(0.5, float(os.environ.get("BEEP_OBSERVER_STOP_MIN_INTERVAL_S", "2.0")))

CMD_PAYLOAD = {"stop": 0x00, "turnleft": 0x05, "turnright": 0x06}
APP_ANALOG_PAYLOAD = {
    "forward": (0x00, 0x64),
    "back": (0x00, 0x9C),
    "backward": (0x00, 0x9C),
    "left": (0x64, 0x00),
    "right": (0x9C, 0x00),
}
SDK_STEP_DEFAULT = int(os.environ.get("BEEP_SDK_STEP", "10"))
SDK_GAIT = os.environ.get("BEEP_SDK_GAIT", "walk")
SDK_PACE = os.environ.get("BEEP_SDK_PACE", "normal")
MOTOR_BACKEND = os.environ.get("BEEP_MOTOR_BACKEND", "sdk")  # sdk or app
MAP_DIR = Path(os.environ.get("BEEP_MAP_DIR", "/home/pi/beep_bridge/maps"))
MAP_RES_M = float(os.environ.get("BEEP_MAP_RES_M", "0.05"))
MAP_SIZE_M = float(os.environ.get("BEEP_MAP_SIZE_M", "8.0"))
EXPLORE_SAFE_FRONT_M = float(os.environ.get("BEEP_EXPLORE_SAFE_FRONT_M", "0.45"))
EXPLORE_SAFE_SIDE_M = float(os.environ.get("BEEP_EXPLORE_SAFE_SIDE_M", "0.25"))
# Crude dead-reckoning constants. Replace with ROS/Cartographer pose when available.
SDK_FORWARD_M_PER_S = float(os.environ.get("BEEP_SDK_FORWARD_M_PER_S", "0.045"))
SDK_STRAFE_M_PER_S = float(os.environ.get("BEEP_SDK_STRAFE_M_PER_S", "0.035"))
SDK_TURN_RAD_PER_S = float(os.environ.get("BEEP_SDK_TURN_RAD_PER_S", "0.45"))
TRICK_SETTLE_S = float(os.environ.get("BEEP_TRICK_SETTLE_S", "2.0"))
MARK_TURN_DIRECTION = os.environ.get("BEEP_MARK_TURN_DIRECTION", "left").strip().lower()
MARK_TURN_DEGREES = float(os.environ.get("BEEP_MARK_TURN_DEGREES", "90.0"))
MARK_TURN_TIMEOUT_S = float(os.environ.get("BEEP_MARK_TURN_TIMEOUT_S", "8.0"))
MARK_TARGET_FRONT_M = max(FOOTPRINT_FRONT_M, float(os.environ.get("BEEP_MARK_TARGET_FRONT_M", str(FOOTPRINT_FRONT_M))))
MARK_MIN_FRONT_M = max(FOOTPRINT_FRONT_M, float(os.environ.get("BEEP_MARK_MIN_FRONT_M", str(FOOTPRINT_FRONT_M))))
TRICK_ACTIONS = {
    "reset": {"id": 255, "label": "Reset / neutral pose", "duration_s": 0.5, "safe_for_fair": True, "aliases": ["neutral", "stand", "home"]},
    "crawl": {"id": 3, "label": "Crawl", "duration_s": 3.0, "safe_for_fair": False, "aliases": ["creep"]},
    "three_axis": {"id": 10, "label": "3-axis body motion", "duration_s": 3.0, "safe_for_fair": True, "aliases": ["3axis", "axis", "body_demo"]},
    "pee": {"id": 11, "label": "Lift leg / pee", "duration_s": 8.0, "safe_for_fair": True, "aliases": ["leg_lift", "mark", "urinate"]},
    "stretch": {"id": 14, "label": "Stretch", "duration_s": 3.0, "safe_for_fair": True, "aliases": ["show", "startup_show", "lazy"]},
    "swing": {"id": 16, "label": "Swing", "duration_s": 3.0, "safe_for_fair": True, "aliases": ["shake", "wobble"]},
    "pray": {"id": 17, "label": "Pray / beg", "duration_s": 3.0, "safe_for_fair": True, "aliases": ["prey", "beg", "begging", "request_food"]},
}
TRICK_ALIASES = {}
for _trick_name, _trick_meta in TRICK_ACTIONS.items():
    TRICK_ALIASES[_trick_name] = _trick_name
    TRICK_ALIASES[str(_trick_meta["id"])] = _trick_name
    for _alias in _trick_meta.get("aliases", []):
        TRICK_ALIASES[str(_alias).lower()] = _trick_name
sdk_dog = None
sdk_error = None

state_lock = threading.RLock()
motion_lock = threading.RLock()
motion_owner_lock = threading.RLock()
motion_context = threading.local()
_active_motion_lease = None
_motion_lease_sequence = 0
events = deque(maxlen=300)
observer_stop_lock = threading.Lock()
observer_last_stop_at = None
last_run = None
state = {
    "version": "0.19.0-app-analog-motion",
    "started_at": time.time(),
    "last_command": None,
    "last_command_at": None,
    "last_motion_at": None,
    "last_error": None,
    "scan_seen": False,
    "scan_at": None,
    "scan_count": 0,
    "sectors": {},
    "moving": False,
    "motion_cancelled": False,
    "motion_lease_id": None,
    "motion_lease_source": None,
    "last_frame_at": None,
    "last_frame_bytes": None,
    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0, "source": "dead_reckoning", "confidence": 0.25, "scan_match_score": None},
    "slam": {"active": False, "pose_at": None, "pose_age_s": None, "map_at": None, "map_age_s": None, "map_width": None, "map_height": None, "resolution_m": None},
    "map": {"active": False, "updated_at": None, "scan_updates": 0, "occupied_cells": 0, "free_cells": 0, "quality": "none"},
}
last_scan = None
slam_grid = None
room_map = None
active_explore = None
scan_match_ref = None
mission_lock = threading.RLock()
_active_mission = None
_latest_mission = None
_mission_sequence = 0


def remember(event, **data):
    item = {"t": round(time.time(), 3), "event": event, **data}
    with state_lock:
        events.append(item)
    return item


class MotionBusy(RuntimeError):
    pass


class MotionLease:
    def __init__(self, lease_id, source, deadline):
        self.lease_id = lease_id
        self.source = str(source)
        self.deadline = float(deadline)
        self.cancel_event = threading.Event()
        self.ended_event = threading.Event()
        self.started_at = time.monotonic()
        self.ended_at = None


def active_motion_lease():
    with motion_owner_lock:
        return _active_motion_lease


def motion_lease_watchdog(lease):
    """Enforce the exact lease deadline independently of its worker thread."""
    while not lease.ended_event.is_set() and not lease.cancel_event.is_set():
        remaining = lease.deadline - time.monotonic()
        if remaining > 0:
            lease.ended_event.wait(min(0.10, remaining))
            continue
        with motion_owner_lock:
            owns_motion = _active_motion_lease is lease
        if not owns_motion or lease.ended_event.is_set():
            return
        lease.cancel_event.set()
        with state_lock:
            state["motion_cancelled"] = True
        remember("motion_lease_watchdog_deadline", lease_id=lease.lease_id, source=lease.source)
        try:
            stop_burst(3)
        except Exception as exc:
            remember("motion_lease_watchdog_stop_failed", lease_id=lease.lease_id, error=repr(exc))
        return


def begin_motion(source="api", max_duration=600.0):
    """Acquire exclusive motion ownership immediately; never queue behind another run."""
    global _active_motion_lease, _motion_lease_sequence
    now = time.monotonic()
    max_duration = max(0.1, min(float(max_duration), 600.0))
    with motion_owner_lock:
        if _active_motion_lease is not None:
            current = _active_motion_lease
            raise MotionBusy(f"motion busy: lease {current.lease_id} from {current.source}")
        _motion_lease_sequence += 1
        lease = MotionLease(f"motion-{_motion_lease_sequence}", source, now + max_duration)
        _active_motion_lease = lease
        motion_context.lease = lease
    with state_lock:
        state["motion_cancelled"] = False
        state["motion_lease_id"] = lease.lease_id
        state["motion_lease_source"] = lease.source
    remember("motion_lease_started", source=lease.source, lease_id=lease.lease_id, max_duration_s=max_duration)
    threading.Thread(target=motion_lease_watchdog, args=(lease,), daemon=True,
                     name=f"beep-motion-watchdog-{lease.lease_id}").start()
    return lease


def end_motion(lease):
    """Release ownership only for the exact lease that acquired it."""
    global _active_motion_lease
    if lease is None:
        return
    lease.ended_at = time.monotonic()
    lease.ended_event.set()
    with motion_owner_lock:
        if _active_motion_lease is lease:
            _active_motion_lease = None
    if getattr(motion_context, "lease", None) is lease:
        del motion_context.lease
    with state_lock:
        if state.get("motion_lease_id") == lease.lease_id:
            state["motion_lease_id"] = None
            state["motion_lease_source"] = None
    remember("motion_lease_ended", source=lease.source, lease_id=lease.lease_id,
             cancelled=lease.cancel_event.is_set(), elapsed_s=round(lease.ended_at - lease.started_at, 3))


def run_owned_motion(source, callback, max_duration=600.0):
    lease = begin_motion(source, max_duration=max_duration)
    try:
        return callback()
    finally:
        end_motion(lease)


def motion_is_cancelled():
    lease = getattr(motion_context, "lease", None)
    if lease is None:
        return bool(state.get("motion_cancelled"))
    if time.monotonic() >= lease.deadline and not lease.cancel_event.is_set():
        lease.cancel_event.set()
        with state_lock:
            state["motion_cancelled"] = True
        remember("motion_lease_deadline", lease_id=lease.lease_id, source=lease.source)
    return lease.cancel_event.is_set()


def _reset_motion_state_for_tests():
    """Test isolation helper; never used by HTTP or physical control paths."""
    global _active_motion_lease
    with motion_owner_lock:
        if _active_motion_lease is not None:
            _active_motion_lease.cancel_event.set()
            _active_motion_lease.ended_event.set()
        _active_motion_lease = None
    if hasattr(motion_context, "lease"):
        del motion_context.lease
    with state_lock:
        state["motion_cancelled"] = False
        state["motion_lease_id"] = None
        state["motion_lease_source"] = None


def _reset_mission_state_for_tests():
    """Test isolation helper for completed mocked mission workers."""
    global _active_mission, _latest_mission, _mission_sequence
    with mission_lock:
        _active_mission = None
        _latest_mission = None
        _mission_sequence = 0


def wait_motion(duration, poll_s=0.05):
    """Wait for a bounded motion window, returning early after a stop latch."""
    deadline = time.monotonic() + max(0.0, float(duration))
    while time.monotonic() < deadline:
        if motion_is_cancelled():
            return False
        lease = getattr(motion_context, "lease", None)
        delay = min(float(poll_s), max(0.0, deadline - time.monotonic()))
        if lease is not None:
            lease.cancel_event.wait(delay)
        else:
            time.sleep(delay)
    return not motion_is_cancelled()


def pkt(cmd: int, payload=()):
    vals = [0x01, cmd, 2 * (len(payload) + 1), *payload]
    vals.append(sum(vals) % 256)
    return ("$" + "".join(f"{v & 255:02X}" for v in vals) + "#").encode()


def app_send(action: str):
    action = str(action).lower()
    aliases = {"turn_left": "turnleft", "turn_right": "turnright",
               "rotate_left": "turnleft", "rotate_right": "turnright"}
    action = aliases.get(action, action)
    if action not in CMD_PAYLOAD and action not in APP_ANALOG_PAYLOAD:
        raise ValueError(f"unknown app action {action!r}")
    with socket.create_connection((HOST, APP_PORT), timeout=1.2) as s:
        s.settimeout(0.25)
        try:
            s.recv(256)
        except Exception:
            pass
        # Standard/control mode, then command.
        s.sendall(pkt(0x0F, [0x01]))
        time.sleep(0.025)
        if action == "stop":
            s.sendall(pkt(0x11, [0x00, 0x00]))
            time.sleep(0.01)
            s.sendall(pkt(0x12, [CMD_PAYLOAD[action]]))
        elif action in APP_ANALOG_PAYLOAD:
            s.sendall(pkt(0x11, APP_ANALOG_PAYLOAD[action]))
        else:
            s.sendall(pkt(0x12, [CMD_PAYLOAD[action]]))
    with state_lock:
        state["last_command"] = "app:" + action
        state["last_command_at"] = time.time()
        state["moving"] = action != "stop"
    remember("app_send", action=action)


def sdk_apply_motion_profile(g=None):
    """Reapply configured gait and pace before newly directed motion.

    Vendor actions may overwrite persistent gait/pace registers. Reapplying
    the profile here makes post-gesture speed deterministic.
    """
    if SDK_GAIT not in ("trot", "walk", "high_walk"):
        raise ValueError(f"unsupported SDK gait {SDK_GAIT!r}")
    if SDK_PACE not in ("normal", "slow", "high"):
        raise ValueError(f"unsupported SDK pace {SDK_PACE!r}")
    g = sdk_init() if g is None else g
    g.gait_type(SDK_GAIT)
    g.pace(SDK_PACE)
    remember("sdk_motion_profile", gait=SDK_GAIT, pace=SDK_PACE)
    return g


def sdk_init():
    global sdk_dog, sdk_error
    if sdk_dog is not None:
        return sdk_dog
    try:
        import DOGZILLALib as dog
        sdk_dog = dog.DOGZILLA()
        sdk_apply_motion_profile(sdk_dog)
        sdk_error = None
        remember("sdk_init", gait=SDK_GAIT, pace=SDK_PACE)
        return sdk_dog
    except Exception as e:
        sdk_error = repr(e)
        with state_lock:
            state["last_error"] = "sdk init failed: " + sdk_error
        raise


def sdk_send(action: str, step=None):
    action = str(action).lower()
    step = SDK_STEP_DEFAULT if step is None else int(step)
    g = sdk_init()
    aliases = {"backward": "back", "turn_left": "turnleft", "turn_right": "turnright", "rotate_left": "turnleft", "rotate_right": "turnright"}
    action = aliases.get(action, action)
    if action == "stop":
        errors = []
        for method, value in ((g.stop, None), (g.move_x, 0), (g.move_y, 0), (g.turn, 0)):
            try:
                method() if value is None else method(value)
            except Exception as exc:
                errors.append(repr(exc))
        if errors:
            raise RuntimeError("SDK stop/axis neutralization failed: " + "; ".join(errors))
    elif action in ("forward", "back"):
        sdk_apply_motion_profile(g)
        g.move_y(0)
        g.turn(0)
        getattr(g, action)(step)
    elif action in ("left", "right"):
        sdk_apply_motion_profile(g)
        g.move_x(0)
        g.turn(0)
        getattr(g, action)(step)
    elif action in ("turnleft", "turnright"):
        sdk_apply_motion_profile(g)
        g.move_x(0)
        g.move_y(0)
        getattr(g, action)(step)
    else:
        raise ValueError(f"unknown sdk action {action!r}")
    with state_lock:
        was_moving = bool(state.get("moving"))
        now = time.time()
        state["last_command"] = "sdk:" + action
        state["last_command_at"] = now
        state["moving"] = action != "stop"
        if action != "stop" or was_moving:
            state["last_motion_at"] = now
    remember("sdk_send", action=action, step=step)


def sdk_curve(direction, forward_step=20, yaw_step=30):
    """Combine independent VX and VYAW registers for a fluent walking arc."""
    direction = str(direction).lower()
    if direction not in ("left", "right"):
        raise ValueError("curve direction must be left or right")
    g = sdk_init()
    sdk_apply_motion_profile(g)
    g.move_x(abs(int(forward_step)))
    signed_yaw = abs(int(yaw_step)) if direction == "left" else -abs(int(yaw_step))
    g.turn(signed_yaw)
    now = time.time()
    with state_lock:
        state["last_command"] = "sdk:curve_" + direction
        state["last_command_at"] = now
        state["last_motion_at"] = now
        state["moving"] = True
    remember("sdk_curve", direction=direction, forward_step=forward_step, yaw_step=signed_yaw)


def sdk_straighten():
    """Clear yaw velocity while preserving the active forward gait."""
    g = sdk_init()
    g.turn(0)
    now = time.time()
    with state_lock:
        state["last_command"] = "sdk:forward"
        state["last_command_at"] = now
        state["last_motion_at"] = now
        state["moving"] = True
    remember("sdk_straighten")


def resolve_trick(name=None, action_id=None):
    key = None
    if action_id is not None:
        key = str(int(action_id))
    elif name is not None:
        key = str(name).strip().lower().replace("-", "_").replace(" ", "_")
    if not key:
        raise ValueError("provide trick name or action id")
    mapped = TRICK_ALIASES.get(key)
    if mapped:
        meta = dict(TRICK_ACTIONS[mapped])
        meta["name"] = mapped
        return meta
    if key.isdigit():
        raw_id = int(key)
        if raw_id <= 0 or raw_id > 255:
            raise ValueError("raw action id must be in 1..255")
        return {"name": f"raw_{raw_id}", "id": raw_id, "label": f"Raw SDK action {raw_id}", "duration_s": TRICK_SETTLE_S, "safe_for_fair": False, "aliases": []}
    raise ValueError(f"unknown trick {key!r}; use /actions")


def truthy(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def tricks_payload():
    items = []
    for name, meta in sorted(TRICK_ACTIONS.items(), key=lambda kv: kv[1]["id"]):
        item = dict(meta)
        item["name"] = name
        items.append(item)
    return {"actions": items, "aliases": dict(sorted(TRICK_ALIASES.items())), "raw_action_ids_supported": True}


def sdk_trick(name=None, action_id=None, dry_run=False, settle_s=None):
    trick = resolve_trick(name=name, action_id=action_id)
    settle_s = trick.get("duration_s", TRICK_SETTLE_S) if settle_s is None else float(settle_s)
    settle_s = max(0.0, min(settle_s, 8.0))
    if dry_run:
        remember("sdk_trick_dry_run", trick=trick)
        return {"ok": True, "dry_run": True, "trick": trick}
    with motion_lock:
        if motion_is_cancelled():
            return {"ok": False, "dry_run": False, "reason": "motion_cancelled", "trick": trick}
        g = sdk_init()
        with state_lock:
            state["last_command"] = "sdk_action:" + trick["name"]
            state["last_command_at"] = time.time()
            state["moving"] = True
            state["last_error"] = None
        remember("sdk_trick_start", trick=trick)
        completed = False
        error = None
        try:
            g.action(int(trick["id"]))
            completed = wait_motion(settle_s) if settle_s else not motion_is_cancelled()
        except Exception as exc:
            error = repr(exc)
            with state_lock:
                state["last_error"] = error
        finally:
            if not completed:
                stop_burst(3)
            with state_lock:
                state["moving"] = False
            remember("sdk_trick_done", trick=trick, settle_s=settle_s, cancelled=not completed, error=error)
    reason = None if completed else ("gesture_exception" if error else "motion_cancelled")
    return {"ok": completed, "dry_run": False, "reason": reason, "error": error, "trick": trick, "settle_s": settle_s}


def motor_send(action: str, step=None):
    if MOTOR_BACKEND == "sdk":
        return sdk_send(action, step=step)
    return app_send(action)


def stop_burst(n=3):
    sdk_errors = []
    sdk_ok = False
    for _ in range(n):
        try:
            sdk_send("stop")
            sdk_ok = True
        except Exception as e:
            sdk_errors.append(repr(e))
        time.sleep(0.06)

    # The SDK is the authoritative motor path. The vendor app socket is only
    # a best-effort secondary stop for camera/control state and must not turn a
    # successful physical stop into a reported motor failure.
    app_error = None
    try:
        app_send("stop")
    except Exception as e:
        app_error = repr(e)

    err = None if sdk_ok else (sdk_errors[-1] if sdk_errors else "sdk_stop_not_confirmed")
    with state_lock:
        state["moving"] = False
        if err:
            state["last_error"] = err
    remember("stop_burst", n=n, error=err, app_error=app_error, sdk_ok=sdk_ok)
    return err


def request_stop(source="api", n=3):
    """Cancel the exact active lease before stopping; its token can never be revived."""
    lease = active_motion_lease()
    if lease is not None:
        lease.cancel_event.set()
    with state_lock:
        state["motion_cancelled"] = True
    remember("motion_cancel_requested", source=str(source), lease_id=None if lease is None else lease.lease_id)
    return stop_burst(n)


def sector_min(msg, deg_a, deg_b):
    vals = []
    angle = msg.angle_min
    inc = msg.angle_increment
    for r in msg.ranges:
        d = math.degrees(angle)
        while d <= -180:
            d += 360
        while d > 180:
            d -= 360
        inside = (deg_a <= d <= deg_b) if deg_a <= deg_b else (d >= deg_a or d <= deg_b)
        if inside and math.isfinite(float(r)) and float(r) > 0.03:
            vals.append(float(r))
        angle += inc
    return min(vals) if vals else None


def scan_callback(msg):
    sectors = {
        "front": sector_min(msg, -20, 20),
        "front_left": sector_min(msg, 20, 65),
        "front_right": sector_min(msg, -65, -20),
        "left": sector_min(msg, 65, 115),
        "right": sector_min(msg, -115, -65),
        "rear": sector_min(msg, 150, -150),
    }
    with state_lock:
        state["scan_seen"] = True
        state["scan_at"] = time.time()
        state["scan_count"] += 1
        state["sectors"] = sectors
        # Store a compact downsampled scan for occupancy mapping. Angles are robot-frame radians.
        ranges = []
        angle = float(msg.angle_min)
        inc = float(msg.angle_increment)
        step = max(1, int(len(msg.ranges) / 720))
        for i in range(0, len(msg.ranges), step):
            r = float(msg.ranges[i])
            if math.isfinite(r) and 0.04 <= r <= float(msg.range_max):
                ranges.append([round(angle + inc * i, 5), round(r, 3)])
        global last_scan
        last_scan = {"at": state["scan_at"], "ranges": ranges, "range_max": float(msg.range_max)}


def quaternion_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def guard_slam_pose(accepted, stationary_anchor, raw, moving):
    """Reject physically impossible Cartographer drift without hiding raw TF."""
    raw = tuple(map(float, raw))
    if accepted is None:
        return raw, raw, True, "initialized"
    accepted = tuple(map(float, accepted))
    if moving:
        distance = math.hypot(raw[0] - accepted[0], raw[1] - accepted[1])
        yaw_delta = abs(norm_angle(raw[2] - accepted[2]))
        if distance > 0.50 or yaw_delta > 0.90:
            return accepted, None, False, "impossible_moving_jump"
        return raw, None, True, "moving_update"
    anchor = accepted if stationary_anchor is None else tuple(map(float, stationary_anchor))
    distance = math.hypot(raw[0] - anchor[0], raw[1] - anchor[1])
    yaw_delta = abs(norm_angle(raw[2] - anchor[2]))
    if distance > 0.12 or yaw_delta > 0.18:
        return accepted, anchor, False, "stationary_drift"
    return raw, anchor, True, "stationary_jitter"


def occupancy_callback(msg):
    global slam_grid
    now = time.time()
    data = list(msg.data)
    known = sum(1 for value in data if int(value) >= 0)
    occupied = sum(1 for value in data if int(value) >= 50)
    slam_grid = FrontierGrid(
        int(msg.info.width),
        int(msg.info.height),
        float(msg.info.resolution),
        float(msg.info.origin.position.x),
        float(msg.info.origin.position.y),
        data,
    )
    with state_lock:
        slam = dict(state.get("slam") or {})
        slam.update({
            "map_at": now,
            "map_age_s": 0.0,
            "map_width": int(msg.info.width),
            "map_height": int(msg.info.height),
            "resolution_m": float(msg.info.resolution),
            "known_cells": known,
            "occupied_cells": occupied,
        })
        state["slam"] = slam


def ros_thread():
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import OccupancyGrid
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener
    except Exception as e:
        with state_lock:
            state["last_error"] = "rclpy import failed: " + repr(e)
        return

    class ScanNode(Node):
        def __init__(self):
            super().__init__("beep_bridge_scan")
            self.create_subscription(LaserScan, "/scan", scan_callback, 10)
            self.create_subscription(OccupancyGrid, "/map", occupancy_callback, 10)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.accepted_slam_pose = None
            self.stationary_anchor = None
            self.create_timer(0.20, self.update_slam_pose)

        def update_slam_pose(self):
            now = time.time()
            try:
                transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
                t = transform.transform.translation
                q = transform.transform.rotation
                yaw = quaternion_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
                raw_pose = (float(t.x), float(t.y), yaw)
                with state_lock:
                    last_motion_at = state.get("last_motion_at")
                    moving_or_settling = bool(state.get("moving")) or bool(
                        last_motion_at and now - float(last_motion_at) <= 2.0
                    )
                    accepted, anchor, pose_valid, guard_reason = guard_slam_pose(
                        self.accepted_slam_pose,
                        self.stationary_anchor,
                        raw_pose,
                        moving_or_settling,
                    )
                    self.accepted_slam_pose = accepted
                    self.stationary_anchor = anchor
                    if pose_valid:
                        state["pose"] = {
                            "x": round(accepted[0], 4),
                            "y": round(accepted[1], 4),
                            "yaw": round(accepted[2], 4),
                            "source": "guarded_cartographer_slam",
                            "confidence": 0.90,
                            "scan_match_score": None,
                        }
                    slam = dict(state.get("slam") or {})
                    slam.pop("tf_error", None)
                    rejected = int(slam.get("pose_guard_rejections") or 0) + (0 if pose_valid else 1)
                    slam.update({
                        "active": True,
                        "pose_at": now,
                        "pose_age_s": 0.0,
                        "pose_valid": pose_valid,
                        "pose_guard_reason": guard_reason,
                        "pose_guard_rejections": rejected,
                        "raw_pose": {"x": round(raw_pose[0], 4), "y": round(raw_pose[1], 4), "yaw": round(raw_pose[2], 4)},
                    })
                    state["slam"] = slam
            except Exception as exc:
                with state_lock:
                    slam = dict(state.get("slam") or {})
                    pose_at = slam.get("pose_at")
                    slam["pose_age_s"] = None if not pose_at else round(now - float(pose_at), 3)
                    slam["active"] = bool(pose_at and now - float(pose_at) <= 1.0)
                    slam["tf_error"] = repr(exc)
                    state["slam"] = slam

    try:
        rclpy.init(args=None)
        node = ScanNode()
        rclpy.spin(node)
    except Exception as e:
        with state_lock:
            state["last_error"] = "ros thread failed: " + repr(e)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


def snapshot(full=False):
    with state_lock:
        out = dict(state)
        out["uptime_s"] = round(time.time() - state["started_at"], 2)
        out["scan_age_s"] = None if not state.get("scan_at") else round(time.time() - state["scan_at"], 3)
        slam = dict(out.get("slam") or {})
        slam["pose_age_s"] = None if not slam.get("pose_at") else round(time.time() - float(slam["pose_at"]), 3)
        slam["map_age_s"] = None if not slam.get("map_at") else round(time.time() - float(slam["map_at"]), 3)
        pose_age = slam.get("pose_age_s")
        slam["active"] = bool(slam.get("pose_at") and pose_age is not None and float(pose_age) <= 1.0)
        map_age = slam.get("map_age_s")
        map_valid = bool(slam.get("map_at") and map_age is not None and float(map_age) <= 2.5 and
                         int(slam.get("map_width") or 0) > 0 and int(slam.get("map_height") or 0) > 0 and
                         float(slam.get("resolution_m") or 0.0) > 0.0)
        slam["usable"] = bool(slam["active"] and slam.get("pose_valid") is True and map_valid)
        if not slam["active"]:
            slam["usable_reason"] = "pose_missing_or_stale"
        elif slam.get("pose_valid") is not True:
            slam["usable_reason"] = "pose_guard_invalid"
        elif not map_valid:
            slam["usable_reason"] = "map_missing_stale_or_empty"
        else:
            slam["usable_reason"] = "ok"
        out["slam"] = slam
        if full:
            out["last_run"] = last_run
        elif last_run:
            out["last_run_summary"] = {
                "mode": last_run.get("mode"),
                "reason": last_run.get("reason"),
                "commanded_s": last_run.get("commanded_s"),
                "final_front": (last_run.get("status") or {}).get("sectors", {}).get("front"),
            }
        return out


def front_distance(s=None):
    s = s or snapshot()
    v = s.get("sectors", {}).get("front")
    return None if v is None else float(v)


def scan_ok(s=None):
    s = s or snapshot()
    return bool(s.get("scan_seen")) and (s.get("scan_age_s") is not None) and float(s.get("scan_age_s")) <= SCAN_STALE_S


def sector_values(s, names):
    sectors = (s or {}).get("sectors") or {}
    values = {}
    for name in names:
        value = sectors.get(name)
        if value is None:
            return None
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value) or value <= 0.0:
            return None
        values[name] = value
    return values


def slam_ok(s=None):
    s = s or snapshot()
    return bool((s.get("slam") or {}).get("usable"))


def run_action(action: str, duration: float, step=None):
    if str(action).lower() == "stop":
        err = request_stop("run_action_stop")
        return {"ok": err is None, "reason": "motion_cancelled", "error": err, "action": "stop", "duration": 0.0, "status": snapshot()}
    duration = min(max(float(duration), 0.0), MAX_MOVE_S)
    if str(action).lower() != "stop" and duration <= 0.0:
        raise ValueError("non-stop movement requires a positive bounded duration")
    with motion_lock:
        if str(action).lower() != "stop" and motion_is_cancelled():
            return {"ok": False, "reason": "motion_cancelled", "action": action, "duration": duration, "status": snapshot()}
        motor_send(action, step=step)
        if duration > 0 and str(action).lower() != "stop":
            completed = wait_motion(duration)
            stop_burst()
            if not completed:
                return {"ok": False, "reason": "motion_cancelled", "action": action, "duration": duration, "status": snapshot()}
            update_pose_for_action(action, duration)
            update_map_from_scan(room_map) if room_map is not None else None
        return {"ok": True, "backend": MOTOR_BACKEND, "action": action, "step": SDK_STEP_DEFAULT if step is None else int(step), "duration": duration, "pose": pose_copy(), "map": map_summary(room_map) if room_map is not None else None, "status": snapshot()}


def run_supervised_calibration(action: str, duration: float):
    """Run one bounded translation while the normal full-envelope supervisor remains authoritative."""
    action = str(action).lower()
    aliases = {"back": "backward"}
    action = aliases.get(action, action)
    if action not in ("forward", "backward", "left", "right"):
        raise ValueError("supervised calibration supports forward/backward/left/right")
    duration = min(max(float(duration), 0.05), 1.2)
    before = snapshot()
    sectors = validated_sector_values(before.get("sectors"))
    if sectors is None:
        return {"ok": False, "reason": "lidar_sector_missing_or_invalid", "action": action, "status": before}
    if action == "forward" and (sectors["front"] < FORWARD_CORRIDOR_M or
                                min(sectors["front_left"], sectors["front_right"]) < FOOTPRINT_DIAGONAL_M or
                                min(sectors["left"], sectors["right"]) < FOOTPRINT_SIDE_M):
        return {"ok": False, "reason": "forward_corridor_blocked", "action": action, "status": before}
    if action == "backward" and (sectors["rear"] < FOOTPRINT_REAR_M or min(sectors["left"], sectors["right"]) < FOOTPRINT_SIDE_M):
        return {"ok": False, "reason": "reverse_envelope_blocked", "action": action, "status": before}
    if action in ("left", "right") and min(sectors.values()) < TURN_SWEEP_M:
        return {"ok": False, "reason": "lateral_envelope_blocked", "action": action, "status": before}
    start_pose = dict(before.get("pose") or {})
    motor_send(action)
    try:
        supervised = supervise_lidar_motion(action, duration, baseline_sectors=sectors, baseline_pose=start_pose)
    finally:
        stop_burst(3)
    after = snapshot()
    return {"ok": bool(supervised.get("ok")), "reason": supervised.get("reason"), "action": action,
            "duration": duration, "supervision": supervised, "before": before, "status": after}


def capture_frame(timeout=4.0, max_bytes=1_500_000):
    """Return one JPEG from the MJPEG stream.

    The Yahboom video server may connect but emit no bytes unless the app
    control state is Standard/Fullscreen. Stationary capture may prepare that
    mode. During an active motion lease, send only that non-motion mode packet;
    the motor-stop packet remains categorically forbidden.
    """
    timeout = float(timeout)
    deadline = time.time() + timeout
    ctrl = None
    moving_capture = active_motion_lease() is not None
    if moving_capture:
        remember("moving_frame_requested")
    try:
        ctrl = socket.create_connection((HOST, APP_PORT), timeout=1.2)
        ctrl.settimeout(0.25)
        try:
            ctrl.recv(256)
        except Exception:
            pass
        ctrl.sendall(pkt(0x0F, [0x01]))  # Standard/control mode; not a motor command.
        if not moving_capture:
            time.sleep(0.05)
            ctrl.sendall(pkt(0x12, [CMD_PAYLOAD["stop"]]))
            time.sleep(0.15)
    except Exception as e:
        remember("camera_standard_failed", moving_capture=moving_capture, error=repr(e))

    try:
        req = urllib.request.Request(CAMERA_URL, headers={"User-Agent": "beep-bridge/0.3"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            buf = bytearray()
            while time.time() < deadline and len(buf) < max_bytes:
                chunk = r.read(4096)
                if not chunk:
                    break
                buf.extend(chunk)
                start = buf.find(b"\xff\xd8")
                end = buf.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                if start >= 0 and end >= 0:
                    jpg = bytes(buf[start:end + 2])
                    with state_lock:
                        state["last_frame_at"] = time.time()
                        state["last_frame_bytes"] = len(jpg)
                    remember("frame", bytes=len(jpg), moving_capture=moving_capture)
                    return jpg
    finally:
        if ctrl is not None:
            try:
                if not moving_capture:
                    ctrl.sendall(pkt(0x12, [CMD_PAYLOAD["stop"]]))
            except Exception:
                pass
            try:
                ctrl.close()
            except Exception:
                pass
    raise TimeoutError("no JPEG frame from MJPEG stream")



def norm_angle(a):
    while a <= -math.pi:
        a += 2 * math.pi
    while a > math.pi:
        a -= 2 * math.pi
    return a


def pose_copy():
    with state_lock:
        p = dict(state.get("pose") or {})
    return {"x": float(p.get("x", 0.0)), "y": float(p.get("y", 0.0)), "yaw": float(p.get("yaw", 0.0)), "source": p.get("source", "dead_reckoning"), "confidence": float(p.get("confidence", 0.25) or 0.25), "scan_match_score": p.get("scan_match_score")}


def reset_pose(x=0.0, y=0.0, yaw=0.0):
    global scan_match_ref
    with state_lock:
        state["pose"] = {"x": float(x), "y": float(y), "yaw": float(yaw), "source": "dead_reckoning", "confidence": 0.25, "scan_match_score": None}
    scan_match_ref = None
    remember("pose_reset", x=float(x), y=float(y), yaw=float(yaw))


def update_pose_for_action(action, duration):
    """Crude dead reckoning. This is deliberately humble, unlike most robot demos."""
    action = str(action).lower()
    duration = float(duration or 0.0)
    with state_lock:
        slam = dict(state.get("slam") or {})
        slam_at = slam.get("pose_at")
        if state.get("pose", {}).get("source") in ("cartographer_slam", "guarded_cartographer_slam") and slam_at and time.time() - float(slam_at) <= 1.0:
            return
        p = dict(state.get("pose") or {"x": 0.0, "y": 0.0, "yaw": 0.0, "source": "dead_reckoning"})
        x, y, yaw = float(p.get("x", 0.0)), float(p.get("y", 0.0)), float(p.get("yaw", 0.0))
        if action == "forward":
            d = SDK_FORWARD_M_PER_S * duration
            x += math.cos(yaw) * d
            y += math.sin(yaw) * d
        elif action in ("back", "backward"):
            d = SDK_FORWARD_M_PER_S * duration
            x -= math.cos(yaw) * d
            y -= math.sin(yaw) * d
        elif action == "right":
            d = SDK_STRAFE_M_PER_S * duration
            x += math.cos(yaw - math.pi / 2) * d
            y += math.sin(yaw - math.pi / 2) * d
        elif action == "left":
            d = SDK_STRAFE_M_PER_S * duration
            x += math.cos(yaw + math.pi / 2) * d
            y += math.sin(yaw + math.pi / 2) * d
        elif action in ("turnleft", "turn_left", "rotate_left"):
            yaw = norm_angle(yaw + SDK_TURN_RAD_PER_S * duration)
        elif action in ("turnright", "turn_right", "rotate_right"):
            yaw = norm_angle(yaw - SDK_TURN_RAD_PER_S * duration)
        prev_conf = float(p.get("confidence", 0.25) or 0.25)
        state["pose"] = {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4), "source": "dead_reckoning", "confidence": max(0.10, prev_conf * 0.82), "scan_match_score": p.get("scan_match_score")}


def new_room_map(name="room"):
    n = int(MAP_SIZE_M / MAP_RES_M)
    if n % 2:
        n += 1
    return {
        "version": 1,
        "name": str(name),
        "created_at": time.time(),
        "updated_at": time.time(),
        "resolution_m": MAP_RES_M,
        "size_m": MAP_SIZE_M,
        "width": n,
        "height": n,
        "origin": [-MAP_SIZE_M / 2.0, -MAP_SIZE_M / 2.0],
        "pose_start": pose_copy(),
        "pose": pose_copy(),
        "log_odds": {},
        "path": [],
        "scan_updates": 0,
        "notes": "LiDAR occupancy trace placed with Cartographer pose when SLAM is active; ROS /map remains the authoritative loop-closed map.",
    }


def ensure_room_map(name="room", reset=False):
    global room_map
    if reset or room_map is None:
        MAP_DIR.mkdir(parents=True, exist_ok=True)
        room_map = new_room_map(name)
        with state_lock:
            state["map"] = {"active": True, "updated_at": room_map["updated_at"], "scan_updates": 0, "occupied_cells": 0, "free_cells": 0}
        remember("map_new", name=name)
    return room_map


def world_to_cell(m, x, y):
    ox, oy = m["origin"]
    ix = int((x - ox) / m["resolution_m"])
    iy = int((y - oy) / m["resolution_m"])
    if 0 <= ix < m["width"] and 0 <= iy < m["height"]:
        return ix, iy
    return None


def bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0); dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy; x += sx
        if e2 <= dx:
            err += dx; y += sy


def add_logodds(m, cell, delta):
    if cell is None:
        return
    key = f"{cell[0]},{cell[1]}"
    m["log_odds"][key] = max(-5.0, min(5.0, float(m["log_odds"].get(key, 0.0)) + float(delta)))



def scan_points(scan=None, max_points=220, max_range=2.5, min_range=0.08):
    scan = scan or last_scan
    if not scan or not scan.get("ranges"):
        return []
    raw = scan.get("ranges") or []
    step = max(1, int(len(raw) / max_points))
    pts = []
    for angle, dist in raw[::step]:
        d = float(dist)
        if min_range <= d <= max_range:
            a = float(angle)
            pts.append((math.cos(a) * d, math.sin(a) * d))
    return pts


def transform_point(pt, pose):
    x, y = pt
    c = math.cos(float(pose.get("yaw", 0.0)))
    ss = math.sin(float(pose.get("yaw", 0.0)))
    return (float(pose.get("x", 0.0)) + c * x - ss * y, float(pose.get("y", 0.0)) + ss * x + c * y)


def transform_points(pts, pose):
    return [transform_point(p, pose) for p in pts]


def nn_score(candidate_world, ref_world, max_pairs=90):
    if not candidate_world or not ref_world:
        return 999.0
    step = max(1, int(len(candidate_world) / max_pairs))
    total = 0.0
    n = 0
    # Brute force is fine here: tiny point clouds, tiny robot, tiny dignity.
    for x, y in candidate_world[::step]:
        best = 0.35
        for rx, ry in ref_world:
            d = (x - rx) * (x - rx) + (y - ry) * (y - ry)
            if d < best * best:
                best = math.sqrt(d)
        total += best
        n += 1
    return total / max(1, n)


def maybe_scan_match_pose():
    """Correct dead-reckoned pose by matching the latest scan to the previous scan.

    This is not Cartographer. It is a small local scan matcher. Its purpose is to
    reduce command/foot-slip drift enough that maps stop looking like modern art.
    """
    global scan_match_ref
    pts = scan_points(max_points=200, max_range=2.2)
    if len(pts) < 35:
        return {"ok": False, "reason": "not_enough_points", "points": len(pts)}
    prior = pose_copy()
    if scan_match_ref is None or len(scan_match_ref.get("world_points", [])) < 35:
        scan_match_ref = {"pose": prior, "robot_points": pts, "world_points": transform_points(pts, prior), "at": time.time()}
        with state_lock:
            state["pose"]["source"] = "dead_reckoning+scan_ref"
            state["pose"]["confidence"] = max(float(state["pose"].get("confidence", 0.25) or 0.25), 0.35)
        return {"ok": True, "reason": "reference_initialized", "points": len(pts), "pose": prior}

    ref = scan_match_ref["world_points"]
    best = None
    # Search around the dead-reckoned prior. Small range by design: avoids snapping
    # to a wrong wall when the room is repetitive.
    yaw_offsets = [-0.24, -0.16, -0.08, 0.0, 0.08, 0.16, 0.24]
    xy_offsets = [-0.08, -0.04, 0.0, 0.04, 0.08]
    for dyaw in yaw_offsets:
        for dx in xy_offsets:
            for dy in xy_offsets:
                cand = {"x": prior["x"] + dx, "y": prior["y"] + dy, "yaw": norm_angle(prior["yaw"] + dyaw)}
                score = nn_score(transform_points(pts, cand), ref)
                if best is None or score < best[0]:
                    best = (score, cand)
    if best is None:
        return {"ok": False, "reason": "no_candidate"}
    score, cand = best
    # Conservative acceptance. If poor, keep odometry but lower confidence.
    if score > 0.22:
        conf = 0.18
        with state_lock:
            state["pose"]["confidence"] = conf
            state["pose"]["scan_match_score"] = round(score, 4)
        scan_match_ref = {"pose": prior, "robot_points": pts, "world_points": transform_points(pts, prior), "at": time.time()}
        return {"ok": False, "reason": "poor_match", "score": round(score, 4), "pose": prior}
    alpha = 0.75 if score < 0.09 else (0.55 if score < 0.14 else 0.35)
    corrected = {
        "x": round(prior["x"] * (1 - alpha) + cand["x"] * alpha, 4),
        "y": round(prior["y"] * (1 - alpha) + cand["y"] * alpha, 4),
        "yaw": round(norm_angle(prior["yaw"] * (1 - alpha) + cand["yaw"] * alpha), 4),
        "source": "dead_reckoning+scan_match",
        "confidence": round(max(0.25, min(0.90, 1.0 - score / 0.24)), 3),
        "scan_match_score": round(score, 4),
    }
    with state_lock:
        state["pose"] = corrected
    scan_match_ref = {"pose": corrected, "robot_points": pts, "world_points": transform_points(pts, corrected), "at": time.time()}
    remember("scan_match", score=round(score, 4), confidence=corrected["confidence"], pose=corrected)
    return {"ok": True, "reason": "corrected", "score": round(score, 4), "pose": corrected}


def local_map_summary():
    pts = scan_points(max_points=360, max_range=2.5)
    if not pts:
        return {"ok": False, "reason": "no_scan"}
    front_clear = min([math.hypot(x, y) for x, y in pts if x > 0 and abs(math.atan2(y, x)) < math.radians(20)] or [None])
    left_clear = min([math.hypot(x, y) for x, y in pts if y > 0 and abs(math.atan2(y, x) - math.pi/2) < math.radians(35)] or [None])
    right_clear = min([math.hypot(x, y) for x, y in pts if y < 0 and abs(math.atan2(y, x) + math.pi/2) < math.radians(35)] or [None])
    return {"ok": True, "points": len(pts), "radius_m": 2.5, "front_clear_m": front_clear, "left_clear_m": left_clear, "right_clear_m": right_clear, "pose": pose_copy()}


def local_map_svg():
    pts = scan_points(max_points=500, max_range=2.5)
    size = 700
    scale = size / 5.0
    cx = cy = size / 2
    dots = []
    # draw robot, heading, safety rings
    rings = []
    for r in [0.25, 0.5, 1.0, 1.5, 2.0]:
        rings.append(f"<circle cx='{cx}' cy='{cy}' r='{r*scale}' fill='none' stroke='#ddd' stroke-width='1'/>")
    for x, y in pts:
        sx = cx + x * scale
        sy = cy - y * scale
        color = '#111' if math.hypot(x, y) < 0.5 else '#444'
        dots.append(f"<circle cx='{sx:.1f}' cy='{sy:.1f}' r='2.2' fill='{color}'/>")
    robot = f"<circle cx='{cx}' cy='{cy}' r='8' fill='red'/><line x1='{cx}' y1='{cy}' x2='{cx+40}' y2='{cy}' stroke='red' stroke-width='3'/>"
    svg = f"<svg xmlns='http://www.w3.org/2000/svg' width='{size}' height='{size}' viewBox='0 0 {size} {size}'><rect width='100%' height='100%' fill='white'/>{''.join(rings)}{''.join(dots)}{robot}<text x='10' y='20' font-size='16'>BEEP local LiDAR map (robot frame)</text></svg>"
    return svg.encode('utf-8')

def update_map_from_scan(m=None, scan_match=True):
    global room_map
    if m is None:
        m = ensure_room_map()
    with state_lock:
        scan = dict(last_scan) if last_scan else None
        pose = dict(state.get("pose") or {})
    if not scan or not scan.get("ranges"):
        return {"ok": False, "reason": "no_scan"}
    match = maybe_scan_match_pose() if scan_match else {"ok": False, "reason": "disabled_for_fast_explore"}
    with state_lock:
        pose = dict(state.get("pose") or {})
    px, py, yaw = float(pose.get("x", 0.0)), float(pose.get("y", 0.0)), float(pose.get("yaw", 0.0))
    start = world_to_cell(m, px, py)
    if start is None:
        return {"ok": False, "reason": "pose_outside_map"}
    updates = 0
    max_use = min(float(scan.get("range_max") or 3.5), MAP_SIZE_M / 2.0)
    for angle, dist in scan["ranges"]:
        dist = min(float(dist), max_use)
        wx = px + math.cos(yaw + float(angle)) * dist
        wy = py + math.sin(yaw + float(angle)) * dist
        end = world_to_cell(m, wx, wy)
        if end is None:
            continue
        cells = list(bresenham(start[0], start[1], end[0], end[1]))
        # Mark free space along the ray, obstacle at endpoint.
        for c in cells[:-1:2]:
            add_logodds(m, c, -0.25)
        add_logodds(m, cells[-1], 0.85)
        updates += 1
    m["updated_at"] = time.time()
    m["pose"] = pose_copy()
    m["path"].append([round(px, 3), round(py, 3), round(yaw, 3), round(time.time(), 3)])
    m["path"] = m["path"][-2000:]
    m["scan_updates"] += 1
    occ = sum(1 for v in m["log_odds"].values() if v > 0.8)
    free = sum(1 for v in m["log_odds"].values() if v < -0.8)
    quality = "high" if float(pose.get("confidence", 0.0) or 0.0) >= 0.65 else ("medium" if float(pose.get("confidence", 0.0) or 0.0) >= 0.35 else "low")
    with state_lock:
        state["map"] = {"active": True, "updated_at": m["updated_at"], "scan_updates": m["scan_updates"], "occupied_cells": occ, "free_cells": free, "quality": quality}
    return {"ok": True, "rays": updates, "occupied_cells": occ, "free_cells": free, "pose": pose_copy(), "scan_match": match, "quality": quality}


def map_summary(m=None):
    m = m or room_map
    if not m:
        return {"active": False, "reason": "no_map"}
    occ = sum(1 for v in m["log_odds"].values() if v > 0.8)
    free = sum(1 for v in m["log_odds"].values() if v < -0.8)
    pose = pose_copy()
    quality = "high" if float(pose.get("confidence", 0.0) or 0.0) >= 0.65 else ("medium" if float(pose.get("confidence", 0.0) or 0.0) >= 0.35 else "low")
    return {"active": True, "name": m.get("name"), "resolution_m": m.get("resolution_m"), "size_m": m.get("size_m"), "width": m.get("width"), "height": m.get("height"), "scan_updates": m.get("scan_updates", 0), "occupied_cells": occ, "free_cells": free, "pose": pose, "quality": quality, "updated_at": m.get("updated_at")}


def save_room_map(name=None):
    m = room_map
    if not m:
        return None
    MAP_DIR.mkdir(parents=True, exist_ok=True)
    safe = ''.join(c if c.isalnum() or c in '-_' else '_' for c in str(name or m.get("name") or "room"))[:60]
    path = MAP_DIR / f"{safe}_{int(time.time())}.json"
    path.write_text(json.dumps(m, sort_keys=True, indent=2))
    remember("map_saved", path=str(path))
    return str(path)


def map_svg(m=None):
    m = m or room_map
    if not m:
        return b"<svg xmlns='http://www.w3.org/2000/svg' width='400' height='400'><text x='20' y='20'>no map</text></svg>"
    w, h = int(m["width"]), int(m["height"])
    scale = max(2, min(6, int(700 / max(w, h))))
    rects = []
    for key, v in m["log_odds"].items():
        if abs(v) < 0.8:
            continue
        ix, iy = map(int, key.split(','))
        color = "#111" if v > 0 else "#e8e8e8"
        rects.append(f"<rect x='{ix*scale}' y='{(h-1-iy)*scale}' width='{scale}' height='{scale}' fill='{color}'/>")
    # path in red
    pts = []
    for x, y, yaw, t in m.get("path", [])[-500:]:
        c = world_to_cell(m, x, y)
        if c:
            pts.append(f"{c[0]*scale},{(h-1-c[1])*scale}")
    path = f"<polyline points='{' '.join(pts)}' fill='none' stroke='red' stroke-width='2'/>" if pts else ""
    svg = f"<svg xmlns='http://www.w3.org/2000/svg' width='{w*scale}' height='{h*scale}' viewBox='0 0 {w*scale} {h*scale}'><rect width='100%' height='100%' fill='white'/>{''.join(rects)}{path}</svg>"
    return svg.encode('utf-8')


def validated_sector_values(sectors):
    required = ("front", "front_left", "front_right", "left", "right", "rear")
    if not isinstance(sectors, dict):
        return None
    values = {}
    for name in required:
        raw = sectors.get(name)
        if raw is None:
            return None
        try:
            value = float(raw)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value) or value <= 0.0:
            return None
        values[name] = value
    return values


def choose_explore_action(sectors):
    values = validated_sector_values(sectors)
    if values is None:
        return "stop", 0.0, "lidar_sector_missing_or_invalid"
    front, fl, fr = values["front"], values["front_left"], values["front_right"]
    left, right, rear = values["left"], values["right"], values["rear"]
    front_breach = front < FOOTPRINT_FRONT_M or min(fl, fr) < FOOTPRINT_DIAGONAL_M
    body_sides_clear = min(left, right) >= FOOTPRINT_SIDE_M
    rear_escape_clear = rear >= FOOTPRINT_REAR_M and body_sides_clear

    if front_breach:
        if min(front, fl, fr, left, right, rear) >= TURN_START_CLEARANCE_M:
            if max(left, fl) >= max(right, fr):
                return "turnleft", 0.30, "front_breach_sweep_left"
            return "turnright", 0.30, "front_breach_sweep_right"
        if (front >= 0.46 and min(fl, fr) >= TURN_SWEEP_M and
                left >= FOOTPRINT_SIDE_M and right >= 0.55 and rear >= FOOTPRINT_REAR_M):
            return "right", 0.35, "bounded_lateral_turn_setup"
        if rear_escape_clear:
            return "back", REVERSE_ESCAPE_MAX_S, "front_footprint_breach_backoff"
        return "stop", 0.0, "footprint_boxed_in_stop"
    if min(left, right) < FOOTPRINT_SIDE_M:
        return "stop", 0.0, "side_footprint_breach_stop"
    if (front >= FORWARD_CORRIDOR_M and fl >= FOOTPRINT_DIAGONAL_M and
            fr >= FOOTPRINT_DIAGONAL_M):
        return "forward", 0.50, "full_body_corridor_clear"

    sweep_clear = min(front, fl, fr, left, right, rear) >= TURN_START_CLEARANCE_M
    if sweep_clear:
        if max(left, fl) >= max(right, fr):
            return "turnleft", 0.30, "footprint_sweep_left"
        return "turnright", 0.30, "footprint_sweep_right"
    if rear_escape_clear:
        return "back", REVERSE_ESCAPE_MAX_S, "turn_sweep_blocked_backoff"
    return "stop", 0.0, "footprint_boxed_in_stop"


def escape_made_progress(action, before, after, before_pose=None, after_pose=None):
    """Require measured geometric improvement before allowing another escape."""
    before_values = validated_sector_values(before)
    after_values = validated_sector_values(after)
    if before_values is None or after_values is None:
        return False
    if action in ("back", "backward"):
        if (after_values["rear"] < FOOTPRINT_REAR_M or
                min(after_values["left"], after_values["right"]) < FOOTPRINT_SIDE_M):
            return False
        before_front = sum(before_values[name] for name in ("front", "front_left", "front_right")) / 3.0
        after_front = sum(after_values[name] for name in ("front", "front_left", "front_right")) / 3.0
        return after_front - before_front >= ESCAPE_CLEARANCE_GAIN_M
    if action in ("turnleft", "turnright"):
        if not before_pose or not after_pose or before_pose.get("yaw") is None or after_pose.get("yaw") is None:
            return False
        yaw_change = abs(norm_angle(float(after_pose["yaw"]) - float(before_pose["yaw"])))
        sweep = min(after_values[name] for name in ("front", "front_left", "front_right", "left", "right", "rear"))
        return yaw_change >= math.radians(15.0) and sweep >= TURN_SWEEP_M
    if action == "right":
        before_front = sum(before_values[name] for name in ("front", "front_left", "front_right")) / 3.0
        after_front = sum(after_values[name] for name in ("front", "front_left", "front_right")) / 3.0
        return (after_values["left"] - before_values["left"] >= 0.008 and
                after_values["right"] >= 0.45 and after_front >= before_front - 0.02)
    return False


def turn_window_stalled(elapsed_s, progress_delta_rad):
    return (float(elapsed_s) >= TURN_PROGRESS_WINDOW_S and
            float(progress_delta_rad) < TURN_PROGRESS_MIN_RAD)


def explore_step_for(action):
    if action == "forward":
        return 20
    if action in ("back", "backward"):
        return 15
    if action in ("left", "right"):
        return 5
    if action in ("turnleft", "turnright"):
        return 30
    return SDK_STEP_DEFAULT


def explore_room(name="room", max_duration=30.0, reset_map=False, save=True, rotate_scan=True):
    global active_explore, last_run
    max_duration = min(max(float(max_duration), 0.0), 180.0)
    started = time.time()
    trace = []
    last_escape_action = None
    consecutive_escape_actions = 0
    m = ensure_room_map(name=name, reset=reset_map)
    active_explore = {"active": True, "started_at": started, "name": name}
    reason = "max_duration"
    with motion_lock:
        try:
            s0 = snapshot()
            if not scan_ok(s0):
                reason = "scan_stale_or_missing_before_start"
                return {"ok": False, "mode": "explore_room", "reason": reason, "status": s0, "map": map_summary(m), "trace_tail": []}
            if not (s0.get("slam") or {}).get("active"):
                reason = "slam_pose_unavailable_before_start"
                return {"ok": False, "mode": "explore_room", "reason": reason, "status": s0, "map": map_summary(m), "trace_tail": []}
            update_map_from_scan(m, scan_match=False)
            if rotate_scan:
                # Rotate only when the complete body sweep is clear and each pulse
                # produces measurable heading progress.
                for i in range(6):
                    if motion_is_cancelled():
                        reason = "motion_cancelled"
                        break
                    if time.time() - started >= max_duration:
                        break
                    before_state = snapshot()
                    before_sectors = before_state.get("sectors") or {}
                    if len(before_sectors) < 6 or min(float(before_sectors.get(name) or 0.0) for name in
                                                      ("front", "front_left", "front_right", "left", "right", "rear")) < TURN_SWEEP_M:
                        reason = "rotation_sweep_clearance_rejected"
                        break
                    before_pose = pose_copy()
                    supervised = guarded_slam_turn(
                        turn="left", degrees=20.0, max_duration=2.0, step=30)
                    after_state = snapshot()
                    after_sectors = after_state.get("sectors") or {}
                    after_pose = pose_copy()
                    progress = bool(supervised["ok"])
                    trace.append({"phase": "rotate_scan", "i": i + 1,
                                  "supervisor_reason": supervised["reason"], "progress": progress,
                                  "sectors_before": before_sectors, "sectors_after": after_sectors})
                    if not supervised["ok"]:
                        reason = str(supervised["reason"])
                        break
                    if not progress:
                        reason = "rotation_no_progress"
                        break
                    time.sleep(0.08)
                    update_map_from_scan(m, scan_match=False)
            while reason == "max_duration" and time.time() - started < max_duration:
                if motion_is_cancelled():
                    reason = "motion_cancelled"
                    break
                st = snapshot()
                if not scan_ok(st):
                    reason = "scan_stale_or_missing"
                    break
                sec = st.get("sectors", {})
                action, dur, why = choose_explore_action(sec)
                if action == "stop":
                    reason = why
                    trace.append({"action": "stop", "why": why, "sectors": sec})
                    break
                if action == "forward":
                    last_escape_action = None
                    consecutive_escape_actions = 0
                elif action == last_escape_action:
                    consecutive_escape_actions += 1
                else:
                    last_escape_action = action
                    consecutive_escape_actions = 1
                if action != "forward" and consecutive_escape_actions > 2:
                    reason = "repeated_escape_action_stop"
                    trace.append({"action": "stop", "why": reason,
                                  "attempted_action": action, "sectors": sec})
                    break
                step = explore_step_for(action)
                before_pose = pose_copy()
                if action in ("turnleft", "turnright"):
                    remaining = max_duration - (time.time() - started)
                    if remaining < 1.0:
                        reason = "max_duration"
                        break
                    supervised = guarded_slam_turn(
                        turn="left" if action == "turnleft" else "right",
                        degrees=20.0, max_duration=min(2.0, remaining), step=step)
                elif action in ("back", "backward"):
                    supervised = yaw_corrected_reverse_escape(sec, before_pose)
                else:
                    motor_send(action, step=step)
                    supervised = supervise_lidar_motion(
                        action, dur, baseline_sectors=sec, baseline_pose=before_pose,
                        stop_on_front_gain_m=ESCAPE_CLEARANCE_GAIN_M if action in ("back", "backward") else None)
                    stop_burst(2)
                after_state = snapshot()
                after_sectors = after_state.get("sectors") or {}
                after_pose = pose_copy()
                progress = (action == "forward" or
                            (action in ("turnleft", "turnright") and bool(supervised["ok"])) or
                            (bool(supervised["ok"]) and (
                                supervised.get("reason") == "reverse_clearance_gain_reached" or
                                supervised.get("recovery_action") == "turn" or
                                escape_made_progress(action, sec, after_sectors, before_pose, after_pose))))
                trace.append({"action": action, "step": step,
                              "duration": round(float(supervised.get("elapsed_s", 0.0)), 2), "why": why,
                              "sectors_before": sec, "sectors_after": after_sectors,
                              "progress": progress, "supervisor_reason": supervised["reason"],
                              "recovery_attempts": supervised.get("attempts")})
                if not supervised["ok"]:
                    reason = str(supervised["reason"])
                    break
                if not progress:
                    reason = "escape_no_progress"
                    break
                time.sleep(0.05)
                update_map_from_scan(m, scan_match=False)
        except Exception as e:
            reason = "exception:" + repr(e)
            with state_lock:
                state["last_error"] = repr(e)
        finally:
            stop_burst(3)
            active_explore = None
    saved_path = save_room_map(name) if save else None
    result = {"ok": reason == "max_duration", "mode": "explore_room", "reason": reason, "elapsed_s": round(time.time() - started, 2), "map": map_summary(m), "saved_path": saved_path, "trace_tail": trace[-30:], "status": snapshot()}
    last_run = result
    remember("explore_done", reason=reason, elapsed_s=result["elapsed_s"], saved_path=saved_path)
    return result


def slam_grid_copy():
    # FrontierGrid is immutable after construction, so sharing the latest complete
    # callback snapshot is atomic enough under CPython and avoids copying thousands of cells.
    return slam_grid


def save_frontier_trace(name, result):
    MAP_DIR.mkdir(parents=True, exist_ok=True)
    safe = ''.join(c if c.isalnum() or c in '-_' else '_' for c in str(name or "dog_frontier"))[:60]
    path = MAP_DIR / f"{safe}_frontiers_{int(time.time())}.json"
    path.write_text(json.dumps(result, sort_keys=True, indent=2))
    return str(path)


def supervise_fluent_forward(duration, poll_s=0.08):
    """Keep an already-started gait active while independently polling safety."""
    started = time.time()
    deadline = started + max(0.0, float(duration))
    while time.time() < deadline:
        if motion_is_cancelled():
            return {"ok": False, "reason": "motion_cancelled", "elapsed_s": time.time() - started}
        st = snapshot()
        slam = st.get("slam") or {}
        if not scan_ok(st):
            return {"ok": False, "reason": "scan_stale_during_fluent_forward", "elapsed_s": time.time() - started}
        if not slam_ok(st):
            return {"ok": False, "reason": "slam_unusable_during_fluent_forward", "elapsed_s": time.time() - started}
        sec = sector_values(st, ("front", "front_left", "front_right", "left", "right", "rear"))
        if sec is None:
            return {"ok": False, "reason": "lidar_sector_missing_during_fluent_forward", "elapsed_s": time.time() - started}
        if (sec["front"] < FORWARD_CORRIDOR_M or sec["front_left"] < FOOTPRINT_DIAGONAL_M or
                sec["front_right"] < FOOTPRINT_DIAGONAL_M or
                min(sec["left"], sec["right"]) < FOOTPRINT_SIDE_M):
            return {"ok": False, "reason": "obstacle_during_fluent_forward", "elapsed_s": time.time() - started}
        time.sleep(min(float(poll_s), max(0.0, deadline - time.time())))
    return {"ok": True, "reason": "window_complete", "elapsed_s": time.time() - started}


def supervise_lidar_motion(action, duration, poll_s=0.08, baseline_sectors=None,
                           baseline_pose=None, stop_on_front_gain_m=None):
    """Supervise one body-relative action against the full LiDAR footprint."""
    action = str(action).lower()
    started = time.time()
    deadline = started + max(0.0, float(duration))
    baseline_values = validated_sector_values(baseline_sectors) if baseline_sectors is not None else None
    baseline_front = None if baseline_values is None else sum(
        baseline_values[name] for name in ("front", "front_left", "front_right")) / 3.0
    baseline_yaw = None
    if baseline_pose is not None and baseline_pose.get("yaw") is not None:
        baseline_yaw = float(baseline_pose["yaw"])
    while time.time() < deadline:
        if motion_is_cancelled():
            return {"ok": False, "reason": "motion_cancelled", "elapsed_s": time.time() - started}
        st = snapshot()
        if not scan_ok(st):
            return {"ok": False, "reason": "scan_stale_during_lidar_walk", "elapsed_s": time.time() - started}
        sec = sector_values(st, ("front", "front_left", "front_right", "left", "right", "rear"))
        if sec is None:
            return {"ok": False, "reason": "lidar_sector_missing_during_lidar_walk", "elapsed_s": time.time() - started}
        if action == "forward":
            if (sec["front"] < FORWARD_CORRIDOR_M or sec["front_left"] < FOOTPRINT_DIAGONAL_M or
                    sec["front_right"] < FOOTPRINT_DIAGONAL_M or min(sec["left"], sec["right"]) < FOOTPRINT_SIDE_M):
                return {"ok": False, "reason": "forward_footprint_breach", "elapsed_s": time.time() - started}
        elif action in ("back", "backward"):
            if sec["rear"] < FOOTPRINT_REAR_M:
                return {"ok": False, "reason": "rear_clearance_breach", "elapsed_s": time.time() - started}
            if min(sec["left"], sec["right"]) < FOOTPRINT_SIDE_M:
                return {"ok": False, "reason": "side_clearance_breach_during_backoff", "elapsed_s": time.time() - started}
            current_front = sum(sec[name] for name in ("front", "front_left", "front_right")) / 3.0
            if baseline_front is None:
                baseline_front = current_front
            elif baseline_front - current_front > BACKOFF_MAX_FRONT_LOSS_M:
                return {"ok": False, "reason": "front_clearance_worsening_during_backoff",
                        "elapsed_s": time.time() - started}
            elif (stop_on_front_gain_m is not None and
                  current_front - baseline_front >= float(stop_on_front_gain_m)):
                return {"ok": True, "reason": "reverse_clearance_gain_reached",
                        "elapsed_s": time.time() - started,
                        "front_clearance_gain_m": current_front - baseline_front}
            yaw = ((st.get("pose") or {}).get("yaw"))
            if baseline_yaw is None and yaw is not None:
                baseline_yaw = float(yaw)
            elif baseline_yaw is not None and yaw is not None:
                drift = abs(norm_angle(float(yaw) - baseline_yaw))
                if drift > BACKOFF_MAX_HEADING_DRIFT_RAD:
                    return {"ok": False, "reason": "heading_drift_during_backoff",
                            "elapsed_s": time.time() - started,
                            "heading_drift_degrees": round(math.degrees(drift), 2)}
        elif action == "right":
            if (sec["front"] < 0.44 or min(sec["front_left"], sec["front_right"]) < TURN_SWEEP_M or
                    sec["left"] < FOOTPRINT_SIDE_M or sec["right"] < 0.45 or
                    sec["rear"] < FOOTPRINT_REAR_M):
                return {"ok": False, "reason": "lateral_setup_clearance_breach",
                        "elapsed_s": time.time() - started}
            current_front = sum(sec[name] for name in ("front", "front_left", "front_right")) / 3.0
            if baseline_front is None:
                baseline_front = current_front
            elif baseline_front - current_front > 0.02:
                return {"ok": False, "reason": "front_clearance_worsening_during_lateral_setup",
                        "elapsed_s": time.time() - started}
            yaw = ((st.get("pose") or {}).get("yaw"))
            if baseline_yaw is None and yaw is not None:
                baseline_yaw = float(yaw)
            elif baseline_yaw is not None and yaw is not None:
                drift = abs(norm_angle(float(yaw) - baseline_yaw))
                if drift > BACKOFF_MAX_HEADING_DRIFT_RAD:
                    return {"ok": False, "reason": "heading_drift_during_lateral_setup",
                            "elapsed_s": time.time() - started}
        elif action in ("turnleft", "turnright"):
            if min(sec.values()) < TURN_SWEEP_M:
                return {"ok": False, "reason": "turn_sweep_clearance_breach", "elapsed_s": time.time() - started}
        else:
            return {"ok": False, "reason": "unsupported_reactive_motion", "elapsed_s": time.time() - started}
        time.sleep(min(float(poll_s), max(0.0, deadline - time.time())))
    return {"ok": True, "reason": "window_complete", "elapsed_s": time.time() - started}


def supervise_lidar_forward(duration, poll_s=0.08):
    """Compatibility wrapper for the local-LiDAR forward supervisor."""
    result = supervise_lidar_motion("forward", duration, poll_s=poll_s)
    if result["reason"] == "forward_footprint_breach":
        result = dict(result, reason="obstacle_during_lidar_walk")
    return result


def yaw_corrected_reverse_escape(baseline_sectors, baseline_pose):
    """Reverse in bounded segments, correcting measured yaw between segments."""
    baseline_values = validated_sector_values(baseline_sectors)
    if baseline_values is None or not baseline_pose or baseline_pose.get("yaw") is None:
        return {"ok": False, "reason": "reverse_baseline_invalid", "elapsed_s": 0.0,
                "attempts": [], "status": snapshot()}
    target_yaw = float(baseline_pose["yaw"])
    baseline_front = sum(
        baseline_values[name] for name in ("front", "front_left", "front_right")) / 3.0
    started = time.time()
    attempts = []

    for attempt in range(1, REVERSE_CORRECTION_ATTEMPTS + 1):
        if motion_is_cancelled():
            return {"ok": False, "reason": "motion_cancelled", "elapsed_s": time.time() - started,
                    "attempts": attempts, "status": snapshot()}
        before = snapshot()
        required = sector_values(before, ("front", "front_left", "front_right", "left", "right", "rear"))
        pose = before.get("pose") or {}
        if (required is None or pose.get("yaw") is None or not scan_ok(before) or not slam_ok(before)):
            return {"ok": False, "reason": "reverse_guard_invalid_before_segment",
                    "elapsed_s": time.time() - started, "attempts": attempts, "status": before}
        if required["rear"] < FOOTPRINT_REAR_M or min(required["left"], required["right"]) < FOOTPRINT_SIDE_M:
            return {"ok": False, "reason": "reverse_envelope_rejected_before_segment",
                    "elapsed_s": time.time() - started, "attempts": attempts, "status": before}

        motor_send("back", step=REVERSE_CORRECTION_STEP)
        segment = supervise_lidar_motion(
            "back", REVERSE_CORRECTION_SEGMENT_S,
            baseline_sectors=baseline_values, baseline_pose={"yaw": target_yaw},
            stop_on_front_gain_m=ESCAPE_CLEARANCE_GAIN_M)
        stop_burst(2)
        after = snapshot()
        after_required = sector_values(
            after, ("front", "front_left", "front_right", "left", "right", "rear"))
        after_pose = after.get("pose") or {}
        gain = None
        drift = None
        if after_required is not None:
            gain = (sum(after_required[name] for name in ("front", "front_left", "front_right")) / 3.0 -
                    baseline_front)
        if after_pose.get("yaw") is not None:
            drift = norm_angle(float(after_pose["yaw"]) - target_yaw)
        item = {"attempt": attempt, "segment": segment,
                "front_clearance_gain_m": gain,
                "heading_drift_degrees": None if drift is None else math.degrees(drift)}
        attempts.append(item)

        if segment.get("ok") and (segment.get("reason") == "reverse_clearance_gain_reached" or
                                  (gain is not None and gain >= ESCAPE_CLEARANCE_GAIN_M)):
            return {"ok": True, "reason": "reverse_clearance_gain_reached",
                    "elapsed_s": time.time() - started, "attempts": attempts, "status": after}
        if segment.get("reason") == "front_clearance_worsening_during_backoff":
            if after_required is not None and min(after_required.values()) >= TURN_START_CLEARANCE_M:
                fallback_direction = ("left" if max(after_required["left"], after_required["front_left"]) >=
                                      max(after_required["right"], after_required["front_right"]) else "right")
                fallback = guarded_slam_turn(
                    turn=fallback_direction, degrees=20.0,
                    max_duration=2.0, tolerance_degrees=5.0, step=30)
                item["fallback_turn"] = fallback
                if fallback.get("ok"):
                    return {"ok": True, "reason": "reverse_worsened_turn_recovery",
                            "recovery_action": "turn", "elapsed_s": time.time() - started,
                            "attempts": attempts, "status": fallback.get("status") or snapshot()}
                return {"ok": False,
                        "reason": "reverse_worsened_turn_failed:" + str(fallback.get("reason")),
                        "elapsed_s": time.time() - started, "attempts": attempts,
                        "status": fallback.get("status") or snapshot()}
            return {"ok": False, "reason": "front_clearance_worsening_during_backoff",
                    "elapsed_s": time.time() - started, "attempts": attempts, "status": after}
        if segment.get("reason") not in ("window_complete", "heading_drift_during_backoff"):
            return {"ok": False, "reason": str(segment.get("reason")),
                    "elapsed_s": time.time() - started, "attempts": attempts, "status": after}
        if drift is None:
            return {"ok": False, "reason": "reverse_heading_unavailable",
                    "elapsed_s": time.time() - started, "attempts": attempts, "status": after}

        if abs(drift) >= math.radians(2.0):
            corrected = guarded_slam_turn(
                turn="right" if drift > 0.0 else "left",
                degrees=max(10.0, abs(math.degrees(drift))),
                max_duration=1.5, tolerance_degrees=5.0, step=20)
            item["correction"] = corrected
            if not corrected.get("ok"):
                return {"ok": False,
                        "reason": "reverse_yaw_correction_failed:" + str(corrected.get("reason")),
                        "elapsed_s": time.time() - started, "attempts": attempts,
                        "status": corrected.get("status") or snapshot()}

    return {"ok": False, "reason": "reverse_clearance_gain_not_reached",
            "elapsed_s": time.time() - started, "attempts": attempts, "status": snapshot()}


def coverage_has_plateaued(samples, now, window_s, min_growth_cells):
    recent = [(float(t), int(c)) for t, c in samples if float(t) >= float(now) - float(window_s)]
    if len(recent) < 2 or recent[-1][0] - recent[0][0] < float(window_s) * 0.90:
        return False
    return max(c for _, c in recent) - min(c for _, c in recent) < int(min_growth_cells)


def lidar_walk(max_duration=60.0, save=True, coverage_goal=False, min_duration=60.0,
               coverage_window=45.0, min_growth_cells=150):
    """Fluent local walk, optionally stopped by guarded-SLAM coverage plateau."""
    global active_explore, last_run
    max_duration = min(max(float(max_duration), 0.0), 600.0 if coverage_goal else 180.0)
    min_duration = min(max(float(min_duration), 0.0), max_duration)
    coverage_window = min(max(float(coverage_window), 10.0), 180.0)
    min_growth_cells = max(1, int(min_growth_cells))
    started = time.time()
    trace = []
    reason = "max_duration"
    current_action = None
    last_escape_action = None
    consecutive_escape_actions = 0
    coverage_samples = deque()
    initial_known_cells = None
    final_known_cells = None
    active_explore = {"active": True, "started_at": started, "name": "coverage_explore" if coverage_goal else "lidar_walk"}
    with motion_lock:
        try:
            initial = snapshot()
            if not scan_ok(initial):
                reason = "scan_stale_or_missing_before_start"
            while reason == "max_duration" and time.time() - started < max_duration:
                if motion_is_cancelled():
                    reason = "motion_cancelled"
                    break
                st = snapshot()
                if not scan_ok(st):
                    reason = "scan_stale_or_missing"
                    break
                if coverage_goal:
                    slam = st.get("slam") or {}
                    pose_age = slam.get("pose_age_s")
                    map_age = slam.get("map_age_s")
                    if (slam.get("pose_valid") is False or pose_age is None or float(pose_age) > 6.0 or
                            map_age is None or float(map_age) > 2.5):
                        reason = "slam_invalid_during_coverage"
                        break
                    known_cells = int(slam.get("known_cells") or 0)
                    if initial_known_cells is None:
                        initial_known_cells = known_cells
                    final_known_cells = known_cells
                    now = time.time()
                    coverage_samples.append((now, known_cells))
                    while coverage_samples and coverage_samples[0][0] < now - coverage_window:
                        coverage_samples.popleft()
                    if (now - started >= min_duration and
                            coverage_has_plateaued(coverage_samples, now, coverage_window, min_growth_cells)):
                        reason = "coverage_plateau"
                        break
                sec = st.get("sectors") or {}
                action, duration, why = choose_explore_action(sec)
                remaining = max_duration - (time.time() - started)
                if action == "stop":
                    stop_burst(3)
                    current_action = None
                    reason = why
                    trace.append({"action": "stop", "why": why, "sectors": sec})
                    break
                if action == "forward":
                    last_escape_action = None
                    consecutive_escape_actions = 0
                    current_action, gait_started = start_or_continue_fluent_forward(current_action, step=20)
                    supervised = supervise_lidar_forward(min(0.50, remaining))
                    trace.append({"action": "forward", "gait_started": gait_started,
                                  "duration": round(float(supervised.get("elapsed_s", 0.0)), 3),
                                  "why": why, "sectors": sec})
                    if not supervised["ok"]:
                        stop_burst(2)
                        current_action = None
                        if supervised["reason"] == "obstacle_during_lidar_walk":
                            trace.append({"action": "stop_reassess", "why": supervised["reason"],
                                          "sectors": snapshot().get("sectors") or {}})
                            time.sleep(0.10)
                            continue
                        reason = str(supervised["reason"])
                        break
                    continue
                if current_action is not None:
                    stop_burst(2)
                    current_action = None
                if action == last_escape_action:
                    consecutive_escape_actions += 1
                else:
                    last_escape_action = action
                    consecutive_escape_actions = 1
                if consecutive_escape_actions > 2:
                    reason = "repeated_escape_action_stop"
                    trace.append({"action": "stop", "why": reason,
                                  "attempted_action": action, "sectors": sec})
                    break
                duration = min(duration, remaining)
                before_pose = pose_copy()
                if action in ("turnleft", "turnright"):
                    if remaining < 1.0:
                        reason = "max_duration"
                        break
                    supervised = guarded_slam_turn(
                        turn="left" if action == "turnleft" else "right",
                        degrees=20.0, max_duration=min(2.0, remaining),
                        step=explore_step_for(action))
                elif action in ("back", "backward"):
                    supervised = yaw_corrected_reverse_escape(sec, before_pose)
                else:
                    motor_send(action, step=explore_step_for(action))
                    supervised = supervise_lidar_motion(
                        action, duration, baseline_sectors=sec, baseline_pose=before_pose,
                        stop_on_front_gain_m=ESCAPE_CLEARANCE_GAIN_M if action in ("back", "backward") else None)
                    stop_burst(2)
                after_state = snapshot()
                after_sectors = after_state.get("sectors") or {}
                after_pose = pose_copy()
                progress = (bool(supervised["ok"]) if action in ("turnleft", "turnright") else
                            bool(supervised["ok"]) and (
                                supervised.get("reason") == "reverse_clearance_gain_reached" or
                                supervised.get("recovery_action") == "turn" or
                                escape_made_progress(action, sec, after_sectors, before_pose, after_pose)))
                trace.append({
                    "action": action,
                    "duration": round(float(supervised.get("elapsed_s", 0.0)), 3),
                    "why": why,
                    "sectors_before": sec,
                    "sectors_after": after_sectors,
                    "progress": progress,
                    "supervisor_reason": supervised["reason"],
                    "recovery_attempts": supervised.get("attempts"),
                })
                if not supervised["ok"]:
                    reason = str(supervised["reason"])
                    break
                if not progress:
                    reason = "escape_no_progress"
                    break
        except Exception as exc:
            reason = "exception:" + repr(exc)
            with state_lock:
                state["last_error"] = repr(exc)
        finally:
            stop_burst(3)
            active_explore = None
    result = {
        "ok": reason in ("max_duration", "coverage_plateau"),
        "mode": "coverage_explore" if coverage_goal else "lidar_walk",
        "localization": "guarded_slam_coverage_with_local_lidar_motion" if coverage_goal else "local_lidar_reactive_no_global_slam",
        "reason": reason,
        "elapsed_s": round(time.time() - started, 2),
        "trace": trace,
        "coverage": {
            "initial_known_cells": initial_known_cells,
            "final_known_cells": final_known_cells,
            "growth_cells": None if initial_known_cells is None or final_known_cells is None else final_known_cells - initial_known_cells,
            "window_s": coverage_window,
            "minimum_growth_cells": min_growth_cells,
        },
        "status": snapshot(),
    }
    last_run = result
    remember("lidar_walk_done", reason=reason, elapsed_s=result["elapsed_s"])
    return result


def mission_snapshot():
    """Return the active mission, or the most recently completed mission."""
    with mission_lock:
        mission = _active_mission or _latest_mission
        if mission is None:
            return None
        value = {key: item for key, item in mission.items() if key not in ("thread", "lease")}
    now = time.time()
    started_at = value.get("started_at")
    finished_at = value.get("finished_at")
    value["elapsed_s"] = None if started_at is None else round((finished_at or now) - float(started_at), 2)
    value["remaining_s"] = None if started_at is None or finished_at is not None else round(max(0.0, float(value["duration_s"]) - (now - float(started_at))), 2)
    with state_lock:
        value["motion"] = {
            "moving": bool(state.get("moving")),
            "last_command": state.get("last_command"),
            "last_command_at": state.get("last_command_at"),
        }
    return value


def _autonomous_mission_worker(mission, lease, options):
    global _active_mission, _latest_mission
    motion_context.lease = lease
    with mission_lock:
        mission["state"] = "running"
        mission["started_at"] = time.time()
    remember("autonomous_mission_started", mission_id=mission["id"], mode=mission["mode"], lease_id=lease.lease_id)
    result = None
    error = None
    try:
        result = lidar_walk(
            max_duration=mission["duration_s"],
            save=bool(options.get("save", False)),
            coverage_goal=mission["mode"] == "coverage",
            min_duration=float(options.get("min_duration", mission["duration_s"])),
            coverage_window=float(options.get("coverage_window", 45.0)),
            min_growth_cells=int(options.get("min_growth_cells", 150)),
        )
    except Exception as exc:
        error = repr(exc)
        remember("autonomous_mission_exception", mission_id=mission["id"], error=error)
    finally:
        try:
            stop_burst(3)
        except Exception as exc:
            error = error or ("final_stop_failed:" + repr(exc))
        end_motion(lease)
        if getattr(motion_context, "lease", None) is lease:
            del motion_context.lease
        with mission_lock:
            mission["finished_at"] = time.time()
            mission["result"] = result
            mission["error"] = error
            if lease.cancel_event.is_set() or (result or {}).get("reason") == "motion_cancelled":
                mission["state"] = "cancelled"
            elif error is not None or not bool((result or {}).get("ok")):
                mission["state"] = "failed"
            else:
                mission["state"] = "completed"
            if _active_mission is mission:
                _active_mission = None
            _latest_mission = mission
        remember("autonomous_mission_finished", mission_id=mission["id"], state=mission["state"],
                 reason=(result or {}).get("reason"), error=error)


def start_autonomous_mission(mode="coverage", duration_s=180.0, **options):
    """Start one fluent navigation mission and return without blocking HTTP."""
    global _active_mission, _mission_sequence
    mode = str(mode).strip().lower()
    if mode not in ("coverage", "local"):
        raise ValueError("mission mode must be coverage or local")
    duration_s = min(max(float(duration_s), 5.0), 600.0 if mode == "coverage" else 180.0)
    initial = snapshot()
    if not scan_ok(initial):
        return {"ok": False, "reason": "scan_stale_or_missing_before_start", "status": initial}
    if mode == "coverage" and not slam_ok(initial):
        return {"ok": False, "reason": "slam_unusable_before_start", "status": initial}
    initial_action, _, initial_reason = choose_explore_action(initial.get("sectors") or {})
    if initial_action == "stop":
        return {"ok": False, "reason": "footprint_clearance_rejected_before_start",
                "safety_reason": initial_reason, "status": initial}
    with mission_lock:
        if _active_mission is not None:
            return {"ok": False, "busy": True, "reason": "autonomous_mission_busy", "mission": mission_snapshot()}
        lease = begin_motion("autonomous_mission", max_duration=duration_s + 2.0)
        if getattr(motion_context, "lease", None) is lease:
            del motion_context.lease
        _mission_sequence += 1
        mission = {
            "id": f"mission-{_mission_sequence}",
            "mode": mode,
            "state": "starting",
            "duration_s": duration_s,
            "created_at": time.time(),
            "started_at": None,
            "finished_at": None,
            "lease_id": lease.lease_id,
            "result": None,
            "error": None,
            "options": dict(options),
        }
        thread = threading.Thread(target=_autonomous_mission_worker, args=(mission, lease, dict(options)),
                                  daemon=True, name=f"beep-{mission['id']}")
        mission["thread"] = thread
        mission["lease"] = lease
        _active_mission = mission
        thread.start()
    return {"ok": True, "accepted": True, "mission": mission_snapshot(), "status": snapshot()}


def cancel_autonomous_mission(mission_id=None, source="mission_cancel"):
    with mission_lock:
        mission = _active_mission
        if mission is None:
            return {"ok": True, "cancelled": False, "reason": "no_active_mission", "mission": mission_snapshot()}
        if mission_id not in (None, "", mission["id"]):
            return {"ok": False, "cancelled": False, "reason": "mission_id_mismatch", "mission": mission_snapshot()}
        lease = active_motion_lease()
        if lease is None or lease.lease_id != mission["lease_id"]:
            return {"ok": True, "cancelled": False, "reason": "mission_finishing", "mission": mission_snapshot()}
    error = request_stop(source)
    return {"ok": error is None, "cancelled": True, "error": error, "mission": mission_snapshot(), "status": snapshot()}


def observer_stop_request(provided_token, mission_id, event_id=None, reason=None, now=None):
    """Authenticated, rate-limited, exact-mission stop capability."""
    global observer_last_stop_at
    if not OBSERVER_STOP_TOKEN:
        return {"ok": False, "reason": "observer_stop_disabled", "status_code": 503}
    if not provided_token or not hmac.compare_digest(str(provided_token), OBSERVER_STOP_TOKEN):
        return {"ok": False, "reason": "observer_auth_rejected", "status_code": 403}
    if not mission_id:
        return {"ok": False, "reason": "mission_id_required", "status_code": 400}
    with mission_lock:
        mission = _active_mission
        if mission is None:
            return {"ok": False, "reason": "no_active_mission", "status_code": 409, "mission": mission_snapshot()}
        if str(mission_id) != str(mission["id"]):
            return {"ok": False, "reason": "mission_id_mismatch", "status_code": 409, "mission": mission_snapshot()}
    observed_now = time.monotonic() if now is None else float(now)
    with observer_stop_lock:
        if observer_last_stop_at is not None and observed_now - observer_last_stop_at < OBSERVER_STOP_MIN_INTERVAL_S:
            return {"ok": False, "reason": "observer_stop_rate_limited", "status_code": 429}
        observer_last_stop_at = observed_now
    result = cancel_autonomous_mission(str(mission_id), source="external_observer")
    result.update({"reason": result.get("reason") or "observer_stop_delivered",
                   "status_code": 200 if result.get("ok") else 409,
                   "event_id": None if event_id is None else str(event_id),
                   "observer_reason": None if reason is None else str(reason)[:240]})
    remember("observer_stop_request", mission_id=str(mission_id), event_id=event_id,
             reason=reason, delivered=bool(result.get("ok")))
    return result


def start_or_continue_fluent_forward(current_action, step):
    """Start the gait once; later planning windows leave it running uninterrupted."""
    if current_action == "forward":
        return "forward", False
    motor_send("forward", step=step)
    return "forward", True


def frontier_explore(name="dog_frontier", max_duration=60.0, chaos=0.45, seed=None, save=True, dry_run=False):
    """Explore reachable occupancy-grid frontiers with a bounded, dog-like motion policy."""
    global active_explore, last_run
    max_duration = min(max(float(max_duration), 0.0), 300.0)
    chaos = min(max(float(chaos), 0.0), 1.0)
    actual_seed = int(seed) if seed is not None else int(time.time_ns() & 0x7fffffff)
    rng = random.Random(actual_seed)
    started = time.time()
    trace = []
    excluded_world = deque(maxlen=16)
    selected_target_world = None
    selected_meta = None
    stall_count = 0
    gait_action = None
    turn_streak_action = None
    turn_streak_count = 0
    turn_streak_start_yaw = None
    reason = "max_duration"
    active_explore = {"active": True, "mode": "frontier_explore", "started_at": started, "name": name, "chaos": chaos, "seed": actual_seed}

    with motion_lock:
        try:
            while time.time() - started < max_duration:
                if motion_is_cancelled():
                    reason = "motion_cancelled"
                    break
                st = snapshot()
                slam = st.get("slam") or {}
                if not scan_ok(st):
                    reason = "scan_stale_or_missing"
                    break
                if not slam_ok(st):
                    reason = "slam_unusable"
                    break
                if slam.get("map_age_s") is None or float(slam["map_age_s"]) > 2.5:
                    reason = "slam_map_unavailable_or_stale"
                    break
                grid = slam_grid_copy()
                if grid is None:
                    reason = "slam_grid_not_received"
                    break

                pose = pose_copy()
                pose_tuple = (float(pose["x"]), float(pose["y"]), float(pose["yaw"]))
                sec = sector_values(st, ("front", "front_left", "front_right", "left", "right", "rear"))
                if sec is None:
                    reason = "lidar_sector_missing"
                    break
                reactive_action, _, reactive_reason = choose_explore_action(sec)
                if reactive_action in ("back", "stop"):
                    reason = "frontier_footprint_clearance_stop:" + reactive_reason
                    break

                path = None
                target_cell = None
                if selected_target_world is not None:
                    if math.hypot(selected_target_world[0] - pose_tuple[0], selected_target_world[1] - pose_tuple[1]) <= 0.28:
                        if gait_action is not None:
                            stop_burst(1)
                            gait_action = None
                        excluded_world.append(selected_target_world)
                        trace.append({"event": "frontier_reached", "target_world": selected_target_world, "pose": pose})
                        selected_target_world = None
                        selected_meta = None
                        stall_count = 0
                    else:
                        target_cell = grid.world_to_cell(*selected_target_world)
                        radius_cells = max(1, int(math.ceil(ROBOT_FOOTPRINT_RADIUS_M / grid.resolution)))
                        blocked = inflate_obstacles(grid, radius_cells)
                        start_cell = nearest_free_cell(grid, grid.world_to_cell(pose_tuple[0], pose_tuple[1]), blocked)
                        if start_cell is not None:
                            path = astar_path(grid, start_cell, target_cell, blocked)
                        if not path:
                            if gait_action is not None:
                                stop_burst(1)
                                gait_action = None
                            excluded_world.append(selected_target_world)
                            trace.append({"event": "frontier_path_lost", "target_world": selected_target_world, "pose": pose})
                            selected_target_world = None
                            selected_meta = None
                            stall_count = 0

                if selected_target_world is None:
                    excluded_cells = [grid.world_to_cell(x, y) for x, y in excluded_world]
                    plan = find_frontier_plan(grid, pose_tuple, rng, chaos=chaos,
                                              robot_radius_m=ROBOT_FOOTPRINT_RADIUS_M,
                                              excluded=excluded_cells)
                    if plan is None:
                        reason = "coverage_complete_or_no_reachable_frontiers"
                        break
                    selected_target_world = tuple(plan["target_world"])
                    turn_streak_action = None
                    turn_streak_count = 0
                    turn_streak_start_yaw = None
                    selected_meta = {
                        "cluster_size": plan["cluster_size"],
                        "frontier_clusters": plan["frontier_clusters"],
                        "reachable_frontiers": plan["reachable_frontiers"],
                        "score": round(float(plan["score"]), 4),
                    }
                    path = plan["path"]
                    target_cell = plan["target_cell"]
                    trace.append({"event": "frontier_selected", "target_world": selected_target_world, **selected_meta, "pose": pose})

                if not path:
                    continue
                lookahead_cells = max(2, int(round(0.32 / grid.resolution)))
                waypoint_cell = path[min(len(path) - 1, lookahead_cells)]
                waypoint_world = grid.cell_to_world(waypoint_cell)
                decision = choose_natural_motion(pose_tuple, waypoint_world, sec, rng, chaos=chaos)
                if reactive_action in ("turnleft", "turnright"):
                    decision = {"action": reactive_action, "duration": 0.30,
                                "why": "reactive_corridor_turn:" + reactive_reason}

                if dry_run:
                    trace.append({
                        "event": "dry_run_plan",
                        "decision": decision,
                        "target_world": selected_target_world,
                        "waypoint_world": [round(waypoint_world[0], 4), round(waypoint_world[1], 4)],
                        "path_cells": len(path),
                        "frontier": selected_meta,
                        "pose": pose,
                        "sectors": sec,
                    })
                    reason = "dry_run_plan_ready"
                    break

                before_pose = pose_tuple
                motion_window = {"ok": True, "reason": "bounded_turn", "elapsed_s": float(decision["duration"])}
                if decision["action"] == "forward":
                    turn_streak_action = None
                    turn_streak_count = 0
                    turn_streak_start_yaw = None
                    gait_action, gait_started = start_or_continue_fluent_forward(gait_action, decision["step"])
                    motion_window = supervise_fluent_forward(float(decision["duration"]))
                    motion_window["gait_started"] = gait_started
                    if not motion_window["ok"]:
                        stop_burst(2)
                        gait_action = None
                else:
                    if gait_action is not None:
                        stop_burst(1)
                        gait_action = None
                    if min(sec.values()) < TURN_SWEEP_M:
                        reason = "turn_sweep_clearance_rejected"
                        break
                    if turn_streak_action == decision["action"]:
                        turn_streak_count += 1
                    else:
                        turn_streak_action = decision["action"]
                        turn_streak_count = 1
                        turn_streak_start_yaw = float(before_pose[2])
                    turn_duration = min(0.30, float(decision["duration"]))
                    decision["duration"] = turn_duration
                    motor_send(decision["action"], step=decision["step"])
                    motion_window = supervise_lidar_motion(decision["action"], turn_duration)
                    if not motion_window["ok"]:
                        reason = str(motion_window["reason"])
                        stop_burst(2)
                        break
                    stop_burst(1)
                    time.sleep(rng.uniform(0.04, 0.10))
                after_pose_dict = pose_copy()
                after_pose = (float(after_pose_dict["x"]), float(after_pose_dict["y"]), float(after_pose_dict["yaw"]))
                after_sectors = (snapshot().get("sectors") or {})
                translated = math.hypot(after_pose[0] - before_pose[0], after_pose[1] - before_pose[1])
                if decision["action"] == "forward" and translated < 0.008:
                    stall_count += 1
                elif decision["action"] == "forward":
                    stall_count = 0

                trace.append({
                    "event": "motion",
                    "action": decision["action"],
                    "step": decision["step"],
                    "duration": decision["duration"],
                    "motion_window": motion_window,
                    "fluent_gait_active": gait_action == "forward",
                    "why": decision["reason"],
                    "heading_error": decision["heading_error"],
                    "translated_m": round(translated, 4),
                    "target_world": selected_target_world,
                    "waypoint_world": [round(waypoint_world[0], 4), round(waypoint_world[1], 4)],
                    "path_cells": len(path),
                    "frontier": selected_meta,
                    "pose": after_pose_dict,
                    "sectors_before": sec,
                    "sectors_after": after_sectors,
                })

                if decision["action"] in ("turnleft", "turnright"):
                    turn_progress = escape_made_progress(
                        decision["action"], sec, after_sectors,
                        {"yaw": before_pose[2]}, {"yaw": after_pose[2]},
                    )
                    trace[-1]["turn_progress"] = turn_progress
                    if not turn_progress:
                        stop_burst(2)
                        gait_action = None
                        reason = "turn_no_progress"
                        break
                    streak_start_yaw = float(after_pose[2] if turn_streak_start_yaw is None else turn_streak_start_yaw)
                    turn_yaw_delta = abs(norm_angle(float(after_pose[2]) - streak_start_yaw))
                    trace[-1]["turn_streak_count"] = turn_streak_count
                    trace[-1]["turn_yaw_delta"] = round(turn_yaw_delta, 4)
                    if ((turn_streak_count >= 4 and turn_yaw_delta < 0.18) or turn_streak_count >= 8):
                        stop_burst(2)
                        gait_action = None
                        reason = "turn_progress_stalled"
                        trace.append({
                            "event": "turn_progress_stalled",
                            "action": decision["action"],
                            "turn_streak_count": turn_streak_count,
                            "turn_yaw_delta": round(turn_yaw_delta, 4),
                            "heading_error": decision.get("heading_error"),
                            "pose": after_pose_dict,
                        })
                        break

                if not motion_window["ok"]:
                    if motion_window["reason"] == "obstacle_during_fluent_forward":
                        continue
                    reason = motion_window["reason"]
                    break

                if stall_count >= 3 and selected_target_world is not None:
                    if gait_action is not None:
                        stop_burst(1)
                        gait_action = None
                    excluded_world.append(selected_target_world)
                    trace.append({"event": "frontier_blacklisted_after_stall", "target_world": selected_target_world})
                    selected_target_world = None
                    selected_meta = None
                    stall_count = 0
        except Exception as exc:
            reason = "exception:" + repr(exc)
            with state_lock:
                state["last_error"] = repr(exc)
        finally:
            stop_burst(3)
            active_explore = None

    result = {
        "ok": reason in ("max_duration", "coverage_complete_or_no_reachable_frontiers", "dry_run_plan_ready"),
        "mode": "frontier_explore",
        "reason": reason,
        "elapsed_s": round(time.time() - started, 2),
        "chaos": chaos,
        "seed": actual_seed,
        "dry_run": bool(dry_run),
        "excluded_frontiers": len(excluded_world),
        "trace_tail": trace[-80:],
        "status": snapshot(),
    }
    saved_path = None
    if save:
        saved_path = save_frontier_trace(name, result)
    result["saved_path"] = saved_path
    last_run = result
    remember("frontier_explore_done", reason=reason, elapsed_s=result["elapsed_s"], saved_path=saved_path, chaos=chaos, seed=actual_seed)
    return result


def forward_until(target_front=FOOTPRINT_FRONT_M, max_duration=8.0, pulse=0.45, stall_window=5, stall_delta=0.03,
                  min_target=FOOTPRINT_FRONT_M, reorient=True):
    """Compatibility endpoint delegated to the full six-sector approach guard."""
    del pulse, stall_window, stall_delta, reorient
    result = forward_continuous_until(
        target_front=max(FOOTPRINT_FRONT_M, float(target_front)),
        max_duration=max_duration,
        min_target=max(FOOTPRINT_FRONT_M, float(min_target)),
    )
    result = dict(result)
    result["mode"] = "forward_until"
    result["controller"] = "forward_continuous_until"
    return result


def forward_continuous_until(target_front=FOOTPRINT_FRONT_M, max_duration=5.0, min_target=FOOTPRINT_FRONT_M,
                             poll_interval=0.05, step=None, max_heading_drift_degrees=15.0):
    """Walk forward continuously with live LiDAR, SLAM and heading guards."""
    global last_run
    min_target = max(FOOTPRINT_FRONT_M, float(min_target))
    target_front = max(float(target_front), min_target)
    max_duration = min(max(float(max_duration), 0.0), FORWARD_UNTIL_MAX_S)
    poll_interval = min(max(float(poll_interval), 0.02), 0.20)
    max_heading_drift = math.radians(max(5.0, min(float(max_heading_drift_degrees), 30.0)))
    started = time.time()
    reason = "max_duration"
    trace = []
    final_state = snapshot()

    with motion_lock:
        if not scan_ok(final_state):
            result = {"ok": False, "mode": "forward_continuous_until",
                      "reason": "scan_stale_or_missing_before_start", "status": final_state, "trace_tail": []}
            last_run = result
            return result
        if not slam_ok(final_state) or (final_state.get("pose") or {}).get("yaw") is None:
            result = {"ok": False, "mode": "forward_continuous_until",
                      "reason": "slam_invalid_before_start", "status": final_state, "trace_tail": []}
            last_run = result
            return result
        start_yaw = float(final_state["pose"]["yaw"])
        initial_sectors = sector_values(final_state, ("front", "front_left", "front_right", "left", "right", "rear"))
        if initial_sectors is None:
            result = {"ok": False, "mode": "forward_continuous_until",
                      "reason": "lidar_sector_missing_before_start", "status": final_state, "trace_tail": []}
            last_run = result
            return result
        initial_front = initial_sectors["front"]
        if (initial_front < FOOTPRINT_FRONT_M or
                min(initial_sectors["front_left"], initial_sectors["front_right"]) < FOOTPRINT_DIAGONAL_M or
                min(initial_sectors["left"], initial_sectors["right"]) < FOOTPRINT_SIDE_M):
            result = {"ok": False, "mode": "forward_continuous_until",
                      "reason": "footprint_clearance_rejected_before_start", "status": final_state, "trace_tail": []}
            last_run = result
            return result
        if initial_front is not None and initial_front <= target_front:
            result = {"ok": True, "mode": "forward_continuous_until",
                      "reason": f"already_at_target:{initial_front:.3f}", "status": final_state, "trace_tail": []}
            last_run = result
            return result

        try:
            motor_send("forward", step=step)
            while time.time() - started < max_duration:
                if motion_is_cancelled():
                    reason = "motion_cancelled"
                    break
                final_state = snapshot()
                elapsed = time.time() - started
                front = front_distance(final_state)
                sectors = final_state.get("sectors") or {}
                pose = final_state.get("pose") or {}
                yaw = pose.get("yaw")
                heading_drift = None if yaw is None else abs(norm_angle(float(yaw) - start_yaw))
                trace.append({"elapsed_s": round(elapsed, 3), "front": front,
                              "front_left": sectors.get("front_left"),
                              "front_right": sectors.get("front_right"),
                              "heading_drift_degrees": None if heading_drift is None else round(math.degrees(heading_drift), 2)})
                if not scan_ok(final_state):
                    reason = "scan_stale_or_missing"
                    break
                if not slam_ok(final_state) or heading_drift is None:
                    reason = "slam_invalid_during_straight_approach"
                    break
                if heading_drift > max_heading_drift:
                    reason = f"heading_drift:{math.degrees(heading_drift):.1f}deg"
                    break
                required = sector_values(final_state, ("front", "front_left", "front_right", "left", "right", "rear"))
                if required is None:
                    reason = "lidar_sector_missing"
                    break
                diagonal = min(required["front_left"], required["front_right"])
                side = min(required["left"], required["right"])
                if required["front"] < FOOTPRINT_FRONT_M or diagonal < FOOTPRINT_DIAGONAL_M or side < FOOTPRINT_SIDE_M:
                    reason = (f"footprint_clearance_breach:front={required['front']:.3f},"
                              f"diagonal={diagonal:.3f},side={side:.3f}")
                    break
                if front is not None and front <= target_front:
                    reason = f"target_reached:{front:.3f}"
                    break
                time.sleep(poll_interval)
        except Exception as e:
            reason = "exception:" + repr(e)
            with state_lock:
                state["last_error"] = repr(e)
        finally:
            stop_burst(3)

    elapsed = time.time() - started
    reached = reason.startswith("target_reached") or reason.startswith("already_at_target")
    result = {"ok": reached, "mode": "forward_continuous_until", "target_front_m": target_front,
              "reason": reason, "commanded_s": round(elapsed, 2), "elapsed_s": round(elapsed, 2),
              "trace_tail": trace[-40:], "status": final_state}
    last_run = result
    remember("forward_continuous_done", reason=reason, commanded_s=round(elapsed, 2), target_front_m=target_front)
    return result


def guarded_slam_turn(turn="left", degrees=90.0, max_duration=MARK_TURN_TIMEOUT_S,
                      tolerance_degrees=5.0, poll_interval=0.05, step=None):
    """Turn through clearance-supervised micro-pivots and settled SLAM measurements."""
    global last_run
    turn = str(turn).lower()
    if turn not in ("left", "right"):
        raise ValueError("turn must be 'left' or 'right'")
    degrees = max(10.0, min(float(degrees), 180.0))
    tolerance_degrees = max(2.0, min(float(tolerance_degrees), 15.0))
    max_duration = max(1.0, min(float(max_duration), 8.0))
    poll_interval = max(0.02, min(float(poll_interval), 0.20))
    target_rad = math.radians(degrees)
    tolerance_rad = math.radians(tolerance_degrees)
    started = time.time()
    reason = "max_duration"
    trace = []
    final_state = snapshot()

    with motion_lock:
        slam = final_state.get("slam") or {}
        pose = final_state.get("pose") or {}
        if motion_is_cancelled():
            return {"ok": False, "mode": "guarded_slam_turn", "reason": "motion_cancelled", "status": final_state, "trace_tail": []}
        if not scan_ok(final_state):
            return {"ok": False, "mode": "guarded_slam_turn", "reason": "scan_stale_or_missing_before_start", "status": final_state, "trace_tail": []}
        if not slam_ok(final_state) or pose.get("yaw") is None:
            return {"ok": False, "mode": "guarded_slam_turn", "reason": "slam_invalid_before_start", "status": final_state, "trace_tail": []}
        required = sector_values(final_state, ("front", "front_left", "front_right", "left", "right", "rear"))
        if required is None:
            return {"ok": False, "mode": "guarded_slam_turn", "reason": "lidar_sector_missing_before_start", "status": final_state, "trace_tail": []}
        if min(required.values()) < TURN_SWEEP_M:
            return {"ok": False, "mode": "guarded_slam_turn", "reason": "clearance_breach_before_start", "status": final_state, "trace_tail": []}
        start_yaw = float(pose["yaw"])
        action = "turnleft" if turn == "left" else "turnright"
        best_progress = 0.0
        stalled_segments = 0
        try:
            while time.time() - started < max_duration:
                if motion_is_cancelled():
                    reason = "motion_cancelled"
                    break
                motor_send(action, step=step)
                segment = supervise_lidar_motion(
                    action, min(0.50, max_duration - (time.time() - started)), poll_s=poll_interval)
                stop_burst(2)
                if not segment.get("ok"):
                    reason = str(segment.get("reason"))
                    break
                time.sleep(0.10)
                final_state = snapshot()
                slam = final_state.get("slam") or {}
                pose = final_state.get("pose") or {}
                sectors = final_state.get("sectors") or {}
                if not scan_ok(final_state):
                    reason = "scan_stale_or_missing"
                    break
                if not slam_ok(final_state) or pose.get("yaw") is None:
                    reason = "slam_invalid_during_turn"
                    break
                required = sector_values(final_state, ("front", "front_left", "front_right", "left", "right", "rear"))
                if required is None:
                    reason = "lidar_sector_missing"
                    break
                nearest = min(required.values())
                if nearest < TURN_SWEEP_M:
                    reason = f"clearance_too_close:{nearest:.3f}"
                    break
                yaw = float(pose["yaw"])
                signed_delta = math.atan2(math.sin(yaw - start_yaw), math.cos(yaw - start_yaw))
                progress = signed_delta if turn == "left" else -signed_delta
                trace.append({"elapsed_s": round(time.time() - started, 3), "yaw": round(yaw, 4),
                              "progress_degrees": round(math.degrees(progress), 2),
                              "nearest_m": round(nearest, 3),
                              "segment_elapsed_s": round(float(segment.get("elapsed_s", 0.0)), 3)})
                if progress >= target_rad - tolerance_rad:
                    reason = f"target_reached:{math.degrees(progress):.1f}deg"
                    break
                if progress < -math.radians(5.0):
                    reason = f"wrong_direction:{math.degrees(progress):.1f}deg"
                    break
                if progress >= best_progress + math.radians(2.0):
                    best_progress = progress
                    stalled_segments = 0
                else:
                    stalled_segments += 1
                    if stalled_segments >= 2:
                        reason = f"turn_no_settled_progress:{math.degrees(progress):.1f}deg"
                        break
        except Exception as e:
            reason = "exception:" + repr(e)
            with state_lock:
                state["last_error"] = repr(e)
        finally:
            stop_burst(3)

    elapsed = time.time() - started
    reached = reason.startswith("target_reached")
    result = {"ok": reached, "mode": "guarded_slam_turn", "turn": turn, "target_degrees": degrees,
              "reason": reason, "elapsed_s": round(elapsed, 2), "trace_tail": trace[-60:], "status": snapshot()}
    last_run = result
    remember("guarded_slam_turn_done", turn=turn, target_degrees=degrees, reason=reason, elapsed_s=round(elapsed, 2))
    return result


def mark_object(target_front=MARK_TARGET_FRONT_M, max_duration=5.0, turn=MARK_TURN_DIRECTION,
                turn_duration=MARK_TURN_TIMEOUT_S, dry_run=False):
    """Approach a target, turn left 90 degrees, then lift the right leg."""
    target_front = max(MARK_MIN_FRONT_M, min(float(target_front), 1.0))
    max_duration = max(0.0, min(float(max_duration), 8.0))
    turn_timeout = max(1.0, min(float(turn_duration), 8.0))
    turn = str(turn).lower()
    if turn not in ("left", "right"):
        raise ValueError("turn must be 'left' or 'right'")
    plan = {"mode": "mark_object", "approach_mode": "continuous", "target_front_m": target_front, "max_duration_s": max_duration,
            "turn": turn, "turn_degrees": MARK_TURN_DEGREES, "turn_control": "guarded_slam_yaw", "turn_timeout_s": turn_timeout,
            "marking_side": "right", "trick": resolve_trick(name="pee")}
    if dry_run:
        remember("mark_object_dry_run", plan=plan)
        return {"ok": True, "dry_run": True, "plan": plan, "status": snapshot()}
    steps = []
    initial = snapshot()
    initial_slam = initial.get("slam") or {}
    if not slam_ok(initial):
        result = {"ok": False, "mode": "mark_object", "reason": "slam_invalid_before_approach", "steps": steps, "status": initial}
        remember("mark_object_abort", reason=result["reason"])
        return result
    approach = forward_continuous_until(target_front=target_front, max_duration=max_duration, min_target=MARK_MIN_FRONT_M)
    steps.append({"step": "approach", "result": approach})
    snap = snapshot()
    required = sector_values(snap, ("front", "front_left", "front_right", "left", "right", "rear"))
    if required is None:
        result = {"ok": False, "mode": "mark_object", "reason": "lidar_sector_missing_after_approach", "steps": steps, "status": snap}
        remember("mark_object_abort", reason=result["reason"])
        return result
    front = required["front"]
    diagonal_clearance = min(required[name] for name in ("front_left", "front_right"))
    side_clearance = min(required[name] for name in ("left", "right"))
    if motion_is_cancelled():
        stop_burst(3)
        result = {"ok": False, "mode": "mark_object", "reason": "motion_cancelled", "steps": steps, "status": snap}
        remember("mark_object_abort", reason=result["reason"], front=front)
        return result
    if (not approach.get("ok") or front is None or front < MARK_MIN_FRONT_M or
            diagonal_clearance < FOOTPRINT_DIAGONAL_M or side_clearance < FOOTPRINT_SIDE_M):
        stop_burst(3)
        if not approach.get("ok"):
            reason = "approach_failed:" + str(approach.get("reason"))
        elif diagonal_clearance < FOOTPRINT_DIAGONAL_M or side_clearance < FOOTPRINT_SIDE_M:
            reason = f"unsafe_footprint_clearance:diagonal={diagonal_clearance:.3f},side={side_clearance:.3f}"
        else:
            reason = "unsafe_after_approach"
        result = {"ok": False, "mode": "mark_object", "reason": reason, "steps": steps, "status": snap}
        remember("mark_object_abort", reason=result["reason"], front=front, side_clearance=side_clearance)
        return result
    turn_result = guarded_slam_turn(turn=turn, degrees=MARK_TURN_DEGREES, max_duration=turn_timeout)
    steps.append({"step": "turn", "result": turn_result})
    if not turn_result.get("ok") or motion_is_cancelled():
        stop_burst(3)
        result = {"ok": False, "mode": "mark_object", "reason": "motion_cancelled" if motion_is_cancelled() else "turn_failed:" + str(turn_result.get("reason")), "steps": steps, "status": snapshot()}
        remember("mark_object_abort", reason=result["reason"])
        return result
    pee_result = sdk_trick(name="pee")
    steps.append({"step": "pee", "result": pee_result})
    reset_result = sdk_trick(name="reset") if pee_result.get("ok") and not motion_is_cancelled() else {"ok": False, "reason": "skipped"}
    steps.append({"step": "reset", "result": reset_result})
    ok = bool(pee_result.get("ok") and reset_result.get("ok") and not motion_is_cancelled())
    result = {"ok": ok, "mode": "mark_object", "reason": None if ok else "gesture_failed_or_cancelled", "steps": steps, "status": snapshot()}
    remember("mark_object_done", ok=ok)
    return result


def handler_failure_requires_stop(path: str, error: BaseException) -> bool:
    """Keep observation and response-transport failures out of motor control."""
    if isinstance(error, (BrokenPipeError, ConnectionResetError)):
        return False
    return path not in {"/frame.jpg", "/camera.jpg"}


class Handler(BaseHTTPRequestHandler):
    server_version = "BEEPBridge/0.5"

    def log_message(self, fmt, *args):
        sys.stderr.write("%s - - [%s] %s\n" % (self.address_string(), self.log_date_time_string(), fmt % args))

    def read_json(self):
        n = int(self.headers.get("content-length", "0") or 0)
        if n <= 0:
            return {}
        return json.loads(self.rfile.read(n).decode("utf-8"))

    def send_json(self, obj, code=200):
        data = json.dumps(obj, sort_keys=True, indent=2).encode("utf-8")
        self.send_response(code)
        self.send_header("content-type", "application/json")
        self.send_header("content-length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_bytes(self, data: bytes, content_type="application/octet-stream", code=200):
        self.send_response(code)
        self.send_header("content-type", content_type)
        self.send_header("content-length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        p = urlparse(self.path)
        qs = parse_qs(p.query)
        try:
            if p.path == "/health":
                self.send_json({"ok": True, "name": "beep_bridge", "status": snapshot()})
            elif p.path == "/status":
                self.send_json(snapshot(full=(qs.get("full") or ["0"])[0] == "1"))
            elif p.path == "/last_run":
                self.send_json({"last_run": last_run})
            elif p.path == "/mission":
                self.send_json({"mission": mission_snapshot(), "status": snapshot()})
            elif p.path == "/config":
                self.send_json({"version": state["version"], "motor_backend": MOTOR_BACKEND, "sdk_step_default": SDK_STEP_DEFAULT, "sdk_gait": SDK_GAIT, "sdk_pace": SDK_PACE,
                                "app_host": HOST, "app_port": APP_PORT, "camera_url": CAMERA_URL,
                                "http_port": HTTP_PORT, "max_move_s": MAX_MOVE_S, "forward_until_max_s": FORWARD_UNTIL_MAX_S,
                                "front_stop_m": FRONT_STOP_M, "side_stop_m": SIDE_STOP_M, "hard_clearance_m": HARD_CLEARANCE_M,
                                "footprint_clearance_m": {"front": FOOTPRINT_FRONT_M, "front_diagonal": FOOTPRINT_DIAGONAL_M,
                                                           "side": FOOTPRINT_SIDE_M, "rear": FOOTPRINT_REAR_M,
                                                           "forward_corridor": FORWARD_CORRIDOR_M, "turn_sweep": TURN_SWEEP_M,
                                                           "turn_start_clearance": TURN_START_CLEARANCE_M,
                                                           "map_radius": ROBOT_FOOTPRINT_RADIUS_M,
                                                           "escape_gain": ESCAPE_CLEARANCE_GAIN_M,
                                                           "reverse_escape_max_s": REVERSE_ESCAPE_MAX_S,
                                                           "reverse_correction_segment_s": REVERSE_CORRECTION_SEGMENT_S,
                                                           "reverse_correction_attempts": REVERSE_CORRECTION_ATTEMPTS,
                                                           "reverse_correction_step": REVERSE_CORRECTION_STEP,
                                                           "backoff_max_front_loss": BACKOFF_MAX_FRONT_LOSS_M,
                                                           "backoff_max_heading_drift_deg": math.degrees(BACKOFF_MAX_HEADING_DRIFT_RAD),
                                                           "turn_progress_window_s": TURN_PROGRESS_WINDOW_S,
                                                           "turn_progress_min_deg": math.degrees(TURN_PROGRESS_MIN_RAD)},
                                "scan_stale_s": SCAN_STALE_S, "sdk_error": sdk_error, "map_dir": str(MAP_DIR), "map_resolution_m": MAP_RES_M, "map_size_m": MAP_SIZE_M, "explore_safe_front_m": EXPLORE_SAFE_FRONT_M, "explore_safe_side_m": EXPLORE_SAFE_SIDE_M, "pose_mode": "guarded_cartographer_slam", "local_map": True,
                                "tricks": {"count": len(TRICK_ACTIONS), "endpoint": "/actions", "settle_s_default": TRICK_SETTLE_S},
                                "motion_ownership": "exclusive_per_run_nonqueueing", "busy_status": 409,
                                "autonomous_mission": {"start": "POST /mission/start", "status": "GET /mission", "cancel": "POST /mission/cancel", "modes": ["coverage", "local"]},
                                "external_observer": {"stop": "POST /observer/stop", "enabled": bool(OBSERVER_STOP_TOKEN),
                                                      "exact_mission_required": True, "min_interval_s": OBSERVER_STOP_MIN_INTERVAL_S},
                                "slam_usable_requires": ["fresh_guarded_pose", "fresh_nonempty_occupancy_map"]})
            elif p.path == "/events":
                self.send_json({"events": list(events)[-80:]})
            elif p.path in ("/actions", "/tricks"):
                self.send_json(tricks_payload())
            elif p.path == "/scan":
                snap = snapshot()
                self.send_json({"sectors": snap.get("sectors", {}), "scan_age_s": snap.get("scan_age_s"), "scan_count": snap.get("scan_count")})
            elif p.path == "/local_map":
                self.send_json({"summary": local_map_summary(), "note": "robot-frame local map; safe for immediate obstacle decisions"})
            elif p.path == "/local_map.svg":
                self.send_bytes(local_map_svg(), "image/svg+xml")
            elif p.path == "/map":
                self.send_json({"summary": map_summary(room_map), "map": room_map if (qs.get("full") or ["0"])[0] == "1" else None})
            elif p.path == "/map.svg":
                self.send_bytes(map_svg(room_map), "image/svg+xml")
            elif p.path == "/pose":
                self.send_json({"pose": pose_copy()})
            elif p.path == "/map_reset":
                name = (qs.get("name") or ["room"])[0]
                reset_pose()
                ensure_room_map(name=name, reset=True)
                update_map_from_scan(room_map)
                self.send_json({"ok": True, "summary": map_summary(room_map)})
            elif p.path == "/observe":
                snap = snapshot()
                self.send_json({"status": snap, "map": map_summary(room_map), "camera": {"url": "/frame.jpg", "last_frame_at": snap.get("last_frame_at"), "last_frame_bytes": snap.get("last_frame_bytes")}})
            elif p.path in ("/frame.jpg", "/camera.jpg"):
                jpg = capture_frame(timeout=float((qs.get("timeout") or ["4"])[0]))
                self.send_bytes(jpg, "image/jpeg")
            elif p.path == "/stop":
                err = request_stop("http_get")
                self.send_json({"ok": err is None, "error": err, "status": snapshot()})
            elif p.path == "/move":
                action = (qs.get("action") or ["stop"])[0]
                duration = float((qs.get("duration") or ["0.2"])[0])
                step = (qs.get("step") or [None])[0]
                if str(action).lower() == "stop":
                    self.send_json(run_action(action, duration, step=step))
                else:
                    self.send_json(run_owned_motion("http_get_move", lambda: run_action(action, duration, step=step), max_duration=duration + 1.0))
            elif p.path == "/supervised_move":
                action = (qs.get("action") or ["backward"])[0]
                duration = float((qs.get("duration") or ["0.8"])[0])
                self.send_json(run_owned_motion("http_get_supervised_move", lambda: run_supervised_calibration(action, duration), max_duration=min(duration, 1.2) + 1.0))
            elif p.path in ("/action", "/trick"):
                name = (qs.get("name") or qs.get("action") or qs.get("trick") or [None])[0]
                action_id = (qs.get("id") or qs.get("action_id") or [None])[0]
                dry = truthy((qs.get("dry_run") or qs.get("dry") or ["0"])[0])
                settle = (qs.get("settle_s") or qs.get("duration") or [None])[0]
                async_requested = truthy((qs.get("async") or ["0"])[0]) or not truthy((qs.get("wait") or ["1"])[0])
                if async_requested and not dry:
                    self.send_json({"ok": False, "error": "asynchronous gestures are disabled; use a synchronous owned motion lease"}, 400)
                elif dry:
                    self.send_json(sdk_trick(name=name, action_id=action_id, dry_run=True, settle_s=settle))
                else:
                    settle_limit = 8.0 if settle is None else min(8.0, max(0.0, float(settle)))
                    self.send_json(run_owned_motion("http_get_trick", lambda: sdk_trick(name=name, action_id=action_id, dry_run=False, settle_s=settle), max_duration=settle_limit + 2.0))
            elif p.path == "/mark_object":
                dry = (qs.get("dry_run") or qs.get("dry") or ["0"])[0] in ("1", "true", "yes")
                target_front = float((qs.get("target_front") or [str(MARK_TARGET_FRONT_M)])[0])
                approach_s = float((qs.get("max_duration") or ["5.0"])[0])
                turn = (qs.get("turn") or [MARK_TURN_DIRECTION])[0]
                turn_timeout = float((qs.get("turn_duration") or [str(MARK_TURN_TIMEOUT_S)])[0])
                callback = lambda: mark_object(target_front=target_front, max_duration=approach_s, turn=turn, turn_duration=turn_timeout, dry_run=dry)
                if dry:
                    self.send_json(callback())
                else:
                    self.send_json(run_owned_motion("http_get_mark_object", callback, max_duration=approach_s + turn_timeout + 12.0))
            elif p.path in ("/slam_turn", "/guarded_turn"):
                dry = truthy((qs.get("dry_run") or qs.get("dry") or ["0"])[0])
                turn = (qs.get("turn") or ["left"])[0]
                degrees = float((qs.get("degrees") or ["90"])[0])
                timeout_s = float((qs.get("max_duration") or [str(MARK_TURN_TIMEOUT_S)])[0])
                if dry:
                    self.send_json({"ok": True, "dry_run": True, "plan": {"mode": "guarded_slam_turn", "turn": turn, "degrees": degrees, "max_duration_s": timeout_s}, "status": snapshot()})
                else:
                    self.send_json(run_owned_motion("http_get_guarded_slam_turn", lambda: guarded_slam_turn(turn=turn, degrees=degrees, max_duration=timeout_s), max_duration=timeout_s + 1.0))
            elif p.path in ("/forward_until", "/learned_forward", "/approach_front"):
                duration = float((qs.get("max_duration") or ["8.0"])[0])
                callback = lambda: forward_until(
                    target_front=float((qs.get("target_front") or qs.get("target") or ["0.10"])[0]), max_duration=duration,
                    pulse=float((qs.get("pulse") or ["0.45"])[0]), stall_window=int((qs.get("stall_window") or ["5"])[0]),
                    stall_delta=float((qs.get("stall_delta") or ["0.03"])[0]), reorient=(qs.get("reorient") or ["1"])[0] != "0")
                self.send_json(run_owned_motion("http_get_forward_until", callback, max_duration=duration + 2.0))
            elif p.path in ("/explore_room", "/explore"):
                duration = float((qs.get("max_duration") or ["30"])[0])
                callback = lambda: explore_room(name=(qs.get("name") or ["room"])[0], max_duration=duration,
                    reset_map=(qs.get("reset") or ["0"])[0] == "1", save=(qs.get("save") or ["1"])[0] != "0",
                    rotate_scan=(qs.get("rotate_scan") or ["1"])[0] != "0")
                self.send_json(run_owned_motion("http_get_explore_room", callback, max_duration=duration + 5.0))
            elif p.path in ("/lidar_walk", "/demo_walk"):
                duration = float((qs.get("max_duration") or ["60"])[0])
                callback = lambda: lidar_walk(max_duration=duration, save=(qs.get("save") or ["1"])[0] != "0")
                self.send_json(run_owned_motion("http_get_lidar_walk", callback, max_duration=duration + 2.0))
            elif p.path in ("/coverage_explore", "/explore_coverage"):
                duration = float((qs.get("max_duration") or ["600"])[0])
                callback = lambda: lidar_walk(max_duration=duration, save=(qs.get("save") or ["1"])[0] != "0", coverage_goal=True,
                    min_duration=float((qs.get("min_duration") or ["60"])[0]), coverage_window=float((qs.get("coverage_window") or ["45"])[0]),
                    min_growth_cells=int((qs.get("min_growth_cells") or ["150"])[0]))
                self.send_json(run_owned_motion("http_get_coverage_explore", callback, max_duration=min(duration + 2.0, 600.0)))
            elif p.path in ("/frontier_explore", "/dog_explore", "/explore_frontiers"):
                seed_value = (qs.get("seed") or [None])[0]
                dry = truthy((qs.get("dry_run") or qs.get("dry") or ["0"])[0])
                duration = float((qs.get("max_duration") or ["60"])[0])
                callback = lambda: frontier_explore(name=(qs.get("name") or ["dog_frontier"])[0], max_duration=duration,
                    chaos=float((qs.get("chaos") or ["0.45"])[0]), seed=None if seed_value in (None, "") else int(seed_value),
                    save=(qs.get("save") or ["1"])[0] != "0", dry_run=dry)
                self.send_json(callback() if dry else run_owned_motion("http_get_frontier_explore", callback, max_duration=duration + 2.0))
            else:
                self.send_json({"error": "not found", "paths": ["/health", "/status", "/mission", "/last_run", "/config", "/events", "/scan", "/observe", "/frame.jpg", "/stop", "/move", "/actions", "/action", "/mark_object", "/slam_turn", "/forward_until", "/explore_room", "/lidar_walk", "/frontier_explore", "/coverage_explore", "/map", "/map.svg", "/local_map", "/local_map.svg", "/pose"]}, 404)
        except MotionBusy as e:
            lease = active_motion_lease()
            self.send_json({"ok": False, "error": str(e), "busy": True,
                            "active_lease": None if lease is None else {"id": lease.lease_id, "source": lease.source}}, 409)
        except Exception as e:
            with state_lock:
                state["last_error"] = repr(e)
            stop_required = handler_failure_requires_stop(p.path, e)
            remember("handler_exception", path=p.path, error=repr(e), stop_required=stop_required)
            if stop_required:
                try:
                    request_stop("handler_exception")
                except Exception:
                    pass
            try:
                self.send_json({"ok": False, "error": repr(e), "status": snapshot()}, 500)
            except (BrokenPipeError, ConnectionResetError):
                pass

    def do_POST(self):
        p = urlparse(self.path)
        try:
            body = self.read_json()
            if p.path == "/stop":
                err = request_stop("http_post")
                self.send_json({"ok": err is None, "error": err, "status": snapshot()})
            elif p.path == "/observer/stop":
                authorization = self.headers.get("Authorization", "")
                provided_token = authorization[7:] if authorization.startswith("Bearer ") else ""
                result = observer_stop_request(provided_token, body.get("mission_id"),
                                               event_id=body.get("event_id"), reason=body.get("reason"))
                self.send_json(result, int(result.get("status_code", 500)))
            elif p.path == "/mission/start":
                result = start_autonomous_mission(
                    mode=body.get("mode", "coverage"),
                    duration_s=body.get("duration_s", body.get("duration", 180.0)),
                    save=bool(body.get("save", False)),
                    min_duration=body.get("min_duration", body.get("duration_s", body.get("duration", 180.0))),
                    coverage_window=body.get("coverage_window", 45.0),
                    min_growth_cells=body.get("min_growth_cells", 150),
                )
                self.send_json(result, 202 if result.get("accepted") else (409 if result.get("busy") else 412))
            elif p.path == "/mission/cancel":
                result = cancel_autonomous_mission(body.get("mission_id"), source="http_mission_cancel")
                self.send_json(result, 200 if result.get("ok") else 409)
            elif p.path == "/move":
                action = body.get("action", "stop")
                duration = float(body.get("duration", 0.2))
                callback = lambda: run_action(action, duration, step=body.get("step"))
                self.send_json(callback() if str(action).lower() == "stop" else run_owned_motion("http_post_move", callback, max_duration=duration + 1.0))
            elif p.path in ("/action", "/trick"):
                async_requested = bool(body.get("async", False)) or body.get("wait", True) is False
                dry = bool(body.get("dry_run", body.get("dry", False)))
                settle = body.get("settle_s", body.get("duration"))
                callback = lambda: sdk_trick(name=body.get("name", body.get("action", body.get("trick"))),
                    action_id=body.get("id", body.get("action_id")), dry_run=dry, settle_s=settle)
                if async_requested and not dry:
                    self.send_json({"ok": False, "error": "asynchronous gestures are disabled; use a synchronous owned motion lease"}, 400)
                elif dry:
                    self.send_json(callback())
                else:
                    settle_limit = 8.0 if settle is None else min(8.0, max(0.0, float(settle)))
                    self.send_json(run_owned_motion("http_post_trick", callback, max_duration=settle_limit + 2.0))
            elif p.path == "/mark_object":
                dry = bool(body.get("dry_run", body.get("dry", False)))
                approach_s = float(body.get("max_duration", 5.0))
                turn_timeout = float(body.get("turn_duration", MARK_TURN_TIMEOUT_S))
                callback = lambda: mark_object(target_front=float(body.get("target_front", MARK_TARGET_FRONT_M)), max_duration=approach_s,
                    turn=body.get("turn", MARK_TURN_DIRECTION), turn_duration=turn_timeout, dry_run=dry)
                self.send_json(callback() if dry else run_owned_motion("http_post_mark_object", callback, max_duration=approach_s + turn_timeout + 12.0))
            elif p.path in ("/slam_turn", "/guarded_turn"):
                dry = bool(body.get("dry_run", body.get("dry", False)))
                turn = body.get("turn", "left")
                degrees = float(body.get("degrees", 90.0))
                timeout_s = float(body.get("max_duration", MARK_TURN_TIMEOUT_S))
                callback = lambda: guarded_slam_turn(turn=turn, degrees=degrees, max_duration=timeout_s)
                if dry:
                    self.send_json({"ok": True, "dry_run": True, "plan": {"mode": "guarded_slam_turn", "turn": turn, "degrees": degrees, "max_duration_s": timeout_s}, "status": snapshot()})
                else:
                    self.send_json(run_owned_motion("http_post_guarded_slam_turn", callback, max_duration=timeout_s + 1.0))
            elif p.path in ("/forward_until", "/learned_forward", "/approach_front"):
                duration = float(body.get("max_duration", 8.0))
                callback = lambda: forward_until(target_front=float(body.get("target_front", body.get("target", 0.10))), max_duration=duration,
                    pulse=float(body.get("pulse", 0.45)), stall_window=int(body.get("stall_window", 5)),
                    stall_delta=float(body.get("stall_delta", 0.03)), min_target=float(body.get("min_target", 0.08)), reorient=bool(body.get("reorient", True)))
                self.send_json(run_owned_motion("http_post_forward_until", callback, max_duration=duration + 2.0))
            elif p.path in ("/explore_room", "/explore"):
                duration = float(body.get("max_duration", 30.0))
                callback = lambda: explore_room(name=body.get("name", "room"), max_duration=duration, reset_map=bool(body.get("reset", False)),
                    save=bool(body.get("save", True)), rotate_scan=bool(body.get("rotate_scan", True)))
                self.send_json(run_owned_motion("http_post_explore_room", callback, max_duration=duration + 5.0))
            elif p.path in ("/frontier_explore", "/dog_explore", "/explore_frontiers"):
                dry = bool(body.get("dry_run", body.get("dry", False)))
                duration = float(body.get("max_duration", 60.0))
                callback = lambda: frontier_explore(name=body.get("name", "dog_frontier"), max_duration=duration,
                    chaos=float(body.get("chaos", 0.45)), seed=body.get("seed"), save=bool(body.get("save", True)), dry_run=dry)
                self.send_json(callback() if dry else run_owned_motion("http_post_frontier_explore", callback, max_duration=duration + 2.0))
            elif p.path == "/map_reset":
                reset_pose(float(body.get("x", 0.0)), float(body.get("y", 0.0)), float(body.get("yaw", 0.0)))
                ensure_room_map(name=body.get("name", "room"), reset=True)
                update_map_from_scan(room_map)
                self.send_json({"ok": True, "summary": map_summary(room_map)})
            else:
                self.send_json({"error": "not found"}, 404)
        except MotionBusy as e:
            lease = active_motion_lease()
            self.send_json({"ok": False, "error": str(e), "busy": True,
                            "active_lease": None if lease is None else {"id": lease.lease_id, "source": lease.source}}, 409)
        except Exception as e:
            with state_lock:
                state["last_error"] = repr(e)
            stop_required = handler_failure_requires_stop(p.path, e)
            remember("handler_exception", path=p.path, error=repr(e), stop_required=stop_required)
            if stop_required:
                try:
                    request_stop("handler_exception")
                except Exception:
                    pass
            try:
                self.send_json({"ok": False, "error": repr(e), "status": snapshot()}, 500)
            except (BrokenPipeError, ConnectionResetError):
                pass


def main():
    MAP_DIR.mkdir(parents=True, exist_ok=True)
    threading.Thread(target=ros_thread, daemon=True).start()
    stop_burst(1)
    httpd = ThreadingHTTPServer((HTTP_BIND, HTTP_PORT), Handler)
    try:
        sdk_init()
    except Exception:
        pass
    print(f"BEEP bridge {state['version']} listening on {HTTP_BIND}:{HTTP_PORT}, backend={MOTOR_BACKEND}, app={HOST}:{APP_PORT}, camera={CAMERA_URL}", flush=True)
    httpd.serve_forever()


if __name__ == "__main__":
    main()
