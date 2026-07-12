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

import json
import math
import os
import socket
import sys
import threading
import time
import urllib.request
from pathlib import Path
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

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

CMD_PAYLOAD = {
    "stop": 0x00,
    "forward": 0x01,
    "back": 0x02,
    "backward": 0x02,
    "left": 0x05,
    "right": 0x06,
}

SDK_STEP_DEFAULT = int(os.environ.get("BEEP_SDK_STEP", "10"))
SDK_GAIT = os.environ.get("BEEP_SDK_GAIT", "walk")
SDK_PACE = os.environ.get("BEEP_SDK_PACE", "slow")
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
TRICK_ACTIONS = {
    "reset": {"id": 255, "label": "Reset / neutral pose", "duration_s": 0.5, "safe_for_fair": True, "aliases": ["neutral", "stand", "home"]},
    "crawl": {"id": 3, "label": "Crawl", "duration_s": 3.0, "safe_for_fair": False, "aliases": ["creep"]},
    "three_axis": {"id": 10, "label": "3-axis body motion", "duration_s": 3.0, "safe_for_fair": True, "aliases": ["3axis", "axis", "body_demo"]},
    "pee": {"id": 11, "label": "Lift leg / pee", "duration_s": 3.5, "safe_for_fair": True, "aliases": ["leg_lift", "mark", "urinate"]},
    "stretch": {"id": 14, "label": "Stretch", "duration_s": 3.0, "safe_for_fair": True, "aliases": ["show", "startup_show", "lazy"]},
    "swing": {"id": 16, "label": "Swing", "duration_s": 3.0, "safe_for_fair": True, "aliases": ["shake", "wobble"]},
    "pray": {"id": 17, "label": "Pray / beg", "duration_s": 3.0, "safe_for_fair": True, "aliases": ["beg", "begging", "request_food"]},
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
events = deque(maxlen=300)
last_run = None
state = {
    "version": "0.8.0-tricks",
    "started_at": time.time(),
    "last_command": None,
    "last_command_at": None,
    "last_error": None,
    "scan_seen": False,
    "scan_at": None,
    "scan_count": 0,
    "sectors": {},
    "moving": False,
    "last_frame_at": None,
    "last_frame_bytes": None,
    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0, "source": "dead_reckoning", "confidence": 0.25, "scan_match_score": None},
    "map": {"active": False, "updated_at": None, "scan_updates": 0, "occupied_cells": 0, "free_cells": 0, "quality": "none"},
}
last_scan = None
room_map = None
active_explore = None
scan_match_ref = None


def remember(event, **data):
    item = {"t": round(time.time(), 3), "event": event, **data}
    with state_lock:
        events.append(item)
    return item


def pkt(cmd: int, payload=()):
    vals = [0x01, cmd, 2 * (len(payload) + 1), *payload]
    vals.append(sum(vals) % 256)
    return ("$" + "".join(f"{v & 255:02X}" for v in vals) + "#").encode()


def app_send(action: str):
    action = str(action).lower()
    if action not in CMD_PAYLOAD:
        raise ValueError(f"unknown app action {action!r}; use {sorted(CMD_PAYLOAD)}")
    with socket.create_connection((HOST, APP_PORT), timeout=1.2) as s:
        s.settimeout(0.25)
        try:
            s.recv(256)
        except Exception:
            pass
        # Standard/control mode, then command.
        s.sendall(pkt(0x0F, [0x01]))
        time.sleep(0.025)
        s.sendall(pkt(0x12, [CMD_PAYLOAD[action]]))
    with state_lock:
        state["last_command"] = "app:" + action
        state["last_command_at"] = time.time()
        state["moving"] = action != "stop"
    remember("app_send", action=action)


def sdk_init():
    global sdk_dog, sdk_error
    if sdk_dog is not None:
        return sdk_dog
    try:
        import DOGZILLALib as dog
        sdk_dog = dog.DOGZILLA()
        try:
            sdk_dog.gait_type(SDK_GAIT)
        except Exception:
            pass
        try:
            sdk_dog.pace(SDK_PACE)
        except Exception:
            pass
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
        g.stop()
    elif action in ("forward", "back", "left", "right", "turnleft", "turnright"):
        getattr(g, action)(step)
    else:
        raise ValueError(f"unknown sdk action {action!r}")
    with state_lock:
        state["last_command"] = "sdk:" + action
        state["last_command_at"] = time.time()
        state["moving"] = action != "stop"
    remember("sdk_send", action=action, step=step)


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
        g = sdk_init()
        with state_lock:
            state["last_command"] = "sdk_action:" + trick["name"]
            state["last_command_at"] = time.time()
            state["moving"] = True
            state["last_error"] = None
        remember("sdk_trick_start", trick=trick)
        g.action(int(trick["id"]))
        if settle_s:
            time.sleep(settle_s)
        with state_lock:
            state["moving"] = False
        remember("sdk_trick_done", trick=trick, settle_s=settle_s)
    return {"ok": True, "dry_run": False, "trick": trick, "settle_s": settle_s}


def motor_send(action: str, step=None):
    if MOTOR_BACKEND == "sdk":
        return sdk_send(action, step=step)
    return app_send(action)


def stop_burst(n=3):
    err = None
    for _ in range(n):
        try:
            # Stop both layers: SDK for actual motors, app for camera/control state.
            try:
                sdk_send("stop")
            except Exception as e:
                err = repr(e)
            try:
                app_send("stop")
            except Exception as e:
                err = repr(e)
        except Exception as e:
            err = repr(e)
        time.sleep(0.06)
    with state_lock:
        state["moving"] = False
        if err:
            state["last_error"] = err
    remember("stop_burst", n=n, error=err)
    return err


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


def ros_thread():
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import LaserScan
    except Exception as e:
        with state_lock:
            state["last_error"] = "rclpy import failed: " + repr(e)
        return

    class ScanNode(Node):
        def __init__(self):
            super().__init__("beep_bridge_scan")
            self.create_subscription(LaserScan, "/scan", scan_callback, 10)

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


def run_action(action: str, duration: float, step=None):
    duration = min(max(float(duration), 0.0), MAX_MOVE_S)
    with motion_lock:
        motor_send(action, step=step)
        if duration > 0 and str(action).lower() != "stop":
            time.sleep(duration)
            stop_burst()
            update_pose_for_action(action, duration)
            update_map_from_scan(room_map) if room_map is not None else None
        return {"ok": True, "backend": MOTOR_BACKEND, "action": action, "step": SDK_STEP_DEFAULT if step is None else int(step), "duration": duration, "pose": pose_copy(), "map": map_summary(room_map) if room_map is not None else None, "status": snapshot()}


def capture_frame(timeout=4.0, max_bytes=1_500_000):
    """Return one JPEG from the MJPEG stream.

    The Yahboom video server may connect but emit no bytes unless the app
    control state is Standard/Fullscreen. Keeping the app socket open while
    reading the first MJPEG frame is more reliable than a one-shot command.
    """
    timeout = float(timeout)
    deadline = time.time() + timeout
    ctrl = None
    try:
        ctrl = socket.create_connection((HOST, APP_PORT), timeout=1.2)
        ctrl.settimeout(0.25)
        try:
            ctrl.recv(256)
        except Exception:
            pass
        ctrl.sendall(pkt(0x0F, [0x01]))  # Standard/control mode
        time.sleep(0.05)
        ctrl.sendall(pkt(0x12, [CMD_PAYLOAD["stop"]]))
        time.sleep(0.15)
    except Exception as e:
        remember("camera_standard_failed", error=repr(e))

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
                    remember("frame", bytes=len(jpg))
                    return jpg
    finally:
        if ctrl is not None:
            try:
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
        "notes": "Dead-reckoned LiDAR occupancy map. Good enough to avoid obvious doom; not loop-closed SLAM.",
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

def update_map_from_scan(m=None):
    global room_map
    if m is None:
        m = ensure_room_map()
    with state_lock:
        scan = dict(last_scan) if last_scan else None
        pose = dict(state.get("pose") or {})
    if not scan or not scan.get("ranges"):
        return {"ok": False, "reason": "no_scan"}
    match = maybe_scan_match_pose()
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


def choose_explore_action(sectors):
    front = sectors.get("front") or 0.0
    fl = sectors.get("front_left") or 0.0
    fr = sectors.get("front_right") or 0.0
    left = sectors.get("left") or 0.0
    right = sectors.get("right") or 0.0
    # BEEP has shown left-biased/stiff forward motion. Prefer lateral escape when
    # the left/front-left sector tightens instead of turning in place repeatedly.
    if (fl < 0.32 or left < 0.28) and right > 0.42:
        return "right", 0.35, "left_close_strafe_right"
    if (fr < 0.32 or right < 0.28) and left > 0.42:
        return "left", 0.35, "right_close_strafe_left"
    if front > max(EXPLORE_SAFE_FRONT_M, 0.50) and fl > 0.34 and fr > 0.34:
        return "forward", 0.35, "soft_front_clear"
    # If forward is blocked but one side is open, use short turns only. DogZilla
    # SDK clamps turn step to at least 30, so duration must stay short.
    if max(left, fl) >= max(right, fr):
        return "turnleft", 0.30, "left_more_open_short_turn"
    return "turnright", 0.30, "right_more_open_short_turn"


def explore_room(name="room", max_duration=30.0, reset_map=False, save=True, rotate_scan=True):
    global active_explore, last_run
    max_duration = min(max(float(max_duration), 0.0), 180.0)
    started = time.time()
    trace = []
    m = ensure_room_map(name=name, reset=reset_map)
    active_explore = {"active": True, "started_at": started, "name": name}
    reason = "max_duration"
    with motion_lock:
        try:
            s0 = snapshot()
            if not scan_ok(s0):
                reason = "scan_stale_or_missing_before_start"
                return {"ok": False, "mode": "explore_room", "reason": reason, "status": s0, "map": map_summary(m), "trace_tail": []}
            update_map_from_scan(m)
            if rotate_scan:
                # In-place panorama: crude but valuable when odometry is unreliable.
                for i in range(6):
                    if time.time() - started >= max_duration:
                        break
                    motor_send("turnleft")
                    dur = 0.25
                    time.sleep(dur)
                    stop_burst(2)
                    update_pose_for_action("turnleft", dur)
                    time.sleep(0.08)
                    up = update_map_from_scan(m)
                    trace.append({"phase": "rotate_scan", "i": i + 1, "update": up, "sectors": snapshot().get("sectors", {})})
            while time.time() - started < max_duration:
                st = snapshot()
                if not scan_ok(st):
                    reason = "scan_stale_or_missing"
                    break
                sec = st.get("sectors", {})
                if (sec.get("front") or 99) < 0.22 or (sec.get("front_left") or 99) < 0.20 or (sec.get("front_right") or 99) < 0.20:
                    # Try one lateral escape if the opposite side is clearly open; otherwise stop.
                    if (sec.get("front_left") or 99) < 0.20 and (sec.get("right") or 0) > 0.45:
                        motor_send("right")
                        dur = 0.28
                        time.sleep(dur)
                        stop_burst(2)
                        update_pose_for_action("right", dur)
                        time.sleep(0.10)
                        up = update_map_from_scan(m)
                        trace.append({"action": "right", "duration": round(dur, 2), "why": "reflex_left_close_escape", "sectors": sec, "pose": pose_copy(), "map_update": up})
                        continue
                    if (sec.get("front_right") or 99) < 0.20 and (sec.get("left") or 0) > 0.45:
                        motor_send("left")
                        dur = 0.28
                        time.sleep(dur)
                        stop_burst(2)
                        update_pose_for_action("left", dur)
                        time.sleep(0.10)
                        up = update_map_from_scan(m)
                        trace.append({"action": "left", "duration": round(dur, 2), "why": "reflex_right_close_escape", "sectors": sec, "pose": pose_copy(), "map_update": up})
                        continue
                    reason = "too_close_reflex_stop"
                    break
                action, dur, why = choose_explore_action(sec)
                motor_send(action)
                time.sleep(dur)
                stop_burst(2)
                update_pose_for_action(action, dur)
                time.sleep(0.10)
                up = update_map_from_scan(m)
                trace.append({"action": action, "duration": round(dur, 2), "why": why, "sectors": sec, "pose": pose_copy(), "map_update": up})
        except Exception as e:
            reason = "exception:" + repr(e)
            with state_lock:
                state["last_error"] = repr(e)
        finally:
            stop_burst(3)
            active_explore = None
    saved_path = save_room_map(name) if save else None
    result = {"ok": not reason.startswith("exception") and not reason.startswith("scan_stale"), "mode": "explore_room", "reason": reason, "elapsed_s": round(time.time() - started, 2), "map": map_summary(m), "saved_path": saved_path, "trace_tail": trace[-30:], "status": snapshot()}
    last_run = result
    remember("explore_done", reason=reason, elapsed_s=result["elapsed_s"], saved_path=saved_path)
    return result

def forward_until(target_front=0.10, max_duration=8.0, pulse=0.45, stall_window=5, stall_delta=0.03,
                  min_target=0.08, reorient=True):
    global last_run
    target_front = max(float(target_front), float(min_target))
    max_duration = min(max(float(max_duration), 0.0), FORWARD_UNTIL_MAX_S)
    pulse = min(max(float(pulse), 0.10), 1.20)
    stall_window = max(int(stall_window), 3)
    stall_delta = max(float(stall_delta), 0.005)

    trace = []
    readings = []
    total = 0.0
    started = time.time()
    reason = "max_duration"

    with motion_lock:
        s0 = snapshot()
        if not scan_ok(s0):
            result = {"ok": False, "mode": "forward_until", "reason": "scan_stale_or_missing_before_start", "status": s0, "trace_tail": []}
            last_run = result
            return result
        if front_distance(s0) is not None and front_distance(s0) <= target_front:
            result = {"ok": True, "mode": "forward_until", "reason": f"already_at_target:{front_distance(s0):.2f}m", "status": s0, "trace_tail": []}
            last_run = result
            return result

        try:
            while total < max_duration:
                s = snapshot()
                before = front_distance(s)
                if not scan_ok(s):
                    reason = "scan_stale_or_missing"
                    break
                if before is not None and before <= target_front:
                    reason = f"target_reached_before:{before:.3f}"
                    break

                run_for = min(pulse, max_duration - total)
                motor_send("forward")
                time.sleep(run_for)
                stop_burst(2)
                total += run_for
                time.sleep(0.08)

                s2 = snapshot()
                after = front_distance(s2)
                readings.append(after)
                entry = {"pulse": len(trace) + 1, "run_s": round(run_for, 2), "total_s": round(total, 2),
                         "front_before": before, "front_after": after, "sectors": s2.get("sectors", {})}
                trace.append(entry)

                if not scan_ok(s2):
                    reason = "scan_stale_or_missing_after_pulse"
                    break
                if after is not None and after <= target_front:
                    reason = f"target_reached:{after:.3f}"
                    break
                if before is not None and after is not None and after - before > 0.08:
                    reason = f"moving_away:{before:.3f}->{after:.3f}"
                    break

                if len(readings) >= stall_window:
                    recent = [x for x in readings[-stall_window:] if x is not None]
                    if len(recent) >= stall_window:
                        progress = recent[0] - recent[-1]
                        if progress < -stall_delta:
                            reason = f"moving_away_or_reversed:{progress:.3f}_over_{stall_window}"
                            break
                        if progress < stall_delta:
                            # One small wiggle to correct the common angled/turning plateau.
                            if reorient and not any(t.get("reorient") for t in trace):
                                sec = s2.get("sectors", {})
                                fl = sec.get("front_left") or 99
                                fr = sec.get("front_right") or 99
                                # Turn toward the side with the closer reading, trying to face the obstacle.
                                turn = "left" if fl < fr else "right"
                                sdk_turn = "turnleft" if turn == "left" else "turnright"
                                motor_send(sdk_turn)
                                time.sleep(0.35)
                                stop_burst(2)
                                trace.append({"reorient": turn, "duration_s": 0.28, "sectors": snapshot().get("sectors", {})})
                                readings.clear()
                                continue
                            reason = f"stalled_progress:{progress:.3f}m_over_{stall_window}_pulses"
                            break
        except Exception as e:
            reason = "exception:" + repr(e)
            with state_lock:
                state["last_error"] = repr(e)
        finally:
            stop_burst(3)

    result = {"ok": not reason.startswith("exception"), "mode": "forward_until", "target_front_m": target_front,
              "reason": reason, "commanded_s": round(total, 2), "elapsed_s": round(time.time() - started, 2),
              "trace_tail": trace[-20:], "status": snapshot()}
    last_run = result
    remember("forward_until_done", reason=reason, commanded_s=round(total, 2), target_front_m=target_front)
    return result


def mark_object(target_front=0.45, max_duration=5.0, turn="right", turn_duration=0.75, dry_run=False):
    """Fair routine: approach frontal object, turn sideways, then run pee trick."""
    target_front = max(0.35, min(float(target_front), 1.0))
    max_duration = max(0.0, min(float(max_duration), 8.0))
    turn_duration = max(0.0, min(float(turn_duration), 2.0))
    turn = str(turn).lower()
    if turn not in ("left", "right"):
        raise ValueError("turn must be 'left' or 'right'")
    plan = {"mode": "mark_object", "target_front_m": target_front, "max_duration_s": max_duration,
            "turn": turn, "turn_duration_s": turn_duration, "trick": resolve_trick(name="pee")}
    if dry_run:
        remember("mark_object_dry_run", plan=plan)
        return {"ok": True, "dry_run": True, "plan": plan, "status": snapshot()}
    steps = []
    approach = forward_until(target_front=target_front, max_duration=max_duration, pulse=0.35, min_target=0.30, reorient=True)
    steps.append({"step": "approach", "result": approach})
    snap = snapshot()
    front = (snap.get("sectors") or {}).get("front")
    if front is None or front < 0.30:
        stop_burst(3)
        result = {"ok": False, "mode": "mark_object", "reason": "unsafe_after_approach", "steps": steps, "status": snap}
        remember("mark_object_abort", reason=result["reason"], front=front)
        return result
    turn_action = "turnright" if turn == "right" else "turnleft"
    steps.append({"step": "turn", "result": run_action(turn_action, turn_duration)})
    steps.append({"step": "pee", "result": sdk_trick(name="pee")})
    result = {"ok": True, "mode": "mark_object", "steps": steps, "status": snapshot()}
    remember("mark_object_done", ok=True)
    return result


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
            elif p.path == "/config":
                self.send_json({"version": state["version"], "motor_backend": MOTOR_BACKEND, "sdk_step_default": SDK_STEP_DEFAULT, "sdk_gait": SDK_GAIT, "sdk_pace": SDK_PACE,
                                "app_host": HOST, "app_port": APP_PORT, "camera_url": CAMERA_URL,
                                "http_port": HTTP_PORT, "max_move_s": MAX_MOVE_S, "forward_until_max_s": FORWARD_UNTIL_MAX_S,
                                "front_stop_m": FRONT_STOP_M, "side_stop_m": SIDE_STOP_M, "scan_stale_s": SCAN_STALE_S, "sdk_error": sdk_error, "map_dir": str(MAP_DIR), "map_resolution_m": MAP_RES_M, "map_size_m": MAP_SIZE_M, "explore_safe_front_m": EXPLORE_SAFE_FRONT_M, "explore_safe_side_m": EXPLORE_SAFE_SIDE_M, "pose_mode": "dead_reckoning+local_scan_matching", "local_map": True,
                                "tricks": {"count": len(TRICK_ACTIONS), "endpoint": "/actions", "settle_s_default": TRICK_SETTLE_S}})
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
                err = stop_burst()
                self.send_json({"ok": err is None, "error": err, "status": snapshot()})
            elif p.path == "/move":
                action = (qs.get("action") or ["stop"])[0]
                duration = float((qs.get("duration") or ["0.2"])[0])
                step = (qs.get("step") or [None])[0]
                self.send_json(run_action(action, duration, step=step))
            elif p.path in ("/action", "/trick"):
                name = (qs.get("name") or qs.get("action") or qs.get("trick") or [None])[0]
                action_id = (qs.get("id") or qs.get("action_id") or [None])[0]
                dry = (qs.get("dry_run") or qs.get("dry") or ["0"])[0] in ("1", "true", "yes")
                settle = (qs.get("settle_s") or qs.get("duration") or [None])[0]
                self.send_json(sdk_trick(name=name, action_id=action_id, dry_run=dry, settle_s=settle))
            elif p.path == "/mark_object":
                dry = (qs.get("dry_run") or qs.get("dry") or ["0"])[0] in ("1", "true", "yes")
                self.send_json(mark_object(
                    target_front=float((qs.get("target_front") or ["0.45"])[0]),
                    max_duration=float((qs.get("max_duration") or ["5.0"])[0]),
                    turn=(qs.get("turn") or ["right"])[0],
                    turn_duration=float((qs.get("turn_duration") or ["0.75"])[0]),
                    dry_run=dry,
                ))
            elif p.path in ("/forward_until", "/learned_forward", "/approach_front"):
                self.send_json(forward_until(
                    target_front=float((qs.get("target_front") or qs.get("target") or ["0.10"])[0]),
                    max_duration=float((qs.get("max_duration") or ["8.0"])[0]),
                    pulse=float((qs.get("pulse") or ["0.45"])[0]),
                    stall_window=int((qs.get("stall_window") or ["5"])[0]),
                    stall_delta=float((qs.get("stall_delta") or ["0.03"])[0]),
                    reorient=(qs.get("reorient") or ["1"])[0] != "0",
                ))
            elif p.path in ("/explore_room", "/explore"):
                self.send_json(explore_room(
                    name=(qs.get("name") or ["room"])[0],
                    max_duration=float((qs.get("max_duration") or ["30"])[0]),
                    reset_map=(qs.get("reset") or ["0"])[0] == "1",
                    save=(qs.get("save") or ["1"])[0] != "0",
                    rotate_scan=(qs.get("rotate_scan") or ["1"])[0] != "0",
                ))
            else:
                self.send_json({"error": "not found", "paths": ["/health", "/status", "/last_run", "/config", "/events", "/scan", "/observe", "/frame.jpg", "/stop", "/move", "/actions", "/action", "/mark_object", "/forward_until", "/explore_room", "/map", "/map.svg", "/local_map", "/local_map.svg", "/pose"]}, 404)
        except Exception as e:
            with state_lock:
                state["last_error"] = repr(e)
            try:
                stop_burst()
            except Exception:
                pass
            self.send_json({"ok": False, "error": repr(e), "status": snapshot()}, 500)

    def do_POST(self):
        p = urlparse(self.path)
        try:
            body = self.read_json()
            if p.path == "/stop":
                err = stop_burst()
                self.send_json({"ok": err is None, "error": err, "status": snapshot()})
            elif p.path == "/move":
                self.send_json(run_action(body.get("action", "stop"), float(body.get("duration", 0.2)), step=body.get("step")))
            elif p.path in ("/action", "/trick"):
                self.send_json(sdk_trick(
                    name=body.get("name", body.get("action", body.get("trick"))),
                    action_id=body.get("id", body.get("action_id")),
                    dry_run=bool(body.get("dry_run", body.get("dry", False))),
                    settle_s=body.get("settle_s", body.get("duration")),
                ))
            elif p.path == "/mark_object":
                self.send_json(mark_object(
                    target_front=float(body.get("target_front", 0.45)),
                    max_duration=float(body.get("max_duration", 5.0)),
                    turn=body.get("turn", "right"),
                    turn_duration=float(body.get("turn_duration", 0.75)),
                    dry_run=bool(body.get("dry_run", body.get("dry", False))),
                ))
            elif p.path in ("/forward_until", "/learned_forward", "/approach_front"):
                self.send_json(forward_until(
                    target_front=float(body.get("target_front", body.get("target", 0.10))),
                    max_duration=float(body.get("max_duration", 8.0)),
                    pulse=float(body.get("pulse", 0.45)),
                    stall_window=int(body.get("stall_window", 5)),
                    stall_delta=float(body.get("stall_delta", 0.03)),
                    min_target=float(body.get("min_target", 0.08)),
                    reorient=bool(body.get("reorient", True)),
                ))
            elif p.path in ("/explore_room", "/explore"):
                self.send_json(explore_room(
                    name=body.get("name", "room"),
                    max_duration=float(body.get("max_duration", 30.0)),
                    reset_map=bool(body.get("reset", False)),
                    save=bool(body.get("save", True)),
                    rotate_scan=bool(body.get("rotate_scan", True)),
                ))
            elif p.path == "/map_reset":
                reset_pose(float(body.get("x", 0.0)), float(body.get("y", 0.0)), float(body.get("yaw", 0.0)))
                ensure_room_map(name=body.get("name", "room"), reset=True)
                update_map_from_scan(room_map)
                self.send_json({"ok": True, "summary": map_summary(room_map)})
            else:
                self.send_json({"error": "not found"}, 404)
        except Exception as e:
            with state_lock:
                state["last_error"] = repr(e)
            try:
                stop_burst()
            except Exception:
                pass
            self.send_json({"ok": False, "error": repr(e), "status": snapshot()}, 500)


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
