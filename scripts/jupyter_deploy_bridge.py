#!/usr/bin/env python3
"""Deploy and restart BEEP bridge through JupyterLab.

This is useful when SSH is unavailable but the Pi's Jupyter server on port 8888 is reachable.

Environment variables:
  BEEP_HOST                 default: 192.168.8.88
  BEEP_JUPYTER_PASSWORD     default: yahboom
  BEEP_BRIDGE_SRC           default: beep_bridge/beep_bridge.py
  BEEP_PLANNER_SRC          default: beep_bridge/frontier_planner.py
  BEEP_INTENT_SRC           default: beep_bridge/semantic_intent.py
  BEEP_SLAM_CONFIG_SRC      default: config/beep_2d.lua
  BEEP_SAVE_SLAM_SRC        default: scripts/save_slam_map.sh
  BEEP_SERVICE_SRC          default: systemd/beep-bridge.service
"""
from __future__ import annotations

import asyncio
import json
import os
import re
import sys
import time
import uuid
from pathlib import Path

import requests
import websockets

HOST = os.environ.get("BEEP_HOST", "192.168.8.88")
PASSWORD = os.environ.get("BEEP_JUPYTER_PASSWORD", "yahboom")
BRIDGE_SRC = Path(os.environ.get("BEEP_BRIDGE_SRC", "beep_bridge/beep_bridge.py"))
PLANNER_SRC = Path(os.environ.get("BEEP_PLANNER_SRC", "beep_bridge/frontier_planner.py"))
INTENT_SRC = Path(os.environ.get("BEEP_INTENT_SRC", "beep_bridge/semantic_intent.py"))
SLAM_CONFIG_SRC = Path(os.environ.get("BEEP_SLAM_CONFIG_SRC", "config/beep_2d.lua"))
SAVE_SLAM_SRC = Path(os.environ.get("BEEP_SAVE_SLAM_SRC", "scripts/save_slam_map.sh"))
SERVICE_SRC = Path(os.environ.get("BEEP_SERVICE_SRC", "systemd/beep-bridge.service"))
BASE = f"http://{HOST}:8888"

def bridge_source_version(content: str) -> str:
    """Fail closed unless the version can be read from the uploaded bridge."""
    match = re.search(r'(?m)^state\s*=\s*\{\s*\n\s*"version"\s*:\s*"([a-zA-Z0-9_.-]{1,64})"\s*,', content)
    if match is None:
        raise ValueError('bridge source has no static state version')
    return match.group(1)


def login() -> tuple[requests.Session, str]:
    session = requests.Session()
    r = session.get(f"{BASE}/login?next=%2Flab", timeout=8)
    r.raise_for_status()
    xsrf = session.cookies.get("_xsrf") or ""
    rr = session.post(
        f"{BASE}/login?next=%2Flab",
        data={"password": PASSWORD, "_xsrf": xsrf},
        headers={"Referer": f"{BASE}/login?next=%2Flab"},
        timeout=8,
        allow_redirects=False,
    )
    rr.raise_for_status()
    m = re.search(r'(username-[^=]+)="?([^";]+)', rr.headers.get("Set-Cookie", ""))
    if m:
        session.cookies.set(m.group(1), m.group(2), domain=HOST, path="/")
    api = session.get(f"{BASE}/api", timeout=8)
    api.raise_for_status()
    return session, xsrf


def put_bridge(session: requests.Session, xsrf: str, content: str) -> None:
    intent_content = INTENT_SRC.read_text()  # Fail before changing the remote if missing.
    headers = {"X-XSRFToken": xsrf, "Content-Type": "application/json"}
    # Best-effort backup.
    old = session.get(f"{BASE}/api/contents/beep_bridge/beep_bridge.py?content=1", timeout=8)
    if old.status_code == 200:
        backup = f"beep_bridge/beep_bridge.py.bak_deploy_{int(time.time())}"
        session.put(
            f"{BASE}/api/contents/{backup}",
            headers=headers,
            json={"type": "file", "format": "text", "content": old.json().get("content", "")},
            timeout=10,
        )
        print(f"backup: {backup}")
    resp = session.put(
        f"{BASE}/api/contents/beep_bridge/beep_bridge.py",
        headers=headers,
        json={"type": "file", "format": "text", "content": content},
        timeout=10,
    )
    resp.raise_for_status()
    print("uploaded: /home/pi/beep_bridge/beep_bridge.py")
    intent_module = session.put(
        f"{BASE}/api/contents/beep_bridge/semantic_intent.py",
        headers=headers,
        json={"type": "file", "format": "text", "content": intent_content},
        timeout=10,
    )
    intent_module.raise_for_status()
    print("uploaded: /home/pi/beep_bridge/semantic_intent.py")
    if PLANNER_SRC.exists():
        planner = session.put(
            f"{BASE}/api/contents/beep_bridge/frontier_planner.py",
            headers=headers,
            json={"type": "file", "format": "text", "content": PLANNER_SRC.read_text()},
            timeout=10,
        )
        planner.raise_for_status()
        print("uploaded: /home/pi/beep_bridge/frontier_planner.py")
    if SLAM_CONFIG_SRC.exists():
        slam_config = session.put(
            f"{BASE}/api/contents/cartographer_ws2/install/cartographer_ros/share/cartographer_ros/configuration_files/beep_2d.lua",
            headers=headers,
            json={"type": "file", "format": "text", "content": SLAM_CONFIG_SRC.read_text()},
            timeout=10,
        )
        slam_config.raise_for_status()
        print("uploaded: Cartographer beep_2d.lua")
    if SAVE_SLAM_SRC.exists():
        save_script = session.put(
            f"{BASE}/api/contents/beep_bridge/save_slam_map.sh",
            headers=headers,
            json={"type": "file", "format": "text", "content": SAVE_SLAM_SRC.read_text()},
            timeout=10,
        )
        save_script.raise_for_status()
        print("uploaded: /home/pi/beep_bridge/save_slam_map.sh")
    if SERVICE_SRC.exists():
        service = session.put(
            f"{BASE}/api/contents/beep_bridge/beep-bridge.service",
            headers=headers,
            json={"type": "file", "format": "text", "content": SERVICE_SRC.read_text()},
            timeout=10,
        )
        service.raise_for_status()
        print("uploaded: /home/pi/beep_bridge/beep-bridge.service")


async def restart_bridge(session: requests.Session, xsrf: str, expected_version: str) -> None:
    headers = {"X-XSRFToken": xsrf, "Content-Type": "application/json"}
    kr = session.post(f"{BASE}/api/kernels", json={"name": "python3"}, headers=headers, timeout=8)
    kr.raise_for_status()
    kernel_id = kr.json()["id"]
    cookie = "; ".join([f"{c.name}={c.value}" for c in session.cookies])
    code = r'''
import subprocess, os, time, pathlib, signal, json, urllib.request
base = pathlib.Path('/home/pi/beep_bridge')
preflight = json.load(urllib.request.urlopen('http://127.0.0.1:8766/mission', timeout=5))
mission = preflight.get('mission')
if ((mission is not None and mission.get('state') not in ('completed', 'failed', 'cancelled'))
        or preflight['status'].get('moving') is not False
        or preflight['status'].get('motion_lease_id') is not None):
    raise SystemExit('refusing bridge restart during active mission, motion or lease')
(base/'logs').mkdir(parents=True, exist_ok=True)
(base/'maps').mkdir(parents=True, exist_ok=True)
if (base/'save_slam_map.sh').exists():
    (base/'save_slam_map.sh').chmod(0o755)
p = subprocess.run(['python3','-m','py_compile',str(base/'beep_bridge.py'),str(base/'frontier_planner.py'),str(base/'semantic_intent.py')], text=True, capture_output=True, timeout=10)
print('py_compile', p.returncode, p.stderr)
if p.returncode:
    raise SystemExit(p.returncode)
smoke = subprocess.run(['python3', '-c', "import sys; sys.path.insert(0, sys.argv[1]); import semantic_intent", str(base)], text=True, capture_output=True, timeout=10)
print('semantic_import', smoke.returncode, smoke.stderr)
if smoke.returncode:
    raise SystemExit(smoke.returncode)
unit = base/'beep-bridge.service'
if unit.exists():
    install = subprocess.run(['sudo','install','-m','0644',str(unit),'/etc/systemd/system/beep-bridge.service'], text=True, capture_output=True, timeout=10)
    print('systemd_install', install.returncode, install.stderr)
    if install.returncode:
        raise SystemExit(install.returncode)
    reload = subprocess.run(['sudo','systemctl','daemon-reload'], text=True, capture_output=True, timeout=10)
    print('systemd_daemon_reload', reload.returncode, reload.stderr)
    if reload.returncode:
        raise SystemExit(reload.returncode)
svc = subprocess.run(['bash','-lc','systemctl list-unit-files beep-bridge.service --no-legend 2>/dev/null | grep -q beep-bridge.service'], text=True, timeout=10)
pidfile = base/'bridge.pid'
if svc.returncode == 0:
    fresh = json.load(urllib.request.urlopen('http://127.0.0.1:8766/mission', timeout=5))
    active = fresh.get('mission')
    if ((active is not None and active.get('state') not in ('completed', 'failed', 'cancelled'))
            or fresh['status'].get('moving') is not False
            or fresh['status'].get('motion_lease_id') is not None):
        raise SystemExit('refusing restart: mission or lease appeared during deployment')
    legacy_terminated = False
    if pidfile.exists():
        try:
            pid = int(pidfile.read_text().strip())
            os.kill(pid, signal.SIGTERM)
            legacy_terminated = True
            print('killed legacy pid', pid)
            pidfile.unlink(missing_ok=True)
            time.sleep(.5)
        except Exception as e:
            print('legacy pid cleanup skipped', repr(e))
    enable = subprocess.run(['sudo','systemctl','enable','beep-bridge'], text=True, capture_output=True, timeout=20)
    print('systemd_enable', enable.returncode, enable.stderr)
    if enable.returncode:
        raise SystemExit(enable.returncode)
    if not legacy_terminated:
        fresh = json.load(urllib.request.urlopen('http://127.0.0.1:8766/mission', timeout=5))
        active = fresh.get('mission')
        if ((active is not None and active.get('state') not in ('completed', 'failed', 'cancelled'))
                or fresh['status'].get('moving') is not False
                or fresh['status'].get('motion_lease_id') is not None):
            raise SystemExit('refusing restart: mission or lease appeared during deployment')
    r = subprocess.run(['sudo','systemctl','restart','beep-bridge'], text=True, capture_output=True, timeout=20)
    print('systemd_restart', r.returncode, r.stderr)
    if r.returncode:
        raise SystemExit(r.returncode)
    time.sleep(2.0)
    active = subprocess.run(['systemctl','is-active','beep-bridge'], text=True, capture_output=True, timeout=5)
    print('systemd_active', active.stdout.strip())
    if active.stdout.strip() != 'active':
        raise SystemExit('beep-bridge did not become active')
    config = None
    for _ in range(10):
        try:
            config = json.load(urllib.request.urlopen('http://127.0.0.1:8766/config', timeout=1))
            break
        except Exception:
            time.sleep(0.5)
    if config is None:
        raise SystemExit('bridge active but /config did not become ready within 15 seconds')
    expected = {'version': __EXPECTED_VERSION__,
                'motor_backend': 'app', 'sdk_gait': 'firmware', 'sdk_pace': 'normal',
                'sdk_body_height': 108, 'sdk_shoulder_yaw': 0, 'sdk_imu': 0}
    mismatches = {key: {'expected': value, 'actual': config.get(key)}
                  for key, value in expected.items() if config.get(key) != value}
    print('bridge_config', json.dumps(config, sort_keys=True))
    if mismatches:
        raise SystemExit('deployed bridge profile mismatch: ' + json.dumps(mismatches, sort_keys=True))
    try:
        slam_map = json.load(urllib.request.urlopen('http://127.0.0.1:8766/slam_map', timeout=5))
    except Exception as exc:
        raise SystemExit(f'/slam_map endpoint check failed: {type(exc).__name__}')
    if not isinstance(slam_map, dict) or 'ok' not in slam_map:
        raise SystemExit('/slam_map returned an invalid schema')
    print('slam_map_endpoint', 'grid-ready' if slam_map.get('ok') else 'route-live/map-pending')
else:
    log = open(base/'logs'/'bridge.log', 'ab', buffering=0)
    env = os.environ.copy()
    env.update({
        'ROS_DOMAIN_ID': '16',
        'ROS_LOCALHOST_ONLY': '0',
        'BEEP_APP_HOST': '192.168.8.88',
        'BEEP_CAMERA_URL': 'http://192.168.8.88:6500/video_feed',
        'BEEP_FORWARD_UNTIL_MAX_S': '12',
        'BEEP_MOTOR_BACKEND': 'app',
        'BEEP_SDK_STEP': '10',
        'BEEP_SDK_GAIT': 'firmware',
        'BEEP_SDK_PACE': 'normal',
        'BEEP_SDK_BODY_HEIGHT': '108',
        'BEEP_SDK_SHOULDER_YAW': '0',
        'BEEP_SDK_IMU': '0',
        'BEEP_MAP_DIR': '/home/pi/beep_bridge/maps',
        'BEEP_SDK_FORWARD_M_PER_S': '0.045',
        'BEEP_SDK_STRAFE_M_PER_S': '0.035',
        'BEEP_SDK_TURN_RAD_PER_S': '0.45',
    })
    if pidfile.exists():
        try:
            pid = int(pidfile.read_text().strip())
            os.kill(pid, signal.SIGTERM)
            print('killed old pid', pid)
            time.sleep(.5)
        except Exception as e:
            print('kill old skipped', repr(e))
    cmd = ['bash','-lc','source /opt/ros/foxy/setup.bash; source /home/pi/cartographer_ws2/install/setup.bash 2>/dev/null || true; exec python3 /home/pi/beep_bridge/beep_bridge.py']
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    pidfile.write_text(str(proc.pid))
    print('started pid', proc.pid)
    time.sleep(1.5)
    print('alive', proc.poll() is None)
'''
    code = code.replace('__EXPECTED_VERSION__', json.dumps(expected_version))
    uri = f"ws://{HOST}:8888/api/kernels/{kernel_id}/channels"
    try:
        async with websockets.connect(uri, additional_headers={"Cookie": cookie}, open_timeout=8) as ws:
            msg = {
                "header": {"msg_id": str(uuid.uuid4()), "username": "gladis", "session": str(uuid.uuid4()), "msg_type": "execute_request", "version": "5.3"},
                "parent_header": {},
                "metadata": {},
                "content": {"code": code, "silent": False, "store_history": False, "user_expressions": {}, "allow_stdin": False, "stop_on_error": True},
                "channel": "shell", "buffers": [],
            }
            await ws.send(json.dumps(msg))
            deadline = time.monotonic() + 130
            output: list[str] = []
            while time.monotonic() < deadline:
                try:
                    m = json.loads(await asyncio.wait_for(ws.recv(), timeout=max(1, deadline - time.monotonic())))
                except asyncio.TimeoutError as exc:
                    raise TimeoutError("restart outcome indeterminate; reconcile live bridge state") from exc
                if m.get("parent_header", {}).get("msg_id") != msg["header"]["msg_id"]:
                    continue
                if m.get("msg_type") == "stream":
                    output.append(m["content"].get("text", ""))
                elif m.get("msg_type") == "error":
                    raise RuntimeError("remote deploy error: " + json.dumps(m["content"]))
                elif m.get("msg_type") == "execute_reply":
                    print("".join(output))
                    if m.get("content", {}).get("status") != "ok":
                        raise RuntimeError("remote deploy failed: " + json.dumps(m["content"]))
                    return
            raise TimeoutError("restart outcome indeterminate; reconcile live bridge state")
    finally:
        primary_error = sys.exc_info()[0]
        try:
            session.delete(f"{BASE}/api/kernels/{kernel_id}", headers={"X-XSRFToken": xsrf}, timeout=8).raise_for_status()
        except Exception:
            if primary_error is None:
                raise
            print("warning: kernel cleanup failed during deployment error", file=sys.stderr)


def main() -> int:
    if not BRIDGE_SRC.exists():
        print(f"missing bridge source: {BRIDGE_SRC}", file=sys.stderr)
        return 2
    if not SERVICE_SRC.exists():
        print(f"missing service source: {SERVICE_SRC}", file=sys.stderr)
        return 2
    content = BRIDGE_SRC.read_text()
    expected_version = bridge_source_version(content)
    session, xsrf = login()
    preflight = session.get(f"http://{HOST}:8766/mission", timeout=5)
    preflight.raise_for_status()
    live = preflight.json()
    mission = live.get("mission")
    if ((mission is not None and mission.get("state") not in ("completed", "failed", "cancelled"))
            or live.get("status", {}).get("moving") is not False
            or live.get("status", {}).get("motion_lease_id") is not None):
        raise RuntimeError("refusing deployment during active mission, motion or lease")
    put_bridge(session, xsrf, content)
    asyncio.run(restart_bridge(session, xsrf, expected_version))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
