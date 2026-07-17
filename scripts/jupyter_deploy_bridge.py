#!/usr/bin/env python3
"""Deploy and restart BEEP bridge through JupyterLab.

This is useful when SSH is unavailable but the Pi's Jupyter server on port 8888 is reachable.

Environment variables:
  BEEP_HOST                 default: 192.168.8.88
  BEEP_JUPYTER_PASSWORD     default: yahboom
  BEEP_BRIDGE_SRC           default: beep_bridge/beep_bridge.py
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
BASE = f"http://{HOST}:8888"


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


def put_bridge(session: requests.Session, xsrf: str) -> None:
    content = BRIDGE_SRC.read_text()
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
    if PLANNER_SRC.exists():
        planner = session.put(
            f"{BASE}/api/contents/beep_bridge/frontier_planner.py",
            headers=headers,
            json={"type": "file", "format": "text", "content": PLANNER_SRC.read_text()},
            timeout=10,
        )
        planner.raise_for_status()
        print("uploaded: /home/pi/beep_bridge/frontier_planner.py")


async def restart_bridge(session: requests.Session, xsrf: str) -> None:
    headers = {"X-XSRFToken": xsrf, "Content-Type": "application/json"}
    kr = session.post(f"{BASE}/api/kernels", json={"name": "python3"}, headers=headers, timeout=8)
    kr.raise_for_status()
    kernel_id = kr.json()["id"]
    cookie = "; ".join([f"{c.name}={c.value}" for c in session.cookies])
    code = r'''
import subprocess, os, time, pathlib, signal
base = pathlib.Path('/home/pi/beep_bridge')
(base/'logs').mkdir(parents=True, exist_ok=True)
(base/'maps').mkdir(parents=True, exist_ok=True)
p = subprocess.run(['python3','-m','py_compile',str(base/'beep_bridge.py'),str(base/'frontier_planner.py')], text=True, capture_output=True, timeout=10)
print('py_compile', p.returncode, p.stderr)
if p.returncode:
    raise SystemExit(p.returncode)
svc = subprocess.run(['bash','-lc','systemctl list-unit-files beep-bridge.service --no-legend 2>/dev/null | grep -q beep-bridge.service'], text=True)
pidfile = base/'bridge.pid'
if svc.returncode == 0:
    if pidfile.exists():
        try:
            pid = int(pidfile.read_text().strip())
            os.kill(pid, signal.SIGTERM)
            print('killed legacy pid', pid)
            pidfile.unlink(missing_ok=True)
            time.sleep(.5)
        except Exception as e:
            print('legacy pid cleanup skipped', repr(e))
    r = subprocess.run(['sudo','systemctl','restart','beep-bridge'], text=True, capture_output=True, timeout=20)
    print('systemd_restart', r.returncode, r.stderr)
    if r.returncode:
        raise SystemExit(r.returncode)
    time.sleep(2.0)
    active = subprocess.run(['systemctl','is-active','beep-bridge'], text=True, capture_output=True, timeout=5)
    print('systemd_active', active.stdout.strip())
else:
    log = open(base/'logs'/'bridge.log', 'ab', buffering=0)
    env = os.environ.copy()
    env.update({
        'ROS_DOMAIN_ID': '16',
        'ROS_LOCALHOST_ONLY': '0',
        'BEEP_APP_HOST': '192.168.8.88',
        'BEEP_CAMERA_URL': 'http://192.168.8.88:6500/video_feed',
        'BEEP_FORWARD_UNTIL_MAX_S': '12',
        'BEEP_MOTOR_BACKEND': 'sdk',
        'BEEP_SDK_STEP': '10',
        'BEEP_SDK_GAIT': 'walk',
        'BEEP_SDK_PACE': 'slow',
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
    uri = f"ws://{HOST}:8888/api/kernels/{kernel_id}/channels"
    async with websockets.connect(uri, additional_headers={"Cookie": cookie}, open_timeout=8) as ws:
        msg = {
            "header": {"msg_id": str(uuid.uuid4()), "username": "gladis", "session": str(uuid.uuid4()), "msg_type": "execute_request", "version": "5.3"},
            "parent_header": {},
            "metadata": {},
            "content": {"code": code, "silent": False, "store_history": False, "user_expressions": {}, "allow_stdin": False, "stop_on_error": True},
            "channel": "shell",
            "buffers": [],
        }
        await ws.send(json.dumps(msg))
        deadline = time.time() + 35
        output: list[str] = []
        while time.time() < deadline:
            m = json.loads(await asyncio.wait_for(ws.recv(), timeout=max(1, deadline - time.time())))
            if m.get("msg_type") == "stream":
                output.append(m["content"].get("text", ""))
            elif m.get("msg_type") == "error":
                output.append("ERROR " + json.dumps(m["content"]))
            elif m.get("msg_type") == "execute_reply":
                print("".join(output))
                return
        raise TimeoutError("restart command timed out")


def main() -> int:
    if not BRIDGE_SRC.exists():
        print(f"missing bridge source: {BRIDGE_SRC}", file=sys.stderr)
        return 2
    session, xsrf = login()
    put_bridge(session, xsrf)
    asyncio.run(restart_bridge(session, xsrf))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
