#!/usr/bin/env python3
"""Restart BEEP's Cartographer stack and bridge through Jupyter.

BEEP's vendor image enables both YahboomStart and XGO_Start, which launch
competing MS200 publishers on /dev/ttyAMA1. Keep a healthy YahboomStart publisher
authoritative, stop the duplicate, and rebuild only downstream SLAM state.
"""
from __future__ import annotations

import asyncio
import json
import sys
import time
import uuid

import requests
import websockets

from jupyter_deploy_bridge import BASE, HOST, login


async def reset_slam(session: requests.Session, xsrf: str) -> None:
    headers = {"X-XSRFToken": xsrf, "Content-Type": "application/json"}
    response = session.post(f"{BASE}/api/kernels", json={"name": "python3"}, headers=headers, timeout=8)
    response.raise_for_status()
    kernel_id = response.json()["id"]
    cookie = "; ".join(f"{c.name}={c.value}" for c in session.cookies)
    code = r'''
import subprocess, time
import json, urllib.request
def require_stopped():
    data = json.load(urllib.request.urlopen('http://127.0.0.1:8766/mission', timeout=5))
    mission = data.get('mission')
    status = data.get('status') or {}
    if ((mission is not None and mission.get('state') not in ('completed', 'failed', 'cancelled'))
            or status.get('moving') is not False or status.get('motion_lease_id') is not None):
        raise RuntimeError('refusing SLAM reset during active motion or lease')
require_stopped()
steps = [
    (['sudo', 'systemctl', 'stop', 'XGO_Start'], 2),
    (['sudo', 'systemctl', 'restart', 'beep-cartographer', 'beep-occupancy-grid'], 4),
    (['sudo', 'systemctl', 'restart', 'beep-bridge'], 3),
]
for index, (command, pause_s) in enumerate(steps):
    if index > 0:
        require_stopped()
    if index == 1:
        lidar = subprocess.run(['systemctl','is-active','YahboomStart'],
                               text=True,capture_output=True,timeout=5)
        if lidar.stdout.strip() != 'active':
            raise RuntimeError('authoritative YahboomStart LiDAR is not active')
        observed = json.load(urllib.request.urlopen('http://127.0.0.1:8766/status', timeout=5))
        scan_age = observed.get('scan_age_s')
        if (not isinstance(scan_age, (int, float)) or not 0 <= scan_age <= .6
                or observed.get('scan_count', 0) < 1):
            raise RuntimeError('authoritative YahboomStart scan is not fresh after XGO stop')
    result = subprocess.run(command, text=True, capture_output=True, timeout=40)
    print('step', command, result.returncode, result.stderr.strip())
    if result.returncode:
        raise SystemExit(result.returncode)
    time.sleep(pause_s)
expected = {
    'XGO_Start': 'inactive',
    'YahboomStart': 'active',
    'beep-cartographer': 'active',
    'beep-occupancy-grid': 'active',
    'beep-bridge': 'active',
}
for service, wanted in expected.items():
    result = subprocess.run(['systemctl', 'is-active', service], text=True, capture_output=True, timeout=5)
    actual = result.stdout.strip()
    print(service, actual)
    if actual != wanted:
        raise SystemExit(1)

def wait_for_usable_slam(seconds):
    previous_scan_count = None
    advancing = False
    for _ in range(seconds):
        require_stopped()
        status = json.load(urllib.request.urlopen('http://127.0.0.1:8766/status', timeout=5))
        count = status.get('scan_count', 0)
        if previous_scan_count is not None and count > previous_scan_count:
            advancing = True
        previous_scan_count = count
        slam = status.get('slam') or {}
        if (advancing and status.get('scan_age_s', float('inf')) < .6
                and slam.get('usable') is True):
            print('SLAM ready: advancing fresh scans, valid pose and map')
            return True
        time.sleep(1)
    print('SLAM not ready:', slam.get('submap_error'), slam.get('tf_error'))
    return False

if not wait_for_usable_slam(15):
    # A downstream-only reset can leave Cartographer subscribed to /scan but
    # receiving no scan samples. Recreate the vendor publisher once, while
    # stopped and unowned, before rebuilding Cartographer's empty trajectory.
    require_stopped()
    baseline = json.load(urllib.request.urlopen('http://127.0.0.1:8766/status', timeout=5)).get('scan_count', 0)
    result = subprocess.run(['sudo', 'systemctl', 'restart', 'YahboomStart'],
                            text=True, capture_output=True, timeout=50)
    print('restart authoritative LiDAR launch', result.returncode, result.stderr.strip())
    if result.returncode:
        raise RuntimeError('YahboomStart restart failed')
    for _ in range(20):
        time.sleep(1)
        require_stopped()
        status = json.load(urllib.request.urlopen('http://127.0.0.1:8766/status', timeout=5))
        if (status.get('scan_count', 0) > baseline
                and status.get('scan_age_s', float('inf')) < .6):
            break
    else:
        raise RuntimeError('fresh LiDAR scans did not resume after YahboomStart restart')
    require_stopped()
    result = subprocess.run(['sudo', 'systemctl', 'restart',
                             'beep-cartographer', 'beep-occupancy-grid'],
                            text=True, capture_output=True, timeout=45)
    print('restart downstream SLAM', result.returncode, result.stderr.strip())
    if result.returncode or not wait_for_usable_slam(30):
        raise RuntimeError('SLAM remains unusable after one publisher/downstream reset')
'''
    uri = f"ws://{HOST}:8888/api/kernels/{kernel_id}/channels"
    try:
        async with websockets.connect(uri, additional_headers={"Cookie": cookie}, open_timeout=8, ping_interval=None) as ws:
            message = {
                "header": {"msg_id": str(uuid.uuid4()), "username": "gladis", "session": str(uuid.uuid4()), "msg_type": "execute_request", "version": "5.3"},
                "parent_header": {}, "metadata": {},
                "content": {"code": code, "silent": False, "store_history": False, "user_expressions": {}, "allow_stdin": False, "stop_on_error": True},
                "channel": "shell", "buffers": [],
            }
            await ws.send(json.dumps(message))
            deadline = time.monotonic() + 240
            output: list[str] = []
            while time.monotonic() < deadline:
                msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=max(1, deadline - time.monotonic())))
                if msg.get("parent_header", {}).get("msg_id") != message["header"]["msg_id"]:
                    continue
                if msg.get("msg_type") == "stream":
                    output.append(msg["content"].get("text", ""))
                elif msg.get("msg_type") == "error":
                    raise RuntimeError("remote SLAM reset error: " + json.dumps(msg["content"]))
                elif msg.get("msg_type") == "execute_reply":
                    print("".join(output), end="")
                    if msg.get("content", {}).get("status") != "ok":
                        raise RuntimeError("remote SLAM reset failed: " + json.dumps(msg["content"]))
                    return
            raise TimeoutError("SLAM reset timed out")
    finally:
        original_error = sys.exc_info()[0]
        try:
            session.delete(f"{BASE}/api/kernels/{kernel_id}",
                           headers=headers, timeout=8).raise_for_status()
        except Exception:
            if original_error is None:
                raise
            print("warning: SLAM reset kernel cleanup failed", file=sys.stderr)


def main() -> int:
    session, xsrf = login()
    preflight = session.get(f"http://{HOST}:8766/mission", timeout=5)
    preflight.raise_for_status()
    status = preflight.json()
    mission = status.get("mission")
    if ((mission is not None and mission.get("state") not in ("completed", "failed", "cancelled"))
            or status.get("status", {}).get("moving") is not False
            or status.get("status", {}).get("motion_lease_id") is not None):
        raise RuntimeError("refusing SLAM reset during active motion or lease")
    asyncio.run(reset_slam(session, xsrf))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
