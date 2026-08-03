#!/usr/bin/env python3
"""Restart BEEP's LiDAR publisher, Cartographer stack, and bridge through Jupyter.

BEEP's vendor image enables both YahboomStart and XGO_Start, which launch
competing MS200 publishers on /dev/ttyAMA1. Keep YahboomStart authoritative and
stop the duplicate before rebuilding SLAM state.
"""
from __future__ import annotations

import asyncio
import json
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
steps = [
    (['sudo', 'systemctl', 'stop', 'XGO_Start'], 2),
    (['sudo', 'systemctl', 'restart', 'YahboomStart'], 7),
    (['sudo', 'systemctl', 'restart', 'beep-cartographer', 'beep-occupancy-grid'], 4),
    (['sudo', 'systemctl', 'restart', 'beep-bridge'], 3),
]
for command, pause_s in steps:
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
'''
    uri = f"ws://{HOST}:8888/api/kernels/{kernel_id}/channels"
    async with websockets.connect(uri, additional_headers={"Cookie": cookie}, open_timeout=8, ping_interval=None) as ws:
        message = {
            "header": {"msg_id": str(uuid.uuid4()), "username": "gladis", "session": str(uuid.uuid4()), "msg_type": "execute_request", "version": "5.3"},
            "parent_header": {}, "metadata": {},
            "content": {"code": code, "silent": False, "store_history": False, "user_expressions": {}, "allow_stdin": False, "stop_on_error": True},
            "channel": "shell", "buffers": [],
        }
        await ws.send(json.dumps(message))
        deadline = time.time() + 60
        output: list[str] = []
        while time.time() < deadline:
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=max(1, deadline - time.time())))
            if msg.get("msg_type") == "stream":
                output.append(msg["content"].get("text", ""))
            elif msg.get("msg_type") == "error":
                raise RuntimeError(json.dumps(msg["content"]))
            elif msg.get("msg_type") == "execute_reply":
                print("".join(output), end="")
                return
        raise TimeoutError("SLAM reset timed out")


def main() -> int:
    session, xsrf = login()
    asyncio.run(reset_slam(session, xsrf))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
