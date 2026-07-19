#!/usr/bin/env python3
"""Reset BEEP's active Cartographer trajectory through its local Jupyter kernel."""
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
for services in (['beep-cartographer', 'beep-occupancy-grid'], ['beep-bridge']):
    result = subprocess.run(['sudo', 'systemctl', 'restart', *services], text=True, capture_output=True, timeout=30)
    print('restart', services, result.returncode, result.stderr.strip())
    if result.returncode:
        raise SystemExit(result.returncode)
    time.sleep(3)
for service in ('beep-cartographer', 'beep-occupancy-grid', 'beep-bridge'):
    result = subprocess.run(['systemctl', 'is-active', service], text=True, capture_output=True, timeout=5)
    print(service, result.stdout.strip())
    if result.stdout.strip() != 'active':
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
