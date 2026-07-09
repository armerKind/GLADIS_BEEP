#!/usr/bin/env python3
"""Install BEEP bridge as a systemd service on the Pi via Jupyter.

This script is intentionally Jupyter-based because BEEP has historically exposed
port 8888 more reliably than SSH. It writes `/etc/systemd/system/beep-bridge.service`,
reloads systemd, enables the unit, and starts/restarts it.

Environment variables:
  BEEP_HOST                 default: 192.168.8.88
  BEEP_JUPYTER_PASSWORD     default: yahboom
  BEEP_SERVICE_FILE         default: systemd/beep-bridge.service
"""
from __future__ import annotations

import asyncio
import base64
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
SERVICE_FILE = Path(os.environ.get("BEEP_SERVICE_FILE", "systemd/beep-bridge.service"))
BASE = f"http://{HOST}:8888"


def login() -> tuple[requests.Session, str]:
    s = requests.Session()
    r = s.get(f"{BASE}/login?next=%2Flab", timeout=8)
    r.raise_for_status()
    xsrf = s.cookies.get("_xsrf") or ""
    rr = s.post(
        f"{BASE}/login?next=%2Flab",
        data={"password": PASSWORD, "_xsrf": xsrf},
        headers={"Referer": f"{BASE}/login?next=%2Flab"},
        timeout=8,
        allow_redirects=False,
    )
    rr.raise_for_status()
    m = re.search(r'(username-[^=]+)="?([^";]+)', rr.headers.get("Set-Cookie", ""))
    if m:
        s.cookies.set(m.group(1), m.group(2), domain=HOST, path="/")
    api = s.get(f"{BASE}/api", timeout=8)
    api.raise_for_status()
    return s, xsrf


async def run_jupyter(session: requests.Session, xsrf: str, code: str, timeout_s: int = 45) -> str:
    headers = {"X-XSRFToken": xsrf, "Content-Type": "application/json"}
    kr = session.post(f"{BASE}/api/kernels", json={"name": "python3"}, headers=headers, timeout=8)
    kr.raise_for_status()
    kernel_id = kr.json()["id"]
    cookie = "; ".join([f"{c.name}={c.value}" for c in session.cookies])
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
        deadline = time.time() + timeout_s
        output: list[str] = []
        while time.time() < deadline:
            m = json.loads(await asyncio.wait_for(ws.recv(), timeout=max(1, deadline - time.time())))
            if m.get("msg_type") == "stream":
                output.append(m["content"].get("text", ""))
            elif m.get("msg_type") == "error":
                output.append("ERROR " + json.dumps(m["content"]))
            elif m.get("msg_type") == "execute_reply":
                return "".join(output)
        raise TimeoutError("Jupyter command timed out")


def main() -> int:
    if not SERVICE_FILE.exists():
        print(f"missing service file: {SERVICE_FILE}", file=sys.stderr)
        return 2
    encoded = base64.b64encode(SERVICE_FILE.read_bytes()).decode("ascii")
    code = f'''
import base64, pathlib, subprocess, os, textwrap
service = base64.b64decode({encoded!r}).decode('utf-8')
path = pathlib.Path('/tmp/beep-bridge.service')
path.write_text(service)
cmds = [
    ['sudo','cp','/tmp/beep-bridge.service','/etc/systemd/system/beep-bridge.service'],
    ['sudo','systemctl','daemon-reload'],
    ['sudo','systemctl','enable','beep-bridge.service'],
    ['sudo','systemctl','restart','beep-bridge.service'],
    ['systemctl','is-enabled','beep-bridge.service'],
    ['systemctl','is-active','beep-bridge.service'],
    ['systemctl','--no-pager','--lines','20','status','beep-bridge.service'],
]
for cmd in cmds:
    print('\n###', ' '.join(cmd))
    p = subprocess.run(cmd, text=True, capture_output=True, timeout=20)
    print('rc', p.returncode)
    print((p.stdout + p.stderr)[-4000:])
    if p.returncode != 0 and cmd[0] == 'sudo':
        raise SystemExit(p.returncode)
'''
    s, xsrf = login()
    print(asyncio.run(run_jupyter(s, xsrf, code, timeout_s=80)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
