#!/usr/bin/env python3
"""Upload an audio file to BEEP through Jupyter and play it via the Pi analog speaker."""
from __future__ import annotations

import argparse
import asyncio
import base64
import json
import time
import uuid
from pathlib import Path

import requests
import websockets

from jupyter_deploy_bridge import BASE, HOST, login


async def execute(session: requests.Session, xsrf: str, code: str) -> None:
    headers = {"X-XSRFToken": xsrf, "Content-Type": "application/json"}
    response = session.post(f"{BASE}/api/kernels", json={"name": "python3"}, headers=headers, timeout=8)
    response.raise_for_status()
    kernel_id = response.json()["id"]
    cookie = "; ".join(f"{c.name}={c.value}" for c in session.cookies)
    uri = f"ws://{HOST}:8888/api/kernels/{kernel_id}/channels"
    async with websockets.connect(uri, additional_headers={"Cookie": cookie}, open_timeout=8, ping_interval=None) as ws:
        message = {
            "header": {"msg_id": str(uuid.uuid4()), "username": "gladis", "session": str(uuid.uuid4()), "msg_type": "execute_request", "version": "5.3"},
            "parent_header": {}, "metadata": {},
            "content": {"code": code, "silent": False, "store_history": False, "user_expressions": {}, "allow_stdin": False, "stop_on_error": True},
            "channel": "shell", "buffers": [],
        }
        await ws.send(json.dumps(message))
        deadline = time.time() + 90
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
        raise TimeoutError("audio playback timed out")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("audio", type=Path)
    parser.add_argument("--gain", type=float, default=3.0)
    args = parser.parse_args()
    data = args.audio.read_bytes()
    session, xsrf = login()
    remote = f"beep_bridge/audio/{args.audio.name}"
    headers = {"X-XSRFToken": xsrf, "Content-Type": "application/json"}
    directory = session.put(
        f"{BASE}/api/contents/beep_bridge/audio",
        headers=headers,
        json={"type": "directory"},
        timeout=15,
    )
    if directory.status_code not in (200, 201, 409):
        directory.raise_for_status()
    response = session.put(
        f"{BASE}/api/contents/{remote}",
        headers=headers,
        json={"type": "file", "format": "base64", "content": base64.b64encode(data).decode("ascii")},
        timeout=30,
    )
    response.raise_for_status()
    pi_path = f"/home/pi/{remote}"
    code = f'''\nimport subprocess\nsource = {pi_path!r}\nout = '/tmp/gladis_tts_playback.wav'\nconvert = subprocess.run(['ffmpeg', '-y', '-i', source, '-af', 'volume={args.gain}', '-ar', '24000', '-ac', '1', out], text=True, capture_output=True)\nprint('ffmpeg', convert.returncode)\nif convert.returncode:\n    print(convert.stderr)\n    raise SystemExit(convert.returncode)\nplay = subprocess.run(['aplay', '-D', 'plughw:1,0', out], text=True, capture_output=True)\nprint('aplay', play.returncode)\nprint(play.stdout)\nprint(play.stderr)\nif play.returncode:\n    raise SystemExit(play.returncode)\n'''
    asyncio.run(execute(session, xsrf, code))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
