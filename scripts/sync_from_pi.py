#!/usr/bin/env python3
"""Sync useful BEEP Pi files into this repository.

Pulls:
- live bridge source into `beep_bridge/beep_bridge.py`
- selected runtime artifacts into `captures/<timestamp>/`
- selected ROS/Cartographer map artifacts into `assets/ros_maps/`

Generated captures remain git-ignored by default. ROS map artifacts are intended to be committed when useful.
"""
from __future__ import annotations

import base64
import json
import os
import re
import time
from pathlib import Path

import requests

HOST = os.environ.get("BEEP_HOST", "192.168.8.88")
PASSWORD = os.environ.get("BEEP_JUPYTER_PASSWORD", "yahboom")
BASE = f"http://{HOST}:8888"
REPO = Path(__file__).resolve().parents[1]


def login() -> requests.Session:
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
    return s


def contents(s: requests.Session, path: str) -> dict:
    r = s.get(f"{BASE}/api/contents/{path}", timeout=20)
    r.raise_for_status()
    return r.json()


def write_file_from_contents(data: dict, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    content = data.get("content") or ""
    if data.get("format") == "base64":
        dest.write_bytes(base64.b64decode(content))
    else:
        dest.write_text(content)


def fetch_file(s: requests.Session, remote: str, dest: Path) -> None:
    data = contents(s, remote)
    if data.get("type") != "file":
        raise RuntimeError(f"not a file: {remote}")
    write_file_from_contents(data, dest)
    print(f"pulled {remote} -> {dest.relative_to(REPO)}")


def main() -> int:
    s = login()
    stamp = time.strftime("pi_live_%Y%m%d_%H%M%S")
    capture = REPO / "captures" / stamp

    fetch_file(s, "beep_bridge/beep_bridge.py", REPO / "beep_bridge" / "beep_bridge.py")

    for remote in [
        "beep_bridge/logs/bridge.log",
        "Version.txt",
    ]:
        try:
            fetch_file(s, remote, capture / Path(remote).name)
        except Exception as e:
            print(f"skip {remote}: {e!r}")

    # Runtime bridge maps are captures, not source.
    try:
        maps = contents(s, "beep_bridge/maps").get("content", [])
        for item in maps:
            if item.get("type") == "file" and item.get("path", "").endswith(".json"):
                fetch_file(s, item["path"], capture / "bridge_maps" / Path(item["path"]).name)
    except Exception as e:
        print(f"skip bridge maps: {e!r}")

    # ROS/Cartographer artifacts can be source-controlled as selected historical artifacts.
    ros_dir = REPO / "assets" / "ros_maps"
    ros_dir.mkdir(parents=True, exist_ok=True)
    inventory = []
    try:
        for item in contents(s, "gladis_maps").get("content", []):
            name = Path(item.get("path", "")).name
            if item.get("type") != "file":
                continue
            if name.endswith(".pid") or name in {"current_ts", "current_base"}:
                continue
            if not (name.endswith(".yaml") or name.endswith(".pgm") or name.endswith(".pbstream")):
                continue
            dest = ros_dir / name
            fetch_file(s, item["path"], dest)
            inventory.append({k: item.get(k) for k in ["path", "size", "last_modified"]})
        (ros_dir / "inventory.json").write_text(json.dumps(inventory, indent=2))
    except Exception as e:
        print(f"skip ROS maps: {e!r}")

    print(f"capture dir: {capture.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
