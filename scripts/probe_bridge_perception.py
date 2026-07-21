#!/usr/bin/env python3
"""Measure BEEP camera, bridge, microphone, and Pi resource baselines.

The probe is observation-only: it aborts if the bridge reports movement and
never invokes a movement endpoint. Onboard checks execute through BEEP's
Jupyter kernel so SSH is not required.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import re
import statistics
import time
import uuid
from pathlib import Path
from typing import Any

import requests
import websockets

ENDPOINTS = ("status", "scan", "local_map", "frame.jpg")
RESULT_START = "__BEEP_PERCEPTION_PROBE_JSON_START__"
RESULT_END = "__BEEP_PERCEPTION_PROBE_JSON_END__"


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * fraction))))
    return ordered[index]


def summarize_latencies(values: list[float]) -> dict[str, float | int | None]:
    p95 = percentile(values, 0.95)
    return {
        "samples": len(values),
        "min_ms": round(min(values), 2) if values else None,
        "median_ms": round(statistics.median(values), 2) if values else None,
        "p95_ms": round(p95, 2) if p95 is not None else None,
        "max_ms": round(max(values), 2) if values else None,
    }


def jpeg_dimensions(data: bytes) -> tuple[int, int] | None:
    """Return JPEG dimensions without requiring Pillow/OpenCV."""
    if len(data) < 4 or data[:2] != b"\xff\xd8":
        return None
    offset = 2
    while offset + 9 <= len(data):
        if data[offset] != 0xFF:
            offset += 1
            continue
        while offset < len(data) and data[offset] == 0xFF:
            offset += 1
        if offset >= len(data):
            break
        marker = data[offset]
        offset += 1
        if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            continue
        if offset + 2 > len(data):
            break
        segment_length = int.from_bytes(data[offset : offset + 2], "big")
        if segment_length < 2 or offset + segment_length > len(data):
            break
        if marker in {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}:
            height = int.from_bytes(data[offset + 3 : offset + 5], "big")
            width = int.from_bytes(data[offset + 5 : offset + 7], "big")
            return width, height
        offset += segment_length
    return None


def bridge_probe(host: str, port: int, samples: int, timeout: float) -> dict[str, Any]:
    base = f"http://{host}:{port}"
    result: dict[str, Any] = {"base_url": base, "samples_requested": samples, "endpoints": {}}

    status_response = requests.get(f"{base}/status", timeout=timeout)
    status_response.raise_for_status()
    status = status_response.json()
    if bool(status.get("moving")):
        raise RuntimeError("bridge reports moving=true; refusing perception probe")
    result["preflight"] = {
        "moving": status.get("moving"),
        "last_command": status.get("last_command"),
        "version": status.get("version"),
        "scan_age_s": status.get("scan_age_s"),
        "pose_source": (status.get("pose") or {}).get("source"),
        "pose_valid": (status.get("slam") or {}).get("pose_valid"),
        "map_age_s": (status.get("slam") or {}).get("map_age_s"),
    }

    for endpoint in ENDPOINTS:
        latencies: list[float] = []
        attempt_latencies: list[float] = []
        sizes: list[int] = []
        errors: list[str] = []
        content_types: set[str] = set()
        latest_body = b""
        for _ in range(samples):
            started = time.perf_counter()
            try:
                response = requests.get(f"{base}/{endpoint}", timeout=timeout)
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                response.raise_for_status()
                latencies.append(elapsed_ms)
                latest_body = response.content
                sizes.append(len(latest_body))
                content_types.add(response.headers.get("content-type", ""))
            except Exception as exc:  # Probe records partial failure instead of hiding it.
                errors.append(f"{type(exc).__name__}: {exc}")
            finally:
                attempt_latencies.append((time.perf_counter() - started) * 1000.0)
        entry: dict[str, Any] = {
            "latency": summarize_latencies(latencies),
            "attempt_latency": summarize_latencies(attempt_latencies),
            "bytes_median": round(statistics.median(sizes)) if sizes else None,
            "content_types": sorted(content_types),
            "errors": errors,
        }
        if endpoint == "frame.jpg" and latest_body:
            dims = jpeg_dimensions(latest_body)
            entry["jpeg_valid"] = latest_body[:2] == b"\xff\xd8" and latest_body[-2:] == b"\xff\xd9"
            entry["dimensions"] = list(dims) if dims else None
        result["endpoints"][endpoint] = entry
    return result


def jupyter_login(host: str, password: str, timeout: float) -> tuple[requests.Session, str, str]:
    base = f"http://{host}:8888"
    session = requests.Session()
    response = session.get(f"{base}/login?next=%2Flab", timeout=timeout)
    response.raise_for_status()
    xsrf = session.cookies.get("_xsrf") or ""
    login_response = session.post(
        f"{base}/login?next=%2Flab",
        data={"password": password, "_xsrf": xsrf},
        headers={"Referer": f"{base}/login?next=%2Flab"},
        timeout=timeout,
        allow_redirects=False,
    )
    login_response.raise_for_status()
    match = re.search(r'(username-[^=]+)="?([^";]+)', login_response.headers.get("Set-Cookie", ""))
    if match:
        session.cookies.set(match.group(1), match.group(2), domain=host, path="/")
    api = session.get(f"{base}/api", timeout=timeout)
    api.raise_for_status()
    return session, xsrf, base


def remote_probe_code(samples: int, mic_seconds: float, preferred_mic: str) -> str:
    return f'''
import audioop
import glob
import importlib.util
import json
import os
import pathlib
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request
import wave

result = {{
    "captured_at": time.time(),
    "hostname": platform.node(),
    "platform": platform.platform(),
    "python": sys.version,
}}
modules = ["cv2", "PIL", "numpy", "requests", "websockets", "webrtcvad", "sounddevice", "rclpy"]
result["modules"] = {{name: importlib.util.find_spec(name) is not None for name in modules}}
result["video_devices"] = sorted(glob.glob("/dev/video*"))
result["audio_devices"] = sorted(glob.glob("/dev/snd/*"))

def run(command, timeout=10):
    try:
        completed = subprocess.run(command, text=True, capture_output=True, timeout=timeout)
        return {{"returncode": completed.returncode, "stdout": completed.stdout.strip(), "stderr": completed.stderr.strip()}}
    except Exception as exc:
        return {{"returncode": None, "stdout": "", "stderr": f"{{type(exc).__name__}}: {{exc}}"}}

result["arecord_list"] = run(["arecord", "-l"])
result["usb_devices"] = run(["lsusb"])
result["camera_listeners"] = run(["bash", "-lc", "ss -ltnp | grep -E ':(6500|8766)\\\\b' || true"])
result["camera_access"] = {{
    "identity": run(["id"]),
    "device": run(["stat", "-c", "%A %U %G %t:%T %n", "/dev/video0"]),
    "users": run(["bash", "-lc", "fuser -v /dev/video0 2>&1 || true"]),
    "owners": run(["bash", "-lc", "for p in $(fuser /dev/video0 2>/dev/null); do ps -ww -p $p -o pid,ppid,etimes,cmd; done"]),
}}
result["v4l2_formats"] = run(["v4l2-ctl", "--device=/dev/video0", "--list-formats-ext"], 15)
v4l_frame = pathlib.Path(tempfile.gettempdir()) / "beep_perception_probe_v4l2.jpg"
result["direct_v4l2"] = run([
    "v4l2-ctl", "--device=/dev/video0",
    "--set-fmt-video=width=640,height=480,pixelformat=MJPG",
    "--stream-mmap=3", "--stream-count=1", f"--stream-to={{v4l_frame}}",
], 20)
if v4l_frame.exists():
    raw_frame = v4l_frame.read_bytes()
    result["direct_v4l2"].update({{
        "bytes": len(raw_frame),
        "jpeg_start": raw_frame[:2].hex(),
        "jpeg_end": raw_frame[-2:].hex() if len(raw_frame) >= 2 else None,
    }})
    v4l_frame.unlink(missing_ok=True)
if result["modules"].get("cv2"):
    direct_camera_code = r"""
import cv2, json, time
capture = cv2.VideoCapture(0, cv2.CAP_V4L2)
answer = {{"opened": bool(capture.isOpened()), "reads": []}}
frame = None
try:
    for _ in range(3):
        started = time.perf_counter()
        ok, candidate = capture.read()
        elapsed = (time.perf_counter() - started) * 1000.0
        answer["reads"].append({{"ok": bool(ok), "elapsed_ms": round(elapsed, 2)}})
        if ok and candidate is not None:
            frame = candidate
    if frame is not None:
        encoded_ok, encoded = cv2.imencode('.jpg', frame)
        answer.update({{
            "width": int(frame.shape[1]),
            "height": int(frame.shape[0]),
            "channels": int(frame.shape[2]) if len(frame.shape) == 3 else 1,
            "mean_bgr": [round(float(v), 2) for v in frame.mean(axis=(0, 1))],
            "jpeg_encode_ok": bool(encoded_ok),
            "jpeg_bytes": int(encoded.size) if encoded_ok else None,
        }})
finally:
    capture.release()
print(json.dumps(answer, sort_keys=True))
"""
    result["direct_camera"] = run([sys.executable, "-c", direct_camera_code], 25)
else:
    result["direct_camera"] = {{"returncode": None, "stdout": "", "stderr": "cv2 unavailable"}}
result["services"] = {{name: run(["systemctl", "is-active", name], 5)["stdout"] for name in ["beep-bridge", "beep-cartographer", "beep-occupancy-grid", "tailscaled"]}}
result["top_processes"] = run(["bash", "-lc", "ps -eo pid,comm,pcpu,pmem,args --sort=-pcpu | head -12"])

meminfo = {{}}
for line in pathlib.Path("/proc/meminfo").read_text().splitlines():
    key, value = line.split(":", 1)
    if key in {{"MemTotal", "MemAvailable", "SwapTotal", "SwapFree"}}:
        meminfo[key] = int(value.strip().split()[0])
disk = shutil.disk_usage("/home/pi")
result["resources"] = {{
    "loadavg": [float(v) for v in pathlib.Path("/proc/loadavg").read_text().split()[:3]],
    "uptime_s": float(pathlib.Path("/proc/uptime").read_text().split()[0]),
    "memory_kib": meminfo,
    "disk_bytes": {{"total": disk.total, "used": disk.used, "free": disk.free}},
}}
thermal = pathlib.Path("/sys/class/thermal/thermal_zone0/temp")
result["resources"]["temperature_c"] = round(float(thermal.read_text().strip()) / 1000.0, 1) if thermal.exists() else None

# Measure onboard localhost bridge latency without any movement calls.
endpoints = ["status", "scan", "local_map", "frame.jpg"]
local = {{}}
for endpoint in endpoints:
    latencies = []
    attempt_latencies = []
    sizes = []
    errors = []
    latest = b""
    for _ in range({samples}):
        started = time.perf_counter()
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:8766/{{endpoint}}", timeout=8) as response:
                latest = response.read()
            latencies.append((time.perf_counter() - started) * 1000.0)
            sizes.append(len(latest))
        except Exception as exc:
            errors.append(f"{{type(exc).__name__}}: {{exc}}")
        finally:
            attempt_latencies.append(round((time.perf_counter() - started) * 1000.0, 3))
    local[endpoint] = {{
        "samples": len(latencies),
        "latency_ms": [round(v, 3) for v in latencies],
        "attempt_latency_ms": attempt_latencies,
        "sizes": sizes,
        "errors": errors,
        "jpeg_magic": latest[:2].hex() if endpoint == "frame.jpg" and latest else None,
    }}
result["onboard_bridge"] = local

# Select the preferred capture path, then fall back to the first USB-named ALSA card.
preferred = {preferred_mic!r}
selected = preferred
listing = result["arecord_list"].get("stdout", "")
if preferred == "auto":
    match = re.search(r"card (\\d+):[^\\n]*(?:USB|PnP)", listing, re.IGNORECASE)
    selected = f"plughw:{{match.group(1)}},0" if match else "plughw:2,0"
recording = pathlib.Path(tempfile.gettempdir()) / "beep_perception_probe.wav"
record = run(["arecord", "-q", "-D", selected, "-f", "S16_LE", "-r", "16000", "-c", "1", "-d", str(max(1, int(round({mic_seconds})))), str(recording)], timeout=max(10, int({mic_seconds}) + 8))
mic = {{"device": selected, "duration_requested_s": {mic_seconds}, "record": record}}
if record.get("returncode") == 0 and recording.exists():
    try:
        with wave.open(str(recording), "rb") as wav:
            frames = wav.readframes(wav.getnframes())
            mic.update({{
                "channels": wav.getnchannels(),
                "sample_rate_hz": wav.getframerate(),
                "sample_width_bytes": wav.getsampwidth(),
                "frames": wav.getnframes(),
                "duration_s": round(wav.getnframes() / float(wav.getframerate()), 3),
                "rms": audioop.rms(frames, wav.getsampwidth()) if frames else 0,
                "peak": audioop.max(frames, wav.getsampwidth()) if frames else 0,
                "bytes": recording.stat().st_size,
            }})
    except Exception as exc:
        mic["analysis_error"] = f"{{type(exc).__name__}}: {{exc}}"
try:
    recording.unlink(missing_ok=True)
except Exception:
    pass
result["microphone"] = mic

# Sample whole-system CPU after active probing has finished. The kernel sleeps
# between /proc/stat reads, producing a useful baseline instead of measuring
# its own short diagnostic burst.
def cpu_counters():
    values = [int(value) for value in pathlib.Path("/proc/stat").read_text().splitlines()[0].split()[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return idle, sum(values)

idle_a, total_a = cpu_counters()
time.sleep(2.0)
idle_b, total_b = cpu_counters()
total_delta = max(1, total_b - total_a)
idle_delta = max(0, idle_b - idle_a)
temperature = pathlib.Path("/sys/class/thermal/thermal_zone0/temp")
result["post_probe_resources"] = {{
    "sample_window_s": 2.0,
    "cpu_utilization_pct": round(100.0 * (1.0 - idle_delta / total_delta), 1),
    "loadavg": [float(v) for v in pathlib.Path("/proc/loadavg").read_text().split()[:3]],
    "memory_available_kib": meminfo.get("MemAvailable"),
    "temperature_c": round(float(temperature.read_text().strip()) / 1000.0, 1) if temperature.exists() else None,
}}

print({RESULT_START!r})
print(json.dumps(result, sort_keys=True))
print({RESULT_END!r})
'''


async def execute_remote_probe(
    host: str,
    password: str,
    samples: int,
    mic_seconds: float,
    preferred_mic: str,
    timeout: float,
) -> dict[str, Any]:
    session, xsrf, base = jupyter_login(host, password, timeout)
    headers = {"X-XSRFToken": xsrf, "Content-Type": "application/json"}
    response = session.post(f"{base}/api/kernels", json={"name": "python3"}, headers=headers, timeout=timeout)
    response.raise_for_status()
    kernel_id = response.json()["id"]
    cookie = "; ".join(f"{cookie.name}={cookie.value}" for cookie in session.cookies)
    uri = f"ws://{host}:8888/api/kernels/{kernel_id}/channels"
    output: list[str] = []
    try:
        async with websockets.connect(uri, additional_headers={"Cookie": cookie}, open_timeout=timeout, ping_interval=None) as ws:
            message = {
                "header": {
                    "msg_id": str(uuid.uuid4()),
                    "username": "gladis",
                    "session": str(uuid.uuid4()),
                    "msg_type": "execute_request",
                    "version": "5.3",
                },
                "parent_header": {},
                "metadata": {},
                "content": {
                    "code": remote_probe_code(samples, mic_seconds, preferred_mic),
                    "silent": False,
                    "store_history": False,
                    "user_expressions": {},
                    "allow_stdin": False,
                    "stop_on_error": True,
                },
                "channel": "shell",
                "buffers": [],
            }
            await ws.send(json.dumps(message))
            # Each camera sample may consume the bridge's full upstream timeout.
            # Budget for that deliberately so a failed camera doesn't hide the
            # microphone and resource results that follow it.
            deadline = time.time() + max(60.0, mic_seconds + 30.0, samples * 10.0 + mic_seconds + 20.0)
            while time.time() < deadline:
                incoming = json.loads(await asyncio.wait_for(ws.recv(), timeout=max(1.0, deadline - time.time())))
                message_type = incoming.get("msg_type")
                if message_type == "stream":
                    output.append(incoming.get("content", {}).get("text", ""))
                elif message_type == "error":
                    raise RuntimeError(json.dumps(incoming.get("content", {})))
                elif message_type == "execute_reply":
                    break
            else:
                raise TimeoutError("onboard perception probe timed out")
    finally:
        try:
            session.delete(f"{base}/api/kernels/{kernel_id}", headers=headers, timeout=timeout)
        except Exception:
            pass

    text = "".join(output)
    match = re.search(re.escape(RESULT_START) + r"\s*(\{.*\})\s*" + re.escape(RESULT_END), text, re.DOTALL)
    if not match:
        raise RuntimeError(f"probe returned no JSON payload: {text[-2000:]}")
    return json.loads(match.group(1))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("BEEP_HOST", "beep.tailb08b32.ts.net"))
    parser.add_argument("--bridge-port", type=int, default=8766)
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--mic-seconds", type=float, default=3.0)
    parser.add_argument("--mic-device", default=os.environ.get("BEEP_MIC_DEVICE", "auto"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    if args.samples < 1 or args.samples > 50:
        parser.error("--samples must be in 1..50")
    if args.mic_seconds < 1 or args.mic_seconds > 10:
        parser.error("--mic-seconds must be in 1..10")

    password = os.environ.get("BEEP_JUPYTER_PASSWORD")
    if not password:
        parser.error("BEEP_JUPYTER_PASSWORD must be set in the private environment")
    report: dict[str, Any] = {
        "schema_version": "1.0",
        "measured_at": time.time(),
        "host": args.host,
        "observation_only": True,
        "host_bridge": bridge_probe(args.host, args.bridge_port, args.samples, args.timeout),
    }
    report["onboard"] = asyncio.run(
        execute_remote_probe(args.host, password, args.samples, args.mic_seconds, args.mic_device, args.timeout)
    )

    encoded = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n")
        print(f"wrote {args.output}")
    else:
        print(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
