"""Temporal frame capture and contact-sheet rendering for BEEP eyes."""
from __future__ import annotations

import io
import json
import math
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from PIL import Image, ImageChops, ImageDraw, ImageFilter, ImageOps, ImageStat


@dataclass(frozen=True)
class CapturedFrame:
    frame_id: str
    captured_at: float
    jpeg: bytes

    def metadata(self) -> dict[str, Any]:
        return {"frame_id": self.frame_id, "captured_at": self.captured_at, "bytes": len(self.jpeg)}


def fetch_json(base_url: str, path: str, timeout_s: float = 8.0) -> dict[str, Any]:
    with urllib.request.urlopen(base_url.rstrip("/") + path, timeout=timeout_s) as response:
        return json.loads(response.read())


def fetch_jpeg(base_url: str, timeout_s: float = 10.0) -> bytes:
    with urllib.request.urlopen(base_url.rstrip("/") + f"/frame.jpg?timeout={min(timeout_s, 8.0):g}", timeout=timeout_s) as response:
        data = response.read()
    if len(data) < 4 or data[:2] != b"\xff\xd8" or data[-2:] != b"\xff\xd9":
        raise ValueError("bridge returned an invalid JPEG")
    return data


def _fetch_jpeg_with_retry(base_url: str, timeout_s: float, attempts: int = 3) -> bytes:
    last_error: Exception | None = None
    for attempt in range(attempts):
        try:
            return fetch_jpeg(base_url, timeout_s)
        except Exception as exc:
            last_error = exc
            if attempt + 1 < attempts:
                time.sleep(0.35)
    assert last_error is not None
    raise last_error


def capture_sequence(base_url: str, count: int = 4, interval_s: float = 1.5, timeout_s: float = 10.0) -> tuple[list[CapturedFrame], dict[str, Any]]:
    """Capture stationary frames and return the latest bridge status.

    This function never invokes movement. It refuses capture if BEEP reports an
    active movement or lease, keeping the first eyes milestone observation-only.
    """
    if count < 2 or count > 8:
        raise ValueError("count must be within [2, 8]")
    if interval_s < 0 or interval_s > 10:
        raise ValueError("interval_s must be within [0, 10]")
    status = fetch_json(base_url, "/status", timeout_s)
    if bool(status.get("moving")) or status.get("motion_lease_id") is not None:
        raise RuntimeError("BEEP is moving; stationary eyes capture refused")
    # The vendor camera may expose one red/stale frame immediately after its
    # control mode switches. Warm it once and exclude that frame from evidence.
    _fetch_jpeg_with_retry(base_url, timeout_s)
    time.sleep(min(0.5, max(0.2, interval_s)))
    frames: list[CapturedFrame] = []
    for index in range(count):
        captured_at = time.time()
        frames.append(CapturedFrame(f"f{index:02d}", captured_at, _fetch_jpeg_with_retry(base_url, timeout_s)))
        if index + 1 < count:
            time.sleep(interval_s)
    final_status = fetch_json(base_url, "/status", timeout_s)
    if bool(final_status.get("moving")) or final_status.get("motion_lease_id") is not None:
        raise RuntimeError("BEEP moved during eyes capture; evidence discarded")
    return frames, final_status


def load_replay_sequence(directory: Path, count: int | None = None) -> tuple[list[CapturedFrame], dict[str, Any]]:
    """Load chronological frames and optional status from a prior capture."""
    packet_path = directory / "observation_packet.json"
    packet = json.loads(packet_path.read_text()) if packet_path.exists() else {}
    metadata = {item.get("frame_id"): item for item in packet.get("frames", []) if isinstance(item, dict)}
    paths = sorted(directory.glob("f*.jpg"))
    if count is not None:
        paths = paths[:count]
    if len(paths) < 2:
        raise ValueError("replay directory requires at least two f*.jpg frames")
    frames = []
    for index, path in enumerate(paths):
        frame_id = path.stem
        captured_at = metadata.get(frame_id, {}).get("captured_at")
        if not isinstance(captured_at, (int, float)):
            captured_at = float(index)
        data = path.read_bytes()
        if data[:2] != b"\xff\xd8" or data[-2:] != b"\xff\xd9":
            raise ValueError(f"invalid replay JPEG: {path.name}")
        frames.append(CapturedFrame(frame_id, float(captured_at), data))
    status_value = packet.get("bridge_status")
    if isinstance(status_value, dict):
        status: dict[str, Any] = status_value
    else:
        status = {
            "moving": False, "motion_lease_id": None, "scan_age_s": None,
            "sectors": {}, "slam": {"usable": False},
        }
    if bool(status.get("moving")) or status.get("motion_lease_id") is not None:
        raise ValueError("replay packet records active movement and is not valid stationary evidence")
    return frames, status


def build_contact_sheet(
    frames: list[CapturedFrame],
    *,
    columns: int = 2,
    panel_size: tuple[int, int] = (480, 360),
    label_height: int = 30,
    quality: int = 88,
) -> bytes:
    """Render chronological panels with frame IDs and relative timestamps."""
    if not frames:
        raise ValueError("at least one frame is required")
    if columns < 1 or columns > 4:
        raise ValueError("columns must be within [1, 4]")
    panel_width, panel_height = panel_size
    if panel_width < 64 or panel_height < 64:
        raise ValueError("panel dimensions are too small")
    rows = math.ceil(len(frames) / columns)
    sheet = Image.new("RGB", (columns * panel_width, rows * (panel_height + label_height)), "black")
    draw = ImageDraw.Draw(sheet)
    first_at = frames[0].captured_at
    for index, frame in enumerate(frames):
        image = Image.open(io.BytesIO(frame.jpeg)).convert("RGB")
        fitted = ImageOps.fit(image, (panel_width, panel_height), method=Image.Resampling.LANCZOS)
        x = (index % columns) * panel_width
        y = (index // columns) * (panel_height + label_height)
        sheet.paste(fitted, (x, y + label_height))
        draw.text((x + 8, y + 7), f"{frame.frame_id}  t+{frame.captured_at - first_at:.1f}s", fill="white")
    output = io.BytesIO()
    sheet.save(output, format="JPEG", quality=quality, optimize=True)
    return output.getvalue()


def frame_quality(frame: CapturedFrame, previous: CapturedFrame | None = None) -> dict[str, Any]:
    image = Image.open(io.BytesIO(frame.jpeg)).convert("RGB")
    stats = ImageStat.Stat(image)
    gray = image.convert("L")
    edge_stddev = ImageStat.Stat(gray.filter(ImageFilter.FIND_EDGES)).stddev[0]
    result: dict[str, Any] = {
        "width": image.width,
        "height": image.height,
        "mean_rgb": [round(value, 2) for value in stats.mean],
        "brightness": round(ImageStat.Stat(gray).mean[0], 2),
        "contrast": round(ImageStat.Stat(gray).stddev[0], 2),
        "edge_stddev": round(edge_stddev, 2),
        "difference_from_previous": None,
    }
    if previous is not None:
        prior = Image.open(io.BytesIO(previous.jpeg)).convert("RGB").resize(image.size)
        result["difference_from_previous"] = round(sum(ImageStat.Stat(ImageChops.difference(image, prior)).mean) / (3.0 * 255.0), 5)
    return result


def write_capture_artifacts(
    output_dir: Path,
    frames: list[CapturedFrame],
    status: dict[str, Any],
    contact_sheet: bytes,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    frame_files = []
    for frame in frames:
        path = output_dir / f"{frame.frame_id}.jpg"
        path.write_bytes(frame.jpeg)
        frame_files.append(str(path))
    sheet_path = output_dir / "contact_sheet.jpg"
    sheet_path.write_bytes(contact_sheet)
    packet = {
        "schema_version": "1.0",
        "packet_id": output_dir.name,
        "captured_at": frames[-1].captured_at,
        "frame_window_s": round(frames[-1].captured_at - frames[0].captured_at, 3),
        "frames": [
            {**frame.metadata(), "quality": frame_quality(frame, frames[index - 1] if index else None)}
            for index, frame in enumerate(frames)
        ],
        "latest_frame_id": frames[-1].frame_id,
        "bridge_status": status,
        "artifacts": {"contact_sheet": str(sheet_path), "latest_frame": frame_files[-1], "frames": frame_files},
    }
    (output_dir / "observation_packet.json").write_text(json.dumps(packet, indent=2, sort_keys=True))
    return packet
