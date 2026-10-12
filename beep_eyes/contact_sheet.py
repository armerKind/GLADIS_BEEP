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

MAX_STATUS_BYTES = 512 * 1024
MAX_MAP_BYTES = 8 * 1024 * 1024
MAX_FRAME_BYTES = 2 * 1024 * 1024


def _read_bounded(response: Any, limit: int, label: str) -> bytes:
    data = response.read(limit + 1)
    if len(data) > limit:
        raise ValueError(f"{label} is too large")
    return data


@dataclass(frozen=True)
class CapturedFrame:
    frame_id: str
    captured_at: float
    jpeg: bytes

    def metadata(self) -> dict[str, Any]:
        return {"frame_id": self.frame_id, "captured_at": self.captured_at, "bytes": len(self.jpeg)}


def fetch_json(base_url: str, path: str, timeout_s: float = 8.0,
              max_bytes: int = MAX_STATUS_BYTES) -> dict[str, Any]:
    with urllib.request.urlopen(base_url.rstrip("/") + path, timeout=timeout_s) as response:
        return json.loads(_read_bounded(response, max_bytes, "bridge JSON response"))


def fetch_slam_map(base_url: str, timeout_s: float = 8.0) -> dict[str, Any]:
    """Fetch the bounded authoritative Cartographer grid for inference context."""
    return fetch_json(base_url, "/slam_map", timeout_s, max_bytes=MAX_MAP_BYTES)


def fetch_jpeg(base_url: str, timeout_s: float = 10.0) -> bytes:
    with urllib.request.urlopen(base_url.rstrip("/") + f"/frame.jpg?timeout={min(timeout_s, 8.0):g}", timeout=timeout_s) as response:
        data = _read_bounded(response, MAX_FRAME_BYTES, "bridge JPEG")
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


def capture_sequence(base_url: str, count: int = 4, interval_s: float = 1.5, timeout_s: float = 10.0,
                     allow_moving: bool = False) -> tuple[list[CapturedFrame], dict[str, Any]]:
    """Capture frames and return the latest bridge status.

    This function never invokes movement. Stationary capture remains the default;
    moving callers must opt in explicitly and use a bridge camera path that does
    not alter the active motor lease.
    """
    if count < 2 or count > 9:
        raise ValueError("count must be within [2, 9]")
    if interval_s < 0 or interval_s > 10:
        raise ValueError("interval_s must be within [0, 10]")
    status = fetch_json(base_url, "/status", timeout_s)
    if not allow_moving and (bool(status.get("moving")) or status.get("motion_lease_id") is not None):
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
    if not allow_moving and (bool(final_status.get("moving")) or final_status.get("motion_lease_id") is not None):
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
        if path.stat().st_size > MAX_FRAME_BYTES:
            raise ValueError(f"replay JPEG is too large: {path.name}")
        data = path.read_bytes()
        if data[:2] != b"\xff\xd8" or data[-2:] != b"\xff\xd9":
            raise ValueError(f"invalid replay JPEG: {path.name}")
        frames.append(CapturedFrame(frame_id, float(captured_at), data))
    if any(current.captured_at <= previous.captured_at for previous, current in zip(frames, frames[1:])):
        raise ValueError("replay frame timestamps must be strictly chronological")
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


def _valid_frame_pose(status: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(status, dict):
        return None
    slam = status.get("slam") or {}
    pose = status.get("pose") or {}
    if slam.get("pose_valid") is not True or slam.get("usable") is not True:
        return None
    try:
        values = [float(pose[key]) for key in ("x", "y", "yaw")]
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    return {"x": values[0], "y": values[1], "yaw": values[2],
            "confidence": pose.get("confidence"), "source": pose.get("source")}


def build_contact_sheet(
    frames: list[CapturedFrame],
    *,
    columns: int = 2,
    panel_size: tuple[int, int] = (480, 360),
    label_height: int = 30,
    quality: int = 88,
    frame_statuses: list[dict[str, Any]] | None = None,
) -> bytes:
    """Render chronological panels; optional status labels expose pose/view yaw per frame."""
    if not frames:
        raise ValueError("at least one frame is required")
    if columns < 1 or columns > 4:
        raise ValueError("columns must be within [1, 4]")
    if frame_statuses is not None and len(frame_statuses) != len(frames):
        raise ValueError("frame_statuses length must match frames")
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
        label = f"{frame.frame_id}  t+{frame.captured_at - first_at:.1f}s"
        draw.text((x + 8, y + 2), label, fill="white")
        if frame_statuses is not None and label_height >= 30:
            pose = _valid_frame_pose(frame_statuses[index])
            if pose is None:
                pose_label = "pose invalid — view direction unknown"
            else:
                pose_label = (f"x={pose['x']:+.2f} y={pose['y']:+.2f} "
                              f"view yaw={math.degrees(pose['yaw']):+.0f}°")
            draw.text((x + 8, y + 16), pose_label, fill="#8ee6ff")
    output = io.BytesIO()
    sheet.save(output, format="JPEG", quality=quality, optimize=True)
    return output.getvalue()


def summarize_navigation_context(status: dict[str, Any], slam_map: dict[str, Any] | None) -> dict[str, Any]:
    """Small model-facing summary; the full occupancy array remains a local artifact."""
    slam_map = slam_map if isinstance(slam_map, dict) else {}
    pose = _valid_frame_pose(status)
    map_ok = False
    map_reason = slam_map.get("reason") or slam_map.get("error")
    width = height = None
    resolution = None
    origin_x = origin_y = origin_yaw = None
    try:
        width, height = int(slam_map["width"]), int(slam_map["height"])
        resolution = float(slam_map["resolution_m"])
        origin_x = float(slam_map.get("origin_x", 0.0))
        origin_y = float(slam_map.get("origin_y", 0.0))
        origin_yaw = float(slam_map.get("origin_yaw", 0.0))
        data = slam_map["data"]
        map_ok = (slam_map.get("ok") is True and 0 < width * height <= 4_000_000
                  and isinstance(data, (list, tuple)) and len(data) == width * height
                  and all(math.isfinite(value) for value in (resolution, origin_x, origin_y, origin_yaw))
                  and resolution > 0)
    except (KeyError, TypeError, ValueError):
        map_ok = False
    map_age = slam_map.get("map_age_s") if map_ok else None
    map_fresh = (isinstance(map_age, (int, float)) and math.isfinite(float(map_age))
                 and 0.0 <= float(map_age) <= 2.5)
    return {
        "map_source": "cartographer_occupancy_grid",
        "map_available": map_ok,
        "map_fresh": map_fresh,
        "map_reason": None if map_ok else map_reason or "missing_or_invalid_grid",
        "map_frame_id": slam_map.get("frame_id", "map") if map_ok else None,
        "map_width": width if map_ok else None,
        "map_height": height if map_ok else None,
        "map_resolution_m": resolution if map_ok else None,
        "map_origin_x": origin_x if map_ok else None,
        "map_origin_y": origin_y if map_ok else None,
        "map_origin_yaw_deg": round(math.degrees(origin_yaw), 1) if map_ok and origin_yaw is not None else None,
        "map_age_s": slam_map.get("map_age_s") if map_ok else None,
        "known_cells": slam_map.get("known_cells") if map_ok else None,
        "occupied_cells": slam_map.get("occupied_cells") if map_ok else None,
        "pose_valid": pose is not None,
        "pose": pose,
        "view_heading_deg": round(math.degrees(pose["yaw"]), 1) if pose is not None else None,
        "view_axis": "front-camera/body-forward approximation; optical pan/tilt and lens FOV are uncalibrated",
        "pose_guard_reason": (status.get("slam") or {}).get("pose_guard_reason"),
    }


def _map_pixel(grid: dict[str, Any], x: float, y: float) -> tuple[float, float] | None:
    width, height = int(grid["width"]), int(grid["height"])
    resolution = float(grid["resolution_m"])
    ox, oy = float(grid.get("origin_x", 0.0)), float(grid.get("origin_y", 0.0))
    origin_yaw = float(grid.get("origin_yaw", 0.0))
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)
    dx, dy = float(x) - ox, float(y) - oy
    local_x, local_y = c * dx + s * dy, -s * dx + c * dy
    px, py = local_x / resolution, height - local_y / resolution
    if not (0 <= px < width and 0 <= py < height):
        return None
    return px, py


def render_slam_map(slam_map: dict[str, Any], frame_statuses: list[dict[str, Any]],
                    current_status: dict[str, Any]) -> Image.Image | None:
    """Render ROS occupancy, per-frame camera poses, and latest camera heading."""
    summary = summarize_navigation_context(current_status, slam_map)
    if not summary["map_available"]:
        return None
    width, height = int(slam_map["width"]), int(slam_map["height"])
    values = slam_map["data"]
    pixels: list[tuple[int, int, int]] = []
    for y in reversed(range(height)):
        for value in values[y * width:(y + 1) * width]:
            value = int(value)
            pixels.append((150, 150, 150) if value < 0 else
                          ((18, 18, 18) if value >= 50 else (248, 248, 248)))
    image = Image.new("RGB", (width, height))
    image.putdata(pixels)
    draw = ImageDraw.Draw(image)
    points: list[tuple[float, float]] = []
    for index, frame_status in enumerate(frame_statuses):
        pose = _valid_frame_pose(frame_status)
        point = _map_pixel(slam_map, pose["x"], pose["y"]) if pose is not None else None
        if point is None:
            continue
        points.append(point)
        r = max(1, min(4, int(round(min(width, height) / 90))))
        color = (0, 145, 210) if index + 1 < len(frame_statuses) else (230, 45, 35)
        draw.ellipse((point[0] - r, point[1] - r, point[0] + r, point[1] + r), fill=color)
        if len(frame_statuses) <= 9:
            draw.text((point[0] + r + 1, point[1] - r - 1), str(index + 1), fill=(0, 70, 130))
    if len(points) > 1:
        draw.line(points, fill=(0, 145, 210), width=max(1, min(3, width // 100)))
    pose = _valid_frame_pose(current_status)
    if pose is not None:
        center = _map_pixel(slam_map, pose["x"], pose["y"])
        if center is not None:
            origin_yaw = float(slam_map.get("origin_yaw", 0.0))
            heading = pose["yaw"] - origin_yaw
            length = max(5.0, min(width, height) * 0.10)
            dx, dy = math.cos(heading) * length, -math.sin(heading) * length
            tip = (center[0] + dx, center[1] + dy)
            draw.line((center, tip), fill=(230, 45, 35), width=max(1, min(3, width // 80)))
            wing = max(3.0, length * 0.30)
            nx, ny = -dy / length, dx / length
            base = (center[0] - dx * 0.20, center[1] - dy * 0.20)
            draw.polygon([tip,
                          (base[0] + nx * wing, base[1] + ny * wing),
                          (base[0] - nx * wing, base[1] - ny * wing)], fill=(230, 45, 35))
    return image


def build_navigation_contact_sheet(
    frames: list[CapturedFrame],
    *,
    frame_statuses: list[dict[str, Any]],
    current_status: dict[str, Any],
    slam_map: dict[str, Any] | None,
    columns: int = 3,
    panel_size: tuple[int, int] = (480, 360),
    label_height: int = 40,
    context_height: int = 360,
    quality: int = 90,
) -> bytes:
    """Build one LLM image containing temporal camera panels plus mapped pose/view context."""
    if len(frame_statuses) != len(frames):
        raise ValueError("frame_statuses length must match frames")
    camera_bytes = build_contact_sheet(frames, columns=columns, panel_size=panel_size,
                                       label_height=label_height, frame_statuses=frame_statuses)
    camera_sheet = Image.open(io.BytesIO(camera_bytes)).convert("RGB")
    context_height = max(160, int(context_height))
    canvas = Image.new("RGB", (camera_sheet.width, camera_sheet.height + context_height), "#20252b")
    canvas.paste(camera_sheet, (0, 0))
    draw = ImageDraw.Draw(canvas)
    top = camera_sheet.height
    summary = summarize_navigation_context(current_status, slam_map)
    draw.text((12, top + 8), "NAVIGATION CONTEXT — top-down Cartographer map, frame poses and camera heading", fill="white")
    map_area_width = int(canvas.width * 0.48)
    map_box = (map_area_width - 24, context_height - 62)
    rendered_map = render_slam_map(slam_map or {}, frame_statuses, current_status)
    if rendered_map is None:
        draw.rectangle((12, top + 38, map_area_width - 12, top + context_height - 12), fill="#55585c")
        draw.text((24, top + 55), "SLAM MAP UNAVAILABLE", fill="#ffd166")
    else:
        scaled = ImageOps.contain(rendered_map, map_box, method=Image.Resampling.NEAREST)
        map_x = 12 + (map_box[0] - scaled.width) // 2
        map_y = top + 38 + (map_box[1] - scaled.height) // 2
        canvas.paste(scaled, (map_x, map_y))
        draw.rectangle((map_x, map_y, map_x + scaled.width - 1, map_y + scaled.height - 1), outline="#aab3bb")
    text_x = map_area_width + 8
    text_y = top + 42
    pose = summary.get("pose")
    pose_frame = summary.get("map_frame_id") or "frame unknown"
    if pose is None:
        pose_text = "Pose: INVALID — position and view direction withheld"
    else:
        pose_text = (f"Pose: VALID [{pose_frame}]  x={pose['x']:+.2f} m  y={pose['y']:+.2f} m  "
                     f"view yaw={summary['view_heading_deg']:+.0f}°  conf={pose.get('confidence')}")
    grid_text = (f"Cartographer /map [{('fresh' if summary['map_fresh'] else 'STALE/age unknown')}] "
                 f"frame={summary['map_frame_id']}: {summary['map_width']}x{summary['map_height']} @ {summary['map_resolution_m']:.3f} m/cell; "
                 f"origin=({summary['map_origin_x']:+.2f},{summary['map_origin_y']:+.2f}) m, "
                 f"grid yaw={summary['map_origin_yaw_deg']:+.1f}°, age={summary['map_age_s']} s") if summary["map_available"] else f"Cartographer /map: unavailable ({summary['map_reason']})"
    sectors = current_status.get("sectors") or {}
    sector_labels = (("F", "front"), ("FL", "front_left"), ("FR", "front_right"),
                     ("L", "left"), ("R", "right"), ("Rear", "rear"))
    rendered_sectors = []
    for label, key in sector_labels:
        try:
            value = float(sectors[key])
            rendered = f"{value:.2f}" if math.isfinite(value) else "?"
        except (KeyError, TypeError, ValueError):
            rendered = "?"
        rendered_sectors.append(f"{label} {rendered}")
    sector_text = "LiDAR m  " + "  ".join(rendered_sectors)
    lines = [grid_text, pose_text,
             "Red arrow = latest body/camera-forward axis; FOV and camera pan/tilt are uncalibrated.",
             "Blue numbered dots = camera-frame poses in chronological order; line = recent path.",
             "Map legend: white free, black occupied, gray unknown; no marker when pose invalid.",
             sector_text,
             f"SLAM guard: {summary.get('pose_guard_reason') or 'unspecified'}; scan age={current_status.get('scan_age_s')} s"]
    for index, line in enumerate(lines):
        draw.text((text_x, text_y + index * 28), line, fill="#f0f2f4")
    output = io.BytesIO()
    canvas.save(output, format="JPEG", quality=quality, optimize=True)
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
