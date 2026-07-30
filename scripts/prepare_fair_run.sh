#!/usr/bin/env bash
set -Eeuo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BRIDGE_URL="${BEEP_BRIDGE_URL:-http://192.168.8.88:8766}"
MARKER="${BEEP_FAIR_READY_FILE:-/tmp/beep-fair-ready.json}"

cd "${REPO_DIR}"
curl -fsS --max-time 4 "${BRIDGE_URL}/stop" >/dev/null
python3 scripts/jupyter_reset_slam.py

python3 - "${BRIDGE_URL}" "${MARKER}" <<'PY'
import json
import math
import os
import sys
import time
import urllib.request

base, marker = sys.argv[1:]

def status():
    with urllib.request.urlopen(base + "/status", timeout=3) as response:
        return json.load(response)

for _ in range(60):
    try:
        current = status()
        slam = current.get("slam") or {}
        if (not current.get("moving") and slam.get("usable") is True
                and current.get("scan_age_s") is not None
                and float(current["scan_age_s"]) <= 0.5):
            break
    except Exception:
        pass
    time.sleep(0.5)
else:
    raise SystemExit("Preparation failed: bridge/SLAM did not become usable")

samples = []
for _ in range(12):
    samples.append(status())
    time.sleep(0.25)

first, last = samples[0], samples[-1]
p0, p1 = first["pose"], last["pose"]
distance = math.hypot(float(p1["x"]) - float(p0["x"]), float(p1["y"]) - float(p0["y"]))
yaw_delta = abs(math.atan2(
    math.sin(float(p1["yaw"]) - float(p0["yaw"])),
    math.cos(float(p1["yaw"]) - float(p0["yaw"])),
))
sectors = last.get("sectors") or {}
required = ("front", "front_left", "front_right", "left", "right", "rear")
checks = {
    "stopped": not any(item.get("moving") for item in samples),
    "slam_usable": all((item.get("slam") or {}).get("usable") is True for item in samples),
    "pose_stable": distance <= 0.03 and math.degrees(yaw_delta) <= 3.0,
    "clearance": all(sectors.get(name) is not None and float(sectors[name]) >= 0.15 for name in required),
}
if not all(checks.values()):
    raise SystemExit("Preparation failed: " + json.dumps(checks, sort_keys=True))

payload = {
    "prepared_at": time.time(),
    "bridge_url": base,
    "version": last.get("version"),
    "pose": last.get("pose"),
    "sectors": sectors,
    "checks": checks,
}
with open(marker, "w", encoding="utf-8") as handle:
    json.dump(payload, handle, indent=2, sort_keys=True)
os.chmod(marker, 0o600)
print(json.dumps(payload, indent=2, sort_keys=True))
print("BEEP is prepared, stopped, and ready for the fast voice trigger.")
PY
