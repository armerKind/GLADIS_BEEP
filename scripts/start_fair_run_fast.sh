#!/usr/bin/env bash
set -Eeuo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BRIDGE_URL="${BEEP_BRIDGE_URL:-http://192.168.8.88:8766}"
MARKER="${BEEP_FAIR_READY_FILE:-/tmp/beep-fair-ready.json}"
DURATION_S="${BEEP_FAIR_DURATION_S:-600}"
SEGMENT_S="${BEEP_FAIR_SEGMENT_S:-45}"
PREY_INTERVAL_S="${BEEP_FAIR_PREY_INTERVAL_S:-150}"
HTTP_TIMEOUT_S="${BEEP_HTTP_TIMEOUT_S:-3}"

stop_beep() {
    curl -fsS --max-time "${HTTP_TIMEOUT_S}" "${BRIDGE_URL}/stop" >/dev/null 2>&1 || true
}
trap stop_beep EXIT INT TERM
cd "${REPO_DIR}"

python3 - "${BRIDGE_URL}" "${MARKER}" "${HTTP_TIMEOUT_S}" <<'PY'
import json
import math
import sys
import time
import urllib.request

base, marker, timeout_text = sys.argv[1:]
http_timeout = float(timeout_text)
try:
    with open(marker, encoding="utf-8") as handle:
        prepared = json.load(handle)
except FileNotFoundError:
    raise SystemExit("Fast start refused: run scripts/prepare_fair_run.sh after placing BEEP")

if time.time() - float(prepared["prepared_at"]) > 7200:
    raise SystemExit("Fast start refused: preparation is older than two hours")
if prepared.get("bridge_url") != base:
    raise SystemExit("Fast start refused: bridge URL differs from preparation")

last_error = None
for _ in range(3):
    try:
        with urllib.request.urlopen(base + "/status", timeout=http_timeout) as response:
            current = json.load(response)
        break
    except Exception as exc:
        last_error = exc
        time.sleep(0.25)
else:
    raise SystemExit(f"Fast start refused: bridge status unavailable: {last_error}")
slam = current.get("slam") or {}
sectors = current.get("sectors") or {}
required = ("front", "front_left", "front_right", "left", "right", "rear")
p0, p1 = prepared["pose"], current.get("pose") or {}
distance = math.hypot(float(p1["x"]) - float(p0["x"]), float(p1["y"]) - float(p0["y"]))
yaw_delta = abs(math.atan2(
    math.sin(float(p1["yaw"]) - float(p0["yaw"])),
    math.cos(float(p1["yaw"]) - float(p0["yaw"])),
))
checks = {
    "same_version": current.get("version") == prepared.get("version"),
    "stopped": current.get("moving") is False and current.get("motion_lease_id") is None,
    "slam_usable": slam.get("usable") is True,
    "fresh_scan": current.get("scan_age_s") is not None and float(current["scan_age_s"]) <= 0.5,
    "fresh_map": slam.get("map_age_s") is not None and float(slam["map_age_s"]) <= 2.5,
    "not_repositioned": distance <= 0.08 and math.degrees(yaw_delta) <= 10.0,
    "clearance": all(sectors.get(name) is not None and float(sectors[name]) >= 0.15 for name in required),
}
if not all(checks.values()):
    raise SystemExit("Fast start refused: " + json.dumps(checks, sort_keys=True))

print(json.dumps({"fast_preflight": "passed", "checks": checks}, sort_keys=True))
PY

if [[ "${BEEP_FAST_CHECK_ONLY:-0}" == "1" ]]; then
    printf 'Fast preflight check passed; no motion requested.\n'
    exit 0
fi

# One preparation authorizes one run. Re-prepare after completion or relocation.
rm -f -- "${MARKER}"

python3 scripts/run_slam_presentation.py \
    --armed \
    --request-timeout "${HTTP_TIMEOUT_S}" \
    --duration "${DURATION_S}" \
    --segment "${SEGMENT_S}" \
    --prey-interval "${PREY_INTERVAL_S}"
