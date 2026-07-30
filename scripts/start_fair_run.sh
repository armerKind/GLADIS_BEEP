#!/usr/bin/env bash
set -Eeuo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BRIDGE_URL="${BEEP_BRIDGE_URL:-http://192.168.8.88:8766}"
DURATION_S="${BEEP_FAIR_DURATION_S:-600}"
SEGMENT_S="${BEEP_FAIR_SEGMENT_S:-45}"
PREY_INTERVAL_S="${BEEP_FAIR_PREY_INTERVAL_S:-150}"

stop_beep() {
    curl -fsS --max-time 4 "${BRIDGE_URL}/stop" >/dev/null 2>&1 || true
}

trap stop_beep EXIT INT TERM
cd "${REPO_DIR}"

printf 'Stopping BEEP before fair preflight...\n'
stop_beep

printf 'Resetting SLAM after placement...\n'
python3 scripts/jupyter_reset_slam.py

printf 'Waiting for bridge and SLAM startup...\n'
python3 - "${BRIDGE_URL}" <<'PY'
import json
import sys
import time
import urllib.request

base = sys.argv[1]
last = None
for _ in range(60):
    try:
        with urllib.request.urlopen(base + "/status", timeout=3) as response:
            last = json.load(response)
        slam = last.get("slam") or {}
        if (not last.get("moving") and slam.get("usable") is True
                and last.get("scan_age_s") is not None
                and float(last["scan_age_s"]) <= 0.5):
            print(json.dumps({
                "version": last.get("version"),
                "moving": last.get("moving"),
                "scan_age_s": last.get("scan_age_s"),
                "slam_usable": slam.get("usable"),
                "map_age_s": slam.get("map_age_s"),
                "sectors": last.get("sectors"),
            }, indent=2, sort_keys=True))
            break
    except Exception:
        pass
    time.sleep(0.5)
else:
    raise SystemExit("BEEP preflight failed: bridge/SLAM did not become usable")
PY

printf '\nValidating the no-pee presentation plan...\n'
python3 scripts/run_slam_presentation.py \
    --dry-run \
    --duration "${DURATION_S}" \
    --segment "${SEGMENT_S}" \
    --prey-interval "${PREY_INTERVAL_S}"

printf '\nNo pee is enabled. Keep immediate physical stop authority throughout.\n'
read -r -p 'Type RUN to start the supervised fair routine: ' confirmation
if [[ "${confirmation}" != "RUN" ]]; then
    printf 'Cancelled. BEEP remains stopped.\n'
    exit 1
fi

printf 'Starting supervised fair routine: %ss, %ss leases, prey every %ss, pee disabled.\n' \
    "${DURATION_S}" "${SEGMENT_S}" "${PREY_INTERVAL_S}"
python3 scripts/run_slam_presentation.py \
    --armed \
    --duration "${DURATION_S}" \
    --segment "${SEGMENT_S}" \
    --prey-interval "${PREY_INTERVAL_S}"
