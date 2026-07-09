#!/usr/bin/env bash
set -euo pipefail

HOST="${BEEP_HOST:-192.168.8.88}"
OUT_DIR="${1:-captures}"
BASE="http://${HOST}:8766"
STAMP="$(date +%Y%m%d_%H%M%S)"
DEST="${OUT_DIR}/${STAMP}"
mkdir -p "${DEST}"

curl --connect-timeout 2 --max-time 8 -sS "${BASE}/status" -o "${DEST}/status.json" || true
curl --connect-timeout 2 --max-time 8 -sS "${BASE}/local_map" -o "${DEST}/local_map.json" || true
curl --connect-timeout 2 --max-time 8 -sS "${BASE}/map" -o "${DEST}/map.json" || true
curl --connect-timeout 2 --max-time 8 -sS "${BASE}/local_map.svg" -o "${DEST}/local_map.svg" || true
curl --connect-timeout 2 --max-time 8 -sS "${BASE}/map.svg" -o "${DEST}/map.svg" || true
curl --connect-timeout 2 --max-time 8 -sS "${BASE}/frame.jpg" -o "${DEST}/frame.jpg" || true

echo "Captured bridge artifacts to ${DEST}"
