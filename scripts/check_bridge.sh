#!/usr/bin/env bash
set -euo pipefail

HOST="${BEEP_HOST:-192.168.8.88}"
BASE="http://${HOST}:8766"

for path in /config /status /local_map /map; do
  echo "### ${BASE}${path}"
  curl --connect-timeout 2 --max-time 8 -sS "${BASE}${path}" | python3 -m json.tool || true
  echo
done

echo "### stop check"
curl --connect-timeout 2 --max-time 8 -sS -X POST -H 'Content-Type: application/json' -d '{}' "${BASE}/stop" | python3 -m json.tool || true
