#!/usr/bin/env bash
set -u
BRIDGE_URL="${BEEP_BRIDGE_URL:-http://192.168.8.88:8766}"

for _ in 1 2 3; do
    curl -fsS --max-time 3 "${BRIDGE_URL}/stop" >/dev/null 2>&1 || true
    sleep 0.15
done

if curl -fsS --max-time 4 "${BRIDGE_URL}/status"; then
    printf '\n'
else
    printf 'Stop was attempted, but bridge status is unreachable. Use physical stop/power immediately.\n' >&2
    exit 1
fi
