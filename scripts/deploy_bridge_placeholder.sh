#!/usr/bin/env bash
set -euo pipefail

HOST="${BEEP_HOST:-192.168.8.88}"
BASE="http://${HOST}:8888"
BRIDGE_SRC="${1:-beep_bridge/beep_bridge.py}"

if [[ ! -f "${BRIDGE_SRC}" ]]; then
  echo "Bridge source not found: ${BRIDGE_SRC}" >&2
  exit 1
fi

cat <<'MSG'
This script is intentionally a placeholder.

Current deployment has been done through the Jupyter REST/WebSocket API because BEEP SSH may be unavailable.
The next repo task is to replace this with either:

1. SSH/scp deploy when port 22 is reliable, or
2. a small Python Jupyter deploy client using the known Pi Jupyter password flow.

For now, use docs/deployment.md or GLADIS's deployment helper from the session history.
MSG
