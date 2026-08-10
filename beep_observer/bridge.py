from __future__ import annotations

import json
import time
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from .core import Assessment, StopOutcome


class BridgeStopSink:
    """Exact-mission stop-only bridge capability. No movement method exists."""

    def __init__(self, base_url: str, token: str | None, timeout_s=1.5):
        self.base_url = base_url.rstrip("/")
        self.token = token or ""
        self.timeout_s = max(0.2, min(float(timeout_s), 5.0))

    @property
    def authenticated(self) -> bool:
        return bool(self.token)

    def _request(self, method: str, path: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        headers = {"Accept": "application/json", "Authorization": f"Bearer {self.token}"}
        if data is not None:
            headers["Content-Type"] = "application/json"
        request = Request(self.base_url + path, method=method, data=data, headers=headers)
        with urlopen(request, timeout=self.timeout_s) as response:
            body = response.read()
        return json.loads(body.decode("utf-8")) if body else {}

    @staticmethod
    def _mission_id(status: dict[str, Any]) -> str | None:
        candidates = [status, status.get("mission") or {}, status.get("active") or {}]
        for candidate in candidates:
            value = candidate.get("mission_id") or candidate.get("id")
            if value:
                return str(value)
        return None

    def stop(self, assessment: Assessment) -> StopOutcome:
        if not self.authenticated:
            return StopOutcome(True, False, "missing_token")
        try:
            mission_status = self._request("GET", "/mission")
            mission_id = self._mission_id(mission_status)
            if mission_id is None:
                return StopOutcome(True, False, "no_active_mission", response=mission_status)
            response = self._request("POST", "/observer/stop", {
                "mission_id": mission_id,
                "event_id": assessment.event_id,
                "reason": ",".join(assessment.reasons) or "external_observer_stop",
                "observed_at": assessment.assessed_at,
            })
            return StopOutcome(True, bool(response.get("ok")), str(response.get("reason", "bridge_response")),
                               mission_id=mission_id, response=response)
        except HTTPError as exc:
            return StopOutcome(True, False, f"http_error:{exc.code}")
        except (URLError, TimeoutError, OSError, ValueError, json.JSONDecodeError) as exc:
            return StopOutcome(True, False, f"transport_error:{type(exc).__name__}")
