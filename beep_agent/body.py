"""Deterministic bridge adapter for typed embodied skills.

The adapter never translates arbitrary text into motor commands. It maps validated
semantic calls onto bounded bridge operations and leaves collision supervision,
lease ownership, and final motor stopping onboard BEEP.
"""
from __future__ import annotations

import json
import math
import urllib.error
import urllib.request
from dataclasses import asdict, dataclass
from typing import Any

from .skills import SkillCall, validate_skill_call

MAX_RESPONSE_BYTES = 2 * 1024 * 1024


class BodyDispatchError(RuntimeError):
    pass


@dataclass(frozen=True)
class DispatchResult:
    ok: bool
    skill: str
    dispatched: bool
    asynchronous: bool
    response: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class BridgeBodyAdapter:
    """Map semantic skills to the bridge without exposing raw motor parameters."""

    def __init__(self, base_url: str, timeout_s: float = 8.0):
        self.base_url = base_url.rstrip("/")
        self.timeout_s = min(max(float(timeout_s), 1.0), 20.0)

    def _json(self, path: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
        data = None if body is None else json.dumps(body).encode("utf-8")
        request = urllib.request.Request(
            self.base_url + path,
            data=data,
            headers={"Content-Type": "application/json", "User-Agent": "beep-middle-layer/1.0"},
            method="GET" if body is None else "POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                raw = response.read(MAX_RESPONSE_BYTES + 1)
        except urllib.error.HTTPError as exc:
            raw = exc.read(MAX_RESPONSE_BYTES + 1)
            if len(raw) > MAX_RESPONSE_BYTES:
                raise BodyDispatchError("bridge error response is too large") from exc
            try:
                payload = json.loads(raw)
            except Exception:
                payload = {"ok": False, "error": f"bridge HTTP {exc.code}"}
            payload.setdefault("http_status", exc.code)
            return payload
        except (OSError, urllib.error.URLError) as exc:
            raise BodyDispatchError(f"bridge request failed for {path}: {exc}") from exc
        if len(raw) > MAX_RESPONSE_BYTES:
            raise BodyDispatchError("bridge response is too large")
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise BodyDispatchError("bridge returned malformed JSON") from exc
        if not isinstance(payload, dict):
            raise BodyDispatchError("bridge response must be an object")
        return payload

    def status(self) -> dict[str, Any]:
        return self._json("/status")

    def mission(self) -> dict[str, Any]:
        return self._json("/mission")

    def bias_mission(self, *, boot_id: str, mission_id: str, lease_id: str,
                     preferred_direction: str, ttl_s: float = 4.0) -> dict[str, Any]:
        """Send only a short-lived direction hint to an already owned mission.

        This does not dispatch a skill, start a lease, or acquire motor authority.
        The bridge checks these exact identities again when it receives the POST.
        """
        if any(type(value) is not str or not value.strip()
               for value in (boot_id, mission_id, lease_id)):
            raise BodyDispatchError("mission bias requires exact boot, mission and lease IDs")
        if preferred_direction not in ("left", "straight", "right"):
            raise BodyDispatchError("mission bias requires a validated direction")
        if type(ttl_s) not in (int, float) or not 0 < ttl_s <= 8 or not math.isfinite(ttl_s):
            raise BodyDispatchError("mission bias TTL must be finite and within (0, 8] seconds")
        response = self._json("/mission/bias", {
            "boot_id": boot_id, "mission_id": mission_id, "lease_id": lease_id,
            "ttl_s": float(ttl_s), "preferred_direction": preferred_direction,
            "preferred_bearing_rad": None,
        })
        if (response.get("ok") is not True or response.get("accepted") is not True
                or response.get("motor_authority") is not False
                or response.get("semantic_execution_enabled") is not False
                or response.get("mission_id") != mission_id
                or response.get("lease_id") != lease_id):
            raise BodyDispatchError("mission bias rejected or bridge acknowledgement invalid")
        return response

    def dispatch(self, call: SkillCall, *, rollout_mode: str = "shadow",
                 mission_id: str | None = None, lease_id: str | None = None) -> DispatchResult:
        call = validate_skill_call(call)
        if rollout_mode not in {"shadow", "stationary", "supervised"}:
            raise BodyDispatchError("rollout mode must be shadow, stationary, or supervised")
        if call.name not in {"explore", "observe", "stop"}:
            raise BodyDispatchError(f"no deterministic body adapter for semantic skill {call.name!r}")
        if rollout_mode == "shadow":
            return DispatchResult(True, call.name, False, False, {"ok": True, "reason": "rollout_not_physically_armed"})
        if rollout_mode == "stationary" and call.name not in {"observe", "stop"}:
            return DispatchResult(True, call.name, False, False, {"ok": True, "reason": "stationary_rollout_blocks_motion"})

        if call.name == "explore":
            response = self._json(
                "/mission/start",
                {
                    "mode": "coverage",
                    "duration_s": float(call.arguments["duration_s"]),
                    "min_duration": float(call.arguments["duration_s"]),
                    "save": False,
                },
            )
            return DispatchResult(bool(response.get("accepted")), call.name, bool(response.get("accepted")), True, response)
        if call.name == "stop":
            if not mission_id or not lease_id:
                raise BodyDispatchError("stop requires exact mission_id and lease_id")
            response = self._json("/mission/cancel", {"mission_id": mission_id, "lease_id": lease_id})
            ok = response.get("ok") is True and response.get("cancelled") is True
            return DispatchResult(ok, call.name, ok, False, response)
        if call.name == "observe":
            response = self._json("/observe")
            return DispatchResult(True, call.name, True, False, response)
        raise BodyDispatchError(f"no deterministic body adapter for semantic skill {call.name!r}")
