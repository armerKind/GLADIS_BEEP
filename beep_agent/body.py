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

from .skills import GESTURE_IDS, SkillCall, canonical_gesture, validate_skill_call
from .gesture_policy import GestureAdmission, GestureIntent

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

    @staticmethod
    def _stationary_preflight(readback: dict) -> dict[str, str]:
        status = readback.get('status') or {}
        fence = readback.get('stationary_preflight')
        if (not isinstance(fence, dict) or set(fence) != {'boot_id', 'token'}
                or not isinstance(fence.get('boot_id'), str)
                or fence['boot_id'] != status.get('boot_id')
                or not isinstance(fence.get('token'), str)
                or not fence['token']):
            raise BodyDispatchError('bridge did not issue an exact stationary preflight token')
        return fence

    def perform_gesture(self, name: str, *, rollout_mode: str = "shadow") -> DispatchResult:
        """Suggest by default; supervised dispatch only after a fresh bridge preflight."""
        canonical = canonical_gesture(name)  # Never accept raw IDs or posture aliases.
        if rollout_mode not in {"shadow", "stationary", "supervised"}:
            raise BodyDispatchError("gesture requires a recognized rollout mode")
        if rollout_mode != "supervised":
            return DispatchResult(True, "gesture", False, False,
                                  {"ok": True, "reason": "gesture_not_physically_armed"})
        preflight = self.mission()
        status = preflight.get("status", {})
        if (preflight.get("mission") is not None or not isinstance(status, dict)
                or status.get("moving") is not False
                or status.get("motion_lease_id") is not None
                or status.get("posture") != "standing"):
            raise BodyDispatchError("gesture requires a fresh stationary standing unowned mission-free readback")
        fence = self._stationary_preflight(preflight)
        response = self._json("/action", {"name": canonical, "stationary_preflight": fence})
        trick = response.get("trick")
        if (response.get("ok") is not True or response.get("dry_run") is not False
                or response.get("http_status") is not None or not isinstance(trick, dict)
                or trick.get("name") != canonical or trick.get("id") != GESTURE_IDS[canonical]):
            raise BodyDispatchError("gesture was refused or bridge receipt did not match the request")
        return DispatchResult(True, "gesture", True, False, response)

    def execute_gesture_intent(self, intent: GestureIntent, admission: GestureAdmission,
                               *, physical_posture_confirmed: bool = False) -> DispatchResult:
        """Supervised-only path; never turn a shadow proposal into motor authority.

        The bridge rechecks admission under its own locks. This host-side
        preflight cannot close a network TOCTOU gap, nor infer physical stance
        from a software status after the action. A lost reply is not retried.
        """
        preflight = self.mission()
        admission.check(intent, preflight,
                        physical_posture_confirmed=physical_posture_confirmed)
        fence = self._stationary_preflight(preflight) if intent.stationary_preflight else None
        if fence is not None and fence['boot_id'] != intent.boot_id:
            raise BodyDispatchError('stationary preflight belongs to another bridge boot')
        admission.claim(intent)  # One-shot before any potentially accepted POST.
        response = None
        error = None
        final = None
        try:
            payload = ({'name': intent.name, 'stationary_preflight': fence}
                if intent.stationary_preflight else {'name': intent.name, 'release': {
                'boot_id': intent.boot_id, 'mission_id': intent.released_mission_id,
                'lease_id': intent.released_lease_id, 'release_id': intent.release_id}})
            response = self._json('/action', payload)
        except Exception as exc:
            error = exc
        finally:
            try:
                final = self.mission()
            except Exception as exc:
                error = error or exc
            # Software 'standing' cannot certify an upright quadruped.
            admission.mark_uncertain('post_action_physical_posture_unverified'
                                     if error is None else 'gesture_outcome_uncertain')
        if error is not None:
            raise BodyDispatchError('gesture transport or cleanup uncertain') from error
        trick = response.get('trick') if isinstance(response, dict) else None
        if (not isinstance(response, dict) or response.get('ok') is not True
                or response.get('dry_run') is not False
                or response.get('http_status') is not None
                or not isinstance(trick, dict) or trick.get('name') != intent.name
                or trick.get('id') != GESTURE_IDS[intent.name]):
            raise BodyDispatchError('gesture refused or receipt mismatched; no retry')
        # A host /stop could cancel a NEW owner's mission after the bridge
        # action lease ends. Lease cleanup belongs to run_owned_gesture's
        # finally/watchdog; never issue an unowned global stop here.
        status = final.get('status') if isinstance(final, dict) else None
        latest = final.get('mission') if isinstance(final, dict) else None
        release_ok = (latest is None if intent.stationary_preflight else
            isinstance(latest, dict) and
            (latest.get('id'), latest.get('lease_id'), latest.get('release_id')) ==
            (intent.released_mission_id, intent.released_lease_id, intent.release_id))
        if (not isinstance(status, dict) or not release_ok
                or status.get('boot_id') != intent.boot_id
                or status.get('moving') is not False
                or status.get('motion_lease_id') is not None
                or status.get('fall_suspected') is not False):
            raise BodyDispatchError('gesture postflight owner or state unverified')
        return DispatchResult(True, 'gesture', True, False,
            {**response, 'bridge_lease_released': True, 'postflight': final,
             'physical_posture_verified': False})

    def dispatch(self, call: SkillCall, *, rollout_mode: str = "shadow",
                 mission_id: str | None = None, lease_id: str | None = None) -> DispatchResult:
        call = validate_skill_call(call)
        if rollout_mode not in {"shadow", "stationary", "supervised"}:
            raise BodyDispatchError("rollout mode must be shadow, stationary, or supervised")
        if call.name not in {"explore", "observe", "stop", "gesture"}:
            raise BodyDispatchError(f"no deterministic body adapter for semantic skill {call.name!r}")
        if rollout_mode == "shadow":
            return DispatchResult(True, call.name, False, False, {"ok": True, "reason": "rollout_not_physically_armed"})
        if rollout_mode == "stationary" and call.name not in {"observe", "stop"}:
            return DispatchResult(True, call.name, False, False, {"ok": True, "reason": "stationary_rollout_blocks_motion"})

        if call.name == "gesture":
            return self.perform_gesture(call.arguments["name"], rollout_mode=rollout_mode)

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
