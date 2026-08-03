"""Gemini transport for BEEP temporal perception.

Secrets are accepted only from process environment and sent in an HTTP header;
they are never written to capture artifacts or request URLs.
"""
from __future__ import annotations

import base64
import json
import os
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Callable

from .schema import (
    GEMINI_COMPACT_JSON_SCHEMA,
    PerceptionValidationError,
    normalize_gemini_response,
    validate_perception_response,
)

DEFAULT_MODEL = "gemini-3.5-flash-lite"
DEFAULT_API_BASE = "https://generativelanguage.googleapis.com/v1beta"
MAX_RESPONSE_BYTES = 2 * 1024 * 1024
Transport = Callable[[str, dict[str, str], bytes, float], dict[str, Any]]


class GeminiError(RuntimeError):
    pass


def _default_transport(url: str, headers: dict[str, str], body: bytes, timeout_s: float) -> dict[str, Any]:
    request = urllib.request.Request(url, data=body, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            data = response.read(MAX_RESPONSE_BYTES + 1)
            if len(data) > MAX_RESPONSE_BYTES:
                raise GeminiError("Gemini response is too large")
            return json.loads(data)
    except urllib.error.HTTPError as exc:
        detail = exc.read(1000).decode("utf-8", errors="replace")
        raise GeminiError(f"Gemini HTTP {exc.code}: {detail}") from exc
    except GeminiError:
        raise
    except Exception as exc:
        raise GeminiError(f"Gemini request failed: {type(exc).__name__}: {exc}") from exc


def _extract_json(response: dict[str, Any]) -> dict[str, Any]:
    try:
        parts = response["candidates"][0]["content"]["parts"]
    except (KeyError, IndexError, TypeError) as exc:
        raise GeminiError("Gemini response has no candidate content") from exc
    text = "".join(part.get("text", "") for part in parts if isinstance(part, dict)).strip()
    if text.startswith("```"):
        lines = text.splitlines()
        text = "\n".join(lines[1:-1]).strip()
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError as exc:
        raise GeminiError("Gemini response was not valid JSON") from exc
    if "scene" in parsed:
        return validate_perception_response(parsed)
    return normalize_gemini_response(parsed)


@dataclass
class GeminiVisionClient:
    api_key: str
    model: str = DEFAULT_MODEL
    timeout_s: float = 20.0
    api_base: str = DEFAULT_API_BASE
    transport: Transport = _default_transport

    @classmethod
    def from_environment(cls, *, model: str | None = None, timeout_s: float = 20.0) -> "GeminiVisionClient":
        key = os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")
        if not key:
            raise GeminiError("GEMINI_API_KEY or GOOGLE_API_KEY is required for live Gemini mode")
        return cls(api_key=key, model=model or os.environ.get("BEEP_EYES_MODEL", DEFAULT_MODEL), timeout_s=timeout_s)

    @classmethod
    def from_hermes_pool(
        cls,
        *,
        hermes_home: str | None = None,
        model: str | None = None,
        timeout_s: float = 20.0,
    ) -> "GeminiVisionClient":
        """Resolve a Gemini API key from Hermes without exposing its value."""
        if hermes_home:
            os.environ["HERMES_HOME"] = os.path.expanduser(hermes_home)
        try:
            import importlib
            load_pool = importlib.import_module("agent.credential_pool").load_pool
        except ImportError as exc:
            raise GeminiError("Hermes Agent source is not importable; use an environment credential") from exc
        entry = load_pool("gemini").select()
        key = entry.runtime_api_key if entry is not None else ""
        if not key:
            raise GeminiError("no available Gemini credential in the Hermes pool")
        return cls(api_key=key, model=model or os.environ.get("BEEP_EYES_MODEL", DEFAULT_MODEL), timeout_s=timeout_s)

    def analyze(self, packet: dict[str, Any], contact_sheet_jpeg: bytes, latest_frame_jpeg: bytes) -> dict[str, Any]:
        prompt = (
            "You are the visual perception route for BEEP, a quadruped robot. "
            "The first image is a chronological contact sheet; the second is the latest full frame. "
            "Describe only visually supported facts, compare panels for change, identify hazards including glass, "
            "and propose at most three steps using only the supplied schema. Never invent raw motor commands. "
            "In each compact step, argument carries left/right for orient, the gesture name for gesture, or spoken text for speak; "
            "amount carries degrees for orient, metres for advance/retreat, or seconds for explore. Use an empty argument and zero amount when unused. "
            "Limits are strict: orient 1-30 degrees, advance/retreat 0.05-0.30 metres, explore 5-30 seconds, and at most three steps. "
            "Movement proposals are advisory and will be checked by LiDAR, SLAM, and a deterministic bridge. "
            "Use short English speech in GLADIS's precise, dry style. Escalate when identity, geometry, intent, "
            "or safety is uncertain. Observation metadata follows:\n" + json.dumps(packet, separators=(",", ":"), sort_keys=True)
        )
        request_body = {
            "contents": [{
                "role": "user",
                "parts": [
                    {"text": prompt},
                    {"inlineData": {"mimeType": "image/jpeg", "data": base64.b64encode(contact_sheet_jpeg).decode("ascii")}},
                    {"inlineData": {"mimeType": "image/jpeg", "data": base64.b64encode(latest_frame_jpeg).decode("ascii")}},
                ],
            }],
            "generationConfig": {
                "temperature": 0.2,
                "maxOutputTokens": 2048,
                "responseMimeType": "application/json",
                "responseJsonSchema": GEMINI_COMPACT_JSON_SCHEMA,
                "thinkingConfig": {"thinkingLevel": "minimal", "includeThoughts": False},
            },
        }
        url = f"{self.api_base.rstrip('/')}/models/{self.model}:generateContent"
        headers = {"Content-Type": "application/json", "x-goog-api-key": self.api_key}
        for attempt in range(2):
            response = self.transport(url, headers, json.dumps(request_body).encode("utf-8"), self.timeout_s)
            try:
                return _extract_json(response)
            except PerceptionValidationError as exc:
                if attempt:
                    raise GeminiError(f"Gemini output failed the local contract after repair: {exc}") from exc
                request_body["contents"][0]["parts"][0]["text"] += (
                    "\nYour previous response violated this local contract: " + str(exc) +
                    ". Regenerate the complete response within every stated bound."
                )
        raise GeminiError("Gemini produced no valid perception response")
