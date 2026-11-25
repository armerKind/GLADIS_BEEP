"""Opt-in stationary social view; never runs without an explicit caller.

HTTP posture/camera authority lives in the deterministic bridge. The visual
caption is qualitative, not a recognized identity or verified camera pitch.
"""
from __future__ import annotations

import json
from typing import Callable
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

from .privacy import CameraOccluded, CameraPrivacyError, ensure_visible


class SocialViewError(RuntimeError):
    pass


def upward_social_turn(base_url: str, voice_loop, describe_frame: Callable[[bytes], str],
                       *, opt_in: bool = False, no_cloud_vision: bool = False):
    """Sit -> one JPEG -> bounded caption -> greeting -> stand in finally.

    `describe_frame` can be a local or consented cloud provider. This helper
    never sends a frame to a provider itself. The caller must opt in to both
    posture control and any provider-side image privacy policy.
    """
    if not opt_in:
        raise SocialViewError("stationary social view requires explicit opt-in")
    if no_cloud_vision:
        raise SocialViewError('no-cloud-vision forbids visual social capture; use direct audio greeting')
    if not callable(describe_frame) or not voice_loop.read_authority()[2]:
        raise SocialViewError("visual description and voice consent are required")
    parts = urlsplit(base_url)
    if (parts.scheme != "http" or not parts.hostname or parts.username or parts.password
            or parts.path not in ("", "/") or parts.query or parts.fragment):
        raise SocialViewError("provide a plain HTTP bridge origin")
    origin = base_url.rstrip("/")

    def posture(value: str) -> dict:
        payload = json.dumps({"posture": value}).encode("utf-8")
        req = Request(origin + "/posture", data=payload, method="POST",
                      headers={"Content-Type": "application/json"})
        with urlopen(req, timeout=12) as response:
            result = json.load(response)
        expected = "sitting" if value == "sit" else "standing"
        status = result.get("status") or {}
        if (result.get("ok") is not True or result.get("posture") != expected
                or status.get("posture") != expected or status.get("posture_action_acknowledged") is not True
                or status.get("moving") is not False
                or status.get("motion_lease_id") is not None):
            raise SocialViewError("posture readback unconfirmed: " + value)
        return result

    # Refusal to sit may mean an active mission: do not blindly issue stand.
    posture("sit")
    try:
        with urlopen(origin + "/frame.jpg", timeout=6) as response:
            frame = response.read(1_500_001)
        if not 4 <= len(frame) <= 1_500_000 or not frame.startswith(b"\xff\xd8") or not frame.endswith(b"\xff\xd9"):
            raise SocialViewError("camera frame missing, oversized or not JPEG")
        try:
            ensure_visible(frame)
        except CameraOccluded as exc:
            # The injected captioner (possibly cloud-backed) never sees dark
            # frames. The fixed text is spoken through the existing audio sink.
            authority = voice_loop.read_authority()
            text = ("My camera is dark. If you fitted a lens cap, excellent privacy; "
                    "less excellent sightseeing. Please select no-cloud vision mode.")
            outcome = {'state': 'camera_occluded', 'audio_confirmed': False,
                       'cloud_vision_calls': 0, 'mean_luminance': exc.visibility.mean_luminance}
            try:
                audio = voice_loop.synthesize(text)
                outcome['audio_confirmed'] = (isinstance(audio, bytes) and 0 < len(audio) <= 2_000_000
                    and voice_loop.read_authority() == authority and voice_loop.play(audio) is True)
            except Exception as audio_exc:
                outcome['audio_error'] = type(audio_exc).__name__
        except CameraPrivacyError as exc:
            raise SocialViewError('camera frame failed local privacy gate') from exc
        else:
            description = describe_frame(frame)
            if (not isinstance(description, str) or not description.strip()
                    or len(description) > 240):
                raise SocialViewError("visual description must be 1..240 characters")
            outcome = voice_loop.initiate_turn(trigger="person_noticed", visual_context=description)
    finally:
        # If this request fails, report an unverified physical posture; never
        # claim that the robot stood merely because cleanup was attempted.
        posture("stand")
    return outcome
