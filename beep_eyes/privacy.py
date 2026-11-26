"""Local-only camera privacy gate and one-way session vision switch.

Darkness is evidence that the camera cannot see, not proof of a physical cap.
No image or image-derived bytes leave the host through this module.
"""
from __future__ import annotations

from dataclasses import dataclass
import io
from pathlib import Path

from PIL import Image, ImageStat, UnidentifiedImageError


class CameraPrivacyError(RuntimeError):
    """Frame cannot be safely admitted to a cloud vision call."""


@dataclass(frozen=True)
class FrameVisibility:
    occluded: bool
    mean_luminance: float
    luminance_stddev: float


class CameraOccluded(CameraPrivacyError):
    def __init__(self, visibility: FrameVisibility):
        super().__init__('camera_occluded: local darkness/low-variance gate')
        self.visibility = visibility


def inspect_frame(jpeg: bytes) -> FrameVisibility:
    """Decode JPEG locally, ignore vendor HUD at top, and sample luminance.

    This intentionally errs on the privacy side: a dark room can look like a
    cap, so neither case is uploaded until an operator selects no-cloud mode.
    """
    if (not isinstance(jpeg, bytes) or not 4 <= len(jpeg) <= 2_000_000
            or jpeg[:2] != b'\xff\xd8' or jpeg[-2:] != b'\xff\xd9'):
        raise CameraPrivacyError('invalid or oversized camera JPEG')
    try:
        with Image.open(io.BytesIO(jpeg)) as source:
            if source.format != 'JPEG' or not (16 <= source.width <= 4096 and 16 <= source.height <= 4096):
                raise CameraPrivacyError('unexpected camera JPEG dimensions')
            source.load()  # Decode fully; a truncated JPEG is not clear evidence.
            gray = source.convert('L')
            gray = gray.crop((0, int(gray.height * 0.10), gray.width, gray.height))
            gray = gray.resize((64, 48), Image.Resampling.BILINEAR)
            mean = ImageStat.Stat(gray).mean[0]
            stddev = ImageStat.Stat(gray).stddev[0]
    except (OSError, ValueError, UnidentifiedImageError, Image.DecompressionBombError) as exc:
        raise CameraPrivacyError('camera JPEG could not be decoded locally') from exc
    # Flat bright covers are also suspicious. This is a conservative heuristic,
    # not a certification that every opaque/printed lens cap is detected.
    occluded = ((mean < 24.0 and stddev < 14.0)
                or (mean > 235.0 and stddev < 8.0))
    return FrameVisibility(occluded, round(mean, 2), round(stddev, 2))


def ensure_visible(jpeg: bytes) -> FrameVisibility:
    result = inspect_frame(jpeg)
    if result.occluded:
        raise CameraOccluded(result)
    return result


class VisionMode:
    """One-way no-cloud latch; operator writes `no-cloud-vision` to the file.

    Unknown/non-regular/oversized control files fail closed. A missing file is
    not permission to re-enable a session that was previously disabled.
    """
    def __init__(self, switch_file: Path, *, no_cloud_vision: bool = False):
        self.switch_file = Path(switch_file)
        self.no_cloud_vision = bool(no_cloud_vision)

    @property
    def cloud_allowed(self) -> bool:
        return not self.no_cloud_vision

    def refresh(self) -> bool:
        if self.no_cloud_vision:
            return True
        try:
            if self.switch_file.exists():
                # The only supported content is the one-way command; any
                # other existing control file also withholds cloud access.
                self.no_cloud_vision = True
                if (not self.switch_file.is_file() or self.switch_file.stat().st_size > 64
                        or self.switch_file.read_text(encoding='ascii').strip() != 'no-cloud-vision'):
                    return True
        except (OSError, UnicodeError):
            self.no_cloud_vision = True
        return self.no_cloud_vision
