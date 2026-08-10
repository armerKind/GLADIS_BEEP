"""External supervisory observer for BEEP.

The observer is intentionally incapable of issuing locomotion commands. Its only
optional side effect is an authenticated, exact-mission emergency stop request.
"""

from .core import (
    Assessment,
    EvidenceStore,
    ExternalObserver,
    Frame,
    MetadataRiskEvaluator,
    ObserverConfig,
    RiskLevel,
    StopOutcome,
)
from .bridge import BridgeStopSink
from .sources import DirectoryFrameSource

__all__ = [
    "Assessment",
    "BridgeStopSink",
    "DirectoryFrameSource",
    "EvidenceStore",
    "ExternalObserver",
    "Frame",
    "MetadataRiskEvaluator",
    "ObserverConfig",
    "RiskLevel",
    "StopOutcome",
]
