"""Phase C shadow goal owner above bounded local leases; no motor or bridge I/O.

A plan_next_segment decision is *not* a route or permission to acquire a
lease. External local authority must verify map, route, footprint and stopped
ownership independently. This class only preserves strategic continuity and
checks exact, explicitly acknowledged lease handoffs.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
import threading

from .session import AgentSession, SessionIdentityError


class CoordinatorError(ValueError):
    pass


@dataclass(frozen=True)
class CoordinatorContext:
    boot_id: str
    map_epoch: int
    map_revision: int
    map_snapshot_id: str
    map_verified: bool
    sensors_fresh: bool
    now_robot: float
    mission_id: str | None
    lease_id: str | None
    local_motion_active: bool

    def __post_init__(self) -> None:
        if (not isinstance(self.boot_id, str) or not self.boot_id
                or not isinstance(self.map_snapshot_id, str) or not self.map_snapshot_id
                or type(self.map_epoch) is not int or self.map_epoch < 0
                or type(self.map_revision) is not int or self.map_revision < 0
                or type(self.map_verified) is not bool or type(self.sensors_fresh) is not bool
                or isinstance(self.now_robot, bool)
                or not isinstance(self.now_robot, (int, float))
                or not isfinite(self.now_robot) or self.now_robot < 0
                or type(self.local_motion_active) is not bool
                or (self.mission_id is None) != (self.lease_id is None)
                or any(not isinstance(v, str) or not v.strip()
                       for v in (self.mission_id, self.lease_id) if v is not None)
                or (self.local_motion_active and self.lease_id is None)):
            raise CoordinatorError("invalid or incomplete local context")

    @property
    def map_identity(self) -> tuple[str, int, int, str]:
        return (self.boot_id, self.map_epoch, self.map_revision, self.map_snapshot_id)


@dataclass(frozen=True)
class CoordinatorDecision:
    action: str
    reason: str
    goal_name: str | None
    goal_revision: int
    session_id: str
    motor_authority: bool = field(default=False, init=False)


class GoalCoordinator:
    """One strategic goal across sequential local leases, advisory only."""

    def __init__(self, session: AgentSession):
        if not isinstance(session, AgentSession) or session.boot_id is None:
            raise CoordinatorError("coordinator requires a boot-bound agent session")
        self.session = session
        self._lock = threading.RLock()
        self._bound_map: tuple[str, int, int, str] | None = None
        self._bound_revision: int | None = None
        self._blocked_on: tuple[int, int, int] | None = None
        self._release_ids: set[str] = set()

    def to_dict(self) -> dict:
        """Checkpoint alongside AgentSession; caller owns atomic persistence."""
        with self._lock, self.session._strategy_lock:
            return {"schema_version": 1, "session_id": self.session.session_id,
                    "boot_id": self.session.boot_id,
                    "bound_map": None if self._bound_map is None else list(self._bound_map),
                    "bound_revision": self._bound_revision,
                    "blocked_on": None if self._blocked_on is None else list(self._blocked_on),
                    "release_ids": sorted(self._release_ids)}

    @classmethod
    def from_dict(cls, session: AgentSession, raw: dict) -> GoalCoordinator:
        if not isinstance(raw, dict) or set(raw) != {
            "schema_version", "session_id", "boot_id", "bound_map",
            "bound_revision", "blocked_on", "release_ids"
        } or type(raw["schema_version"]) is not int or raw["schema_version"] != 1:
            raise CoordinatorError("invalid coordinator checkpoint schema")
        owner = cls(session)
        bound, blocked, receipts = raw["bound_map"], raw["blocked_on"], raw["release_ids"]
        if (raw["session_id"], raw["boot_id"]) != (session.session_id, session.boot_id):
            raise CoordinatorError("coordinator checkpoint belongs to another session or boot")
        if (bound is not None and (not isinstance(bound, list) or len(bound) != 4
                or bound[0] != session.boot_id or not isinstance(bound[3], str)
                or not bound[3] or any(type(v) is not int or v < 0 for v in bound[1:3]))):
            raise CoordinatorError("invalid bound map checkpoint")
        if (blocked is not None and (not isinstance(blocked, list) or len(blocked) != 3
                or any(type(v) is not int or v < 0 for v in blocked))):
            raise CoordinatorError("invalid blocked revision checkpoint")
        revision = raw["bound_revision"]
        if (revision is not None and (type(revision) is not int or revision < 1
                or revision > session.strategy["goal_revision"])):
            raise CoordinatorError("invalid bound goal revision")
        if (bound is None) != (revision is None) or (bound is None) != (session.execution["lease_id"] is None):
            raise CoordinatorError("checkpoint does not match active session lease")
        if (not isinstance(receipts, list) or len(receipts) > 4096
                or any(not isinstance(v, str) or not v.strip() for v in receipts)
                or len(set(receipts)) != len(receipts)):
            raise CoordinatorError("invalid release receipts")
        owner._bound_map = None if bound is None else tuple(bound)
        owner._bound_revision = revision
        owner._blocked_on = None if blocked is None else tuple(blocked)
        owner._release_ids = set(receipts)
        return owner

    def _decision(self, action: str, reason: str) -> CoordinatorDecision:
        return CoordinatorDecision(action, reason,
            None if self.session.selection is None else self.session.selection.goal.name,
            self.session.strategy["goal_revision"], self.session.session_id)

    def tick(self, context: CoordinatorContext) -> CoordinatorDecision:
        if not isinstance(context, CoordinatorContext):
            raise CoordinatorError("expected validated local context")
        with self._lock, self.session._strategy_lock:
            if context.boot_id != self.session.boot_id:
                return self._decision("identity_mismatch", "another robot boot")
            bound = self.session.execution["lease_id"]
            expected = ((self.session.mission_id, bound) if bound is not None else (None, None))
            if (context.mission_id, context.lease_id) != expected:
                return self._decision("owner_mismatch", "external owner differs from session")
            if bound is not None:
                if not context.local_motion_active:
                    return self._decision("await_owner_release", "local lease not yet acknowledged released")
                if (not context.map_verified or not context.sensors_fresh
                        or context.map_identity != self._bound_map):
                    return self._decision("local_revalidate", "map or sensors changed during local lease")
                if self._bound_revision != self.session.strategy["goal_revision"]:
                    return self._decision("goal_revision_changed", "local owner must assess revised goal")
                return self._decision("continue_local", "onboard controller owns ongoing segment")
            if context.local_motion_active:
                return self._decision("owner_mismatch", "unbound local motion")
            if self.session.strategy["goal_revision"] == 0 or self.session.selection is None:
                return self._decision("await_strategy", "no validated strategic proposal")
            if self.session.selection.goal.name in {"rest", "observe", "acknowledge_person"}:
                return self._decision("intentional_stationary", "goal has no translating segment")
            if not context.map_verified:
                return self._decision("revalidate_map", "map provenance unverified")
            if not context.sensors_fresh:
                return self._decision("revalidate_sensors", "local sensors stale")
            if self._blocked_on == (context.map_epoch, context.map_revision,
                                    self.session.strategy["goal_revision"]):
                return self._decision("blocked_await_new_evidence", "no automatic retry into blocked space")
            return self._decision("plan_next_segment", "advisory only; no route or motor lease")

    def bind_confirmed(self, context: CoordinatorContext, *, goal_revision: int) -> None:
        """Record an externally confirmed exact owner, not create a motor lease."""
        with self._lock, self.session._strategy_lock:
            if (type(goal_revision) is not int or goal_revision != self.session.strategy["goal_revision"]
                    or context.mission_id is None or context.lease_id is None
                    or not context.local_motion_active):
                raise CoordinatorError("stale goal or unconfirmed local owner")
            vacant = CoordinatorContext(context.boot_id, context.map_epoch,
                context.map_revision, context.map_snapshot_id, context.map_verified,
                context.sensors_fresh, context.now_robot, None, None, False)
            if self.tick(vacant).action != "plan_next_segment":
                raise CoordinatorError("goal/map/owner not eligible for handoff")
            try:
                self.session.bind_motion(mission_id=context.mission_id,
                    lease_id=context.lease_id, boot_id=context.boot_id)
            except SessionIdentityError as exc:
                raise CoordinatorError("local owner binding rejected") from exc
            self._bound_map = context.map_identity
            self._bound_revision = goal_revision

    def complete_segment(self, context: CoordinatorContext, *, result: str,
                         release_id: str) -> None:
        """Acknowledge an exact released local owner; no stop is sent here."""
        with self._lock, self.session._strategy_lock:
            if (result not in {"completed", "blocked", "cancelled"}
                    or not isinstance(release_id, str) or not release_id.strip()
                    or release_id in self._release_ids or context.local_motion_active
                    or self._bound_map is None or self._bound_revision is None
                    or context.boot_id != self.session.boot_id
                    or (context.mission_id, context.lease_id) !=
                       (self.session.mission_id, self.session.execution["lease_id"])
                    or context.lease_id is None):
                raise CoordinatorError("terminal report is stale, active, or not exact owner")
            try:
                self.session.release_motion(mission_id=context.mission_id,
                                            lease_id=context.lease_id)
            except SessionIdentityError as exc:
                raise CoordinatorError("local release rejected") from exc
            self._release_ids.add(release_id)
            if len(self._release_ids) > 4096:
                self._release_ids.remove(min(self._release_ids))  # Exact owner still fences old reports.
            if result == "blocked":
                self._blocked_on = (context.map_epoch, context.map_revision,
                                    self.session.strategy["goal_revision"])
            self._bound_map = None
            self._bound_revision = None
