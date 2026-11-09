# Phase B offline cognition checkpoint

**Status:** Offline tactical/strategy shadow-contract slice reviewed and accepted by Max on November 8, 2026. An additional [host-only strategy adapter and paired checkpoint](OFFLINE_STRATEGY_WIRING.md) now exists. No Pi producer, actual GLADIS model connection, audio mixer, motor dispatch, bridge deployment or hardware validation is included; **the live Phase B integration gates below remain open**.

## Implemented

- `beep_agent/tactical.py`: strict bounded `TacticalEvent` v1, exact session/boot/map and active-lease checks, robot-monotonic TTL, bounded deduplication and cue cooldown. Moving-camera raw differences and uncorroborated gait VAD are downgraded to ambiguous attention candidates; quiet confirmed speech can remain an acknowledgment *candidate* when the mixer policy permits it. Only independently verified local hazards can retain a yield suggestion. Every `TacticalDisposition.motor_authority` is false; no cue is played here.
- `beep_agent/strategy.py`: durable lifetime charter, typed `GoalProposal` v1, exact world-sequence/goal-revision/boot/map/TTL admission, target-track corroboration and operator-fixed-goal rejection. `StrategicOwner` commits one goal revision into `AgentSession` under a lock, preserves it through perception updates and lease handoffs, and exposes a bounded memory context. An old proposal ID cannot impersonate a newer goal.
- `beep_agent/session.py` and `executive.py`: strategic goal revision is persisted with the session; deliberate observe/rest returns a nonphysical observe call. Existing packet-local heuristic continues as the fallback until a GLADIS proposal is admitted.

## Run tests

- `python -m unittest discover -s tests -p 'test_tactical_events.py' -v`
- `python -m unittest discover -s tests -p 'test_strategy_phase_b.py' -v`
- `python -m unittest discover -s tests -q`

Synthetic tests cover delayed/stale model replies, exact lease handoff, wrong map revision, gait-pitch salience, footsteps versus corroborated quiet speech, cue cooldown, duplicate tactical IDs, operator override, marked-object geometry, and concurrent same-revision goal proposals. They do **not** measure Pi inference latency or detect real speech through gait noise.

## Remaining Phase B gates

1. Wire actual camera/VAD/LiDAR producers with measured acquisition clocks and provenance. Frame difference and mic RMS alone must never claim a person, recognized keyword, or motor stop.
2. Connect the *one* actual GLADIS strategic model session to the host-tested journal-backed adapter, then prove live single-inference/stale-window behavior. The injected offline provider and persistent adapter object are **not evidence that an LLM is running**; its pending queue is not restart-durable.
3. Route any cached cue through the real consent/privacy/audio mixer policy, measure gait-only noise and quiet speech, and preserve a separate reliable stop method.
4. Benchmark optional fast classifier on BEEP's Pi under ROS/video/audio load before asserting a latency bound. Add replay fixtures from actual gait and speech; prove no second goal planner and no motor side effects.

**Physical gate:** The semantic executor remains hard-disabled. Offline tactical `request_yield` is only an advisory request to the deterministic local policy, never a stop or steering command.
