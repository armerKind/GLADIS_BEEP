# BEEP handoff for a new GLADIS session

Read this page first, then [AUTONOMOUS_EMBODIMENT_ARCHITECTURE.md](AUTONOMOUS_EMBODIMENT_ARCHITECTURE.md) for the design, [bridge_api.md](bridge_api.md) for the HTTP surface, and [deployment.md](deployment.md) before touching the Pi.

## 1. Repository cutover summary

Stack 1 is archived on branch `archive/stack1-cap-prototype` (the state at commit `fddbd20`). Stack 1 was the cap-search prototype and its stop-think-act policy (`beep_eyes/policy.py`, `course.py`), the `ClosedLoopExecutor` and `TargetConfirmation`, the bridge's `IntentMailbox` (`semantic_intent.py`, `/mission/intent`, `/mission/executor/claim`), and the unbuilt external room-camera observer (`beep_observer/`, `/observer/stop`). The fair/presentation scripts and the per-phase gate documents went with it. Check out that branch if you need any of them.

`main` now holds only the Stack 2 embodiment architecture plus the hardware, ROS, vision and audio infrastructure. Bridge source is `0.22.0-stack2-cutover`. It is **not yet deployed**: the Pi still runs `0.21.1-phase-c-audit`, which still serves the removed routes. Local `lidar_walk`, `frontier_explore` and coverage missions no longer pass through the semantic kill-switch. `/status`, `/config` and `/mission` still report `semantic_execution_enabled: false` for backwards-compatible readback.

Validate from the repository root:

```bash
python -m compileall -q beep_agent beep_audio beep_bridge beep_eyes scripts tests
python -m unittest discover -s tests -p 'test_*.py'
```

The loopback HTTP tests fail behind a corporate proxy unless `NO_PROXY=127.0.0.1,localhost` is set.

## 2. What works on physical hardware today (verified Nov 11, 2026)

These results are from the deployed `0.21.1-phase-c-audit` bridge, stationary. No motion trial took place on that build.

- **Onboard bridge** (`beep_bridge/beep_bridge.py`, `192.168.8.88:8766`):
  - Stop priority comes from splitting `app_transition_lock` from `app_io_lock`.
  - A 6-sector LiDAR guard runs at about 10 Hz.
  - Cartographer 2D SLAM is healthy: `SubmapList` every 0.5 s, `/map` every 1.0 s, and the `tf2_ros` chain `odom -> base_footprint -> base_link` resolves in under 0.5 ms median.
  - `frontier_planner.py` provides A* pathing over the inflated occupancy grid.
- **Vision** (`beep_eyes/`):
  - Direct MJPEG stream on port 6500 at 640x480.
  - `MovingFrameRing` filters for frame diversity, and a contact-sheet generator builds the panels.
  - Live `gemini-3.5-flash-lite` structured scene perception (`gemini.py`, `schema.py`).
- **Audio discovery** (`beep_audio/`):
  - Unidirectional PortAudio/ALSA streams were verified while stationary.
  - USB mic: native 48 kHz, or 16 kHz through `plughw`. Analog speaker: 16 kHz.
  - Mic and speaker clocks differ by about 1,024 ppm.
  - ALSA card numbers swap across boots, so bind devices by card name, never by index.

## 3. Stack 2 core modules ready for live wiring

All of these run offline with no bridge motor path. They are tested against synthetic or recorded evidence and are not yet fed by live producers.

**`beep_agent/`**

| Module | Role |
|---|---|
| `events.py` | Immutable acquisition envelope (`EventEnvelope`) with source, timestamp quality and map/TF provenance |
| `journal.py` | Durable append-only `EventJournal`; detects truncation or replacement and never repairs silently |
| `map_revision.py` | `MapRevisionTracker`: conservative revision counter over Cartographer submap placements; fails closed on gaps |
| `pose_history.py` | Bounded ring of capture-time local SLAM transforms, used to look up the pose at exposure time |
| `place_graph.py` | Topological graph of observed safe viewpoints (places, not routes) |
| `episodes.py` | Bounded, provenance-linked summaries over journal records |
| `phase_a.py` | Recorder that journals each decision before mutating the pose ring and place graph; deterministic replay |
| `tactical.py` | `TacticalGate`: strict admission of local attention events; never plans goals or motion |
| `strategy.py` | `StrategicOwner`: typed goal proposals under the lifetime charter, without motor authority |
| `visual_grounding.py` | Exposure-time viewpoint cones from calibrated camera, local TF and matching map revision |
| `viewpoint_scorer.py` | Scores known-free inspection viewpoints inside a ray wedge (a viewpoint, not an object position or path) |
| `coordinator.py` | `GoalCoordinator`: keeps one strategic goal across exact, acknowledged lease handoffs |
| `checkpoint.py` | `CombinedCheckpoint`: atomic, generation-fenced session + coordinator document |
| `model_session.py` | Long-lived GLADIS strategic model-session adapter; one in-flight request, stale windows superseded |
| `observe_supervisor.py` | `ObserveOnlySupervisor`: host-only loop joining recorder, tactical gate, strategy and checkpoint |

Also in `beep_agent/`: `session.py`, `world_model.py`, `drives.py`, `executive.py` and `skills.py` form the persistent `AgentSession` that `scripts/run_moving_eyes.py` updates each window. `body.py` (`BridgeBodyAdapter`) maps typed `explore`/`observe`/`stop` skills onto bounded bridge calls. `fsutil.py` provides `flock` and directory `fsync` on the Pi and falls back cleanly on Windows.

**`beep_audio/`**

| Module | Role |
|---|---|
| `capture.py` | Bounded 16 kHz mono PCM frame contract (no device opened) |
| `vad.py` | Energy VAD with hysteresis (not speech or stop-word recognition) |
| `echo.py` | Echo-reference and barge-in contract; synthetic fixed-delay model, to be replaced by real AEC |
| `adaptive.py` | Independent-clock alignment with linear resampling for the measured mic/speaker drift |
| `social.py` | Social-interruption and ASR handoff state, without playback or cloud calls |

## 4. Immediate next engineering tasks (hardware-first roadmap)

1. **Locomotion reliability.** `BACKOFF_MAX_HEADING_DRIFT_RAD` in `beep_bridge.py` caps reverse heading drift at 5°, and that hair-trigger aborts escapes. Relax it, and hand off from reverse escape into an in-place turn. Acceptance: BEEP completes a 3–5 minute continuous exploration walk without aborting at about 0.46 m.
2. **Read-only camera and telemetry.** `capture_frame()` currently sends the `pkt(0x0F, [1])` mode packet and a stop packet on every `/frame.jpg`. Send the mode packet once at init, then serve frames read-only with no stationary `stop` packets. Expose `MapRevisionTracker` state (`boot_id`, `map_epoch`, `map_revision`) and IMU/battery telemetry in `/status`.
3. **Live camera grounding and goal bias.**
   - Accept `timestamp_quality == "ingest_approximate"` in `events.py` and `visual_grounding.py`, widening the cone by a latency-derived angular margin. Today grounding requires `sensor_exposure`.
   - Add optional `bbox_left`/`bbox_right` to `beep_eyes/schema.py`.
   - Add a lease-fenced `POST /mission/bias` endpoint so Stack 2 (`GoalCoordinator` / `score_viewpoint`) can steer `frontier_planner.py` goal selection without raw motor commands.
