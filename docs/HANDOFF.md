# BEEP handoff for a new GLADIS session

Read this page first, then [AUTONOMOUS_EMBODIMENT_ARCHITECTURE.md](AUTONOMOUS_EMBODIMENT_ARCHITECTURE.md) for the design, [bridge_api.md](bridge_api.md) for the HTTP surface, and [deployment.md](deployment.md) before touching the Pi.

## 1. Repository cutover summary

Stack 1 is archived on branch `archive/stack1-cap-prototype` (the state at commit `fddbd20`). Stack 1 was the cap-search prototype and its stop-think-act policy (`beep_eyes/policy.py`, `course.py`), the `ClosedLoopExecutor` and `TargetConfirmation`, the bridge's `IntentMailbox` (`semantic_intent.py`, `/mission/intent`, `/mission/executor/claim`), and the unbuilt external room-camera observer (`beep_observer/`, `/observer/stop`). The fair/presentation scripts and the per-phase gate documents went with it. Check out that branch if you need any of them.

`main` now holds Stack 2 plus the hardware, ROS, vision and audio infrastructure. Bridge source and Pi deployment are `0.22.2-stack2-greeting-optional`, read back on November 15, 2026. **Stationary Stage 1 passed on previous build `0.22.1` after camera and mapping recovery.** The first Stage 2 request on `0.22.1` returned HTTP 500 before an observed motor packet. On `0.22.2`, one operator-supervised reverse request reached `app_send backward` but the front-triplet loss guard aborted after 0.080 s of supervision; the operator saw **no movement**. At the operator's request, a second guarded request with a 1.0 s ceiling aborted after 0.405 s on the same front-triplet condition; the operator saw **body movement but no travel**. Stopped baseline LiDAR showed the triplet can cross the 0.02 m loss threshold without motion. No reverse translation was validated. Those trials used the former 0.02 m loss limit; the subsequent source-only `0.22.3-reverse-tolerance` change raises it to 0.06 m and bounds reverse joystick speed at 60% by default. It does not change rear/side clearance, yaw, freshness, or lease guards. This version is not yet deployed or physically verified. The bridge stopped and released its lease after each prior attempt. The previous known-good Stack 1 Pi bridge was `0.21.1-phase-c-audit`. Local `lidar_walk`, `frontier_explore` and coverage missions no longer pass through the semantic kill-switch. `/status`, `/config` and `/mission` still report `semantic_execution_enabled: false` for backwards-compatible readback.

The new source prepares camera mode once at startup, reads `/frame.jpg` from port 6500 without a motor-stop packet, and retries the mode handshake only after a stream failure. The first November 15 attempt failed (ten HTTP 500 camera responses and no usable provenance). Live Pi inspection then confirmed the vendor disconnect resets `g_mode='Home'`; its source-shaped Standard-mode fix was deployed and read back with `beep-vendor-app` active. `XGO_Start` was already inactive, `YahboomStart` active, base/laser static TF present, and Cartographer was inserting scans into submaps. The old Cartographer trajectory reported raw pose approximately (−92.6, −80.7) m and produced a 1975×1747 grid that exceeded the bridge's one-million-cell safety bound, permanently shedding the map subscription. A stopped-only downstream SLAM reset restored a small valid grid; bridge restart rearmed the feed. The bridge now clears a stale `submap_error` on a healthy submap callback. A subsequent **stationary Stage 1 pass** measured verified revision 0 and grid snapshots `grid-52` through `grid-63` over eleven seconds, and ten decodable 640×480 JPEGs with unchanged `last_command`/`last_command_at` and zero new app-send/stop events. The readback is in the session scratch `beep-stage1-live-gate.json`; this is not motion commissioning. The live moving-eyes runner writes a Stack 2 journal and combined checkpoint under an already-running bridge mission; proposals remain advisory. Its opt-in `--bias-mission` sends a four-second lease-fenced direction hint from a current tracked attention entity only after `continue_local`; it never dispatches motor skills. A person entering view alone keeps the `explore` goal; an escalation, reported hazard, or low confidence selects `observe` and withholds bias. On a shadow-attached external lease, a later validated `explore` revision may resume advisory continuity without granting a route or motor authority; goal-specific confirmed leases still require reassessment. Coordinator checkpoint v3 persists shadow-attachment provenance; v2 loads conservatively without it. The bridge still owns LiDAR clearance and motor control. **Nothing in this slice has moving-hardware verification.** The vision-derived goal proposal is a limited adapter, not a commissioned GLADIS strategic model. `--resume-checkpoint` requires the original journal next to the checkpoint; legacy session-only resume is refused.

Validate from the repository root:

```bash
python -m compileall -q beep_agent beep_audio beep_bridge beep_eyes scripts tests
python -m unittest discover -s tests -p 'test_*.py'
```

The loopback HTTP tests fail behind a corporate proxy unless `NO_PROXY=127.0.0.1,localhost` is set.

## 2. Stationary physical readback (verified Nov 15, 2026)

The Stage 1 readback above is from deployed `0.22.1-stack2-live-observe` after recovery. The additional audio and application details below were verified November 11 on `0.21.1-phase-c-audit`; neither build has a moving-hardware Stack 2 trial.

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

Also in `beep_agent/`: `session.py`, `world_model.py`, `drives.py`, `executive.py` and `skills.py` form the persistent `AgentSession` that `scripts/run_moving_eyes.py` updates each window. `body.py` (`BridgeBodyAdapter`) maps typed `explore`/`observe`/`stop` skills onto bounded bridge calls; its separate `bias_mission()` posts only exact-owner advisory hints. `fsutil.py` provides `flock` and directory `fsync` on the Pi and falls back cleanly on Windows.

**`beep_audio/`**

| Module | Role |
|---|---|
| `capture.py` | Bounded 16 kHz mono PCM frame contract (no device opened) |
| `vad.py` | Energy VAD with hysteresis (not speech or stop-word recognition) |
| `echo.py` | Echo-reference and barge-in contract; synthetic fixed-delay model, to be replaced by real AEC |
| `adaptive.py` | Independent-clock alignment with linear resampling for the measured mic/speaker drift |
| `social.py` | Social-interruption and ASR handoff state, without playback or cloud calls |

## 4. Immediate next engineering tasks (hardware-first roadmap)

1. **Locomotion reliability.** `BACKOFF_MAX_HEADING_DRIFT_RAD` caps reverse heading drift at 5°. Do not relax it from the available saved traces: the November 7/8 `heading_drift_during_backoff` event is not present in local `captures/` as a per-sample reverse/SLAM trace. The app backend maps default reverse step 12 to a saturated 100% joystick request; this is a command-scaling concern, not proof of the physical cause. Obtain actual reverse command, guarded/raw yaw, six-sector and independent video samples before changing recovery. Acceptance remains a supervised 3–5 minute continuous walk without collision or uncontrolled abort.
2. **Camera and telemetry.** Stationary camera decoupling and map provenance passed on the Pi after vendor-source patch and downstream mapping reset. Repeat the stopped gate after later startup/deployment before using camera/map observations in motion. IMU/battery telemetry remains unverified/unimplemented; do not invent readings.
3. **Live camera grounding and goal bias.**
   - `events.py` already accepts `ingest_approximate` for qualitative acquisition, but metric `visual_grounding.py` still requires `sensor_exposure`. Do not widen into metric claims without measured exposure/latency and local TF.
   - Add optional `bbox_left`/`bbox_right` to `beep_eyes/schema.py`.
   - `POST /mission/bias` and the opt-in moving-eyes client are implemented offline. Validate them on the Pi only after the stationary/reverse safety gates; qualitative left/center/right labels are not metric map bearings or routes.
