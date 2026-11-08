# BEEP autonomous embodiment architecture

**Status:** design contract, not a deployed controller. Revised November 8, 2026 after review of the original implementation plan and Gemini's proposed refinements, including five implementation traps. This tracked document is the canonical architecture; the earlier local plan is not required to read it. No part of this document authorizes physical motion.

## Objective and identity

BEEP should behave as a coherent, self-directed robot dog: GLADIS chooses and revises goals from a durable lifetime charter, memory, LiDAR/map, moving video, audio, and the consequences of its actions. Max may override or talk to it, but need not supply an object or route each session. “Consciousness-simulating” means continuity of attention, experience, curiosity and social response—not a claim of subjective sentience. The cap was a replaceable perception test, not the organism's life purpose.

One **embodied session** outlives model calls and motion leases. One active GLADIS strategic owner speaks for BEEP; a future onboard Hermes harness may host that owner only via an explicit handoff, not as a second planner. Pi-resident deterministic control remains the sole motor authority.

```text
  GLADIS / System 2 (seconds; goals, memory, conversation, strategy)
          ↑ evidence + outcomes        ↓ revisioned typed goal proposal
  world belief + place graph       deterministic admission / policy
          ↑                             ↓ exact-session directive
  temporal camera, audio, map      Pi navigation worker (local gait)
          ↑                             ↓ vendor-app motor backend
  Pi local event/attention path    fresh LiDAR / guarded SLAM / watchdog
           \__________________________ local safety veto __________/
```

The **Pi safety loop** reacts to LiDAR, invalid pose/map, collision envelope, cancellation and deadline without any model call. A local map/route loop selects safe corridors and retains compatible gait while steering. Camera/audio collection is concurrent. The slower strategic loop never sets the motor clock; the last validated directive may continue until its own bounded expiry. A hazard, close inspection, conversation requiring a pose change, rest or explicit stop may intentionally interrupt it.

## 1. Latency gap: local attention, not a second fast planner

Gemini is right that a multi-second strategic response is too slow for immediate interaction. It is **not** evidence that a sub-second VLM/SLM will run on this Pi alongside Cartographer, video and audio. Begin with a measured local **attention/event path**: VAD onset/end, loud transient, new motion/visual novelty, fresh LiDAR hazard and local watchdog state. Frame-difference spikes are salience candidates, *not* person/cat classifiers. LiDAR/local safety, not an unverified VLM, handles a crossing animal or person in the immediate path. Benchmark any optional fast visual classifier separately before assigning it a deadline or authority.

Proposed `TacticalEvent` (create `beep_agent/tactical.py`; schema and fake-clock tests in `tests/test_tactical_events.py`):

```json
{
  "schema_version": 1,
  "event_id": "evt-...", "session_id": "...", "boot_id": "...",
  "map_epoch": 4, "map_revision": 7, "mission_id": "...", "lease_id": "...",
  "source": "vad_onset|vad_end|visual_salience|local_hazard|speech_keyword",
  "acquired_at_robot": 0.0, "ttl_ms": 2000,
  "confidence": 0.0,
  "suggestion": "notify_strategy|attention_candidate|ack_cue|request_yield",
  "bearing_hint_rad": null, "evidence_ids": ["..."]
}
```

**Authority:** `notify_strategy` and `attention_candidate` update interest only. A cached acknowledgment sound is allowed only through the audio mixer/privacy policy, not as evidence of understanding a sentence. `request_yield` may enter the **existing deterministic local safety policy**, which must independently validate the stimulus, clearance, motion profile and exact lease; VAD or frame difference alone does not command pace, yaw or a stop. No tactical event changes `goal_id`, sets a route, selects a gait, or bypasses GLADIS. A fast model, if later measured, remains a replaceable event producer under the same contract.

A human saying “Hey BEEP” can produce an immediate attentive audio cue while GLADIS prepares a real response. A human saying “stop” must not depend solely on a slow cloud transcript; commission a separate reliable local/physical stop path. Do not call an RMS spike a recognized stop command.

**Self-induced salience trap:** gait pitch, camera motion, servos and footfalls are not independently interesting events. Condition visual novelty on locomotion state and camera posture: do not emit `request_yield` from raw whole-frame differences while moving. Preserve moving-camera observation using tracked/egomotion-compensated residuals, semantic corroboration or a changed-view track; if compensation is untrustworthy, mark the frame ambiguous rather than suppressing all visual attention. Condition VAD on a measured motor-noise model and double-talk evidence, with rate limits and hysteresis for acknowledgment cues. A higher energy floor during gait is a possible tested parameter, **not** a universal filter: quiet human speech and a stop request must not disappear behind footstep noise. Replay gait-only video/audio, a moving human, quiet speech during gait and false VAD edges; no self-generated event may cause repeated yield/cue cycles. The separate stop path remains independent of these salience heuristics.

## 2. Visual-to-map grounding: a bearing hypothesis, not an invented waypoint

Adopt the geometric idea **only after calibration**. The current BEEP camera optical yaw/FOV and posture-dependent extrinsics are not calibrated; 2D LiDAR may miss a floor cap, table top or glass. The first occupied cell in an image bearing could be a wall *behind* the target. It is therefore a potential **viewpoint**, not the target's metric coordinate.

Proposed `VisualBearing` (new `beep_agent/visual_grounding.py`; `tests/test_visual_grounding.py`):

```json
{
  "frame_id": "...", "track_id": "object-0001",
  "boot_id": "...", "map_epoch": 4, "map_revision": 7,
  "acquired_at_robot": 0.0, "capture_event_id": "evt-frame-...",
  "bbox_norm": [0.35, 0.30, 0.55, 0.72],
  "surface_class": "floor_level|elevated|unknown|occluded",
  "camera_posture": "stand|sit|unknown",
  "calibration_id": "...", "semantic_confidence": 0.0
}
```

**Pipeline:** (1) the model returns a bounded box or `null`, plus surface class and provenance; (2) fetch the guarded capture-time local pose from a short Pi-side history, not the global `map → base_link` pose remembered at inference completion; reject missing/ambiguous epoch, invalid pose, changed camera posture, or excessive interpolation gap; (3) with calibrated horizontal FOV and body-to-camera yaw, convert normalized horizontal center `u` in `[0,1]` to approximate bearing `θ_body(t) + yaw_camera(posture) + atan((2u−1)·tan(HFOV/2))`; expand by box width and measured calibration/SLAM uncertainty, not an arbitrary narrow constant; (4) intersect the cone with free/unknown/occupied map **as uncertainty**, score reachable inflated-map standoff viewpoints, and let fresh local LiDAR validate any route. Never raycast through unknown space and announce the target is at the first occupied cell.

**Loop-closure trap:** `map_epoch` tracks resets, not every global pose-graph correction. BEEP's Cartographer config currently has `provide_odom_frame = true`, `published_frame = "base_footprint"` and `use_odometry = false`: its continuous `odom` frame is *local SLAM*, **not an independent wheel/IMU odometer**. Resolve `odom → base_link` through the actual TF chain (`odom → base_footprint → base_link`), not by assuming Cartographer publishes that transform directly. Stamp each camera acquisition and its resolved local transform with both a robot acquisition timestamp and a ROS TF stamp; validate their clock relationship rather than treating them as the same clock. Retain the local transform and uncertainty in the short ring. For a fresh map query, compose the latest validated optimized `map → odom` with the **capture-time** `odom → base_link` and camera extrinsics. Do not simply subtract two historical `map → base_link` poses across loop closure. Track a map/optimization revision or detect untrusted corrections, and invalidate/reobserve old bearing hypotheses if the relevant submap geometry moved non-rigidly, TF history is missing, or map/pose consistency fails; even a corrected current transform is not a universal repair for older submaps. The bridge currently publishes guarded `map → base_link` snapshots but **does not provide this capture-time transform ring or revision contract**. Fake-TF tests must inject an optimization jump with an unchanged epoch, interpolation gaps, clock skew and non-rigid map changes; none may silently yield a precise waypoint.

For `floor_level`, choose an observable safe viewpoint near the cone; short floor objects may be invisible to planar LiDAR. For `elevated`, propose a wider standoff and, once **stopped and posture-safe**, a `sit`/upward-camera inspection. Neither the table-leg cluster nor the wall is assumed to be the object. A second changed-angle view can narrow a track bearing; only calibrated geometry and consistent localization permit metric triangulation. Until then, a typed `investigate_bearing` remains advisory.

Acceptance: saved cap frames with uncalibrated FOV yield `bearing_only`, **not** a made-up `x,y`; calibrated synthetic scenes produce a conservative viewpoint cone; map reset, stale pose, missing transform, glass/unknown cells or elevated target with no safe standoff return no route.

## 3. Vendor TCP residual motion: reject the 150 ms “proof”

**Do not implement 120–160 ms velocity pulses or claim ≤2 cm residual travel.** BEEP's vendor interface retains axis/profile state, app-native straightening can involve release→restart, and short sub-gait pulses have previously produced inconsistent, unstable bridge motion. A completed `socket.write()` does not prove the app consumed exactly one command or that firmware/servos will stop after one scheduler tick. Repeated zero/start packets can destroy the fluent gait we are trying to build. The proposed numeric bound is unsupported.

Keep one vendor-equivalent persistent motion state under one backend lock. Fence *new* writes after cancellation, serialize the complete stop and any compound transition, give stop priority, and test an already-entered blocked write plus replacement-owner race. **Stop-lock starvation is a present code risk:** `app_transaction()` holds `app_io_lock` across connect, optional greeting, packet writes and delays; owned movement writes are nonblocking but connect/greeting, unowned/stop `sendall`, and lock acquisition still have separate latency paths. `stop_burst()` waits for that same lock. Cancellation visibility cannot by itself free a thread stuck *inside* I/O while holding it. Put explicit end-to-end monotonic deadlines on every transaction and lock wait, fail closed on timeout, and test stalled connect, greeting, partial send and stop send independently. Design a stop-priority I/O worker or separately proven stop path that preserves exact-owner fencing; do not assume closing the socket from another thread will interrupt a Linux blocking syscall, or that replacing the FD prevents already-buffered commands. An independent hardware motor-disable/deadman is the last-resort safety mechanism if transport cannot meet the measured stop budget.

Measure worst-case `sensor_age + scheduling + lock_wait + socket/app consumption + servo/gait stopping` delay and physical stopping distance across speeds and profiles; add body envelope, gait sway, localization uncertainty and response margin. No host-side lock can retract bytes already accepted by TCP. If worst-case residual effect cannot be bounded empirically and by protocol, keep semantic motor dispatch disabled; never invent a 2 cm guarantee. Existing guarded missions and exact cancellation remain separate from the disabled semantic mailbox.

The next hardware gate requires two consistent settled trials per primitive and a stop-under-blocked-write test with physical observation. A shorter command horizon is an *experiment*, not an architecture guarantee; never turn it into a mandatory stop–start every 80 ms.

## 4. Discrepancy monitor: progress signal, not battery or fall sensor

A comparison of commanded gait to guarded SLAM displacement, LiDAR range trends and camera optic-flow/tilt cues is useful to detect **lack of progress**. It is not a reliable voltage meter or fall detector. Commanded step is not calibrated speed; quadruped gait has startup lag, stance oscillation and slip; Cartographer can jump or drift; turning in place has near-zero translation by design. A 10-minute decline in a velocity ratio is confounded by floor, obstacles, posture and localization. “η≈0” does not distinguish cable snag, wall, frozen map, gait transition or fall.

Proposed `MotionDiscrepancy` (new `beep_bridge/progress_monitor.py`; `tests/test_progress_monitor.py`) compares *same-map-epoch* settled progress and independent range/vision evidence within a phase-specific window. Require enough gait cycles, finite healthy pose and straight-gait classification; return `progress_ok | insufficient_evidence | stalled | localization_conflict | possible_fall`, each with contributing signals. `stalled` may prompt a local guarded stop/reassessment after confirmation; `possible_fall` is suspicion, not certified attitude. A gross pose/vision conflict is an immediate veto. Avoid a fixed 1.5 s rule before measuring the actual gait-cycle minimum and localization settling time.

For battery: inspect whether the servo controller exposes trustworthy voltage/current data; otherwise use a bounded runtime/charge policy, supervised sessions and a conservative reserve. Treat declining progress as a maintenance clue, **never** as the sole low-battery trigger. Reliable fall detection needs independent attitude/contact sensing or nearby human supervision; onboard camera and 2D LiDAR cannot certify roll.

## 5. Memory hierarchy: adopt with map-epoch discipline

Gemini's three-tier memory suggestion is sound, but fixed 1.5 m PlaceNodes are arbitrary in this room. Keep:

1. **Working ring:** seconds of timestamped frames/audio, active tracks and fresh LiDAR status. Bounded, local and privacy-aware.
2. **Topological place graph:** observed safe viewpoints, connected traversable regions/doorways, frontier candidates and directional dead ends. Each node stores map epoch, evidence, last visited/viewed, novelty/uncertainty, associated object tracks and reachable edge validation; derive granularity from observed geometry and actual safe footprint, not a universal distance grid. Map reset invalidates metric edges while semantic place labels remain uncertain memories.
3. **Episodes:** short GLADIS-authored summaries of goal outcome, surprise, route failure and social interaction, linked to journal event IDs and evidence. Retrieve a few relevant summaries and graph nodes for strategic prompting, not the entire 45-minute trace.

The **append-only diagnostic journal is a separate audit substrate** with retention/privacy rules; prompt compaction does not authorize deleting raw decision/stop evidence needed for replay. Summaries must distinguish observed, inferred and disproved facts (e.g. “green object was a shoe,” not “found cap”). Implement in `beep_agent/world_model.py` + `beep_agent/place_graph.py` + `beep_agent/episodes.py` with `tests/test_place_graph.py` and `tests/test_episode_compaction.py`.

**Place-node spawning trap:** specify a reproducible policy before building the graph. For a candidate safe viewpoint, compare **aligned, footprint-reachable observed free-space sets** in one map revision (for example, intersection-over-union against the current node); require a sustained loss of overlap *and* meaningful translation/new visibility before spawning. Independently spawn at a persistent, footprint-traversable doorway/clearance bottleneck or confirmed branch/frontier. Add hysteresis, minimum node separation, edge validation and merge/dedupe after relocalization; missing scan coverage or a momentary occlusion cannot manufacture a new place. The proposed 50% overlap is a **tunable test candidate**, not a proven universal threshold. A long featureless corridor must retain a sparse return/continuation breadcrumb when visibility or safe retrace would otherwise be lost; it must not silently produce zero nodes. Test open room, corridor, doorway, repeated viewpoint/rotation, partial occlusion and loop closure, including counts and reachable edges. Geometry beyond the current footprint or unresolved map revision remains unknown rather than a new node.

## 6. Audio: barge-in is required; AEC is a benchmarked option

Do not mute the microphone for the entire TTS reply: a person must be able to interrupt, particularly for a stop. Feed the exact outbound PCM into a render-reference stream and evaluate SpeexDSP or WebRTC audio processing against the USB mic/analog speaker. Echo cancellation requires alignment of playback and capture streams; two independent ALSA devices may drift, and 3× speaker gain can challenge double-talk. The quoted “<3% CPU” is not established for this Pi, device pair or configuration. Benchmark CPU, latency and near-end speech intelligibility with BEEP speaking, walking and a human interrupting.

Phase D adds `beep_audio/echo.py` and `tests/test_audio_barge_in.py`: keep mic/VAD running during playback, attempt AEC with timestamps/delay estimation, detect double-talk and cut/duck TTS on confirmed near-end speech; route transcription through the ordinary audio-event contract. The USB capture and analog playback clocks are independent: estimate relative sample-rate drift from **time-stamped captured and actually rendered frames plus a stable delay estimate**, not an instantaneous ALSA buffer-fill ratio (which also includes scheduling jitter). Check the chosen AEC implementation/version for a real drift-control interface; if absent or inadequate, adaptively resample and align the render reference with bounded correction, reset/reacquire on underruns or route changes, and measure echo return loss and near-end speech over sustained playback. Do not claim a generic “WebRTC drift mode” exists in every backend/version. Replay injected positive/negative ppm drift, playback jitter, robot gait and human double-talk; clock synchronization failure must degrade to short TTS chunks and separate stop authority, not falsely certify barge-in. Until verified, use short TTS chunks and a **separate reliable physical stop**. A mic RMS signal 6 dB above a guessed echo envelope is not a trustworthy speech/stop detector; never treat it as the safety channel. Privacy/consent rules still govern continuous recording and cloud upload.

Primary AEC references: Speex's [echo cancellation API/manual](https://www.speex.org/docs/manual/speex-manual/node7.html) and WebRTC's [audio processing interface](https://webrtc.googlesource.com/src/+/558c2dc5397cbb85db138ef17e36c816c4dad50e/api/audio/audio_processing.h). These establish available techniques, not BEEP-specific CPU or barge-in performance.

## 7. Ownership, contracts and milestones

The three lifetimes from the original design remain: one canonical embodied session, interruptible strategic goal, immutable exclusive navigation lease. A short local attention event never changes the strategic goal. GLADIS proposes; the Pi validates freshness/boot/map epoch/map revision/exact lease/goal revision; the local controller applies only bounded, supported physical effects and reports `accepted/rejected/applied/completed`. Compatible goal-region updates keep the gait and lease; incompatible behaviors intentionally stop. The Pi watchdog and six-sector guard do not depend on GLADIS, System 1 or the audio/cloud provider.

**Phase A prerequisite — version the spatial event envelope before journaling.** `EventEnvelope` v1 is a *proposed new journal schema*, not the existing `beep_agent/session.py` snapshot schema. Define its fields and replay contract in Phase A, before the place graph creates metric nodes; do not bolt the revision/pose fields onto old journal entries in Phase C. An illustrative frame-acquisition event (non-spatial events retain the same keys with `capture_local_transform: null`):

```json
{
  "schema_version": 1, "event_id": "evt-frame-...", "event_kind": "frame_acquired",
  "source": "camera",
  "robot_id": "beep", "session_id": "...", "boot_id": "...",
  "mission_id": null, "lease_id": null,
  "acquired_at_robot": 0.0, "received_at_robot": 0.0,
  "robot_clock": "boot_monotonic", "timestamp_quality": "sensor_exposure",
  "acquired_at_ros_ns": 0,
  "map_epoch": 4, "map_revision": 7, "map_revision_status": "verified",
  "map_snapshot_id": "grid-...",
  "capture_local_transform": {
    "parent_frame": "odom", "child_frame": "base_link",
    "stamp_ros_ns": 0, "x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0,
    "source": "cartographer_local_slam", "valid": true,
    "uncertainty": null
  },
  "evidence_id": "frame-...", "payload": {}
}
```

The bridge owns a monotonically increasing `map_revision` **within each `(boot_id, map_epoch)`**, advanced on a verified optimized-map correction/relocalization (for example, changed global placement of an already known submap). A new LiDAR scan, ordinary local-pose tick or version change of a still-growing submap is **not** by itself a loop closure; occupancy-content updates belong to `map_snapshot_id`. Cartographer's published `SubmapList` includes submap poses and versions but is **not** an explicit global optimization counter. The bridge must validate its detection/provenance rather than fabricating a precise revision from the clock or raw `/map` publication count. If it cannot establish this, record `map_revision: null, map_revision_status: "unknown"` and withhold metric graph updates/visual routes. A transient verified-feed gap invalidates metric state and a healthy superset recovers at an **incremented** revision; a missing known submap requires a new epoch. `map_snapshot_id` separately identifies the occupancy-grid snapshot used for free-space evidence; it is not interchangeable with an optimization revision. Each `place_observation` additionally journals `grid_geometry` (`origin_x_m`, `origin_y_m`, `origin_yaw_rad`, `resolution_m`, `frame_id: "map"`); raw grid indices are converted to fixed map-frame 0.10 m buckets before comparison. Legacy records without grid geometry remain nonmetric. `acquired_at_robot` is a boot-scoped monotonic timestamp, whereas `acquired_at_ros_ns` and transform stamps are ROS time; verify conversion and interpolation bounds before associating them. Agreement between arrival time and TF time is **not** proof of camera exposure time: `timestamp_quality: "sensor_exposure"` requires actual producer evidence before camera spatial readiness; `ingest_approximate` and legacy `unknown` remain nonspatial. Retain the acquisition-time local transform **in the append-only record**, even if the live pose ring later evicts it. If spatial TF is absent or invalid, persist `capture_local_transform: null` and forbid metric effects rather than inventing coordinates. A revision change triggers reproject/revalidate or invalidate affected node geometry and edges; it does not erase the embodied session or semantic memory. `TacticalEvent` and `VisualBearing` carry the same revision and may link to the acquisition event via `capture_event_id`; no event can promote a stale old-revision pose to a current waypoint just by changing its revision number.

**Revised implementation order:**

- **A — identity + memory:** define `EventEnvelope` v1 with boot-scoped `map_revision`/status, map snapshot ID, capture-time `odom → base_link` transform and dual-clock stamps; build the local pose ring and validated revision producer before the graph/journal consumes metric events. Then embodied session across leases, map-revision-aware place graph with deterministic spawning/dedupe, append-only journal and episodes. Offline tests for schema round-trip, unknown-revision fail-closed, map reset, same-epoch loop closure, clock skew, invalid transform, open room/corridor/doorway node counts and prompt compaction preserving diagnostic trace. No live bridge capability is implied.
- **B — strategic + tactical cognition:** lifetime charter and self-chosen goals; strict `TacticalEvent` producer from VAD/visual salience; no second planner, no unsupervised pace/gait change; benchmark optional fast classifier. Tests with delayed System 2, gait-induced camera/audio noise, quiet human speech during gait and immediate Pi safety/vocal acknowledgment.
- **C — visual grounding + motion review:** consume Phase A's immutable acquisition event/local transform and revision without migrating its journal schema; combine with a current optimized transform for a conservative viewpoint cone. Add pure navigation-directive validation, discrepancy monitor, bounded-stop-lock transport races and measured stop-distance characterization. Keep semantic motor dispatch off until physical residual bounds and vendor parity are proven.
- **D — speech:** bounded capture, transcription, AEC/double-talk and independent-clock drift benchmarks, cached acknowledgment, interrupted TTS, separate stop authority. Privacy/consent gate before ambient upload.
- **E — physical commissioning:** BEEP powered later by Max; deploy while stopped, verify exact live version/owner/sensors, then observe-only, short guarded route revision and stop, and finally supervised sustained gait plus social barge-in. Report measured gait-active intervals, displacement, view diversity, model-stall continuity and exact stop state. No plan revision itself moves BEEP.

**Acceptance examples:** A delayed multi-frame model does not pause valid gait; gait-induced frame differences/footsteps do not monopolize attention; a genuine VAD edge cues attention but does not steer; a true obstacle is handled locally before model response; a sighting yields a bearing/viewpoint hypothesis rather than an invented world coordinate even after same-epoch loop closure; an open room avoids duplicate nodes while a corridor remains returnable; changed strategic interest safely revises a compatible route within the *same* lease; a blocked vendor write does not indefinitely delay stop, and no stale movement follows it; a turn-in-place does not look like battery depletion; a human can interrupt TTS despite playback-clock drift; a map reset invalidates old geometry without erasing identity.

**Existing versus proposed (Phase A offline checkpoint; [external review packet](PHASE_A_GATE.md)):** `beep_agent/events.py`, `pose_history.py`, `map_revision.py`, `place_graph.py`, `journal.py`, `episodes.py` and `phase_a.py` now implement an **offline-only** v1 envelope, bounded local transform ring, conservative submap-placement revision observer, provenance-linked viewpoint graph, append-only event replay, episodes and exact-owner session rebinding. Their tests use synthetic verified map/TF evidence. The live bridge does **not yet** publish boot/map epochs, a complete healthy `SubmapList` feed, a capture-time `odom → base_link` transform, or a trustworthy MJPEG exposure timestamp; no live geometric node or route is authorized from this code. Existing `scripts/run_beep_explore.py` and `run_moving_eyes.py` still support one bounded mission with parallel shadow vision and optional two-view target stop. `beep_agent/drives.py` is a heuristic prototype, not GLADIS's persistent strategic mind. `beep_bridge/beep_bridge.py` has local guards, leases and watchdog but semantic motor execution remains hard-disabled. Tactical events, visual target grounding, discrepancy monitoring and AEC remain **design proposals**, not deployed features.
