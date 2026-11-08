# BEEP autonomous embodiment architecture

**Status:** design contract, not a deployed controller. Revised November 8, 2026 after review of the original implementation plan and Gemini's proposed refinements. This tracked document is the canonical architecture; the earlier local plan is not required to read it. No part of this document authorizes physical motion.

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
  "map_epoch": 4, "mission_id": "...", "lease_id": "...",
  "source": "vad_onset|vad_end|visual_salience|local_hazard|speech_keyword",
  "acquired_at_robot": 0.0, "ttl_ms": 2000,
  "confidence": 0.0,
  "suggestion": "notify_strategy|attention_candidate|ack_cue|request_yield",
  "bearing_hint_rad": null, "evidence_ids": ["..."]
}
```

**Authority:** `notify_strategy` and `attention_candidate` update interest only. A cached acknowledgment sound is allowed only through the audio mixer/privacy policy, not as evidence of understanding a sentence. `request_yield` may enter the **existing deterministic local safety policy**, which must independently validate the stimulus, clearance, motion profile and exact lease; VAD or frame difference alone does not command pace, yaw or a stop. No tactical event changes `goal_id`, sets a route, selects a gait, or bypasses GLADIS. A fast model, if later measured, remains a replaceable event producer under the same contract.

A human saying “Hey BEEP” can produce an immediate attentive audio cue while GLADIS prepares a real response. A human saying “stop” must not depend solely on a slow cloud transcript; commission a separate reliable local/physical stop path. Do not call an RMS spike a recognized stop command.

## 2. Visual-to-map grounding: a bearing hypothesis, not an invented waypoint

Adopt the geometric idea **only after calibration**. The current BEEP camera optical yaw/FOV and posture-dependent extrinsics are not calibrated; 2D LiDAR may miss a floor cap, table top or glass. The first occupied cell in an image bearing could be a wall *behind* the target. It is therefore a potential **viewpoint**, not the target's metric coordinate.

Proposed `VisualBearing` (new `beep_agent/visual_grounding.py`; `tests/test_visual_grounding.py`):

```json
{
  "frame_id": "...", "track_id": "object-0001",
  "boot_id": "...", "map_epoch": 4,
  "acquired_at_robot": 0.0,
  "bbox_norm": [0.35, 0.30, 0.55, 0.72],
  "surface_class": "floor_level|elevated|unknown|occluded",
  "camera_posture": "stand|sit|unknown",
  "calibration_id": "...", "semantic_confidence": 0.0
}
```

**Pipeline:** (1) the model returns a bounded box or `null`, plus surface class and provenance; (2) fetch the guarded pose at **capture time** from a short Pi-side pose history, not at inference completion; reject missing/ambiguous epoch, invalid pose, changed camera posture, or excessive interpolation gap; (3) with calibrated horizontal FOV and body-to-camera yaw, convert normalized horizontal center `u` in `[0,1]` to approximate bearing `θ_body(t) + yaw_camera(posture) + atan((2u−1)·tan(HFOV/2))`; expand by box width and measured calibration/SLAM uncertainty, not an arbitrary narrow constant; (4) intersect the cone with free/unknown/occupied map **as uncertainty**, score reachable inflated-map standoff viewpoints, and let fresh local LiDAR validate any route. Never raycast through unknown space and announce the target is at the first occupied cell.

For `floor_level`, choose an observable safe viewpoint near the cone; short floor objects may be invisible to planar LiDAR. For `elevated`, propose a wider standoff and, once **stopped and posture-safe**, a `sit`/upward-camera inspection. Neither the table-leg cluster nor the wall is assumed to be the object. A second changed-angle view can narrow a track bearing; only calibrated geometry and consistent localization permit metric triangulation. Until then, a typed `investigate_bearing` remains advisory.

Acceptance: saved cap frames with uncalibrated FOV yield `bearing_only`, **not** a made-up `x,y`; calibrated synthetic scenes produce a conservative viewpoint cone; map reset, stale pose, missing transform, glass/unknown cells or elevated target with no safe standoff return no route.

## 3. Vendor TCP residual motion: reject the 150 ms “proof”

**Do not implement 120–160 ms velocity pulses or claim ≤2 cm residual travel.** BEEP's vendor interface retains axis/profile state, app-native straightening can involve release→restart, and short sub-gait pulses have previously produced inconsistent, unstable bridge motion. A completed `socket.write()` does not prove the app consumed exactly one command or that firmware/servos will stop after one scheduler tick. Repeated zero/start packets can destroy the fluent gait we are trying to build. The proposed numeric bound is unsupported.

Keep one vendor-equivalent persistent motion state under one backend lock. Fence *new* writes after cancellation, serialize the complete stop and any compound transition, give stop priority, and test an already-entered blocked write plus replacement-owner race. Measure worst-case `sensor_age + scheduling + socket/app consumption + servo/gait stopping` delay and physical stopping distance across speeds and profiles; add body envelope, gait sway, localization uncertainty and response margin. No host-side lock can retract bytes already accepted by TCP. If worst-case residual effect cannot be bounded empirically and by protocol, keep semantic motor dispatch disabled; consider an independent hardware motor-enable/deadman path rather than inventing a 2 cm guarantee. Existing guarded missions and exact cancellation remain separate from the disabled semantic mailbox.

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

## 6. Audio: barge-in is required; AEC is a benchmarked option

Do not mute the microphone for the entire TTS reply: a person must be able to interrupt, particularly for a stop. Feed the exact outbound PCM into a render-reference stream and evaluate SpeexDSP or WebRTC audio processing against the USB mic/analog speaker. Echo cancellation requires alignment of playback and capture streams; two independent ALSA devices may drift, and 3× speaker gain can challenge double-talk. The quoted “<3% CPU” is not established for this Pi, device pair or configuration. Benchmark CPU, latency and near-end speech intelligibility with BEEP speaking, walking and a human interrupting.

Phase D adds `beep_audio/echo.py` and `tests/test_audio_barge_in.py`: keep mic/VAD running during playback, attempt AEC with timestamps/delay estimation, detect double-talk and cut/duck TTS on confirmed near-end speech; route transcription through the ordinary audio-event contract. Until verified, use short TTS chunks and a **separate reliable physical stop**. A mic RMS signal 6 dB above a guessed echo envelope is not a trustworthy speech/stop detector; never treat it as the safety channel. Privacy/consent rules still govern continuous recording and cloud upload.

Primary AEC references: Speex's [echo cancellation API/manual](https://www.speex.org/docs/manual/speex-manual/node7.html) and WebRTC's [audio processing interface](https://webrtc.googlesource.com/src/+/558c2dc5397cbb85db138ef17e36c816c4dad50e/api/audio/audio_processing.h). These establish available techniques, not BEEP-specific CPU or barge-in performance.

## 7. Ownership, contracts and milestones

The three lifetimes from the original design remain: one canonical embodied session, interruptible strategic goal, immutable exclusive navigation lease. A short local attention event never changes the strategic goal. GLADIS proposes; the Pi validates freshness/boot/map epoch/exact lease/revision; the local controller applies only bounded, supported physical effects and reports `accepted/rejected/applied/completed`. Compatible goal-region updates keep the gait and lease; incompatible behaviors intentionally stop. The Pi watchdog and six-sector guard do not depend on GLADIS, System 1 or the audio/cloud provider.

**Revised implementation order:**

- **A — identity + memory:** event envelope, embodied session across leases, working ring and epoch-aware place graph, append-only journal, episodes. Tests for map reset, stale object bearing and prompt compaction preserving diagnostic trace.
- **B — strategic + tactical cognition:** lifetime charter and self-chosen goals; strict `TacticalEvent` producer from VAD/visual salience; no second planner, no unsupervised pace/gait change; benchmark optional fast classifier. Tests with delayed System 2 and immediate Pi safety/vocal acknowledgment.
- **C — visual grounding + motion review:** calibrated pose-history lookup and conservative viewpoint cone; pure navigation-directive validation; discrepancy monitor; vendor transport race/stop-distance characterization. Keep semantic motor dispatch off until physical residual bounds and vendor parity are proven.
- **D — speech:** bounded capture, transcription, AEC/double-talk benchmarks, cached acknowledgment, interrupted TTS, separate stop authority. Privacy/consent gate before ambient upload.
- **E — physical commissioning:** BEEP powered later by Max; deploy while stopped, verify exact live version/owner/sensors, then observe-only, short guarded route revision and stop, and finally supervised sustained gait plus social barge-in. Report measured gait-active intervals, displacement, view diversity, model-stall continuity and exact stop state. No plan revision itself moves BEEP.

**Acceptance examples:** A delayed multi-frame model does not pause valid gait; a VAD edge cues attention but does not steer; a true obstacle is handled locally before model response; a sighting yields a bearing/viewpoint hypothesis rather than an invented world coordinate; changed strategic interest safely revises a compatible route within the *same* lease; a blocked vendor write cannot be followed by stale movement after stop; a turn-in-place does not look like battery depletion; a human can interrupt TTS; a map reset invalidates old geometry without erasing identity.

**Existing versus proposed:** current `scripts/run_beep_explore.py` and `run_moving_eyes.py` support one bounded mission with parallel shadow vision and optional two-view target stop. `beep_agent/drives.py` is a heuristic prototype, not GLADIS's persistent strategic mind. `beep_bridge/beep_bridge.py` has local guards, leases and watchdog but semantic motor execution remains hard-disabled. All `TacticalEvent`, visual grounding, discrepancy monitoring, place graph and AEC components above are **design proposals**, not deployed features.
