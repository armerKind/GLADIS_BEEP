# Continuous strategy loop — current boundary and next hardware gate

The system-level design, including reviewed fast attention, metric-grounding limits, vendor transport and audio, is in [Autonomous Embodiment Architecture](AUTONOMOUS_EMBODIMENT_ARCHITECTURE.md). This file is a shorter implementation checkpoint.

BEEP's intended architecture is a persistent semantic goal above a locally guarded locomotion controller. Local LiDAR/SLAM owns immediate steering, collision response, lease deadlines, and motor stop; a camera/audio reasoning loop updates world state and strategy asynchronously. No network inference is allowed to become the gait clock. Intentional inspection, social interaction, genuine hazards, cancellation, or rest may stop locomotion.

## Implemented offline (November 8, 2026)

- `run_beep_explore.py` starts a bounded onboard coverage mission; `run_moving_eyes.py` captures while it moves and builds distinct chronological visual windows. Inference errors leave locomotion and capture alone.
- `--target 'green baseball cap' --stop-on-target` enables a **generic**, strict two-window target confirmation. The detector requires a confident attended object matching the requested descriptive words, the same persistent local track and exact lease, fresh frames, and a measured change of camera pose (at least 0.20 m translation or 0.35 rad rotation). A repeated image, static pose, doubtful model reply, mismatched lease, or single sighting is not a find.
- The observer writes the evidence and exits on confirmation. It sends **no motor command**. The launcher then cancels its **exact** mission/lease, polls for owner release, and includes the finding in its result. Mock perception cannot trigger a physical found-stop. This is target-triggered intentional stopping, not stop–think–start inference.
- Two saved real cap model responses/metadata, reduced to privacy-light tracked fixtures, exercise the confirmation logic. An in-process replay tests the runner's exit and evidence artifact; a fake bridge tests exact-owner cancellation. No physical claim follows from these tests.

- `beep_agent/coordinator.py` now preserves one validated strategic goal across synthetic exact-owner local lease releases, exposes only shadow `plan_next_segment`/`continue_local`/revalidation decisions, and can checkpoint alongside `AgentSession`. The [Phase C gate](PHASE_C_GATE.md) describes the replay of two saved qualitative cap response packets with explicitly synthetic map/lease transitions. Neither those packets nor the coordinator authorize a route or motor command.

## Not yet implemented

- **Live** goal handoff across successive independently verified onboard movement leases. Today the observer and physical mission wrapper still end after one bounded mission. The shadow coordinator is not connected to the local mission API, has no live verified map/sensor feed, and its checkpoint is not atomically journaled with the session. Do not relax the footprint to force progress.
- A strategy update that safely changes navigation course during a running lease. The visual executive remains shadow-only; the onboard semantic motor interlock stays disabled. Do not interpret a target description as steering authority.
- Live microphone capture, speech recognition/event timing, dialogue policy, speaker output, and echo/ducking behavior while gait and camera continue. Known ALSA capture/playback devices are discovery facts, not yet an audio loop.
- A physical test of the new stop-on-target flow. The last verified bridge deployment predates this source change. When BEEP is next powered, deploy while stopped, verify version/status, then run one supervised bounded session and inspect the exact confirmation/stop trace.

The target name is configurable; the cap was merely one test. The next implementation should preserve camera/world/goal identity across local motion handoffs and measure gait-active time, rather than claiming a timed wrapper was continuous exploration.
