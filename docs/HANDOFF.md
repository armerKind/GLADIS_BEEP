# Fresh-agent handoff — 2026-11-06

> **Offline strategy wiring, November 9, 2026:** [Strategy wiring gate](OFFLINE_STRATEGY_WIRING.md) describes one atomic local JSON checkpoint for `AgentSession` + `GoalCoordinator` and an injected, one-in-flight journal-backed strategic model-session adapter with newest-window supersession. Its audit fixes preserve stationary/historical-lease evidence across handoffs, snapshot world state at prompt construction, supply optional bounded episode/place context and cache journal lookups with incremental external-tail validation. Phase D's offline contracts were approved and closed at `ee09afd`. No live GLADIS provider, Pi producer, audio/bridge deployment, or hardware motion was activated; journal/checkpoint cross-file atomicity and the adapter's restart-durable inference queue remain open.

> **Phase D offline contracts complete, November 9, 2026:** [Phase D gate](PHASE_D_GATE.md) covers bounded timestamp-regressed capture/render clock alignment with linear PCM resampling and synthetic fixed-gain echo subtraction, plus a fake-mixer social interruption, bounded consented PCM handoff and exact-owner one-shot transcript receipt. Slice 2's four audit findings are patched with multi-turn rewarm, smoothed capture timestamps and jitter rejection, bounded pre-onset PCM, and consistent unmeasured-onset abort. These are offline contracts, not actual AEC, ALSA/mixer, cloud ASR, social strategy or motor integration. Hardware motion remains gated.

> **Phase D offline audio start, November 8, 2026:** [Phase D gate](PHASE_D_GATE.md) records bounded injected PCM capture, energy-VAD hysteresis and a fixed-delay synthetic echo/reference and barge-in decision contract. No actual ALSA capture, adaptive AEC, TTS mixer, physical stop, or speech recognition is integrated. Phase C coordinator audit fixes now attribute blocked reports to the bound segment, preserve FIFO receipts, fail closed on lease-owner divergence and admit a new verified occupancy snapshot for fresh *advisory* planning. Hardware motion remains gated.

> **Offline goal-coordinator replay, November 8, 2026:** `beep_agent/coordinator.py` keeps one validated strategic goal across exact, externally acknowledged synthetic lease handoffs with no route or motor authority. `scripts/replay_goal_coordinator.py` consumes saved qualitative cap packets while labeling all map/lease transitions synthetic; [Phase C gate](PHASE_C_GATE.md) records the limitation. No live mission API, atomic coordinator/session journal, live audio, or semantic dispatch was connected. Source bridge remains `0.21.1-phase-c-audit` and deployed bridge remains `0.20.9-steering-sweep`.

> **Phase C offline second slice, November 8, 2026:** [Phase C gate](PHASE_C_GATE.md) now includes a capture-origin, known-free map-wedge viewpoint scorer with heading recomputed from the current pose; a shadow two-window gait/range/vision discrepancy monitor; and source-only app-transport refactoring that allows a fake-socket stop to pass a stalled motion connect/greeting. The source bridge is `0.21.1-phase-c-audit`; last confirmed deployed is `0.20.9-steering-sweep`. Stop's own transport, already-entered write, physical distance and all live producer/integration gates remain open. **Do not deploy or enable semantic motor dispatch based on these synthetic tests.**

> **Phase B offline start, November 8, 2026:** [Phase B gate](PHASE_B_GATE.md) covers strict tactical attention and one revisioned GLADIS goal-proposal owner. These are shadow contracts only: no live VAD/camera/ROS producer, persistent model-session adapter, audio cue output, or motor dispatch exists in this Phase B slice. Do not describe it as an autonomous running robot.

> **Phase A external-audit correction, November 8, 2026:** See [Phase A gate](PHASE_A_GATE.md). Boot-bound READY/between-lease perception, multi-node graph revalidation and same-revision revisits, occupancy-grid origin shifts, transient submap-feed recovery, and journal append scaling are fixed offline with regression tests. Camera spatial readiness now requires proven exposure-time quality; the bridge does not currently supply that proof. Phase B may proceed offline, not as live autonomous or motor-enabled behavior.

> **Architecture update, November 8, 2026:** The canonical design is [Autonomous Embodiment Architecture](AUTONOMOUS_EMBODIMENT_ARCHITECTURE.md). The green cap was found in two moving-camera views on November 8; [trial evidence](CAP_SEARCH_TRIAL.md) and [continuous-strategy checkpoint](CONTINUOUS_STRATEGY.md) supersede the original unfinished-task paragraph below. Semantic motor execution remains disabled. Do not treat the historical 2026-11-06 instructions as the active task.

> **Current update, November 7, 2026:** This file records the original historical handoff, not the current deployment. See [closed-loop status](CLOSED_LOOP_STATUS.md) and [deployment](deployment.md). BEEP's `/scan` publisher was healthy in ROS domain 16; a corrupt 9312×3293 occupancy grid starved the bridge's scan subscriber and drove invalid localization. GLADIS deployed `0.20.9-steering-sweep`, recovered fresh scans and a valid map/pose, and corrected vendor-app joystick scaling after finding that requested step 10 had been transmitted as 10% (actual SDK speed 1). Guarded local trials showed correlated forward SLAM/range progress totaling about 0.46 m. Similar-looking camera frames over that short straight run are not evidence that the camera is frozen: LiDAR/SLAM should drive the gait while temporal vision supplies scene context in parallel. The offline suite passed 291 tests. Semantic executor motion remains hard-disabled. The green cap was merely a replaceable test target, **not** the mission objective.

## Read this first

Max requested a committed, pushed checkpoint and a fresh session because repeated explanations had replaced execution. The goal is an embodied AI robot dog that feels convincingly alive: sustained fluent exploration, current perception driving useful actions, curiosity and social behaviour. The immediate unfinished task was searching for a **green baseball cap**. The search has NOT been performed and the cap has NOT been found. Do not mistake target-aware prompting, shadow decisions, or a stationary camera frame for search progress.

Start with this file, `docs/EMBODIED_MIDDLE_LAYER.md`, `beep_agent/body.py`, `scripts/run_moving_eyes.py`, and `scripts/run_beep_mission.py`. Historical documents contain superseded hardware checkpoints; distinguish source, last reported deployment, and newly measured live state.

## Checkpoint and verification

- Base source at handoff: `48c1f50` (cap-search perception context), following `f439090` (SLAM context panels). Both were locally committed but two commits ahead of origin/main when this checkpoint began.
- Repository bridge source identifies as `0.20.4-slam-map-panel`.
- Full offline suite executed on 2026-11-06: `python -m unittest discover -s tests -v` — **195 tests, OK**, 7.552 seconds. This includes the final perception edits; it is not hardware validation.
- Test log retained locally at `/home/max/.hermes/profiles/sol/cache/scratch/beep-handoff-tests.log` (temporary, not a repository artifact).
- Working tree was clean before handoff documentation. Ignored content includes `.hermes/` local plans, `captures/`, and Python bytecode. These remain local deliberately; do not bulk-add private camera captures, tokens, or generated diagnostics.
- This handoff operation did not contact or move BEEP, deploy code, restart services, or change network configuration.

## Last reported physical state — recheck, do not assume

The immediately preceding Telegram session reported restoring `beep-bridge` through authenticated Jupyter maintenance, without reboot or movement. Its reported health identity was `0.20.3-vendor-app-reverse`, stopped with no active lease and fresh LiDAR; later it reported usable SLAM/map. A live frame showed a dim, red-tinted room and no distinct cap. The source map-panel changes were not deployed. These are recovered session reports, not fresh hardware checks in this handoff.

Earlier README/deployment statements that the bridge remained down and no restart occurred were stale and have been corrected. Current power, connectivity, pose and service state are unknown until read again. Do not reset SLAM or reboot merely to simplify diagnosis.

## What actually exists

- `beep_bridge/beep_bridge.py`: onboard HTTP API, local LiDAR guards, asynchronous mission worker, leases/watchdog, vendor-app motor backend, map/pose endpoints.
- `beep_eyes/`: temporal capture/panels, structured model perception, shadow policy, sensor-gated course recommendations.
- `beep_agent/`: persistent world model, drives, goal selection, executive, session serialization, typed body adapter.
- `scripts/run_moving_eyes.py`: continuous perception during a separately started mission. Updates state and writes next decisions/course recommendations. **Always shadow; dispatch_performed is false.**
- `scripts/run_beep_mission.py`: consumes saved responses under mission ID `replay`, optionally dispatches the final decision once. **Not a continuous live agent supervisor.**
- `BridgeBodyAdapter`: implements explore, observe, stop only. Other semantic skills explicitly lack adapters.

## Independent review and verified gaps

A fresh independent CLI agent completed a read-only audit using provider `openai-codex`, model `gpt-6-astra`. Its session ID is `local-10eae0d7ec1a39238366d7f55771ca1bfb6f35c74e35951f3e9f81d5aae8d934`. The parent directly reread the principal code locations before incorporating these findings:

1. **No perception-to-action feedback loop.** Moving-eyes computes advisory intent but never applies it to the active navigation mission. Parallel perception and locomotion are not yet integrated agency.
2. **Ownership hole in adapter stop.** `beep_agent/body.py` sends an empty `/mission/cancel` payload. Require the executor's exact mission identity; a stale agent must not cancel a replacement mission. Audit the bridge endpoint and existing tests together.
3. **Freshness hole.** Moving-eyes calls `session.update(... observed_at=completed_at)` after inference. Frame-acquisition time, not model-completion time, must govern whether visual evidence is fresh enough to act on.
4. **Replay is not live control.** The one-shot mission script does not bind the returned physical mission into continued perception and execution outcomes.

Do not fix this by changing `shadow` to `supervised`, by mapping arbitrary model text to motor commands, or by writing a hard-coded cap-search choreography.

## Next implementation: one closed-loop vertical slice

Build a mission-owned executor joining moving perception, bounded semantic intent, onboard deterministic validation and execution outcome feedback. Start with course adjustment and stop/yield during an existing exploration mission; explicitly reject unsupported target skills.

- Carry exact mission/lease identity, decision ID, source-frame timestamps, expiry and bounded typed intent.
- Have the onboard controller consume intent at a deterministic navigation boundary, rechecking fresh local safety and ownership.
- Preserve compatible gait; do not cancel/restart or stop after every perception panel.
- Feed accepted/rejected/applied/completed outcomes into the same AgentSession and re-perceive.
- Deduplicate retries and resume; reject stale/superseded results.
- Keep shadow as the default until the slice is tested and explicitly armed.

Acceptance tests with fake bridge/controller:

1. Successive visual windows produce an allowed intent and a revised decision based on its outcome in the SAME mission.
2. Shadow mode makes zero action requests; unsupported skills fail clearly.
3. Expired frame evidence, delayed inference, changed ownership, invalid sensors and superseded intent cannot dispatch movement.
4. Retries, duplicate packets and resume cannot execute the same intent twice.
5. Compatible updates do not restart missions or inject routine stops; lease/watchdog/local safety retain veto power.
6. Stop for mission A cannot cancel replacement mission B; rejection is not reported as success.

Run the entire offline suite, commit coherent changes, then reach the hardware gate. Before powered trials recheck identity, stopped/no lease, fresh camera/LiDAR and valid localization where required. Hardware success requires actual changing views, useful exploration and truthful outcomes—not only green tests.

## Operating constraints

- Nico owns BEEP; Max is developing it. Preserve DOGZILLA_WIFI and automatic Tailscale on BEEP. Prefer direct `192.168.8.88`; bridge `:8766`, Jupyter `:8888`, app `:6000`, camera `:6500`. These are known addresses, not a current reachability claim.
- Vendor-app movements have been safe. Bridge mapping/timing/recreated gait failures are GLADIS integration failures; do not label the robot or vendor primitives unsafe.
- Onboard camera/LiDAR do not reliably establish roll. Preserve collision/lease safeguards; do not interpret urgency as permission to bypass them.
- Develop offline continuously; request power only for genuine hardware gates. Ready/go or voice explore means proceed after safety checks, not repetitive confirmation.
- Analog speaker uses `plughw:1,0` with 3x amplification; USB mic uses `plughw:2,0`, 16 kHz mono. Internal voice PCB is unused.
- Keep technical tool chatter out of Telegram. Report results and blockers concisely. Do not make Max act as the task scheduler.
- Commit frequently; update repo means commit, push, fetch and verify equality.

## Delegation and fresh sessions

The built-in delegate tool was attempted for two independent audits in this handoff, but failed before spawning: configured OpenRouter delegation had no API key. A real separate CLI agent was then successfully run with explicit `--provider openai-codex -m gpt-6-astra`; its findings are above. Do not claim the failed tool spawned workers, and do not waste another session treating that configuration failure as a reason to stop. The CLI fallback is proven; persistent delegation configuration was not modified.

Fresh Telegram context is started by the user sending `/new`; this handoff does not rewrite the live gateway session database. Suggested next message:

> Continue BEEP from /home/max/GLADIS_BEEP/docs/HANDOFF.md. Implement and test the missing closed-loop executor using independent workers/review; keep hardware motion gated. Do not deliver another plan in place of code and exercised tests.
