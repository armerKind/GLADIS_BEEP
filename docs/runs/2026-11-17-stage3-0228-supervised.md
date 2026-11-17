# Stage 3: 0.22.8 consecutive-backoff handoffs (2026-11-17)

**Requested limit:** 90 s coverage mission with `scripts/run_moving_eyes.py --bias-mission --duration 90`. **Actual mission:** 38.05 s, state `failed`, reason `front_clearance_worsening_during_backoff`. The request was not completed.

## Source and deployment

- `0.22.8-consecutive-backoff-turns`, source commit `dfe7c0d`. Full `NO_PROXY=127.0.0.1,localhost` suite: **380 tests passed**; Python compilation and `git diff --check` passed. Deployed through Jupyter over Tailscale; remote `/config` and `/mission` read back `0.22.8`.
- A stopped backoff can already hand off on `front_clearance_worsening_during_backoff`; the prior limitation was the lifetime one-handoff cap. Both local `lidar_walk` and legacy `explore_room` now allow at most two **consecutive** stopped/revalidated backoff-to-turn handoffs, resetting only on a completed supervised forward/curve segment. Front-loss still stops reverse; a post-stop turn requires fresh scan/SLAM, stationary status, and all six sectors >= 0.42 m. Any in-motion turn sweep breach still stops. Tests exercise two consecutive handoffs, resetting across completed forward, and the 0.456 m front-loss post-stop case.
- The operator confirmed physical proximity, clear floor, and visibly resting BEEP. Repeated remote preflight showed no active mission/lease, fresh advancing scans, valid pose, usable SLAM and verified changing map snapshots. Preflight sectors had minimum about 0.455 m. A camera request returned JPEG markers and `/stop` returned OK while status remained stopped/unowned. The operator subsequently confirmed BEEP visibly at rest; a software stop alone does not prove physical rest.

## Live result

- Mission ID is retained only in local scratch capture. It began with one ordinary right turn (`target_reached:31.4deg`) near the previous ending pose. A reverse after a front-diagonal breach ended `reverse_clearance_gain_not_reached` and made one stopped/revalidated **right-turn handoff** (`target_reached:38.1deg`). Subsequent forward progress reset the handoff counter.
- The trace has **70 entries**: 45 forward, 16 right-curve, two reverse, two right turns, five stop/reassess. Of the two right turns, **one** was a backoff handoff and one an ordinary initial turn. No packet-veto reassessment was recorded.
- The later reverse stopped on `front_clearance_worsening_during_backoff` after about **0.352 s** of continuous reverse supervision (about **0.673 s** including setup/stop), with negligible reported heading drift (-0.42°). Its stopped scan had left **0.376 m**, front-left 0.532 m, front 0.634 m, rear 1.644 m, right 2.723 m. Left was **below the 0.42 m turn sweep**: a turn handoff was correctly refused even though the counter had reset. Four later stationary polls held left around 0.375–0.377 m; this was not a single isolated scan. Do not assume a right turn is safe because the right sector alone is wide open; the full turn sweep includes left.
- Coverage known cells increased **10,238 -> 10,529** (291 cells). This is map growth, not distance or continuous travel.
- Moving eyes: **11 inference windows**, **5 admitted frame events in the Stack 2 journal**, **1 `continue_local` and 4 `goal_revision_changed` coordinator decisions** across the admitted windows, **0 accepted bias hints**. Six other inference windows withheld provenance; the vision described very low light. No motor skill was dispatched by vision.
- Operator response: **“Stopped normally.”** This does not independently establish absence of contact or fall. Two redundant stop requests returned OK. Six post-run readbacks were stopped/unowned, fresh scanning, usable SLAM, verified map revision; four later polls reconfirmed.

## Interpretation and next gate

The previous 28.17 s run had a 0.456 m post-stop minimum, consistent with turn eligibility if a fresh stationary scan and all other guards held. A table-leg/ray-sweep explanation is plausible, not established without a synchronized triggering LiDAR scan and physical geometry measurement. The new run demonstrates the counter reset and first handoff on hardware, **not** a second front-loss-to-turn handoff: the later left sector was persistently below the turn floor. Do not lower the sweep or retry motion from the current pose without new operator direction and fresh preflight.

Detailed local trace: `/home/max/.hermes/profiles/sol/cache/scratch/beep-0228-stage3-live.json`; moving-eyes log alongside it. Raw images/journal remain in local `captures/eyes/moving/20261117T202137-moving-d251efc5/` rather than committed.
