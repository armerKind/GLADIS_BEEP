# BEEP Bridge HTTP API

Current bridge line:

```text
0.22.12-stationary-social-posture (repository source only; not deployed)
```

Last deployed/read-back bridge: `0.22.11-turn-reassess`. The new posture route and helper have only host-side mock coverage; the SDK sit/reset action IDs and physical camera pitch still need supervised commissioning. Stationary Stage 1 passed on `0.22.1-stack2-live-observe` (November 15, 2026). The historical `0.22.2` optional-greeting build was deployed/read back on the Pi and given two explicitly supervised reverse trials, both aborted on front-triplet LiDAR loss before travel was observed.

Base URLs:

```text
http://192.168.8.88:8766                   direct DOGZILLA_WIFI path
http://beep.tailb08b32.ts.net:8766         Tailscale path when online
```

Autonomous navigation and positive-duration primitive endpoints are synchronous: the HTTP response arrives after bounded execution and cleanup attempts. Vendor presets have their own finite action/settle behavior and do not call `stop_burst()`. Client timeouts must exceed the requested duration. Never infer success from a request that timed out before returning; query `/health` and issue `/stop`.

## Health and state

| Endpoint | Purpose |
|---|---|
| `GET /health` | Bridge liveness plus runtime status |
| `GET /status` | Compact state, LiDAR sectors, guarded pose, and SLAM summary |
| `GET /status?full=1` | Status plus full last-run data |
| `GET /last_run` | Full trace from the last autonomous routine |
| `GET /config` | Runtime configuration and motor backend |
| `GET /motion/profile` | Current firmware-gait, pace, body-height, shoulder-yaw and IMU profile |
| `POST /motion/profile` | Apply a posture profile while stationary; HTTP 409 during motion |
| `POST /posture` | Exclusive stationary sit/stand SDK preset, HTTP 409 on motion/mission/lease |
| `GET /events` | Recent rolling event log |
| `GET /pose` | Current accepted pose |

Important status fields:

- `moving`, `posture` (`standing`/`sitting`) and `last_command`
- `scan_age_s` and `sectors`
- `pose.source`, initially a dead-reckoning fallback and normally `guarded_cartographer_slam` after accepted ROS callbacks
- `slam.raw_pose`
- `slam.pose_valid`, `pose_guard_reason`, and `pose_guard_rejections`
- `slam.pose_age_s`, `map_age_s`, dimensions, resolution, known cells, and occupied cells
- `boot_id` and `robot_monotonic_s`; `slam.map_epoch`, `map_revision`,
  `map_revision_status` (`verified` or `unknown`), and `map_snapshot_id`.
  `/slam_map` carries the same provenance, including on a stale-grid error.
  Snapshot IDs advance on validated `/map` callbacks, not on camera requests.
  A grid refresh can change only `map_snapshot_id`, leaving boot/epoch/revision
  stable for an attached Stack 2 lease. Missing `cartographer_ros_msgs` sets
  `slam.submap_error` without disabling `/scan`, `/map`, or TF subscriptions;
  the systemd unit sources `/home/pi/cartographer_ws2/install/setup.bash`
  before starting the bridge when present; a failed setup aborts startup,
  while a missing optional workspace still permits the ROS LiDAR loop.

Cartographer's five-second stationary motion filter can make the generic `slam.active` flag briefly false between stationary pose updates. Controllers apply mode-specific freshness rules. Fresh LiDAR is mandatory for LiDAR-aware approach/navigation routines; generic `/move` and vendor `/action` do not perform scan checks.

## Sensors and observation

| Endpoint | Purpose |
|---|---|
| `GET /scan` | LiDAR sector minima, scan age, and scan count |
| `GET /frame.jpg` | Single JPEG from port 6500; normal reads never connect to port 6000 or send motor-stop packets |
| `GET /camera.jpg` | Camera alias |
| `GET /observe` | Status plus legacy map and camera metadata |
| `GET /slam_map` | Current authoritative Cartographer occupancy grid with origin pose, resolution, age, and cells |
| `GET /local_map` | Robot-frame local safety-map summary |
| `GET /local_map.svg` | Robot-frame local map rendering |

Camera success depends on `/dev/video0`, the Yahboom service on port `6500`, and lighting. Camera failure does not weaken movement safety, but it does reduce human supervision.
The bridge sends the non-motor camera-mode handshake once at startup under
`app_io_lock`; only an actual stream failure retries it. A retry never sends
the app `0x12` stop packet. This needs stationary Pi validation before deployment.

## Stop semantics

```text
GET  /stop
POST /stop
```

`/stop` makes three SDK stop attempts by default and then attempts the optional Yahboom app-socket stop. Reported success means at least one SDK attempt succeeded. App-socket timeout/failure is diagnostic only and must not turn a successful SDK stop into a false motor error; failed SDK calls can still prevent a physical stop.

A redundant `/stop` after every physical routine is intentional.

## Bounded primitive movement

```text
GET  /move?action=<action>&duration=<seconds>&step=<step>
POST /move
```

POST example:

```json
{
  "action": "forward",
  "duration": 0.35,
  "step": 10
}
```

Supported primitive names:

```text
forward  back  left  right  turnleft  turnright  stop
```

Duration is clamped by `BEEP_MAX_MOVE_S` (default 5 seconds). Non-stop actions require a positive duration; `duration=0` is rejected. A primitive issues one bounded movement command, waits continuously, then attempts `stop_burst()`. `/move` performs no LiDAR freshness or clearance check. Autonomous recovery is different: reverse uses the captured full-scale app joystick packet `0x11 [0x00, 0x9C]`, supervised continuously against fresh six-sector LiDAR and usable SLAM, stopping at 0.08 m frontal-clearance gain or aborting on any front-triplet sector <0.38 m or two distinct worsening scans exceeding 0.12 m mean frontal loss, 20-degree yaw drift, rear/side breach, stale data, cancellation, or 1.2 seconds. A failed reverse is not repeated automatically; a stopped mission can select up to two consecutive fully guarded turns after backoff reassessment, resetting on completed forward progress. In-place turns likewise hold one yaw command and poll sensors/SLAM until the requested angle is reached (with at least 10 degrees required for reported success) or a guard stops motion.

## Vendor preset actions

`POST /posture` accepts exactly `{"posture":"sit"}` or `{"posture":"stand"}`. It uses the repository's named SDK presets (sit action 12; reset/neutral action 255) with explicit environment overrides `BEEP_SIT_ACTION_ID` and `BEEP_STAND_ACTION_ID` (1..255), plus bounded settle durations `BEEP_SIT_SETTLE_S` (4.0 default) and `BEEP_STAND_SETTLE_S` (2.0 default). **These IDs come from source mapping, not a physical BEEP posture validation.** A successful SDK receipt is not proof of stable stance or face-level framing; verify with a nearby human before live use. Failure or cancellation latches the software state to `sitting`, sets `posture_action_acknowledged=false`, and blocks locomotion and repeat sit until an explicit successful stand. This receipt bit is **not physical pose verification**. Reboot initializes software to `standing` without measuring actual pose; physical preflight is still required. Direct `/action` sit/reset requests are refused in favor of this route. Neither endpoint can acquire a posture lease during an active mission, another motion lease, or `moving=true`; locomotion and mission admission refuse `sitting`. A lost posture response cannot establish that an action succeeded.

`beep_eyes.social_view.upward_social_turn(..., opt_in=True)` is a separate **host-side** helper: sit, read a bounded `/frame.jpg` JPEG, invoke an injected qualitative captioner, pass the caption to `VoiceLoop.initiate_turn('person_noticed')`, then request stand and verify readback in `finally`. It is not wired to an automatic camera/person trigger or live voice runner. It requires voice consent and explicit opt-in; choose an image provider only under its own household privacy policy. If cleanup loses the connection or fails, actual posture is unknown and a local human must check BEEP; a failed sit request does not blindly issue stand over a possibly active mission.

### Direct audio (0.22.13)

`POST /audio/play?gain=3.0` accepts a fixed-length raw `audio/mpeg` or `audio/wav` body of 1..2,000,000 bytes. Gain defaults to 3.0 and clamps to 0.5..6.0. This endpoint is independent of motor ownership and the SLAM/LiDAR threads: a dedicated nonqueueing audio lock rejects simultaneous playback with 409. It discovers the analog `Headphones` ALSA card by name, unmutes its mixer, converts through `highpass=f=300,volume=<gain>,alimiter=limit=0.95:level=0` with bounded ffmpeg duration/output, and returns `{"ok":true,"elapsed_s":...}` only when `aplay` exits successfully. Bad content type returns 415, missing/invalid length or gain returns 400, oversized body 413, conversion/playback failure 502, and timeout 504. A lost response is **not** evidence that audio did not play: inspect memory and ask the nearby operator before retrying. This endpoint is not authenticated independently of the existing bridge listener; restrict reachability to the trusted LAN/tailnet and do not expose port 8766 to the public internet.

`scripts/run_social_voice.py --speak --bridge-url http://<BEEP-tailnet-IP>:8766 --memory-file <host-bank> ...` synthesizes Sonia on the host, POSTs audio directly over HTTP, and commits the host memory only after HTTP 200 plus `ok:true`. `scripts/social_relay.py --speak --bridge-url ...` first checks the Pi bank, uses this direct audio path, then synchronizes the confirmed memory record back to Pi via Jupyter; *audio is not sent through Jupyter*. A failed memory sync leaves the host bank for reconciliation and must not trigger automatic replay. The older Jupyter audio-transfer route remains an explicit fallback.

| Endpoint | Purpose |
|---|---|
| `GET /actions` or `/tricks` | List known named actions |
| `GET /action?name=stretch&dry_run=1` | Resolve without executing |
| `GET /action?name=pee` | Execute synchronously |
| `GET /action?name=pee&wait=0` | Fire-and-forget |
| `GET /action?id=<1..255>` | Raw vendor action ID |
| `GET/POST /action` or `/trick` | Named/raw action request |

Known names include:

```text
reset  pee  stretch  swing  pray  three_axis  crawl
```

`pee` means only preset 11: lift the right leg. It does not perform target detection, approach, turning, LiDAR gating, or `stop_burst()` cleanup.

## Marking sequence

```text
GET  /mark_object?dry_run=1
POST /mark_object
```

Default sequence:

1. continuously approach generic frontal LiDAR geometry,
2. stop no closer than the 0.38 m default autonomous front floor,
3. perform a LiDAR-sweep-guarded, measured-yaw turn (`90°` by default),
4. execute `pee`.

The routine aborts before turning if the approach does not reach its target safely. Human-leg detection and person-relative target selection are not implemented.

Example:

```json
{
  "target_front": 0.55,
  "max_duration": 5.0,
  "turn": "left",
  "turn_duration": 8.0,
  "dry_run": true
}
```

## Approach behavior

```text
GET/POST /forward_until
GET/POST /learned_forward
GET/POST /approach_front
```

Example:

```json
{
  "target_front": 0.55,
  "max_duration": 5.0,
  "pulse": 0.20,
  "stall_window": 4,
  "stall_delta": 0.01,
  "reorient": false
}
```

`pulse`, `stall_window`, `stall_delta`, and `reorient` remain accepted for API compatibility but are ignored. The endpoint delegates to `forward_continuous_until`; open-loop pulses and blind reorientation are disabled. `max_duration` is clamped by `BEEP_FORWARD_UNTIL_MAX_S` (default 12 seconds), and every autonomous center-front target is clamped to at least 0.38 m by default. The approach requires all six sectors, at least 0.30 m on both front diagonals, and at least 0.24 m on both sides throughout. The subsequent guarded turn requires 0.28 m in every sector and aborts any 0.75-second window that produces less than 10 degrees of measured yaw.

## Navigation modes

### Legacy exploration

```text
GET/POST /explore_room
GET/POST /explore
```

This older endpoint requires a fresh scan and `slam.active` before starting, then maintains the bridge's legacy map. Every motion pulse now uses the same six-sector footprint supervisor and escape-progress checks as the local controller, including turn-sweep validation. It still does not continuously enforce guarded-pose validity, pose/map freshness, or pose rejection during the loop. `save=1` writes the legacy bridge-map JSON, not the action trace. It is retained for compatibility; prefer `/frontier_explore`, `/coverage_explore`, or `/lidar_walk` according to the localization objective.

### Frontier exploration

```text
GET/POST /frontier_explore
GET/POST /dog_explore
GET/POST /explore_frontiers
```

Example dry run:

```text
GET /frontier_explore?name=living_room&max_duration=90&chaos=0.35&seed=42&dry_run=1
```

Dry-run mode still requires fresh LiDAR, pose, and map, executes final stop attempts, and saves its trace by default unless `save=0`.

Behavior:

- uses the authoritative ROS occupancy grid,
- detects and clusters free/unknown boundaries,
- inflates obstacles by a 0.28 m default LiDAR-center radius,
- runs A* through known free space,
- keeps selected frontiers until reached or lost,
- maintains fluent forward gait across compatible replanning windows,
- uses six-sector-supervised turns capped at 0.30 seconds,
- aborts the first turn that does not produce at least 15 degrees of measured heading progress while preserving sweep clearance,
- refuses stale/rejected SLAM or stale LiDAR,
- attempts final SDK stop cleanup.

`chaos` is clamped to `0..1`; `seed` makes weighted selection repeatable. `max_duration` is clamped to 300 seconds.

A completion reason of `coverage_complete_or_no_reachable_frontiers` must be read with the trace. If map updates eliminate routes during turns and no forward motion occurred, it means “no currently reachable planner frontier,” not “the robot traversed the whole room.”

### Stationary motion-profile commissioning

Normal SDK locomotion uses a posture baseline derived from vendor app V2.0.7:
preserve the firmware gait, use normal pace, body height 108, shoulder yaw 0,
and IMU stabilization off. This is not a claim that app width 50 equals SDK step
10; those controls require powered calibration.
The profile is applied once before locomotion and restored after gestures; it is
not rewritten on every velocity update. A stop neutralizes velocity axes without
mutating persistent pace, width, or posture.

```http
GET /motion/profile
POST /motion/profile
Content-Type: application/json

{"body_height":105,"shoulder_yaw":0,"imu":false,"apply":true}
```

`body_height` is restricted to the vendor app's accepted range `76..110`, and
`shoulder_yaw` to `-10..10`. The bridge rejects changes while `moving=true` or a
motion lease exists. `imu` and `apply` must be JSON booleans (or recognized
boolean strings/0/1); malformed values are rejected. If profile I/O fails, the
bridge restores the previous software profile and attempts a physical rollback;
a failed rollback remains an explicit error and leaves the profile dirty for the
next safe initialization. This endpoint is for commissioning on clear floor, not
for live gait improvisation.

### Asynchronous autonomous mission

```text
POST /mission/start
GET  /mission
POST /mission/cancel
POST /mission/bias
```

`POST /mission/start` accepts JSON such as:

```json
{
  "mode": "coverage",
  "duration_s": 190,
  "min_duration": 190,
  "coverage_window": 45,
  "min_growth_cells": 150,
  "save": false
}
```

The bridge validates fresh LiDAR, all six sectors, full-body startup clearance and, for `coverage` and `frontier`, usable guarded SLAM before acquiring an immutable exclusive lease. Unsafe startup geometry returns HTTP `412` without moving. It then returns HTTP `202` with a mission ID while a dedicated onboard worker runs the fluent controller. `GET /mission` and ordinary `/status` requests remain responsive during locomotion, allowing the middle layer to reason, inspect SLAM/LiDAR progress, or cancel without waiting for a long navigation response.

Only one asynchronous or synchronous motion owner may exist. Conflicts return HTTP `409`; they never queue. Cancellation may include the current `mission_id`, preventing a stale planner decision from cancelling a newer mission. Each lease also has an independent deadline watchdog that latches cancellation and attempts repeated SDK stops even if the movement worker stalls.

Modes:

- `coverage`: fluent local LiDAR motion plus guarded-SLAM map/pose and coverage checks, maximum 600 seconds;
- `local`: fluent local LiDAR motion without a global-map claim, maximum 180 seconds.
- `frontier`: occupancy-grid frontier planner under the same asynchronous lease,
  maximum 180 seconds; only this mode consumes frontier-scoring bias.

`POST /mission/bias` accepts JSON including the exact `boot_id`, `mission_id`,
`lease_id` read from `/status` and `/mission`, plus `ttl_s` in `(0, 8]` and at
least one of `preferred_direction` (`left`, `straight`, `right`) or
`preferred_bearing_rad` (finite map-frame radians in `[-π, π]`). Example:

```json
{"boot_id":"<current-boot>","mission_id":"<current-mission>","lease_id":"<current-lease>","ttl_s":4.0,"preferred_direction":"left"}
```

Owner mismatch returns 409; invalid hints return 400. Hints expire on the
robot's monotonic clock and cannot survive lease replacement or cancellation.
Both preference fields `null` explicitly clear the current hint.
Bearing takes precedence over direction when both are supplied. Frontier hints
add only a small heading score to already reachable candidates. Local/coverage
steering hints apply only with fresh six-sector LiDAR and a forward/turn-sweep
clearance pass; they neither command motors directly nor enable semantic
execution (`motor_authority=false`, `semantic_execution_enabled=false`).

The synchronous `/demo_walk`, `/lidar_walk`, `/coverage_explore`, and Friday presentation coordinator remain available unchanged for rehearsal and compatibility.

### Local LiDAR walk

```text
GET /lidar_walk?max_duration=90
GET /demo_walk?max_duration=90
```

The `0.22.9-room-scale-envelope` default profile uses LiDAR-center clearance values (metres): hard 0.20 (floor 0.15), front 0.38 (floor 0.28), diagonal 0.30 (floor 0.22), `FRONT_STOP_M` 0.30 (floor 0.22), side 0.24 (floor 0.18), rear 0.28 (floor 0.22), turn sweep/start 0.28 (floor 0.22), forward corridor 0.45 (floor 0.32), side-steer 0.36 (floor 0.26), and occupancy-grid radius 0.28 (floor 0.22). The hard and front-stop floors are startup clamps, not permission to skip the packet guard. `BEEP_CLEARANCE_PROFILE=commissioning_030` is a distinct legacy override; read `/config.clearance_profile` before motion. The 0.17 m corner-radius estimate is operator-provided, not measured from the photo; these ranges do not prove a particular swept-leg margin.

This presentation-safe fallback makes **no global-SLAM claim**. It:

- keeps one fluent forward SDK gait active across compatible clear windows,
- adjusts yaw in gait with `curveleft`/`curveright`, then clears yaw without neutralizing forward motion,
- polls all six local LiDAR sectors approximately every 80 ms,
- requires 0.45 m forward corridor, 0.38 m front, 0.30 m front diagonals, 0.24 m sides, and 0.28 m rear under the default room-scale profile,
- preserves fluent forward gait across compatible clear windows and steers with in-gait curves rather than stop/start nudges,
- uses a single continuous in-place yaw command (step 30) with live LiDAR/SLAM monitoring; no 0.30-second pivot bursts, and reports a turn only after at least 10 degrees of settled yaw confirmed across three samples,
- uses one continuous app-parity reverse command (step 10 by default, mapping to 100% vendor joystick; optional override is clamped to 10–12) with live six-sector LiDAR and usable-SLAM guards; it ends at 0.08 m frontal-clearance gain or aborts on any front-triplet sector <0.38 m or after two distinct worsening scans exceeding 0.12 m mean frontal loss, 20 degrees yaw drift, rear/side breach, stale data, cancellation, or 1.2 seconds,
- permits an in-place turn when all six sectors meet the 0.28 m start/sweep floor; a genuinely blocked (<0.28 m) sweep can select bounded reverse only with rear/side clearance,
- keeps reverse bounded and one-shot per escape: no automatic reverse retries or yaw-correction bursts; local/legacy walks may reassess up to two consecutive stopped backoffs into guarded turns when scans, SLAM and all sectors meet the 0.28 m start/sweep floor, resetting the handoff counter only after a successful forward segment; frontier exploration drops a blocked route after a successful reverse and replans a new route,
- uses autonomous lateral movement only as a narrowly bounded right-shift turn setup when its front/diagonal/side/rear guards pass; no reverse arcs. The `0.22.10` planner, packet fence and lateral supervisor share directional same-scan admission: `right` requires destination right >=0.24 and away left >=0.20; `left` requires destination left >=0.24 and away right >=0.20. Both also require front >=0.38, diagonals >=0.28 and rear >=0.28 (metres, startup defaults). In-place turns retain their all-sector >=0.28 m sweep,
- requires all six sectors ≥0.28 m before and throughout a turn; the former 0.03 m pre-turn margin is removed at the operator's request, but any measured in-motion breach below the floor aborts,
- in `0.22.11`, allows opposite yaw readings less than 15° during guarded turns; after a wrong-way, no-progress or timed-out turn, the commanded turn stops and local/coverage walks may attempt up to two consecutive stopped, fresh-SLAM, full-sweep reassessments (reset only after a completed forward segment). Frontier mode excludes the failed goal and replans under the same recovery cap. Sensor loss, cancellation, fall and clearance failures remain terminal,
- fails closed on missing sectors, boxed-in geometry, failed progress, or repeated identical escapes; people in view trigger yielding and route replanning, not terminal semantic-goal failure,
- retains the firmware gait/normal pace and does not add per-command posture writes.

The continuous reverse/turn rewrite is deployed, but not physically commissioned. Two supervised reverse trials sent motor packets; the front-clearance guard stopped them after ~0.08 s and ~0.40 s of supervision. The operator saw no movement on the first, and body movement without travel on the second. No turn trial was performed. These aborts do not validate autonomous recovery.

### Coverage-driven exploration

```text
GET /coverage_explore?max_duration=600&min_duration=60&coverage_window=45&min_growth_cells=150
GET /explore_coverage?...same parameters...
```

Coverage mode uses the same fluent local LiDAR movement but requires `pose_valid` not to be false, pose age `<=6.0 s`, and map age `<=2.5 s`. It records Cartographer `known_cells` and stops with `coverage_plateau` when samples span at least 90% of the requested window and `max(known_cells)-min(known_cells) < min_growth_cells`, after `min_duration` has elapsed.

Defaults and clamps:

```text
max_duration       default/max 600 s
min_duration       default 60 s
coverage_window    default 45 s; range 10..180 s
min_growth_cells   default 150
```

The maximum is a safety backstop, not the exploration objective. Coverage plateau is evidence that the currently observable map stopped expanding; it cannot prove that inaccessible or occluded space was physically visited.

Coverage returns:

```json
{
  "mode": "coverage_explore",
  "reason": "coverage_plateau",
  "coverage": {
    "initial_known_cells": 7333,
    "final_known_cells": 10659,
    "growth_cells": 3326,
    "window_s": 45.0,
    "minimum_growth_cells": 150
  }
}
```

## Mapping endpoints

| Endpoint | Purpose |
|---|---|
| `GET /map` | Legacy bridge-map summary |
| `GET /map?full=1` | Full legacy sparse map |
| `GET /slam_map` | Authoritative ROS occupancy grid (flat row-major data; `-1` unknown, `0..49` free, `50..100` occupied) plus origin and freshness |
| `GET /map.svg` | Legacy map rendering |
| `GET /map_reset?name=<room>` | Reset legacy pose/map only |
| `POST /map_reset` | Reset legacy pose/map with JSON pose/name |

These endpoints do **not** restart Cartographer. Use `scripts/jupyter_reset_slam.py` for a clean ROS trajectory and `scripts/jupyter_save_slam.py` to export `.pbstream`, `.pgm`, and `.yaml` artifacts.

## Safety and limitations

- Fresh local LiDAR is mandatory for physical autonomy.
- Immediate collision decisions use local sectors, not network round trips.
- Cartographer has no trusted odometry or integrated IMU input.
- Pose guard rejects implausible localization; it does not repair the missing sensor inputs.
- Global/map-directed claims require accepted guarded pose and fresh ROS map.
- Turn-progress watchdogs prevent indefinite tippy-tapping when heading does not converge.
- Autonomous cleanup paths make SDK stop attempts; software cannot guarantee success if every SDK call fails.
- No movement authentication is currently implemented; expose only on trusted networks.
- No battery telemetry is available.
- If bridge, Jupyter, SSH, Tailscale, and direct DogZilla access all fail, report the action as unexecuted rather than fabricating optimism.
