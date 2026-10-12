# BEEP Bridge HTTP API

Current bridge line:

```text
0.17.3-moving-camera-handshake
```

This repository version is physically deployed and verified on BEEP. The powered gate covered asynchronous mission start/status/exact cancellation, camera-timeout isolation from motion, and four-frame temporal Gemini perception during gait.

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
| `GET /events` | Recent rolling event log |
| `GET /pose` | Current accepted pose |

Important status fields:

- `moving` and `last_command`
- `scan_age_s` and `sectors`
- `pose.source`, initially a dead-reckoning fallback and normally `guarded_cartographer_slam` after accepted ROS callbacks
- `slam.raw_pose`
- `slam.pose_valid`, `pose_guard_reason`, and `pose_guard_rejections`
- `slam.pose_age_s`, `map_age_s`, dimensions, resolution, known cells, and occupied cells

Cartographer's five-second stationary motion filter can make the generic `slam.active` flag briefly false between stationary pose updates. Controllers apply mode-specific freshness rules. Fresh LiDAR is mandatory for LiDAR-aware approach/navigation routines; generic `/move` and vendor `/action` do not perform scan checks.

## Sensors and observation

| Endpoint | Purpose |
|---|---|
| `GET /scan` | LiDAR sector minima, scan age, and scan count |
| `GET /frame.jpg` | Single JPEG; active leases bypass app-control preparation so capture cannot inject a motor stop |
| `GET /camera.jpg` | Camera alias |
| `GET /observe` | Status plus legacy map and camera metadata |
| `GET /slam_map` | Current authoritative Cartographer occupancy grid with origin pose, resolution, age, and cells |
| `GET /local_map` | Robot-frame local safety-map summary |
| `GET /local_map.svg` | Robot-frame local map rendering |

Camera success depends on `/dev/video0`, the Yahboom service on port `6500`, and lighting. Camera failure does not weaken movement safety, but it does reduce human supervision.

## Stop semantics

```text
GET  /stop
POST /stop
```

`/stop` makes three SDK stop attempts by default and then attempts the optional Yahboom app-socket stop. Reported success means at least one SDK attempt succeeded. App-socket timeout/failure is diagnostic only and must not turn a successful SDK stop into a false motor error; failed SDK calls can still prevent a physical stop.

A redundant `/stop` after every physical routine is intentional.

### External observer stop capability

```text
POST /observer/stop
Authorization: Bearer <BEEP_OBSERVER_STOP_TOKEN>
```

This endpoint is disabled unless the bridge process has a non-empty `BEEP_OBSERVER_STOP_TOKEN`. It accepts only an exact active `mission_id`, an optional `event_id`, and a bounded reason string. Invalid authentication returns `403`; stale/missing missions return `409`; bridge-side rate limiting returns `429`. Accepted requests delegate to exact-ID mission cancellation and its repeated stop sequence. The endpoint cannot issue movement. See [`EXTERNAL_OBSERVER.md`](EXTERNAL_OBSERVER.md).

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

Duration is clamped by `BEEP_MAX_MOVE_S` (default 5 seconds). Non-stop actions require a positive duration; `duration=0` is rejected. A primitive issues one bounded movement command, waits continuously, then attempts `stop_burst()`. `/move` performs no LiDAR freshness or clearance check. Autonomous recovery is different: reverse is a single low-speed command supervised continuously against fresh six-sector LiDAR and usable SLAM, stopping at 0.08 m frontal-clearance gain or aborting on 0.02 m loss, 5-degree yaw drift, rear/side breach, stale data, cancellation, or 1.2 seconds. A failed attempt is not retried automatically. In-place turns likewise hold one yaw command and poll sensors/SLAM until the requested angle is reached (with at least 10 degrees required for reported success) or a guard stops motion.

## Vendor preset actions

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
2. stop no closer than the 0.55 m autonomous front floor,
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

`pulse`, `stall_window`, `stall_delta`, and `reorient` remain accepted for API compatibility but are ignored. The endpoint delegates to `forward_continuous_until`; open-loop pulses and blind reorientation are disabled. `max_duration` is clamped by `BEEP_FORWARD_UNTIL_MAX_S` (default 12 seconds), and every autonomous center-front target is clamped to at least 0.55 m. The approach requires all six sectors, at least 0.42 m on both front diagonals, and at least 0.35 m on both sides throughout. The subsequent guarded turn requires 0.42 m in every sector and aborts any 0.75-second window that produces less than 10 degrees of measured yaw.

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
- inflates obstacles by a conservative 0.35 m robot radius,
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

The bridge validates fresh LiDAR, all six sectors, full-body startup clearance and, for `coverage`, usable guarded SLAM before acquiring an immutable exclusive lease. Unsafe startup geometry returns HTTP `412` without moving. It then returns HTTP `202` with a mission ID while a dedicated onboard worker runs the fluent controller. `GET /mission` and ordinary `/status` requests remain responsive during locomotion, allowing the middle layer to reason, inspect SLAM/LiDAR progress, or cancel without waiting for a long navigation response.

Only one asynchronous or synchronous motion owner may exist. Conflicts return HTTP `409`; they never queue. Cancellation may include the current `mission_id`, preventing a stale planner decision from cancelling a newer mission. Each lease also has an independent deadline watchdog that latches cancellation and attempts repeated SDK stops even if the movement worker stalls.

Modes:

- `coverage`: fluent local LiDAR motion plus guarded-SLAM map/pose and coverage checks, maximum 600 seconds;
- `local`: fluent local LiDAR motion without a global-map claim, maximum 180 seconds.

The synchronous `/demo_walk`, `/lidar_walk`, `/coverage_explore`, and Friday presentation coordinator remain available unchanged for rehearsal and compatibility.

### Local LiDAR walk

```text
GET /lidar_walk?max_duration=90
GET /demo_walk?max_duration=90
```

This presentation-safe fallback makes **no global-SLAM claim**. It:

- keeps one fluent forward SDK gait active across compatible clear windows,
- adjusts yaw in gait with `curveleft`/`curveright`, then clears yaw without neutralizing forward motion,
- polls all six local LiDAR sectors approximately every 80 ms,
- requires 0.65 m forward corridor, 0.55 m front, 0.42 m front diagonals, 0.35 m sides, and 0.45 m rear under the default conservative profile,
- preserves fluent forward gait across compatible clear windows and steers with in-gait curves rather than stop/start nudges,
- uses a single continuous in-place yaw command (step 30) with live LiDAR/SLAM monitoring; no 0.30-second pivot bursts, and reports a turn only after at least 10 degrees of settled yaw confirmed across three samples,
- uses one continuous low-speed reverse command (step 12 by default) with live six-sector LiDAR and usable-SLAM guards; it ends at 0.08 m frontal-clearance gain or aborts on 0.02 m frontal loss, 5 degrees yaw drift, rear/side breach, stale data, cancellation, or 1.2 seconds,
- keeps reverse bounded and one-shot: no automatic retries or yaw-correction bursts; frontier exploration drops a blocked route after a successful reverse and replans a new route,
- uses autonomous lateral movement only as a narrowly bounded right-shift turn setup when its front/diagonal/side/rear guards pass; no reverse arcs,
- requires a 0.45 m start margin before a turn while preserving the 0.42 m hard sweep floor during motion,
- fails closed on missing sectors, boxed-in geometry, failed progress, or repeated identical escapes; people in view trigger yielding and route replanning, not terminal semantic-goal failure,
- retains the firmware gait/normal pace and does not add per-command posture writes.

The continuous reverse/turn rewrite is currently local-source code and unit-tested; it is not deployed or physically commissioned. A successful unit test is not a locomotion result. Live reverse and 10-degree pivot trials remain required before promoting the new control path to autonomous semantic navigation.

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
