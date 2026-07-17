# BEEP Bridge API

Base URL when BEEP is online:

```text
http://192.168.8.88:8766
```

## Health and state

- `GET /health` — bridge liveness plus compact status
- `GET /status` — compact state, LiDAR sectors, pose, map summary
- `GET /status?full=1` — status plus full last-run data
- `GET /last_run` — full trace for the last autonomous routine
- `GET /config` — runtime configuration
- `GET /events` — rolling event log
- `GET /pose` — current pose estimate

## Sensors

- `GET /scan` — LiDAR sector minima plus scan age/count
- `GET /frame.jpg` — one camera JPEG from the Yahboom MJPEG stream
- `GET /camera.jpg` — alias for `/frame.jpg`
- `GET /observe` — compact status plus map/camera metadata

## Motion

- `GET /stop` / `POST /stop` — stop burst
- `GET /move?action=<action>&duration=<seconds>&step=<step>` — bounded primitive
- `POST /move` — JSON movement request
- `GET /actions` / `/tricks` — list known Yahboom/DOGZILLA preset SDK actions
- `GET /action?name=<pee|stretch|swing|pray|three_axis|crawl|reset>&dry_run=1` — resolve or run a named preset action
- `GET /action?name=pee&wait=0` or `GET /action?name=pee&async=1` — trigger a preset action and return immediately; preferred for Telegram/voice demos
- `GET /action?id=<1..255>` — run a raw vendor preset action ID
- `POST /action` — JSON action request, e.g. `{ "name": "pee", "dry_run": true }`; use `{ "name": "pee", "wait": false }` for fire-and-forget
- `GET /mark_object?dry_run=1` / `POST /mark_object` — marking routine: one fluent continuous walk to 0.25m under live LiDAR stop control, calibrated 90° left turn, then `pee` (the preset lifts the right leg, placing it beside the target). It aborts before turning if the approach does not reach its target.
- `GET /explore_room?max_duration=30&reset=1&rotate_scan=0` — Cartographer-SLAM-localized LiDAR exploration with visible step-20 forward strides, obstacle-driven turns/strafe escapes, map-frame poses, final stop, and optional JSON trace save. The endpoint refuses to move when the Cartographer pose is unavailable or stale.
- `GET /frontier_explore?max_duration=90&chaos=0.45` — systematic frontier exploration over the authoritative ROS occupancy grid. It clusters free/unknown boundaries, inflates obstacles for BEEP's leg sweep, runs A* through known free space, keeps a selected frontier until reached or stalled, and replans after every supervised gait window. Consecutive forward windows retain one continuous gait command—there are no stop commands between compatible path segments. Stops remain mandatory for hazards, turns, lost paths, reached goals, stale SLAM/LiDAR, completion, and cleanup. `chaos` is clamped to `0..1`: it varies selection among the best reachable frontiers, stride timing, turn duration, and occasional side-opening “curiosity glances” without permitting movement outside planned/sensed-safe space. Optional `seed=<int>` makes a run reproducible. Add `dry_run=1` to return the selected frontier, route, and first gait decision without moving.

Aliases: `/dog_explore` and `/explore_frontiers`.

`/health` and `/status` expose `pose.source=guarded_cartographer_slam` plus a `slam` object containing pose/map freshness, ROS occupancy-grid dimensions, resolution, known cells, occupied cells, raw TF pose, pose-guard validity/reason, and rejection count. The guard anchors stationary pose to a 12cm/0.18rad jitter envelope, rejects impossible moving jumps, and allows two seconds for delayed post-motion scan matching. Frontier navigation refuses to proceed while a pose update is rejected. The authoritative room map is ROS `/map`; the bridge JSON map is only a compact trace/diagnostic rendering.

Example:

```json
{
  "action": "forward",
  "duration": 0.35,
  "step": 10
}
```

Supported SDK-backed actions include:

```text
forward
back
left
right
turnleft
turnright
stop
```

## Approach behavior

- `GET /forward_until`
- `POST /forward_until`
- `/learned_forward` — alias
- `/approach_front` — alias

Example:

```json
{
  "target_front": 0.10,
  "max_duration": 5.0,
  "pulse": 0.20,
  "stall_window": 4,
  "stall_delta": 0.01,
  "reorient": false
}
```

## Mapping

- `GET /local_map` — robot-frame local safety map summary
- `GET /local_map.svg` — robot-frame local LiDAR map SVG
- `GET /map` — global occupancy-map summary
- `GET /map?full=1` — full sparse map data
- `GET /map.svg` — global map SVG
- `GET /map_reset?name=<room>` — reset pose and map
- `POST /map_reset` — reset pose and map with JSON body

## Exploration

- `GET /explore_room`
- `POST /explore_room`
- `/explore` — alias

Example:

```json
{
  "name": "living_room_test",
  "max_duration": 20,
  "reset": true,
  "save": true,
  "rotate_scan": true
}
```

## Safety notes

- All bounded movements end with stop bursts.
- Cartographer uses penalized online correlative matching because BEEP has no odometry or ROS IMU feed; the bridge pose guard is the final drift gate.
- Local map is more trustworthy than global map for immediate safety.
- Global map quality is marked `low`, `medium`, or `high` based on pose confidence.
- Current scan matching is local and does not perform loop closure.
- If bridge/Jupyter/SSH all time out, assume BEEP is powered down or battery-empty and do not issue further movement commands.
