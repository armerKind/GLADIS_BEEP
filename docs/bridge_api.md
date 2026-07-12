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
- `GET /action?id=<1..255>` — run a raw vendor preset action ID
- `POST /action` — JSON action request, e.g. `{ "name": "pee", "dry_run": true }`
- `GET /mark_object?dry_run=1` / `POST /mark_object` — fair routine: guarded approach, turn sideways, then `pee`

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
- Local map is more trustworthy than global map for immediate safety.
- Global map quality is marked `low`, `medium`, or `high` based on pose confidence.
- Current scan matching is local and does not perform loop closure.
- If bridge/Jupyter/SSH all time out, assume BEEP is powered down or battery-empty and do not issue further movement commands.
