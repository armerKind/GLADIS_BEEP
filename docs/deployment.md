# BEEP Deployment and Operations

## Source and deployment state

See [HANDOFF.md](HANDOFF.md) for the fresh-agent checkpoint and next engineering tasks.

Repository source: `0.22.9.1-room-scale-lateral-fence`; last deployed/read-back bridge: `0.22.9-room-scale-envelope` pending hotfix deployment. The new source proposes 0.28 m start/in-motion turn sweep and 0.45 m forward corridor; historical 0.42–0.45 m turn-gap closure remains documented below. Any below-floor turn sweep still aborts. The prior `0.22.5` version removes the Stack 1 intent mailbox, `/mission/intent`, `/mission/executor/claim` and `/observer/stop`.

Previous deployed/read-back bridge: `0.22.1-stack2-live-observe` (November 15, 2026). **Stationary Stage 1 passed** after a stopped-only downstream SLAM reset and guarded vendor camera patch. Across eleven seconds, `robot_monotonic_s` advanced `1747.49 → 1758.50`, scans `534 → 644`, verified map revision stayed `0`, `/slam_map` was usable on all twelve polls, and occupancy snapshots advanced `grid-52 → grid-63`. Ten successive `/frame.jpg` responses were decodable 640×480 JPEGs. `last_command=app:stop` and `last_command_at=1794750002.9193337` stayed unchanged; no new `app_send` or stop events appeared. Final readback: no mission, `moving=false`, lease null, `slam.usable=true`. Earlier failed gate and root-cause details are in `HANDOFF.md`. This is **not** moving-hardware validation.

The first supervised reverse request on `0.22.1` returned HTTP 500 after a roughly 0.25-second control-greeting timeout; no reverse packet or moving state was observed. After `0.22.2` deployment via Tailscale, a stopped-only restart of `YahboomStart` restored actual `/scan` publishing (the ROS publisher existed but emitted no scans). Stationary preflight then passed 12/12 polls with verified advancing map, fresh six-sector scans and two decodable JPEGs; camera reads added no motor events. Five read-only control connections returned 14-byte greetings in 3–5 ms, so the intermittent old greeting timeout was **not** reproduced on this boot. The single operator-supervised `GET /supervised_move?action=backward&duration=0.6` returned HTTP 200 with `ok=false`, `reason=front_clearance_worsening_during_backoff`, and supervisor elapsed 0.080 s. One `app_send backward` event was observed, then the stop burst; the ~20 Hz recorder captured two `moving=true` samples. The mean of front/front-left/front-right fell from ~1.105 m at baseline to ~1.080 m at the guarded sample, breaching the unchanged 0.02 m front-loss guard; front-center alone did not decline. The physically present operator reported **no visible movement**. Thus the packet-reached-app event is not evidence of gait or reverse gain; the ~0.08-second abort is consistent with stationary ray variation, but one trial cannot prove the precise sensor cause. Final state was stopped, lease null, mission null, SLAM usable. Trace: session scratch `beep-stage2-reverse-baseline-v2.json`. **Do not retry automatically, weaken the guard, or call this a successful reverse calibration.**

Run the offline suite before any deployment (see the README's Development section). Restart the Cartographer/occupancy/bridge services only when stopped and unowned and after diagnosing a corrupt map; a mapping restart changes the pose origin, so restart the bridge last to clear its old guarded anchor.

At the operator's explicit request, **one** further supervised trial used `duration=1.0` on the same `0.22.2` build and unchanged guards. A five-second stopped 20 Hz baseline beforehand showed the front-triplet mean varied by up to 0.034 m (17/100 samples more than 0.02 m below its first sample), principally from front-right. The second request again sent one `backward` packet and returned HTTP 200 with `ok=false`, `reason=front_clearance_worsening_during_backoff`, after 0.405 s of supervision; first stop packet was ~0.444 s after `app_send backward`. The 20 Hz recorder had nine `moving=true` samples (a bridge command flag, **not** proof of translation). At the guard, front-right dipped to ~1.161 m from ~1.278 m while front-center remained ~1.107 m; the mean fell from ~1.108 m to ~1.069 m. The operator observed **body movement but no travel**. Final state remained stopped, no lease or mission, SLAM usable. Trace: session scratch `beep-stage2-reverse-baseline-v3.json`. The requested longer ceiling did not prolong motion because the guard correctly ended it first. Do not automatically repeat, remove the front-loss guard, or attribute this bridge result to the vendor-app primitive.

`0.22.3` commissioning: BEEP was reachable through its backup AP but the USB AIDA interface reported `connected` without IPv4/default route, leaving its tailnet peer offline. A stopped-only `nmcli connection up AIDA ifname wlan0` restored the Pi's `10.191.225.71/24` address and default route without touching the internal AP. After disconnecting GLADIS from the backup AP, two tailnet pings and bridge/Jupyter HTTP passed over the standard Tailscale address. The bridge deployment passed on-Pi Python compilation, service restart and `/config` readback (`reverse_front_loss_max_m=0.06`, `reverse_step=6`). Eight consecutive post-deploy polls had advancing fresh scans, usable SLAM, rear ~1.94 m, side minimum ~1.08 m, no mission and no motion lease.

Under direct physical supervision, exactly one `GET /supervised_move?action=backward&duration=1.0` sent `app_send backward`, then returned HTTP 200 with `ok=false`, `reason=front_clearance_worsening_during_backoff`, after 0.726 s of supervision. A 20 Hz **status** trace sampled 23 polls/19 distinct `scan_count` values; one sample's front-triplet mean was 1.033 m versus the pre-command 1.115 m (0.082 m loss, exceeding the new 0.06 m ceiling). The first stop packet was observed after the backward packet; two redundant `/stop` calls succeeded. The physically present operator reported **“Movement was there. A little back but as well a bit turning.”** This validates some backward translation, not a straight or sustained gait. Final and later readbacks were `moving=false`, lease null, mission null, fresh scans. SLAM then reported `usable=false`, `pose_guard_invalid` / `stationary_drift`; at that time, map-directed motion required a stopped-only localization reset and stationary recheck. Trace: profile scratch `beep-reverse-tolerance-live-v1.json`. No other `0.22.3` motion trial followed.

`0.22.4` deployment was pushed and installed through the Tailscale Jupyter path after five stopped/unowned/fresh-scan polls. The Pi compiled the uploaded source, restarted the service, and `/config` read back version `0.22.4-app-reverse-settling`, app backend, firmware gait, reverse step 10, front-loss maximum 0.02 m, turn-start 0.45 m, and sweep floor 0.42 m. The first `scripts/jupyter_reset_slam.py` run had no mission or lease and reported all services active/inactive as expected, but live `/mission` showed fresh advancing `/scan`, no map or pose, and `submap_error=ValueError('empty submap list')`. Read-only ROS probes confirmed `/scan` in `laser_frame`, valid ranges/timestamps, transform availability from `laser_frame` to `base_link` and `base_footprint`, and an empty `/submap_list`. A stopped-only Cartographer restart did not help. A subsequent stopped/unowned restart of **YahboomStart** (which owns LiDAR and vendor ROS launch), followed by Cartographer and occupancy grid, restored populated submaps and fresh map/pose; `slam.usable=true` persisted across eight advancing-scan polls. The exact reason the earlier subscriber did not ingest scans is unproven. The reset helper now verifies live readiness and, only if that fails, makes one guarded upstream-plus-downstream recovery attempt before failing rather than reporting service status as success. **One 0.8-second physical reverse trial followed the recovered-SLAM stationary gate.** Ten advancing-scan polls held an unchanged boot/version, five changing map snapshots, verified map, usable/valid pose, all six finite clear sectors and ≤0.006 m stationary x/y spread. `GET /supervised_move?action=backward&duration=0.8` sent one `app_send backward` event, then returned `ok=false`, `reason=front_clearance_worsening_during_backoff` after **0.402 s of supervision** (0.746 s total HTTP request including stop). The operator beside BEEP reported **“Moved a slight bit backwards.”** The 20 Hz host sampler saw 15 status polls/12 distinct scans but did not catch the guard's losing scan; do not label the abort false or infer distance from those samples. Two redundant `/stop` requests returned `ok=true`. The final readback was stopped/unowned with no mission; five settled polls retained `slam.usable=true`, fresh map/pose and no submap error. Pre-to-final accepted pose changed only ~0.009 m and ~1.1°—not reliable evidence of physical distance at this scale. This validates a slight operator-observed backward movement and surviving post-stop SLAM, **not** sustained straight reverse or completion of the requested 0.8 s. Trace: profile scratch `beep-0224-supervised-reverse-live.json`. No repeat was sent.

`0.22.5` was pushed as `71ce608` after 375 passing host tests and deployed through the Tailscale Jupyter path while stopped/unowned. Live `/config` read back reverse step 10, firmware gait, 0.08 m front-loss and 20° yaw limits; twelve post-deploy polls had advancing fresh scans, five changing map snapshots, verified usable SLAM and clear rear/side/front sectors. **Exactly one** `GET /supervised_move?action=backward&duration=0.8` returned `ok=true`, `reason=window_complete`, with **0.800 s** of supervised motor window. Two redundant `/stop` requests succeeded; five settled polls retained `slam.usable=true`, and final `/mission` showed stopped/unowned. The physically present operator reported **backward movement with noticeable turning**. Guarded pre-to-final pose displacement was ~0.056 m with ~19.3° yaw change; that pose estimate is diagnostic, not an independent physical ruler. Front-triplet mean changed from ~1.005 to ~1.024 m, and 28 host status samples/19 distinct scans showed no sustained front-loss breach. This validates a full-duration guarded reverse and post-stop SLAM, **not a straight or commissioned autonomous escape**. Trace: profile scratch `beep-0225-supervised-reverse-live.json`.

See [the `0.22.6` supervised Stage 3 run report](runs/2026-11-16-stage3-supervised.md) and its [sanitized summary](runs/2026-11-16-stage3-summary.json). The requested 90 s asynchronous coverage mission ended after 25.49 s on an app packet collision veto; the operator saw no contact or fall. One reverse reached measured clearance gain, no post-reverse turn handoff executed, six Gemini windows were journaled and four bias hints accepted. Stopped/unowned and usable SLAM were verified afterward. Do not relabel this as a completed 90 s walk.

`0.22.7` source addresses the discovered curve-override bug: the 0.22.6 walk could replace a guarded turn selection with `forward` while the front sector was below the 0.65 m forward packet fence. The old trace last showed `curveright` with 0.688 m front; a later host poll showed 0.624 m front, but the exact vetoing scan/action was not captured. The new source refuses that override, records action/scan/sectors for future packet vetoes, and handles only **pre-write** clearance vetoes as bounded stop/reassess opportunities. A veto after any transmitted bytes or an ownership/cancellation failure still cancels the lease; rear/side reverse and complete turn-sweep floors are unchanged. Host tests do not establish physical success.

The [November 17 `0.22.7` Stage 3 run](runs/2026-11-17-stage3-supervised.md) and [sanitized counters](runs/2026-11-17-stage3-summary.json) document the stopped-only ROS publisher/SLAM recovery, 379 passing host tests, deployed version, and one physically supervised 90 s coverage request that ended after 28.17 s on `front_clearance_worsening_during_backoff`. It made two reverse escapes and one successful stopped-backoff-to-turn handoff, with four admitted Gemini windows and one accepted bias hint. No packet collision veto recurred; the new pre-write veto handling branch remains unexercised on hardware. Operator reported no contact/fall, and repeated final readbacks showed stopped/unowned and usable SLAM. Do not claim a 90 s completion.

`0.22.8` replaces the lifetime one-backoff-turn-handoff limit in `lidar_walk()` and `explore_room()` with at most two consecutive stopped/revalidated handoffs per obstacle encounter. A completed supervised forward segment resets the counter; the prior front-loss stop remains immediate, its reason was already eligible for the stopped turn, and other scan/SLAM/sweep/lease gates remain. The November 17 source trace alone does not prove the side-table ray-sweep causal hypothesis. A run contact sheet shows table legs and shelf geometry, but no synchronized triggering LiDAR ray or body yaw measurement.

The [November 17 `0.22.8` repeat](runs/2026-11-17-stage3-0228-supervised.md) and [sanitized counters](runs/2026-11-17-stage3-0228-summary.json) document 380 passing host tests, deployed readback, and one physically supervised 90 s coverage request ending after **38.05 s** on the same reverse front-loss reason. One stopped-backoff-to-right-turn handoff succeeded after the first reverse; the full trace included 45 forward and 16 right-curve segments before a later backoff. The latter's post-stop left sector was **0.376 m**, still ~0.375–0.377 m in later scans, below the full **0.42 m** turn sweep. Thus the new counter was not the terminal gate: a second turn was correctly denied despite open right-floor clearance. Operator reported normal stop; repeated readbacks were stopped/unowned and usable-SLAM. Five Stack 2 windows were journaled, with zero accepted bias hints. Do not claim a 90 s completion.

`0.22.9` source adopts the operator's room-scale LiDAR-center defaults and 0.28 m occupancy inflation, retaining packet-level clearance, scan/SLAM, ownership, stop and turn-sweep checks. The photo shows visible space between BEEP and a soft backpack but cannot independently measure an 0.17 m corner radius or prove >0.11 m swept-leg margin. The old `0.22.8` post-stop left sector (~0.376 m) is above the proposed 0.28 m turn floor, but that historical scan is not a live motion gate. The requested reverse front-loss rule now trips immediately below 0.38 m on any front-triplet sector or after >0.12 m mean loss across two advancing worsening scans. In particular, reverse that *starts* closer than 0.38 m will abort; this is a known recovery limit, not a bypass of the front guard. The forward corridor is 0.45 m; side steering has its own 0.36 m guard. See [bridge_api.md](bridge_api.md) for startup floors and profile details. Do not equate passing host tests with physical commissioning.

The first physically supervised [0.22.9 room-scale run](runs/2026-11-17-stage3-0229-supervised.md) stopped after **19.50 s**, not 90 s: a low-level packet guard vetoed lateral `right` at left 0.279 m. The high-level lateral envelope allowed >=0.24 m left and >=0.36 m right, but the packet guard applied the 0.28 m all-sector in-place turn sweep. `0.22.9.1` aligns this one lateral packet fence with its supervisor; it does not relax in-place turn checks. Six Gemini windows yielded four journal events, one `continue_local`, three `goal_revision_changed` and zero accepted hints; operator reported normal stop, final polls stopped/unowned with usable SLAM. A fresh gate is mandatory before any retry.

## Pi paths

```text
/home/pi/beep_bridge/beep_bridge.py
/home/pi/beep_bridge/frontier_planner.py
/home/pi/cartographer_ws2/install/cartographer_ros/share/cartographer_ros/configuration_files/beep_2d.lua
/home/pi/gladis_maps/
```

## Runtime services

BEEP normally runs:

```text
beep-bridge.service
beep-cartographer.service
beep-occupancy-grid.service
YahboomStart.service
tailscaled.service
```

The repository units are in `systemd/`. The bridge service sources ROS 2 Foxy and the Cartographer overlay with `ROS_DOMAIN_ID=16`; the deployed bridge uses the vendor app motor backend. ROS diagnostics run without domain 16 can falsely report zero `/scan` publishers. The direct serial owner is the `MS200` node from `YahboomStart.service`.

`YahboomStart.service` is the authoritative vendor launch for the MS200 LiDAR on `/dev/ttyAMA1`. Keep the redundant `XGO_Start.service` disabled: both vendor units launch `oradar_scan`, and two publishers competing for the same serial port can leave `/scan` advertised but silent.

## Jupyter is an onboard maintenance path

JupyterLab listens on port `8888` on BEEP. The browser may run on GLADIS, but notebook kernels and commands execute on BEEP's Raspberry Pi. The helpers use Jupyter because it has sometimes remained easier to authorize than Tailscale SSH.

Host-side Jupyter helpers require Python packages `requests` and `websockets`.

Set the destination explicitly when using Tailscale:

```bash
export BEEP_HOST=beep.tailb08b32.ts.net
```

Local DogZilla access uses the default `192.168.8.88`.

The scripts retain Yahboom's stock-image password as their vendor default. Override changed credentials with `BEEP_JUPYTER_PASSWORD`; do not commit private alternatives, tokens, or generated login material.

Implemented source overrides:

```text
BEEP_BRIDGE_SRC
BEEP_PLANNER_SRC
BEEP_SLAM_CONFIG_SRC
BEEP_SAVE_SLAM_SRC
```

## Deploy bridge, planner, Cartographer config, and map exporter

From the repository root:

```bash
python3 scripts/jupyter_deploy_bridge.py
```

The deployment helper:

1. logs into BEEP's Jupyter server,
2. backs up the deployed bridge,
3. uploads `beep_bridge.py` and `frontier_planner.py`,
4. uploads `config/beep_2d.lua`,
5. uploads and makes executable `scripts/save_slam_map.sh`,
6. compiles the Python source on BEEP,
7. restarts `beep-bridge.service`,
8. verifies that the service is active.

For the **opt-in** vendor camera disconnect fix, set `BEEP_PATCH_VENDOR_CAMERA=1` when deploying while stationary. It uploads `scripts/patch_vendor_app.py`, rejects an unfamiliar vendor single-client/camera source shape, checks the stopped/no-lease mission again and verifies startup stretch is disabled, backs up the installed vendor file, patches only Standard-mode disconnect persistence, then restarts the vendor camera service before the bridge. The control socket still closes after each request; never hold the vendor's single-client port 6000 socket across a camera read and block motor/stop packets. Applied on the Pi on November 15; source was read back, the service was active, and ten valid JPEGs passed the zero-stop gate. Repeat after a vendor-image replacement.

The helper checks stopped/no lease and rejects an active mission before upload and again just before restart; a completed mission in the `/mission` history is allowed. **These are observational checks, not an atomic maintenance lock.** Do not run deployment while another client can start motion concurrently. If the helper errors or times out after upload, query `/health` and the service before retrying; service activation can lag its HTTP listener. It cleans up its temporary Jupyter kernel even on failures.

Verify afterward:

```bash
curl -s http://beep.tailb08b32.ts.net:8766/config | python3 -m json.tool
curl -s http://beep.tailb08b32.ts.net:8766/health | python3 -m json.tool
curl -s http://beep.tailb08b32.ts.net:8766/stop | python3 -m json.tool
```

## Motion-profile commissioning gate

Do not start autonomous exploration immediately after deploying a changed motion
profile. On a full battery and clear level floor:

1. Confirm `/status` reports `moving=false` and no motion lease.
2. Apply the vendor baseline with `POST /motion/profile` using body height `108`,
   shoulder yaw `0`, and IMU `false`.
3. Observe the stationary stance, then run one bounded 1.5-second forward command
   at SDK step `10`; stop and inspect stability and straightness.
4. If the stance is visibly unstable, repeat the same single pass at heights
   `105`, then `95`. Change only height, never gait and height together.
5. Accept a profile only after two straight passes without stumble, lateral drift,
   or persistent yaw. Then run one 8-second supervised forward-biased mission.
6. Autonomous room exploration remains blocked until that gate passes.

This isolates posture from navigation policy. It also prevents the traditional
robotics method of changing five variables and learning nothing.

Before declaring repository source deployed, require `/health` field `status.version` to equal the source version under review, `status.semantic_execution_enabled == false`, `status.moving == false`, and no active lease. Also verify `/slam_map.ok`, sustained six-sector scan/map/pose freshness and no repeated mapper resets after restart. These stationary criteria passed on November 11, 2026 for `0.21.1-phase-c-audit` plus readback-only commit `ddb363a` (the full evidence record is on branch `archive/stack1-cap-prototype`, `docs/PHASE_E_STATIONARY_READBACK.md`). The last guarded **physical motion** trials were on the older `0.20.9-steering-sweep`; this deployment does not establish physical stop priority or authorize motion. A successful upload alone is not deployment.

The Jupyter deployment helper uploads and installs the checked-in systemd unit,
runs `daemon-reload`, explicitly restarts the service, and rejects a live `/config`
that does not match the configured motion profile. Both systemd and the legacy fallback
select `BEEP_MOTOR_BACKEND=app`, matching the vendor app's control protocol for the
entire locomotion lease, including reverse. The selected backend owns the complete
motion lease. The alternate backend is contacted only if the owner's repeated
stop attempts fail; it is not used for ordinary steering or successful stops.

## Install the bridge systemd unit

```bash
python3 scripts/install_systemd_service.py
```

This installer currently targets `beep-bridge.service`. Cartographer and occupancy-grid units are prerequisites and must be installed separately from:

```text
systemd/beep-cartographer.service
systemd/beep-occupancy-grid.service
```

With working SSH, one reproducible installation is:

```bash
scp systemd/beep-cartographer.service systemd/beep-occupancy-grid.service pi@beep:/tmp/
ssh pi@beep 'sudo cp /tmp/beep-cartographer.service /tmp/beep-occupancy-grid.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now beep-cartographer beep-occupancy-grid'
```

Verify on BEEP:

```bash
systemctl is-active beep-bridge beep-cartographer beep-occupancy-grid
```

## Cartographer lifecycle

The active configuration is `config/beep_2d.lua`:

- one 2D LiDAR scan stream,
- no wheel odometry,
- no IMU data,
- 3.5 m maximum range,
- online correlative scan matching,
- stronger translation/rotation penalties,
- 5 s stationary motion-filter interval,
- sparse pose-graph optimization and constraints.

### Start a clean trajectory

Use this after moving BEEP to a different room or after raw localization/map corruption:

```bash
python3 scripts/jupyter_reset_slam.py
```

The helper stops the duplicate `XGO_Start` launch, **keeps** the healthy `YahboomStart` publisher, requires its scan to remain fresh, then restarts Cartographer, occupancy-grid publication, and the bridge. It refuses active motion/leases and discards the unsaved trajectory. On November 15, XGO was already inactive, YahboomStart was active with one MS200 publisher, the base/laser TF chain was present, and Cartographer logged 10 Hz scan insertion and submaps; the old raw pose was approximately (−92.6, −80.7) m and its 1975×1747 grid exceeded the bridge's million-cell ceiling. This stopped-only downstream reset restored a 131×107 grid, a fresh guarded pose, revision 0 and usable `/slam_map`; it was a mapping-origin reset, not evidence that TF was missing. Verify numeric sectors, a fresh guarded pose near the new origin, and a fresh non-empty map before any autonomous movement.

### Pose freshness nuance

While stationary, Cartographer's tuned motion filter may publish accepted pose updates only about every five seconds. The generic `slam.active` field can therefore flicker false between updates even when the map remains fresh. Map-directed controllers use mode-appropriate freshness gates:

- frontier exploration expects tightly fresh pose updates,
- coverage exploration accepts the intentional stationary cadence but still requires a valid guarded pose and fresh map,
- movement always requires fresh local LiDAR.

### Save an active trajectory

```bash
BEEP_MAP_BASE=/home/pi/gladis_maps/living_room \
python3 scripts/jupyter_save_slam.py
```

This calls `/home/pi/beep_bridge/save_slam_map.sh` (uploaded by `jupyter_deploy_bridge.py`), finishes trajectory `0`, and exports:

```text
<base>.pbstream
<base>.pgm
<base>.yaml
```

Finishing the trajectory stops live mapping. Restart/reset Cartographer before a later mapping run. Loading a saved state in production pure-localization mode is not yet automated.

## Exploration workflow

### 1. Preflight

```bash
curl -s http://beep.tailb08b32.ts.net:8766/stop
curl -s http://beep.tailb08b32.ts.net:8766/health | python3 -m json.tool
```

Require:

- `moving: false`,
- fresh `scan_age_s`,
- safe initial sectors,
- fresh map where required,
- `slam.pose_valid: true` for map-directed modes.

### 2. Dry-run a frontier plan

```bash
curl 'http://beep.tailb08b32.ts.net:8766/frontier_explore?name=room&max_duration=90&chaos=0.35&dry_run=1'
```

A dry run must not move the robot. Frontier completion is credible only when the trace shows physical route progress; turns-only traces can lose frontiers as the live map changes.

### 3. Coverage-driven room exploration

```bash
curl 'http://beep.tailb08b32.ts.net:8766/coverage_explore?max_duration=600&min_duration=60&coverage_window=45&min_growth_cells=150'
```

Coverage mode uses fluent local LiDAR motion while monitoring guarded-SLAM known-cell growth. `coverage_plateau` means map growth stayed below the configured threshold for the complete window. The maximum duration is a safety ceiling, not the completion objective.

The synchronous endpoint remains useful for compatibility and the Friday presentation coordinator. For normal embodied operation prefer the asynchronous mission API below so `/status` and cancellation remain responsive throughout locomotion.

### 4. Local fallback

```bash
curl 'http://beep.tailb08b32.ts.net:8766/lidar_walk?max_duration=90'
```

Use this when global SLAM is rejected. It is explicitly local reactive navigation and must not be described as globally mapped coverage.

## Asynchronous mission and moving-eyes hardware gate

Run these stages only with BEEP physically supervised and enough battery/power stability for the requested duration.

### 1. Prime camera while stationary

```bash
curl -fsS http://192.168.8.88:8766/frame.jpg -o /tmp/beep-prime.jpg
curl -fsS http://192.168.8.88:8766/status | python3 -m json.tool
```

Priming allows the vendor MJPEG service to enter its usable state before motion. During an active lease the bridge deliberately bypasses app-control preparation so moving frame capture cannot inject a stop; if the stream is unavailable, capture fails without disturbing locomotion.

### 2. Start a short asynchronous mission

```bash
HTTP_CODE=$(curl -sS -o /tmp/beep-mission.json -w '%{http_code}' \
  -X POST http://192.168.8.88:8766/mission/start \
  -H 'Content-Type: application/json' \
  -d '{"mode":"coverage","duration_s":10,"min_duration":10,"save":false}')
test "$HTTP_CODE" = 202
python3 -m json.tool < /tmp/beep-mission.json

curl -fsS http://192.168.8.88:8766/mission | python3 -m json.tool
curl -fsS http://192.168.8.88:8766/status | python3 -m json.tool
```

`POST /mission/start` must return HTTP 202 quickly while movement continues onboard. `/mission` and `/status` must remain responsive. Conflicting movement must return HTTP 409 rather than queue.

### 3. Validate exact mission cancellation

```bash
MISSION_ID=$(python3 -c 'import json; print(json.load(open("/tmp/beep-mission.json"))["mission"]["id"])')
curl -fsS -X POST http://192.168.8.88:8766/mission/cancel \
  -H 'Content-Type: application/json' \
  -d "{\"mission_id\":\"$MISSION_ID\"}" | python3 -m json.tool
curl -fsS http://192.168.8.88:8766/stop | python3 -m json.tool
```

Require final `moving: false` and `motion_lease_id: null`. A stale mission ID must not cancel a newer owner.

### 4. Validate moving panels

Start another short mission, then attach moving eyes from GLADIS:

```bash
HERMES_HOME="$HOME/.hermes/profiles/sol" \
PYTHONPATH="$PWD:$HOME/.hermes/hermes-agent" \
python3 scripts/run_moving_eyes.py \
  --base-url http://192.168.8.88:8766 \
  --panel 9 --fps 3 --duration 10 \
  --credential-source hermes \
  --hermes-home "$HOME/.hermes/profiles/sol"
```

Compare `--panel 4`, `6`, and `9` on real gait vibration before changing the default. Verify chronological timestamps, usable neighboring frames despite blur, continued capture during Gemini latency, no app-control stop events, responsive mission telemetry, and final stopped/no-lease state. The moving-eyes runner observes and plans but does not dispatch motors.

`--bias-mission` is **off by default**. After stationary provenance and supervised motion commissioning, it may send a four-second advisory `POST /mission/bias` for a confidently tracked left/center/right attention entity (`center` maps to `straight`). It requires the Stack 2 coordinator's `continue_local`, a fresh qualitative view, and a re-read exact mission/lease; the bridge rechecks ownership and LiDAR independently. Person entry alone does not change an existing `explore` goal. A reported hazard, escalation, or weak confidence selects `observe` and withholds advice; when evidence becomes clear, a shadow-attached `explore` goal can resume on the same verified lease. This does **not** stop an already-running onboard mission for a visual hazard. It never starts a mission, sends a route, or enables motor authority. Inspect `semantic_decision.json` (`mission_bias`) and `summary.json` (`bias_accepted`) for accepted/rejected advice. The published `box_far_response.json` and `eyes_response.json` both report hazards; their hazard-free variants are used only to isolate the person-entry continuity test. Do not use this flag as a substitute for the pending reverse-trace diagnosis.

## Audio playback

Generate or provide an audio file, then play it through the Pi analog output:

```bash
python3 scripts/jupyter_play_audio.py message.mp3 --gain 3
```

The Pi requires `ffmpeg`, `aplay`, and a working `plughw:1,0` device. The helper creates `/home/pi/beep_bridge/audio/` through Jupyter before uploading the file.

Working device:

```text
plughw:1,0
```

The helper converts to 24 kHz mono WAV and applies 3× software gain. Higher gain can distort. Playback requires Jupyter/network reachability; successful local TTS generation alone does not prove speaker playback.

## Sync and captures

```bash
python3 scripts/sync_from_pi.py
scripts/capture_bridge_state.sh
```

- Runtime captures go under ignored `captures/`.
- Selected `.pbstream`, `.pgm`, and `.yaml` files may be retained under `assets/ros_maps/`.
- Do not overwrite newer local source with a stale Pi copy without reviewing the diff.

## Movement calibration

The app-port `forward` primitive is not dependable for gait locomotion; use `DOGZILLALib`.

Current controller values:

```text
primitive default step       10
fluent forward VX            20
fluent arc yaw VYAW          30
SDK move_x maximum           20
SDK move_y maximum           18
SDK nonzero turn minimum     30
```

Frontier in-place turns use step 30 and calibrated longer durations. A controlled 1.3 s right turn produced meaningful physical rotation. Higher arc yaw values are not yet validated.

## Networking and boot recovery

The README's Networking section lists the last verified addresses; adapter history is on branch `archive/stack1-cap-prototype` (`docs/fair_remote_control_status.md`). Initial enrollment uses:

```bash
sudo tailscale up --ssh --hostname beep
```

Expected topology:

```text
wlan0  DOGZILLA_WIFI / 192.168.8.88
wlan1  RTL8188EUS external 2.4 GHz uplink
```

If a cold boot fails with the nano adapter inserted:

1. power off,
2. remove the nano adapter,
3. wait about ten seconds,
4. boot and wait for the normal stretch plus `DOGZILLA_WIFI`,
5. insert the nano adapter,
6. wait for uplink and Tailscale.

## Failure handling

- If LiDAR is stale, do not move.
- If Cartographer jumps, reset or use explicitly local navigation; do not relabel dead reckoning as SLAM.
- If Jupyter and Tailscale disappear, try the direct DogZilla network only when physically in range.
- If all paths fail, report the action as not executed.
- After mission completion or cancellation, verify `moving=false` and `motion_lease_id=null`; redundant `/stop` is intentional. Do not expect `moving=false` while intentionally polling an active asynchronous mission.
- If the remote client disappears, the onboard mission remains bounded by its immutable lease and deadline watchdog; restore control and verify final state rather than assuming either continued motion or successful stop.
- No battery telemetry exists.
