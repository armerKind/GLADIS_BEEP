# 0.22.11 supervised coverage attempt — 2026-11-22

## Delivery and preflight

- Source commit `aa18b38` was pushed to `main`. Full host suite: **388 tests passed**, with `NO_PROXY=127.0.0.1,localhost`.
- Deployed `0.22.11-turn-reassess` via Tailscale/Jupyter and read it back from the bridge. `/config` reported a 15° opposite-heading guard and two consecutive turn recoveries. The Pi booted with **zero scans and unusable SLAM**, so the stationary/unowned recovery restarted the authoritative `YahboomStart` publisher and downstream Cartographer/occupancy services; it did not restart the robot. Scans then advanced and SLAM became usable with a verified map/pose. Two preflight samples showed moving `false`, null lease, fresh scans, usable SLAM, turn sectors above 0.28 m. The operator confirmed they were within reach, BEEP upright and the floor clear before launch.

## One coverage attempt

- Requested `duration_s: 90`, `min_duration: 90`, moving-eyes observer `scripts/run_moving_eyes.py --bias-mission --duration 90`.
- Mission identity: `mission-fd5a7a5778414de18c10d5a544b87ad5-1` (for trace correlation only). During polling, last live sample at **23.5 s** showed running/moving with advancing scan/map. Tailscale bridge requests then timed out; the monitor's automatic cleanup could not verify/cancel during the outage. The assistant instructed the operator to use a physical stop and made no second mission request.
- On reconnection, `/mission` showed **59.26 s**, `moving: false` but the mission was still `running` with an active lease. The assistant cancelled that exact mission and issued `/stop`; readback showed `cancelled`, `moving: false`, null motion lease, **73.2 s** mission elapsed, **72.87 s** result elapsed, reason `motion_cancelled`. These are the measured outcome, **not a completed 90-second run**. The operator reported BEEP stationary and upright with no contact or fall.
- Result trace: **141** entries: 100 forward, 29 right-curves, 6 left-curves, 4 stop/reassess entries, one right turn, one left turn. There was **no wrong-direction turn abort** or `turn_stop_reassess` in this hardware run. Thus the relaxed threshold was host-tested and the run continued past the previous 34.44-second failure duration, but the new stopped-turn recovery path was **not physically exercised**. The directional lateral packet guard still had no lateral action in this trace.
- Coverage grew from **7,676** to **18,432** known cells in the result (**10,756** growth). Moving-eyes log has two inferred scene windows with two accepted rightward lease-fenced mission hints, plus two provenance-withheld entries; these are advisory, not motor dispatch. The observer process did not produce a normal exit report after control-link loss.
- During the final cancellation/readback, the bridge reported `BrokenPipeError` for the vendor app connection. An immediately later sample still had fresh LiDAR but SLAM `pose_guard_invalid` due to stationary drift; a subsequent status request timed out again. Do not infer continuing pose validity, complete network reliability, or a completed mission from the earlier samples. Do not restart motion until the uplink and full stationary sensor/pose/map preflight have been re-established and supervision is freshly confirmed.

## Local artifacts

- Mission launcher: `/home/max/.hermes/profiles/sol/cache/scratch/beep_02211_stage3_once.py`.
- Moving-eyes log: `/home/max/.hermes/profiles/sol/cache/scratch/beep-02211-moving-eyes.log`.
- Moving-eyes capture root: `/home/max/.hermes/profiles/sol/cache/scratch/beep-02211-eyes/`.
- Launcher's nominal JSON output was **not** written, because control-link loss interrupted its cleanup; the results above came from the recovered `/mission` readback.
