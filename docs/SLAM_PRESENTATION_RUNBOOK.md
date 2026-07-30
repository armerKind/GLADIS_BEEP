# BEEP SLAM Presentation Runbook

## Scope

- Up to 10 minutes of autonomous LiDAR + guarded Cartographer SLAM walking.
- Optional stationary `prey`/beg gesture (Yahboom action 17).
- Experimental opt-in corner marking with the pee/right-leg gesture (action 11); disabled by default.
- Camera and microphone are not queried.
- Hard wall clearance: 0.15 m. Marking approach target: 0.20 m.
- Marking turns use guarded Cartographer yaw and stop at approximately 90°; an 8-second timeout accommodates the measured slow-gait turn rate.
- Pee action 11 receives an 8-second completion window so the vendor leg shake is not truncated.
- A human supervisor remains within immediate reach.

## Preconditions

1. BEEP is fully charged or connected to a stable supply.
2. BEEP stands on a dry, level, non-slip floor.
3. Remove cables, feet, bags, fragile objects and drop hazards from the walking area.
4. Keep the laptop connected to `DOGZILLA_WIFI`; wired LAN may provide internet.
5. Verify the stationary plan:

   ```bash
   python3 scripts/run_slam_presentation.py --dry-run --duration 600 --segment 45
   ```

   Continue only if it returns `"ok": true` and reports fresh, valid SLAM.

## Emergency stop

Preferred: press `Ctrl-C` in the presentation terminal. The coordinator sends a final stop.

Independent bridge stop:

```bash
curl -fsS http://192.168.8.88:8766/stop
```

A stop cancels the exact active motion lease. Its cancellation token is never cleared, and overlapping motion requests receive HTTP 409 instead of waiting to execute later.

## Staged rehearsal

Do not skip stages after a software or SLAM configuration change.

### 1. Stationary prey gesture

```bash
curl -fsS 'http://192.168.8.88:8766/action?name=prey&wait=1'
curl -fsS 'http://192.168.8.88:8766/action?name=reset&wait=1'
curl -fsS http://192.168.8.88:8766/stop
```

Confirm BEEP remains balanced and returns to neutral.

### 2. Ten-second guarded SLAM lease

```bash
curl -fsS 'http://192.168.8.88:8766/coverage_explore?max_duration=10&min_duration=10&coverage_window=10&min_growth_cells=1&save=0'
curl -fsS http://192.168.8.88:8766/stop
```

Confirm clearances, gait, turning and stop behavior.

### 3. Thirty-second coordinator run, gestures disabled

```bash
python3 scripts/run_slam_presentation.py --armed --duration 30 --segment 10 --no-prey
```

### 4. Two-minute coordinator run with prey only

```bash
python3 scripts/run_slam_presentation.py --armed --duration 120 --segment 30 --prey-interval 60
```

### 5. Marking test

Only test this in a suitable right-wall corner with at least 0.45 m left-turn clearance. First inspect the dry plan:

```bash
curl -fsS 'http://192.168.8.88:8766/mark_object?target_front=0.20&max_duration=5&turn=left&dry_run=1'
```

For the live test, remove `&dry_run=1`. Keep one hand near BEEP and stop immediately if side clearance narrows.

### 6. Full presentation rehearsal

```bash
python3 scripts/run_slam_presentation.py --armed --duration 600 --segment 45 --prey-interval 150
```

Add `--enable-pee --pee-interval 210` only after the isolated guarded-turn and complete marking tests pass in the presentation area.

## Runtime behavior

- Every navigation lease is capped at 60 seconds; the standard show uses 45 seconds.
- Between leases, BEEP is stopped and LiDAR, SLAM pose, map freshness and 0.15 m clearance are revalidated.
- `prey` runs only while stopped and is followed by neutral reset.
- Pee is attempted only when a front wall, adjacent right wall and clear left turn form a supported corner geometry. Otherwise it is skipped.
- Any stale scan, stale map, invalid pose, request failure, operator interrupt or latched stop aborts the routine and sends a final stop.
- Losing the coordinator leaves at most the current bounded bridge lease; the bridge stops at lease end.
