# BEEP Eyes: Temporal Perception Architecture

## Objective

Give GLADIS a closed perception loop through BEEP's front camera:

```text
continuous camera capture during locomotion
→ bounded timestamped frame ring
→ newest chronological 4/6/9-frame panel
→ interpret scene and change
→ select an attention target
→ propose bounded heading, movement, gesture, or speech skills
→ apply deterministic policy and bridge safety gates
→ keep the exclusive mission lease moving under local LiDAR/SLAM guards
→ merge completed semantic evidence into the persistent world model
→ continue, adjust at a safe boundary, inspect deliberately, or abort for a real hazard
```

The model proposes semantic skills. It never emits motor registers, velocities, arbitrary action IDs, or direct bridge commands. The deterministic bridge remains the sole motor and safety authority.

## Current milestone: continuous moving temporal eyes

Repository implementation is complete through mock/integration validation. The stationary camera/Gemini path has been exercised on BEEP, but bridge `0.17.1-moving-eyes` and moving capture during real gait await the next powered hardware gate. Panel count remains configurable because 4/6/9 must be compared under actual vibration rather than selected through numerology.

Implemented in `beep_eyes/`, `scripts/run_beep_eyes.py`, and `scripts/run_moving_eyes.py`:

1. Keep stationary capture fail-closed by default while allowing an explicit moving-evidence path.
2. During an active motion lease, fetch frames without opening the vendor app-control socket or sending its hidden motor-stop packet.
3. Capture continuously at approximately 3 FPS into a thread-safe bounded ring independently of Gemini inference.
4. Build newest chronological panels of exactly 4, 6, or 9 frames; nine is the default for motion-blur redundancy.
5. Retain blurred frames as uncertainty so useful neighboring panels can still support perception.
6. Record image quality, timestamps, inter-frame difference, and per-frame motion context.
7. Render labeled 2×2, 3×2, or 3×3 sheets and include the latest full-resolution frame.
8. Permit one Gemini request at a time while capture continues; after completion, supersede stale pending evidence with the newest ring window.
9. Validate the typed response, update the persistent world model, and prepare the next semantic decision while locomotion continues.
10. Apply deterministic LiDAR, SLAM, visual-hazard, rollout-mode, and confidence policy gates.
11. Write observation, model, policy, and semantic artifacts under ignored `captures/eyes/` paths.
12. Perform no direct motor dispatch from the moving-eyes process.

The panel gives temporal context and blur tolerance cheaply; the latest frame preserves detail that would otherwise be reduced by layout. Ordinary blur or one failed frame is not a stop condition. LiDAR collision response remains local and much faster than model inference.

## Typed model output

Canonical output contains:

- scene summary and observed changes;
- people and objects with stable packet-local IDs, relative position, object proximity/visible geometry, and confidence;
- visual hazards, including explicit glass/stairs/drop categories;
- one attention target;
- a goal and up to three proposed skills;
- global confidence and uncertainty;
- an escalation flag and reason.

Allowed skills:

- `observe`
- `stop`
- `orient` — left/right, 1–30°
- `advance` / `retreat` — 0.05–0.30 m
- `explore` — 5–30 s
- `gesture` — `pray`, `stretch`, or `swing`
- `speak` — at most 240 characters

Unknown fields and unknown skills fail validation. One contract-repair inference is allowed if Gemini violates a local bound. A second violation fails closed.

People and objects must have unique IDs. A selected person/object target must reference an entity actually reported in the same inference; direction targets are restricted to left, center, or right. Bridge JSON is capped at 512 KiB, each camera/replay JPEG at 2 MiB, and Gemini responses at 2 MiB.

## Policy modes

### `shadow`

- No proposed step is eligible.
- No dispatch exists in the runner.
- Used for perception-quality evaluation and prompt/schema work.

### `stationary`

- Policy may mark `observe`, `stop`, speech, and stationary gesture proposals eligible.
- Movement remains prohibited.
- `BridgeBodyAdapter` implements stationary `observe` and `stop`. Speech and gesture dispatch adapters remain pending.

### `supervised`

- A stationary proposal may begin a bounded motion lease only with fresh complete LiDAR sectors, sufficient clearance, usable SLAM, no conflicting lease, adequate confidence, and no relevant visual hazard.
- During an existing autonomous mission, moving visual evidence updates semantic context and plans ahead; it does not inject raw motor commands into the active controller.
- Policy eligibility is not execution.
- A future course adapter may update semantic intent only at a deterministic safe boundary; ordinary perception continues without stopping the active gait.

## Glass and visual safety

Camera perception augments LiDAR and SLAM; it does not replace them. A confident visual glass/stairs/drop/person/animal/vehicle hazard blocks movement eligibility. Absence of a detected visual hazard is not evidence of safe clearance. LiDAR and SLAM remain mandatory, while humans retain physical stop authority during supervised motion.

## Credentials

The eyes package requires Pillow for contact-sheet and quality analysis. If it is not already available:

```bash
uv venv .venv
uv pip install --python .venv/bin/python -r requirements-eyes.txt
```

Live Gemini mode resolves credentials in this order:

1. `GEMINI_API_KEY` or `GOOGLE_API_KEY` in the process environment;
2. an explicitly selected Hermes profile credential pool.

The API key is sent in the `x-goog-api-key` HTTP header. It is never placed in URLs, artifacts, logs, repository files, or command output.

Example using the protected `sol` profile:

```bash
HERMES_HOME="$HOME/.hermes/profiles/sol" \
PYTHONPATH="$PWD:$HOME/.hermes/hermes-agent" \
python3 scripts/run_beep_eyes.py \
  --base-url http://100.100.141.33:8766 \
  --credential-source hermes \
  --hermes-home "$HOME/.hermes/profiles/sol" \
  --mode shadow
```

Attach continuous nine-frame perception to an already active mission:

```bash
HERMES_HOME="$HOME/.hermes/profiles/sol" \
PYTHONPATH="$PWD:$HOME/.hermes/hermes-agent" \
python3 scripts/run_moving_eyes.py \
  --base-url http://192.168.8.88:8766 \
  --panel 9 --fps 3 --duration 190 \
  --credential-source hermes \
  --hermes-home "$HOME/.hermes/profiles/sol"
```

The moving runner requires an active asynchronous mission unless `--allow-no-mission` is supplied for a bench test. It observes and plans but never owns or dispatches the motors.

Fixture-only development needs no robot and no model credential:

```bash
PYTHONPATH="$PWD" python3 scripts/run_beep_eyes.py \
  --replay-dir captures/eyes/<saved-run> \
  --mock-response tests/fixtures/eyes_response.json \
  --mode shadow
```

A saved run contains `f*.jpg` frames and, when available, its original `observation_packet.json`. Replay never contacts BEEP.

## Continuous development procedure

Development no longer waits for BEEP by default.

### Work continuously offline

- schema and prompt iteration;
- saved-frame/contact-sheet replay;
- provider transport tests;
- scene-tracking and memory logic;
- policy and lease state machines;
- speech/gesture selection;
- mock bridge and deterministic simulation;
- unit, integration, and regression tests;
- documentation and repository maintenance.

### Request BEEP power only for hardware gates

- camera or microphone endpoint changes;
- sensor timing and image-quality baselines;
- live model inference on current surroundings;
- LiDAR/SLAM integration changes;
- speaker-volume or gesture verification;
- staged stationary social interaction;
- supervised bounded physical movement.

At a hardware gate GLADIS should state exactly what must be powered, the expected duration, physical supervision requirements, and the stop method. After collecting reusable fixtures and measurements, development returns offline.

## Rollout plan

1. **Shadow perception** — current.
2. **Saved-frame replay and evaluation corpus** — measure hazards, people, change, and false confidence without BEEP.
3. **Persistent scene tracker** — track packet-local entities and attention across cycles.
4. **Stationary social loop** — supervised speech and one whitelisted gesture while navigation is stopped.
5. **Visual heading proposals** — compare against LiDAR sectors and SLAM, still shadow-only.
6. **Continuous moving temporal panels** — source and mock integration complete; verify 4/6/9 quality and latency on hardware.
7. **Single bounded orient and approach skills** — supervised and fail-closed when a deliberate viewpoint change is required.
8. **Fluid perceive–plan–move loop** — camera and inference continue during local navigation; routine perception does not stop gait.
9. **Longer embodied sessions** — only after measured reliability, cancellation, connectivity, moving-camera, and human-stop tests.

## Proven baseline

The initial real-camera shadow run produced a stable four-panel sequence. Gemini correctly described a centered white box as an obstacle, found no person, proposed only `observe`, passed the strict normalized contract, and caused no dispatch. Later offline integration proved that an active mock mission can fill a moving ring, render all 4/6/9 layouts, continue capture during inference, update the world model, and prepare a semantic next action without issuing any motor POST. These prove the stationary real-camera route and moving software architecture respectively; they do not yet prove moving-image quality on physical gait. Civilization remains provisionally intact.
