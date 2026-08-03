# BEEP Eyes: Temporal Perception Architecture

## Objective

Give GLADIS a closed perception loop through BEEP's front camera:

```text
capture temporal evidence
→ interpret scene and change
→ select an attention target
→ propose bounded heading, movement, gesture, or speech skills
→ apply deterministic policy and bridge safety gates
→ act under an exclusive short lease
→ stop and perceive again
```

The model proposes semantic skills. It never emits motor registers, velocities, arbitrary action IDs, or direct bridge commands. The deterministic bridge remains the sole motor and safety authority.

## Current milestone: temporal shadow eyes

Implemented in `beep_eyes/` and `scripts/run_beep_eyes.py`:

1. Refuse capture while BEEP reports movement or an active lease.
2. Discard one vendor-camera warm-up frame.
3. Retry transient camera startup failures.
4. Capture four strictly chronological JPEG frames, normally 1.5 seconds apart, with bounded response sizes.
5. Record dimensions, brightness, contrast, edge variation, RGB means, and inter-frame difference.
6. Render a labeled 2×2 contact sheet.
7. Send the contact sheet and latest full-resolution frame to Gemini 3.5 Flash-Lite.
8. Request a compact typed wire response and normalize it into the stricter local schema.
9. Validate scene entities, hazards, attention, confidence, uncertainty, and at most three bounded skill proposals.
10. Evaluate the proposals against deterministic status, LiDAR, SLAM, visual-hazard, rollout-mode, and confidence gates.
11. Write observation, model, and policy artifacts under ignored `captures/eyes/` paths.
12. Perform no action dispatch.

The contact sheet gives temporal context cheaply; the latest frame preserves detail that would otherwise be reduced by panel layout. Minimal model thinking is preferred for low-latency scene routing.

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
- Dispatch is still absent until a separate stationary executor and supervised test exist.

### `supervised`

- Policy may mark bounded movement eligible only with fresh complete LiDAR sectors, sufficient clearance, usable SLAM, no active lease, adequate model confidence, and no relevant visual hazard.
- Policy eligibility is not execution.
- A future executor must acquire one immutable bridge lease, revalidate immediately, run one skill, stop, and re-perceive.

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
6. **Single bounded orient skill** — supervised lease, stop, re-perceive.
7. **Single bounded approach/retreat skill** — supervised and fail-closed.
8. **Repeated perceive–act loop** — one skill per cycle, immediate stop and revalidation.
9. **Longer embodied sessions** — only after measured reliability, cancellation, connectivity, and human-stop tests.

## Proven baseline

The initial real-camera shadow run produced a stable four-panel sequence. Gemini correctly described a centered white box as an obstacle, found no person, proposed only `observe`, passed the strict normalized contract, and caused no dispatch. This proves the end-to-end camera → multipanel Gemini → typed response → policy path, not autonomous movement readiness. Civilization remains provisionally intact.
