# Embodied Middle Layer

## Purpose

GLADIS is not a remote-control script waiting for Max to provide every action. The middle layer turns perception into persistent situated agency:

```text
temporal perception
→ persistent world identity
→ intrinsic drives and personality
→ self-selected embodied goal
→ one typed semantic skill
→ deterministic safety/lease policy
→ body-specific skill adapter
→ bounded physical effect under continuous local and temporal perception
→ continue, adjust, deliberately inspect, or stop for a real boundary
```

Max provides advice, constraints, collaboration, and human authority at genuine physical boundaries. He is not required to choreograph ordinary perception, investigation, interaction, or development.

The architecture is body-agnostic. BEEP is the current quadruped body; the same world, drive, goal, and skill contracts should remain useful with humanoid or other robotic bodies. Only body-specific adapters change.

## Persistent world model

`beep_agent/world_model.py` converts packet-local Gemini IDs into stable local tracks such as `object-0001`.

It maintains:

- people and objects across perception packets;
- conservative alias/description matching;
- confidence, relative position, proximity, visible geometry, engagement, and observation counts;
- current hazards and attention;
- duplicate-packet rejection and bounded track expiry.

Ambiguous observations become separate tracks rather than being silently merged. The world model is local deterministic state, not model conversation memory.

## Personality and drives

`beep_agent/drives.py` currently represents:

- curiosity;
- social responsiveness;
- marking drive;
- caution;
- GLADIS's precise, dry, observant expression style.

The goal arbiter chooses among:

- acquire current perception when the world is unknown;
- acknowledge a person who is actively engaging;
- investigate a confident novel object;
- investigate and potentially mark an object when geometry becomes suitable;
- seek novel safe space when nothing else deserves attention.

Safety is not outsourced to personality. Drives select intent; deterministic policy decides whether the body may act.

## Callable semantic skills

`beep_agent/skills.py` defines strict function schemas for:

- `observe`
- `orient`
- `approach_target`
- `inspect_target`
- `mark_target`
- `explore`
- `speak`
- `gesture`
- `stop`

No function accepts raw velocity, motor registers, arbitrary vendor action IDs, URLs, or shell commands.

Important compound intents:

- `approach_target`: one 0.05–0.20 m guarded segment with fresh target and local safety evidence; deliberate close inspection may stop, ordinary exploration does not;
- `inspect_target`: acquire a bounded alternate viewpoint to reveal actual object geometry;
- `mark_target`: request deterministic approach, sideways alignment, and marking only at a visually supported outward corner;
- `explore`: a bounded 5–600 second asynchronous mission (90 seconds by executive default) with continuous local safety, mission telemetry, and moving temporal perception.

## Executive behaviour

`beep_agent/executive.py` chooses one next semantic skill from a selected goal and persistent world state.

For an investigate-and-mark goal:

```text
target absent             → observe/search
target left or right       → bounded orient
target far or mid          → bounded approach
target distance unknown    → observe for proximity
target near, no corner     → alternate inspection viewpoint
target near, corner seen   → mark_target intent
```

A marking request in shadow/stationary rollout is always `dry_run=true`. Even a supervised semantic request is not execution: it still requires the future body adapter, fresh sensors, exclusive lease, final target validation, and human physical stop authority during initial trials.

Social goals produce stationary speech. Living hazards in the intended path cause observation/yielding rather than movement.

## Latest physical checkpoint

The last verified embodied hardware increment used the synchronous guarded bridge to execute `explore(duration_s=5)`. BEEP moved for 5.38 seconds, used six forward strides and two right turns, released the exclusive lease, and finished stopped. Post-action perception then reported a near flat cabinet/wall face, no supported outward corner, and 0.465 m front clearance. Curiosity selected a bounded left-view `inspect_target`, which was intentionally not dispatched because that trial allowed one physical skill only.

The newer asynchronous mission worker, `BridgeBodyAdapter`, and continuous moving-eyes supervisor are source-complete and mock/integration tested but not yet deployed to the powered robot. The next physical checkpoint must therefore validate deployment, moving camera capture, cancellation, and final stopped/no-lease state before longer autonomy claims.

## Body-adapter state

`BridgeBodyAdapter` now implements three deterministic mappings:

- `explore` → asynchronous `POST /mission/start` in guarded coverage mode;
- `stop` → exact mission cancellation and repeated onboard stop;
- `observe` → stationary bridge observation.

Autonomous `explore` defaults to a 90-second fluent mission. Starting it returns immediately, so the middle layer can reason from persistent world state, `/mission` telemetry, and continuous moving-camera windows while BEEP moves. A background ring captures at approximately 3 FPS and supplies chronological 4/6/9-frame panels (default nine) to one Gemini inference at a time; newer evidence supersedes stale pending windows. The onboard controller still owns approximately 80 ms obstacle supervision, fluent gait, lease enforcement, and final stop. Gemini supplies semantics and planning context, not delayed motor reflexes.

`ContinuousMovingCapture` and `run_moving_eyes.py` additionally provide the live perception supervisor. It does not dispatch a semantic skill; it continuously merges moving visual evidence into the world model and prepares the next decision while the current mission remains under deterministic bridge authority.

Remaining adapters are:

1. `speak` → stationary TTS through BEEP's analog speaker;
2. `gesture` → stopped-state whitelisted bridge action;
3. `orient` → measured guarded heading change;
4. `approach_target` → one short lease using fresh LiDAR/SLAM and current visual target;
5. `inspect_target` → bounded viewpoint acquisition and geometry re-perception;
6. `mark_target` → bind the stable target/corner to the existing deterministic approach-sideways-mark routine;
7. semantic course updates → apply a prepared intent at a deterministic navigation boundary without routine gait stops.

The next hardware activation is moving-camera validation during a short supervised mission, followed by stationary speech/gesture, supervised semantic course adjustment, approach, inspection geometry, and only then marking. Agency without staged embodiment is merely a more articulate accident.
