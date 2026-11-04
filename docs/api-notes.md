# API notes

Contracts of the transformation crates, by topic. Commands run from the
repository root; every output directory must be new.

## Requirements, components and verification records

`hymeko_formats::traceability::TraceabilityExport` validates a bounded trace
profile through `hymeko_query::traceability`, then uses the registered
requirements SysML, DOT and CSV templates. It retains named parallel
relationships and distinguishes allocation, satisfaction, verification links
and authored outcomes (-1 unknown, 0 fail, 1 pass). These outcomes are
evidence records, not physical test executions.

The facade requires one flat scope, unique ASCII identifiers, unweighted
binary relations with checked endpoint roles, and delimiter-safe requirement
text. Unsupported input is rejected before export. Generic template rendering
alone does not provide this domain validation. SysML extraction checks the
emitted subset; native SysML tool conformance is not claimed. Derivation is
represented as a named connection, not a native `deriveReqt` assertion.

```sh
cargo run --locked -p hymeko_formats --example traceability -- data/paper/traceability_verification.hymeko verification_cell trace-output
```

For the complete snapshot and negative-input evidence, point
`HYMEKO_TRACE_EVIDENCE` at a new directory and cross-check it:

```powershell
$env:HYMEKO_TRACE_EVIDENCE = Join-Path (Get-Location) 'trace-evidence/outputs'
cargo test --locked -p hymeko_formats --test traceability
python tools/traceability/check.py trace-evidence/outputs --result trace-evidence/results.json
```

The checker uses three format-specific readers, the Core/Query projection and
a hand-authored full reference. It reports unmet requirements as diagnostic
results; agreement between views does not make the modeled system compliant.

### Simulation evidence for the trace example

`traceability_simulation.hymeko` imports the FANUC model and trace witness,
adds a fixed vacuum cup, and binds six observation rules to the actual test
identities. `SimulationBundle` resets the synthetic outcomes to Unknown, emits
the robot through `RobotProjection`, and produces Geometry pose references and
the declared measurement contract. The simulator that runs the bundle is a
separate consumer; no RL environment or new physics solver is involved.

```sh
cargo run --locked -p hymeko_formats --example trace_simulation -- prepare data/paper/traceability_simulation.hymeko experiment NEW_BUNDLE
# run the bundle in a simulator, which writes RUN/evidence.json, then:
cargo run --locked -p hymeko_formats --example trace_simulation -- record data/paper/traceability_simulation.hymeko experiment ABSOLUTE_RUN/evidence.json NEW_RECORDED_VIEWS
```

`record` writes the observations through Core's checked value updates. The
simulated model uses an **ideal vacuum weld**, not a seal or frictional
gripper model. The native integration timestep is declared in the source
(0.5 ms). Missing evidence remains Unknown; censored observations retain
explicit bounds. Unmet requirements remain failed outcomes in SysML, DOT and
CSV. Repeatability is deterministic simulation replay, not hardware
repeatability, and the full path error is reported separately from station
positioning accuracy.

## Mechanism bindings

`hymeko_mechanism::kinematics::assembly::KinematicAssembly` binds one authored
group by canonical `DeclId`. It reuses the existing `KinematicModel`, `FkPlan`,
`VisualSet` and collision projection. A robot arm, hinged door and linear
slider use the same API; local names may repeat across groups. Namespace
ancestry selects membership; signed joint incidences define topology. Outside
parent frames are allowed as anchors, while references to another group's body
are refused.

Bind once with `KinematicAssembly::read(&query, group_id)`. Supply observations
through `set_state(coordinates, root_se3)` or `apply_values(updates)`, then
borrow `visual_poses()`, `collider_placements()` and
`attachment_pose(frame_id)`. All derive from the same accepted link frames. A
failed sample leaves the previous accepted state intact; repeated checkpoint
coordinates retain root placement. Position limits do not clamp measured
observations. The host admits the source/model basis and recreates bindings
after structural changes; numeric updates do not edit IR.

Optional joint `coordinate -> leaf` fields bind scalar radians or metres; use
`joint_index(DeclId)` for producer vector bindings. Unbound coordinates start
at zero, including fixed joints. A group's `attachment_kind` names an explicit
edge kind such as `kinematics.elements.frame_attachment`; instances bind
`(+ body, - frame)` with local `origin` in metres and `rpy_deg`. These are
oriented tool or sensor reference frames, distinct from force application
points and physical attachment constraints. Visual and physical geometry
remain separate projections.

`QueryEngine::scoped(root)` is the underlying borrowed, root-inclusive
selection. Predicates retain their full IR context, so shared imported
vocabulary and axes remain resolvable. The global model extractor keeps its
behaviour; use `KinematicModel::from_scope` for isolated mechanisms.

The [three-mechanism fixture](../hymeko_mechanism/tests/fixtures/assembly/room.hymeko)
tests common placement, numeric updates, visual and collider offsets, attached
tool frames, canonical identity isolation and refused inputs:

```sh
cargo test --locked -p hymeko_mechanism --test assembly
cargo test --locked -p hymeko_query --lib engine::scope::tests::
```

Geometry's kinematic field reader accepts inherited physical properties and
material colours on canonical IR. Local declarations take precedence, followed
by bases in authored depth-first order; repeated or cyclic bases terminate.
Invalid local values do not silently fall back to inherited values. Use
untyped property mixins for reusable defaults: a template inheriting `link` is
itself a physical link. Inherited references retain their canonical target;
they do not retarget to same-named children of a derived instance.

Physics over a mechanism read here is kawakern's:
`kawakern::Engine<IrHandles>` owns articulated rigid dynamics and ground
contact, and `kawakern_dynamics` projects physical systems into Tensor's
numerical kernels.

## Robot formats

`generate_description`, the registered transforms and `TransformEngine` use
the same model-based URDF, SDF and MJCF emitters. `RobotProjection::mjcf()`
uses that same MJCF path. Explicit `render_from_templates` for these three
robot formats returns `None`; their static templates are not registered
backends. Diagram and traceability templates keep their Query-based rendering
and explicit errors. URDF and SDF preserve non-cardinal axis components. SDF
retains link, joint, visual and collision frames, including the authored
world-root offset in the model frame.

```sh
cargo run --locked -p hymeko_formats --bin robot_projection -- \
  data/robotics/sim/dual_fanuc/fanuc_lrmate200id.hymeko robot NEW_DIRECTORY
```

It writes `robot.xml`, `robot_model.json` and `robot_fk.json`. FK timing
requires `--benchmark-fk`; ordinary projection does not measure performance.
Existing files are never replaced. The extractor reads the document's
kinematic declarations and the supplied name labels the export, so provide a
single-robot document.

Library and command-line regressions:
`cargo test --locked -p hymeko_formats --lib --bins --test research_api`.

## Structural diagrams

`hymeko_formats::snapshot::emit_dot_graph` renders the canonical IR with
values, scope, inheritance and reference relations, separate arc junctions and
signed incidences. Arrow direction follows explicit `PortDirection`,
independently of sign. DOT/SVG IDs retain declaration, arc and incidence
provenance within the compiled document. The kinematic DOT and Mermaid
transforms remain domain-specific views.

```sh
npm ci --prefix tools/diagram --ignore-scripts --no-audit --no-fund
cargo run --locked -p hymeko_formats --example diagram -- MODEL.hymeko MODEL.dot
node tools/diagram/render.mjs MODEL.dot MODEL.svg
```

The example delegates compilation to Core and export to the format owner; the
development-only Viz.js tool supplies Graphviz layout and SVG rendering. This
view is not physical geometry or lossless serialization, and the full
structural view becomes dense on large models.

Focused checks: `cargo test --locked -p hymeko_formats --test diagram` and
`node --test tools/diagram/render.test.mjs`.

## Relational projection

Identity across revisions is the canonical path; ids are the IR's and hold
within one projection only. Every reference is a deferrable foreign key to
`declaration`, so a projection loads in one transaction.

```sh
cargo run --locked -p hymeko_relational --example project -- MODEL.hymeko SCHEMA > model.sql
psql -v ON_ERROR_STOP=1 -f model.sql      # into an empty (or dropped) SCHEMA
```

Focused checks: `cargo test --locked -p hymeko_relational`. A projection has
been checked against PostgreSQL 17 by loading the script and querying kind
tables with inherited values, `effective_field` providers, and category trees
by a recursive query over `base`.

## Generated environments

Research consumers use `hymeko_formats::environment`, `metaheuristic` and
`robot_projection`. These are domain projections over Core's retained
compilation: Query owns semantic assertions, Graph owns search and spanning
bases, Geometry owns kinematics and collision, and Emitter owns canonical
source output. Campaign policy and command-line parsing live in
`src/bin/adversarial_grid.rs`.

### Polar and hexagonal mazes

`hex` has axial radius 2 (19 cells, six-neighbour connectivity); `polar` has
two concentric rings of six sectors with radial and angular passages. Graph's
rank-2 weighted spanning basis selects a seeded passage tree.

```sh
cargo run --locked -p hymeko_formats --bin adversarial_grid -- --generate-maze hex 0 NEW_HEX
cargo run --locked -p hymeko_formats --bin adversarial_grid -- --generate-maze polar 2 NEW_POLAR --loops 3
```

Each directory contains canonical `source.hymeko` cell, passage and wall
declarations, `geometry.json`, Query and Graph checks in `validation.json`, and
`view.hymeko` for a primitive renderer. Wall boxes carry explicit yaw; polar
walls use three facets per sector. The world-space route includes arc
midpoints and passes Geometry's swept-capsule check with a 0.85 m radius
against all wall boxes. This is a geometric route witness, not robot
navigation.

`--loops` adds a bounded number of extra passages while keeping the seeded
spanning tree: 0..24 for hex, 0..7 for polar. The export also composes
`model.hymeko`, emits `scene.xml` through the MJCF owner, and writes
Query-projected initial conditions and contact settings in `physics.json`.
Three free boxes give a bounded interaction fixture for a MuJoCo backend.
Generation does not start a simulation.

Appending an actor path (a robot `.hymeko`) additionally exports `robot.xml`
and `robot.json` through the same Core, Query, Geometry and MJCF owners and
checks the actor footprint against the validated corridor.

For search-driven generation, append `--weights GENES.json`: a JSON array of
finite values in `[0,1]`, one per candidate passage (42 for hex, 18 for
polar). `validation.json` supplies the lexicographically ordered
`priority_pairs` and the encoding contract. Each priority becomes an exact
53-bit integer edge weight; Graph's minimum-weight spanning basis prefers
lower weights, with its deterministic arc-order tie break. `genome.json`
retains the vector. Core compilation, Query checks, connectivity and
swept-route clearance remain mandatory.

### Search over mazes

```sh
cargo run --release --locked -p hymeko_formats --bin adversarial_grid -- \
  --benchmark-mazes NEW_RESULTS 10 16 30
```

The benchmark compares Graph BFS, Dijkstra and A* on the same endpoints and
unit passage costs, over tree, three-loop and fully open hex and polar maps.
It checks equal optimal costs, path incidences, deterministic replay and swept
clearance against the walls. A* uses Euclidean distance divided by the largest
passage chord, a consistent lower bound for hop cost.

`environment::maze::search::MazeNavigation` borrows a retained
`CompiledWorld`; `evaluate(start, goal, 0)` checks correctness without reading
a clock. Positive repetitions measure warm searches with rotated ordering and
separate workspaces. Compilation, index construction, witness checks and
plotting are outside search timings. BFS has no expansion counters, so its
counter field is null. Caps: 30 seeds, 64 distinct ordered pairs per map,
1000 repeats and one million timed calls. The fully open topology is identical
across seeds; different endpoint samples are not independent map
realizations. Figures need Python with matplotlib and Pillow
(`hymeko_formats/tools/plot_maze_search.py`); `completed.json` is written only after plotting
succeeds.

Focused regression: `cargo test --locked -p hymeko_formats --test maze_search`.

### Legged terrain

`environment::locomotion` binds a declared locomotion profile (robot source and
root, base joint and link, feet, controller id, stance height, nominal speed,
step length, swing height, step-up, slope, friction, lateral margin) to the
robot compiled through Core and projected by `RobotProjection`. It refuses
unbounded joint defaults, missing inertia tensors and declarations the
measured body cannot satisfy, and derives `TerrainBounds` (step-up, slope,
friction, hurdle, foothold, passage width, roughness, speed).

`environment::terrain` adds a robot-scaled `square` profile and robot-scaled
hex and polar topologies, then places up to three features on route segments:
ramp, steps, rough tiles, stepping-stone footholds, low-friction patch, narrow
posts, step-over hurdle and a kinematic moving obstacle. Magnitudes are
relative to the bounds; gene intensity maps to [0, 1.25] of a bound.
Verification levels stay separate: topology (Graph route, Query counts),
geometry (feature bounds, placement, swept wall clearance) and temporal (route
at the declared speed within 80 % of the horizon); dynamics is the
simulator's. Categories: `valid`, `invalid_world`, `infeasible_task`.

```sh
adversarial_grid --project-legged PROFILE.hymeko NEW_DIR [--horizon S]
adversarial_grid --generate-terrain square|hex|polar SEED NEW_DIR PROFILE.hymeko [--weights GENES.json] [--horizon S] [--task reach|coverage]
```

Outputs: canonical `source.hymeko`, `validation.json`, `model.hymeko` and
`scene.xml` (surface roles, friction and mocap motion queried back into
`physics.json`), `terrain.json` (route, footholds, features, task) and
`robot.xml`/`robot.json`. World frame Z up, floor top at z = 0, metres,
seconds and radians in JSON; HyMeKo mounts keep degrees. The coverage task
walks the maze tree depth-first through every cell.

Owner tests: `cargo test --locked -p hymeko_formats --lib environment::`.

### Physics scene export

```sh
cargo run --locked -p hymeko_formats --bin adversarial_grid -- \
  --export-physics GRID.hymeko ACTOR.hymeko NEW_DIRECTORY
```

The export validates the grid through Query, retains the Graph route witness,
and maps cells to a centred 2 m grid. Geometry verifies the actor footprint
(including collider offsets) and the fixed box colliders. Outputs are
`model.hymeko`, `scene.hymeko` and `composition.json`. This is data
composition; a simulator executes the physics.

## Metaheuristic contracts

The `hymeko_formats::metaheuristic` library validates three fixed profiles:
DE/rand/1/bin with clamping and deferred generations, a GA with tournament-2
selection, uniform/reset variation and stable mu+lambda survival, and a
bounded sequential HHO. Core owns compilation and canonical identity; Query
checks exact profile fields, operator rules and typed read/write
dependencies.

```sh
cargo test --locked -p hymeko_formats --lib metaheuristic::
cargo run --locked -p hymeko_formats --bin metaheuristic_contract -- \
  hymeko_formats/data/metaheuristics/ga_tournament_uniform_reset.hymeko NEW_GA_CONTRACT
```

The output contains canonical HyMeKo, `operator_ids.json`, common textual
semantics and a DOT graph. The GA profile has eight operators and 22 state
slots. `hho_sequential_budget.hymeko` is the static contract of an adapted HHO
reference: 11 operators, 26 states, charged repeated parent evaluations,
scan-only leader updates and two declared energy clocks. These are
fixed-profile structural contracts, not an arbitrary expression interpreter
or an equivalence proof.

The [catalog](../hymeko_formats/data/metaheuristics/catalog/README.md)
describes ten methods (DE, GA, PSO, HHO, GWO, WOA, BA, FA, CS, ABC) as typed
operator and state descriptions; `metaheuristic_contract --structural` exports
their name-free support and mechanism projections. This is a closed-corpus
structural comparison, not ten optimizer implementations or a performance
ranking.

## Mesh and material paths

Meshes, materials and recipe execution are geometry's `hymeko_mesh`;
`hymeko_mechanism` re-exports `hymeko_mesh::{mesh, material}`, so
`hymeko_mechanism::mesh` and `hymeko_mechanism::material` name the same types.
Their algorithms (regularisation, normal smoothing, boundary bridging, convex
union, subdivision limits, spline surfaces, skin and eye material profiles)
are tested in the geometry repository.

The `surface_prepare` example is this repository's adapter over them:

```sh
cargo build --locked -p hymeko_mechanism --no-default-features --features authoring,mesh-optimization --release --example surface_prepare
```

`surface_prepare quads|normals|follow|bridge|union|limit|fair INPUT.json OUTPUT.json`
takes a `TriangleMesh` plus the operation's options and writes the result; no
geometry algorithm lives in the adapter. `mesh-optimization` builds
meshoptimizer (C++) through `cc`.

`hymeko_formats/tools/robot-mesh.py` exports explicitly selected URDF/COLLADA
visual links as `TriangleMesh` JSON assets (NumPy, no Rust runtime
dependency). It bakes fixed joint, visual and mesh transforms in metres,
preserves normals and reflected winding, splits geometry by COLLADA diffuse
colour (URDF material colours override), and rejects unsupported formats
instead of guessing. Each manifest part has its own asset ID, RGBA and link
frame; textures are unsupported. Limits: 1M vertices and 6M indices per source
geometry, at most 32 material groups. COLLADA matrices follow the
[Khronos 1.5.1 errata](https://www.khronos.org/files/collada_1_5_1_release_notes.pdf)
(XML rows, column vectors).
