# hymeko_transformation

The HyMeKo IR→output (transformation) layer — a standalone cargo workspace.

- **`hymeko_emitter/`** — editor IR (slotmap, `IRDelta`) + M2T emitters (`.hymeko`, SysML v2, Rust stubs, Lean 4)
- **`hymeko_query/`** — pattern-matching query engine over the hypergraph IR
- **`hymeko_formats/`** — built-in format emitters (URDF / SDF / MJCF / DOT / Mermaid / Gazebo world) — a query plugin
- **`hymeko_webxr/`** — WebXR/spatial vocabulary, semantic validation and borrowed queries over the canonical IR.
  `meta_room.hymeko` specialises the spatial vocabulary for XR rooms (room frame, root fiducial via
  `@fiducial_registration`, landmarks, tracked devices); `RoomQuery` binds them by kind, never by path.
  Frame/anchor attributes are effective fields (a kind's value is the instance default). Room models
  in other repositories import copies written by `hymeko_webxr/tools/sync-vocabulary.ps1`.

`hymeko_core` and `parser` are external repos (`hymeko/core`, `hymeko/parser`), pulled as `git` dependencies.

## Build & test

```sh
cargo build --workspace
cargo test  --workspace
```

## Fixtures

- `data/{robotics,minimal_examples,nn,paper,robotics_imported}/` — `.hymeko` fixtures the tests load.
- `transforms/` — the per-format template/query files (`queries.hymeko`, templates) the emitters render from.

## Structural diagrams from HyMeKo

`hymeko_formats::snapshot::emit_dot_graph` renders the canonical IR with values,
scope/inheritance/reference relations, separate arc junctions and signed incidences.
Arrow direction follows explicit `PortDirection`, independently of sign. DOT/SVG
IDs retain declaration, arc and incidence provenance within the compiled document.
The existing kinematic DOT/Mermaid registry transforms remain domain-specific views.

```powershell
npm ci --prefix tools/diagram --ignore-scripts --no-audit --no-fund
cargo run -p hymeko_formats --example diagram --locked --offline -- MODEL.hymeko MODEL.dot
node tools/diagram/render.mjs MODEL.dot MODEL.svg
```

The example delegates compilation to Core and export to the existing format owner;
the development-only Viz.js tool supplies Graphviz layout/SVG rendering. Edit the
HyMeKo source and regenerate the outputs. This view is not physical geometry or
lossless serialization. The full structural view can become dense on large models.

Focused checks: `cargo test -p hymeko_formats --test diagram --locked --offline`
and `node --test tools/diagram/render.test.mjs`.

## Provenance / notes

- Migrated from the `hymeko_framework_rust` monorepo (emitter + query + formats). `core`/`parser` kept external as git deps.
- `test_mdsd_reuse` was gated out — it reaches into the sibling `hymeko_ros2_demo` crate (a demo, outside this layer).

## Polar and hexagonal mazes

The existing `adversarial_grid` example also exports two small maze profiles:
`hex` has axial radius 2 (19 cells, six-neighbor connectivity), while `polar`
has two concentric rings of six sectors with radial and angular passages.
Graph's rank-2 weighted spanning basis selects a seeded passage tree; these are
randomized mazes, not a new adversarial difficulty comparison.

Run explicit generation on **Amaterasu**, with a fresh directory per export:

```bash
cargo run --locked --offline -p hymeko_formats --example adversarial_grid -- --generate-maze hex 0 /tmp/NEW_HEX
cargo run --locked --offline -p hymeko_formats --example adversarial_grid -- --generate-maze polar 0 /tmp/NEW_POLAR
```

Each directory contains canonical `source.hymeko` cell/passage/wall declarations,
`geometry.json`, Query and Graph checks in `validation.json`, and `view.hymeko`
for the existing Kawamirai primitive renderer. Wall boxes carry explicit yaw;
polar walls use three facets per sector. The world-space route includes arc
midpoints and passes Geometry's swept-capsule check with a 0.85 m radius around
the route against all actual wall boxes. This is a geometric route witness,
not robot navigation.

The same export also composes `model.hymeko`, emits `scene.xml` through the
existing MJCF owner, and writes Query-projected initial conditions and contact
settings in `physics.json`. Three free boxes provide a bounded interaction
fixture for the existing MuJoCo backend. Actual dynamics and Kawamirai replay
are separate, explicit Amaterasu steps; generation does not start simulation.

Pure owner checks:
`cargo test --locked --offline -p hymeko_formats --example adversarial_grid`.
The maze tests cover repeatable seeds, different passage systems, canonical
validation, and clearance at every possible profile portal. The existing square
grid CLI and its output format remain supported. Results and reproduction are
in the local [maze report](reports/2026-10-06-generalized-mazes.md), archived under
`\\192.168.1.228\hymeko\reports\by-repository`.

To include the existing Cyberdisc robot, append its actor path to the same CLI:
`--generate-maze hex 0 /tmp/NEW_ROBOT_MAZE ../robotics/examples/cyberdisc/actor.hymeko`.
This additionally exports `robot.xml` and `robot.json` through the same Core,
Query, Geometry and MJCF owners. It retains all eleven declared visual parts and
checks the actor footprint against the validated corridor. The existing Simulator
`python/maze_contact.py ... --robot` runs the bounded six-second wheel-drive and
box-pushing exercise on Amaterasu; `--no-drive` supplies its zero-command control.
See [the simulator usage](../../hymeko_simulator/docs/maze-contacts.md).

## Kawakern physics engine

The generated adversarial grid can import the reusable Cyberdisc actor and
export a real Kawakern scene through the existing Formats example:

```bash
cargo run -p hymeko_formats --example adversarial_grid --locked --offline -- \
  --export-physics GRID.hymeko ../robotics/examples/cyberdisc/actor.hymeko NEW_DIRECTORY
```

The directory must not exist. The export validates the grid through Query,
retains the Graph route witness, and maps cells to a centred 2 m grid. Geometry
verifies actor footprint (including collider offsets) and all 64 fixed box
colliders. Outputs are `model.hymeko`, `scene.hymeko` and `composition.json`.
This is data composition; Simulator executes physics on Amaterasu and Kawamirai
renders its actual state. See the local
[integration report](reports/2026-09-29-adversarial-cyberdisc.md), archived under
`\\192.168.1.228\hymeko\reports\by-repository`.

**Kawakern** names the existing HyMeKo physical engine family used by Kawamirai.
`hymeko_geometry::engine::Engine` owns articulated rigid dynamics and ground contact;
`hymeko_dynamics` projects physical systems, including linear elasticity, into the
existing Tensor numerical kernels. Graph owns signed incidence and octree structure.
This name does not rename crates, add dependencies, or imply full MuJoCo parity.

The [local stack integration](docs/stack-integration.md) records the shared
Geometry owner used by Simulator, Kawamirai, Quest and the XRobotAssist bridge.

Geometry's kinematic field reader accepts inherited physical properties and
material colors on canonical IR. Local declarations take precedence, followed
by bases in authored depth-first order; repeated/cyclic bases terminate. Invalid
local values do not silently fall back to inherited values. Use untyped property
mixins for reusable defaults: a template inheriting `link` is itself a physical
link. Inherited references retain their canonical target; they do not retarget
to same-named children of a derived instance. Core/Parser semantics are unchanged.

### Host robot visual import ? 2026-09-24

`hymeko_formats/tools/robot-mesh.py` exports explicitly selected URDF/COLLADA visual
links as validated-client Geometry TriangleMesh JSON assets. NumPy is a host-tool
requirement; no Rust runtime dependency was added. Use `--urdf`, `--frames`,
`--visual-links`, `--output`, `--manifest`, and `--package-root`. It bakes fixed
joint/visual/mesh transforms in metres, preserves normals and reflected winding,
and rejects unsupported formats instead of guessing. HIVE transports the output;
Kawamirai owns its decoded presentation cache. `test_robot_mesh.py` has three
correctness fixtures. COLLADA material bindings split visual geometry by diffuse
RGBA; URDF material colors override them where declared. Each manifest part has
its own immutable asset ID, RGBA and unchanged observed-link frame. Textures are
explicitly unsupported. Limits remain 1M vertices / 6M indices per source geometry,
with at most 32 material groups. The UR5 export has 20 parts and 95,694 triangles.
COLLADA matrix interpretation follows the Khronos 1.5.1 errata (XML rows, column
vectors): https://www.khronos.org/files/collada_1_5_1_release_notes.pdf .
URDF visual-origin semantics: https://docs.ros.org/en/kilted/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html .
