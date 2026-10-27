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

- **`hymeko_relational/`** — the relational reading of a compiled model: the canonical IR as tables
  (`declaration`, own `field`s, `effective_field`s with the declaration that provides each, `base`s with
  `<isa>` told apart from an instance's type, `arc`s with their signs) and a `kind_<name>` table per kind —
  a row per instance at any depth, a typed column (text, number, reference, JSON) per field it carries.
  `postgres` renders the PostgreSQL that creates and fills them. Bases, is-a and effective fields are
  `hymeko_query`'s (`effective_fields`, `isa_tag`), read and never re-derived.

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

## Relational projection

Identity across revisions is the canonical path; ids are the IR's and hold within one projection only.
Every reference is a deferrable foreign key to `declaration`, so a projection loads in one transaction.

```sh
cargo run -p hymeko_relational --example project --locked --offline -- MODEL.hymeko SCHEMA > model.sql
psql -v ON_ERROR_STOP=1 -f model.sql      # into an empty (or dropped) SCHEMA
```

Focused checks: `cargo test -p hymeko_relational --locked --offline`. A projection is checked against a real
PostgreSQL 17 by loading the script (a throwaway `postgres:17` container) and querying it: kind tables with
inherited values, `effective_field` providers, category trees by a recursive query over `base`.

## Provenance / notes

- Migrated from the `hymeko_framework_rust` monorepo (emitter + query + formats). `core`/`parser` kept external as git deps.
- `test_mdsd_reuse` was gated out — it reaches into the sibling `hymeko_ros2_demo` crate (a demo, outside this layer).

## Library ownership and command-line clients

Maze generation now accepts a bounded number of extra passages (`--loops`),
preserving the seeded spanning tree. Hex maps permit 0..24 loops (19 cells),
polar maps 0..7 (12 cells). Omitting the flag preserves tree generation.
On Amaterasu, using new output directories:

```sh
cargo run --release --locked --offline -p hymeko_formats --bin adversarial_grid -- \
  --generate-maze polar 2 results/polar-looped --loops 3
cargo run --release --locked --offline -p hymeko_formats --bin adversarial_grid -- \
  --benchmark-mazes results/maze-search 10 16 30
```

The benchmark compares existing Graph BFS, Dijkstra and A* on the same endpoints
and unit passage costs, over tree / three-loop / fully open hex and polar maps.
It checks equal optimal costs, path incidences, deterministic replay and swept
clearance against the actual walls. A* uses Euclidean distance divided by the
largest passage chord, a consistent lower bound for hop cost. Metric distance,
dynamic obstacles and physical robot completion are separate objectives.

`environment::maze::search::MazeNavigation` borrows a retained `CompiledWorld`;
`evaluate(start, goal, 0)` checks correctness without reading a clock. Positive
repetitions measure warm searches with rotated ordering and separate workspaces.
Compilation, index construction, witness checks and plotting are outside search
timings. BFS's public API has no expansion counters, so its counter field is null.
CLI caps are 30 seeds, 64 distinct ordered pairs per map, 1000 repeats and one
million total timed calls. The fully open topology is identical across seeds;
different endpoint samples are not independent map realizations.

Outputs include source/view HyMeKo documents, geometry and Query receipts,
`results.json` with all paths/counters/raw timing samples, `summary.json`,
`comparison.png/svg`, `maps.png/svg`, and `paths.gif`. The GIF animates a checked
path witness, not a simulated robot. The existing host matplotlib/Pillow packages
produce figures; `completed.json` is written only after plotting succeeds.
The binary expects its source-tree `tools/plot_maze_search.py` to remain present.

Kubota mapping integration is pending identification of the intended paper/code.
These generators are **not a GNG implementation**. A relevant Kubota-coauthored
method is [Fast MS-DBL-GNG (Siow et al., 2024)](https://doi.org/10.3390/biomimetics9090560),
which extracts topological features from observed point clouds. Mapping and world
generation are distinct operations: the proposed integration is observation ->
topological map -> constrained variation -> geometry -> remapping and validation.
It needs explicit free-space/passability semantics: a point-cloud surface graph
does not itself certify a robot navigation graph. The exact variant, data source
and round-trip criteria must be settled before claiming a reproduction.

Focused regression: `cargo test --locked --offline -p hymeko_formats --test maze_search`.

Research consumers use `hymeko_formats::environment`, `metaheuristic` and
`robot_projection`. These are domain projections over Core's retained compilation;
Query owns semantic assertions, Graph owns search/spanning bases, Geometry owns
kinematics/collision and Emitter owns canonical source output. Campaign policy,
command-line parsing and opt-in checks live in `src/bin/adversarial_grid.rs`.
No research executable or reusable domain implementation is loaded from `examples`.

The three commands are now `cargo run -p hymeko_formats --bin adversarial_grid`,
`--bin metaheuristic_contract`, and `--bin robot_projection`. The former
`mujoco_crosscheck` example is replaced by the last command:

```sh
cargo run --locked --offline -p hymeko_formats --bin robot_projection -- \
  data/robotics/sim/dual_fanuc/fanuc_lrmate200id.hymeko robot NEW_DIRECTORY
```

It writes `robot.xml`, `robot_model.json` and `robot_fk.json`, retaining the previous
artifact schemas. FK timing requires `--benchmark-fk`; ordinary projection does
not measure performance. Existing files are never replaced. Geometry's current
extractor reads the document's kinematic declarations; the supplied name labels
the export, so provide a single-robot document.

`generate_description(..., OutputFormat::Mjcf)`, the registered typed transform and
`RobotProjection::mjcf()` now use the same articulated MJCF emitter. The incomplete
flat-body MJCF template has been removed. `TransformRegistry::render` selects the
registered backend; explicit `render_from_templates("mjcf", ...)` now returns
`None`. Template errors in other formats remain errors, without a backend fallback.

Library/CLI regressions: `cargo test --locked --offline -p hymeko_formats --lib
--bins --test research_api`. Physics and campaign execution remain explicit
Amaterasu operations. Previously archived run commands and binaries are historical
provenance and are not rewritten by this migration.

## Ten-method structural identification

The [source-cited corpus and usage](hymeko_formats/data/metaheuristics/catalog/README.md)
compares DE, GA, PSO, HHO, GWO, WOA, BA, FA, CS and ABC through typed HyMeKo
operator/state descriptions. The existing `metaheuristic_contract --structural`
mode exports name-free support and mechanism projections using Core/Query.
The Hystl identification test reports exact, ambiguous and unknown signatures,
plus similarity matrices and explicit arithmetic counterexamples. This is a
closed-corpus structural test, not ten optimizer implementations or a performance
ranking. The exact DE/GA contracts below remain unchanged.

## Bounded HHO execution contract

`hho_sequential_budget.hymeko` adds the exact static contract for the explicitly
adapted HHO research reference: 11 operators, 26 states, charged repeated parent
evaluations, scan-only leader updates and two declared energy clocks. Export it
through the same `metaheuristic_contract` binary used by DE/GA. The descriptor,
[execution choices and paired numerical pilot](../hystl/research/metaheuristics-pilot/HHO.md)
keep the authored profile, executable fixture and measured evidence distinct.

## Metaheuristic dependency contracts

The `hymeko_formats::metaheuristic` library validates the bounded HHO profile above
and these two existing profiles:
DE/rand/1/bin with clamping and deferred generations, and the research GA with
tournament-2 selection, uniform/reset variation and stable mu+lambda survival.
Core owns compilation and canonical identity; Query checks exact profile fields,
operator rules and typed read/write dependencies. The same validator serves all three
profiles, and the previous DE export is unchanged.

```sh
cargo test --locked --offline -p hymeko_formats --lib metaheuristic::
cargo run --locked --offline -p hymeko_formats --bin metaheuristic_contract -- \
  hymeko_formats/data/metaheuristics/ga_tournament_uniform_reset.hymeko /path/to/NEW_GA_CONTRACT
```

The output contains canonical HyMeKo, `operator_ids.json`, common textual
semantics and a DOT graph. The selected GA profile has eight operators and
22 state slots. These are fixed-profile structural contracts, not an arbitrary
expression interpreter or an equivalence proof. See the
[shared mathematical definitions and GA trace checker](../hystl/research/metaheuristics-pilot/DEFINITIONS.md)
and [validation report](reports/2026-10-06-metaheuristic-definitions.md).

## Polar and hexagonal mazes

The `hymeko_formats::environment` library exports two small maze profiles:
`hex` has axial radius 2 (19 cells, six-neighbor connectivity), while `polar`
has two concentric rings of six sectors with radial and angular passages.
Graph's rank-2 weighted spanning basis selects a seeded passage tree; these are
randomized mazes, not a new adversarial difficulty comparison.

Run explicit generation on **Amaterasu**, with a fresh directory per export:

```bash
cargo run --locked --offline -p hymeko_formats --bin adversarial_grid -- --generate-maze hex 0 /tmp/NEW_HEX
cargo run --locked --offline -p hymeko_formats --bin adversarial_grid -- --generate-maze polar 0 /tmp/NEW_POLAR
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
`cargo test --locked --offline -p hymeko_formats --lib environment::`.
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

For search-driven generation, append `--weights GENES.json` to that same command.
The file must contain a JSON array of finite values in `[0,1]`, one per candidate
passage: 42 for hex, 18 for polar. `validation.json` supplies the lexicographically
ordered `priority_pairs` and the encoding contract. Each priority becomes an exact
53-bit integer edge weight; the existing Graph minimum-weight spanning-basis
operation prefers lower weights, with its deterministic arc-order tie break.
`genome.json` retains the supplied vector. Core compilation, Query checks,
connectivity and swept-route clearance remain mandatory. Seed-only generation
retains its prior canonical and physical outputs; the validation metadata is additive.

The [research experiment](../hystl/research/metaheuristics-pilot/README.md#search-driven-world-generation)
compares a frozen GA, the existing DE reference and random proposals using the
actual Cyberdisc run as the objective. Optimization and simulation stay explicit
Amaterasu operations; normal owner tests do not launch that experiment.

## Kawakern physics engine

The generated adversarial grid can import the reusable Cyberdisc actor and
export a real Kawakern scene through the Formats application binary:

```bash
cargo run -p hymeko_formats --bin adversarial_grid --locked --offline -- \
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

### Native mesh preparation (2026-10-21)

`hymeko_geometry::mesh::regularize::SurfaceRegularizer` produces real quad
faces and corner UVs from a triangular source. Quality-ranked local pairing,
conforming face/edge refinement and bounded tangential relaxation preserve
protected seams and boundaries. Projection walks connected source faces and
backs off before folding a quad. The reported source distance is sampled at
vertices; it is not a Hausdorff bound. This is a local quality-based method,
not QuadriFlow or a global cross-field optimizer.

`mesh::normals::NormalSmoothing` smooths normals across coincident UV records
within compatible crease cones without changing geometry or UVs. Texture
rasterization and anatomical authoring remain the consumer's responsibilities.

```powershell
cargo build --locked --offline -p hymeko_geometry --no-default-features --features mesh-optimization --release --example surface_prepare
cargo test --locked --offline -p hymeko_geometry --no-default-features --test mesh_regularization
```

The `surface_prepare quads|normals INPUT.json OUTPUT.json` authoring example
accepts a `TriangleMesh`, `RegularizeOptions` and optional `control_lod` (0..3).
Control reduction uses the existing optional mesh-optimization feature. There
is no Blender invocation or new runtime dependency. The output contains either
a `QuadSurface` with its triangulated GPU representation, or the updated mesh.

### Bounded surface boundary joining

`hymeko_geometry::mesh::bridge::BoundaryBridge::indices` triangulates a gap between
two existing ordered boundaries, including concave rims. It follows source winding,
retains positions/attributes, caps edge length and work, and excludes triangles
already present in the source. Rings together are bounded to2048 vertices. The
`surface_prepare bridge INPUT.json OUTPUT.json` adapter exposes the same operation;
no geometry algorithm lives in the character consumer. Focused tests cover unequal,
reversed and concave rings plus invalid inputs. This is not a general solid boolean.

### Closed surface union with a convex addition

`hymeko_geometry::mesh::solid::SurfaceUnion::with_convex` performs a bounded,
transactional union of two closed, positively oriented surfaces; the addition
must be convex. It constructs canonical edge/face intersections, partitions
source faces, reuses the boundary triangulator and retains exterior regions.
`UnionSurface` carries the resulting `TriangleMesh`, source face IDs and
`VertexOrigin` barycentric weights for application-owned attributes. Original
records remain exact; new normal/UV/tangent records are interpolated. LOD0 is
rebuilt. Output checks use the actual float32 geometric aliases.

This is a targeted authoring operation, not a complete CSG kernel. Inputs must
already be free of self-intersections; combinatorial validation does not prove
that precondition. Coplanar, tangential and numerically ambiguous contacts are
rejected, as are multiple isolated contours within one source triangle. Relative
contact tolerance, intersection storage, face counts and work are bounded.
The existing `surface_prepare union INPUT.json OUTPUT.json` adapter accepts
`mesh`, `addition` and optional `union_options`; it adds no dependency or renderer
logic. Build with `--no-default-features --features authoring,mesh-optimization`.
Focused tests: `cargo test --locked --offline -p hymeko_geometry
--no-default-features --lib mesh::solid::tests`.

# Quad control-surface limit samples

`hymeko_geometry::mesh::subdivision::SubdivisionSurface` evaluates Catmull–Clark
limits at control-vertex parameter locations on an oriented manifold quad cage.
Regular vertices follow the bicubic B-spline limit; extraordinary vertices retain
their valence, boundary curves use the cubic rule with single-face corners fixed.
The borrowed cage is bounded; disconnected fans and inconsistent winding fail.
`QuadSurface::source_vertices` retains original-to-control correspondence, including
UV aliases. `SurfaceFairing::fit_targets` reuses displacement, support and orientation
guards for a supplied field without running fairing iterations. Seam targets must
agree, and failures leave the input unchanged. This is an additive local API.

The existing `surface_prepare limit INPUT.json OUTPUT.json` adapter combines these
owners; build with `--no-default-features --features authoring,mesh-optimization`.
It returns the quad control cage, limit positions and fitted original triangle mesh.

`SubdivisionSurface::limit_positions_with_fixed_boundary` holds boundary CVs exact
for clipped fitting patches. Several oriented open fans may meet at a pinned
corner; malformed winding and a closed fan meeting a boundary remain errors.
Interior CVs retain the same limit rule. The original `limit_positions` stays
strictly manifold. `surface_prepare limit` exposes this through the optional
`fixed_patch_boundary` field; `surface_prepare fair` accepts optional `targets`
and delegates to the existing bounded, transactional `SurfaceFairing::fit_targets`.
It is fixed-density authoring, not an arbitrary-(u,v) evaluator or GPU tessellator.
No new dependency. Reference: https://opensubdiv.org/docs/subdivision_surfaces.html.
Focused checks: `cargo test --locked --offline -p hymeko_geometry --no-default-features
--lib mesh::subdivision` and `--test mesh_regularization`.
## Optional skin optical profile

`PbrMaterial::skin: Option<SkinProfile>` opts a material into the consumer's skin
shading. `SkinProfile` validates RGB Gaussian diffusion sigma in world metres,
strength, Beckmann slope scale, IOR, specular scale and skin normal strength. The Gaussian profile is an
artistic visible-surface approximation, not measured tissue coefficients. When
present, the existing AO texture's alpha carries the explicit skin mask; its red
channel retains AO and green multiplies skin specular (white is neutral).
`roughness` scales the squared mapped perceptual roughness, with neutral default1;
`normal_scale` defaults to1 and is bounded to0..4. Old serialized materials default to `None`. Renderer resources,
BRDF evaluation and screen-space filtering remain in Kawamirai.
The current artist-authored human diffusion default is sigma4.5/2/1mm (RGB),
strength0.9. Explicitly authored profiles retain their supplied values. This is
visible-surface diffusion, not measured tissue data or backside transmission.
Focused checks: `cargo test --locked --offline -p hymeko_geometry
--no-default-features --lib material::tests`.
