# hymeko_transformation

The HyMeKo IR→output (transformation) layer — a standalone cargo workspace.

- **`hymeko_emitter/`** — editor IR (slotmap, `IRDelta`) + M2T emitters (`.hymeko`, SysML v2, Rust stubs, Lean 4)
- **`hymeko_query/`** — pattern-matching query engine over the hypergraph IR
- **`hymeko_formats/`** — built-in format emitters (URDF / SDF / MJCF / DOT / Mermaid / Gazebo world) — a query plugin
- **`hymeko_webxr/`** — WebXR/spatial vocabulary, semantic validation and borrowed queries over the canonical IR

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

## Kawakern physics engine

**Kawakern** names the existing HyMeKo physical engine family used by Kawamirai.
`hymeko_geometry::engine::Engine` owns articulated rigid dynamics and ground contact;
`hymeko_dynamics` projects physical systems, including linear elasticity, into the
existing Tensor numerical kernels. Graph owns signed incidence and octree structure.
This name does not rename crates, add dependencies, or imply full MuJoCo parity.
