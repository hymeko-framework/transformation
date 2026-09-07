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

## Provenance / notes

- Migrated from the `hymeko_framework_rust` monorepo (emitter + query + formats). `core`/`parser` kept external as git deps.
- `test_mdsd_reuse` was gated out — it reaches into the sibling `hymeko_ros2_demo` crate (a demo, outside this layer).

## Ryoshi physics engine

**Ryoshi** names the existing HyMeKo physical engine family used by Kawamirai.
`hymeko_geometry::engine::Engine` owns articulated rigid dynamics and ground contact;
`hymeko_dynamics` projects physical systems, including linear elasticity, into the
existing Tensor numerical kernels. Graph owns signed incidence and octree structure.
This name does not rename crates, add dependencies, or imply full MuJoCo parity.
