# transformation

Projections of the canonical HyMeKo IR. A model that
[core](https://github.com/hymeko-framework/core) compiled is read here and
turned into something else: query results, HyMeKo source again, robot
descriptions (URDF, SDF, MJCF), diagrams, SysML v2, relational tables, WebXR
scene semantics, kinematic models and generated worlds. Nothing here parses,
resolves or keeps a second model; every crate borrows Core's `Ir`.

## Crates

| Crate | Contents | Framework dependencies |
|---|---|---|
| [`hymeko_query`](hymeko_query/) | Pattern-matching query engine over the hypergraph IR; borrowed semantic queries (bases and is-a, effective fields, own slots, root-scoped selections); the template engine the emitters render with | core, parser |
| [`hymeko_emitter`](hymeko_emitter/) | Editor IR (slotmap, `IRDelta`) and model-to-text emitters: `.hymeko` source, SysML v2, Rust stubs, Lean 4 | core |
| [`hymeko_mechanism`](hymeko_mechanism/) | Mechanisms read out of IR: links, joints, frames, actuation, visuals and cameras into Geometry's kinematic model with Core declaration handles (`kinematics::IrHandles`); kinematic assemblies; geometry-processing recipes | core, query, tensor, geometry |
| [`hymeko_formats`](hymeko_formats/) | Format emitters (URDF, SDF, MJCF, DOT, Mermaid, Gazebo world, PyTorch dataflow, SysML) and domain projections: traceability export, structural diagrams, generated environments, metaheuristic contracts, robot projection | core, query, emitter, mechanism, graph, tensor, kawakern |
| [`hymeko_webxr`](hymeko_webxr/) | WebXR and spatial vocabulary, semantic validation and borrowed queries over the canonical IR | core, query |
| [`hymeko_relational`](hymeko_relational/) | The IR as relational tables (declarations, own and effective fields, bases, arcs, one table per kind) and the PostgreSQL that holds them | core, query |

## Use

```toml
[dependencies]
hymeko_core = { git = "https://github.com/hymeko-framework/core.git" }
hymeko_formats = { git = "https://github.com/hymeko-framework/transformation.git" }
```

```rust
use std::path::Path;

use hymeko::module_store::{execution::CompiledModel, source_provider::StdFsProvider};
use hymeko::util::real_parser::RealParser;
use hymeko_formats::{OutputFormat, generate_description};

fn urdf(path: &Path) -> Result<String, String> {
    let model = CompiledModel::compile(StdFsProvider::new(), RealParser, path)
        .map_err(|e| format!("{e:?}"))?;
    generate_description(&model.program().ir, model.interner(), "robot", OutputFormat::Urdf)
        .map_err(|e| e.to_string())
}
```

`generate_description`, the registered transforms and `TransformEngine` share
one model emitter per robot format. The robots under
[`data/robotics/`](data/robotics/) are ready inputs, for example
`data/robotics/sim/dual_fanuc/fanuc_lrmate200id.hymeko`. The contracts of the
emitters and projections are in [docs/api-notes.md](docs/api-notes.md); the
WebXR vocabulary is specified in
[docs/webxr-semantic-model.md](docs/webxr-semantic-model.md).

## Command-line tools

`hymeko_formats` ships thin binaries over the same library API:

| Binary | Purpose |
|---|---|
| `robot_projection` | Exports a compiled robot as MJCF plus its model and forward-kinematics JSON |
| `metaheuristic_contract` | Validates a metaheuristic profile and exports its structural contract |
| `adversarial_grid` | Generates square, hexagonal and polar mazes and legged terrain, compares Graph searches over them, and exports physics scenes |
| `robot_family` | Generates, checks and prepares robot-family candidates |
| `graph_rank` | Runs a compiled graph resource with its declared run profile |

The binaries that write files refuse to overwrite: their output directories
must be new.

## Build and test

```sh
cargo build --workspace --locked
cargo test --workspace --locked -j 2
```

A few tests read robot models from a separate robotics checkout placed beside
this one. They are `#[ignore]`d and do not run by default.

Two optional tools have their own runtimes: `tools/diagram` renders DOT to SVG
with Viz.js (Node), and `tools/traceability` cross-checks a traceability export
(Python).

## Repository layout

- `data/` — `.hymeko` fixtures the tests and examples load: robots, minimal
  examples, neural networks, SysML and the traceability case.
- `transforms/` — one directory per registered transform: its
  `queries.hymeko` and templates.
- `generated/` — a checked-in example of generated launch files.

## Where the line is

- Parsing, resolution, canonical IR, hashing and caches: parser and core.
- Meshes, materials and kinematic computation without IR: geometry's
  `hymeko_mesh` and `hymeko_kinematics`. `hymeko_mechanism` re-exports both
  with Core declaration handles.
- Physics stepping over a kinematic model: kawakern
  (`kawakern::Engine<IrHandles>`).
- Graph search, spanning bases and other graph algorithms: graph.
- Drawing and runtime XR sessions belong to consumers; `hymeko_webxr`
  describes authored scenes only.

## License

MIT. See [LICENSE](LICENSE).
