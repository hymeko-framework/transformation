//! Text generation from query results — **template-driven**.
//!
//! `generate_description` dispatches every registered format through the
//! template engine (`TransformRegistry::render_from_templates`). The
//! format-specific output lives in `<workspace>/transforms/<name>/`
//! (`queries.hymeko` + `template.*`), not in Rust code. This is the
//! canonical zero-hard-coding path used by the CLI `Compile` subcommand.
//!
//! Per-format rich Rust emitters still exist in `crate::{urdf, sdf}` for
//! tests and callers that predate the template-driven pipeline, but
//! nothing in the main dispatch path touches them any more.

use std::path::{Path, PathBuf};

use hymeko::ir::ir::Ir;

use hymeko_query::NameResolver;
use hymeko_query::transforms::TransformConfig;

#[derive(Debug, Clone, Copy)]
pub enum OutputFormat {
    Urdf,
    Sdf17,
    Mjcf,
    DotGraph,
    /// PyTorch dataflow projection — emits an `nn.Module` Python source
    /// file from the hierarchical hypergraph IR via
    /// `transforms/torch_dataflow/`.
    TorchDataflow,
    /// SysML 2 textual concrete syntax — emits a `package` containing
    /// `part def`s for link/joint types and connection instances for
    /// the IR's hyperedges, via `transforms/sysml/`.
    Sysml,
}

impl OutputFormat {
    /// Transform-registry name used to look up the `<workspace>/transforms/<name>/`
    /// directory containing `queries.hymeko` + `template.*`.
    fn transform_name(self) -> &'static str {
        match self {
            OutputFormat::Urdf => "urdf",
            OutputFormat::Sdf17 => "sdf",
            OutputFormat::Mjcf => "mjcf",
            OutputFormat::DotGraph => "dot",
            OutputFormat::TorchDataflow => "torch_dataflow",
            OutputFormat::Sysml => "sysml",
        }
    }
}

/// Resolve the workspace-level `transforms/` directory.
fn default_transforms_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("hymeko_formats has a parent")
        .join("transforms")
}

/// Unified codegen entry point. Pure template dispatch — no format-specific
/// Rust string builders live in this module.
pub fn generate_description<R: NameResolver>(
    ir: &Ir,
    resolver: &R,
    robot_name: &str,
    format: OutputFormat,
) -> Result<String, CodegenError> {
    let reg = crate::default_registry();
    let cfg = TransformConfig::default().with_name(robot_name);
    let name = format.transform_name();
    let root = default_transforms_root();

    reg.render_from_templates(name, ir, resolver, &cfg, &root)
        .ok_or_else(|| CodegenError::QueryFailed(format!(
            "no template registered for `{name}`"
        )))?
        .map_err(CodegenError::QueryFailed)
}

#[derive(Debug)]
pub enum CodegenError {
    QueryFailed(String),
    MissingField(String),
    InvalidTopology(String),
}

impl std::fmt::Display for CodegenError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::QueryFailed(s) => write!(f, "Query failed: {s}"),
            Self::MissingField(s) => write!(f, "Missing required field: {s}"),
            Self::InvalidTopology(s) => write!(f, "Invalid topology: {s}"),
        }
    }
}
