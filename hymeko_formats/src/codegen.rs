//! Shared format dispatch through the registered backend.
//! Templates use Query's template engine. MJCF uses the single articulated
//! model emitter registered by Formats; no flat-body template bypass remains.

use std::path::{Path, PathBuf};

use hymeko::ir::ir::Ir;

use hymeko_geometry::transforms::TransformConfig;
use hymeko_query::NameResolver;

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

/// Unified codegen entry point. Backend choice belongs to the registry.
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

    reg.render(
        name,
        &hymeko_query::QueryEngine::new(ir, resolver),
        &cfg,
        &root,
    )
    .ok_or_else(|| CodegenError::QueryFailed(format!("no transform registered for `{name}`")))?
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
