//! Domain transform plugin API for HyMeKo.
//!
//! Module structure:
//!   mod.rs            — DomainTransform trait, Registry, Config, Diagnostic
//!   model_view.rs     — ModelView enum, ModelKind, extract() generic functions
//!   transform_engine.rs — TransformEngine (CLI orchestrator, generic on R)
//!
//! The separation ensures:
//!   - DomainTransform is dyn-compatible (no generics)
//!   - Extraction is generic on R: NameResolver (concrete, sized)
//!   - TransformEngine bridges the two layers
//!
//! **No format-specific code lives here.** The six built-in emitters
//! (URDF, SDF, MJCF, DOT, Mermaid, Gazebo world) are provided by the
//! `hymeko_formats` crate and registered via
//! `hymeko_formats::register_defaults(&mut reg)` /
//! `hymeko_formats::default_registry()`.

pub mod model_view;
pub mod transform_engine;

// Re-exports
pub use model_view::{ModelView, ModelKind, extract, extract_kinematic};
pub use transform_engine::TransformEngine;

use std::path::Path;

// ─── Core trait (dyn-compatible: no generics) ─────────────────────────────

/// A domain transform generates output from an extracted model.
///
/// **Dyn-safe**: stored as `Box<dyn DomainTransform>` in the registry.
/// Never touches `Ir` or `NameResolver` — those are handled by
/// `model_view::extract()` before this trait is called.
pub trait DomainTransform {
    /// Unique identifier (e.g., "urdf", "sdf", "mjcf", "dot").
    fn name(&self) -> &'static str;

    /// Output file extension.
    fn extension(&self) -> &'static str;

    /// What model kind this transform consumes.
    fn accepts(&self) -> ModelKind;

    /// Generate output. Returns `None` if model kind doesn't match.
    fn emit(&self, model: &ModelView, config: &TransformConfig) -> Option<String>;

    /// Subdirectory name under `<workspace>/transforms/` that holds the
    /// transform's `queries.hymeko` + `template.<ext>` pair. Returns
    /// `None` for transforms that don't (yet) have a template; the
    /// registry's `render_from_templates` then falls back to whatever
    /// the caller wires up. This is the hook that makes the generation
    /// pipeline data-driven — templates are *files*, not `push_str`
    /// calls.
    fn template_dir(&self) -> Option<&'static str> {
        None
    }

    /// Validate before generation. Default checks joint topology.
    fn validate(&self, model: &ModelView) -> Vec<Diagnostic> {
        let mut diags = Vec::new();
        if let Some(km) = model.as_kinematic() {
            let link_names: std::collections::HashSet<&str> =
                km.links.iter().map(|l| l.name.as_str()).collect();
            for joint in &km.joints {
                if !link_names.contains(joint.parent_link.as_str()) {
                    diags.push(Diagnostic::error(format!(
                        "Joint '{}': unknown parent '{}'", joint.name, joint.parent_link
                    )));
                }
                if !link_names.contains(joint.child_link.as_str()) {
                    diags.push(Diagnostic::error(format!(
                        "Joint '{}': unknown child '{}'", joint.name, joint.child_link
                    )));
                }
            }
        }
        diags
    }
}

// ─── Configuration ────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct TransformConfig {
    pub robot_name: String,
    pub emit_comments: bool,
    pub indent: String,
    pub options: std::collections::HashMap<String, String>,
}

impl Default for TransformConfig {
    fn default() -> Self {
        Self {
            robot_name: "robot".into(),
            emit_comments: true,
            indent: "  ".into(),
            options: Default::default(),
        }
    }
}

impl TransformConfig {
    pub fn with_name(mut self, name: &str) -> Self {
        self.robot_name = name.into(); self
    }
    pub fn with_option(mut self, key: &str, value: &str) -> Self {
        self.options.insert(key.into(), value.into()); self
    }
}

// ─── Diagnostics ──────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct Diagnostic {
    pub level: DiagLevel,
    pub message: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagLevel { Info, Warning, Error }

impl Diagnostic {
    pub fn info(msg: impl Into<String>) -> Self { Self { level: DiagLevel::Info, message: msg.into() } }
    pub fn warning(msg: impl Into<String>) -> Self { Self { level: DiagLevel::Warning, message: msg.into() } }
    pub fn error(msg: impl Into<String>) -> Self { Self { level: DiagLevel::Error, message: msg.into() } }
    pub fn is_error(&self) -> bool { self.level == DiagLevel::Error }
}

// ─── Registry ─────────────────────────────────────────────────────────────

pub struct TransformRegistry {
    pub transforms: Vec<Box<dyn DomainTransform>>,
}

impl Default for TransformRegistry {
    /// Returns an **empty** registry. `hymeko_query` intentionally ships
    /// no format-specific knowledge: call
    /// `hymeko_formats::register_defaults(&mut reg)` (or
    /// `hymeko_formats::default_registry()`) to wire up the six
    /// built-in transforms, or `register()` your own.
    fn default() -> Self {
        Self::new()
    }
}

impl TransformRegistry {
    pub fn new() -> Self { Self { transforms: Vec::new() } }

    pub fn register(&mut self, t: Box<dyn DomainTransform>) {
        self.transforms.push(t);
    }

    pub fn get(&self, name: &str) -> Option<&dyn DomainTransform> {
        self.transforms.iter().find(|t| t.name() == name).map(|t| t.as_ref())
    }

    pub fn by_extension(&self, ext: &str) -> Option<&dyn DomainTransform> {
        self.transforms.iter().find(|t| t.extension() == ext).map(|t| t.as_ref())
    }

    pub fn available(&self) -> Vec<&str> {
        self.transforms.iter().map(|t| t.name()).collect()
    }

    /// Emit all formats from a pre-extracted model.
    pub fn emit_all(&self, model: &ModelView, config: &TransformConfig) -> Vec<(String, String)> {
        self.transforms.iter().filter_map(|t| {
            let output = t.emit(model, config)?;
            Some((format!("{}.{}", config.robot_name, t.extension()), output))
        }).collect()
    }

    /// Write all formats to disk.
    pub fn write_all(
        &self, model: &ModelView, config: &TransformConfig, dir: &Path,
    ) -> std::io::Result<Vec<std::path::PathBuf>> {
        let mut paths = Vec::new();
        for (filename, content) in self.emit_all(model, config) {
            let path = dir.join(&filename);
            std::fs::write(&path, &content)?;
            paths.push(path);
        }
        Ok(paths)
    }

    /// Render a registered transform through the **template engine**
    /// (`hymeko_query::rewrite::template::execute_transform`) — the
    /// canonical data-driven path. The transform's output is produced
    /// by rendering the `template.<ext>` file against query results
    /// from its `queries.hymeko`, *not* by Rust-side `push_str`
    /// calls. Every transform with a registered template directory
    /// can go through this entry point.
    ///
    /// `transforms_root` is the workspace-level `transforms/` directory
    /// (i.e. the one containing `urdf/`, `sdf/`, `mjcf/`, …). In tests
    /// this is resolved via `env!("CARGO_MANIFEST_DIR")/../transforms`;
    /// in production the CLI passes the path from its config.
    ///
    /// Returns `None` when the transform has no registered template
    /// directory. Returns `Some(Err)` on I/O / parse failure so callers
    /// can surface the error.
    pub fn render_from_templates<R: crate::traits::NameResolver>(
        &self,
        name: &str,
        ir: &hymeko::ir::ir::Ir,
        resolver: &R,
        config: &TransformConfig,
        transforms_root: &Path,
    ) -> Option<Result<String, String>> {
        let t = self.get(name)?;
        let subdir = t.template_dir()?;
        let dir = transforms_root.join(subdir);
        Some(render_via_template(ir, resolver, name, &dir, config))
    }
}

/// Load `queries.hymeko` + `template.<ext>` from `dir`, build a
/// [`crate::rewrite::template::TransformSpec`], and hand off to
/// [`crate::rewrite::template::execute_transform`].
///
/// Extension matching is done by scanning the directory for a single
/// file whose name starts with `"template."` — this keeps the helper
/// ignorant of per-format quirks like `template.urdf.xml` vs
/// `template.world.sdf` vs `template.mmd`.
fn render_via_template<R: crate::traits::NameResolver>(
    ir: &hymeko::ir::ir::Ir,
    resolver: &R,
    name: &str,
    dir: &Path,
    config: &TransformConfig,
) -> Result<String, String> {
    use crate::rewrite::template::{execute_transform, TransformSpec};

    let query_path = dir.join("queries.hymeko");
    let query_source = std::fs::read_to_string(&query_path)
        .map_err(|e| format!("reading {}: {e}", query_path.display()))?;

    let template_source = find_template_file(dir)
        .map_err(|e| format!("locating template in {}: {e}", dir.display()))?;

    let spec = TransformSpec {
        name: name.to_string(),
        query_source,
        template_source,
    };

    let mut cfg_map: std::collections::HashMap<String, String> =
        config.options.clone();
    cfg_map.entry("robot_name".to_string()).or_insert(config.robot_name.clone());
    // Gazebo world uses {{config:world_name}} — default to `empty`
    // if the caller didn't specify it.
    cfg_map.entry("world_name".to_string()).or_insert("empty".to_string());

    execute_transform(ir, resolver, &spec, &cfg_map)
}

/// Find the single `template.*` file in a transform directory and
/// return its contents.
fn find_template_file(dir: &Path) -> Result<String, String> {
    let entries = std::fs::read_dir(dir).map_err(|e| format!("read_dir: {e}"))?;
    for entry in entries.flatten() {
        let path = entry.path();
        if path
            .file_name()
            .and_then(|n| n.to_str())
            .is_some_and(|n| n.starts_with("template."))
        {
            return std::fs::read_to_string(&path)
                .map_err(|e| format!("reading {}: {e}", path.display()));
        }
    }
    Err("no `template.*` file found".to_string())
}
