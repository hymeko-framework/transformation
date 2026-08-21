//! Transform engine — CLI orchestrator for the full pipeline.
//!
//! ```text
//! .hymeko → compile → IR → extract(R) → ModelView → emit → {URDF, SDF, ...}
//!                          ^^^^^^^^^^^               ^^^^^
//!                          generic (R concrete)      dyn-safe (no R)
//! ```
//!
//! The generic `R: NameResolver` lives ONLY in `TransformEngine` methods.
//! Everything downstream (`DomainTransform::emit`, `ModelView`) is type-erased.

use hymeko::ir::ir::Ir;
use crate::traits::NameResolver;
use crate::transforms::model_view::{extract, ModelKind};
use crate::transforms::{
    TransformConfig, TransformRegistry, Diagnostic,
};
use std::path::Path;

/// Orchestrates compile → extract → emit.
///
/// ```rust,ignore
/// let engine = TransformEngine::default();
///
/// // Single format
/// let urdf = engine.emit_format(&ir, &interner, "moveo", "urdf")?;
///
/// // Multiple formats
/// let results = engine.emit_formats(&ir, &interner, "moveo", &["urdf", "sdf", "mjcf"]);
///
/// // All formats to directory
/// engine.write_all(&ir, &interner, "moveo", &output_dir)?;
///
/// // Validate
/// let diags = engine.validate(&ir, &interner, "moveo");
/// ```
pub struct TransformEngine {
    pub registry: TransformRegistry,
    /// Default model kind for extraction. Can be overridden per call.
    pub default_model: ModelKind,
}

impl Default for TransformEngine {
    fn default() -> Self {
        Self {
            registry: TransformRegistry::default(),
            default_model: ModelKind::Kinematic,
        }
    }
}

impl TransformEngine {
    /// Extract model + emit a single format.
    pub fn emit_format<R: NameResolver>(
        &self,
        ir: &Ir,
        resolver: &R,
        robot_name: &str,
        format: &str,
    ) -> Option<String> {
        let model = extract(ir, resolver, robot_name, self.default_model);
        let config = TransformConfig::default().with_name(robot_name);
        let t = self.registry.get(format)?;
        t.emit(&model, &config)
    }

    /// Extract model + emit multiple formats.
    pub fn emit_formats<R: NameResolver>(
        &self,
        ir: &Ir,
        resolver: &R,
        robot_name: &str,
        formats: &[&str],
    ) -> Vec<(String, String)> {
        let model = extract(ir, resolver, robot_name, self.default_model);
        let config = TransformConfig::default().with_name(robot_name);

        formats.iter().filter_map(|&fmt| {
            let t = self.registry.get(fmt)?;
            let output = t.emit(&model, &config)?;
            Some((format!("{}.{}", robot_name, t.extension()), output))
        }).collect()
    }

    /// Extract model + emit ALL registered formats.
    pub fn emit_all<R: NameResolver>(
        &self,
        ir: &Ir,
        resolver: &R,
        robot_name: &str,
    ) -> Vec<(String, String)> {
        let model = extract(ir, resolver, robot_name, self.default_model);
        let config = TransformConfig::default().with_name(robot_name);
        self.registry.emit_all(&model, &config)
    }

    /// Extract + emit all + write to directory.
    pub fn write_all<R: NameResolver>(
        &self,
        ir: &Ir,
        resolver: &R,
        robot_name: &str,
        output_dir: &Path,
    ) -> std::io::Result<Vec<std::path::PathBuf>> {
        let model = extract(ir, resolver, robot_name, self.default_model);
        let config = TransformConfig::default().with_name(robot_name);
        self.registry.write_all(&model, &config, output_dir)
    }

    /// Validate against all transforms.
    pub fn validate<R: NameResolver>(
        &self,
        ir: &Ir,
        resolver: &R,
        robot_name: &str,
    ) -> Vec<(String, Vec<Diagnostic>)> {
        let model = extract(ir, resolver, robot_name, self.default_model);
        self.registry.transforms.iter().map(|t| {
            (t.name().to_string(), t.validate(&model))
        }).collect()
    }

    /// Extract + emit with custom config.
    pub fn emit_with_config<R: NameResolver>(
        &self,
        ir: &Ir,
        resolver: &R,
        config: &TransformConfig,
        format: &str,
    ) -> Option<String> {
        let model = extract(ir, resolver, &config.robot_name, self.default_model);
        let t = self.registry.get(format)?;
        t.emit(&model, config)
    }
}
