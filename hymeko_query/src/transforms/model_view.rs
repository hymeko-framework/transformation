//! Generalized model extraction from HyMeKo IR.
//!
//! Design:
//! - `ModelView` is an enum of all extractable model types
//! - `extract()` is a generic free function (keeps R concrete, avoids dyn sizing)
//! - Each variant has a dedicated `extract_*` function
//! - New model types = new enum variant + new extract function
//! - DomainTransform receives `&ModelView` (no generics, dyn-safe)
//!
//! ```text
//!                        ┌─ KinematicModel ──→ URDF, SDF, MJCF, DOT
//! IR + R: NameResolver ──┤─ ElectricalModel ──→ SPICE, KiCad (future)
//!                        └─ CommModel ────────→ DDS config (future)
//! ```

use hymeko::ir::ir::Ir;
use crate::engine::QueryEngine;
use crate::traits::NameResolver;
use crate::kinematics::kinematic::{self, KinematicModel};

// ─── Model kinds ──────────────────────────────────────────────────────────

/// Which model type to extract from the IR.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ModelKind {
    Kinematic,
    // Future:
    // Electrical,
    // Communication,
    // BehaviorTree,
}

// ─── Model view (the extracted result) ────────────────────────────────────

/// A type-erased model extracted from the IR.
///
/// This is what `DomainTransform::emit()` receives.
/// Each variant carries the domain-specific model struct.
pub enum ModelView {
    Kinematic(KinematicModel),
    // Future:
    // Electrical(ElectricalModel),
    // Communication(CommModel),
}

impl ModelView {
    /// Downcast to kinematic model.
    pub fn as_kinematic(&self) -> Option<&KinematicModel> {
        match self {
            ModelView::Kinematic(m) => Some(m),
            #[allow(unreachable_patterns)]
            _ => None,
        }
    }

    /// What kind of model this is.
    pub fn kind(&self) -> ModelKind {
        match self {
            ModelView::Kinematic(_) => ModelKind::Kinematic,
        }
    }
}

// ─── Extraction (generic on R, free functions) ────────────────────────────

/// Extract a model of the given kind from compiled IR.
///
/// This is the **single entry point** for all extraction.
/// It's generic on `R: NameResolver` so `QueryEngine` gets a concrete type.
/// The generic is erased here — callers of `DomainTransform::emit` never
/// see `R`, they only see `&ModelView`.
///
/// ```rust,ignore
/// let model = extract(&ir, &interner, "moveo", ModelKind::Kinematic);
/// let urdf = registry.get("urdf").unwrap().emit(&model, &config);
/// ```
pub fn extract<R: NameResolver>(
    ir: &Ir,
    resolver: &R,
    name: &str,
    kind: ModelKind,
) -> ModelView {
    match kind {
        ModelKind::Kinematic => extract_kinematic(ir, resolver, name),
    }
}

/// Extract a kinematic model (links, joints, topology).
pub fn extract_kinematic<R: NameResolver>(
    ir: &Ir,
    resolver: &R,
    robot_name: &str,
) -> ModelView {
    let engine = QueryEngine::new(ir, resolver);
    let model = kinematic::extract_kinematic_model(&engine, robot_name);
    ModelView::Kinematic(model)
}

// Future extractors follow the same pattern:
//
// pub fn extract_electrical<R: NameResolver>(ir: &Ir, resolver: &R, name: &str) -> ModelView {
//     let engine = QueryEngine::new(ir, resolver);
//     let model = electrical::extract_electrical_model(&engine, name);
//     ModelView::Electrical(model)
// }