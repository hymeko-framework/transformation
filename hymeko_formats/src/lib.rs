//! HyMeKo built-in format plugins.
//!
//! `hymeko_query` owns the plugin API — the [`DomainTransform`] trait,
//! the [`TransformRegistry`], the template dispatcher — but has zero
//! hard-coded knowledge of any specific output format. This crate is
//! the first (and currently only) consumer of that API: it ships the
//! six standard formats (URDF, SDF, MJCF, DOT, Mermaid, Gazebo world)
//! as concrete [`DomainTransform`] implementations and exposes
//! [`register_defaults`] / [`default_registry`] so CLI / tests /
//! downstream consumers can assemble a ready-to-use registry in one
//! call.
//!
//! [`DomainTransform`]: hymeko_query::transforms::DomainTransform
//! [`TransformRegistry`]: hymeko_query::transforms::TransformRegistry

#![allow(
    clippy::collapsible_if,
    clippy::get_first,
    clippy::needless_borrow,
)]

pub mod binary_graph;
pub mod codegen;
pub mod gazebo;
pub mod sdf;
pub mod snapshot;
pub mod transforms;
pub mod urdf;
pub mod xml_util;

pub use binary_graph::{BinaryEdge, BinaryGraph};
pub use codegen::{generate_description, CodegenError, OutputFormat};
pub use transforms::{
    DotTransform, GazeboWorldTransform, MermaidTransform, MjcfTransform,
    RequirementsDotTransform, RequirementsSysmlTransform, SdfTransform,
    SysmlTransform, TorchDataflowTransform, UrdfTransform,
};

use hymeko_query::transforms::TransformRegistry;

/// Register the built-in transforms into an existing registry.
///
/// This is the single point that binds format names to concrete
/// implementations. `hymeko_query::transforms::TransformRegistry::new()`
/// starts empty; call this (or [`default_registry`]) to wire up the
/// standard set.
pub fn register_defaults(reg: &mut TransformRegistry) {
    reg.register(Box::new(UrdfTransform));
    reg.register(Box::new(SdfTransform));
    reg.register(Box::new(MjcfTransform));
    reg.register(Box::new(DotTransform));
    reg.register(Box::new(GazeboWorldTransform));
    reg.register(Box::new(MermaidTransform));
    reg.register(Box::new(TorchDataflowTransform));
    reg.register(Box::new(SysmlTransform));
    reg.register(Box::new(RequirementsSysmlTransform));
    reg.register(Box::new(RequirementsDotTransform));
}

/// Build a fresh [`TransformRegistry`] with all built-in transforms
/// registered. Equivalent to the previous
/// `TransformRegistry::default()` behaviour before the extraction —
/// kept as a convenience for call sites that want the full default
/// set without manually constructing the registry.
pub fn default_registry() -> TransformRegistry {
    let mut reg = TransformRegistry::new();
    register_defaults(&mut reg);
    reg
}
