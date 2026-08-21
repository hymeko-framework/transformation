//! Gazebo world emission — T11 (Paper 2).
//!
//! Template-driven: output is produced by rendering
//! `<workspace>/transforms/gazebo/template.world.sdf` against the query
//! set in `<workspace>/transforms/gazebo/queries.hymeko`.
//!
//! The public entry point [`generate_gazebo_world`] is preserved for
//! call-site compatibility (tests, docs, the launch-bundle builder);
//! internally it composes a [`TransformConfig`] with the requested
//! `robot_name` + `world_name` and hands off to
//! [`TransformRegistry::render_from_templates`]. The registry must have
//! the `gazebo` transform registered by the caller (via
//! [`crate::register_defaults`] or direct `register()` call).

use std::path::{Path, PathBuf};

use hymeko::ir::ir::Ir;

use hymeko_query::traits::NameResolver;
use hymeko_query::transforms::TransformConfig;

/// Resolve the workspace-level `transforms/` directory. This uses
/// `CARGO_MANIFEST_DIR`, which resolves at compile time to
/// `hymeko_formats/`; the transforms dir is one level up.
///
/// For deployments where the binary ships without the templates
/// alongside, callers should use
/// [`TransformRegistry::render_from_templates`] directly and pass a
/// resolved path instead.
pub fn default_transforms_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("hymeko_formats has a parent")
        .join("transforms")
}

/// Generate a full SDF 1.8 world document for a HyMeKo-described robot.
///
/// Data-driven — the output comes from
/// `transforms/gazebo/template.world.sdf` + `queries.hymeko`, not a
/// Rust string builder.
pub fn generate_gazebo_world<R: NameResolver>(
    ir: &Ir,
    resolver: &R,
    robot_name: &str,
    world_name: &str,
) -> String {
    let reg = crate::default_registry();
    let cfg = TransformConfig::default()
        .with_name(robot_name)
        .with_option("world_name", world_name);
    reg.render_from_templates("gazebo", ir, resolver, &cfg, &default_transforms_root())
        .expect("gazebo transform registered in hymeko_formats::default_registry")
        .unwrap_or_else(|e| panic!("gazebo template render failed: {e}"))
}
