//! Gazebo plugin extraction for T11 (Paper 2).
//!
//! Walks hyperedges inheriting `sim_plugin` or `control_plugin` (as
//! declared in `data/robotics/meta_kinematics.hymeko`) and pulls out the
//! child string values that populate an SDF `<plugin>` tag:
//!
//! ```text
//! @sim_control_plugin: meta_kinematics.kinematics.sim_plugin {
//!     plugin "gz_ros2_control::GazeboSimROS2ControlPlugin";
//!     filename "gz_ros2_control-system";
//!     parameters "moveo_control.yaml";
//! }
//! ```
//!
//! Becomes a [`GazeboPluginInfo`] with `plugin =
//! "gz_ros2_control::GazeboSimROS2ControlPlugin"`, `filename =
//! "gz_ros2_control-system"`, `parameters = "moveo_control.yaml"`, kind =
//! `Sim`. The Gazebo-world emitter consumes these and renders them as
//! `<plugin name=... filename=...>` tags inside the world XML.

use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{Ir, ValueR};

use crate::traits::NameResolver;
use crate::{Predicate, QueryEngine};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GazeboPluginKind {
    /// Inherits `sim_plugin` — world-level simulator plugin
    /// (e.g. `gz_ros2_control::GazeboSimROS2ControlPlugin`).
    Sim,
    /// Inherits `control_plugin` — per-model controller plugin
    /// (e.g. `gz_ros2_control/GazeboSimSystem`).
    Control,
}

#[derive(Debug, Clone)]
pub struct GazeboPluginInfo {
    pub did: DeclId,
    /// Fully-qualified edge name (e.g. `"sim_control_plugin"`).
    pub edge_name: String,
    pub kind: GazeboPluginKind,
    /// Value of the `plugin` child — the C++ class name.
    pub plugin_class: Option<String>,
    /// Value of the `filename` child — the shared-library basename.
    pub filename: Option<String>,
    /// Value of the `parameters` child — the parameter YAML path
    /// (typically `<robot>_control.yaml`).
    pub parameters: Option<String>,
}

impl GazeboPluginInfo {
    /// Returns `true` if the plugin has enough information to emit a
    /// usable `<plugin>` tag. Only `plugin_class` is required —
    /// `filename` is commonly omitted on **control** plugins (they're
    /// loaded via the sim plugin's shared library; see
    /// `gz_ros2_control/GazeboSimSystem`).
    pub fn is_complete(&self) -> bool {
        self.plugin_class.is_some()
    }
}

/// Resolve a single string-valued child of `did` with the given name.
fn find_child_str<R: NameResolver>(
    ir: &Ir,
    res: &R,
    did: DeclId,
    name: &str,
) -> Option<String> {
    ir.decl_children(did).find_map(|cid| {
        let child = &ir.decl_nodes[cid.0];
        if res.resolve(child.name) != name {
            return None;
        }
        match &child.anno.value {
            Some(ValueR::Str(sid)) => Some(res.resolve(*sid).to_string()),
            _ => None,
        }
    })
}

fn build_plugin_info<R: NameResolver>(
    ir: &Ir,
    res: &R,
    did: DeclId,
    kind: GazeboPluginKind,
) -> GazeboPluginInfo {
    let edge_name = res.resolve(ir.decl_nodes[did.0].name).to_string();
    GazeboPluginInfo {
        did,
        edge_name,
        kind,
        plugin_class: find_child_str(ir, res, did, "plugin"),
        filename: find_child_str(ir, res, did, "filename"),
        parameters: find_child_str(ir, res, did, "parameters"),
    }
}

/// Walk the IR for every hyperedge inheriting `sim_plugin` or
/// `control_plugin`. Each match becomes a [`GazeboPluginInfo`]; the
/// caller decides what to do with incomplete records (the Gazebo world
/// emitter filters on [`GazeboPluginInfo::is_complete`]).
pub fn extract_gazebo_plugins<R: NameResolver>(
    engine: &QueryEngine<'_, R>,
) -> Vec<GazeboPluginInfo> {
    let ir = engine.ir();
    let res = engine.resolver();
    let mut out = Vec::new();

    for m in engine.query(&Predicate::edge().and(Predicate::inherits("sim_plugin"))) {
        out.push(build_plugin_info(ir, res, m.id, GazeboPluginKind::Sim));
    }
    for m in engine.query(&Predicate::edge().and(Predicate::inherits("control_plugin"))) {
        out.push(build_plugin_info(ir, res, m.id, GazeboPluginKind::Control));
    }

    out
}
