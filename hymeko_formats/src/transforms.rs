//! Built-in `DomainTransform` implementations for the six standard
//! formats (URDF, SDF, MJCF, DOT, Mermaid, Gazebo world).
//!
//! These live outside `hymeko_query` so the core query/transform crate
//! has zero hard-coded format knowledge. Callers assemble a registry by
//! calling [`crate::register_defaults`] or [`crate::default_registry`].
//!
//! Every transform here declares a `template_dir()`, so the canonical
//! code path is still `TransformRegistry::render_from_templates` — the
//! Rust string builders below are retained only as:
//!   * `emit()` fall-backs used by `TransformRegistry::emit_all` /
//!     `write_all` (the non-template registry surface exercised by the
//!     ecosystem tests), and
//!   * the rich `generate_urdf_from_model` / `generate_sdf_from_model`
//!     entry points consumed directly by legacy tests.

use std::fmt::Write;

use std::collections::{HashMap, HashSet};

use hymeko_query::kinematics::joints::{JointInfo, JointType};
use hymeko_query::kinematics::kinematic::{GeometryShape, KinematicModel, LinkInfo};
use hymeko_query::transforms::{
    Diagnostic, DomainTransform, ModelKind, ModelView, TransformConfig,
};

use crate::xml_util::xml_escape;

// ═══════════════════════════════════════════════════════════════════════════
// Transform implementations
// ═══════════════════════════════════════════════════════════════════════════

// ─── URDF ─────────────────────────────────────────────────────────────────

pub struct UrdfTransform;

impl DomainTransform for UrdfTransform {
    fn name(&self) -> &'static str {
        "urdf"
    }
    fn extension(&self) -> &'static str {
        "urdf"
    }
    fn accepts(&self) -> ModelKind {
        ModelKind::Kinematic
    }
    fn emit(&self, model: &ModelView, config: &TransformConfig) -> Option<String> {
        let km = model.as_kinematic()?;
        Some(emit_urdf_stub(km, &config.robot_name))
    }
    fn template_dir(&self) -> Option<&'static str> {
        Some("urdf")
    }
}

// ─── SDF ──────────────────────────────────────────────────────────────────

pub struct SdfTransform;

impl DomainTransform for SdfTransform {
    fn name(&self) -> &'static str {
        "sdf"
    }
    fn extension(&self) -> &'static str {
        "sdf"
    }
    fn accepts(&self) -> ModelKind {
        ModelKind::Kinematic
    }
    fn emit(&self, model: &ModelView, config: &TransformConfig) -> Option<String> {
        let km = model.as_kinematic()?;
        Some(emit_sdf_stub(km, &config.robot_name))
    }
    fn template_dir(&self) -> Option<&'static str> {
        Some("sdf")
    }
}

// ─── MJCF ─────────────────────────────────────────────────────────────────

pub struct MjcfTransform;

impl DomainTransform for MjcfTransform {
    fn name(&self) -> &'static str {
        "mjcf"
    }
    fn extension(&self) -> &'static str {
        "xml"
    }
    fn accepts(&self) -> ModelKind {
        ModelKind::Kinematic
    }

    fn emit(&self, model: &ModelView, config: &TransformConfig) -> Option<String> {
        let km = model.as_kinematic()?;
        Some(emit_mjcf(km, config))
    }
    fn template_dir(&self) -> Option<&'static str> {
        Some("mjcf")
    }

    fn validate(&self, model: &ModelView) -> Vec<Diagnostic> {
        let mut diags = Vec::new();
        if let Some(km) = model.as_kinematic() {
            let mut child_counts: std::collections::HashMap<&str, usize> =
                std::collections::HashMap::new();
            for j in &km.joints {
                *child_counts.entry(j.child_link.as_str()).or_insert(0) += 1;
            }
            for (name, count) in &child_counts {
                if *count > 1 {
                    diags.push(Diagnostic::error(format!(
                        "'{}' is child in {} joints — MJCF requires tree",
                        name, count
                    )));
                }
            }
        }
        diags
    }
}

// ─── DOT ──────────────────────────────────────────────────────────────────

pub struct DotTransform;

impl DomainTransform for DotTransform {
    fn name(&self) -> &'static str {
        "dot"
    }
    fn extension(&self) -> &'static str {
        "dot"
    }
    fn accepts(&self) -> ModelKind {
        ModelKind::Kinematic
    }
    fn emit(&self, model: &ModelView, config: &TransformConfig) -> Option<String> {
        let km = model.as_kinematic()?;
        Some(emit_dot(km, config))
    }
    fn template_dir(&self) -> Option<&'static str> {
        Some("dot")
    }
}

// ─── Torch (dataflow projection of hierarchical hypergraph) ─────────────

/// PyTorch dataflow-projection emitter. Consumes the hierarchical
/// hypergraph IR (or its π_dataflow projection): top-level layers,
/// tensors, and dataflow hyperedges. Layer hypervertex bodies (inner
/// ports / neurons / factors) are not walked — they are encoded
/// implicitly via each layer's `d_in`, `d_out`, `ggk` fields.
///
/// The emit() body is a stub because this transform is template-only:
/// it goes through `TransformRegistry::render_from_templates` against
/// `transforms/torch_dataflow/{queries.hymeko, template.py}`. The
/// kinematic ModelView is not the right input shape for an NN.
pub struct TorchDataflowTransform;

impl DomainTransform for TorchDataflowTransform {
    fn name(&self) -> &'static str {
        "torch_dataflow"
    }
    fn extension(&self) -> &'static str {
        "py"
    }
    fn accepts(&self) -> ModelKind {
        ModelKind::Kinematic
    }
    fn emit(&self, _model: &ModelView, _config: &TransformConfig) -> Option<String> {
        // Template-only path; emit() is unused for this transform.
        None
    }
    fn template_dir(&self) -> Option<&'static str> {
        Some("torch_dataflow")
    }
}

// ─── Mermaid flowchart ───────────────────────────────────────────────────

/// Emits a Mermaid `flowchart TD` of the kinematic chain — renders inline
/// on GitHub, in the VS Code Mermaid preview, Obsidian, and most docs
/// sites without any Graphviz toolchain. Lossier than DOT for N-ary
/// hyperedges (Mermaid has no native hyperedge), but gives browser-native
/// rendering with zero external dependency.
pub struct MermaidTransform;

impl DomainTransform for MermaidTransform {
    fn name(&self) -> &'static str {
        "mermaid"
    }
    fn extension(&self) -> &'static str {
        "mmd"
    }
    fn accepts(&self) -> ModelKind {
        ModelKind::Kinematic
    }
    fn emit(&self, model: &ModelView, config: &TransformConfig) -> Option<String> {
        let km = model.as_kinematic()?;
        Some(emit_mermaid(km, config))
    }
    fn template_dir(&self) -> Option<&'static str> {
        Some("mermaid")
    }
}

// ─── SysML 2 textual ──────────────────────────────────────────────────────

/// `DomainTransform` front-end for SysML 2 textual emission.
///
/// Maps the kinematic model into a SysML 2 `package`:
/// - `link` decls → `part def Link` instances
/// - joint kinds → `connection def`-typed connection instances
/// - `+arc-ref` / `-arc-ref` → first / second connection endpoint
///
/// The template-driven path (`transforms/sysml/template.sysml`) is the
/// canonical emitter; this Rust `emit()` is a minimal fallback used by
/// direct API callers that bypass `render_from_templates`.
pub struct SysmlTransform;

impl DomainTransform for SysmlTransform {
    fn name(&self) -> &'static str {
        "sysml"
    }
    fn extension(&self) -> &'static str {
        "sysml"
    }
    fn accepts(&self) -> ModelKind {
        ModelKind::Kinematic
    }
    fn emit(&self, model: &ModelView, config: &TransformConfig) -> Option<String> {
        let km = model.as_kinematic()?;
        Some(emit_sysml(km, config))
    }
    fn template_dir(&self) -> Option<&'static str> {
        Some("sysml")
    }
}

// ─── Requirements traceability (SysML v2 + DOT) — SMC #5 witness ───────────

/// SysML v2 requirements-traceability emission (paper witness, SMC #5).
/// Template-only: rendered via `render_from_templates` from
/// `transforms/requirements_sysml/` — it consumes requirement/block decls and
/// signed trace hyperedges, not a kinematic model, so `emit()` has no Rust
/// fallback. The same template is reused in the WASM editor via `include_str!`.
pub struct RequirementsSysmlTransform;

impl DomainTransform for RequirementsSysmlTransform {
    fn name(&self) -> &'static str {
        "requirements_sysml"
    }
    fn extension(&self) -> &'static str {
        "sysml"
    }
    fn accepts(&self) -> ModelKind {
        ModelKind::Kinematic // unused on the template path; no kinematic extraction occurs
    }
    fn emit(&self, _model: &ModelView, _config: &TransformConfig) -> Option<String> {
        None // template-only — see render_from_templates("requirements_sysml", …)
    }
    fn template_dir(&self) -> Option<&'static str> {
        Some("requirements_sysml")
    }
}

/// DOT traceability-graph emission for the same requirements witness (SMC #5).
/// Template-only, paired with [`RequirementsSysmlTransform`].
pub struct RequirementsDotTransform;

impl DomainTransform for RequirementsDotTransform {
    fn name(&self) -> &'static str {
        "requirements_dot"
    }
    fn extension(&self) -> &'static str {
        "dot"
    }
    fn accepts(&self) -> ModelKind {
        ModelKind::Kinematic // unused on the template path
    }
    fn emit(&self, _model: &ModelView, _config: &TransformConfig) -> Option<String> {
        None // template-only — see render_from_templates("requirements_dot", …)
    }
    fn template_dir(&self) -> Option<&'static str> {
        Some("requirements_dot")
    }
}

// ─── Gazebo world (gz sim) ────────────────────────────────────────────────

/// `DomainTransform` front-end for the Gazebo world emitter.
///
/// The full emitter ([`crate::gazebo::generate_gazebo_world`]) needs
/// access to the raw `Ir` + `NameResolver` to walk `sim_plugin` /
/// `control_plugin` hyperedges — information that the `ModelView`
/// abstraction doesn't carry. The registry entry therefore emits a
/// plugins-stripped world skeleton (physics + ground plane + inline
/// robot model) so round-trip + count assertions still work; callers
/// that need the full plugin-populated output use the free function
/// directly, the same way URDF/SDF consumers already do.
pub struct GazeboWorldTransform;

impl DomainTransform for GazeboWorldTransform {
    fn name(&self) -> &'static str {
        "gazebo"
    }
    fn extension(&self) -> &'static str {
        "world.sdf"
    }
    fn accepts(&self) -> ModelKind {
        ModelKind::Kinematic
    }
    fn emit(&self, model: &ModelView, config: &TransformConfig) -> Option<String> {
        let km = model.as_kinematic()?;
        Some(emit_gazebo_world_stub(km, &config.robot_name, "default"))
    }
    fn template_dir(&self) -> Option<&'static str> {
        Some("gazebo")
    }
    fn validate(&self, model: &ModelView) -> Vec<Diagnostic> {
        // Same tree-topology check as MJCF — gz sim requires a DAG.
        let mut diags = Vec::new();
        if let Some(km) = model.as_kinematic() {
            let mut child_counts: std::collections::HashMap<&str, usize> =
                std::collections::HashMap::new();
            for j in &km.joints {
                *child_counts.entry(j.child_link.as_str()).or_insert(0) += 1;
            }
            for (name, count) in &child_counts {
                if *count > 1 {
                    diags.push(Diagnostic::error(format!(
                        "gz sim: link `{}` is child of {} joints — tree required",
                        name, count
                    )));
                }
            }
        }
        diags
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Emit functions
// ═══════════════════════════════════════════════════════════════════════════

fn emit_urdf_stub(model: &KinematicModel, name: &str) -> String {
    // Delegate to the model-view rich emitter. Set the model name so the
    // `<robot name=...>` header matches the requested `config.robot_name`.
    let mut m = model.clone();
    m.name = name.to_string();
    crate::urdf::generate_urdf_from_model(&m)
}

fn emit_gazebo_world_stub(_model: &KinematicModel, robot_name: &str, world_name: &str) -> String {
    // This deliberately mirrors the shape of
    // `generate_gazebo_world` but drops the inline robot model and the
    // extracted `sim_plugin` / `control_plugin` tags — the `ModelView`
    // abstraction doesn't expose the raw `Ir`, which is required to
    // walk those hyperedges. The stub is still useful as a launchable
    // floor-only world and as a round-trip anchor for tests that
    // exercise the registry surface.
    let mut out = String::new();
    out.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    out.push_str("<sdf version=\"1.8\">\n");
    out.push_str(&format!("  <world name=\"{}\">\n", xml_escape(world_name)));
    out.push_str("    <!-- TODO: delegate to formats::gazebo::generate_gazebo_world -->\n");
    out.push_str("    <!-- Robot model (\"");
    out.push_str(&xml_escape(robot_name));
    out.push_str("\") + sim_plugin / control_plugin tags populated by the full emitter -->\n");
    out.push_str(
        "    <plugin filename=\"gz-sim-physics-system\" name=\"gz::sim::systems::Physics\"/>\n",
    );
    out.push_str("    <plugin filename=\"gz-sim-user-commands-system\" name=\"gz::sim::systems::UserCommands\"/>\n");
    out.push_str("    <plugin filename=\"gz-sim-scene-broadcaster-system\" name=\"gz::sim::systems::SceneBroadcaster\"/>\n");
    out.push_str("  </world>\n");
    out.push_str("</sdf>\n");
    out
}

fn emit_sdf_stub(model: &KinematicModel, name: &str) -> String {
    let mut m = model.clone();
    m.name = name.to_string();
    crate::sdf::generate_sdf_from_model(&m)
}

fn emit_mjcf(model: &KinematicModel, config: &TransformConfig) -> String {
    let mut out = String::with_capacity(4096);
    out.push_str(&format!("<mujoco model=\"{}\">\n", config.robot_name));
    out.push_str("  <compiler angle=\"radian\" meshdir=\".\"/>\n");
    out.push_str("  <option gravity=\"0 0 -9.81\" timestep=\"0.001\"/>\n\n");

    // Assets
    out.push_str("  <asset>\n");
    for link in &model.links {
        if let Some(ref c) = link.color {
            out.push_str(&format!(
                "    <material name=\"mat_{}\" rgba=\"{} {} {} {}\"/>\n",
                link.name,
                c.get(0).unwrap_or(&0.5),
                c.get(1).unwrap_or(&0.5),
                c.get(2).unwrap_or(&0.5),
                c.get(3).unwrap_or(&1.0),
            ));
        }
    }
    out.push_str("  </asset>\n\n");

    // Build O(1)-lookup indices once, before tree walk. The
    // previous implementation did `model.{joints,links}.iter().find(...)`
    // and `.iter().filter(...)` inside the recursion, giving an
    // O(|J| + |L|) scan per recursion level and an empirically
    // measured ~O(s^1.25) overall (see paper §VI-F, MJCF row). With
    // these indices the per-link work becomes O(1) and the whole
    // walk is O(|L| + |J|).
    //
    // The body walk uses an explicit stack (not Rust recursion): long
    // serial chains (e.g. scaling fixtures with 200+ links) overflowed
    // the thread stack in `emit_mjcf_body`; cycles in bad joint graphs
    // are skipped via an active-link set.
    let ctx = MjcfCtx::build(model);

    out.push_str("  <worldbody>\n");
    // MuJoCo's <worldbody> *is* the world frame; emitting a `<body name="world">` collides
    // with the implicit worldbody ("repeated name 'world'"). Map a root `world` frame onto
    // it: emit its children as the top-level bodies (each child's incoming fixed-joint
    // origin is preserved by emit_mjcf_body_open). Non-world roots pass through unchanged.
    let roots: Vec<String> = find_roots(model)
        .into_iter()
        .flat_map(|r| {
            if r == "world" {
                model
                    .joints
                    .iter()
                    .filter(|j| j.parent_link == "world")
                    .map(|j| j.child_link.clone())
                    .collect::<Vec<_>>()
            } else {
                vec![r]
            }
        })
        .collect();
    emit_mjcf_body_stack(&mut out, &ctx, &roots);
    out.push_str("  </worldbody>\n\n");

    // Adjacent links meet at their shared joint, so their collision geoms overlap there; the
    // resulting self-contact applies a force that can pin the joint between them (measured on the
    // planar grasper: a shoulder joint frozen at 0 under a ±1.2 rad command, tracking error
    // 1.19 rad). Emit an explicit parent→child exclusion per joint so every consumer of the MJCF
    // gets a physically self-consistent scene. `world` is the implicit worldbody (not an emitted
    // body), so world-rooted joints have no parent body to exclude against and are skipped.
    let mut seen = std::collections::HashSet::new();
    let mut contact = String::new();
    for j in &model.joints {
        if j.parent_link == "world" {
            continue;
        }
        if seen.insert((j.parent_link.as_str(), j.child_link.as_str())) {
            contact.push_str(&format!(
                "    <exclude body1=\"{}\" body2=\"{}\"/>\n",
                j.parent_link, j.child_link
            ));
        }
    }
    if !contact.is_empty() {
        out.push_str("  <contact>\n");
        out.push_str(&contact);
        out.push_str("  </contact>\n\n");
    }

    out.push_str("  <actuator>\n");
    for j in &model.joints {
        if j.joint_type != JointType::Fixed {
            // Bound the torque to the joint's declared effort limit (N·m); an unlimited
            // joint leaves ctrlrange off (the consumer supplies its own default).
            let ctrl = j
                .limits
                .as_ref()
                .filter(|lim| lim.effort > 0.0)
                .map(|lim| format!(" ctrlrange=\"{:.4} {:.4}\"", -lim.effort, lim.effort))
                .unwrap_or_default();
            out.push_str(&format!(
                "    <motor name=\"act_{}\" joint=\"{}\" gear=\"1\"{}/>\n",
                j.name, j.name, ctrl
            ));
        }
    }
    out.push_str("  </actuator>\n");
    out.push_str("</mujoco>\n");
    out
}

/// Pre-built O(1) lookup tables for the MJCF body walk.
struct MjcfCtx<'a> {
    /// child_link → its single incoming joint (URDF/MJCF require tree
    /// topology, so each link has at most one).
    incoming: HashMap<&'a str, &'a JointInfo>,
    /// link_name → the link record.
    link: HashMap<&'a str, &'a LinkInfo>,
    /// parent_link → ordered list of (child_link, joint_index_in_model)
    /// child names; we keep a Vec so emission order matches the
    /// pre-fix implementation byte-for-byte.
    children: HashMap<&'a str, Vec<&'a str>>,
}

impl<'a> MjcfCtx<'a> {
    fn build(model: &'a KinematicModel) -> Self {
        let mut incoming = HashMap::with_capacity(model.joints.len());
        let mut children: HashMap<&'a str, Vec<&'a str>> =
            HashMap::with_capacity(model.joints.len());
        for j in &model.joints {
            // `incoming` keeps the first joint per child — mirrors the
            // `iter().find(...)` semantics of the pre-fix code on
            // (the well-formed) tree-topology inputs MJCF accepts.
            incoming.entry(j.child_link.as_str()).or_insert(j);
            children
                .entry(j.parent_link.as_str())
                .or_default()
                .push(j.child_link.as_str());
        }
        let mut link = HashMap::with_capacity(model.links.len());
        for l in &model.links {
            link.insert(l.name.as_str(), l);
        }
        Self {
            incoming,
            link,
            children,
        }
    }
}

fn emit_mjcf_body_open(out: &mut String, ctx: &MjcfCtx<'_>, link_name: &str, indent: usize) {
    let pad = " ".repeat(indent);
    let pad2 = " ".repeat(indent + 2);
    let incoming = ctx.incoming.get(link_name).copied();

    out.push_str(&format!("{}<body name=\"{}\"", pad, link_name));
    if let Some(j) = incoming {
        if let Some(xyz) = j.origin_xyz {
            out.push_str(&format!(" pos=\"{} {} {}\"", xyz[0], xyz[1], xyz[2]));
        }
        if let Some(rpy) = j.origin_rpy_rad() {
            if rpy.iter().any(|&v| v.abs() > 1e-9) {
                out.push_str(&format!(
                    " euler=\"{:.4} {:.4} {:.4}\"",
                    rpy[0], rpy[1], rpy[2]
                ));
            }
        }
    }
    out.push_str(">\n");

    if let Some(link) = ctx.link.get(link_name).copied() {
        // The link's geometry centroid (its `origin [x y z]` offset) — used for both the COM and
        // the geom pos, so the mass sits at the rod's centre and the geom spans from the joint.
        let centroid = link
            .origin
            .as_ref()
            .filter(|o| o.len() >= 3)
            .map(|o| format!("{} {} {}", o[0], o[1], o[2]))
            .unwrap_or_else(|| "0 0 0".to_string());
        if let Some(mass) = link.mass {
            let i = mass * 0.01;
            // <inertial> COM at the geometry centroid (not the joint), so a link's mass is centred
            // on its rod. A richer inertia tensor can refine the diaginertia later.
            out.push_str(&format!(
                "{}<inertial mass=\"{}\" pos=\"{}\" diaginertia=\"{} {} {}\"/>\n",
                pad2, mass, centroid, i, i, i
            ));
        }
        if let Some(ref geom) = link.geometry {
            // Explicit MuJoCo collision categories (additive): omitted entirely when None (historical default).
            let coll = geom
                .collision
                .map(|c| format!(" contype=\"{}\" conaffinity=\"{}\"", c.contype, c.conaffinity))
                .unwrap_or_default();
            let mat = link
                .color
                .as_ref()
                .map(|_| format!(" material=\"mat_{}\"", link.name))
                .unwrap_or_default();
            match geom.shape {
                // Capsule: MJCF uses `fromto` (the cylinder-axis endpoints), NOT pos/size. Dims = [length, radius].
                // The rod spans from the joint (0,0,0) to the far endpoint 2*origin (origin is the rod midpoint), so
                // a connected arm's capsule reaches from its joint to the next joint. Radius = dimensions[1].
                GeometryShape::Capsule if geom.dimensions.len() >= 2 => {
                    let (ex, ey, ez) = link
                        .origin
                        .as_ref()
                        .filter(|o| o.len() >= 3)
                        .map(|o| (o[0] * 2.0, o[1] * 2.0, o[2] * 2.0))
                        .unwrap_or((0.0, 0.0, 0.0));
                    out.push_str(&format!(
                        "{}<geom type=\"capsule\" fromto=\"0 0 0 {} {} {}\" size=\"{}\"{}{}/>  \n",
                        pad2, ex, ey, ez, geom.dimensions[1], mat, coll
                    ));
                }
                _ => {
                    let gs = match geom.shape {
                        GeometryShape::Box if geom.dimensions.len() >= 3 => {
                            let d = &geom.dimensions;
                            format!("type=\"box\" size=\"{} {} {}\"", d[0] / 2.0, d[1] / 2.0, d[2] / 2.0)
                        }
                        GeometryShape::Cylinder if geom.dimensions.len() >= 2 => {
                            let d = &geom.dimensions;
                            format!("type=\"cylinder\" size=\"{} {}\"", d[0], d[1] / 2.0)
                        }
                        GeometryShape::Sphere if !geom.dimensions.is_empty() => {
                            format!("type=\"sphere\" size=\"{}\"", geom.dimensions[0])
                        }
                        _ => String::new(),
                    };
                    if !gs.is_empty() {
                        // geom centred at the link's geometry centroid (the `origin` offset) — a rod from the joint.
                        out.push_str(&format!(
                            "{}<geom {} pos=\"{}\"{}{}/>  \n", pad2, gs, centroid, mat, coll));
                    }
                }
            }
        }
    }

    if let Some(j) = incoming {
        if j.joint_type != JointType::Fixed {
            let jtype = match j.joint_type {
                JointType::Revolute | JointType::Continuous => "hinge",
                JointType::Prismatic => "slide",
                JointType::Fixed => unreachable!(),
            };
            let ax = j.axis.unwrap_or([0.0, 0.0, 1.0]);
            let range = j
                .limits
                .as_ref()
                .map(|lim| {
                    let d2r = std::f64::consts::PI / 180.0;
                    format!(" range=\"{:.4} {:.4}\"", lim.lower * d2r, lim.upper * d2r)
                })
                .unwrap_or_default();
            out.push_str(&format!(
                "{}<joint name=\"{}\" type=\"{}\" axis=\"{} {} {}\"{}/>\n",
                pad2, j.name, jtype, ax[0], ax[1], ax[2], range
            ));
        }
    }
}

fn emit_mjcf_body_close(out: &mut String, indent: usize) {
    let pad = " ".repeat(indent);
    out.push_str(&format!("{}</body>\n", pad));
}

/// Depth-safe MJCF `<worldbody>` emission: explicit stack + cycle guard.
fn emit_mjcf_body_stack(out: &mut String, ctx: &MjcfCtx<'_>, roots: &[String]) {
    enum Op {
        Close { link: String, indent: usize },
        Open { link: String, indent: usize },
    }

    let mut active: HashSet<String> = HashSet::new();
    let mut stack: Vec<Op> = Vec::new();
    for root in roots.iter().rev() {
        stack.push(Op::Open {
            link: root.clone(),
            indent: 4,
        });
    }

    while let Some(op) = stack.pop() {
        match op {
            Op::Close { link, indent } => {
                emit_mjcf_body_close(out, indent);
                active.remove(&link);
            }
            Op::Open { link, indent } => {
                if !active.insert(link.clone()) {
                    continue;
                }
                emit_mjcf_body_open(out, ctx, link.as_str(), indent);
                stack.push(Op::Close {
                    link: link.clone(),
                    indent,
                });
                if let Some(kids) = ctx.children.get(link.as_str()) {
                    for child in kids.iter().rev() {
                        stack.push(Op::Open {
                            link: (*child).to_string(),
                            indent: indent + 2,
                        });
                    }
                }
            }
        }
    }
}

fn emit_dot(model: &KinematicModel, config: &TransformConfig) -> String {
    let mut out = String::with_capacity(2048);
    out.push_str(&format!("digraph \"{}\" {{\n", config.robot_name));
    out.push_str("  rankdir=TB;\n");
    out.push_str("  node [shape=box, style=rounded, fontname=\"sans-serif\", fontsize=10];\n");
    out.push_str("  edge [fontname=\"sans-serif\", fontsize=9];\n\n");

    for link in &model.links {
        let label = link.mass.map_or(link.name.clone(), |m| {
            format!("{}\\n{:.1} kg", link.name, m)
        });
        out.push_str(&format!("  \"{}\" [label=\"{}\"];\n", link.name, label));
    }
    out.push('\n');

    for j in &model.joints {
        let style = match j.joint_type {
            JointType::Fixed => "dashed",
            JointType::Revolute | JointType::Continuous => "bold",
            JointType::Prismatic => "dotted",
        };
        let axis_label = j
            .axis
            .map(|ax| {
                let letter = if ax[0].abs() > 0.5 {
                    "X"
                } else if ax[1].abs() > 0.5 {
                    "Y"
                } else {
                    "Z"
                };
                format!("{}\\n({})", j.name, letter)
            })
            .unwrap_or_else(|| j.name.clone());
        out.push_str(&format!(
            "  \"{}\" -> \"{}\" [label=\"{}\", style={}];\n",
            j.parent_link, j.child_link, axis_label, style
        ));
    }

    out.push_str("}\n");
    out
}

fn emit_mermaid(model: &KinematicModel, config: &TransformConfig) -> String {
    let mut out = String::with_capacity(2048);
    let _ = writeln!(
        out,
        "%% Generated by hymeko_formats::transforms::MermaidTransform"
    );
    let _ = writeln!(out, "%% Robot: {}", config.robot_name);
    let _ = writeln!(out, "flowchart TD");

    // Shared class definitions — links are rectangles, roots (world-style
    // frames that don't appear in model.links) are pill-shaped.
    let _ = writeln!(
        out,
        "    classDef link fill:#FFE4B5,stroke:#8B4513,stroke-width:2px,color:#000;"
    );
    let _ = writeln!(
        out,
        "    classDef root fill:#DDD,stroke:#555,stroke-width:2px,color:#000;"
    );
    let _ = writeln!(out);

    let link_ids: std::collections::HashSet<&str> =
        model.links.iter().map(|l| l.name.as_str()).collect();
    let roots = find_roots(model);

    // Emit explicit link nodes.
    for link in &model.links {
        let mass_suffix = link
            .mass
            .map(|m| format!("<br/>{:.2} kg", m))
            .unwrap_or_default();
        let _ = writeln!(
            out,
            "    {}[\"<b>{}</b>{mass_suffix}\"]:::link",
            mermaid_id(&link.name),
            escape_label(&link.name)
        );
    }

    // Emit root frames (world-style anchors that aren't `link`-typed in
    // the fixture — they appear only as joint parents).
    for r in &roots {
        if link_ids.contains(r.as_str()) {
            continue;
        }
        let _ = writeln!(
            out,
            "    {}([\"{}\"]):::root",
            mermaid_id(r),
            escape_label(r)
        );
    }
    let _ = writeln!(out);

    // Emit joint edges. Revolute / continuous arrows use the default
    // solid style; fixed joints are dashed (`-.->`); prismatic are
    // dotted (`-- dotted -->` via label trick).
    for j in &model.joints {
        let axis_letter = j.axis.map(|ax| {
            if ax[0].abs() > 0.5 {
                'X'
            } else if ax[1].abs() > 0.5 {
                'Y'
            } else {
                'Z'
            }
        });
        let label = match (j.joint_type, axis_letter) {
            (JointType::Fixed, _) => format!("{} (fixed)", j.name),
            (JointType::Revolute, Some(a)) => format!("{} (rev, {a})", j.name),
            (JointType::Continuous, Some(a)) => format!("{} (cont, {a})", j.name),
            (JointType::Prismatic, Some(a)) => format!("{} (prismatic, {a})", j.name),
            (JointType::Revolute, None) => format!("{} (rev)", j.name),
            (JointType::Continuous, None) => format!("{} (cont)", j.name),
            (JointType::Prismatic, None) => format!("{} (prismatic)", j.name),
        };
        let arrow = match j.joint_type {
            JointType::Fixed => "-.->|\"",
            _ => "-->|\"",
        };
        let _ = writeln!(
            out,
            "    {} {arrow}{}\"| {}",
            mermaid_id(&j.parent_link),
            escape_label(&label),
            mermaid_id(&j.child_link)
        );
    }

    out
}

/// Sanitise a HyMeKo identifier so it's a legal Mermaid node id.
/// Mermaid ids can have ASCII alnum + underscore; we replace anything
/// else with `_`.
fn mermaid_id(s: &str) -> String {
    s.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

/// Escape characters that would break a Mermaid `["label"]` literal.
fn escape_label(s: &str) -> String {
    s.replace('"', "&quot;").replace('|', "&vert;")
}

/// SysML 2 textual fallback emitter — mirrors `transforms/sysml/template.sysml`
/// but built up in Rust for direct API callers (registry-template path is
/// preferred, see `crate::codegen::generate_description`).
fn emit_sysml(model: &KinematicModel, config: &TransformConfig) -> String {
    let mut out = String::with_capacity(2048);
    let _ = writeln!(
        out,
        "// SysML 2 textual, generated by HyMeKo for {}",
        config.robot_name
    );
    let _ = writeln!(out);
    let _ = writeln!(out, "package {} {{", sysml_id(&config.robot_name));
    let _ = writeln!(out);
    let _ = writeln!(out, "    part def Link {{");
    let _ = writeln!(out, "        attribute mass : Real;");
    let _ = writeln!(out, "    }}");
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "    part def FixedJoint      {{ end parent : Link; end child : Link; }}"
    );
    let _ = writeln!(out, "    part def RevoluteJoint   {{ end parent : Link; end child : Link; attribute axis : Vector3; }}");
    let _ = writeln!(out, "    part def ContinuousJoint {{ end parent : Link; end child : Link; attribute axis : Vector3; }}");
    let _ = writeln!(out, "    part def PrismaticJoint  {{ end parent : Link; end child : Link; attribute axis : Vector3; }}");
    let _ = writeln!(out);

    // Link instances.
    for link in &model.links {
        let _ = writeln!(out, "    part {} : Link {{", sysml_id(&link.name));
        if let Some(m) = link.mass {
            let _ = writeln!(out, "        :>> mass = {};", m);
        }
        let _ = writeln!(out, "    }}");
    }
    let _ = writeln!(out);

    // Joint connections.
    for j in &model.joints {
        let conn_def = match j.joint_type {
            JointType::Fixed => "FixedJoint",
            JointType::Revolute => "RevoluteJoint",
            JointType::Continuous => "ContinuousJoint",
            JointType::Prismatic => "PrismaticJoint",
        };
        let _ = writeln!(
            out,
            "    connection {} : {} {{",
            sysml_id(&j.name),
            conn_def
        );
        let _ = writeln!(out, "        end ::> {};", sysml_id(&j.parent_link));
        let _ = writeln!(out, "        end ::> {};", sysml_id(&j.child_link));
        let _ = writeln!(out, "    }}");
    }
    let _ = writeln!(out, "}}");
    out
}

/// Sanitise a HyMeKo identifier so it's a legal SysML 2 name.
/// SysML 2 names accept ASCII alnum + underscore; we replace anything
/// else with `_` (same convention as `mermaid_id` above).
fn sysml_id(s: &str) -> String {
    s.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

// ─── Helpers ──────────────────────────────────────────────────────────────

fn find_roots(model: &KinematicModel) -> Vec<String> {
    // Find parent_link names that never appear as child_link.
    // This handles frames like "world" that aren't in model.links.
    let children: std::collections::HashSet<&str> =
        model.joints.iter().map(|j| j.child_link.as_str()).collect();
    let parents: std::collections::HashSet<&str> = model
        .joints
        .iter()
        .map(|j| j.parent_link.as_str())
        .collect();
    let mut roots: Vec<String> = parents
        .difference(&children)
        .map(|s| s.to_string())
        .collect();
    // Fallback: if no joints exist, use links not appearing as children
    if roots.is_empty() {
        roots = model
            .links
            .iter()
            .filter(|l| !children.contains(l.name.as_str()))
            .map(|l| l.name.clone())
            .collect();
    }
    roots
}

#[cfg(test)]
mod mjcf_emit_tests {
    use super::*;
    use hymeko::common::ids::DeclId;
    use hymeko_query::kinematics::joints::JointInfo;
    use hymeko_query::kinematics::kinematic::{GeometryInfo, GeometryShape, LinkInfo};

    fn deep_chain_model(n: usize) -> KinematicModel {
        assert!(n >= 2);
        let mut links = Vec::with_capacity(n);
        for i in 0..n {
            links.push(LinkInfo {
                did: DeclId::new(i),
                name: format!("L{i}"),
                mass: Some(1.0),
                geometry: Some(GeometryInfo {
                    shape: GeometryShape::Box,
                    dimensions: vec![0.05, 0.05, 0.05],
                    collision: None,
                }),
                origin: None,
                color: None,
            });
        }
        let mut joints = Vec::with_capacity(n - 1);
        for i in 0..(n - 1) {
            joints.push(JointInfo {
                did: DeclId::new(1000 + i),
                name: format!("j{i}"),
                joint_type: JointType::Fixed,
                parent_link: format!("L{i}"),
                child_link: format!("L{}", i + 1),
                axis: None,
                origin_xyz: None,
                origin_rpy_deg: None,
                limits: None,
            });
        }
        KinematicModel {
            name: "deep".to_string(),
            links,
            joints,
        }
    }

    /// Regression: recursive `emit_mjcf_body` overflowed the stack on
    /// long serial chains (scaling fixtures, `highArityFixedPool`).
    #[test]
    fn mjcf_emits_deep_serial_chain_without_stack_overflow() {
        let model = deep_chain_model(600);
        let cfg = hymeko_query::transforms::TransformConfig::default().with_name("deep");
        let xml = emit_mjcf(&model, &cfg);
        assert!(xml.contains("<mujoco"));
        assert!(xml.contains("</mujoco>"));
        let closes = xml.matches("</body>").count();
        assert_eq!(closes, 600, "expected one </body> per link");
    }

    /// Adjacent links overlap at their shared joint; the emitter must exclude each parent→child
    /// contact so the self-contact does not pin the joint (the planar-grasper frozen-shoulder bug).
    #[test]
    fn mjcf_emits_parent_child_contact_excludes() {
        let model = deep_chain_model(4); // L0→L1→L2→L3, 3 parent→child joints, no `world`
        let cfg = hymeko_query::transforms::TransformConfig::default().with_name("chain");
        let xml = emit_mjcf(&model, &cfg);
        assert!(xml.contains("<contact>"), "expected a <contact> block");
        assert!(xml.contains("<exclude body1=\"L0\" body2=\"L1\"/>"));
        assert!(xml.contains("<exclude body1=\"L2\" body2=\"L3\"/>"));
        assert_eq!(xml.matches("<exclude").count(), 3, "one exclude per parent→child joint");
    }

    /// `world` is the implicit worldbody, not an emitted body: world-rooted joints must NOT emit an
    /// exclude (it would name a nonexistent body and fail to load).
    #[test]
    fn mjcf_skips_world_rooted_contact_exclude() {
        let mut model = deep_chain_model(3); // L0→L1→L2
        // Re-root the chain at `world`: world → L0.
        model.joints.insert(
            0,
            JointInfo {
                did: DeclId::new(2000),
                name: "fix_world".to_string(),
                joint_type: JointType::Fixed,
                parent_link: "world".to_string(),
                child_link: "L0".to_string(),
                axis: None,
                origin_xyz: Some([0.0, 0.0, 0.0]),
                origin_rpy_deg: None,
                limits: None,
            },
        );
        let cfg = hymeko_query::transforms::TransformConfig::default().with_name("chain");
        let xml = emit_mjcf(&model, &cfg);
        assert!(!xml.contains("body1=\"world\""), "world must not appear in an exclude");
        assert!(xml.contains("<exclude body1=\"L0\" body2=\"L1\"/>"));
    }
}
