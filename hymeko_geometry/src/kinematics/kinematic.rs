//! Kinematic model extraction from query results.
//!
//! This is the shared intermediate that URDF, SDF, Gazebo, and
//! Isaac Sim transforms all consume. It navigates the IR using
//! DeclIds from query results, extracting links, joints, topology,
//! geometry, axes, and origin transforms.

use crate::kinematics::joints::{JointInfo, JointLimits, JointType};
use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{Ir, ValueR};
use hymeko_query::traits::NameResolver;
use hymeko_query::{Predicate, QueryEngine};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeometryShape {
    Box,
    Cylinder,
    Sphere,
    /// A capsule (a cylinder capped by two hemispheres). Dimensional contract (2026-07-23, Galambos planar
    /// extension): `dimensions[0]` = the **rod/segment length** (the cylinder-axis extent, i.e. the `fromto`
    /// endpoint distance used by the golden MuJoCo builder — NOT including the hemispherical caps, which the
    /// emitter adds as `size` beyond each endpoint); `dimensions[1]` = the capsule **radius**.
    Capsule,
}

/// Explicit collision categories for a geometry (additive, 2026-07-23). Maps directly to MuJoCo
/// `contype` / `conaffinity`. `None` (no `collision {}` sub-node) preserves the historical behaviour exactly
/// (the emitter omits both attributes → MuJoCo defaults).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CollisionMask {
    pub contype: i64,
    pub conaffinity: i64,
}

#[derive(Debug, Clone)]
pub struct GeometryInfo {
    pub shape: GeometryShape,
    pub dimensions: Vec<f64>,
    /// Optional explicit collision mask; `None` ⇒ historical default behaviour (emitter omits contype/conaffinity).
    pub collision: Option<CollisionMask>,
}

#[derive(Debug, Clone)]
pub struct LinkInfo {
    pub did: DeclId,
    pub name: String,
    pub mass: Option<f64>,
    pub geometry: Option<GeometryInfo>,
    pub origin: Option<Vec<f64>>,
    pub color: Option<Vec<f64>>,
    /// COM inertia in the link axes, kg m^2: xx, yy, zz, xy, xz, yz.
    /// Absent retains the historical point-mass numerical model.
    pub inertia_com: Option<[f64; 6]>,
}

#[derive(Debug, Clone)]
pub struct KinematicModel {
    pub name: String,
    pub links: Vec<LinkInfo>,
    pub joints: Vec<JointInfo>,
    /// Declared `frame` elements, by name.
    ///
    /// A frame is not a body: it carries no mass and no geometry, and nothing about it is posed. It
    /// exists to be joined TO — `world` is the frame a floating base hangs from — so a joint whose
    /// parent is a frame anchors its child to something outside the mechanism rather than to
    /// another part of it. Kinematics has to be able to tell that apart from a joint that names a
    /// link which does not exist, and the only place that distinction is available is here.
    pub frames: Vec<String>,
    /// Geometric force transmissions over the existing links and their DeclIds.
    pub actuation: super::actuation::Actuation,
}

/// Extract a complete kinematic model from the IR.
pub fn extract_kinematic_model<R: NameResolver>(
    engine: &QueryEngine<'_, R>,
    robot_name: &str,
) -> KinematicModel {
    let ir = engine.ir();
    let res = engine.resolver();

    // Query links
    let link_matches = engine.query(&Predicate::node().and(Predicate::inherits("link")));

    let links: Vec<LinkInfo> = link_matches
        .iter()
        .map(|m| {
            let mass = find_child_num(ir, res, m.id, "mass");
            let geometry = extract_geometry(ir, res, m.id);
            let origin = find_child_list(ir, res, m.id, "origin");
            let color = find_child_ref_value_list(ir, res, m.id, "color");
            let inertia_com = ir
                .decl_children(m.id)
                .find(|c| res.resolve(ir.decl_nodes[c.raw()].name) == "inertia_com")
                .map(|c| {
                    ["xx", "yy", "zz", "xy", "xz", "yz"]
                        .map(|key| find_child_num(ir, res, c, key).unwrap_or(f64::NAN))
                });

            LinkInfo {
                did: m.id,
                name: m.name.clone(),
                mass,
                geometry,
                origin,
                color,
                inertia_com,
            }
        })
        .collect();

    let frames: Vec<String> = engine
        .query(&Predicate::node().and(Predicate::inherits("frame")))
        .iter()
        .map(|m| m.name.clone())
        .collect();

    // Query joints by type
    let mut joints = Vec::new();
    let joint_types = [
        ("fixed_joint", JointType::Fixed),
        ("conti_joint", JointType::Continuous),
        ("rev_joint", JointType::Revolute),
        ("prismatic_joint", JointType::Prismatic),
    ];

    for (base_name, jtype) in &joint_types {
        let results = engine.query(&Predicate::edge().and(Predicate::inherits(base_name)));

        for m in &results {
            // Read directly from bindings — no re-query needed
            let mut parent_link = String::from("unknown");
            let mut child_link = String::from("unknown");
            // The arc binding already carries `target: DeclId` — the arena handle — and its own
            // doc says domain transforms read it directly, zero re-traversal. Taking the name and
            // dropping the handle is the copy this pair exists to stop.
            let mut parent = DeclId::NONE;
            let mut child = DeclId::NONE;
            let mut axis: Option<[f64; 3]> = None;
            let mut origin_xyz: Option<[f64; 3]> = None;
            let mut origin_rpy: Option<[f64; 3]> = None;

            for b in &m.arc_bindings {
                let is_link = check_inherits_simple(ir, res, b.target, "link", 4)
                    || check_inherits_simple(ir, res, b.target, "frame", 4);
                let is_axis = check_inherits_simple(ir, res, b.target, "axis_definition", 4);

                if b.sign == 1 && is_link {
                    // Parent link (+ sign, inherits from link)
                    parent = b.target;
                    parent_link = b.target_name.clone();

                    // Extract origin from weight annotations on this binding
                    if let Some(ref weights) = b.weights {
                        if let Some(first) = weights.first() {
                            origin_xyz = extract_3vec(first);
                        }
                        if weights.len() >= 2 {
                            origin_rpy = extract_3vec(&weights[1]);
                        }
                    }
                } else if b.sign == -1 && is_link {
                    // Child link (- sign, inherits from link)
                    child = b.target;
                    child_link = b.target_name.clone();
                } else if b.sign == -1 && is_axis {
                    // Axis (- sign, inherits from axis_definition)
                    let ax_vals = find_child_list(ir, res, b.target, "ax");
                    if let Some(v) = ax_vals {
                        if v.len() >= 3 {
                            axis = Some([v[0], v[1], v[2]]);
                        }
                    }
                }
            }

            joints.push(JointInfo {
                did: m.id,
                name: m.name.clone(),
                joint_type: *jtype,
                parent,
                child,
                parent_link,
                child_link,
                axis,
                origin_xyz,
                origin_rpy_deg: origin_rpy,
                limits: extract_joint_limits(ir, res, m.id),
            });
        }
    }

    let actuation = super::actuation::Actuation::extract(engine, robot_name, &links, &joints);
    KinematicModel {
        name: robot_name.to_string(),
        links,
        joints,
        frames,
        actuation,
    }
}

// ============================================================
// IR navigation helpers
// ============================================================

/// Find a direct child node by name and return its numeric value.
fn find_child_num<R: NameResolver>(ir: &Ir, res: &R, did: DeclId, name: &str) -> Option<f64> {
    ir.decl_children(did).find_map(|cid| {
        let child = &ir.decl_nodes[cid.raw()];
        if res.resolve(child.name) == name {
            match &child.anno.value {
                Some(ValueR::Num(v)) => Some(*v),
                _ => None,
            }
        } else {
            None
        }
    })
}

/// Find a direct child node by name and return its list value as Vec<f64>.
fn find_child_list<R: NameResolver>(ir: &Ir, res: &R, did: DeclId, name: &str) -> Option<Vec<f64>> {
    ir.decl_children(did).find_map(|cid| {
        let child = &ir.decl_nodes[cid.raw()];
        if res.resolve(child.name) == name {
            extract_num_list(&child.anno.value)
        } else {
            None
        }
    })
}

/// Find a child that has a Ref value (like `color -> body_color;`),
/// follow the ref to the target, and extract its value as Vec<f64>.
fn find_child_ref_value_list<R: NameResolver>(
    ir: &Ir,
    res: &R,
    did: DeclId,
    child_name: &str,
) -> Option<Vec<f64>> {
    ir.decl_children(did).find_map(|cid| {
        let child = &ir.decl_nodes[cid.raw()];
        if res.resolve(child.name) != child_name {
            return None;
        }
        match &child.anno.value {
            Some(ValueR::Ref(target_did)) => {
                if target_did.is_none() {
                    return None;
                }
                let target = &ir.decl_nodes[target_did.raw()];
                extract_num_list(&target.anno.value)
            }
            _ => None,
        }
    })
}

fn find_child_ref_target(ir: &Ir, did: DeclId) -> Option<DeclId> {
    ir.decl_children(did).find_map(|cid| {
        let child = &ir.decl_nodes[cid.raw()];
        match &child.anno.value {
            Some(ValueR::Ref(target_did)) => Some(*target_did),
            _ => None,
        }
    })
}

fn extract_num_list(val: &Option<ValueR>) -> Option<Vec<f64>> {
    match val {
        Some(ValueR::List(items)) => {
            let nums: Vec<f64> = items
                .iter()
                .filter_map(|v| match v {
                    ValueR::Num(n) => Some(*n),
                    _ => None,
                })
                .collect();
            if nums.is_empty() { None } else { Some(nums) }
        }
        _ => None,
    }
}
fn extract_value_list(ir: &Ir, did: DeclId) -> Option<Vec<f64>> {
    let decl = &ir.decl_nodes[did.raw()];
    extract_num_list(&decl.anno.value)
}

/// Extract geometry from a child named `link_geometry` that inherits
/// from box/cylinder/sphere.
fn extract_geometry<R: NameResolver>(ir: &Ir, res: &R, link_did: DeclId) -> Option<GeometryInfo> {
    ir.decl_children(link_did)
        .find(|cid| res.resolve(ir.decl_nodes[cid.raw()].name) == "link_geometry")
        .and_then(|cid| geometry_decl(ir, res, cid).unwrap_or_else(|message| panic!("{message}")))
}

pub(super) fn geometry_decl<R: NameResolver>(
    ir: &Ir,
    res: &R,
    cid: DeclId,
) -> Result<Option<GeometryInfo>, String> {
    if let Some(nid) = ir.as_node(cid) {
        let mut unknown_shape: Option<String> = None;
        for base in &ir.nodes[nid.raw()].bases {
            let target = base.target();
            if target.is_none() {
                continue;
            }
            let base_name = res.resolve(ir.decl_nodes[target.raw()].name);
            let shape = match base_name {
                "box" => Some(GeometryShape::Box),
                "cylinder" => Some(GeometryShape::Cylinder),
                "sphere" => Some(GeometryShape::Sphere),
                "capsule" => Some(GeometryShape::Capsule),
                other => {
                    unknown_shape = Some(other.to_string());
                    None
                }
            };
            if let Some(shape) = shape {
                let dims = find_child_list(ir, res, cid, "dimension").unwrap_or_default();
                let collision = extract_collision(ir, res, cid);
                return Ok(Some(GeometryInfo {
                    shape,
                    dimensions: dims,
                    collision,
                }));
            }
        }
        // An EXPLICITLY supplied but unknown geometry shape must hard-fail (not silently disappear).
        // Missing geometry (no shape base at all) remains allowed and returns None below.
        if let Some(bad) = unknown_shape {
            return Err(format!(
                "unknown geometry shape {bad:?} in `link_geometry` (decl {cid:?}); \
                     expected one of box / cylinder / sphere / capsule"
            ));
        }
    }
    Ok(None)
}

/// Read an optional `collision {{ contype ...; conaffinity ...; }}` sub-node of a `link_geometry` node.
/// Absence ⇒ `None` (historical default). Both fields are required together when the block is present.
fn extract_collision<R: NameResolver>(ir: &Ir, res: &R, geom_did: DeclId) -> Option<CollisionMask> {
    ir.decl_children(geom_did).find_map(|cid| {
        if res.resolve(ir.decl_nodes[cid.raw()].name) != "collision" {
            return None;
        }
        let contype = find_child_num(ir, res, cid, "contype")? as i64;
        let conaffinity = find_child_num(ir, res, cid, "conaffinity")? as i64;
        Some(CollisionMask {
            contype,
            conaffinity,
        })
    })
}

fn extract_3vec_from_value(val: Option<&ValueR>) -> Option<[f64; 3]> {
    match val? {
        ValueR::List(items) if items.len() >= 3 => {
            let mut arr = [0.0f64; 3];
            for (i, item) in items.iter().take(3).enumerate() {
                if let ValueR::Num(n) = item {
                    arr[i] = *n;
                }
            }
            Some(arr)
        }
        _ => None,
    }
}

fn extract_3vec(val: &ValueR) -> Option<[f64; 3]> {
    match val {
        ValueR::List(items) if items.len() >= 3 => {
            let mut arr = [0.0f64; 3];
            for (i, item) in items.iter().take(3).enumerate() {
                if let ValueR::Num(n) = item {
                    arr[i] = *n;
                }
            }
            Some(arr)
        }
        _ => None,
    }
}

/// Simple transitive inheritance check (standalone, not requiring QueryEngine).
fn check_inherits_simple<R: NameResolver>(
    ir: &Ir,
    res: &R,
    did: DeclId,
    base_name: &str,
    depth: usize,
) -> bool {
    if depth == 0 || did.is_none() {
        return false;
    }

    if let Some(nid) = ir.as_node(did) {
        for base_ref in &ir.nodes[nid.raw()].bases {
            let target = base_ref.target();
            if target.is_none() {
                continue;
            }
            let tname = res.resolve(ir.decl_nodes[target.raw()].name);
            if tname == base_name {
                return true;
            }
            if check_inherits_simple(ir, res, target, base_name, depth - 1) {
                return true;
            }
        }
    }
    if let Some(eid) = ir.as_edge(did) {
        for base_ref in &ir.edges[eid.raw()].bases {
            let target = base_ref.target();
            if target.is_none() {
                continue;
            }
            let tname = res.resolve(ir.decl_nodes[target.raw()].name);
            if tname == base_name {
                return true;
            }
            if check_inherits_simple(ir, res, target, base_name, depth - 1) {
                return true;
            }
        }
    }

    false
}

/// Find a direct child named `name` whose value is a resolved `Ref`, returning the ref
/// target; if absent on `did`, walk `did`'s bases (node and edge) up to `depth` — a joint
/// declares its `limit` directly (`@j0: rev_joint { limit -> joint0_limit; }`) **or**
/// inherits it from `rev_joint`/`prismatic_joint`. Unresolved refs are skipped.
///
/// # Preconditions
/// `depth` bounds the inheritance walk (cycle/overflow guard).
/// # Postconditions
/// Returns `Some(target)` for the nearest `name`-named ref (own children before bases),
/// else `None`.
fn find_inherited_ref_target<R: NameResolver>(
    ir: &Ir,
    res: &R,
    did: DeclId,
    name: &str,
    depth: usize,
) -> Option<DeclId> {
    if did.is_none() || depth == 0 {
        return None;
    }
    for cid in ir.decl_children(did) {
        let child = &ir.decl_nodes[cid.raw()];
        if res.resolve(child.name) == name {
            if let Some(ValueR::Ref(target)) = &child.anno.value {
                if !target.is_none() {
                    return Some(*target);
                }
            }
        }
    }
    if let Some(nid) = ir.as_node(did) {
        for base in &ir.nodes[nid.raw()].bases {
            if let Some(t) = find_inherited_ref_target(ir, res, base.target(), name, depth - 1) {
                return Some(t);
            }
        }
    }
    if let Some(eid) = ir.as_edge(did) {
        for base in &ir.edges[eid.raw()].bases {
            if let Some(t) = find_inherited_ref_target(ir, res, base.target(), name, depth - 1) {
                return Some(t);
            }
        }
    }
    None
}

fn extract_joint_limits<R: NameResolver>(ir: &Ir, res: &R, did: DeclId) -> Option<JointLimits> {
    // Legacy form: `limit_lower`/`limit_upper`/... as direct children of the joint.
    let mut lower = find_child_num(ir, res, did, "limit_lower");
    let mut upper = find_child_num(ir, res, did, "limit_upper");
    let mut effort = find_child_num(ir, res, did, "limit_effort");
    let mut velocity = find_child_num(ir, res, did, "limit_velocity");

    // Referenced form: `limit -> joint_rev_limit { lower; upper; effort; velocity; }`,
    // declared on the joint or inherited from its base joint type. Follow the `limit` ref
    // and read the fields off the target node (named without the `limit_` prefix). Without
    // this the emitted joints are unlimited and the actuators carry no ctrlrange.
    if lower.is_none() && upper.is_none() {
        if let Some(limit_did) = find_inherited_ref_target(ir, res, did, "limit", 4) {
            lower = find_child_num(ir, res, limit_did, "lower");
            upper = find_child_num(ir, res, limit_did, "upper");
            effort = find_child_num(ir, res, limit_did, "effort");
            velocity = find_child_num(ir, res, limit_did, "velocity");
        }
    }

    // Only produce limits if at least lower+upper are present
    match (lower, upper) {
        (Some(lo), Some(hi)) => Some(JointLimits {
            lower: lo,
            upper: hi,
            effort: effort.unwrap_or(0.0),
            velocity: velocity.unwrap_or(0.0),
        }),
        _ => None,
    }
}

impl LinkInfo {
    /// A physical inertia has positive principal moments and satisfies their triangle inequalities.
    pub fn valid_inertia(&self) -> bool {
        let Some([xx, yy, zz, xy, xz, yz]) = self.inertia_com else {
            return true;
        };
        if ![xx, yy, zz, xy, xz, yz].iter().all(|x| x.is_finite())
            || self.mass.is_none_or(|m| !m.is_finite() || m <= 0.0)
        {
            return false;
        }
        let determinant =
            xx * yy * zz + 2.0 * xy * xz * yz - xx * yz * yz - yy * xz * xz - zz * xy * xy;
        if xx <= 0.0 || xx * yy <= xy * xy || determinant <= 0.0 {
            return false;
        }
        // S = trace(I)/2 E - I is the second moment matrix; it must be positive semidefinite.
        let [a, b, c] = [
            (yy + zz - xx) / 2.0,
            (xx + zz - yy) / 2.0,
            (xx + yy - zz) / 2.0,
        ];
        let eps = 1e-12 * (xx + yy + zz).powi(3);
        a >= 0.0
            && b >= 0.0
            && c >= 0.0
            && a * b >= xy * xy
            && a * c >= xz * xz
            && b * c >= yz * yz
            && a * b * c - 2.0 * xy * xz * yz - a * yz * yz - b * xz * xz - c * xy * xy >= -eps
    }
}
