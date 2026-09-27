//! WebXR semantic validation over the canonical `Ir`.
//!
//! This is the production validator the WebXR authoring path relies on. HyMeKo has no schema
//! validation — a green compile proves parse+resolve, not meaning — so this reads the canonical
//! [`hymeko::ir::ir::Ir`] back and enforces the WebXR spatial contract. It is **semantic only**: it
//! never materialises a scene DTO or a second scene graph, holds no runtime XR handle, and produces
//! no rendering state.
//!
//! # Contract
//! - Works over `&Ir + &Interner`, through the [`hymeko::ir::access`] accessor boundary (never raw
//!   `ir.nodes`/`ir.edges`/`ir.arcs` table indexing).
//! - Linear: `O(declarations + incidences)` — one classification pass buckets relation/kind instances,
//!   the checks consume the buckets, and no per-entity temporary allocation is made (a single reusable
//!   participant buffer is threaded through). Every inheritance/parent walk is cycle-safe (a depth
//!   budget), because the `Ir` does not *guarantee* an acyclic containment/relation graph.
//! - Errors are ids-only ([`DeclId`]/[`SymId`]), one variant per validation family, deterministic; no
//!   panic/`unwrap` on the validation path.
//!
//! The canonical / runtime boundary is a documented contract: authoring pose, placement, affordance,
//! capability *intent* and asset references live in the `Ir`; `XRSession`/`XRFrame`/`XRSpace`/
//! `XRAnchor`, GL/DOM resources, controller button indices and per-frame poses do not. This module
//! enforces the "no runtime-handle field" half of that boundary (see [`SemanticError::RuntimeHandleField`]).

use std::collections::{HashMap, HashSet};

use hymeko::common::ids::{DeclId, SymId};
use hymeko::ir::ir::{DeclKind, Ir, ValueR};
use hymeko::resolution::interner::Interner;

pub mod query;
pub mod room;
pub use query::{
    CameraFrame, FiducialBinding, FiducialKind, FrameParent, Placement, Pose, SpatialFrame,
    SpatialQuery, SpatialQueryError,
};
pub use room::{ArucoMarker, Room, RoomQuery, TrackedDevice};

/// Vocabulary sources. HyMeKo imports resolve beside the importing file, so room models carry
/// copies of these; the copies are checked against these texts, which remain the originals.
pub const META_SPATIAL: &str = include_str!("../data/webxr/meta_spatial.hymeko");
pub const META_ROOM: &str = include_str!("../data/webxr/meta_room.hymeko");

/// The tolerance below which a quaternion's norm is treated as un-normalizable (zero quaternion).
const QUAT_NORM_EPSILON: f64 = 1.0e-6;

/// Maximum inheritance / parent depth walked. Real chains are a handful deep; this bounds a walk (so
/// a cycle terminates) without a per-call heap allocation.
const MAX_WALK_DEPTH: u32 = 1024;

/// A WebXR semantic-contract violation. Ids-only, deterministic, one variant per family.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SemanticError {
    /// The webxr / meta_spatial vocabulary was not imported (kinds not resolvable).
    VocabularyMissing,
    /// The scene selects zero or more than one session mode.
    SessionModeCardinality { scene: DeclId, found: u32 },
    /// The scene selects zero or more than one primary reference space.
    ReferenceSpaceCardinality { scene: DeclId, found: u32 },
    /// A capability is requested both required and optional.
    CapabilityRequiredOptionalOverlap { capability: DeclId },
    /// A capability request names a decl that is not a known capability.
    UnknownCapability { request: DeclId, target: DeclId },
    /// A capability request's `requirement` attribute is missing or not `required`/`optional`.
    BadCapabilityRequirement { request: DeclId },
    /// A placement's frame participant is missing or is not a spatial frame.
    FrameMissingOrNotAFrame { placement: DeclId, frame: DeclId },
    /// The `@parent_frame` relation contains a cycle reached at `frame`.
    FrameCycle { frame: DeclId },
    /// A pose attribute has the wrong number of components.
    PoseBadLength {
        relation: DeclId,
        attr: SymId,
        expected: u32,
        found: u32,
    },
    /// A pose attribute component is non-finite (NaN/Inf) or not a number.
    PoseNonFinite {
        relation: DeclId,
        attr: SymId,
        index: u32,
    },
    /// A quaternion is zero / not normalizable within tolerance.
    QuaternionNotNormalizable { relation: DeclId },
    /// A scale component is not strictly positive.
    ScaleNonPositive { relation: DeclId, index: u32 },
    /// A relation's participant roles are the wrong cardinality/kind.
    RelationRole {
        relation: DeclId,
        archetype: SymId,
        reason: RoleReason,
    },
    /// An interaction names a decl that is not a known action.
    UnknownAction { interaction: DeclId, target: DeclId },
    /// A visual asset's `uri` attribute is missing or not a string.
    AssetUriNotString { asset: DeclId },
    /// A relation incidence targets a declaration that does not exist.
    DanglingReference { relation: DeclId, target: DeclId },
    /// A declaration carries an attribute whose name is reserved for a runtime WebXR handle.
    RuntimeHandleField { decl: DeclId, field: SymId },
}

/// Why a relation's roles are invalid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RoleReason {
    /// Expected exactly one subject (`+`) of the required kind; found a different count.
    SubjectCount { found: u32 },
    /// A participant is not of the kind its role requires.
    WrongKind { participant: DeclId },
    /// Expected at least one target object; found none.
    NoTarget,
}

impl std::fmt::Display for SemanticError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Ids-only rendering; no attacker-controlled source text is embedded.
        write!(f, "{self:?}")
    }
}

impl std::error::Error for SemanticError {}

/// Validate a compiled scene against the WebXR spatial contract. Returns every violation found
/// (empty == valid). Reads only `ir` and `it`, through the accessor boundary; linear and cycle-safe.
#[must_use]
pub fn validate(ir: &Ir, it: &Interner) -> Vec<SemanticError> {
    let mut out = Vec::new();
    let reg = match Registry::resolve(ir, it) {
        Some(r) => r,
        None => {
            out.push(SemanticError::VocabularyMissing);
            return out;
        }
    };
    let b = Buckets::collect(ir, &reg);
    let mut buf: Vec<(i8, DeclId)> = Vec::new(); // one reusable participant buffer for every relation
    check_session(ir, &reg, &b, &mut buf, &mut out);
    check_capabilities(ir, it, &reg, &b, &mut buf, &mut out);
    check_frames_and_placements(ir, it, &reg, &b, &mut buf, &mut out);
    check_interactions_and_anchoring(ir, &reg, &b, &mut buf, &mut out);
    check_assets(ir, &reg, &b, &mut out);
    check_runtime_handle_fields(ir, &reg, &mut out);
    out
}

// ---------------------------------------------------------------------------------------------
// vocabulary registry (resolved once, via accessors)
// ---------------------------------------------------------------------------------------------

struct Registry {
    scene: DeclId,
    spatial_frame: DeclId,
    spatial_entity: DeclId,
    visual_asset: DeclId,
    semantic_anchor: DeclId,
    action: DeclId,
    placement: DeclId,
    parent_frame: DeclId,
    anchoring: DeclId,
    interaction: DeclId,
    capability_request: DeclId,
    session_config: DeclId,
    session_modes: Vec<DeclId>,
    reference_spaces: Vec<DeclId>,
    capabilities: Vec<DeclId>,
    actions: Vec<DeclId>,
    sym_position: Option<SymId>,
    sym_orientation: Option<SymId>,
    sym_scale: Option<SymId>,
    sym_requirement: Option<SymId>,
    sym_uri: Option<SymId>,
    runtime_handle_fields: Vec<SymId>,
}

impl Registry {
    fn resolve(ir: &Ir, it: &Interner) -> Option<Self> {
        let spatial = find_root(ir, it, "spatial")?;
        let kinds = child_named(ir, it, spatial, "kinds")?;
        let relations = child_named(ir, it, spatial, "relations")?;
        let webxr = find_root(ir, it, "webxr")?;
        let session = child_named(ir, it, webxr, "session")?;
        let k = |n: &str| child_named(ir, it, kinds, n);
        let r = |n: &str| child_named(ir, it, relations, n);
        let container = |n: &str| child_named(ir, it, webxr, n);
        Some(Registry {
            scene: k("scene")?,
            spatial_frame: k("spatial_frame")?,
            spatial_entity: k("spatial_entity")?,
            visual_asset: k("visual_asset")?,
            semantic_anchor: k("semantic_anchor")?,
            action: k("action")?,
            placement: r("placement")?,
            parent_frame: r("parent_frame")?,
            anchoring: r("anchoring")?,
            interaction: r("interaction")?,
            capability_request: child_named(ir, it, session, "capability_request")?,
            session_config: child_named(ir, it, session, "session_config")?,
            session_modes: children(ir, container("session_modes")?),
            reference_spaces: children(ir, container("reference_spaces")?),
            capabilities: children(ir, container("capabilities")?),
            actions: children(ir, container("actions")?),
            sym_position: it.get_id("position"),
            sym_orientation: it.get_id("orientation"),
            sym_scale: it.get_id("scale"),
            sym_requirement: it.get_id("requirement"),
            sym_uri: it.get_id("uri"),
            runtime_handle_fields: RUNTIME_HANDLE_NAMES
                .iter()
                .filter_map(|n| it.get_id(n))
                .collect(),
        })
    }

    fn is_kind_in(&self, ir: &Ir, d: DeclId, set: &[DeclId]) -> bool {
        set.iter().any(|&k| isa_transitive(ir, d, k))
    }

    fn name(&self, ir: &Ir, d: DeclId) -> SymId {
        ir.decl_node(d).map(|n| n.name).unwrap_or(SymId::NONE)
    }
}

/// One classification pass: every scene / relation-instance / visual-asset decl, bucketed by its
/// `<isa>` archetype. `O(declarations)` with a short bounded isa-walk each; buckets allocate once.
#[derive(Default)]
struct Buckets {
    scenes: Vec<DeclId>,
    session_configs: Vec<DeclId>,
    capability_requests: Vec<DeclId>,
    placements: Vec<DeclId>,
    parent_frames: Vec<DeclId>,
    interactions: Vec<DeclId>,
    anchorings: Vec<DeclId>,
    visual_assets: Vec<DeclId>,
}

impl Buckets {
    fn collect(ir: &Ir, reg: &Registry) -> Self {
        let mut b = Buckets::default();
        for (d, node) in ir.decls() {
            match node.kind {
                DeclKind::Edge | DeclKind::HyperArc => {
                    if d == reg.placement
                        || d == reg.parent_frame
                        || d == reg.anchoring
                        || d == reg.interaction
                        || d == reg.capability_request
                        || d == reg.session_config
                    {
                        continue; // the archetype itself is not an instance
                    }
                    if isa_transitive(ir, d, reg.placement) {
                        b.placements.push(d);
                    } else if isa_transitive(ir, d, reg.parent_frame) {
                        b.parent_frames.push(d);
                    } else if isa_transitive(ir, d, reg.interaction) {
                        b.interactions.push(d);
                    } else if isa_transitive(ir, d, reg.anchoring) {
                        b.anchorings.push(d);
                    } else if isa_transitive(ir, d, reg.capability_request) {
                        b.capability_requests.push(d);
                    } else if isa_transitive(ir, d, reg.session_config) {
                        b.session_configs.push(d);
                    }
                }
                DeclKind::Node => {
                    if d == reg.scene || d == reg.visual_asset {
                        continue;
                    }
                    if isa_transitive(ir, d, reg.scene) {
                        b.scenes.push(d);
                    } else if isa_transitive(ir, d, reg.visual_asset) {
                        b.visual_assets.push(d);
                    }
                }
            }
        }
        b
    }
}

/// Attribute names reserved for runtime WebXR handles — a source scene must never carry them.
const RUNTIME_HANDLE_NAMES: &[&str] = &[
    "xrsession",
    "xrframe",
    "xrspace",
    "xranchor",
    "session_object",
    "gl_resource",
    "webgl_texture",
    "dom_node",
    "button_index",
    "viewer_pose",
    "permission_state",
    "tracking_confidence",
];

// ---------------------------------------------------------------------------------------------
// accessor helpers (never index ir.nodes/ir.edges/ir.arcs directly)
// ---------------------------------------------------------------------------------------------

fn find_root(ir: &Ir, it: &Interner, name: &str) -> Option<DeclId> {
    let sym = it.get_id(name)?;
    ir.decls()
        .find(|(_, node)| node.parent.is_none() && node.name == sym)
        .map(|(d, _)| d)
}

fn child_named(ir: &Ir, it: &Interner, parent: DeclId, name: &str) -> Option<DeclId> {
    let sym = it.get_id(name)?;
    ir.child_decls(parent)
        .find(|&c| ir.decl_node(c).map(|n| n.name == sym).unwrap_or(false))
}

fn children(ir: &Ir, parent: DeclId) -> Vec<DeclId> {
    ir.child_decls(parent).collect()
}

/// Invoke `f` for each direct inheritance base of `d`, borrowing (no allocation).
#[inline]
fn each_base(ir: &Ir, d: DeclId, f: &mut impl FnMut(DeclId)) {
    if let Some((_, node)) = ir.node_for_decl(d) {
        for base in &node.bases {
            f(base.target());
        }
        return;
    }
    if let Some((_, edge)) = ir.edge_for_decl(d) {
        for base in &edge.bases {
            f(base.target());
        }
    }
}

/// Transitive `<isa>`: does `d` reach `target` through inheritance bases? Allocation-free and
/// depth-bounded ([`MAX_WALK_DEPTH`]), so an inheritance cycle terminates instead of looping.
fn isa_transitive(ir: &Ir, d: DeclId, target: DeclId) -> bool {
    fn go(ir: &Ir, d: DeclId, target: DeclId, budget: u32) -> bool {
        if d == target {
            return true;
        }
        if budget == 0 {
            return false;
        }
        let mut hit = false;
        each_base(ir, d, &mut |b| {
            if !hit && go(ir, b, target, budget - 1) {
                hit = true;
            }
        });
        hit
    }
    go(ir, d, target, MAX_WALK_DEPTH)
}

/// Fill `buf` with the signed incidence participants `(sign, target)` of a relation edge instance
/// (all arcs). Reuses `buf` — no per-relation allocation once warmed.
fn edge_participants_into(ir: &Ir, edge: DeclId, buf: &mut Vec<(i8, DeclId)>) {
    buf.clear();
    if let Some((_, rec)) = ir.edge_for_decl(edge) {
        for &aid in &rec.arcs {
            if let Some(arc) = ir.arc(aid) {
                for sref in &arc.refs {
                    buf.push((sref.sign(), sref.target()));
                }
            }
        }
    }
}

/// A decl's direct child attribute `attr` value, if present.
fn attr_value(ir: &Ir, d: DeclId, attr: SymId) -> Option<&ValueR> {
    ir.child_decls(d)
        .find(|&c| ir.decl_node(c).map(|n| n.name == attr).unwrap_or(false))
        .and_then(|c| ir.decl_node(c).and_then(|n| n.anno.value.as_ref()))
}

/// The scene that (transitively) contains `d`, found by a bounded parent walk.
fn owning_scene(ir: &Ir, d: DeclId, scenes: &HashSet<DeclId>) -> Option<DeclId> {
    let mut cur = ir.parent_of(d);
    let mut budget = MAX_WALK_DEPTH;
    while let Some(p) = cur {
        if scenes.contains(&p) {
            return Some(p);
        }
        if budget == 0 {
            break;
        }
        budget -= 1;
        cur = ir.parent_of(p);
    }
    None
}

fn check_dangling(ir: &Ir, relation: DeclId, buf: &[(i8, DeclId)], out: &mut Vec<SemanticError>) {
    for (_, target) in buf {
        if ir.decl_node(*target).is_none() {
            out.push(SemanticError::DanglingReference {
                relation,
                target: *target,
            });
        }
    }
}

// ---------------------------------------------------------------------------------------------
// checks (consume the buckets; no full-Ir rescans)
// ---------------------------------------------------------------------------------------------

fn check_session(
    ir: &Ir,
    reg: &Registry,
    b: &Buckets,
    buf: &mut Vec<(i8, DeclId)>,
    out: &mut Vec<SemanticError>,
) {
    let scene_set: HashSet<DeclId> = b.scenes.iter().copied().collect();
    let mut configs_per_scene: HashMap<DeclId, u32> = HashMap::new();
    for &cfg in &b.session_configs {
        let scene = owning_scene(ir, cfg, &scene_set);
        edge_participants_into(ir, cfg, buf);
        let modes = buf
            .iter()
            .filter(|(s, _)| *s > 0)
            .map(|(_, t)| *t)
            .collect::<Vec<_>>();
        let spaces = buf
            .iter()
            .filter(|(s, _)| *s < 0)
            .map(|(_, t)| *t)
            .collect::<Vec<_>>();
        let scene_id = scene.unwrap_or(cfg);
        if modes.len() != 1
            || !modes
                .iter()
                .all(|&t| reg.is_kind_in(ir, t, &reg.session_modes))
        {
            out.push(SemanticError::SessionModeCardinality {
                scene: scene_id,
                found: modes.len() as u32,
            });
        }
        if spaces.len() != 1
            || !spaces
                .iter()
                .all(|&t| reg.is_kind_in(ir, t, &reg.reference_spaces))
        {
            out.push(SemanticError::ReferenceSpaceCardinality {
                scene: scene_id,
                found: spaces.len() as u32,
            });
        }
        if let Some(sc) = scene {
            *configs_per_scene.entry(sc).or_insert(0) += 1;
        }
    }
    for &scene in &b.scenes {
        let n = configs_per_scene.get(&scene).copied().unwrap_or(0);
        if n != 1 {
            out.push(SemanticError::SessionModeCardinality { scene, found: n });
        }
    }
}

fn check_capabilities(
    ir: &Ir,
    it: &Interner,
    reg: &Registry,
    b: &Buckets,
    buf: &mut Vec<(i8, DeclId)>,
    out: &mut Vec<SemanticError>,
) {
    let mut required: HashSet<DeclId> = HashSet::new();
    let mut optional: HashSet<DeclId> = HashSet::new();
    for &req in &b.capability_requests {
        edge_participants_into(ir, req, buf);
        let requirement = requirement_of(ir, it, reg, req);
        for &(sign, target) in buf.iter() {
            if sign < 0 {
                if !reg.is_kind_in(ir, target, &reg.capabilities) {
                    out.push(SemanticError::UnknownCapability {
                        request: req,
                        target,
                    });
                } else {
                    match requirement {
                        Some(true) => {
                            required.insert(target);
                        }
                        Some(false) => {
                            optional.insert(target);
                        }
                        None => out.push(SemanticError::BadCapabilityRequirement { request: req }),
                    }
                }
            }
        }
    }
    for cap in required.intersection(&optional) {
        out.push(SemanticError::CapabilityRequiredOptionalOverlap { capability: *cap });
    }
}

/// `Some(true)` = required, `Some(false)` = optional, `None` = missing/invalid.
fn requirement_of(ir: &Ir, it: &Interner, reg: &Registry, req: DeclId) -> Option<bool> {
    match attr_value(ir, req, reg.sym_requirement?)? {
        ValueR::Str(s) => match it.resolve(*s) {
            "required" => Some(true),
            "optional" => Some(false),
            _ => None,
        },
        _ => None,
    }
}

fn check_frames_and_placements(
    ir: &Ir,
    _it: &Interner,
    reg: &Registry,
    b: &Buckets,
    buf: &mut Vec<(i8, DeclId)>,
    out: &mut Vec<SemanticError>,
) {
    // parent-frame acyclicity
    let mut parent: HashMap<DeclId, DeclId> = HashMap::new();
    for &pf in &b.parent_frames {
        edge_participants_into(ir, pf, buf);
        let child = buf.iter().find(|(s, _)| *s > 0).map(|(_, t)| *t);
        let par = buf.iter().find(|(s, _)| *s < 0).map(|(_, t)| *t);
        if let (Some(c), Some(p)) = (child, par) {
            parent.insert(c, p);
        }
    }
    for &start in parent.keys() {
        if frame_chain_has_cycle(&parent, start) {
            out.push(SemanticError::FrameCycle { frame: start });
        }
    }

    // placements: dangling, role cardinality, frame kind, pose
    let arch = reg.name(ir, reg.placement);
    for &pl in &b.placements {
        edge_participants_into(ir, pl, buf);
        check_dangling(ir, pl, buf, out);
        let subjects = buf.iter().filter(|(s, _)| *s > 0).count() as u32;
        let subject_ok = buf
            .iter()
            .filter(|(s, _)| *s > 0)
            .all(|(_, t)| isa_transitive(ir, *t, reg.spatial_entity));
        if subjects != 1 || !subject_ok {
            out.push(SemanticError::RelationRole {
                relation: pl,
                archetype: arch,
                reason: RoleReason::SubjectCount { found: subjects },
            });
        }
        for &(sign, frame) in buf.iter() {
            if sign < 0 && !isa_transitive(ir, frame, reg.spatial_frame) {
                out.push(SemanticError::FrameMissingOrNotAFrame {
                    placement: pl,
                    frame,
                });
            }
        }
        check_pose(ir, reg, pl, out);
    }
}

fn frame_chain_has_cycle(parent: &HashMap<DeclId, DeclId>, start: DeclId) -> bool {
    let mut cur = start;
    let mut steps = 0u32;
    while let Some(&p) = parent.get(&cur) {
        cur = p;
        steps += 1;
        if cur == start || steps as usize > parent.len() {
            return true;
        }
    }
    false
}

enum PosKind {
    Finite,
    Positive,
    Quaternion,
}

fn check_pose(ir: &Ir, reg: &Registry, relation: DeclId, out: &mut Vec<SemanticError>) {
    if let Some(sym) = reg.sym_position {
        check_vec(ir, relation, sym, 3, PosKind::Finite, out);
    }
    if let Some(sym) = reg.sym_orientation {
        check_vec(ir, relation, sym, 4, PosKind::Quaternion, out);
    }
    if let Some(sym) = reg.sym_scale {
        check_vec(ir, relation, sym, 3, PosKind::Positive, out);
    }
}

fn check_vec(
    ir: &Ir,
    relation: DeclId,
    attr: SymId,
    n: u32,
    kind: PosKind,
    out: &mut Vec<SemanticError>,
) {
    let value = match attr_value(ir, relation, attr) {
        Some(v) => v,
        None => return, // an absent pose component defaults to identity; presence is validated
    };
    let items = match value {
        ValueR::List(items) => items,
        _ => {
            out.push(SemanticError::PoseBadLength {
                relation,
                attr,
                expected: n,
                found: 0,
            });
            return;
        }
    };
    if items.len() as u32 != n {
        out.push(SemanticError::PoseBadLength {
            relation,
            attr,
            expected: n,
            found: items.len() as u32,
        });
        return;
    }
    let mut norm2 = 0.0f64;
    for (i, comp) in items.iter().enumerate() {
        match comp {
            ValueR::Num(x) if x.is_finite() => {
                norm2 += x * x;
                if matches!(kind, PosKind::Positive) && *x <= 0.0 {
                    out.push(SemanticError::ScaleNonPositive {
                        relation,
                        index: i as u32,
                    });
                }
            }
            _ => out.push(SemanticError::PoseNonFinite {
                relation,
                attr,
                index: i as u32,
            }),
        }
    }
    if matches!(kind, PosKind::Quaternion) && norm2 < QUAT_NORM_EPSILON {
        out.push(SemanticError::QuaternionNotNormalizable { relation });
    }
}

fn check_interactions_and_anchoring(
    ir: &Ir,
    reg: &Registry,
    b: &Buckets,
    buf: &mut Vec<(i8, DeclId)>,
    out: &mut Vec<SemanticError>,
) {
    let arch_i = reg.name(ir, reg.interaction);
    for &ix in &b.interactions {
        edge_participants_into(ir, ix, buf);
        check_dangling(ir, ix, buf, out);
        let inputs = buf.iter().filter(|(s, _)| *s > 0).count() as u32;
        let targets = buf.iter().filter(|(s, _)| *s < 0).count() as u32;
        if inputs != 1 {
            out.push(SemanticError::RelationRole {
                relation: ix,
                archetype: arch_i,
                reason: RoleReason::SubjectCount { found: inputs },
            });
        }
        if targets == 0 {
            out.push(SemanticError::RelationRole {
                relation: ix,
                archetype: arch_i,
                reason: RoleReason::NoTarget,
            });
        }
        for &(sign, act) in buf.iter() {
            if sign == 0
                && !reg.is_kind_in(ir, act, &reg.actions)
                && !isa_transitive(ir, act, reg.action)
            {
                out.push(SemanticError::UnknownAction {
                    interaction: ix,
                    target: act,
                });
            }
        }
    }

    let arch_a = reg.name(ir, reg.anchoring);
    for &an in &b.anchorings {
        edge_participants_into(ir, an, buf);
        check_dangling(ir, an, buf, out);
        let frame_ok = buf
            .iter()
            .filter(|(s, _)| *s < 0)
            .all(|(_, t)| isa_transitive(ir, *t, reg.spatial_frame));
        let anchor_ok = buf
            .iter()
            .filter(|(s, _)| *s == 0)
            .any(|(_, t)| isa_transitive(ir, *t, reg.semantic_anchor));
        if let Some(&(_, t)) = buf
            .iter()
            .find(|(s, t)| *s > 0 && !isa_transitive(ir, *t, reg.spatial_entity))
        {
            out.push(SemanticError::RelationRole {
                relation: an,
                archetype: arch_a,
                reason: RoleReason::WrongKind { participant: t },
            });
        }
        if !frame_ok || !anchor_ok {
            out.push(SemanticError::RelationRole {
                relation: an,
                archetype: arch_a,
                reason: RoleReason::NoTarget,
            });
        }
    }
}

fn check_assets(ir: &Ir, reg: &Registry, b: &Buckets, out: &mut Vec<SemanticError>) {
    let uri = match reg.sym_uri {
        Some(s) => s,
        None => return,
    };
    for &asset in &b.visual_assets {
        match attr_value(ir, asset, uri) {
            Some(ValueR::Str(_)) => {}
            _ => out.push(SemanticError::AssetUriNotString { asset }),
        }
    }
}

fn check_runtime_handle_fields(ir: &Ir, reg: &Registry, out: &mut Vec<SemanticError>) {
    if reg.runtime_handle_fields.is_empty() {
        return;
    }
    for (_, node) in ir.decls() {
        if reg.runtime_handle_fields.contains(&node.name) {
            out.push(SemanticError::RuntimeHandleField {
                decl: node.parent,
                field: node.name,
            });
        }
    }
}
