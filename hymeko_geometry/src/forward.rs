//! Forward kinematics: the world pose of every link.
//!
//! The framework could always say what a mechanism *is* — links, joints, axes, origins, all read out
//! of the IR — but never where its parts *are*. It did not need to: its consumers were URDF, SDF and
//! MJCF emitters, which write the tree out and let Gazebo or MuJoCo do the composing. A consumer that
//! has to draw the mechanism, or reason about where a tool tip ends up, needs the poses themselves.
//!
//! ## No second graph
//!
//! The chain is not re-encoded as an adjacency structure to walk. The IR already carries it: a joint
//! is an *edge* whose incidences are signed — the `+` incidence names the parent link, the `-` one
//! the child — and [`crate::kinematics`] surfaces exactly those. So a link's pose is defined by the
//! one relation that already exists:
//!
//! ```text
//!   pose(link) = pose(parent(link)) ∘ origin(joint) ∘ motion(joint, q)
//! ```
//!
//! and the whole computation is that relation, memoised, resolved from each link upwards until it
//! reaches the root. No ordering pass, no derived topology.
//!
//! ## Rigid motions come from the Tensor pillar
//!
//! Composition is [`Se`] — $SE(n)$ in `hymeko_clifford`, where a rotation is carried by a rotor and
//! composition is the geometric product. No transform type is defined here.

use std::collections::HashMap;

use hymeko_clifford::Se;

use hymeko::common::ids::DeclId;

use crate::kinematics::joints::{JointInfo, JointType};
use crate::kinematics::kinematic::KinematicModel;

/// Joint positions by joint name: an angle in radians for a revolute or continuous joint, a distance
/// for a prismatic one. A joint absent from the map sits at zero.
pub type JointValues = HashMap<String, f64>;

/// Why the poses could not be resolved.
///
/// Each variant names a specific structural defect, because the caller can usually point at the
/// declaration responsible.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FkError {
    /// The model declares no links.
    NoLinks,
    /// A link is the child of more than one joint, so the path to it — and hence its pose — is not
    /// unique. Carries the link and the joints that claim it.
    MultipleParents {
        /// The over-parented link.
        link: String,
        /// The joints claiming it as their child.
        joints: Vec<String>,
    },
    /// Following parents from this link never reaches a root. Carries the cycle, in walk order.
    Cycle(Vec<String>),
    /// More than one link has no parent joint: a forest, not one mechanism. Carries the roots, sorted.
    MultipleRoots(Vec<String>),
    /// A joint names a parent link that the model does not declare.
    UnknownLink {
        /// The joint holding the dangling reference.
        joint: String,
        /// The link name it refers to.
        link: String,
    },
}

impl core::fmt::Display for FkError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NoLinks => write!(f, "model declares no links"),
            Self::MultipleParents { link, joints } => write!(
                f,
                "link {link:?} is the child of {} joints ({joints:?}); its pose is not unique",
                joints.len()
            ),
            Self::Cycle(c) => write!(f, "joint chain closes on itself: {c:?}"),
            Self::MultipleRoots(r) => write!(
                f,
                "expected one root link, found {}: {r:?}",
                r.len()
            ),
            Self::UnknownLink { joint, link } => {
                write!(f, "joint {joint:?} refers to undeclared link {link:?}")
            }
        }
    }
}

impl std::error::Error for FkError {}

/// The resolved world pose of every link.
#[derive(Debug, Clone)]
pub struct LinkPoses {
    /// The link with no parent joint — the frame every pose is expressed in.
    pub root: String,
    poses: HashMap<String, Se>,
}

impl LinkPoses {
    /// The world pose of one link, if the model declares it.
    pub fn get(&self, link: &str) -> Option<&Se> {
        self.poses.get(link)
    }

    /// Every `(link, pose)` pair. Order is unspecified.
    pub fn iter(&self) -> impl Iterator<Item = (&String, &Se)> {
        self.poses.iter()
    }

    /// How many links were resolved.
    pub fn len(&self) -> usize {
        self.poses.len()
    }

    /// Whether no links were resolved. Never true for a value returned by [`forward_kinematics`].
    pub fn is_empty(&self) -> bool {
        self.poses.is_empty()
    }
}

/// The motion a joint contributes at position `value`, in its own frame — applied after the joint's
/// fixed origin.
///
/// A revolute or continuous joint turns about its axis; a prismatic one slides along it; a fixed one
/// contributes nothing. A joint that declares no axis is read as the URDF default, $+x$.
///
/// `value` is the joint coordinate itself, applied as declared. Coupling between joints — a mimic
/// joint, a transmission ratio — is deliberately NOT folded in here: that would make `value` a
/// pre-image of the joint coordinate rather than the coordinate, conflating what the joint positions
/// are with where the links end up. Such a relation resolves into actual joint positions before this
/// function is reached.
pub fn joint_motion(joint: &JointInfo, value: f64) -> Se {
    let axis = joint.axis.unwrap_or([1.0, 0.0, 0.0]);
    match joint.joint_type {
        JointType::Fixed => Se::identity(3),
        JointType::Revolute | JointType::Continuous => Se::se3_axis_angle(axis, value),
        JointType::Prismatic => Se::se3_axis_translation(axis, value),
    }
}

/// The joint's fixed origin: where the child frame sits in the parent frame before the joint moves.
///
/// This is the `origin` carried as weight annotations on the joint's parent incidence — an `xyz`
/// list and an `rpy` list in **degrees**, which is the convention the source files use.
pub fn joint_origin(joint: &JointInfo) -> Se {
    Se::se3_rpy_xyz(
        joint.origin_rpy_rad().unwrap_or([0.0; 3]),
        joint.origin_xyz.unwrap_or([0.0; 3]),
    )
}

/// Resolve the world pose of every link at the given joint positions.
///
/// The root link sits at the identity; every other link's pose is its parent's composed with the
/// connecting joint's origin and motion. Fails rather than guessing if the structure is not a single
/// rooted chain — see [`FkError`].
pub fn forward_kinematics(model: &KinematicModel, q: &JointValues) -> Result<LinkPoses, FkError> {
    if model.links.is_empty() {
        return Err(FkError::NoLinks);
    }

    // The one index this needs: which joint brings each link in. That is a lookup, not a derived
    // graph — the relation is the IR's own parent/child incidence, read straight off the joint.
    let mut parent_joint: HashMap<&str, &JointInfo> = HashMap::new();
    let mut claimed: HashMap<&str, Vec<String>> = HashMap::new();
    for j in &model.joints {
        claimed
            .entry(j.child_link.as_str())
            .or_default()
            .push(j.name.clone());
        parent_joint.insert(j.child_link.as_str(), j);
    }
    if let Some((link, joints)) = claimed
        .iter()
        .filter(|(_, js)| js.len() > 1)
        .min_by_key(|(l, _)| *l)
    {
        let mut joints = joints.clone();
        joints.sort();
        return Err(FkError::MultipleParents {
            link: (*link).to_string(),
            joints,
        });
    }

    let declared: HashMap<&str, ()> = model.links.iter().map(|l| (l.name.as_str(), ())).collect();
    let frames: HashMap<&str, ()> = model.frames.iter().map(|f| (f.as_str(), ())).collect();

    // A link is a root two ways: nothing brings it in at all, or what brings it in is joined to a
    // FRAME rather than to another link. The second is how every floating base in the robotics
    // library is written — `world --@base--> torso` — and reading it as anything else made the
    // whole mechanism look like a loop: no link was parentless, so the walk concluded it must have
    // closed on itself and reported a cycle between `torso` and a `world` that is not a body at all.
    let anchor: HashMap<&str, &JointInfo> = model
        .joints
        .iter()
        .filter(|j| frames.contains_key(j.parent_link.as_str()))
        .map(|j| (j.child_link.as_str(), j))
        .collect();
    let mut roots: Vec<String> = model
        .links
        .iter()
        .filter(|l| {
            !parent_joint.contains_key(l.name.as_str()) || anchor.contains_key(l.name.as_str())
        })
        .map(|l| l.name.clone())
        .collect();
    roots.sort();
    let root = match roots.len() {
        0 => {
            // Every link has a parent and none of them is anchored, so following them must close a
            // loop; name it.
            let start = model.links[0].name.as_str();
            return Err(FkError::Cycle(trace_cycle(start, &parent_joint)));
        }
        1 => roots.remove(0),
        _ => return Err(FkError::MultipleRoots(roots)),
    };

    // pose(link) = pose(parent) ∘ origin ∘ motion, memoised. Resolving upwards means each link is
    // visited once and no evaluation order has to be established in advance.
    //
    // The root sits at its anchoring joint's own origin and motion, not at the identity: that joint
    // is what states the standing height of a floating base, and dropping it would put every model
    // in the library on the floor plane it is meant to stand above.
    let mut poses: HashMap<String, Se> = HashMap::new();
    poses.insert(
        root.clone(),
        match anchor.get(root.as_str()) {
            Some(j) => joint_origin(j).compose(&joint_motion(j, q.get(&j.name).copied().unwrap_or(0.0))),
            None => Se::identity(3),
        },
    );
    for link in &model.links {
        let mut chain: Vec<&str> = Vec::new();
        let mut cursor: &str = link.name.as_str();
        // walk up to the nearest link whose pose is known
        while !poses.contains_key(cursor) {
            if chain.contains(&cursor) {
                let mut cycle: Vec<String> = chain.iter().map(|s| (*s).to_string()).collect();
                cycle.push(cursor.to_string());
                return Err(FkError::Cycle(cycle));
            }
            chain.push(cursor);
            let j = parent_joint[cursor];
            if !declared.contains_key(j.parent_link.as_str()) {
                // A frame parent is an anchor, and an anchored link is already a root with a pose,
                // so the walk can only reach here on a parent that is neither link nor frame.
                return Err(FkError::UnknownLink {
                    joint: j.name.clone(),
                    link: j.parent_link.clone(),
                });
            }
            cursor = j.parent_link.as_str();
        }
        // …then back down, composing as we go
        for name in chain.iter().rev() {
            let j = parent_joint[*name];
            let value = q.get(&j.name).copied().unwrap_or(0.0);
            let pose = poses[j.parent_link.as_str()]
                .compose(&joint_origin(j))
                .compose(&joint_motion(j, value));
            poses.insert((*name).to_string(), pose);
        }
    }

    Ok(LinkPoses { root, poses })
}

/// Follow parents from `start` until a link repeats, and return the loop in walk order.
fn trace_cycle(start: &str, parent_joint: &HashMap<&str, &JointInfo>) -> Vec<String> {
    let mut seen: Vec<String> = Vec::new();
    let mut cursor = start;
    loop {
        if seen.iter().any(|s| s == cursor) {
            seen.push(cursor.to_string());
            return seen;
        }
        seen.push(cursor.to_string());
        match parent_joint.get(cursor) {
            Some(j) => cursor = j.parent_link.as_str(),
            None => return seen,
        }
    }
}


// ─── The prepared plan ──────────────────────────────────────────────────────────────────────────

/// The topology, resolved once.
///
/// [`forward_kinematics`] answers a single question completely, and pays for that on every call: it
/// builds three hash maps, hashes a `String` for every link and joint, and clones every joint name
/// into a fresh vector. Measured on a 33-link quadruped that bookkeeping is 18 of 51 microseconds —
/// 36% — and none of it depends on the joint values, which is the only thing that actually changes
/// between calls.
///
/// A plan does that work once. Afterwards a pose costs the rigid-motion compositions and nothing
/// else: no hashing, no lookup, no allocation. `tests/fk_plan.rs` pins the last of those with a
/// counting allocator rather than with a timing measurement, because a timing measurement passes on
/// a fast machine having proved nothing.
///
/// # The topology comes from the arena
///
/// Links and joints are matched by [`DeclId`] — the IR's own index — and only fall back to matching
/// by name where a model carries no handles, which is the case for a hand-built fixture and for
/// nothing that came out of a compile. The distinction is the point: the IR is a unified arena, and
/// re-deriving its structure from resolved strings is the copy this type exists to avoid.
pub struct FkPlan {
    /// For each link, the joint that brings it in and the link it hangs from.
    parent: Vec<Option<(usize, usize)>>,
    /// Link indices ordered so that every parent precedes its children.
    order: Vec<usize>,
    /// The root link's index.
    root: usize,
    /// Joint index by name, so a caller can fill a flat value vector once.
    joint_slot: HashMap<String, usize>,
}

impl FkPlan {
    /// Resolve a model's topology. The same structural failures [`forward_kinematics`] reports.
    pub fn new(model: &KinematicModel) -> Result<Self, FkError> {
        if model.links.is_empty() {
            return Err(FkError::NoLinks);
        }
        let n = model.links.len();

        // link index, by handle where there is one and by name otherwise
        let mut by_handle: HashMap<DeclId, usize> = HashMap::with_capacity(n);
        let mut by_name: HashMap<&str, usize> = HashMap::with_capacity(n);
        for (i, l) in model.links.iter().enumerate() {
            if l.did != DeclId::NONE {
                by_handle.insert(l.did, i);
            }
            by_name.insert(l.name.as_str(), i);
        }
        let frames: HashMap<&str, ()> = model.frames.iter().map(|f| (f.as_str(), ())).collect();
        let locate = |handle: DeclId, name: &str| -> Option<usize> {
            by_handle
                .get(&handle)
                .copied()
                .or_else(|| by_name.get(name).copied())
        };

        let mut parent: Vec<Option<(usize, usize)>> = vec![None; n];
        let mut claimed: Vec<Vec<&str>> = vec![Vec::new(); n];
        let mut anchored = vec![false; n];
        for (ji, j) in model.joints.iter().enumerate() {
            let Some(child) = locate(j.child, &j.child_link) else {
                continue; // a joint whose child is not a link of this model brings nothing in
            };
            claimed[child].push(j.name.as_str());
            match locate(j.parent, &j.parent_link) {
                Some(p) => parent[child] = Some((ji, p)),
                None if frames.contains_key(j.parent_link.as_str()) => {
                    // joined to a frame: an external anchor, so the child is a root that carries
                    // this joint's own origin and motion
                    anchored[child] = true;
                    parent[child] = Some((ji, usize::MAX));
                }
                None => {
                    return Err(FkError::UnknownLink {
                        joint: j.name.clone(),
                        link: j.parent_link.clone(),
                    })
                }
            }
        }
        if let Some((i, js)) = claimed
            .iter()
            .enumerate()
            .find(|(_, js)| js.len() > 1)
        {
            let mut joints: Vec<String> = js.iter().map(|s| (*s).to_string()).collect();
            joints.sort();
            return Err(FkError::MultipleParents {
                link: model.links[i].name.clone(),
                joints,
            });
        }

        let mut roots: Vec<usize> = (0..n)
            .filter(|&i| parent[i].is_none() || anchored[i])
            .collect();
        let root = match roots.len() {
            0 => {
                let start = model.links[0].name.as_str();
                let lookup: HashMap<&str, &JointInfo> = model
                    .joints
                    .iter()
                    .map(|j| (j.child_link.as_str(), j))
                    .collect();
                return Err(FkError::Cycle(trace_cycle(start, &lookup)));
            }
            1 => roots.remove(0),
            _ => {
                let mut names: Vec<String> =
                    roots.iter().map(|&i| model.links[i].name.clone()).collect();
                names.sort();
                return Err(FkError::MultipleRoots(names));
            }
        };

        // an order in which every parent precedes its children
        let mut order = Vec::with_capacity(n);
        let mut placed = vec![false; n];
        order.push(root);
        placed[root] = true;
        let mut cursor = 0;
        while cursor < order.len() {
            let current = order[cursor];
            cursor += 1;
            for i in 0..n {
                if placed[i] {
                    continue;
                }
                if let Some((_, p)) = parent[i] {
                    if p == current {
                        order.push(i);
                        placed[i] = true;
                    }
                }
            }
        }
        if order.len() != n {
            let unplaced = (0..n).find(|&i| !placed[i]).unwrap_or(0);
            let lookup: HashMap<&str, &JointInfo> = model
                .joints
                .iter()
                .map(|j| (j.child_link.as_str(), j))
                .collect();
            return Err(FkError::Cycle(trace_cycle(
                model.links[unplaced].name.as_str(),
                &lookup,
            )));
        }

        Ok(FkPlan {
            parent,
            order,
            root,
            joint_slot: model
                .joints
                .iter()
                .enumerate()
                .map(|(i, j)| (j.name.clone(), i))
                .collect(),
        })
    }

    /// Which slot of the value vector drives `joint`.
    pub fn slot(&self, joint: &str) -> Option<usize> {
        self.joint_slot.get(joint).copied()
    }

    /// A value vector sized for [`Self::resolve`], zero everywhere.
    pub fn zero_values(&self) -> Vec<f64> {
        vec![0.0; self.joint_slot.len()]
    }

    /// A pose buffer sized for [`Self::resolve`].
    pub fn identity_poses(&self) -> Vec<Se> {
        (0..self.parent.len()).map(|_| Se::identity(3)).collect()
    }

    /// The root link's index.
    pub fn root(&self) -> usize {
        self.root
    }

    /// Write every link's world pose into `out`, in the order the model lists its links.
    ///
    /// `q` is indexed by joint, `out` by link. Both are the caller's, and both are reused across
    /// calls — which is the whole point: this function performs no allocation.
    ///
    /// # Panics
    /// If `q` is shorter than the joint count or `out` shorter than the link count.
    pub fn resolve(&self, model: &KinematicModel, q: &[f64], out: &mut [Se]) {
        assert!(q.len() >= self.joint_slot.len(), "q must cover every joint");
        assert!(out.len() >= self.parent.len(), "out must cover every link");
        for &i in &self.order {
            match self.parent[i] {
                None => out[i] = Se::identity(3),
                Some((ji, p)) => {
                    let j = &model.joints[ji];
                    // Left to right, exactly as `forward_kinematics` composes. Composition is
                    // associative in the algebra and NOT in floating point, so grouping the two
                    // local factors first — which reads better and saves nothing — makes this a
                    // different function whose answers differ in the last bit.
                    out[i] = if p == usize::MAX {
                        // anchored to a frame: the joint's own origin and motion ARE the world pose
                        joint_origin(j).compose(&joint_motion(j, q[ji]))
                    } else {
                        out[p]
                            .compose(&joint_origin(j))
                            .compose(&joint_motion(j, q[ji]))
                    };
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematics::kinematic::LinkInfo;
    use hymeko::common::ids::DeclId;

    const TAU: f64 = std::f64::consts::TAU;

    fn link(name: &str) -> LinkInfo {
        LinkInfo {
            did: DeclId::NONE,
            name: name.to_string(),
            mass: None,
            geometry: None,
            origin: None,
            color: None,
        }
    }

    fn joint(
        name: &str,
        parent: &str,
        child: &str,
        jtype: JointType,
        xyz: Option<[f64; 3]>,
        axis: Option<[f64; 3]>,
    ) -> JointInfo {
        JointInfo {
            did: DeclId::NONE,
            name: name.to_string(),
            joint_type: jtype,
            // A hand-built fixture has no arena behind it, so the handles are absent and the plan
            // falls back to matching by name. That fallback is itself under test — see
            // `a_plan_built_without_handles_still_resolves`.
            parent: DeclId::NONE,
            child: DeclId::NONE,
            parent_link: parent.to_string(),
            child_link: child.to_string(),
            axis,
            origin_xyz: xyz,
            origin_rpy_deg: None,
            limits: None,
        }
    }

    /// base ──j1──▶ upper ──j2──▶ tool, both revolute about +z, each offset 1 along +x.
    fn two_link_arm() -> KinematicModel {
        KinematicModel {
            name: "arm".into(),
            links: vec![link("base"), link("upper"), link("tool")],
            joints: vec![
                joint(
                    "j1",
                    "base",
                    "upper",
                    JointType::Revolute,
                    Some([1.0, 0.0, 0.0]),
                    Some([0.0, 0.0, 1.0]),
                ),
                joint(
                    "j2",
                    "upper",
                    "tool",
                    JointType::Revolute,
                    Some([1.0, 0.0, 0.0]),
                    Some([0.0, 0.0, 1.0]),
                ),
            ],
            frames: Vec::new(),
        }
    }

    fn close(a: &[f64], b: [f64; 3]) -> bool {
        a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-9)
    }

    #[test]
    fn at_zero_the_chain_lies_along_its_origins() {
        let poses = forward_kinematics(&two_link_arm(), &JointValues::new()).unwrap();
        assert_eq!(poses.root, "base");
        assert_eq!(poses.len(), 3);
        assert!(close(&poses.get("base").unwrap().translation, [0.0, 0.0, 0.0]));
        assert!(close(&poses.get("upper").unwrap().translation, [1.0, 0.0, 0.0]));
        assert!(close(&poses.get("tool").unwrap().translation, [2.0, 0.0, 0.0]));
    }

    #[test]
    fn a_revolute_joint_swings_everything_below_it() {
        // Turn the base joint a quarter turn about +z: the upper link stays where the origin put it
        // (the rotation happens *at* that point) but the tool swings from +x to +y.
        let mut q = JointValues::new();
        q.insert("j1".into(), TAU / 4.0);
        let poses = forward_kinematics(&two_link_arm(), &q).unwrap();
        assert!(close(&poses.get("upper").unwrap().translation, [1.0, 0.0, 0.0]));
        assert!(close(&poses.get("tool").unwrap().translation, [1.0, 1.0, 0.0]));
    }

    #[test]
    fn joint_values_compose_down_the_chain() {
        // Both joints a quarter turn: the second turn is applied in the frame the first produced,
        // so the tool folds back toward -x rather than continuing around.
        let mut q = JointValues::new();
        q.insert("j1".into(), TAU / 4.0);
        q.insert("j2".into(), TAU / 4.0);
        let poses = forward_kinematics(&two_link_arm(), &q).unwrap();
        assert!(close(&poses.get("tool").unwrap().translation, [1.0, 1.0, 0.0]));
        // …and the tool's orientation has accumulated both turns: its local +x now points -x.
        assert!(close(
            &poses.get("tool").unwrap().rotate(&[1.0, 0.0, 0.0]),
            [-1.0, 0.0, 0.0]
        ));
    }

    #[test]
    fn a_prismatic_joint_extends_along_its_axis() {
        let mut m = two_link_arm();
        m.joints[1].joint_type = JointType::Prismatic;
        let mut q = JointValues::new();
        q.insert("j2".into(), 0.5);
        let poses = forward_kinematics(&m, &q).unwrap();
        // j2's axis is +z, so the tool lifts rather than reaching further along x
        assert!(close(&poses.get("tool").unwrap().translation, [2.0, 0.0, 0.5]));
    }

    #[test]
    fn a_fixed_joint_ignores_its_value() {
        let mut m = two_link_arm();
        m.joints[1].joint_type = JointType::Fixed;
        let mut q = JointValues::new();
        q.insert("j2".into(), 1.234);
        let poses = forward_kinematics(&m, &q).unwrap();
        assert!(close(&poses.get("tool").unwrap().translation, [2.0, 0.0, 0.0]));
    }

    #[test]
    fn a_branching_tree_resolves_every_limb() {
        // base ──▶ left, base ──▶ right: two children of one parent is a tree, not an error.
        let m = KinematicModel {
            name: "fork".into(),
            links: vec![link("base"), link("left"), link("right")],
            joints: vec![
                joint("jl", "base", "left", JointType::Fixed, Some([0.0, 1.0, 0.0]), None),
                joint("jr", "base", "right", JointType::Fixed, Some([0.0, -1.0, 0.0]), None),
            ],
            frames: Vec::new(),
        };
        let poses = forward_kinematics(&m, &JointValues::new()).unwrap();
        assert!(close(&poses.get("left").unwrap().translation, [0.0, 1.0, 0.0]));
        assert!(close(&poses.get("right").unwrap().translation, [0.0, -1.0, 0.0]));
    }

    #[test]
    fn a_link_with_two_parent_joints_is_rejected() {
        let mut m = two_link_arm();
        m.joints.push(joint("j3", "base", "tool", JointType::Fixed, None, None));
        match forward_kinematics(&m, &JointValues::new()) {
            Err(FkError::MultipleParents { link, joints }) => {
                assert_eq!(link, "tool");
                assert_eq!(joints, vec!["j2".to_string(), "j3".to_string()]);
            }
            other => panic!("expected MultipleParents, got {other:?}"),
        }
    }

    #[test]
    fn a_disconnected_model_is_rejected_rather_than_silently_posed() {
        let mut m = two_link_arm();
        m.links.push(link("stray"));
        match forward_kinematics(&m, &JointValues::new()) {
            Err(FkError::MultipleRoots(r)) => assert_eq!(r, vec!["base".to_string(), "stray".to_string()]),
            other => panic!("expected MultipleRoots, got {other:?}"),
        }
    }

    #[test]
    fn a_closed_loop_is_reported_as_a_cycle() {
        // base ──▶ upper ──▶ tool ──▶ base : every link has a parent, so there is no root.
        let mut m = two_link_arm();
        m.joints.push(joint("j3", "tool", "base", JointType::Fixed, None, None));
        match forward_kinematics(&m, &JointValues::new()) {
            Err(FkError::Cycle(c)) => {
                assert!(c.len() >= 2, "cycle must name the loop, got {c:?}");
                assert_eq!(c.first(), c.last(), "a cycle closes on itself: {c:?}");
            }
            other => panic!("expected Cycle, got {other:?}"),
        }
    }

    #[test]
    fn a_joint_naming_an_undeclared_parent_is_reported() {
        let mut m = two_link_arm();
        m.joints[1].parent_link = "ghost".into();
        match forward_kinematics(&m, &JointValues::new()) {
            Err(FkError::UnknownLink { joint, link }) => {
                assert_eq!(joint, "j2");
                assert_eq!(link, "ghost");
            }
            other => panic!("expected UnknownLink, got {other:?}"),
        }
    }

    #[test]
    fn a_model_with_no_links_is_rejected() {
        let m = KinematicModel {
            name: "empty".into(),
            links: vec![],
            joints: vec![],
            frames: Vec::new(),
        };
        assert!(matches!(
            forward_kinematics(&m, &JointValues::new()),
            Err(FkError::NoLinks)
        ));
    }

    #[test]
    fn a_single_link_is_its_own_root_at_the_identity() {
        let m = KinematicModel {
            name: "one".into(),
            links: vec![link("base")],
            joints: vec![],
            frames: Vec::new(),
        };
        let poses = forward_kinematics(&m, &JointValues::new()).unwrap();
        assert_eq!(poses.root, "base");
        assert_eq!(*poses.get("base").unwrap(), Se::identity(3));
    }

    /// A floating base: `world` is a FRAME, not a link, and `@base` hangs the mechanism from it.
    ///
    /// This is how every model in the robotics library anchors, and reading it wrong is not a near
    /// miss: no link is parentless, so the root search finds none, concludes the chain must have
    /// closed on itself, and reports a cycle between the first link and a `world` that is not a body
    /// at all. Every robot then collapses to the identity — a heap of parts at the origin.
    #[test]
    fn a_joint_to_a_frame_anchors_the_root() {
        let mut m = two_link_arm();
        m.frames = vec!["world".into()];
        m.joints.insert(
            0,
            joint(
                "anchor",
                "world",
                "base",
                JointType::Fixed,
                Some([0.0, 0.0, 0.25]),
                None,
            ),
        );

        let poses = forward_kinematics(&m, &JointValues::new()).expect("a frame anchor is a root");
        assert_eq!(poses.root, "base");
        // and the anchoring joint's own origin is kept: it is what states the standing height of a
        // floating base, and dropping it would put every model in the library on the floor
        let base = poses.get("base").unwrap();
        assert!((base.translation[2] - 0.25).abs() < 1e-12, "{:?}", base.translation);
        // the rest of the chain rides on it
        let tool = poses.get("tool").unwrap();
        assert!((tool.translation[2] - 0.25).abs() < 1e-12, "{:?}", tool.translation);
        assert!((tool.translation[0] - 2.0).abs() < 1e-12, "{:?}", tool.translation);
    }

    /// A parent that is neither a link nor a declared frame is still a mistake, not an anchor.
    #[test]
    fn a_joint_to_nothing_is_not_an_anchor() {
        let mut m = two_link_arm();
        m.joints.push(joint(
            "dangling",
            "nowhere",
            "base",
            JointType::Fixed,
            None,
            None,
        ));
        // `base` is now claimed by a joint whose parent does not exist and is not a frame, so it is
        // not a root — and nothing else is either.
        assert!(matches!(
            forward_kinematics(&m, &JointValues::new()),
            Err(FkError::UnknownLink { .. }) | Err(FkError::Cycle(_))
        ));
    }
}
