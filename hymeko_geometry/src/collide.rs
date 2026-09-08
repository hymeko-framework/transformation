//! Collision detection over the shapes a mechanism declares.
//!
//! # One algorithm, four shapes
//!
//! `box`, `cylinder`, `sphere` and `capsule` are all **convex**, and every convex set is fully
//! described for these purposes by its support function — the farthest point in a given direction.
//! So there is no table of shape-pair routines here: there is one distance algorithm (GJK) and four
//! support functions, and adding a fifth shape is adding a fifth support function and nothing else.
//!
//! The alternative, a routine per ordered pair, is ten routines for five shapes and fifteen for six.
//! Each of them is a separate opportunity to get a sign wrong in a way that only shows up when two
//! particular parts touch.
//!
//! # The pair filter comes from the hypergraph
//!
//! A joint's parent and child ALWAYS overlap — they share the joint's location, that is what a joint
//! is — so testing them is not merely wasteful, it reports a permanent collision. The set of pairs
//! worth testing is therefore the complement of the incidence relation, which the kinematic model
//! already carries. [`BroadPhase::new`] takes it from there rather than asking the caller to
//! maintain a second list that can drift from the first.
//!
//! # What is here and what is not
//!
//! Boolean convex intersection through GJK. General signed distance, penetration depth and
//! witness points are not implemented here; sphere contact geometry lives in the engine.

use hymeko_clifford::Se3;

use crate::kinematics::joints::JointInfo;
use crate::kinematics::kinematic::{GeometryShape, KinematicModel};

/// A convex body in its own frame, centred on the frame's origin.
///
/// Internal dimensions are half-extents. Authored boxes use full side lengths,
/// and cylinders use [radius, full height], as in the MJCF/URDF projection.
/// The legacy capsule convention is separate and still requires an axis/length audit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Convex {
    Sphere {
        radius: f64,
    },
    /// Half-extents along each local axis.
    Cuboid {
        half: [f64; 3],
    },
    /// Axis along local `z`.
    Cylinder {
        radius: f64,
        half_height: f64,
    },
    /// A segment along local `z`, thickened by `radius`.
    Capsule {
        radius: f64,
        half_height: f64,
    },
}

impl Convex {
    /// Read a declared geometry, or `None` where the declaration gives nothing to collide with.
    ///
    /// A link without geometry is not an error: it is a frame, or a massless connector, and it takes
    /// part in the kinematics without taking part in the collisions.
    ///
    /// Deliberately exhaustive over [`GeometryShape`] with no catch-all arm, so that adding a shape
    /// to the declaration language is a compile error here rather than a silently uncollidable
    /// body.
    pub fn from_declaration(shape: GeometryShape, dimensions: &[f64]) -> Option<Self> {
        match shape {
            GeometryShape::Sphere => dimensions.first().map(|&r| Self::Sphere { radius: r }),
            GeometryShape::Box => (dimensions.len() >= 3).then(|| Self::Cuboid {
                half: [
                    dimensions[0] * 0.5,
                    dimensions[1] * 0.5,
                    dimensions[2] * 0.5,
                ],
            }),
            GeometryShape::Cylinder => (dimensions.len() >= 2).then(|| Self::Cylinder {
                radius: dimensions[0],
                half_height: dimensions[1] * 0.5,
            }),
            GeometryShape::Capsule => (dimensions.len() >= 2).then(|| Self::Capsule {
                radius: dimensions[0],
                half_height: dimensions[1],
            }),
        }
    }

    /// The farthest point of the shape in direction `d`, in the shape's own frame.
    ///
    /// `d` need not be normalised. Where the farthest point is not unique — a box face seen
    /// edge-on, a cylinder cap — any of them will do: GJK only needs a point of the supporting set,
    /// and the tie-break is settled consistently by the sign tests below so that a degenerate
    /// direction cannot make the simplex cycle.
    #[inline]
    pub fn support(&self, d: [f64; 3]) -> [f64; 3] {
        match *self {
            Self::Sphere { radius } => scale_to(d, radius),
            Self::Cuboid { half } => [
                if d[0] >= 0.0 { half[0] } else { -half[0] },
                if d[1] >= 0.0 { half[1] } else { -half[1] },
                if d[2] >= 0.0 { half[2] } else { -half[2] },
            ],
            Self::Cylinder {
                radius,
                half_height,
            } => {
                let planar = (d[0] * d[0] + d[1] * d[1]).sqrt();
                let (x, y) = if planar > 1e-12 {
                    (radius * d[0] / planar, radius * d[1] / planar)
                } else {
                    (0.0, 0.0)
                };
                [
                    x,
                    y,
                    if d[2] >= 0.0 {
                        half_height
                    } else {
                        -half_height
                    },
                ]
            }
            Self::Capsule {
                radius,
                half_height,
            } => {
                let tip = if d[2] >= 0.0 {
                    half_height
                } else {
                    -half_height
                };
                let r = scale_to(d, radius);
                [r[0], r[1], r[2] + tip]
            }
        }
    }

    /// The radius of the smallest sphere about the origin containing the shape.
    ///
    /// Used only to bound a search; never as the collision shape itself. A 40 cm link's bounding
    /// sphere is 20 cm in every direction, which is a fine cull and a terrible geometry.
    pub fn bounding_radius(&self) -> f64 {
        match *self {
            Self::Sphere { radius } => radius,
            Self::Cuboid { half } => {
                (half[0] * half[0] + half[1] * half[1] + half[2] * half[2]).sqrt()
            }
            Self::Cylinder {
                radius,
                half_height,
            }
            | Self::Capsule {
                radius,
                half_height,
            } => (radius * radius + half_height * half_height).sqrt(),
        }
    }
}

/// `d` scaled to length `len`, or the zero vector if `d` has no direction.
#[inline]
fn scale_to(d: [f64; 3], len: f64) -> [f64; 3] {
    let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if n < 1e-300 {
        return [0.0; 3];
    }
    [d[0] * len / n, d[1] * len / n, d[2] * len / n]
}

// --- vector helpers -----------------------------------------------------------------------------

#[inline]
fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
#[inline]
fn neg(a: [f64; 3]) -> [f64; 3] {
    [-a[0], -a[1], -a[2]]
}
#[inline]
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
#[inline]
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
#[inline]
fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// `(a x b) x c`, which appears three times in the simplex logic and is easy to write backwards.
#[inline]
fn triple(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> [f64; 3] {
    let ac = dot(a, c);
    let bc = dot(b, c);
    [
        b[0] * ac - a[0] * bc,
        b[1] * ac - a[1] * bc,
        b[2] * ac - a[2] * bc,
    ]
}

// --- one collider -------------------------------------------------------------------------------

/// A shape attached to a link, at an offset within that link's frame.
#[derive(Debug, Clone, Copy)]
pub struct Collider {
    /// Which link of the model this rides on.
    pub link: usize,
    pub shape: Convex,
    /// Where the shape sits in the link's frame. Declared geometry is centred on the link's
    /// `origin`, which is also where its mass sits.
    pub offset: [f64; 3],
}

/// A collider resolved into world coordinates, once.
///
/// # Why this type exists
///
/// A GJK query asks its two shapes for a support point some sixty times, and the obvious way to
/// answer — rotate the direction into the shape's frame with the pose, take the support, rotate the
/// result back — costs two rotor sandwiches per call. A sandwich is two geometric products over
/// eight-component multivectors, about 128 multiplies. That was measured at 1 to 3.5 microseconds
/// per pair, against a whole-model contact budget of about ten.
///
/// Nothing about the pose changes during a query, so it is extracted once: the rotation as a 3x3 and
/// the shape's centre as a point. Every support then costs two matrix-vector products, nine
/// multiplies each.
#[derive(Debug, Clone, Copy)]
pub struct Placement {
    /// Row-major rotation of the link this collider rides on.
    pub rotation: [f64; 9],
    /// The shape's centre in world coordinates.
    pub centre: [f64; 3],
}

impl Placement {
    /// Resolve one collider at one pose.
    ///
    /// The centre goes through the matrix too — `R o + t` — rather than through [`SeFixed::apply`],
    /// which is another rotor sandwich for a vector whose rotation has just been computed anyway.
    pub fn of(collider: &Collider, pose: &Se3) -> Self {
        let mut rotation = [0.0f64; 9];
        pose.rotation_matrix_quaternion_into(&mut rotation);
        let o = mat_vec(&rotation, collider.offset);
        let t = &pose.translation;
        Self {
            rotation,
            centre: [o[0] + t[0], o[1] + t[1], o[2] + t[2]],
        }
    }

    /// The farthest point of `shape` in world direction `d`.
    #[inline]
    pub fn support(&self, shape: &Convex, d: [f64; 3]) -> [f64; 3] {
        let local = mat_t_vec(&self.rotation, d);
        let p = shape.support(local);
        let w = mat_vec(&self.rotation, p);
        [
            w[0] + self.centre[0],
            w[1] + self.centre[1],
            w[2] + self.centre[2],
        ]
    }

    /// A world-axis-aligned box containing the shape, from its bounding sphere.
    ///
    /// Loose on purpose. A tight AABB of a rotated box needs the support function in six
    /// directions; the sphere bound rejects nearly as many pairs at a fraction of the cost, and
    /// everything it lets through meets an exact test anyway.
    #[inline]
    pub fn aabb(&self, shape: &Convex) -> ([f64; 3], [f64; 3]) {
        let r = shape.bounding_radius();
        (
            [self.centre[0] - r, self.centre[1] - r, self.centre[2] - r],
            [self.centre[0] + r, self.centre[1] + r, self.centre[2] + r],
        )
    }
}

/// Resolve every collider at the given link poses, into a caller-owned buffer.
pub fn place(colliders: &[Collider], poses: &[Se3], out: &mut Vec<Placement>) {
    out.clear();
    out.extend(colliders.iter().map(|c| Placement::of(c, &poses[c.link])));
}

#[inline]
fn mat_vec(a: &[f64; 9], v: [f64; 3]) -> [f64; 3] {
    [
        a[0] * v[0] + a[1] * v[1] + a[2] * v[2],
        a[3] * v[0] + a[4] * v[1] + a[5] * v[2],
        a[6] * v[0] + a[7] * v[1] + a[8] * v[2],
    ]
}

#[inline]
fn mat_t_vec(a: &[f64; 9], v: [f64; 3]) -> [f64; 3] {
    [
        a[0] * v[0] + a[3] * v[1] + a[6] * v[2],
        a[1] * v[0] + a[4] * v[1] + a[7] * v[2],
        a[2] * v[0] + a[5] * v[1] + a[8] * v[2],
    ]
}

// --- GJK ----------------------------------------------------------------------------------------

/// What a narrow-phase test found.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Contact {
    /// Signed separation: positive when apart, negative by the penetration depth when overlapping.
    pub separation: f64,
    /// Unit normal pointing from `a` towards `b`. Meaningless when `separation` is not finite.
    pub normal: [f64; 3],
}

/// The most GJK iterations any pair is allowed.
///
/// Convergence is geometric and twenty is already generous for these shapes; the cap exists so that
/// a pathological pair costs a bounded amount rather than stalling the frame it appears in.
const GJK_MAX_ITERATIONS: usize = 32;

/// Do two colliders overlap?
///
/// Boolean GJK: build a simplex in the Minkowski difference `A - B` and ask whether it can be made
/// to contain the origin. Cheaper than the distance query because it stops the moment the answer is
/// known, and it is the only question the broad phase's survivors usually need.
pub fn intersects(a: &Convex, pa: &Placement, b: &Convex, pb: &Placement) -> bool {
    let support = |d: [f64; 3]| sub(pa.support(a, d), pb.support(b, neg(d)));

    let mut simplex = [[0.0f64; 3]; 4];
    let mut n = 1;
    simplex[0] = support([1.0, 0.0, 0.0]);
    if norm(simplex[0]) < 1e-300 {
        return true; // the origin is a support point: the sets touch
    }
    let mut dir = neg(simplex[0]);

    for _ in 0..GJK_MAX_ITERATIONS {
        let p = support(dir);
        if dot(p, dir) < 0.0 {
            return false; // a supporting plane separates them
        }
        // shift and insert
        simplex[n] = p;
        n += 1;
        match evolve(&mut simplex, &mut n, &mut dir) {
            Evolution::Contains => return true,
            Evolution::Continue => {}
        }
    }
    // Out of iterations without a separating plane. Reporting "touching" is the safe direction: a
    // spurious contact is a small spurious force, a missed one is a body passing through another.
    true
}

/// The same query starting from poses, for a caller with one pair to test rather than a frame of
/// them. Resolves both placements and throws them away; inside a loop, resolve once with [`place`].
pub fn intersects_at(a: &Collider, xa: &Se3, b: &Collider, xb: &Se3) -> bool {
    intersects(
        &a.shape,
        &Placement::of(a, xa),
        &b.shape,
        &Placement::of(b, xb),
    )
}

enum Evolution {
    Contains,
    Continue,
}

/// One step of the simplex: drop the vertices that cannot contribute and pick the next direction.
fn evolve(s: &mut [[f64; 3]; 4], n: &mut usize, dir: &mut [f64; 3]) -> Evolution {
    match *n {
        2 => {
            let (b, a) = (s[0], s[1]); // `a` is the newest
            let ab = sub(b, a);
            let ao = neg(a);
            if dot(ab, ao) > 0.0 {
                *dir = triple(ab, ao, ab);
                if norm(*dir) < 1e-14 {
                    // the origin lies on the segment
                    return Evolution::Contains;
                }
            } else {
                s[0] = a;
                *n = 1;
                *dir = ao;
            }
            Evolution::Continue
        }
        3 => {
            let (c, b, a) = (s[0], s[1], s[2]);
            let (ab, ac, ao) = (sub(b, a), sub(c, a), neg(a));
            let abc = cross(ab, ac);

            if dot(cross(abc, ac), ao) > 0.0 {
                if dot(ac, ao) > 0.0 {
                    s[0] = c;
                    s[1] = a;
                    *n = 2;
                    *dir = triple(ac, ao, ac);
                } else {
                    s[0] = b;
                    s[1] = a;
                    *n = 2;
                    return evolve(s, n, dir);
                }
            } else if dot(cross(ab, abc), ao) > 0.0 {
                s[0] = b;
                s[1] = a;
                *n = 2;
                return evolve(s, n, dir);
            } else if dot(abc, ao) > 0.0 {
                *dir = abc;
            } else {
                s.swap(0, 1); // flip the winding so the tetrahedron case sees a consistent normal
                *dir = neg(abc);
            }
            Evolution::Continue
        }
        4 => {
            let (d, c, b, a) = (s[0], s[1], s[2], s[3]);
            let (ab, ac, ad, ao) = (sub(b, a), sub(c, a), sub(d, a), neg(a));
            let abc = cross(ab, ac);
            let acd = cross(ac, ad);
            let adb = cross(ad, ab);

            if dot(abc, ao) > 0.0 {
                s[0] = c;
                s[1] = b;
                s[2] = a;
                *n = 3;
                return evolve(s, n, dir);
            }
            if dot(acd, ao) > 0.0 {
                s[0] = d;
                s[1] = c;
                s[2] = a;
                *n = 3;
                return evolve(s, n, dir);
            }
            if dot(adb, ao) > 0.0 {
                s[0] = b;
                s[1] = d;
                s[2] = a;
                *n = 3;
                return evolve(s, n, dir);
            }
            Evolution::Contains
        }
        _ => {
            *dir = neg(s[0]);
            Evolution::Continue
        }
    }
}

// --- the broad phase ----------------------------------------------------------------------------

/// Which pairs of colliders are worth an exact test.
///
/// Built once from the model, because the answer does not depend on the pose: a joint's parent and
/// child and same-link colliders are excluded by the existing structural contract.
#[derive(Debug, Clone)]
pub struct BroadPhase {
    links: Vec<usize>,
    adjacent: Vec<(usize, usize)>,
    pairs: std::sync::OnceLock<Vec<(usize, usize)>>,
}

impl BroadPhase {
    /// Every collider pair except those the model says cannot meaningfully collide.
    ///
    /// The exclusions are structural, and they come from the incidence relation rather than from a
    /// hand-maintained list:
    ///
    /// * **a joint's parent and child**, which share the joint's location by definition — testing
    ///   them reports a permanent collision, not a real one;
    /// * **two colliders on the same link**, which are rigidly attached and cannot move relative to
    ///   each other, so whatever they do they do forever.
    pub fn new(model: &KinematicModel, colliders: &[Collider]) -> Self {
        Self {
            links: colliders.iter().map(|c| c.link).collect(),
            adjacent: adjacency(model),
            pairs: std::sync::OnceLock::new(),
        }
    }

    /// The candidate pairs, in a fixed order.
    /// Explicit diagnostic: materializes O(n²) structural pairs on first call.
    /// Normal collision queries never request this exhaustive list.
    pub fn pairs(&self) -> &[(usize, usize)] {
        self.pairs.get_or_init(|| {
            let mut pairs = Vec::new();
            for i in 0..self.links.len() {
                for j in i + 1..self.links.len() {
                    if self.allowed(i, j) {
                        pairs.push((i, j));
                    }
                }
            }
            pairs
        })
    }

    fn allowed(&self, i: usize, j: usize) -> bool {
        let (a, b) = (self.links[i], self.links[j]);
        a != b && self.adjacent.binary_search(&ordered(a, b)).is_err()
    }

    /// Replace `out` with the overlapping, structurally allowed pairs in fixed order.
    ///
    /// Output and AABB buffers are reused. Graph builds O(n) transient octree scratch
    /// from current placements, so motion cannot leave a stale spatial index.
    /// Placements remain shared with the narrow phase. Explicit `pairs()` diagnostics
    /// are independent and are never materialized by this query.
    pub fn overlapping(
        &self,
        colliders: &[Collider],
        placements: &[Placement],
        boxes: &mut Vec<([f64; 3], [f64; 3])>,
        out: &mut Vec<(usize, usize)>,
    ) {
        boxes.clear();
        out.clear();
        for (c, p) in colliders.iter().zip(placements) {
            boxes.push(p.aabb(&c.shape));
        }
        self.with_bounds(boxes, out);
    }

    /// Query caller-built (including swept) bounds with the same structural exclusions.
    pub(crate) fn with_bounds(
        &self,
        boxes: &[([f64; 3], [f64; 3])],
        out: &mut Vec<(usize, usize)>,
    ) {
        out.clear();
        hymeko_graph::octree::BoundsIndex::new(boxes).for_each_overlap(|i, j| {
            if self.allowed(i, j) {
                out.push((i, j));
            }
        });
        out.sort_unstable();
    }
}

#[inline]
fn ordered(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
}

/// The link pairs a joint connects, as an ordered set.
///
/// Taken from the joints' own parent and child rather than from the kinematic plan, so a joint whose
/// parent is a frame — an anchor to the world — contributes nothing, which is right: the world is
/// not a link and has no collider.
fn adjacency(model: &KinematicModel) -> Vec<(usize, usize)> {
    let index: std::collections::HashMap<&str, usize> = model
        .links
        .iter()
        .enumerate()
        .map(|(i, l)| (l.name.as_str(), i))
        .collect();
    let mut out: Vec<(usize, usize)> = model
        .joints
        .iter()
        .filter_map(|j: &JointInfo| {
            let p = index.get(j.parent_link.as_str())?;
            let c = index.get(j.child_link.as_str())?;
            Some(ordered(*p, *c))
        })
        .collect();
    out.sort_unstable();
    out.dedup();
    out
}

/// Every collider a model declares, in link order.
pub fn colliders_of(model: &KinematicModel) -> Vec<Collider> {
    model
        .links
        .iter()
        .enumerate()
        .filter_map(|(i, l)| {
            let g = l.geometry.as_ref()?;
            let shape = Convex::from_declaration(g.shape, &g.dimensions)?;
            let offset = l
                .origin
                .as_ref()
                .filter(|o| o.len() >= 3)
                .map(|o| [o[0], o[1], o[2]])
                .unwrap_or([0.0; 3]);
            Some(Collider {
                link: i,
                shape,
                offset,
            })
        })
        .collect()
}

#[cfg(test)]
mod broad_tests {
    use super::*;
    #[test]
    fn spatial_queries_preserve_exclusions_without_materializing_diagnostic_pairs() {
        fn send_sync<T: Send + Sync>() {}
        send_sync::<BroadPhase>();
        let broad = BroadPhase {
            links: vec![0, 1, 2, 2],
            adjacent: vec![(0, 1)],
            pairs: std::sync::OnceLock::new(),
        };
        let colliders: Vec<_> = broad
            .links
            .iter()
            .map(|&link| Collider {
                link,
                shape: Convex::Sphere { radius: 1.0 },
                offset: [0.0; 3],
            })
            .collect();
        let placements: Vec<_> = colliders
            .iter()
            .map(|c| Placement::of(c, &Se3::identity()))
            .collect();
        let mut boxes = Vec::new();
        let mut pairs = Vec::new();
        assert!(broad.pairs.get().is_none());
        broad.overlapping(&colliders, &placements, &mut boxes, &mut pairs);
        assert!(broad.pairs.get().is_none());
        assert_eq!(pairs, [(0, 2), (0, 3), (1, 2), (1, 3)]);
        assert_eq!(broad.pairs(), pairs);
        assert!(broad.pairs.get().is_some());
    }
}
