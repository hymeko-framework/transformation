//! Forward dynamics in time linear in the degrees of freedom.
//!
//! # Why this exists
//!
//! The obvious way to get accelerations out of a mechanism is the one every textbook starts with:
//! build the Jacobian of every mass with respect to every coordinate, assemble `M = sum_i m_i
//! J_i^T J_i`, and solve `M qdd = f`. It is correct, it is easy to believe, and it costs a Jacobian
//! (in the worst case `2n + 1` kinematics passes, if the columns are taken by finite differences),
//! an `n^2` assembly and an `n^3` factorisation — for a mechanism whose coupling was already known
//! exactly, in closed form, before any of that started.
//!
//! It was known because the **topology** was already resolved. [`FkPlan`] holds a parent array and
//! a topological order; that is precisely the structure the articulated-body algorithm sweeps, and
//! it is the reason the algorithm is linear. Assembling a mass matrix throws the structure away and
//! then pays a cubic solve to rediscover it numerically.
//!
//! # What the topology buys, concretely
//!
//! * **The Jacobian is sparse and its sparsity is the ancestor relation.** A body responds to a
//!   coordinate only if that coordinate is on the path from the root to it. The dense `n^2`
//!   assembly spends most of its time multiplying structural zeros.
//! * **A joint's effect on its parent is rank one.** So it folds in as the sweep passes
//!   ([`ArticulatedInertia::downdate`]) instead of being assembled and factorised.
//! * **Branches are independent.** The critical path is the tree's *depth*, not `n` — two arms do
//!   not interact until the torso. Nothing here is parallel yet, but the sweeps are written over
//!   the plan's order so that they could be.
//!
//! # The one thing a tree cannot express
//!
//! A closed loop. A tree has none by definition, and a mechanism that has them — a parallel
//! gripper, a four-bar linkage — needs one constraint per independent loop, which is the cyclomatic
//! number of the graph, **not** `n`. That is the only place a dense solve is genuinely owed, and
//! its size is the number of loops. This module handles the spanning tree; the loop closure is a
//! separate problem with its own, much smaller, matrix.
//!
//! # What this is
//!
//! Featherstone's articulated-body algorithm, in body coordinates, over the plan's order. Three
//! sweeps: velocities down, articulated inertias up, accelerations down. Gravity enters as a
//! fictitious base acceleration rather than as a force on every body, which is the standard trick
//! and costs nothing.
//!
//! Reference: R. Featherstone, *Rigid Body Dynamics Algorithms*, Springer 2008, Table 7.1.

use hymeko_clifford::{ArticulatedInertia, Se3, SpatialInertia, Twist, Wrench};

use crate::forward::{joint_motion_se3, joint_origin_se3, FkError, FkPlan};
use crate::kinematics::joints::JointType;
use crate::kinematics::kinematic::KinematicModel;

/// Floor on a joint's articulated inertia, in kg m^2.
///
/// `d = S^T I^A S` is how much inertia the joint actually has to push against. It is zero for a
/// genuinely indeterminate joint — a point mass sitting exactly on its own rotation axis has no
/// inertia about that axis — and the acceleration is then undefined rather than large. This floor
/// is the exact analogue of the ridge term an explicit mass matrix needs on the same configuration,
/// and it is far below any real link's contribution: a 1 kg mass 1 mm off the axis already gives
/// 1e-6.
const INERTIA_FLOOR: f64 = 1e-12;

/// A mechanism prepared for forward dynamics.
///
/// Built once. Everything that does not depend on the joint values — the topology, each joint's
/// motion subspace, each link's spatial inertia — is resolved here, so a step is arithmetic over
/// flat arrays with no lookup, no allocation and no branching on names.
#[derive(Debug, Clone)]
pub struct AbaPlan {
    plan: FkPlan,
    /// Per joint, the motion subspace in the **child's** frame. `None` for a fixed joint, which
    /// transmits force but has no coordinate.
    subspace: Vec<Option<Twist>>,
    /// Per link, its spatial inertia in its own frame.
    inertia: Vec<SpatialInertia>,
    /// Number of joints that actually move.
    movable: usize,
}

/// Working memory for one call to [`AbaPlan::accelerations`].
///
/// Owned by the caller and reused, which is what lets a step allocate nothing. Sized by
/// [`AbaPlan::scratch`].
#[derive(Debug, Clone)]
pub struct AbaScratch {
    /// Each link's pose in its parent's frame.
    x: Vec<Se3>,
    /// Spatial velocity, acceleration, and the velocity-product bias, per link.
    v: Vec<Twist>,
    a: Vec<Twist>,
    c: Vec<Twist>,
    /// Articulated inertia and bias force, per link.
    ia: Vec<ArticulatedInertia>,
    pa: Vec<Wrench>,
    /// The force transmitted through each link's inbound joint. Used by inverse dynamics only.
    f: Vec<Wrench>,
    /// Per joint: `I^A S`, the scalar `S^T I^A S`, and `tau - S^T p^A`.
    u: Vec<Wrench>,
    d: Vec<f64>,
    tau_bias: Vec<f64>,
}

impl AbaPlan {
    /// Resolve a model's topology, joint axes and inertias once.
    ///
    /// Links without a declared mass are treated as massless, which is correct rather than merely
    /// convenient: a massless link still transmits force, and the articulated inertia handles that
    /// with no special case.
    ///
    /// # Errors
    /// Whatever [`FkPlan::new`] rejects — a cycle, a joint naming a link that does not exist, a
    /// mechanism with no root.
    pub fn new(model: &KinematicModel) -> Result<Self, FkError> {
        let plan = FkPlan::new(model)?;

        let subspace: Vec<Option<Twist>> = model
            .joints
            .iter()
            .map(|j| {
                let axis = normalise(j.axis.unwrap_or([0.0, 0.0, 1.0]))?;
                match j.joint_type {
                    JointType::Fixed => None,
                    JointType::Revolute | JointType::Continuous => Some(Twist::revolute(axis)),
                    JointType::Prismatic => Some(Twist::prismatic(axis)),
                }
            })
            .collect();

        let inertia: Vec<SpatialInertia> = model
            .links
            .iter()
            .map(|l| {
                let m = l.mass.unwrap_or(0.0);
                if m <= 0.0 {
                    return SpatialInertia::ZERO;
                }
                let com = l
                    .origin
                    .as_ref()
                    .filter(|o| o.len() >= 3)
                    .map(|o| [o[0], o[1], o[2]])
                    .unwrap_or([0.0; 3]);
                // A point mass, because that is all the declaration carries. Inventing a rotational
                // inertia would be a physical claim the source never made; the moment a link
                // declares one, `SpatialInertia::rigid` takes it here and nothing else changes.
                SpatialInertia::point_mass(m, com)
            })
            .collect();

        let movable = subspace.iter().filter(|s| s.is_some()).count();
        Ok(Self {
            plan,
            subspace,
            inertia,
            movable,
        })
    }

    /// The underlying kinematic plan, for poses.
    pub fn kinematics(&self) -> &FkPlan {
        &self.plan
    }

    /// How many joints actually move. Fixed joints still occupy a slot in the value vectors.
    pub fn movable(&self) -> usize {
        self.movable
    }

    /// The length every value vector must have: one slot per joint, fixed ones included.
    pub fn slots(&self) -> usize {
        self.subspace.len()
    }

    /// Working memory sized for this mechanism.
    pub fn scratch(&self) -> AbaScratch {
        let links = self.inertia.len();
        let joints = self.subspace.len();
        AbaScratch {
            x: vec![Se3::identity(); links],
            v: vec![Twist::ZERO; links],
            a: vec![Twist::ZERO; links],
            c: vec![Twist::ZERO; links],
            ia: vec![ArticulatedInertia::ZERO; links],
            pa: vec![Wrench::ZERO; links],
            f: vec![Wrench::ZERO; links],
            u: vec![Wrench::ZERO; joints],
            d: vec![0.0; joints],
            tau_bias: vec![0.0; joints],
        }
    }

    /// Solve `M(q) qdd + C(q, qd) qd + g(q) = tau` for `qdd`, without ever forming `M`.
    ///
    /// All four value vectors are indexed by **joint slot**, the same indexing
    /// [`FkPlan::zero_values`] produces; a fixed joint's entries are read as zero and written as
    /// zero. `gravity` is the acceleration due to gravity in the base frame — `[0.0, 0.0, -9.81]`
    /// for the usual convention.
    ///
    /// Cost is three sweeps of the tree: `O(n)` in the joints, with a constant that is a handful of
    /// 3x3 products per link. No allocation.
    ///
    /// # Panics
    /// If any slice is shorter than [`Self::slots`] (or, for poses, the link count).
    // The state is three parallel value vectors and the working memory is a fourth thing; bundling
    // them into a struct would hide that the caller owns each one and reuses it, which is the whole
    // reason a step allocates nothing.
    #[allow(clippy::too_many_arguments)]
    pub fn accelerations(
        &self,
        model: &KinematicModel,
        q: &[f64],
        qd: &[f64],
        tau: &[f64],
        gravity: [f64; 3],
        s: &mut AbaScratch,
        qdd: &mut [f64],
    ) {
        let n = self.subspace.len();
        assert!(q.len() >= n, "q must cover every joint");
        assert!(qd.len() >= n, "qd must cover every joint");
        assert!(tau.len() >= n, "tau must cover every joint");
        assert!(qdd.len() >= n, "qdd must cover every joint");

        // Gravity as a fictitious base acceleration. Every body then feels its own weight without
        // an external force being applied to any of them, which is both cheaper and exactly
        // equivalent — the base is fixed, so the fiction never has to be undone.
        let base_a = Twist {
            w: [0.0; 3],
            v: [-gravity[0], -gravity[1], -gravity[2]],
        };
        let root = self.plan.root();

        // --- sweep one, root to leaves: velocities and the velocity-product bias ---
        for &i in &self.plan.order {
            match self.plan.parent[i] {
                None => {
                    s.x[i] = Se3::identity();
                    s.v[i] = Twist::ZERO;
                    s.c[i] = Twist::ZERO;
                }
                Some((ji, p)) => {
                    let joint = &model.joints[ji];
                    let x = joint_origin_se3(joint).compose(&joint_motion_se3(joint, q[ji]));
                    // The world is not a link. A joint that hangs off it hangs off the base, whose
                    // pose is the identity and whose velocity is zero.
                    let vp = if p == usize::MAX {
                        Twist::ZERO
                    } else {
                        s.v[p]
                    };
                    let parent_here = vp.to_child(&x);
                    match self.subspace[ji] {
                        Some(sub) => {
                            let vj = sub.scale(qd[ji]);
                            s.v[i] = parent_here.add(&vj);
                            // The bias a joint's own motion induces. This is the Coriolis and
                            // centrifugal term, and it is exactly what a mass-matrix formulation
                            // built from position Jacobians alone cannot express.
                            s.c[i] = s.v[i].cross_motion(&vj);
                        }
                        None => {
                            s.v[i] = parent_here;
                            s.c[i] = Twist::ZERO;
                        }
                    }
                    s.x[i] = x;
                }
            }
            s.ia[i] = self.inertia[i].articulated();
            s.pa[i] = s.v[i].cross_force(&self.inertia[i].apply(&s.v[i]));
        }

        // --- sweep two, leaves to root: articulated inertias and bias forces ---
        for idx in (0..self.plan.order.len()).rev() {
            let i = self.plan.order[idx];
            let Some((ji, p)) = self.plan.parent[i] else {
                continue;
            };
            let x = s.x[i];

            let (up, pp) = match self.subspace[ji] {
                Some(sub) => {
                    let u = s.ia[i].apply(&sub);
                    let d = (sub.dot(&u)).max(INERTIA_FLOOR);
                    let bias = tau[ji] - sub.dot(&s.pa[i]);
                    s.u[ji] = u;
                    s.d[ji] = d;
                    s.tau_bias[ji] = bias;

                    // Rank one: the joint is free in exactly one direction, so exactly one
                    // direction of the subtree's stiffness is removed. This is the step that makes
                    // the whole recursion linear rather than cubic.
                    let ia = s.ia[i].downdate(&u, 1.0 / d);
                    let pa = s.pa[i].add(&ia.apply(&s.c[i])).add(&u.scale(bias / d));
                    (ia, pa)
                }
                // A fixed joint is a weld: nothing gives way, so nothing is removed.
                None => (s.ia[i], s.pa[i]),
            };

            if p != usize::MAX {
                let contribution = up.to_parent(&x);
                s.ia[p].add_assign(&contribution);
                s.pa[p] = s.pa[p].add(&pp.to_parent(&x));
            }
        }

        // --- sweep three, root to leaves: accelerations ---
        s.a[root] = base_a;
        for &i in &self.plan.order {
            let Some((ji, p)) = self.plan.parent[i] else {
                continue;
            };
            let x = s.x[i];
            let ap = if p == usize::MAX { base_a } else { s.a[p] };
            let here = ap.to_child(&x).add(&s.c[i]);
            match self.subspace[ji] {
                Some(sub) => {
                    let acc = (s.tau_bias[ji] - here.dot(&s.u[ji])) / s.d[ji];
                    qdd[ji] = acc;
                    s.a[i] = here.add(&sub.scale(acc));
                }
                None => {
                    qdd[ji] = 0.0;
                    s.a[i] = here;
                }
            }
        }
    }
}

impl AbaPlan {
    /// Solve `tau = M(q) qdd + C(q, qd) qd + g(q)` — the other direction.
    ///
    /// Given a motion, what has to be applied to produce it. This is what a controller needs and
    /// what [`Self::accelerations`] cannot answer: forward dynamics says where a robot goes, inverse
    /// dynamics says what to do to send it somewhere.
    ///
    /// The recursive Newton-Euler algorithm, and it is **the same two sweeps** the forward solver
    /// already makes, minus the third. Velocities and accelerations propagate down the tree; the
    /// force each body needs is `I a + v x* (I v)`; those forces sum back up, and each joint's
    /// torque is its share along its own axis. `O(n)`, no allocation.
    ///
    /// Three uses it makes possible immediately:
    ///
    /// * **Gravity compensation** — `inverse_dynamics(q, 0, 0)` is exactly the torque that holds a
    ///   mechanism where it is. Without it an arm sags and a controller spends its whole budget
    ///   fighting a force it could have known in closed form.
    /// * **Computed torque** — `inverse_dynamics(q, qd, qdd_wanted)` plus a small feedback term
    ///   follows a trajectory instead of chasing it.
    /// * **The mass matrix**, for anything that genuinely wants one: column `k` is
    ///   `inverse_dynamics(q, 0, e_k)` with gravity off. Still `n` calls, but exact, and with no
    ///   finite differences anywhere.
    ///
    /// # Panics
    /// If any slice is shorter than [`Self::slots`].
    #[allow(clippy::too_many_arguments)]
    pub fn inverse_dynamics(
        &self,
        model: &KinematicModel,
        q: &[f64],
        qd: &[f64],
        qdd: &[f64],
        gravity: [f64; 3],
        s: &mut AbaScratch,
        tau: &mut [f64],
    ) {
        let n = self.subspace.len();
        assert!(q.len() >= n, "q must cover every joint");
        assert!(qd.len() >= n, "qd must cover every joint");
        assert!(qdd.len() >= n, "qdd must cover every joint");
        assert!(tau.len() >= n, "tau must cover every joint");

        let base_a = Twist {
            w: [0.0; 3],
            v: [-gravity[0], -gravity[1], -gravity[2]],
        };
        // --- sweep one, root to leaves: velocities, accelerations, and the force each body needs ---
        for &i in &self.plan.order {
            match self.plan.parent[i] {
                None => {
                    s.x[i] = Se3::identity();
                    s.v[i] = Twist::ZERO;
                    s.a[i] = base_a;
                }
                Some((ji, p)) => {
                    let joint = &model.joints[ji];
                    let x = joint_origin_se3(joint).compose(&joint_motion_se3(joint, q[ji]));
                    let (vp, ap) = if p == usize::MAX {
                        (Twist::ZERO, base_a)
                    } else {
                        (s.v[p], s.a[p])
                    };
                    let v_here = vp.to_child(&x);
                    let a_here = ap.to_child(&x);
                    match self.subspace[ji] {
                        Some(sub) => {
                            let vj = sub.scale(qd[ji]);
                            s.v[i] = v_here.add(&vj);
                            // `v x vJ` is the velocity-product term; `S qdd` is the motion asked for
                            s.a[i] = a_here.add(&sub.scale(qdd[ji])).add(&s.v[i].cross_motion(&vj));
                        }
                        None => {
                            s.v[i] = v_here;
                            s.a[i] = a_here;
                        }
                    }
                    s.x[i] = x;
                }
            }
            s.f[i] = self.inertia[i]
                .apply(&s.a[i])
                .add(&s.v[i].cross_force(&self.inertia[i].apply(&s.v[i])));
        }

        // --- sweep two, leaves to root: the forces sum up, each joint takes its share ---
        for idx in (0..self.plan.order.len()).rev() {
            let i = self.plan.order[idx];
            let Some((ji, p)) = self.plan.parent[i] else {
                continue;
            };
            tau[ji] = match self.subspace[ji] {
                // A fixed joint has no axis to take a share along; whatever it carries goes to the
                // parent whole, and its reported torque is zero rather than undefined.
                Some(sub) => sub.dot(&s.f[i]),
                None => 0.0,
            };
            if p != usize::MAX {
                let up = s.f[i].to_parent(&s.x[i]);
                s.f[p] = s.f[p].add(&up);
            }
        }
    }
}

/// A unit axis, or `None` when the joint declares a degenerate one.
///
/// A zero axis is not a fixed joint — it is a declaration error — but refusing to build the whole
/// mechanism over one would be worse than treating that joint as welded, which is what returning
/// `None` does here and what forward kinematics already does with the same axis.
fn normalise(a: [f64; 3]) -> Option<[f64; 3]> {
    let n = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    if n < 1e-12 {
        return None;
    }
    Some([a[0] / n, a[1] / n, a[2] / n])
}
