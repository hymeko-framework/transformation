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

/// How small `d = S^T I^A S` may be, RELATIVE to the articulated inertia it came from, before the
/// joint is treated as having no inertia about its own axis at all.
///
/// `d` is a difference of terms of size `|I^A|`, so its rounding error is about `eps |I^A|`, and
/// this floor is about forty-five of those. Below it `d` carries no information — and it can
/// perfectly well come out NEGATIVE, because a positive semi-definite matrix that has been through a
/// dozen congruences and downdates is only semi-definite to within rounding.
///
/// Forty-five and not more. The first attempt used `1e-12`, four thousand rounding units, and an
/// overnight fuzz found what that discards: a 0.94 kg mass sitting 1.4 micrometres off its own axis
/// gives `d = 1.9e-12` against a scale of 2.8 — three thousand times the noise, an entirely real
/// number — and it was being thrown away and the joint reported as free. At this floor the same
/// joint resolves to five significant figures instead. The rule is "indistinguishable from zero",
/// which is a property of the arithmetic, not "small", which is a property of nothing.
///
/// # Why this is relative, and why an absolute floor was a bug
///
/// This started as `d.max(1e-12)`, by analogy with the ridge an explicit mass matrix needs on the
/// same configuration. The analogy is wrong, and a structural fuzz found it: clamping turns a
/// negative `d` into a small POSITIVE one, so `1/d` comes out at `1e12` with the wrong sign, and
/// the rank-one downdate `I^A - U U^T/d` then ADDS an enormous term instead of subtracting a
/// negligible one. On a long chain that compounds once per level. The measured cascade, on a
/// twenty-two-link chain with masses spanning `1e-6` to `1e3` kg, was
/// `|I^A|` = 3.9e-8, then 1.6e3, 1.3e4, 3.4e18, 7.5e46, 7.1e105, and infinity — one level at a
/// time. `hymeko_geometry/tests/articulated.rs::a_chain_of_wildly_mismatched_masses_stays_finite`
/// holds the exact mechanism a structural fuzz found it on.
///
/// A relative threshold cannot do that: below it the downdate is skipped entirely, which is also
/// the correct limit. For a positive semi-definite `I^A`, `S^T I^A S = 0` implies `I^A S = 0`, so
/// `U` is zero and the rank-one term it would remove is zero. Nothing is subtracted because there
/// was nothing there — whether the subtree is massless, or merely carries all its mass on the one
/// axis this joint turns about.
const INERTIA_RELATIVE_FLOOR: f64 = 1e-14;

/// The absolute backstop, for a subtree with no inertia at all, where the relative threshold is
/// zero and every comparison against it would pass.
const INERTIA_ABSOLUTE_FLOOR: f64 = 1e-30;

/// The scale an articulated inertia's own entries are judged against: the traces of its two
/// symmetric blocks.
///
/// Six loads rather than a pass over all twenty-seven entries, which measured at a fifth of the
/// whole sweep when it was written the obvious way. It is a valid magnitude because both blocks are
/// positive semi-definite, so their diagonals are non-negative and their traces bound them.
#[inline]
fn scale_of(ia: &ArticulatedInertia) -> f64 {
    ia.aa[0] + ia.aa[4] + ia.aa[8] + ia.ll[0] + ia.ll[4] + ia.ll[8]
}

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
    /// Added to each joint's own inertia `d` before the recursion divides by it.
    ///
    /// Physically this is a rotor's inertia, reflected through a gearbox and seen at the joint —
    /// which is real, often dominant on a geared actuator, and absent from the link geometry that
    /// would otherwise be the only source of `d`.
    ///
    /// It is also where **implicit damping** enters, and that is not a trick. Solving
    /// `M dqdot = h (f - C qdot)` explicitly is stable only while `h < 2 d / c`, and on a real arm
    /// `d` spans nearly five orders of magnitude between the shoulder and a fingertip, so no single
    /// step suits both ends: measured, a 240 Hz step with 0.15 damping sent two of three robots to
    /// NaN within five steps. Taking the damping at the NEW velocity instead gives
    /// `(M + h C) dqdot = h (f - C qdot)`, and `M + h C` differs from `M` by a joint-space diagonal
    /// — which in this recursion is exactly `d + h c`. Unconditionally stable, and it costs an
    /// addition.
    armature: Vec<f64>,
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
        let armature = vec![0.0; subspace.len()];
        Ok(Self {
            plan,
            subspace,
            inertia,
            movable,
            armature,
        })
    }

    /// Set the per-joint inertia added to `d`. See [`Self::armature`].
    ///
    /// For implicit joint damping, pass `step * damping` for each joint. For a geared actuator, pass
    /// the reflected rotor inertia; for both, their sum.
    ///
    /// # Panics
    /// If `armature` does not cover every joint, or if any entry is negative — a negative one would
    /// subtract inertia the mechanism has, and past a point invert the sign of an acceleration.
    pub fn set_armature(&mut self, armature: &[f64]) {
        assert!(
            armature.len() >= self.subspace.len(),
            "armature must cover every joint"
        );
        assert!(
            armature.iter().all(|&a| a >= 0.0),
            "a negative armature removes inertia the mechanism has"
        );
        self.armature.copy_from_slice(&armature[..self.subspace.len()]);
    }

    /// The per-joint inertia currently added to `d`.
    pub fn armature(&self) -> &[f64] {
        &self.armature
    }

    /// How many links the mechanism has.
    pub fn links(&self) -> usize {
        self.inertia.len()
    }

    /// The joint that brings a link in, if any. The root has none, and neither does a link the
    /// mechanism does not reach.
    pub fn inbound_joint(&self, link: usize) -> Option<usize> {
        self.plan.parent.get(link).and_then(|p| p.map(|(ji, _)| ji))
    }

    /// The link a link hangs from, if it hangs from a link at all. `None` for the root and for a
    /// link anchored to a frame rather than to another body.
    pub fn kinematics_parent(&self, link: usize) -> Option<usize> {
        match self.plan.parent.get(link)? {
            Some((_, p)) if *p != usize::MAX => Some(*p),
            _ => None,
        }
    }

    /// The links in an order where every parent precedes its children.
    ///
    /// Exposed so a caller can put a set of links into a sweepable order without knowing how the
    /// topology was resolved.
    pub fn order(&self) -> &[usize] {
        &self.plan.order
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
                    let d = sub.dot(&u) + self.armature[ji];
                    let floor = (INERTIA_RELATIVE_FLOOR * scale_of(&s.ia[i]))
                        .max(INERTIA_ABSOLUTE_FLOOR);
                    if d <= floor {
                        // Nothing below this joint resists it. `d` is zero to within its own
                        // rounding — or negative, which means the same thing and says only that the
                        // accumulated inertia has drifted off positive semi-definite. Either way
                        // `U` is zero to the same accuracy, so the rank-one term is zero and the
                        // joint passes its subtree to the parent unchanged. Dividing by the floor
                        // instead would amplify the rounding by `1/floor` at every level; see
                        // `INERTIA_RELATIVE_FLOOR`.
                        s.u[ji] = Wrench::ZERO;
                        s.d[ji] = 0.0; // the sentinel sweep three reads
                        s.tau_bias[ji] = 0.0;
                        (s.ia[i], s.pa[i])
                    } else {
                        let bias = tau[ji] - sub.dot(&s.pa[i]);
                        s.u[ji] = u;
                        s.d[ji] = d;
                        s.tau_bias[ji] = bias;

                        // Rank one: the joint is free in exactly one direction, so exactly one
                        // direction of the subtree's stiffness is removed. This is the step that
                        // makes the whole recursion linear rather than cubic.
                        let ia = s.ia[i].downdate(&u, 1.0 / d);
                        let pa = s.pa[i].add(&ia.apply(&s.c[i])).add(&u.scale(bias / d));
                        (ia, pa)
                    }
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
                // `d == 0` is the sentinel from sweep two: a joint with no inertia about its own
                // axis. Its acceleration is not determined by anything — no inertia, and no torque
                // that could act on one — so it is zero, and the body simply follows its parent.
                Some(_) if s.d[ji] == 0.0 => {
                    qdd[ji] = 0.0;
                    s.a[i] = here;
                }
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
                //
                // The armature resists an acceleration like any other inertia, so it appears here
                // for the same reason it appears in `d`: the two directions have to agree, or the
                // round trip between them stops closing.
                Some(sub) => sub.dot(&s.f[i]) + self.armature[ji] * qdd[ji],
                None => 0.0,
            };
            if p != usize::MAX {
                let up = s.f[i].to_parent(&s.x[i]);
                s.f[p] = s.f[p].add(&up);
            }
        }
    }
}

impl AbaPlan {
    /// Everything about the mechanism's inertia at one configuration that does not depend on what is
    /// applied to it: the local transforms, the articulated inertias, and each joint's `U` and `d`.
    ///
    /// # Why this is split out
    ///
    /// Building a constraint problem needs `M^-1 J_k^T` for every constraint `k`, and calling
    /// [`Self::accelerations`] once per constraint recomputes all of that every time. With the
    /// velocity and gravity at zero — which is what a mobility column is — the first two sweeps
    /// depend on `q` alone, and only the third depends on the right-hand side.
    ///
    /// The first two sweeps are also where the cost is: a congruence on a general symmetric spatial
    /// inertia is a few hundred multiplies per link, while a wrench or a twist moving between frames
    /// is a couple of dozen. So a mechanism with nineteen contacts was doing the expensive part
    /// nineteen times to get nineteen cheap answers.
    ///
    /// **No mass matrix is formed here.** This is the articulated-body recursion stopped halfway,
    /// which is a different thing: what is kept is one rank-one-downdated inertia per link, not an
    /// `n` by `n` matrix, and there is no factorisation to be conditioned badly.
    ///
    /// Pair with [`Self::solve_factorised`]. Any call to [`Self::accelerations`] or
    /// [`Self::inverse_dynamics`] on the same scratch invalidates it.
    pub fn factorise(&self, model: &KinematicModel, q: &[f64], s: &mut AbaScratch) {
        let n = self.subspace.len();
        assert!(q.len() >= n, "q must cover every joint");

        for &i in &self.plan.order {
            s.x[i] = match self.plan.parent[i] {
                None => Se3::identity(),
                Some((ji, _)) => {
                    let joint = &model.joints[ji];
                    joint_origin_se3(joint).compose(&joint_motion_se3(joint, q[ji]))
                }
            };
            // At rest there is no velocity product and no bias force; both sweeps below start clean.
            s.v[i] = Twist::ZERO;
            s.c[i] = Twist::ZERO;
            s.ia[i] = self.inertia[i].articulated();
            s.pa[i] = Wrench::ZERO;
        }

        for idx in (0..self.plan.order.len()).rev() {
            let i = self.plan.order[idx];
            let Some((ji, p)) = self.plan.parent[i] else {
                continue;
            };
            let up = match self.subspace[ji] {
                Some(sub) => {
                    let u = s.ia[i].apply(&sub);
                    let d = sub.dot(&u) + self.armature[ji];
                    let floor =
                        (INERTIA_RELATIVE_FLOOR * scale_of(&s.ia[i])).max(INERTIA_ABSOLUTE_FLOOR);
                    if d <= floor {
                        s.u[ji] = Wrench::ZERO;
                        s.d[ji] = 0.0;
                        s.ia[i]
                    } else {
                        s.u[ji] = u;
                        s.d[ji] = d;
                        s.ia[i].downdate(&u, 1.0 / d)
                    }
                }
                None => s.ia[i],
            };
            if p != usize::MAX {
                s.ia[p].add_assign(&up.to_parent(&s.x[i]));
            }
        }
    }

    /// The links from `body` up to the root, `body` first.
    ///
    /// This is the support of a constraint's generalised force, and it is a **path**: a force at a
    /// point on one body is felt by the joints between it and the root, and by no others. The set is
    /// closed under taking parents, which is what lets a union of several of them be swept in the
    /// plan's own order.
    ///
    /// # Panics
    /// If `body` is not a link.
    pub fn reach(&self, body: usize, out: &mut Vec<usize>) {
        assert!(body < self.inertia.len(), "body {body} is not a link");
        out.clear();
        let mut here = body;
        out.push(here);
        while let Some((_, parent)) = self.plan.parent[here] {
            if parent == usize::MAX {
                break; // anchored to the world, which is not a link
            }
            out.push(parent);
            here = parent;
        }
    }

    /// [`Self::solve_factorised`] restricted to the parts of the tree that can matter.
    ///
    /// # The two sets, and why they differ
    ///
    /// `up` are the links whose inbound joint carries any of the applied torque — for a contact,
    /// its [`Self::reach`]. Everywhere else the bias force stays zero, so sweeping those links is
    /// arithmetic on zeros.
    ///
    /// `down` is where the answer is wanted, and it is **not** the same set and cannot be. An
    /// acceleration introduced at one joint propagates to that link's whole subtree, so
    /// `M^-1 J^T` is genuinely dense however sparse `J` is. What saves the work is that a
    /// constraint problem never reads all of it: `A[k][l] = J_k . (M^-1 J_l^T)` and `J_k` is
    /// supported on `reach(k)`, so the union of the reaches is all any of them will look at.
    ///
    /// # Preconditions
    ///
    /// `up` must be ordered deepest first and `down` in the plan's own order, `down` must contain
    /// `up`, and `down` must be closed under taking parents. A union of reaches satisfies all three.
    /// Entries of `qdd` outside `down` are left as they were — stale, and not to be read.
    ///
    /// # Panics
    /// If a slice is shorter than [`Self::slots`].
    pub fn solve_factorised_over(
        &self,
        tau: &[f64],
        up: &[usize],
        down: &[usize],
        s: &mut AbaScratch,
        qdd: &mut [f64],
    ) {
        let n = self.subspace.len();
        assert!(tau.len() >= n, "tau must cover every joint");
        assert!(qdd.len() >= n, "qdd must cover every joint");

        // Only what the sweeps will touch. `up` is inside `down`, so clearing `down` clears both.
        for &i in down {
            s.pa[i] = Wrench::ZERO;
            if let Some((ji, _)) = self.plan.parent[i] {
                s.tau_bias[ji] = 0.0;
            }
        }

        for &i in up {
            let Some((ji, p)) = self.plan.parent[i] else {
                continue;
            };
            let pp = match self.subspace[ji] {
                Some(_) if s.d[ji] == 0.0 => s.pa[i],
                Some(sub) => {
                    let bias = tau[ji] - sub.dot(&s.pa[i]);
                    s.tau_bias[ji] = bias;
                    s.pa[i].add(&s.u[ji].scale(bias / s.d[ji]))
                }
                None => s.pa[i],
            };
            if p != usize::MAX {
                s.pa[p] = s.pa[p].add(&pp.to_parent(&s.x[i]));
            }
        }

        s.a[self.plan.root()] = Twist::ZERO;
        for &i in down {
            let Some((ji, p)) = self.plan.parent[i] else {
                continue;
            };
            let ap = if p == usize::MAX {
                Twist::ZERO
            } else {
                s.a[p]
            };
            let here = ap.to_child(&s.x[i]);
            match self.subspace[ji] {
                Some(_) if s.d[ji] == 0.0 => {
                    qdd[ji] = 0.0;
                    s.a[i] = here;
                }
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

    /// `qdd = M(q)^-1 tau` using a factorisation from [`Self::factorise`], at zero velocity and zero
    /// gravity — one column of the mobility matrix, and nothing else.
    ///
    /// Two cheap sweeps: the applied torques become bias forces on the way up, and accelerations on
    /// the way down. No inertia is touched.
    ///
    /// # Panics
    /// If a slice is shorter than [`Self::slots`].
    pub fn solve_factorised(&self, tau: &[f64], s: &mut AbaScratch, qdd: &mut [f64]) {
        let n = self.subspace.len();
        assert!(tau.len() >= n, "tau must cover every joint");
        assert!(qdd.len() >= n, "qdd must cover every joint");

        for &i in &self.plan.order {
            s.pa[i] = Wrench::ZERO;
        }

        for idx in (0..self.plan.order.len()).rev() {
            let i = self.plan.order[idx];
            let Some((ji, p)) = self.plan.parent[i] else {
                continue;
            };
            let pp = match self.subspace[ji] {
                Some(_) if s.d[ji] == 0.0 => {
                    s.tau_bias[ji] = 0.0;
                    s.pa[i]
                }
                Some(sub) => {
                    let bias = tau[ji] - sub.dot(&s.pa[i]);
                    s.tau_bias[ji] = bias;
                    s.pa[i].add(&s.u[ji].scale(bias / s.d[ji]))
                }
                None => s.pa[i],
            };
            if p != usize::MAX {
                s.pa[p] = s.pa[p].add(&pp.to_parent(&s.x[i]));
            }
        }

        s.a[self.plan.root()] = Twist::ZERO;
        for &i in &self.plan.order {
            let Some((ji, p)) = self.plan.parent[i] else {
                continue;
            };
            let ap = if p == usize::MAX {
                Twist::ZERO
            } else {
                s.a[p]
            };
            let here = ap.to_child(&s.x[i]);
            match self.subspace[ji] {
                Some(_) if s.d[ji] == 0.0 => {
                    qdd[ji] = 0.0;
                    s.a[i] = here;
                }
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

    /// Fill the scratch's local transforms for a configuration.
    ///
    /// Split out of [`Self::constraint_force`] because every constraint at one configuration shares
    /// them, and rebuilding them per constraint is a full forward-kinematics pass per contact.
    pub fn prepare_transforms(&self, model: &KinematicModel, q: &[f64], s: &mut AbaScratch) {
        for &i in &self.plan.order {
            s.x[i] = match self.plan.parent[i] {
                None => Se3::identity(),
                Some((ji, _)) => {
                    let joint = &model.joints[ji];
                    joint_origin_se3(joint).compose(&joint_motion_se3(joint, q[ji]))
                }
            };
        }
    }

    /// The joint torques a unit force along `world_direction`, applied at `world_point` on `body`,
    /// produces — one column of the constraint Jacobian's transpose.
    ///
    /// # Why this and not a Jacobian
    ///
    /// A constraint needs `J^T lambda`, not `J`. Building `J` means a column per coordinate; this
    /// needs one **upward sweep from the contact to the root**, which is `O(depth)` and touches only
    /// the joints actually between the two. A foot's contact does not reach the other three legs,
    /// and nothing here spends time discovering that.
    ///
    /// It is the second sweep of inverse dynamics with everything at rest and one external wrench,
    /// which is why it shares the machinery rather than adding any.
    ///
    /// `poses` are the world poses from [`FkPlan::resolve_se3`]; `tau` is written in full, zero on
    /// every joint the contact cannot reach.
    ///
    /// **[`Self::prepare_transforms`] or [`Self::factorise`] must have been called on `s` at this
    /// configuration first**, which is why neither the model nor `q` appears here: this walks a
    /// prepared scratch. The local transforms do not depend on which contact is being resolved, and
    /// a mechanism with nineteen contacts was rebuilding all of them nineteen times.
    ///
    /// # Panics
    /// If `body` is not a link, or if a slice is shorter than the mechanism needs.
    #[allow(clippy::too_many_arguments)]
    pub fn constraint_force(
        &self,
        poses: &[Se3],
        body: usize,
        world_point: [f64; 3],
        world_direction: [f64; 3],
        s: &mut AbaScratch,
        tau: &mut [f64],
    ) {
        let n = self.subspace.len();
        assert!(tau.len() >= n, "tau must cover every joint");
        assert!(body < self.inertia.len(), "body {body} is not a link");
        tau[..n].fill(0.0);

        // The applied wrench in the contacting body's OWN frame: a pure force through a point is a
        // force and the moment that point takes about the frame's origin.
        let inverse = poses[body].inverse();
        let dir = inverse.rotate(&world_direction);
        let at = inverse.apply(&world_point);
        let force = [dir[0], dir[1], dir[2]];
        let moment = [
            at[1] * force[2] - at[2] * force[1],
            at[2] * force[0] - at[0] * force[2],
            at[0] * force[1] - at[1] * force[0],
        ];
        let mut carried = Wrench {
            n: moment,
            f: force,
        };

        // Up the tree, one joint at a time. Each joint takes its share along its own axis and hands
        // the rest to its parent; a fixed joint takes nothing and hands on everything.
        let mut here = body;
        while let Some((ji, parent)) = self.plan.parent[here] {
            if let Some(sub) = self.subspace[ji] {
                tau[ji] = sub.dot(&carried);
            }
            if parent == usize::MAX {
                break; // anchored to the world, which absorbs whatever is left
            }
            carried = carried.to_parent(&s.x[here]);
            here = parent;
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
