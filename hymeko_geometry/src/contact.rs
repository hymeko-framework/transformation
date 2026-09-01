//! Contacts, as the extra edges a spanning tree does not have.
//!
//! # Where this sits
//!
//! The articulated-body algorithm solves a **tree** in time linear in its joints. A tree has no
//! cycles, which is exactly why it can be solved by sweeping. A contact closes a cycle — through the
//! ground, or through another body — and that is the one thing a sweep cannot absorb.
//!
//! So the work splits along the structure, and the split is the point:
//!
//! * the **spanning tree** costs `O(n)` and is [`AbaPlan::accelerations`];
//! * the **extra edges** cost a dense solve whose size is the number of those edges — the graph's
//!   cyclomatic number — and **not** the number of joints.
//!
//! A quadruped with four feet on the ground is four constraints, whether it has twenty-three joints
//! or two hundred. That is the whole reason to say any of this in terms of a hypergraph rather than
//! in terms of a mass matrix.
//!
//! # The formulation
//!
//! At the velocity level, which is what every stable engine actually integrates:
//!
//! ```text
//! qdot_plus = qdot_free + M^-1 J^T p,    A p + b >= 0,   p >= 0,   p . (A p + b) = 0
//! ```
//!
//! with `A = J M^-1 J^T` the Delassus operator and `qdot_free` the velocity a step would reach with
//! no contact. Working in velocities rather than accelerations avoids needing `Jdot qdot`, which is
//! the term that is fiddly to compute and easy to get subtly wrong; a penetration that has already
//! happened is corrected by a bias term instead, which is both simpler and more forgiving of a large
//! step.
//!
//! `A` is built one column at a time, and **a column is one articulated-body solve**:
//! `M^-1 J_k^T` is what the mechanism does when the only torque applied is the one that constraint
//! exerts. So building `A` costs `c` sweeps and `c^2` dot products, and never forms `M`.

use hymeko_clifford::Se3;

use crate::articulated::{AbaPlan, AbaScratch};
use crate::collide::{Collider, Placement};

/// One scalar constraint on the mechanism.
#[derive(Debug, Clone)]
pub struct Constraint {
    /// One column of `J^T`: the joint torques a unit multiplier produces, from
    /// [`AbaPlan::constraint_force`].
    pub force: Vec<f64>,
    /// Added to the constraint's own velocity before the inequality is applied. Negative pushes the
    /// bodies apart; this is where penetration correction and restitution enter.
    pub bias: f64,
    /// A contact pushes and never pulls, so its multiplier is non-negative. A loop closure holds
    /// both ways and its multiplier is free.
    pub unilateral: bool,
}

/// A contact found against the ground plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroundContact {
    /// Which collider is touching.
    pub collider: usize,
    /// The lowest point, in world coordinates.
    pub point: [f64; 3],
    /// Signed gap to the plane. **Positive is apart**, negative is penetrating.
    pub separation: f64,
}

/// Every collider whose lowest point is within `margin` of `height`, above or below.
///
/// **Exact, and one support evaluation each.** The lowest point of a convex body is its support in
/// `-z`, by definition, so a plane contact needs no iteration and no penetration-depth search — the
/// two things that make general body-body contact expensive. For a walking machine the ground is the
/// contact that matters, and it is the cheap one.
///
/// # Why a margin, and why the contact is kept while still apart
///
/// A contact created only after the bodies have already overlapped is a contact created too late:
/// the step that produced the overlap has already happened, and the solver's job becomes undoing it
/// rather than preventing it. A body exactly touching would also be missed, since a strict `below the
/// plane` test excludes the boundary it is standing on.
///
/// So a collider within `margin` gets a constraint while it is still apart, and the constraint's
/// bias then permits exactly the approach that closes the remaining gap in one step and no more —
/// see [`ground_constraints`]. A body a millimetre up still falls that millimetre; it just cannot
/// pass through.
pub fn ground_contacts(
    colliders: &[Collider],
    placements: &[Placement],
    height: f64,
    margin: f64,
    out: &mut Vec<GroundContact>,
) {
    out.clear();
    for (i, (c, p)) in colliders.iter().zip(placements).enumerate() {
        let low = p.support(&c.shape, [0.0, 0.0, -1.0]);
        let separation = low[2] - height;
        if separation <= margin {
            out.push(GroundContact {
                collider: i,
                point: low,
                separation,
            });
        }
    }
}

/// How a penetration is worked off and how hard a contact bounces.
#[derive(Debug, Clone, Copy)]
pub struct ContactParameters {
    /// Fraction of the remaining penetration removed per step, in `[0, 1]`.
    ///
    /// One would remove it all in a single step, which turns a body that has been pushed slightly
    /// into the floor into a body that leaps off it. A fifth is the usual compromise.
    pub restitution_of_position: f64,
    /// Penetration tolerated without correction. Without it, a resting body chatters between
    /// touching and not touching, and the contact set changes every step.
    pub slop: f64,
    /// How far above the plane a collider still gets a constraint. See [`ground_contacts`].
    pub margin: f64,
    /// Bounciness, in `[0, 1]`. Zero is a dead landing.
    pub restitution: f64,
    /// Iterations of the projected Gauss-Seidel sweep.
    pub iterations: usize,
}

impl Default for ContactParameters {
    fn default() -> Self {
        Self {
            restitution_of_position: 0.2,
            slop: 1e-4,
            margin: 1e-3,
            restitution: 0.0,
            iterations: 30,
        }
    }
}

/// Working memory and results for one step's worth of constraints.
///
/// Everything is a caller-owned buffer that grows to the largest contact set seen and is then
/// reused, so a step with a steady contact set allocates nothing.
#[derive(Debug, Default, Clone)]
pub struct ConstraintSolver {
    /// The Delassus operator `A = J M^-1 J^T`, row-major, `c` by `c`.
    a: Vec<f64>,
    /// `b_k = J_k qdot_free + bias_k`.
    b: Vec<f64>,
    /// The multipliers — impulses, in newton-seconds for a contact.
    impulse: Vec<f64>,
    /// `M^-1 J^T`, one column of length `slots` per constraint.
    response: Vec<f64>,
    scratch: Vec<f64>,
}

impl ConstraintSolver {
    /// A solver with no memory allocated yet.
    pub fn new() -> Self {
        Self::default()
    }

    /// The impulses from the last [`Self::solve`], one per constraint.
    pub fn impulses(&self) -> &[f64] {
        &self.impulse
    }

    /// The Delassus operator from the last solve, row-major.
    ///
    /// Exposed because it is the interesting object: symmetric, positive semidefinite, and of size
    /// the constraint count rather than the coordinate count.
    pub fn delassus(&self) -> &[f64] {
        &self.a
    }

    /// Solve for the constraint impulses and apply them to `velocity`.
    ///
    /// `velocity` arrives as the free velocity — what the step would reach with no contact — and
    /// leaves corrected. Returns the number of constraints that ended up carrying load, which is a
    /// cheaper thing to watch than the impulses themselves.
    ///
    /// # The scratch must already be factorised at this `q`
    ///
    /// Either [`AbaPlan::factorise`] or [`AbaPlan::accelerations`] establishes that, and a physics
    /// step has already called the second one to get the free velocity — the articulated inertias,
    /// `U` and `d` it leaves behind are exactly what this needs. Refactorising here would be a
    /// second full inertia sweep per step for nothing.
    ///
    /// [`AbaPlan::inverse_dynamics`] does NOT establish it: it never forms an articulated inertia.
    ///
    /// Neither the model nor `q` appears here, for the same reason they no longer appear on
    /// [`AbaPlan::constraint_force`]: this walks a prepared scratch, and taking them would suggest
    /// otherwise.
    ///
    /// # Panics
    /// If any constraint's force vector is shorter than the plan's slot count.
    pub fn solve(
        &mut self,
        aba: &AbaPlan,
        constraints: &[Constraint],
        parameters: &ContactParameters,
        scratch: &mut AbaScratch,
        velocity: &mut [f64],
    ) -> usize {
        let c = constraints.len();
        let n = aba.slots();
        if c == 0 {
            return 0;
        }

        self.a.clear();
        self.a.resize(c * c, 0.0);
        self.b.clear();
        self.b.resize(c, 0.0);
        self.impulse.clear();
        self.impulse.resize(c, 0.0);
        self.response.clear();
        self.response.resize(c * n, 0.0);
        self.scratch.clear();
        self.scratch.resize(n, 0.0);

        // One column of `M^-1 J^T` per constraint: what the mechanism does when the ONLY torque is
        // the one that constraint exerts. Gravity off and velocity zero, because this is the
        // mechanism's response to a unit impulse and nothing else.
        // The factorisation is the caller's, and it is ALREADY THERE: a step computes the free
        // acceleration first, and `accelerations` leaves exactly the same articulated inertias, `U`
        // and `d` in the scratch that `factorise` would. Redoing it here was a second full inertia
        // sweep per step, comparable in cost to the dynamics itself.
        for (k, constraint) in constraints.iter().enumerate() {
            assert!(constraint.force.len() >= n, "constraint {k} is too short");
            aba.solve_factorised(&constraint.force, scratch, &mut self.scratch);
            self.response[k * n..k * n + n].copy_from_slice(&self.scratch[..n]);
        }

        for (k, constraint) in constraints.iter().enumerate() {
            for l in 0..c {
                let column = &self.response[l * n..l * n + n];
                self.a[k * c + l] = constraint
                    .force
                    .iter()
                    .zip(column)
                    .map(|(f, r)| f * r)
                    .sum();
            }
            self.b[k] = constraint
                .force
                .iter()
                .zip(velocity.iter())
                .map(|(f, v)| f * v)
                .sum::<f64>()
                + constraint.bias;
        }

        // Projected Gauss-Seidel. Each sweep visits the constraints in turn and moves one multiplier
        // to satisfy its own row exactly, projecting it back onto the sign it is allowed. It
        // converges for a positive semidefinite `A`, which this is; it is what most engines actually
        // run, and it degrades into a slightly soft contact rather than into an explosion when it
        // runs out of iterations.
        for _ in 0..parameters.iterations {
            for (k, constraint) in constraints.iter().enumerate() {
                let row = &self.a[k * c..k * c + c];
                let diagonal = row[k];
                if diagonal <= 1e-12 {
                    continue; // this constraint has no inertia behind it to push against
                }
                let residual: f64 = self.b[k]
                    + row
                        .iter()
                        .zip(&self.impulse)
                        .map(|(a, p)| a * p)
                        .sum::<f64>();
                let mut next = self.impulse[k] - residual / diagonal;
                if constraint.unilateral && next < 0.0 {
                    next = 0.0;
                }
                self.impulse[k] = next;
            }
        }

        for (k, _) in constraints.iter().enumerate() {
            let p = self.impulse[k];
            if p == 0.0 {
                continue;
            }
            let column = &self.response[k * n..k * n + n];
            for (v, r) in velocity.iter_mut().zip(column) {
                *v += p * r;
            }
        }

        self.impulse.iter().filter(|p| **p != 0.0).count()
    }
}

/// Turn ground contacts into constraints, one per contact, along the plane's normal.
///
/// `velocity` is the mechanism's current joint velocities, which the restitution term needs.
///
/// **The scratch must already hold this configuration's local transforms**, from
/// [`AbaPlan::prepare_transforms`], [`AbaPlan::factorise`] or [`AbaPlan::accelerations`]. A step has
/// already called the last of those; rebuilding them here is a forward-kinematics pass for nothing.
///
/// The bias is where a penetration that has already happened gets worked off and where bounce
/// enters. Both are velocities: a step of `h` removes `beta/h` of the penetration beyond the slop,
/// so the correction is a rate rather than a jump.
///
/// # Panics
/// If a contact names a collider that is not in `colliders`.
#[allow(clippy::too_many_arguments)]
pub fn ground_constraints(
    aba: &AbaPlan,
    poses: &[Se3],
    colliders: &[Collider],
    contacts: &[GroundContact],
    velocity: &[f64],
    parameters: &ContactParameters,
    step: f64,
    scratch: &mut AbaScratch,
    out: &mut Vec<Constraint>,
) {
    out.clear();
    let n = aba.slots();
    for contact in contacts {
        let mut force = vec![0.0; n];
        aba.constraint_force(
            poses,
            colliders[contact.collider].link,
            contact.point,
            [0.0, 0.0, 1.0],
            scratch,
            &mut force,
        );
        // The bias is one number doing two jobs, and the sign of the separation says which.
        //
        //   apart      the constraint must permit closing the gap, and no more: a body a millimetre
        //              up may approach at `gap / h` and arrive exactly on the plane.
        //   penetrating a fraction of the overlap beyond the slop is worked off per step, as a
        //              velocity rather than a jump, so a body pushed into the floor rises out of it
        //              instead of being flung.
        let bias_position = if contact.separation > 0.0 {
            contact.separation / step
        } else {
            let overlap = (-contact.separation - parameters.slop).max(0.0);
            -parameters.restitution_of_position * overlap / step
        };
        // `J qdot` is how fast the contact point is closing on the plane, and it falls out of the
        // constraint's own force vector — the same row, read the other way.
        let approaching = force
            .iter()
            .zip(velocity)
            .map(|(f, v)| f * v)
            .sum::<f64>()
            .min(0.0);
        out.push(Constraint {
            force,
            bias: bias_position + parameters.restitution * approaching,
            unilateral: true,
        });
    }
}
