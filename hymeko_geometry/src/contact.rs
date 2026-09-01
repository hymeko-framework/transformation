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

/// Below this, `A[k][k]` means the mechanism has no mobility along the constraint at all.
///
/// `A[k][k] = J_k M^-1 J_k^T` is the inverse of the mass the constraint feels, so zero means
/// infinite mass — a welded body, or one whose only joint cannot move it in that direction.
const MOBILITY_FLOOR: f64 = 1e-12;

/// How a multiplier is limited.
///
/// A contact is three constraints, not one: the normal, which pushes and never pulls, and two
/// tangent directions whose impulse is bounded by Coulomb's law. They are separate rows of the same
/// problem because they act along different directions, and they are coupled because the bound on
/// the tangents is set by the normal's answer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Limit {
    /// `p >= 0`. A contact normal pushes and never pulls.
    Pushing,
    /// Unbounded. A loop closure or a bilateral joint limit holds in both directions.
    Free,
    /// One of a tangent pair inside a friction cone: `|(p_self, p_partner)| <= mu * p[normal]`.
    ///
    /// The pair is confined to a DISC rather than clamped separately into a square.
    /// A square is the pyramidal cone most engines use, and it makes friction depend on the
    /// direction of sliding — a block slides more easily along a diagonal than along an axis, which
    /// is an artefact of the discretisation and not of the physics.
    Friction {
        /// Index of the normal constraint whose impulse bounds this pair.
        normal: usize,
        /// The other tangent of the pair.
        partner: usize,
        /// Coefficient of friction.
        mu: f64,
    },
}

/// One scalar constraint on the mechanism.
#[derive(Debug, Clone)]
pub struct Constraint {
    /// One column of `J^T`: the joint torques a unit multiplier produces, from
    /// [`AbaPlan::constraint_force`].
    ///
    /// Dense storage, sparse content: see [`Self::reach`].
    pub force: Vec<f64>,
    /// Where `force` can be non-zero — the links between the contacting body and the root.
    ///
    /// A force at a point on one body is felt by the joints between it and the root and by no
    /// others, so this is the support of the column, and it is a path. Carrying it turns two `O(n)`
    /// operations into `O(depth)` ones: the sweep that builds the mobility column, and the dot
    /// product that turns two columns into an entry of the Delassus operator.
    pub reach: Vec<usize>,
    /// Added to the constraint's own velocity before the inequality is applied. Negative pushes the
    /// bodies apart; this is where penetration correction and restitution enter.
    pub bias: f64,
    /// What bounds this constraint's multiplier.
    pub limit: Limit,
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
    /// Coefficient of Coulomb friction. Zero makes every contact frictionless and drops the two
    /// tangent rows entirely, which is a third of the constraint problem.
    pub friction: f64,
    /// Most sweeps of projected Gauss-Seidel to make. A cap, not a target — see [`Self::tolerance`].
    pub iterations: usize,
    /// Stop once no impulse moved by more than this fraction of the largest one.
    ///
    /// The change in the impulses is free to watch — it falls out of the sweep — where the residual
    /// itself would cost another `c^2` per check.
    ///
    /// # The two regimes, measured
    ///
    /// **Frictionless**, which is a linear complementarity problem: machine precision in about ten
    /// sweeps, on a standing quadruped and on a nineteen-contact pile alike.
    ///
    /// **With friction**, which is a cone complementarity problem and genuinely harder: the residual
    /// falls 1.2e-1 at ten sweeps, 5.8e-3 at thirty, 8.8e-5 at sixty, 2.1e-8 at a hundred and
    /// twenty, and reaches machine precision around five hundred. The bound on the tangent pair
    /// moves as the normal impulse does, and that coupling is what costs the sweeps.
    ///
    /// So the cap matters here in a way it did not before, and what it buys is legible rather than
    /// guessed at. Zero disables the early exit and makes [`Self::iterations`] a target again.
    pub tolerance: f64,
}

impl Default for ContactParameters {
    fn default() -> Self {
        Self {
            restitution_of_position: 0.2,
            slop: 1e-4,
            margin: 1e-3,
            restitution: 0.0,
            friction: 0.8,
            // Chosen against the integrator around it, not against machine precision. Semi-implicit
            // Euler at 240 Hz carries a first-order error of its own; a contact residual of 5.8e-3
            // is already well under it, and thirty-two sweeps is where that lands. Sixty-four buys
            // two more decimal places for twice the time and nothing the step can use.
            //
            // With friction the cap is what actually stops the iteration — the tolerance is not
            // reached — so this is a cost knob, and the price of turning it is on record above.
            iterations: 32,
            tolerance: 1e-13,
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
    /// `M^-1 J^T`, one column of length `slots` per constraint. Only the union of the reaches is
    /// written; everything else is stale and never read.
    response: Vec<f64>,
    scratch: Vec<f64>,
    /// Membership in the union of the reaches, by link.
    touched: Vec<bool>,
    /// That union, in the plan's order — the links the mobility columns are computed over.
    descend: Vec<usize>,
    /// Sweeps the last solve actually made.
    sweeps: usize,
    /// The limits of the last solve, so the residual can be read against the right condition.
    limits: Vec<Limit>,
}

impl ConstraintSolver {
    /// A solver with no memory allocated yet.
    pub fn new() -> Self {
        Self::default()
    }

    /// One row's unconstrained Gauss-Seidel update: the impulse that would satisfy row `k` exactly,
    /// given every other impulse as it stands.
    ///
    /// The projection onto whatever the multiplier is allowed to be happens outside, because a
    /// contact normal, a free row and a friction pair are allowed different things and only the last
    /// of those needs two rows at once.
    fn gauss_seidel(&self, k: usize) -> f64 {
        let c = self.impulse.len();
        let diagonal = self.a[k * c + k];
        if diagonal <= MOBILITY_FLOOR {
            return self.impulse[k];
        }
        let residual: f64 = self.b[k]
            + self.a[k * c..k * c + c]
                .iter()
                .zip(&self.impulse)
                .map(|(a, p)| a * p)
                .sum::<f64>();
        self.impulse[k] - residual / diagonal
    }

    /// How many Gauss-Seidel sweeps the last solve made, which is at most
    /// [`ContactParameters::iterations`] and usually far fewer.
    pub fn sweeps(&self) -> usize {
        self.sweeps
    }

    /// The impulses from the last [`Self::solve`], one per constraint.
    pub fn impulses(&self) -> &[f64] {
        &self.impulse
    }

    /// How far the last solve missed the complementarity conditions, relative to the problem's own
    /// scale.
    ///
    /// The solution of `A p + b >= 0, p >= 0, p . (A p + b) = 0` satisfies `min(p_k, r_k) = 0` for
    /// every `k`, where `r = A p + b`. That single expression covers all three conditions at once —
    /// a negative impulse, a violated constraint, and a constraint carrying load while separating
    /// all show up in it — so the largest `|min(p_k, r_k)|` is the error of the whole solve.
    ///
    /// Normalised by `max(|p|, |r|)`, because the absolute size of an impulse depends on the step
    /// and the mass and says nothing about how well it was found.
    ///
    /// This is the ONLY approximate step in the chain. The articulated solves are exact to rounding
    /// and the contact geometry is closed-form; projected Gauss-Seidel iterates. So this is the
    /// number that says whether the physics is converged, and it is worth watching rather than
    /// assuming.
    ///
    /// # Two different failures, and they must not be confused
    ///
    /// The returned pair separates them, because more iterations fix one and never fix the other:
    ///
    /// * **unconverged** — over the constraints the mechanism can actually push against. This is the
    ///   iteration's own error and it shrinks; measured, it reaches machine precision in about ten
    ///   sweeps on a standing quadruped.
    /// * **unsatisfiable** — over the constraints with no mobility behind them, where
    ///   `A[k][k]` is zero to rounding. Pushing there does nothing, so a violation cannot be
    ///   removed by any impulse, and iterating is not what is missing. It means the CONFIGURATION is
    ///   impossible: a body inside the floor that no joint can lift.
    ///
    /// Both are relative to the problem's own scale, because the absolute size of an impulse depends
    /// on the step and the mass and says nothing about how well it was found.
    pub fn complementarity_residual(&self) -> (f64, f64) {
        let c = self.impulse.len();
        if c == 0 {
            return (0.0, 0.0);
        }
        let (mut unconverged, mut unsatisfiable, mut scale) = (0.0f64, 0.0f64, 0.0f64);
        for k in 0..c {
            let r: f64 = self.b[k]
                + (0..c)
                    .map(|l| self.a[k * c + l] * self.impulse[l])
                    .sum::<f64>();
            // For a pushing row, `min(p, r)` is the whole complementarity condition. A free row
            // wants `r = 0`. A friction row wants `r = 0` while it is inside the cone, and is
            // allowed a residual once it saturates — that residual IS the sliding.
            let miss = match self.limits.get(k) {
                Some(Limit::Pushing) | None => self.impulse[k].min(r).abs(),
                Some(Limit::Free) => r.abs(),
                Some(Limit::Friction {
                    normal,
                    partner,
                    mu,
                }) => {
                    // Saturation is a property of the PAIR, not of either component. A pair on the
                    // cone's boundary split as `(0.6 r, 0.8 r)` has neither component at `r`, and
                    // testing them one at a time called both "inside the cone" and reported their
                    // residuals — which read as a solver that plateaus at 5-8e-2 and never
                    // converges. It was the measurement that was stuck, not the solve.
                    let bound = mu * self.impulse[*normal].max(0.0);
                    let length =
                        (self.impulse[k].powi(2) + self.impulse[*partner].powi(2)).sqrt();
                    if length >= bound * (1.0 - 1e-9) {
                        0.0 // sliding, and a residual here is the friction doing its job
                    } else {
                        r.abs()
                    }
                }
            };
            if self.a[k * c + k] <= MOBILITY_FLOOR {
                unsatisfiable = unsatisfiable.max(miss);
            } else {
                unconverged = unconverged.max(miss);
            }
            scale = scale.max(self.impulse[k].abs()).max(r.abs());
        }
        let scale = scale.max(f64::MIN_POSITIVE);
        (unconverged / scale, unsatisfiable / scale)
    }

    /// How many constraints the mechanism has no mobility along — the ones the sweep cannot move.
    pub fn unsatisfiable(&self) -> usize {
        let c = self.impulse.len();
        (0..c)
            .filter(|&k| self.a[k * c + k] <= MOBILITY_FLOOR)
            .count()
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
        self.limits.clear();
        self.limits.extend(constraints.iter().map(|c| c.limit));

        // The union of the reaches, in the plan's own order — everything any constraint will read of
        // any mobility column, and nothing more. Built once per step rather than once per column.
        //
        // A reach is closed under taking parents and so is a union of them, which is exactly the
        // precondition the restricted sweep needs.
        self.touched.clear();
        self.touched.resize(aba.links(), false);
        for constraint in constraints {
            for &link in &constraint.reach {
                self.touched[link] = true;
            }
        }
        self.descend.clear();
        self.descend
            .extend(aba.order().iter().copied().filter(|&i| self.touched[i]));

        // One column of `M^-1 J^T` per constraint: what the mechanism does when the ONLY torque is
        // the one that constraint exerts. Gravity off and velocity zero, because this is the
        // mechanism's response to a unit impulse and nothing else.
        // The factorisation is the caller's, and it is ALREADY THERE: a step computes the free
        // acceleration first, and `accelerations` leaves exactly the same articulated inertias, `U`
        // and `d` in the scratch that `factorise` would. Redoing it here was a second full inertia
        // sweep per step, comparable in cost to the dynamics itself.
        for (k, constraint) in constraints.iter().enumerate() {
            assert!(constraint.force.len() >= n, "constraint {k} is too short");
            aba.solve_factorised_over(
                &constraint.force,
                &constraint.reach,
                &self.descend,
                scratch,
                &mut self.scratch,
            );
            self.response[k * n..k * n + n].copy_from_slice(&self.scratch[..n]);
        }

        // Both of these are dot products against `force_k`, which is zero outside its reach — so
        // both run over the reach and not over every coordinate. The entries of `response` outside
        // the union are stale by construction, and this is why nothing reads them.
        for (k, constraint) in constraints.iter().enumerate() {
            for l in 0..c {
                let column = &self.response[l * n..l * n + n];
                let mut sum = 0.0;
                for &link in &constraint.reach {
                    if let Some(joint) = aba.inbound_joint(link) {
                        sum += constraint.force[joint] * column[joint];
                    }
                }
                self.a[k * c + l] = sum;
            }
            let mut sum = 0.0;
            for &link in &constraint.reach {
                if let Some(joint) = aba.inbound_joint(link) {
                    sum += constraint.force[joint] * velocity[joint];
                }
            }
            self.b[k] = sum + constraint.bias;
        }

        // Projected Gauss-Seidel. Each sweep visits the constraints in turn and moves one multiplier
        // to satisfy its own row exactly, projecting it back onto the sign it is allowed. It
        // converges for a positive semidefinite `A`, which this is; it is what most engines actually
        // run, and it degrades into a slightly soft contact rather than into an explosion when it
        // runs out of iterations.
        let mut sweeps = 0usize;
        for _ in 0..parameters.iterations {
            sweeps += 1;
            let mut moved: f64 = 0.0;
            let mut largest: f64 = 0.0;
            for (k, constraint) in constraints.iter().enumerate() {
                match constraint.limit {
                    // The second of a friction pair is settled when the first is reached, because
                    // the projection needs both at once.
                    Limit::Friction { partner, .. } if partner < k => continue,
                    Limit::Friction {
                        normal,
                        partner,
                        mu,
                    } => {
                        // `mu * p_n` is the radius of the disc Coulomb allows, and `p_n` is whatever
                        // the normal row has reached so far — the coupling that makes friction
                        // depend on load.
                        let bound = mu * self.impulse[normal].max(0.0);

                        // Update both rows in sequence — the second sees the first — and THEN
                        // project the pair onto the disc together.
                        //
                        // Projecting the pair rather than clamping each row separately is what keeps
                        // friction isotropic: a per-row clamp confines the impulse to a square, and
                        // a square makes a block slide more easily along a diagonal than along an
                        // axis, which is an artefact of the discretisation and not of the physics.
                        let before = (self.impulse[k], self.impulse[partner]);
                        self.impulse[k] = self.gauss_seidel(k);
                        self.impulse[partner] = self.gauss_seidel(partner);
                        let length = (self.impulse[k].powi(2) + self.impulse[partner].powi(2)).sqrt();
                        if length > bound && length > 0.0 {
                            self.impulse[k] *= bound / length;
                            self.impulse[partner] *= bound / length;
                        }
                        moved = moved
                            .max((self.impulse[k] - before.0).abs())
                            .max((self.impulse[partner] - before.1).abs());
                        largest = largest
                            .max(self.impulse[k].abs())
                            .max(self.impulse[partner].abs());
                    }
                    limit => {
                        if self.a[k * c + k] <= MOBILITY_FLOOR {
                            // Nothing behind this constraint to push against: `A[k][k]` is
                            // `J_k M^-1 J_k^T`, so a zero means the mechanism cannot move along it
                            // at all. Leaving the impulse at zero is the only defensible answer —
                            // any other value would be an impulse that does nothing, chosen
                            // arbitrarily — and the residual reports it separately so it is not
                            // mistaken for slow convergence.
                            continue;
                        }
                        let mut next = self.gauss_seidel(k);
                        if matches!(limit, Limit::Pushing) && next < 0.0 {
                            next = 0.0;
                        }
                        moved = moved.max((next - self.impulse[k]).abs());
                        largest = largest.max(next.abs());
                        self.impulse[k] = next;
                    }
                }
            }
            if parameters.tolerance > 0.0 && moved <= parameters.tolerance * largest {
                break;
            }
        }
        self.sweeps = sweeps;

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
        let mut reach = Vec::new();
        aba.reach(colliders[contact.collider].link, &mut reach);
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
        let normal = out.len();
        out.push(Constraint {
            force,
            reach: reach.clone(),
            bias: bias_position + parameters.restitution * approaching,
            limit: Limit::Pushing,
        });

        if parameters.friction <= 0.0 {
            continue;
        }
        // Two tangents, and the pair is what Coulomb bounds — not either one alone. The bias is
        // zero: friction wants the contact point to stop, not to reach some target speed, and
        // whether it can is the cone's business rather than this one's.
        let (first, second) = (out.len(), out.len() + 1);
        for (k, direction) in tangents([0.0, 0.0, 1.0]).into_iter().enumerate() {
            let mut tangent = vec![0.0; n];
            aba.constraint_force(
                poses,
                colliders[contact.collider].link,
                contact.point,
                direction,
                scratch,
                &mut tangent,
            );
            out.push(Constraint {
                force: tangent,
                reach: reach.clone(),
                bias: 0.0,
                limit: Limit::Friction {
                    normal,
                    partner: if k == 0 { second } else { first },
                    mu: parameters.friction,
                },
            });
        }
    }
}

/// Two unit directions spanning the plane perpendicular to `normal`.
///
/// The pair is arbitrary and it does not matter which: the friction bound is a disc, so it is
/// invariant under rotating the basis within the plane. That invariance is the whole reason to
/// project onto a disc instead of clamping into a square, where the choice WOULD matter.
fn tangents(normal: [f64; 3]) -> [[f64; 3]; 2] {
    // any axis not parallel to the normal, orthogonalised against it
    let seed = if normal[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let along = seed[0] * normal[0] + seed[1] * normal[1] + seed[2] * normal[2];
    let mut first = [
        seed[0] - along * normal[0],
        seed[1] - along * normal[1],
        seed[2] - along * normal[2],
    ];
    let length = (first[0] * first[0] + first[1] * first[1] + first[2] * first[2]).sqrt();
    for c in first.iter_mut() {
        *c /= length;
    }
    let second = [
        normal[1] * first[2] - normal[2] * first[1],
        normal[2] * first[0] - normal[0] * first[2],
        normal[0] * first[1] - normal[1] * first[0],
    ];
    [first, second]
}
