//! Ryoshi rigid-body engine: existing articulated dynamics and contact solver.
//! A whole physics step: the tree, the contacts, and the choice of how to advance.
//!
//! # Why the integrator is a choice the engine makes and not the caller
//!
//! Measured on a chaotic double pendulum, at equal cost in acceleration evaluations, a fifth-order
//! Runge-Kutta is between three and six orders of magnitude more accurate than the semi-implicit
//! Euler this engine otherwise ships — and Dormand-Prince at 15 Hz beats symplectic Euler at
//! 3840 Hz for a forty-third of the work. On smooth motion, first order is not a reasonable choice.
//!
//! **A contact is not smooth.** The acceleration jumps, and a scheme whose accuracy comes from
//! sampling across the step samples across the jump: its order collapses to first while it still
//! pays for six stages. That is why every contact engine ships Euler, and it is not ignorance.
//!
//! Both facts are true at once, and which one applies is decided by the contact set — which this
//! engine already computes every step. So it decides, per step, rather than making the caller
//! choose once and be wrong half the time.
//!
//! # How the regime is decided, and why it is a trial rather than a prediction
//!
//! Predicting whether a big step will run into something needs a bound on how fast every point of
//! every collider is moving, which is a bound on the whole Jacobian and is neither cheap nor tight.
//! So the step is TAKEN and then checked: if nothing was touching before and nothing is touching
//! after, the big step stands. Otherwise it is rolled back — the state is two vectors — and the
//! interval is covered by small semi-implicit steps instead.
//!
//! Against a ground plane this cannot miss a collision, because passing through the plane leaves the
//! collider below it and the check sees that. It is not general continuous collision detection: two
//! thin bodies could pass through each other within one step and the trial would accept it.

use hymeko_clifford::Se3;

use crate::articulated::{AbaPlan, AbaScratch};
use crate::collide::{colliders_of, place, BroadPhase, Collider, Placement};
use crate::contact::{
    ground_constraints, ground_contacts, Constraint, ConstraintSolver, ContactParameters,
    GroundContact,
};
use crate::forward::{FkError, FkPlan};
use crate::kinematics::kinematic::KinematicModel;

/// How the engine advances between contacts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FreeFlight {
    /// Never take the big step. Every interval is covered by [`Engine::contact_step`].
    ///
    /// What to use when the answer must not depend on whether anything happened to be touching.
    Never,
    /// Dormand-Prince 5(4) at up to `step` while nothing is touching.
    ///
    /// Descended from Runge-Kutta-Fehlberg 4(5), Erwin Fehlberg's 1969 method at NASA's Marshall
    /// centre. Six acceleration evaluations per step, and fifth order.
    ///
    /// # What it buys, and what it does not
    ///
    /// On a double pendulum over one second of smooth motion, against a 50 kHz reference:
    ///
    /// ```text
    ///   Euler @   240 Hz   1.09e-2 rad    0.08 ms
    ///   Euler @ 61440 Hz   4.25e-5 rad   19.52 ms
    ///   this  @    30 Hz   6.99e-6 rad    0.07 ms
    ///   this  @   960 Hz   5.58e-13 rad   2.09 ms
    /// ```
    ///
    /// More accurate at thirty steps a second than Euler at sixty-one thousand, for a two hundred
    /// and eightieth of the time. **For a target accuracy it is not close.**
    ///
    /// It does NOT make a fixed frame rate cheaper. `advance` never steps past what it was asked
    /// for, so a caller handing it a sixtieth of a second gets one six-evaluation step where Euler
    /// would take four one-evaluation ones — half again the cost, for an answer that is orders of
    /// magnitude better. The saving appears only when the caller asks for intervals at least as long
    /// as `step`: a rollout, a planner, anything not tied to a display.
    ///
    /// # When not to
    ///
    /// The damping is explicit inside every stage here, so a stiffly damped mechanism wants
    /// [`FreeFlight::Never`]. And the accuracy argument assumes the acceleration is a smooth function
    /// of the state, which a mechanism with degenerate joints is not: measured, eleven per cent of
    /// the library arm's (joint, pose) pairs have no resolvable inertia about the joint's own axis,
    /// and its `qdd` is forced to zero there. High order buys nothing across a discontinuity.
    DormandPrince { step: f64 },
}

/// A mechanism, its colliders, and the state of a simulation over them.
pub struct Engine {
    model: KinematicModel,
    aba: AbaPlan,
    plan: FkPlan,
    colliders: Vec<Collider>,
    broad: BroadPhase,
    solver: ConstraintSolver,

    /// Joint positions and rates, by plan slot.
    pub q: Vec<f64>,
    pub qd: Vec<f64>,
    /// Applied joint torques, by plan slot.
    pub torque: Vec<f64>,

    /// Gravity in the base frame.
    pub gravity: [f64; 3],
    /// Height of the ground plane, or `None` for a mechanism with nothing to stand on.
    pub ground: Option<f64>,
    pub contact: ContactParameters,
    /// The step taken whenever anything is touching. First order, and unconditionally stable in the
    /// joint damping.
    pub contact_step: f64,
    /// Viscous damping per joint, taken at the new velocity.
    pub damping: f64,
    /// What to do when nothing is touching.
    pub free_flight: FreeFlight,

    // scratch, so a step allocates nothing once the contact set has settled
    scratch: AbaScratch,
    free: Vec<f64>,
    velocity: Vec<f64>,
    armature: Vec<f64>,
    poses: Vec<Se3>,
    placed: Vec<Placement>,
    boxes: Vec<([f64; 3], [f64; 3])>,
    candidates: Vec<(usize, usize)>,
    contacts: Vec<GroundContact>,
    constraints: Vec<Constraint>,
    stages: Vec<Vec<f64>>,
    rates: Vec<Vec<f64>>,
    trial: (Vec<f64>, Vec<f64>),

    /// Counters, so a caller can see which regime the motion actually spent its time in.
    free_steps: usize,
    contact_steps: usize,
    rejected: usize,
}

impl Engine {
    /// Prepare a mechanism for simulation.
    ///
    /// # Errors
    /// Whatever [`AbaPlan::new`] rejects — a cycle, a joint naming a link that does not exist.
    pub fn new(model: KinematicModel) -> Result<Self, FkError> {
        let aba = AbaPlan::new(&model)?;
        let plan = FkPlan::new(&model)?;
        let colliders = colliders_of(&model);
        let broad = BroadPhase::new(&model, &colliders);
        let n = aba.slots();
        let scratch = aba.scratch();
        let poses = plan.identity_poses_se3();
        Ok(Self {
            model,
            aba,
            plan,
            colliders,
            broad,
            solver: ConstraintSolver::new(),
            q: vec![0.0; n],
            qd: vec![0.0; n],
            torque: vec![0.0; n],
            gravity: [0.0, 0.0, -9.80665],
            ground: Some(0.0),
            contact: ContactParameters::default(),
            contact_step: 1.0 / 240.0,
            damping: 0.0,
            free_flight: FreeFlight::DormandPrince { step: 1.0 / 30.0 },
            scratch,
            free: vec![0.0; n],
            velocity: vec![0.0; n],
            armature: vec![0.0; n],
            poses,
            placed: Vec::new(),
            boxes: Vec::new(),
            candidates: Vec::new(),
            contacts: Vec::new(),
            constraints: Vec::new(),
            stages: vec![vec![0.0; n]; 6],
            rates: vec![vec![0.0; n]; 6],
            trial: (vec![0.0; n], vec![0.0; n]),
            free_steps: 0,
            contact_steps: 0,
            rejected: 0,
        })
    }

    /// The mechanism.
    pub fn model(&self) -> &KinematicModel {
        &self.model
    }

    /// The prepared dynamics, for a caller that wants a mobility column or an inverse solve.
    pub fn dynamics(&self) -> &AbaPlan {
        &self.aba
    }

    /// The contacts found at the last step.
    pub fn contacts(&self) -> &[GroundContact] {
        &self.contacts
    }

    /// The constraint impulses from the last step.
    pub fn impulses(&self) -> &[f64] {
        self.solver.impulses()
    }

    /// Steps taken in free flight, steps taken in contact, and big steps that were rolled back.
    pub fn regime_counts(&self) -> (usize, usize, usize) {
        (self.free_steps, self.contact_steps, self.rejected)
    }

    /// World poses of the links, as of the last step.
    pub fn poses(&self) -> &[Se3] {
        &self.poses
    }

    /// Advance the simulation by `dt` of simulated time.
    ///
    /// The interval is covered by whatever mixture of regimes fits it, so the caller does not have
    /// to know or care which. A `dt` of zero or less does nothing.
    pub fn advance(&mut self, dt: f64) {
        let mut remaining = dt.max(0.0);
        let mut guard = 0usize;
        while remaining > 1e-12 && guard < 100_000 {
            guard += 1;
            let taken = self.one_step(remaining);
            remaining -= taken;
        }
    }

    /// One step of at most `budget`, returning how much of it was used.
    fn one_step(&mut self, budget: f64) -> f64 {
        if let FreeFlight::DormandPrince { step } = self.free_flight {
            let big = step.min(budget);
            // No comparison against `contact_step` here. Whether the free step is larger is the
            // caller's business — a small one is a deliberately accurate run, and an earlier version
            // silently fell through to Euler for exactly that case, which made a reference run a
            // copy of the thing it was meant to be a reference for.
            if self.clearance_is_positive() {
                self.trial.0.copy_from_slice(&self.q);
                self.trial.1.copy_from_slice(&self.qd);
                self.dormand_prince(big);
                if self.clearance_is_positive() {
                    self.free_steps += 1;
                    return big;
                }
                // it ran into something; the state is two vectors, so putting it back is free
                self.q.copy_from_slice(&self.trial.0);
                self.qd.copy_from_slice(&self.trial.1);
                self.rejected += 1;
            }
        }
        let small = self.contact_step.min(budget);
        self.semi_implicit(small);
        self.contact_steps += 1;
        small
    }

    /// Is every collider clear of the ground, with the contact margin to spare?
    ///
    /// Also refreshes the poses and placements, which the next step needs anyway.
    fn clearance_is_positive(&mut self) -> bool {
        let Some(height) = self.ground else {
            return true;
        };
        self.plan.resolve_se3(&self.model, &self.q, &mut self.poses);
        place(&self.colliders, &self.poses, &mut self.placed);
        ground_contacts(
            &self.colliders,
            &self.placed,
            height,
            self.contact.margin,
            &mut self.contacts,
        );
        self.contacts.is_empty()
    }

    /// Accelerations at a state, with the damping torque added.
    fn accelerate(&mut self, q: &[f64], qd: &[f64], out: &mut [f64]) {
        for ((slot, &applied), &rate) in self.velocity.iter_mut().zip(&self.torque).zip(qd) {
            *slot = applied - self.damping * rate;
        }
        self.aba.accelerations(
            &self.model,
            q,
            qd,
            &self.velocity,
            self.gravity,
            &mut self.scratch,
            out,
        );
    }

    /// Semi-implicit Euler with implicit damping, then the contact impulses.
    ///
    /// The scheme for whenever anything is touching, and the one whose damping is unconditionally
    /// stable: `M + h C` differs from `M` by a joint-space diagonal, which the articulated recursion
    /// absorbs as an armature.
    fn semi_implicit(&mut self, h: f64) {
        let n = self.aba.slots();
        self.armature.clear();
        self.armature.resize(n, h * self.damping);
        self.aba.set_armature(&self.armature);

        for k in 0..n {
            self.velocity[k] = self.torque[k] - self.damping * self.qd[k];
        }
        let mut free = std::mem::take(&mut self.free);
        self.aba.accelerations(
            &self.model,
            &self.q,
            &self.qd,
            &self.velocity,
            self.gravity,
            &mut self.scratch,
            &mut free,
        );
        for ((slot, &rate), &acc) in self.velocity.iter_mut().zip(&self.qd).zip(&free) {
            *slot = rate + h * acc;
        }
        self.free = free;

        if let Some(height) = self.ground {
            self.plan.resolve_se3(&self.model, &self.q, &mut self.poses);
            place(&self.colliders, &self.poses, &mut self.placed);
            self.broad
                .overlapping(&self.colliders, &self.placed, &mut self.boxes, &mut self.candidates);
            ground_contacts(
                &self.colliders,
                &self.placed,
                height,
                self.contact.margin,
                &mut self.contacts,
            );
            if !self.contacts.is_empty() {
                ground_constraints(
                    &self.aba,
                    &self.poses,
                    &self.colliders,
                    &self.contacts,
                    &self.qd,
                    &self.contact,
                    h,
                    &mut self.scratch,
                    &mut self.constraints,
                );
                let mut velocity = std::mem::take(&mut self.velocity);
                self.solver.solve(
                    &self.aba,
                    &self.constraints,
                    &self.contact,
                    &mut self.scratch,
                    &mut velocity,
                );
                self.velocity = velocity;
            }
        }

        for k in 0..n {
            self.qd[k] = self.velocity[k];
            self.q[k] += h * self.qd[k];
        }
        self.armature.clear();
        self.armature.resize(n, 0.0);
        self.aba.set_armature(&self.armature);
    }

    /// Dormand-Prince 5(4) at a fixed step, for an interval with nothing touching.
    ///
    /// The armature stays zero: it exists to make the damping implicit in a semi-implicit velocity
    /// update and there is no such update here, so the damping is explicit inside every stage. That
    /// is a stiffness limit, and it is the reason a stiffly damped mechanism should use
    /// [`FreeFlight::Never`].
    fn dormand_prince(&mut self, h: f64) {
        const A: [&[f64]; 6] = [
            &[],
            &[1.0 / 5.0],
            &[3.0 / 40.0, 9.0 / 40.0],
            &[44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0],
            &[
                19372.0 / 6561.0,
                -25360.0 / 2187.0,
                64448.0 / 6561.0,
                -212.0 / 729.0,
            ],
            &[
                9017.0 / 3168.0,
                -355.0 / 33.0,
                46732.0 / 5247.0,
                49.0 / 176.0,
                -5103.0 / 18656.0,
            ],
        ];
        const B: [f64; 6] = [
            35.0 / 384.0,
            0.0,
            500.0 / 1113.0,
            125.0 / 192.0,
            -2187.0 / 6784.0,
            11.0 / 84.0,
        ];

        let n = self.aba.slots();
        let (q0, d0) = (self.q.clone(), self.qd.clone());
        let mut stages = std::mem::take(&mut self.stages);
        let mut rates = std::mem::take(&mut self.rates);

        for stage in 0..6 {
            self.q.copy_from_slice(&q0);
            self.qd.copy_from_slice(&d0);
            for (j, &coefficient) in A[stage].iter().enumerate() {
                if coefficient == 0.0 {
                    continue;
                }
                for i in 0..n {
                    self.q[i] += h * coefficient * rates[j][i];
                    self.qd[i] += h * coefficient * stages[j][i];
                }
            }
            rates[stage].copy_from_slice(&self.qd);
            let (q, qd) = (self.q.clone(), self.qd.clone());
            let mut out = std::mem::take(&mut stages[stage]);
            self.accelerate(&q, &qd, &mut out);
            stages[stage] = out;
        }

        self.q.copy_from_slice(&q0);
        self.qd.copy_from_slice(&d0);
        for i in 0..n {
            for (stage, &weight) in B.iter().enumerate() {
                if weight == 0.0 {
                    continue;
                }
                self.q[i] += h * weight * rates[stage][i];
                self.qd[i] += h * weight * stages[stage][i];
            }
        }
        self.stages = stages;
        self.rates = rates;
    }
}
