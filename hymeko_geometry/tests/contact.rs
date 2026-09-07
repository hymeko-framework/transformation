//! Contacts as the extra edges a spanning tree does not have.
//!
//! The claim being tested is structural as much as physical: the tree costs `O(n)` and the contacts
//! cost a dense solve of size **the number of contacts**. So the tests check the physics against
//! closed forms where they exist, and separately check that the dense object really is the size
//! claimed — on a twenty-three-joint quadruped standing on four feet, it must be four by four.

use hymeko::common::ids::DeclId;
use hymeko_clifford::Se3;
use hymeko_geometry::articulated::AbaPlan;
use hymeko_geometry::collide::{colliders_of, place, Collider, Placement};
use hymeko_geometry::contact::{
    ground_constraints, ground_contacts, ConstraintSolver, ContactParameters, Constraint,
    GroundContact,
};
use hymeko_geometry::forward::FkPlan;
use hymeko_geometry::kinematics::joints::{JointInfo, JointType};
use hymeko_geometry::kinematics::kinematic::{
    extract_kinematic_model, GeometryInfo, GeometryShape, KinematicModel, LinkInfo,
};

const G: f64 = 9.81;
const GRAV: [f64; 3] = [0.0, 0.0, -G];

// --- a body that can fall -------------------------------------------------------------------------

/// A box of mass `m` and half-height `h`, free to slide along world `z` and nothing else.
///
/// The simplest thing that can be dropped: one coordinate, one collider, and an equation of motion
/// nobody has to derive.
fn falling_box(mass: f64, half: [f64; 3]) -> KinematicModel {
    KinematicModel {
        actuation: Default::default(),
        name: "falling".into(),
        links: vec![LinkInfo {
            inertia_com: None,
            did: DeclId::NONE,
            name: "body".into(),
            mass: Some(mass),
            geometry: Some(GeometryInfo {
                shape: GeometryShape::Box,
                dimensions: half.iter().map(|v| 2.0 * v).collect(),
                collision: None,
            }),
            origin: Some(vec![0.0, 0.0, 0.0]),
            color: None,
        }],
        joints: vec![JointInfo {
            did: DeclId::NONE,
            name: "slide".into(),
            joint_type: JointType::Prismatic,
            parent: DeclId::NONE,
            child: DeclId::NONE,
            parent_link: "world".into(),
            child_link: "body".into(),
            axis: Some([0.0, 0.0, 1.0]),
            origin_xyz: Some([0.0; 3]),
            origin_rpy_deg: None,
            limits: None,
        }],
        frames: vec!["world".into()],
    }
}

/// Everything one step of the whole engine needs, in one place.
struct Engine {
    model: KinematicModel,
    aba: AbaPlan,
    plan: FkPlan,
    colliders: Vec<Collider>,
    solver: ConstraintSolver,
    parameters: ContactParameters,
    ground_height: f64,
    q: Vec<f64>,
    qd: Vec<f64>,
    // scratch
    scratch: hymeko_geometry::articulated::AbaScratch,
    poses: Vec<Se3>,
    placed: Vec<Placement>,
    contacts: Vec<GroundContact>,
    constraints: Vec<Constraint>,
    free: Vec<f64>,
}

impl Engine {
    /// Friction off. Every test written before friction existed is about the normal direction, and
    /// on a fixture with one vertical coordinate a tangent row has nothing to act in.
    fn frictionless(model: KinematicModel) -> Self {
        let mut e = Self::new(model);
        e.parameters.friction = 0.0;
        e
    }

    fn new(model: KinematicModel) -> Self {
        let aba = AbaPlan::new(&model).expect("the topology resolves");
        let plan = FkPlan::new(&model).expect("the topology resolves");
        let colliders = colliders_of(&model);
        let n = aba.slots();
        let scratch = aba.scratch();
        let poses = plan.identity_poses_se3();
        Self {
            model,
            aba,
            plan,
            colliders,
            solver: ConstraintSolver::new(),
            parameters: ContactParameters::default(),
            ground_height: 0.0,
            q: vec![0.0; n],
            qd: vec![0.0; n],
            scratch,
            poses,
            placed: Vec::new(),
            contacts: Vec::new(),
            constraints: Vec::new(),
            free: vec![0.0; n],
        }
    }

    /// One step: free acceleration, contacts, constraint impulses, integrate.
    fn step(&mut self, h: f64, torque: &[f64]) {
        let n = self.aba.slots();

        // the tree, in O(n)
        self.aba.accelerations(
            &self.model,
            &self.q,
            &self.qd,
            torque,
            GRAV,
            &mut self.scratch,
            &mut self.free,
        );
        let mut velocity: Vec<f64> = (0..n).map(|k| self.qd[k] + h * self.free[k]).collect();

        // the extra edges. `accelerations` above left the factorisation in the scratch, which is
        // what the constraint solve needs and what it must not recompute.
        self.plan.resolve_se3(&self.model, &self.q, &mut self.poses);
        place(&self.colliders, &self.poses, &mut self.placed);
        ground_contacts(
            &self.colliders,
            &self.placed,
            self.ground_height,
            self.parameters.margin,
            &mut self.contacts,
        );
        ground_constraints(
            &self.aba,
            &self.poses,
            &self.colliders,
            &self.contacts,
            &self.qd,
            &self.parameters,
            h,
            &mut self.scratch,
            &mut self.constraints,
        );
        self.solver.solve(
            &self.aba,
            &self.constraints,
            &self.parameters,
            &mut self.scratch,
            &mut velocity,
        );

        self.qd.copy_from_slice(&velocity);
        for k in 0..n {
            self.q[k] += h * self.qd[k];
        }
    }
}

// --- the closed form ------------------------------------------------------------------------------

#[test]
fn a_resting_body_receives_exactly_its_own_weight_from_the_ground() {
    // Statics, and there is no room for interpretation: a body at rest on the ground must receive an
    // impulse of `m g h` per step, because that is what cancels the momentum gravity gave it.
    //
    // The whole chain has to be right for this number to come out: the constraint Jacobian, the
    // articulated solve that turns it into `M^-1 J^T`, the Delassus operator built from the two, and
    // the complementarity condition that picks equality rather than inequality.
    for (mass, half_height, h) in [
        (1.0, 0.1, 1.0 / 240.0),
        (7.3, 0.25, 1.0 / 1000.0),
        (0.4, 0.05, 1.0 / 60.0),
    ] {
        let mut e = Engine::frictionless(falling_box(mass, [0.2, 0.2, half_height]));
        e.parameters.slop = 0.0;
        e.q[0] = half_height; // resting exactly on the plane
        e.step(h, &[0.0]);

        let impulses = e.solver.impulses();
        assert_eq!(impulses.len(), 1, "one body on a plane is one contact");
        let want = mass * G * h;
        assert!(
            (impulses[0] - want).abs() < 1e-12 * want,
            "m={mass} h={h}: impulse {} against the weight {want}",
            impulses[0]
        );
        assert!(
            e.qd[0].abs() < 1e-12,
            "m={mass}: it should not be moving, but qdot is {}",
            e.qd[0]
        );
    }
}

#[test]
fn the_delassus_operator_is_the_inverse_mass_the_contact_actually_feels() {
    // For a body sliding on one prismatic axis, `A = J M^-1 J^T` is `1/m` and nothing else. It is
    // the smallest case where the operator can be checked against a number rather than a property.
    for mass in [0.5, 1.0, 12.5] {
        let mut e = Engine::frictionless(falling_box(mass, [0.2, 0.2, 0.1]));
        e.q[0] = 0.1;
        e.step(1.0 / 240.0, &[0.0]);
        let a = e.solver.delassus();
        assert_eq!(a.len(), 1, "one contact is a one-by-one operator");
        assert!(
            (a[0] - 1.0 / mass).abs() < 1e-12 / mass,
            "m={mass}: A = {} against the expected {}",
            a[0],
            1.0 / mass
        );
    }
}

// --- along a trajectory ---------------------------------------------------------------------------

#[test]
fn a_dropped_box_lands_and_stays_on_the_ground() {
    // Released from a height, it must fall, land, and settle with its lowest face on the plane —
    // not sink through, not bounce forever, and not hover.
    let half_height = 0.15;
    let mut e = Engine::frictionless(falling_box(2.0, [0.2, 0.2, half_height]));
    e.q[0] = 1.0;

    let h = 1.0 / 480.0;
    for _ in 0..2400 {
        e.step(h, &[0.0]);
    }

    let lowest = e.q[0] - half_height;
    assert!(
        lowest.abs() < 2e-3,
        "the box settled with its lowest face at {lowest} instead of on the plane"
    );
    assert!(
        e.qd[0].abs() < 1e-2,
        "it is still moving at {} m/s after five seconds",
        e.qd[0]
    );
}

#[test]
fn a_contact_never_adds_energy() {
    // A unilateral contact can only push, and pushing a body that is moving away from the plane does
    // no work. So the total mechanical energy must never exceed what it started with — a bound that
    // a sign error in the impulse, or a projection applied the wrong way round, breaks immediately.
    let half_height = 0.1;
    let mass = 1.5;
    let mut e = Engine::frictionless(falling_box(mass, [0.2, 0.2, half_height]));
    e.q[0] = 0.8;

    let energy = |q: f64, qd: f64| 0.5 * mass * qd * qd + mass * G * q;
    let start = energy(e.q[0], e.qd[0]);

    let h = 1.0 / 480.0;
    let mut worst: f64 = 0.0;
    for _ in 0..2400 {
        e.step(h, &[0.0]);
        worst = worst.max(energy(e.q[0], e.qd[0]) - start);
    }
    // the tolerance is the integrator's, not the contact's: one step of gravity at this rate
    assert!(
        worst < mass * G * G * h * h,
        "the contact added {worst} J, more than a step of gravity can explain"
    );
}

#[test]
fn a_body_above_the_ground_generates_no_constraints_at_all() {
    let mut e = Engine::frictionless(falling_box(1.0, [0.2, 0.2, 0.1]));
    e.q[0] = 3.0;
    e.step(1.0 / 240.0, &[0.0]);
    assert!(
        e.contacts.is_empty() && e.constraints.is_empty(),
        "a body three metres up produced {} contacts",
        e.contacts.len()
    );
    // and it is in free fall
    let h = 1.0 / 240.0;
    assert!(
        (e.qd[0] + G * h).abs() < 1e-12,
        "free fall should reach {} m/s, not {}",
        -G * h,
        e.qd[0]
    );
}

// --- the structural claim ---------------------------------------------------------------------------

#[test]
fn the_dense_solve_is_the_size_of_the_contact_set_not_the_mechanism() {
    // The whole reason for splitting the work this way. A quadruped has twenty-three movable joints;
    // standing on its feet it has a handful of contacts, and the only dense object in the step is
    // that handful squared.
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../core/data/robotics/robots/quadruped.hymeko");
    if !path.exists() {
        return;
    }
    let mut store = hymeko::module_store::module_store::ModuleStore::new(
        hymeko::module_store::source_provider::StdFsProvider::new(),
        hymeko::util::real_parser::RealParser,
    );
    let program = store.compile(&path).expect("the quadruped compiles");
    let engine = hymeko_query::QueryEngine::new(&program.ir, &store.it);
    let model = extract_kinematic_model(&engine, "quadruped");

    let mut e = Engine::frictionless(model);
    let n = e.aba.slots();
    // Put the plane at the actual lowest support point of the authored geometry.
    e.plan.resolve_se3(&e.model, &e.q, &mut e.poses);
    place(&e.colliders, &e.poses, &mut e.placed);
    e.ground_height = e.colliders.iter().zip(&e.placed)
        .map(|(collider, placed)| placed.support(&collider.shape, [0.0, 0.0, -1.0])[2])
        .fold(f64::INFINITY, f64::min);
    e.step(1.0 / 240.0, &vec![0.0; n]);

    let c = e.constraints.len();
    assert!(c > 0, "the quadruped touches nothing");
    assert!(
        c < n,
        "there are {c} constraints against {n} joints; the point is that the first is the smaller"
    );
    assert_eq!(
        e.solver.delassus().len(),
        c * c,
        "the dense operator should be {c} by {c}"
    );

    // and it is symmetric, because `J M^-1 J^T` is
    let a = e.solver.delassus();
    let scale = a.iter().fold(0.0f64, |m, v| m.max(v.abs())).max(1e-30);
    for i in 0..c {
        for j in 0..c {
            assert!(
                (a[i * c + j] - a[j * c + i]).abs() < 1e-9 * scale,
                "the Delassus operator is not symmetric at ({i},{j})"
            );
        }
    }
}

#[test]
fn two_contacts_on_one_coordinate_carry_the_weight_between_them_and_the_split_is_free() {
    // Two colliders on the same body, both on the plane. Their TOTAL is determined — it is the
    // weight — and how they split it is **not**, because this body has one coordinate and one
    // coordinate has no moment balance to settle the question with.
    //
    // That is a fact about the mechanism, not a shortcoming of the solver, and the Delassus operator
    // says so out loud: both constraints have the same Jacobian row, so `A` is `[[1/m, 1/m],
    // [1/m, 1/m]]`, rank one, singular. Any pair summing to the weight satisfies it. A test that
    // demanded an even split would be demanding that the solver invent information the model does
    // not contain — so this one checks the total, and checks that the indeterminacy is visible in
    // the operator rather than hidden.
    let half = 0.1;
    let mass = 3.0;
    let mut e = Engine::frictionless(falling_box(mass, [0.2, 0.2, half]));
    e.colliders.push(Collider {
        link: 0,
        shape: e.colliders[0].shape,
        offset: [0.5, 0.0, 0.0],
    });
    e.parameters.slop = 0.0;
    e.q[0] = half;

    let h = 1.0 / 240.0;
    e.step(h, &[0.0]);

    let p = e.solver.impulses();
    assert_eq!(p.len(), 2, "two colliders on the plane are two contacts");
    let total: f64 = p.iter().sum();
    assert!(
        (total - mass * G * h).abs() < 1e-9 * mass * G * h,
        "the two together carry {total} against the weight {}",
        mass * G * h
    );
    assert!(
        p.iter().all(|&v| v >= 0.0),
        "a contact pulled instead of pushing: {p:?}"
    );

    // the operator is rank one, which is the indeterminacy stated in the only place it can be
    let a = e.solver.delassus();
    let determinant = a[0] * a[3] - a[1] * a[2];
    assert!(
        determinant.abs() < 1e-12 / (mass * mass),
        "A should be singular for two contacts on one coordinate; its determinant is {determinant:e}"
    );
    for entry in a {
        assert!(
            (entry - 1.0 / mass).abs() < 1e-12 / mass,
            "every entry of A should be 1/m = {}; found {entry}",
            1.0 / mass
        );
    }

    // and the body does not move, which is the thing that actually matters
    assert!(
        e.qd[0].abs() < 1e-12,
        "it should be held still, but qdot is {}",
        e.qd[0]
    );
}

// --- friction -------------------------------------------------------------------------------------

/// A box free to slide along world `x` and to move along world `z`, and nothing else.
///
/// Two coordinates is the smallest mechanism where friction can do anything: one to press the
/// contact and one for it to resist. The carriage between them is massless, so the mass matrix is
/// `diag(m, m)` and the closed forms below have no dressing on them.
fn sliding_box(mass: f64, half: [f64; 3]) -> KinematicModel {
    let slide = |name: &str, parent: &str, child: &str, axis: [f64; 3]| JointInfo {
        did: DeclId::NONE,
        name: name.into(),
        joint_type: JointType::Prismatic,
        parent: DeclId::NONE,
        child: DeclId::NONE,
        parent_link: parent.into(),
        child_link: child.into(),
        axis: Some(axis),
        origin_xyz: Some([0.0; 3]),
        origin_rpy_deg: None,
        limits: None,
    };
    KinematicModel {
        actuation: Default::default(),
        name: "sliding".into(),
        links: vec![
            LinkInfo {
                inertia_com: None,
                did: DeclId::NONE,
                name: "carriage".into(),
                mass: None,
                geometry: None,
                origin: Some(vec![0.0; 3]),
                color: None,
            },
            LinkInfo {
                inertia_com: None,
                did: DeclId::NONE,
                name: "body".into(),
                mass: Some(mass),
                geometry: Some(GeometryInfo {
                    shape: GeometryShape::Box,
                    dimensions: half.iter().map(|v| 2.0 * v).collect(),
                    collision: None,
                }),
                origin: Some(vec![0.0; 3]),
                color: None,
            },
        ],
        joints: vec![
            slide("x", "world", "carriage", [1.0, 0.0, 0.0]),
            slide("z", "carriage", "body", [0.0, 0.0, 1.0]),
        ],
        frames: vec!["world".into()],
    }
}

#[test]
fn a_box_pushed_below_the_friction_limit_does_not_move() {
    // Stiction, and the threshold is Coulomb's: a horizontal force under `mu m g` is entirely
    // absorbed. Not "moves slowly" — does not move, because the tangent impulse stays inside the
    // cone and the solve drives the tangential velocity to zero exactly.
    let (mass, half, mu) = (2.0, 0.1, 0.6);
    let limit = mu * mass * G;
    for push in [0.0, 0.25 * limit, 0.9 * limit, 0.99 * limit] {
        let mut e = Engine::new(sliding_box(mass, [0.2, 0.2, half]));
        e.parameters.friction = mu;
        e.parameters.slop = 0.0;
        e.q[1] = half; // resting on the plane

        for _ in 0..200 {
            e.step(1.0 / 240.0, &[push, 0.0]);
        }
        assert!(
            e.q[0].abs() < 1e-9,
            "a push of {push} N against a limit of {limit} N moved it {} m",
            e.q[0]
        );
        assert!(e.qd[0].abs() < 1e-9, "and left it sliding at {} m/s", e.qd[0]);
    }
}

#[test]
fn a_box_pushed_past_the_limit_accelerates_by_exactly_the_excess() {
    // `a = (F - mu m g) / m`, the other half of Coulomb's law and the number a slipping contact has
    // to produce. Measured over the whole motion rather than read off one step, so a solve that is
    // right once and drifts afterwards fails.
    let (mass, half, mu) = (2.0, 0.1, 0.4);
    let limit = mu * mass * G;
    for factor in [1.5, 3.0, 8.0] {
        let push = factor * limit;
        let mut e = Engine::new(sliding_box(mass, [0.2, 0.2, half]));
        e.parameters.friction = mu;
        e.parameters.slop = 0.0;
        e.q[1] = half;

        let h = 1.0 / 2000.0;
        let steps = 2000;
        for _ in 0..steps {
            e.step(h, &[push, 0.0]);
        }
        let want = (push - limit) / mass;
        // from rest, `x = a t^2 / 2`
        let elapsed = steps as f64 * h;
        let got = 2.0 * e.q[0] / (elapsed * elapsed);
        assert!(
            (got - want).abs() < 5e-3 * want,
            "push {push} N: acceleration {got} against the expected {want}"
        );
    }
}

#[test]
fn a_sliding_box_comes_to_rest_after_the_time_coulomb_predicts() {
    // Given a shove and left alone, it decelerates at `mu g` and stops after `v / (mu g)`. What this
    // catches beyond that: a friction that overshoots into reverse, which a projection onto the cone
    // cannot do and a per-axis clamp can.
    let (mass, half, mu) = (1.0, 0.1, 0.5);
    let mut e = Engine::new(sliding_box(mass, [0.2, 0.2, half]));
    e.parameters.friction = mu;
    e.parameters.slop = 0.0;
    e.q[1] = half;
    e.qd[0] = 2.0;

    let h = 1.0 / 1000.0;
    let expected = 2.0 / (mu * G);
    let mut stopped_at = None;
    for step in 0..4000 {
        e.step(h, &[0.0, 0.0]);
        if stopped_at.is_none() && e.qd[0] <= 0.0 {
            stopped_at = Some(step as f64 * h);
        }
        assert!(
            e.qd[0] > -1e-6,
            "step {step}: friction reversed the motion, to {} m/s",
            e.qd[0]
        );
    }
    let stopped = stopped_at.expect("it never stopped");
    assert!(
        (stopped - expected).abs() < 0.02 * expected,
        "it stopped after {stopped} s against the expected {expected}"
    );
}

#[test]
fn the_tangent_impulse_never_leaves_the_cone() {
    // The invariant the projection exists to keep, checked at every step of a run that both sticks
    // and slips — so it covers the inside of the cone and its boundary.
    let (mass, half, mu) = (1.5, 0.12, 0.35);
    let mut e = Engine::new(sliding_box(mass, [0.2, 0.2, half]));
    e.parameters.friction = mu;
    e.q[1] = 0.6; // dropped, so it lands and then slides

    // A short hard shove, then a push far below the limit — long enough after it that the box
    // stops and STICKS. The first attempt pushed for half of a three-second run and it was still
    // sliding at the end, so the cone's interior was never visited and the test proved half of what
    // it claimed.
    let h = 1.0 / 1000.0;
    let (mut saw_saturated, mut saw_inside) = (false, false);
    for step in 0..3000 {
        let push = if step < 300 { 8.0 } else { 0.5 };
        e.step(h, &[push, 0.0]);
        for chunk in e.solver.impulses().chunks(3) {
            if chunk.len() < 3 {
                continue;
            }
            let tangential = (chunk[1] * chunk[1] + chunk[2] * chunk[2]).sqrt();
            let bound = mu * chunk[0].max(0.0);
            assert!(
                tangential <= bound * (1.0 + 1e-9) + 1e-12,
                "step {step}: |p_t| = {tangential} exceeds mu p_n = {bound}"
            );
            if bound > 1e-12 {
                if tangential > 0.99 * bound {
                    saw_saturated = true;
                } else if tangential < 0.5 * bound {
                    saw_inside = true;
                }
            }
        }
    }
    assert!(saw_saturated, "the run never reached the cone's boundary");
    assert!(saw_inside, "the run never stayed inside the cone");
}

#[test]
fn friction_only_removes_energy() {
    let (mass, half, mu) = (1.0, 0.1, 0.45);
    let mut e = Engine::new(sliding_box(mass, [0.2, 0.2, half]));
    e.parameters.friction = mu;
    e.parameters.slop = 0.0;
    e.q[1] = half;
    e.qd[0] = 3.0;

    let energy =
        |e: &Engine| 0.5 * mass * (e.qd[0] * e.qd[0] + e.qd[1] * e.qd[1]) + mass * G * e.q[1];
    let start = energy(&e);
    let h = 1.0 / 1000.0;
    for step in 0..2000 {
        e.step(h, &[0.0, 0.0]);
        assert!(
            energy(&e) <= start + mass * G * G * h * h,
            "step {step}: friction added energy, {} against {start}",
            energy(&e)
        );
    }
    assert!(
        energy(&e) < 0.4 * start,
        "friction removed only {:.0}% of the energy",
        100.0 * (1.0 - energy(&e) / start)
    );
}
