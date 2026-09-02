//! The whole step, and the regime it chooses to take it in.
//!
//! Two things have to be true at once for the choice to be worth making: the big step must be much
//! more accurate for its cost where the motion is smooth, and it must never let a body through the
//! floor. The second is what the tests here spend most of their effort on, because it is the one
//! that turns a speed-up into a wrong answer.

use hymeko::common::ids::DeclId;
use hymeko_geometry::engine::{Engine, FreeFlight};
use hymeko_geometry::kinematics::joints::{JointInfo, JointType};
use hymeko_geometry::kinematics::kinematic::{
    GeometryInfo, GeometryShape, KinematicModel, LinkInfo,
};

const G: f64 = 9.80665;

/// A box on a prismatic joint along world `z`: it falls, it lands, and nothing else happens.
fn falling_box(mass: f64, half_height: f64) -> KinematicModel {
    KinematicModel {
        name: "falling".into(),
        links: vec![LinkInfo {
            did: DeclId::NONE,
            name: "body".into(),
            mass: Some(mass),
            geometry: Some(GeometryInfo {
                shape: GeometryShape::Box,
                dimensions: vec![0.2, 0.2, half_height],
                collision: None,
            }),
            origin: Some(vec![0.0; 3]),
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

/// A double pendulum with nothing to collide with — the smooth regime, undiluted.
fn double_pendulum() -> KinematicModel {
    let link = |name: &str, mass: Option<f64>, com: [f64; 3]| LinkInfo {
        did: DeclId::NONE,
        name: name.into(),
        mass,
        geometry: None,
        origin: Some(com.to_vec()),
        color: None,
    };
    let hinge = |name: &str, parent: &str, child: &str, at: [f64; 3]| JointInfo {
        did: DeclId::NONE,
        name: name.into(),
        joint_type: JointType::Revolute,
        parent: DeclId::NONE,
        child: DeclId::NONE,
        parent_link: parent.into(),
        child_link: child.into(),
        axis: Some([0.0, 1.0, 0.0]),
        origin_xyz: Some(at),
        origin_rpy_deg: None,
        limits: None,
    };
    KinematicModel {
        name: "double".into(),
        links: vec![
            link("base", None, [0.0; 3]),
            link("upper", Some(2.0), [0.0, 0.0, -0.5]),
            link("lower", Some(1.0), [0.0, 0.0, -0.4]),
        ],
        joints: vec![
            hinge("shoulder", "base", "upper", [0.0; 3]),
            hinge("elbow", "upper", "lower", [0.0, 0.0, -0.5]),
        ],
        frames: Vec::new(),
    }
}

// --- the safety property ------------------------------------------------------------------------

#[test]
fn a_big_free_step_never_lets_a_body_through_the_floor() {
    // The property that turns the speed-up into a correct answer rather than a fast wrong one. The
    // box is dropped from a range of heights with a free step forty times the contact step, so the
    // approach lands at every phase relative to the plane — including the ones where a body moving
    // at ten metres a second would cross it in the middle of a step.
    for height in [0.2, 0.35, 0.5, 0.77, 1.0, 1.5, 2.0, 3.0] {
        let half = 0.1;
        let mut e = Engine::new(falling_box(2.0, half)).expect("the topology resolves");
        e.contact.friction = 0.0;
        e.contact_step = 1.0 / 240.0;
        e.free_flight = FreeFlight::DormandPrince { step: 1.0 / 6.0 };
        e.q[0] = height;

        let mut lowest = f64::INFINITY;
        for _ in 0..600 {
            e.advance(1.0 / 60.0);
            lowest = lowest.min(e.q[0] - half);
        }
        // The bound is one contact step's travel at the impact speed, plus the slop. Some
        // penetration is not a failure and cannot be avoided by a discrete scheme: the body crosses
        // the plane inside a step and the impulse acts at its end. Going deeper than a whole step's
        // travel WOULD be a failure, because it would mean the contact was not seen when it happened.
        let impact = (2.0 * G * (height - half)).max(0.0).sqrt();
        let allowed = impact * e.contact_step + e.contact.slop;
        assert!(
            lowest > -allowed,
            "dropped from {height} m at {impact:.2} m/s, the lowest face reached {lowest} m;              one step's travel is {allowed:.2e}, so it was not caught when it crossed"
        );
        assert!(
            (e.q[0] - half).abs() < 5e-3,
            "dropped from {height} m, it settled at {} instead of on the plane",
            e.q[0] - half
        );
    }
}

#[test]
fn the_engine_takes_big_steps_while_falling_and_small_ones_once_it_lands() {
    // The regime switch, counted rather than assumed. Falling from two metres takes about 0.64 s,
    // and the run is five seconds, so most of it is spent resting — but the fall itself must be
    // covered by big steps or the switch is not doing anything.
    let half = 0.1;
    let mut e = Engine::new(falling_box(1.0, half)).expect("the topology resolves");
    e.contact.friction = 0.0;
    e.free_flight = FreeFlight::DormandPrince { step: 1.0 / 30.0 };
    e.q[0] = 2.0;

    // the fall alone
    while e.q[0] - half > 0.3 {
        e.advance(1.0 / 60.0);
    }
    let (free, contact, rejected) = e.regime_counts();
    assert!(
        free > contact,
        "falling through open air should be mostly big steps; it was {free} free against \
         {contact} in contact"
    );
    assert_eq!(rejected, 0, "nothing was near the floor, so nothing should have been rolled back");

    // and once it has landed
    for _ in 0..300 {
        e.advance(1.0 / 60.0);
    }
    let (_, contact_after, rejected_after) = e.regime_counts();
    assert!(
        contact_after > contact + 200,
        "resting on the plane should be small steps; it took {} more",
        contact_after - contact
    );
    assert!(
        rejected_after > 0,
        "the approach to the floor should have rolled a big step back at least once"
    );
}

// --- the accuracy the regime buys -----------------------------------------------------------------

#[test]
fn free_flight_at_a_thirtieth_beats_contact_stepping_at_a_two_hundred_and_fortieth() {
    // A double pendulum with nothing to touch, so the engine is in free flight throughout. The
    // question is whether the big step is worth taking, and the answer has to be measured against a
    // reference rather than against intuition.
    //
    // Dormand-Prince makes six evaluations per step, so at 30 Hz it makes 180 per second against
    // semi-implicit Euler's 240 at 240 Hz. Less work, and it should still be far closer.
    let reference = {
        let mut e = Engine::new(double_pendulum()).expect("the topology resolves");
        e.ground = None;
        e.free_flight = FreeFlight::DormandPrince { step: 1.0 / 20_000.0 };
        e.contact_step = 1.0 / 20_000.0;
        e.q = vec![1.1, -0.6];
        e.qd = vec![0.4, 2.2];
        e.advance(1.0);
        e.q.clone()
    };

    let run = |free: FreeFlight| {
        let mut e = Engine::new(double_pendulum()).expect("the topology resolves");
        e.ground = None;
        e.free_flight = free;
        e.contact_step = 1.0 / 240.0;
        e.q = vec![1.1, -0.6];
        e.qd = vec![0.4, 2.2];
        e.advance(1.0);
        e.q.iter()
            .zip(&reference)
            .fold(0.0f64, |m, (a, b)| m.max((a - b).abs()))
    };

    let euler = run(FreeFlight::Never);
    let adaptive = run(FreeFlight::DormandPrince { step: 1.0 / 30.0 });
    assert!(
        adaptive < euler / 1000.0,
        "for less work, free flight was off by {adaptive:e} rad and contact stepping by {euler:e}; \
         the higher order should be worth orders of magnitude on smooth motion"
    );
}

#[test]
fn a_mechanism_with_no_ground_never_leaves_free_flight() {
    let mut e = Engine::new(double_pendulum()).expect("the topology resolves");
    e.ground = None;
    e.q = vec![1.1, -0.6];
    e.advance(1.0);
    let (free, contact, rejected) = e.regime_counts();
    assert!(free > 0, "nothing to touch, so it should have been in free flight");
    assert_eq!(contact, 0, "it took {contact} contact steps with no ground");
    assert_eq!(rejected, 0);
}

// --- the two regimes must agree on the physics ----------------------------------------------------

#[test]
fn turning_free_flight_off_changes_the_speed_and_not_the_answer() {
    // The regime is an implementation choice, so it must not change where the box ends up — only how
    // long it takes to get the answer. Dropped from a metre, both must land it on the plane.
    let half = 0.12;
    let settle = |free: FreeFlight| {
        let mut e = Engine::new(falling_box(1.7, half)).expect("the topology resolves");
        e.contact.friction = 0.0;
        e.free_flight = free;
        e.q[0] = 1.0;
        for _ in 0..400 {
            e.advance(1.0 / 60.0);
        }
        (e.q[0], e.qd[0])
    };
    let (with, wv) = settle(FreeFlight::DormandPrince { step: 1.0 / 30.0 });
    let (without, ov) = settle(FreeFlight::Never);

    assert!(
        (with - half).abs() < 5e-3 && (without - half).abs() < 5e-3,
        "both should rest on the plane: {with} and {without} against {half}"
    );
    assert!(
        wv.abs() < 1e-2 && ov.abs() < 1e-2,
        "and both at rest: {wv} and {ov}"
    );
}

#[test]
fn a_resting_body_is_held_at_the_right_height_by_the_engine_as_a_whole() {
    // The contact physics reaching through the engine rather than being called directly: the box
    // must sit with its lowest face on the plane, and the impulse must be its weight for the step.
    let half = 0.15;
    let mass = 2.5;
    let mut e = Engine::new(falling_box(mass, half)).expect("the topology resolves");
    e.contact.friction = 0.0;
    e.contact.slop = 0.0;
    e.free_flight = FreeFlight::Never;
    e.q[0] = half;
    e.advance(e.contact_step);

    assert_eq!(e.impulses().len(), 1, "one body on a plane is one contact");
    let want = mass * G * e.contact_step;
    assert!(
        (e.impulses()[0] - want).abs() < 1e-9 * want,
        "the ground pushed with {} against the weight {want}",
        e.impulses()[0]
    );
    assert!(e.qd[0].abs() < 1e-9, "and it should not have moved: {}", e.qd[0]);
}
