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

#[test]
fn authored_stops_constrain_both_directions_and_release_inward() {
    use hymeko_geometry::kinematics::joints::JointLimits;
    for flight in [FreeFlight::Never, FreeFlight::DormandPrince { step: 0.1 }] {
        for sign in [-1.0, 1.0] {
            let mut model = falling_box(2.0, 0.1);
            model.joints[0].limits = Some(JointLimits {
                lower: -0.2,
                upper: 0.2,
                effort: 0.0,
                velocity: 0.0,
            });
            let mut e = Engine::new(model).unwrap();
            e.ground = None;
            e.gravity = [0.0; 3];
            e.free_flight = flight;
            e.qd[0] = sign * 10.0;
            e.advance(0.1);
            assert!((e.q[0] - sign * 0.2).abs() < 1e-9, "{:?}", e.q);
            assert!(e.qd[0].abs() < 1e-9);
            e.torque[0] = sign * 5.0;
            e.advance(0.1);
            assert!((e.q[0] - sign * 0.2).abs() < 1e-9);
            e.torque[0] = -sign * 5.0;
            e.advance(0.05);
            assert!(e.qd[0] * sign < 0.0);
            assert!(e.q[0].abs() < 0.2);
        }
    }
}

#[test]
fn stops_share_the_contact_solve_and_retain_coupled_inertia() {
    use hymeko_geometry::kinematics::joints::JointLimits;
    let limits = || JointLimits {
        lower: -0.2,
        upper: 0.2,
        effort: 0.0,
        velocity: 0.0,
    };
    let mut model = double_pendulum();
    model.joints[1].limits = Some(limits());
    let mut e = Engine::new(model).unwrap();
    e.ground = None;
    e.gravity = [0.0; 3];
    e.free_flight = FreeFlight::Never;
    e.q[1] = 0.2;
    e.qd[1] = 1.0;
    let mut free = Engine::new(double_pendulum()).unwrap();
    free.ground = None;
    free.gravity = [0.0; 3];
    free.free_flight = FreeFlight::Never;
    free.q.clone_from(&e.q);
    free.qd.clone_from(&e.qd);
    free.advance(1.0 / 240.0);
    e.advance(1.0 / 240.0);
    assert!(e.q[1] <= 0.2 + 1e-9);
    assert!(
        e.qd[0].abs() > 0.01,
        "impulse must reach the other link: {:?}",
        e.qd
    );
    // Independent point-mass double-pendulum inertia at the impulse pose.
    let m11 = 0.91 + 0.4 * 0.2_f64.cos();
    let m12 = 0.16 + 0.2 * 0.2_f64.cos();
    let momentum = |v: &[f64]| m11 * v[0] + m12 * v[1];
    let kinetic =
        |v: &[f64]| 0.5 * (m11 * v[0] * v[0] + 2.0 * m12 * v[0] * v[1] + 0.16 * v[1] * v[1]);
    assert!((momentum(&e.qd) - momentum(&free.qd)).abs() < 1e-10);
    assert!(kinetic(&e.qd) <= kinetic(&free.qd) + 1e-12);
    let mut model = falling_box(2.0, 0.1);
    model.joints[0].limits = Some(limits());
    let mut e = Engine::new(model).unwrap();
    e.q[0] = 0.15;
    for _ in 0..120 {
        e.advance(1.0 / 240.0);
    }
    assert!((e.q[0] - 0.1).abs() < 1e-9);
    assert!(!e.contacts().is_empty());
    e.gravity = [0.0, 0.0, 9.81];
    for _ in 0..120 {
        e.advance(1.0 / 240.0);
    }
    assert!((e.q[0] - 0.2).abs() < 1e-9);
}

#[test]
fn malformed_stops_reject_and_continuous_joints_ignore_position_bounds() {
    use hymeko_geometry::kinematics::joints::JointLimits;
    for (lo, hi) in [(1.0, -1.0), (f64::NAN, 1.0), (0.0, f64::INFINITY)] {
        let mut model = double_pendulum();
        model.joints[0].limits = Some(JointLimits {
            lower: lo,
            upper: hi,
            effort: 1.0,
            velocity: 1.0,
        });
        let error = Engine::new(model.clone()).err().unwrap();
        assert!(error.to_string().contains("position limits"));
        model.joints[0].joint_type = JointType::Continuous;
        let mut e = Engine::new(model).unwrap();
        e.ground = None;
        e.gravity = [0.0; 3];
        e.q[0] = 2.0;
        e.qd[0] = 1.0;
        e.advance(1.0 / 240.0);
        assert!(e.q[0] > 2.0);
    }
}

#[test]
fn impending_plane_crossing_is_constrained_before_penetration() {
    // Sphere avoids coupling this regression to the separate box-size bug.
    for step in [1.0 / 60.0, 1.0 / 240.0, 1.0 / 960.0] {
        for speed in [1.0, 10.0, 100.0] {
            for fraction in [0.15, 0.5, 0.95, 1.5] {
                let mut model = falling_box(2.0, 0.1);
                model.links[0].geometry = Some(GeometryInfo {
                    shape: GeometryShape::Sphere,
                    dimensions: vec![0.1],
                    collision: None,
                });
                let mut e = Engine::new(model).unwrap();
                e.gravity = [0.0; 3];
                e.contact.friction = 0.0;
                e.free_flight = FreeFlight::Never;
                e.contact_step = step;
                let gap = speed * step * fraction;
                e.q[0] = 0.1 + gap;
                e.qd[0] = -speed;
                e.advance(step);
                let expected = 0.1 + (gap - speed * step).max(0.0);
                assert!(
                    (e.q[0] - expected).abs() < 1e-10,
                    "dt={step}, speed={speed}, phase={fraction}: z={} expected={expected}",
                    e.q[0],
                );
                assert!(e.qd[0] <= 1e-10, "zero restitution must not add a rebound");
            }
        }
    }
}

/// A box on a prismatic joint along world `z`: it falls, it lands, and nothing else happens.
fn falling_box(mass: f64, half_height: f64) -> KinematicModel {
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
                dimensions: vec![0.4, 0.4, 2.0 * half_height],
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
        inertia_com: None,
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
        actuation: Default::default(),
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
            assert!(
                e.qd[0] <= 1e-10,
                "a zero-restitution landing must not bounce"
            );
        }
        // Linear translation admits the contact before crossing, to roundoff accuracy.
        assert!(
            lowest >= -1e-10,
            "dropped from {height} m, the lowest face reached {lowest} m"
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
    assert_eq!(
        rejected, 0,
        "nothing was near the floor, so nothing should have been rolled back"
    );

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
        e.free_flight = FreeFlight::DormandPrince {
            step: 1.0 / 20_000.0,
        };
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
    assert!(
        free > 0,
        "nothing to touch, so it should have been in free flight"
    );
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
    assert!(
        e.qd[0].abs() < 1e-9,
        "and it should not have moved: {}",
        e.qd[0]
    );
}

fn sphere_pair(masses: [f64; 2]) -> Engine {
    let mut model = falling_box(masses[0], 0.1);
    model.links[0].name = "a".into();
    model.links[0].geometry.as_mut().unwrap().shape = GeometryShape::Sphere;
    model.links[0].geometry.as_mut().unwrap().dimensions = vec![0.1];
    model.joints[0].child_link = "a".into();
    model.joints[0].axis = Some([1.0, 0.0, 0.0]);
    let mut second = model.links[0].clone();
    second.name = "b".into();
    second.mass = Some(masses[1]);
    model.links.push(second);
    let mut joint = model.joints[0].clone();
    joint.name = "second".into();
    joint.child_link = "b".into();
    model.joints.push(joint);
    let mut anchor = model.links[0].clone();
    anchor.name = "anchor".into();
    anchor.mass = None;
    anchor.geometry = None;
    model.links.push(anchor);
    for joint in &mut model.joints {
        joint.parent_link = "anchor".into();
    }
    let mut e = Engine::new(model).unwrap();
    e.ground = None;
    e.gravity = [0.0; 3];
    e.contact.friction = 0.0;
    e.enable_sphere_collisions().unwrap();
    e
}

#[test]
fn sphere_impact_matches_mass_restitution_and_energy_oracles() {
    for masses in [[1.0, 1.0], [1.0, 3.0], [5.0, 0.2]] {
        for restitution in [0.0, 0.5, 1.0] {
            for speed in [1.0, 100.0] {
                for phase in [1e-8, 0.15, 0.5, 0.95] {
                    let mut e = sphere_pair(masses);
                    let dt = 1.0 / 240.0;
                    e.q = vec![0.0, 0.2 + speed * dt * phase];
                    e.qd = vec![speed, 0.0];
                    e.contact.restitution = restitution;
                    e.advance(2.0 * dt);
                    let [a, b] = masses;
                    let expected = [
                        speed * (a - restitution * b) / (a + b),
                        speed * (1.0 + restitution) * a / (a + b),
                    ];
                    for (actual, target) in e.qd.iter().zip(expected) {
                        assert!(
                            (actual - target).abs() < 1e-7,
                            "{masses:?} e={restitution} v={speed} phase={phase}: {:?} expected {expected:?}",
                            e.qd
                        );
                    }
                    assert!(e.q[1] - e.q[0] >= 0.2 - 1e-9);
                    assert!((a * e.qd[0] + b * e.qd[1] - a * speed).abs() < 1e-8);
                    let after = 0.5 * (a * e.qd[0].powi(2) + b * e.qd[1].powi(2));
                    let loss =
                        0.5 * a * b / (a + b) * (1.0 - restitution * restitution) * speed * speed;
                    assert!((0.5 * a * speed * speed - after - loss).abs() < 1e-6);
                }
            }
        }
    }
}

#[test]
fn sphere_pair_releases_and_unsupported_contact_models_reject() {
    let mut e = sphere_pair([1.0, 2.0]);
    e.q = vec![0.0, 0.2];
    e.qd = vec![-1.0, 1.0];
    e.advance(0.1);
    assert_eq!(e.qd, vec![-1.0, 1.0]);
    assert!(
        Engine::new(falling_box(1.0, 0.1))
            .unwrap()
            .enable_sphere_collisions()
            .is_err()
    );
    assert!(
        Engine::new(double_pendulum())
            .unwrap()
            .enable_sphere_collisions()
            .is_err()
    );
}

#[test]
fn pair_impulse_preserves_shared_parent_momentum_and_misses_offset_spheres() {
    let base = sphere_pair([1.0, 3.0]);
    let mut model = base.model().clone();
    model
        .links
        .iter_mut()
        .find(|l| l.name == "anchor")
        .unwrap()
        .mass = Some(2.0);
    let mut joint = model.joints[0].clone();
    joint.name = "base".into();
    joint.parent_link = "world".into();
    joint.child_link = "anchor".into();
    model.joints.push(joint);
    let mut e = Engine::new(model.clone()).unwrap();
    e.ground = None;
    e.gravity = [0.0; 3];
    e.contact.friction = 0.0;
    e.contact.restitution = 1.0;
    e.enable_sphere_collisions().unwrap();
    e.q = vec![0.0, 0.4, 0.0];
    e.qd = vec![1.0, 0.0, 0.0];
    e.advance(0.5);
    assert!((e.qd[0] + 0.5).abs() < 1e-9 && (e.qd[1] - 0.5).abs() < 1e-9);
    assert!(
        e.qd[2].abs() < 1e-9,
        "internal impulse must not accelerate the shared reference mass"
    );
    model.joints[1].origin_xyz = Some([0.0, 0.21, 0.0]);
    let mut e = Engine::new(model).unwrap();
    e.ground = None;
    e.gravity = [0.0; 3];
    e.enable_sphere_collisions().unwrap();
    e.q = vec![0.0, 0.4, 0.0];
    e.qd = vec![100.0, 0.0, 0.0];
    e.advance(0.01);
    assert_eq!(e.qd, vec![100.0, 0.0, 0.0]);
}
