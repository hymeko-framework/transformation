use hymeko_clifford::Se3;
use hymeko_geometry::kinematics::{
    dh::standard_dh_chain,
    ik::{Envelope, IkError, IkOptions, IkTarget, solve_standard_dh},
    joints::{JointEncoding, JointType},
};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

/// The nominal UR5 table the room model declares (`robot.kinematics.dh`).
fn ur5() -> Vec<(JointEncoding, JointType)> {
    let a = [0., -0.425, -0.39225, 0., 0., 0.];
    let d = [0.089159, 0., 0., 0.10915, 0.09465, 0.0823];
    let alpha = [FRAC_PI_2, 0., 0., FRAC_PI_2, -FRAC_PI_2, 0.];
    (0..6)
        .map(|i| {
            (
                JointEncoding::Dh {
                    theta: 0.,
                    d: d[i],
                    a: a[i],
                    alpha: alpha[i],
                },
                JointType::Revolute,
            )
        })
        .collect()
}
const LIMITS: [[f64; 2]; 6] = [[-TAU, TAU]; 6];
const START: [f64; 6] = [0.3, -1.2, 1.4, -1.6, -1.5, 0.4];

fn pose(q: &[f64]) -> Se3 {
    standard_dh_chain(&ur5(), q).unwrap()[6]
}

fn rotation(pose: &Se3) -> [f64; 9] {
    let mut m = [0.; 9];
    pose.rotation_matrix_into(&mut m);
    m
}

/// `pose` moved by `local` along its own axes.
fn shifted(pose: Se3, local: [f64; 3]) -> Se3 {
    let r = rotation(&pose);
    let mut moved = pose;
    for i in 0..3 {
        moved.translation[i] += (0..3).map(|m| r[i * 3 + m] * local[m]).sum::<f64>();
    }
    moved
}

/// Where `frame` lies in `target`'s axes.
fn displacement(target: &Se3, frame: &Se3) -> [f64; 3] {
    let r = rotation(target);
    std::array::from_fn(|i| {
        (0..3)
            .map(|m| r[m * 3 + i] * (frame.translation[m] - target.translation[m]))
            .sum()
    })
}

fn envelope(bounds: [[f64; 2]; 6]) -> Envelope {
    Envelope { bounds }
}

#[test]
fn a_frame_already_inside_the_envelope_stays_where_it_is() {
    let goal = shifted(pose(&START), [0.02, -0.01, 0.]);
    let target = IkTarget {
        frame: 6,
        pose: goal,
        orientation: true,
        envelope: envelope([
            [-0.03, 0.03],
            [-0.03, 0.03],
            [0., 0.],
            [0., 0.],
            [0., 0.],
            [0., 0.],
        ]),
    };
    let there = solve_standard_dh(&ur5(), &LIMITS, &START, &target, &IkOptions::default()).unwrap();
    assert!(
        there.converged && there.iterations == 0 && there.coordinates == START,
        "{there:?}"
    );
    // The exact pose is a move away.
    let exact = solve_standard_dh(
        &ur5(),
        &LIMITS,
        &START,
        &IkTarget {
            envelope: Envelope::POINT,
            ..target
        },
        &IkOptions::default(),
    )
    .unwrap();
    assert!(exact.converged && exact.iterations > 0);
}

#[test]
fn an_outside_frame_stops_at_the_near_edge_with_less_motion_than_the_exact_pose() {
    let goal = shifted(pose(&START), [0.2, 0., 0.]);
    let target = IkTarget {
        frame: 6,
        pose: goal,
        orientation: true,
        envelope: envelope([
            [-0.05, 0.05],
            [-0.01, 0.01],
            [-0.01, 0.01],
            [-0.05, 0.05],
            [-0.05, 0.05],
            [-0.05, 0.05],
        ]),
    };
    let options = IkOptions {
        tolerance: [1e-7, 1e-6],
        iterations: 256,
        ..IkOptions::default()
    };
    let near = solve_standard_dh(&ur5(), &LIMITS, &START, &target, &options).unwrap();
    assert!(near.converged, "{near:?}");
    let local = displacement(&goal, &pose(&near.coordinates));
    assert!(
        (local[0] + 0.05).abs() < 1e-5,
        "it stops at the edge facing where it came from: {local:?}"
    );
    assert!(
        local[1].abs() <= 0.01 + 1e-6 && local[2].abs() <= 0.01 + 1e-6,
        "{local:?}"
    );
    let exact = solve_standard_dh(
        &ur5(),
        &LIMITS,
        &START,
        &IkTarget {
            envelope: Envelope::POINT,
            ..target
        },
        &options,
    )
    .unwrap();
    let travel = |q: &[f64]| {
        q.iter()
            .zip(START)
            .map(|(a, b)| (a - b).abs())
            .fold(0., f64::max)
    };
    assert!(
        exact.converged && travel(&near.coordinates) < travel(&exact.coordinates),
        "{near:?} {exact:?}"
    );
    // Without orientation the box still lies along the target's axes.
    let position_only = solve_standard_dh(
        &ur5(),
        &LIMITS,
        &START,
        &IkTarget {
            orientation: false,
            ..target
        },
        &options,
    )
    .unwrap();
    let local = displacement(&goal, &pose(&position_only.coordinates));
    assert!(
        position_only.converged && (local[0] + 0.05).abs() < 1e-5,
        "{local:?}"
    );
}

#[test]
fn a_free_spin_about_the_tool_axis_is_a_yaw_range() {
    // The last joint turns the flange about its own z axis: the same point, another yaw.
    let mut spun = START;
    spun[5] += 1.0;
    let target = IkTarget {
        frame: 6,
        pose: pose(&spun),
        orientation: true,
        envelope: Envelope::POINT,
    };
    let fixed = solve_standard_dh(&ur5(), &LIMITS, &START, &target, &IkOptions::default()).unwrap();
    assert!(
        fixed.converged && fixed.iterations > 0 && (fixed.coordinates[5] - spun[5]).abs() < 1e-3,
        "{fixed:?}"
    );
    let free = IkTarget {
        envelope: envelope([[0., 0.], [0., 0.], [0., 0.], [0., 0.], [0., 0.], [-PI, PI]]),
        ..target
    };
    let kept = solve_standard_dh(&ur5(), &LIMITS, &START, &free, &IkOptions::default()).unwrap();
    assert!(
        kept.converged && kept.iterations == 0 && kept.coordinates == START,
        "{kept:?}"
    );
}

#[test]
fn an_envelope_out_of_order_or_range_is_refused() {
    let target = IkTarget {
        frame: 6,
        pose: pose(&START),
        orientation: true,
        envelope: Envelope::POINT,
    };
    let options = IkOptions::default();
    let mut bounds = [[0.; 2]; 6];
    for (index, bad) in [
        (0, [0.1, -0.1]),
        (2, [f64::NAN, 0.]),
        (3, [-4., 0.]),
        (4, [0., 1.7]),
        (5, [0., f64::INFINITY]),
    ] {
        bounds[index] = bad;
        let refused = solve_standard_dh(
            &ur5(),
            &LIMITS,
            &START,
            &IkTarget {
                envelope: envelope(bounds),
                ..target
            },
            &options,
        );
        assert_eq!(refused, Err(IkError::Target), "bound {index} {bad:?}");
        bounds[index] = [0.; 2];
    }
    // An envelope offset from the pose is a region beside it, not an error.
    bounds[2] = [0.05, 0.1];
    assert!(
        solve_standard_dh(
            &ur5(),
            &LIMITS,
            &START,
            &IkTarget {
                envelope: envelope(bounds),
                ..target
            },
            &options
        )
        .is_ok()
    );
}

#[test]
fn the_nearest_pose_of_an_inside_frame_is_the_frame() {
    let goal = pose(&START);
    let mut turned = START;
    turned[3] += 0.1;
    turned[5] -= 0.4;
    let frame = pose(&turned);
    let wide = envelope([
        [-1., 1.],
        [-1., 1.],
        [-1., 1.],
        [-PI, PI],
        [-FRAC_PI_2, FRAC_PI_2],
        [-PI, PI],
    ]);
    let (position, rotation_now) = wide.nearest(
        &goal.translation,
        &rotation(&goal),
        &frame.translation,
        &rotation(&frame),
        true,
    );
    let expected = rotation(&frame);
    assert!(
        (0..3).all(|i| (position[i] - frame.translation[i]).abs() < 1e-12),
        "{position:?}"
    );
    assert!(
        (0..9).all(|i| (rotation_now[i] - expected[i]).abs() < 1e-12),
        "{rotation_now:?} {expected:?}"
    );
}
