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
    (0..6).map(|i| (JointEncoding::Dh { theta: 0., d: d[i], a: a[i], alpha: alpha[i] }, JointType::Revolute)).collect()
}
const LIMITS: [[f64; 2]; 6] = [[-TAU, TAU]; 6];

#[test]
fn a_rigid_pick_offset_reaches_the_target_and_leaves_later_joints_fixed() {
    use hymeko_geometry::kinematics::ik::solve_standard_dh_at;
    let chain=ur5();
    let truth=[0.3,-1.2,1.4,-1.6,-1.5,0.4];
    let offset=Se3::translation(&[0.08,-0.03,0.05]).compose(&Se3::axis_angle([0.,1.,0.],0.2));
    for frame in [3,6] {
        let wanted=pose(&truth,frame).compose(&offset);
        let target=IkTarget {frame,pose:wanted,orientation:frame==6,envelope:Envelope::POINT};
        let start:Vec<_>=truth.iter().enumerate().map(|(i,q)|q+if i<frame {0.08}else{0.}).collect();
        let solved=solve_standard_dh_at(&chain,&LIMITS,&start,&target,&offset,&IkOptions::ARM).unwrap();
        assert!(solved.converged,"{solved:?}");
        assert_eq!(solved.coordinates[frame..],start[frame..]);
        let actual=pose(&solved.coordinates,frame).compose(&offset);
        assert!(actual.translation.iter().zip(wanted.translation).all(|(a,b)|(a-b).abs()<1e-5));
        let mut invalid=offset;invalid.translation[0]=f64::NAN;
        assert_eq!(solve_standard_dh_at(&chain,&LIMITS,&start,&target,&invalid,&IkOptions::ARM),Err(IkError::Target));
    }
}

fn pose(q: &[f64], frame: usize) -> Se3 {
    standard_dh_chain(&ur5(), q).unwrap()[frame]
}

#[test]
fn a_reachable_tool_pose_is_found_again_from_a_different_start() {
    let chain = ur5();
    for (truth, offset) in [
        ([0.3, -1.2, 1.4, -1.6, -1.5, 0.4], 0.35),
        ([-0.8, -0.6, 0.9, -1.9, -1.4, -0.7], -0.3),
        ([1.9, -1.8, -1.2, 0.5, 1.3, 2.2], 0.25),
    ] {
        let target = IkTarget { frame: 6, pose: pose(&truth, 6), orientation: true, envelope: Envelope::POINT };
        let start: Vec<f64> = truth.iter().map(|q| q + offset).collect();
        let solution = solve_standard_dh(&chain, &LIMITS, &start, &target, &IkOptions::default()).unwrap();
        assert!(solution.converged, "{solution:?}");
        let reached = pose(&solution.coordinates, 6);
        let gap: f64 = (0..3).map(|i| (reached.translation[i] - target.pose.translation[i]).powi(2)).sum::<f64>().sqrt();
        assert!(gap < 1e-5, "the tool reaches the target: {gap}");
        let (a, b) = (reached.quaternion(), target.pose.quaternion());
        let dot: f64 = (0..4).map(|i| a[i] * b[i]).sum();
        assert!(dot.abs() > 1. - 1e-8, "and its orientation");
    }
}

#[test]
fn a_middle_frame_moves_only_the_joints_before_it() {
    let chain = ur5();
    let start = [0.2, -1.0, 1.1, -1.4, -1.5, 0.3];
    let mut goal = pose(&start, 3);
    goal.translation[2] += 0.08;
    goal.translation[0] -= 0.05;
    let solution = solve_standard_dh(&chain, &LIMITS, &start, &IkTarget { frame: 3, pose: goal, orientation: false, envelope: Envelope::POINT }, &IkOptions::default()).unwrap();
    assert!(solution.converged && solution.orientation_error_rad == 0., "{solution:?}");
    assert_eq!(solution.coordinates[3..], start[3..], "the wrist keeps its start coordinates");
    assert!(solution.coordinates[..3] != start[..3]);
}

#[test]
fn limits_hold_and_an_unreachable_target_stops_at_the_nearest_reach() {
    let chain = ur5();
    let start = [0., -FRAC_PI_2, 0.5, -FRAC_PI_2, -FRAC_PI_2, 0.];
    // A narrow elbow: the target needs more bend than allowed.
    let mut narrow = LIMITS;
    narrow[2] = [0.4, 0.6];
    let bent = pose(&[0., -FRAC_PI_2, 1.8, -FRAC_PI_2, -FRAC_PI_2, 0.], 3);
    let clamped = solve_standard_dh(&chain, &narrow, &start, &IkTarget { frame: 3, pose: bent, orientation: false, envelope: Envelope::POINT }, &IkOptions::default()).unwrap();
    assert!((0.4..=0.6).contains(&clamped.coordinates[2]), "{:?}", clamped.coordinates);
    assert!(clamped.coordinates.iter().zip(&narrow).all(|(q, [lo, hi])| (*lo..=*hi).contains(q)));
    // Two metres out is beyond the 0.95 m reach: the arm stretches toward it and says so.
    let far = Se3::translation(&[2., 0.3, 0.4]);
    let reach = solve_standard_dh(&chain, &LIMITS, &start, &IkTarget { frame: 6, pose: far, orientation: false, envelope: Envelope::POINT }, &IkOptions::default()).unwrap();
    assert!(!reach.converged && reach.coordinates.iter().all(|q| q.is_finite()));
    let before = (0..3).map(|i| (pose(&start, 6).translation[i] - far.translation[i]).powi(2)).sum::<f64>().sqrt();
    assert!(reach.position_error_m < before - 0.3, "it got closer: {} from {before}", reach.position_error_m);
    assert!(reach.position_error_m > 2.05 - 1.0, "but not closer than the reach allows: {}", reach.position_error_m);
}

#[test]
fn invalid_requests_are_refused_not_solved() {
    let chain = ur5();
    let start = [0.; 6];
    let target = IkTarget { frame: 6, pose: pose(&start, 6), orientation: true, envelope: Envelope::POINT };
    let options = IkOptions::default();
    assert_eq!(solve_standard_dh(&chain, &LIMITS, &start, &IkTarget { frame: 0, ..target }, &options), Err(IkError::Frame { frame: 0, joints: 6 }));
    assert_eq!(solve_standard_dh(&chain, &LIMITS, &start, &IkTarget { frame: 7, ..target }, &options), Err(IkError::Frame { frame: 7, joints: 6 }));
    assert!(matches!(solve_standard_dh(&chain, &LIMITS, &start[..5], &target, &options), Err(IkError::Coordinates { .. })));
    let mut reversed = LIMITS;
    reversed[4] = [PI, -PI];
    assert_eq!(solve_standard_dh(&chain, &reversed, &start, &target, &options), Err(IkError::Limits { index: 4 }));
    let nowhere = IkTarget { pose: Se3::translation(&[f64::NAN, 0., 0.]), ..target };
    assert_eq!(solve_standard_dh(&chain, &LIMITS, &start, &nowhere, &options), Err(IkError::Target));
    assert_eq!(solve_standard_dh(&chain, &LIMITS, &start, &target, &IkOptions { damping: 0., ..options }), Err(IkError::Target));
    // Already there: no iteration is spent.
    let there = solve_standard_dh(&chain, &LIMITS, &start, &target, &options).unwrap();
    assert!(there.converged && there.iterations == 0 && there.coordinates == start);
}
