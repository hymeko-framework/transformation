use super::*;
use hymeko_clifford::Se3;

pub(super) fn impact(mu: f64, rhs: [f64; 3]) -> Problem {
    Problem {
        matrix: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        rhs: rhs.to_vec(),
        limits: vec![
            Limit::Pushing,
            Limit::Friction {
                normal: 0,
                partner: 2,
                mu,
            },
            Limit::Friction {
                normal: 0,
                partner: 1,
                mu,
            },
        ],
        response: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        reference: vec![0.0; 3],
    }
}

#[test]
fn oblique_impacts_preserve_signorini_and_coulomb_in_all_methods() {
    for mu in [0.0, 0.2, 0.8, 1.5] {
        for speed in [-3.0, 0.0, 0.1, 2.0] {
            let p = impact(mu, [-1.0, speed, 0.4 * speed]);
            let t = speed.hypot(0.4 * speed);
            let scale = if t > mu { mu / t } else { 1.0 };
            let expected = [1.0, -speed * scale, -0.4 * speed * scale];
            for method in [Method::Pgs, Method::Admm, Method::AdmmSbsh] {
                let mut solver = Replay::new(&p, method, &[Some([0.0; 3]); 3]).unwrap();
                let settings = Settings {
                    max_sweeps: 4096,
                    tolerance: 1e-8,
                    ..Default::default()
                };
                let result = solver.solve(settings).unwrap();
                assert!(result.accepted, "mu={mu} speed={speed} {result:?}");
                for (got, want) in result.impulse.iter().zip(expected) {
                    assert!(
                        (got - want).abs() < 1e-7,
                        "{result:?} expected={expected:?}"
                    );
                }
                assert_eq!(result.impulse, solver.solve(settings).unwrap().impulse);
                assert!(result.sweeps <= settings.max_sweeps);
            }
        }
    }
}

#[test]
fn sliding_direction_error_is_not_hidden_by_cone_saturation() {
    let p = impact(0.8, [-1.0, 2.0, 0.0]);
    assert!(p.residual(&[1.0, -0.8, 0.0]) < 1e-12);
    assert!(p.residual(&[1.0, 0.8, 0.0]) > 0.1);
    assert!(p.residual(&[1.0, 0.0, 0.8]) > 0.1);
    // An associated cone-QP projection changes the normal impulse to 1.585... .
    let normal = (1.0 + 0.8 * 2.0) / (1.0 + 0.8_f64.powi(2));
    assert!(p.residual(&[normal, -0.8 * normal, 0.0]) > 0.1);
}

#[test]
fn coupled_unilateral_system_matches_independent_closed_form() {
    let p = Problem {
        matrix: vec![2.0, 0.5, 0.5, 1.0],
        rhs: vec![-1.0, -1.0],
        limits: vec![Limit::Pushing; 2],
        response: vec![],
        reference: vec![0.0; 2],
    };
    for method in [Method::Pgs, Method::Admm, Method::AdmmSbsh] {
        let mut replay =
            Replay::new(&p, method, &[Some([-1.0, 0.0, 0.0]), Some([1.0, 0.0, 0.0])]).unwrap();
        let result = replay
            .solve(Settings {
                tolerance: 1e-9,
                ..Default::default()
            })
            .unwrap();
        assert!(result.accepted, "{result:?}");
        assert!((result.impulse[0] - 2.0 / 7.0).abs() < 1e-8);
        assert!((result.impulse[1] - 6.0 / 7.0).abs() < 1e-8);
    }
}

#[test]
fn malformed_inputs_empty_system_and_work_caps_are_explicit() {
    let mut p = impact(0.8, [-1.0, 2.0, 0.0]);
    assert!(Replay::new(&p, Method::Pgs, &[]).is_err());
    p.matrix[0] = f64::NAN;
    assert!(p.validate().is_err());
    p.matrix[0] = 1.0;
    p.limits[1] = Limit::Friction {
        normal: 0,
        partner: 3,
        mu: 0.8,
    };
    assert!(p.validate().is_err());
    let empty = Problem {
        matrix: vec![],
        rhs: vec![],
        limits: vec![],
        response: vec![],
        reference: vec![],
    };
    for method in [Method::Pgs, Method::Admm, Method::AdmmSbsh] {
        let mut solver = Replay::new(&empty, method, &[]).unwrap();
        assert!(solver.solve(Settings::default()).unwrap().accepted);
        assert!(
            solver
                .solve(Settings {
                    max_sweeps: 0,
                    ..Default::default()
                })
                .is_err()
        );
    }
    let p = impact(0.8, [-1.0, 3.0, 0.0]);
    let mut solver = Replay::new(&p, Method::Admm, &[None; 3]).unwrap();
    let one = solver
        .solve(Settings {
            max_sweeps: 1,
            tolerance: 1e-12,
            ..Default::default()
        })
        .unwrap();
    assert_eq!(one.sweeps, 1);
    assert!(!one.accepted);
}

#[test]
fn immobile_violation_is_not_a_converged_fast_solve() {
    let p = Problem {
        matrix: vec![0.0],
        rhs: vec![-1.0],
        limits: vec![Limit::Pushing],
        response: vec![],
        reference: vec![0.0],
    };
    for method in [Method::Pgs, Method::Admm] {
        let mut replay = Replay::new(&p, method, &[None]).unwrap();
        let result = replay
            .solve(Settings {
                max_sweeps: 8,
                ..Default::default()
            })
            .unwrap();
        assert!(!result.accepted);
        assert!(result.residual.is_infinite());
    }
    let mut opening = p.clone();
    opening.rhs[0] = 1.0;
    assert_eq!(opening.residual(&[0.0]), 0.0);
}

#[test]
fn anisotropic_friction_oracle_distinguishes_the_legacy_projection_fixed_point() {
    // p=(1,-.48,-.64), u=(0,.3,.4) satisfy Signorini and maximum dissipation.
    let mut p = impact(0.8, [-1.0, 1.26, 1.04]);
    p.matrix[4] = 2.0;
    let expected = [1.0, -0.48, -0.64];
    assert!(p.residual(&expected) < 1e-12);
    let settings = Settings {
        max_sweeps: 4096,
        tolerance: 1e-8,
        ..Default::default()
    };
    let old = Replay::new(&p, Method::Pgs, &[None; 3])
        .unwrap()
        .solve(settings)
        .unwrap();
    assert!(
        !old.accepted && old.residual > 1e-3,
        "legacy defect must remain visible: {old:?}"
    );
    for equilibrate in [false, true] {
        let result = Replay::new(&p, Method::Admm, &[None; 3])
            .unwrap()
            .solve(Settings {
                equilibrate,
                ..settings
            })
            .unwrap();
        assert!(result.accepted, "{result:?}");
        for (actual, expected) in result.impulse.iter().zip(expected) {
            assert!((actual - expected).abs() < 1e-7);
        }
    }
    println!(
        "ANISOTROPIC legacy_residual={} expected={expected:?}",
        old.residual
    );
}

#[test]
fn pose_cycle_closes_despite_penetration_but_detects_an_independent_edge_error() {
    for height in [1.0, 0.0, -0.01, -0.5] {
        let mut a = Se3::identity();
        let mut b = Se3::identity();
        let mut c = Se3::identity();
        a.translation = [0.0, 0.0, height];
        b.translation = [0.2, 0.0, 0.0];
        c.translation = [0.0, 0.3, 0.0];
        let ab = a.inverse().compose(&b);
        let bc = b.inverse().compose(&c);
        let ca = c.inverse().compose(&a);
        let cycle = ab.compose(&bc).compose(&ca);
        assert!(cycle.translation.iter().all(|v| v.abs() < 1e-12));
        let mut inconsistent = bc;
        inconsistent.translation[0] += 0.01;
        let cycle = ab.compose(&inconsistent).compose(&ca);
        assert!((cycle.translation[0] - 0.01).abs() < 1e-12);
    }
}

#[test]
fn hybrid_dispatch_uses_the_original_residual_and_shares_the_work_cap() {
    let p = impact(0.8, [-1.0, 2.0, 0.0]);
    let mut replay = Replay::new(&p, Method::Hybrid, &[None; 3]).unwrap();
    let simple = replay.solve(Settings::default()).unwrap();
    assert!(simple.accepted && simple.iterations == 0 && simple.sweeps <= 16);
    let mut p = impact(0.8, [-1.0, 1.26, 1.04]);
    p.matrix[4] = 2.0;
    let mut replay = Replay::new(&p, Method::Hybrid, &[None; 3]).unwrap();
    let coupled = replay.solve(Settings::default()).unwrap();
    assert!(coupled.accepted && coupled.iterations > 0, "{coupled:?}");
    assert!(coupled.sweeps <= Settings::default().max_sweeps);
    let capped = replay
        .solve(Settings {
            max_sweeps: 16,
            ..Default::default()
        })
        .unwrap();
    assert!(!capped.accepted && capped.iterations == 0 && capped.sweeps == 16);
}

#[test]
fn cone_block_scaling_preserves_solution_with_large_unrelated_mobility() {
    let base = impact(0.8, [-1.0, 1.26, 1.04]);
    let mut p = Problem {
        matrix: vec![0.0; 16],
        rhs: vec![-1.0, 1.26, 1.04, 10.0],
        limits: base.limits.clone(),
        response: vec![],
        reference: vec![0.0; 4],
    };
    p.limits.push(Limit::Pushing);
    for k in 0..3 {
        p.matrix[k * 4 + k] = if k == 1 { 2.0 } else { 1.0 };
    }
    p.matrix[15] = 1e5;
    let mut replay = Replay::new(&p, Method::Admm, &[None; 4]).unwrap();
    let unscaled = replay
        .solve(Settings {
            equilibrate: false,
            ..Default::default()
        })
        .unwrap();
    let scaled = replay.solve(Settings::default()).unwrap();
    assert!(
        !unscaled.accepted && scaled.accepted,
        "unscaled={unscaled:?}, scaled={scaled:?}"
    );
    for (actual, expected) in scaled.impulse.iter().zip([1.0, -0.48, -0.64, 0.0]) {
        assert!((actual - expected).abs() < 1e-5);
    }
}

#[test]
fn bounded_replay_accepts_the_declared_cap_and_rejects_larger_or_asymmetric_inputs() {
    let n = 512;
    let mut p = Problem {
        matrix: vec![0.0; n * n],
        rhs: vec![-1.0; n],
        limits: vec![Limit::Pushing; n],
        response: vec![],
        reference: vec![0.0; n],
    };
    for k in 0..n {
        p.matrix[k * n + k] = 1.0;
    }
    let result = Replay::new(&p, Method::Pgs, &vec![None; n])
        .unwrap()
        .solve(Settings::default())
        .unwrap();
    assert!(result.accepted && result.impulse.iter().all(|p| *p == 1.0));
    p.matrix[1] = 0.5;
    assert!(p.validate().is_err());
    p.matrix[1] = 0.0;
    p.rhs.push(0.0);
    assert!(p.validate().is_err());
    let free = Problem {
        matrix: vec![2.0],
        rhs: vec![3.0],
        limits: vec![Limit::Free],
        response: vec![],
        reference: vec![0.0],
    };
    for method in [Method::Pgs, Method::Admm, Method::AdmmSbsh, Method::Hybrid] {
        let result = Replay::new(&free, method, &[None])
            .unwrap()
            .solve(Settings::default())
            .unwrap();
        assert!(result.accepted && (result.impulse[0] + 1.5).abs() < 1e-5);
    }
}

#[test]
fn certificate_rejects_arithmetic_overflow_and_overlapping_cones() {
    let mut p = impact(0.8, [-1.0, 0.0, 0.0]);
    p.matrix[4] = f64::MAX;
    assert!(p.validate().is_ok());
    assert_eq!(p.residual(&[1.0, 2.0, 0.0]), f64::INFINITY);
    p.rhs.resize(5, 0.0);
    p.reference.resize(5, 0.0);
    p.matrix = vec![0.0; 25];
    p.limits.extend([
        Limit::Friction {
            normal: 0,
            partner: 4,
            mu: 0.8,
        },
        Limit::Friction {
            normal: 0,
            partner: 3,
            mu: 0.8,
        },
    ]);
    assert!(p.validate().is_err());
}
