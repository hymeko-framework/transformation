use super::{
    local::{Contact, Work},
    *,
};

fn anisotropic() -> Problem {
    let mut p = tests::impact(0.8, [-1.0, 1.26, 1.04]);
    p.matrix[4] = 2.0;
    p
}

#[test]
fn nonlinear_blocks_solve_the_legacy_anisotropic_friction_failure() {
    let problem = anisotropic();
    for method in [Method::Nsgs, Method::NsgsFixed] {
        let mut replay = Replay::new(&problem, method, &[None; 3]).unwrap();
        let result = replay
            .solve(Settings {
                tolerance: 1e-10,
                ..Default::default()
            })
            .unwrap();
        assert!(result.accepted, "{result:?}");
        for (actual, expected) in result.impulse.iter().zip([1.0, -0.48, -0.64]) {
            assert!((actual - expected).abs() < 1e-8, "{result:?}");
        }
        assert!(result.newton_steps > 0 && result.sweeps <= 16);
        let again = replay
            .solve(Settings {
                tolerance: 1e-10,
                ..Default::default()
            })
            .unwrap();
        assert_eq!(result.impulse, again.impulse);
        assert_eq!(result.newton_steps, again.newton_steps);
    }
}

#[test]
fn alart_curnier_jacobian_matches_finite_differences_inside_each_regime() {
    let contact = Contact::new(&anisotropic(), [0, 1, 2]);
    for (p, rhs) in [
        ([0.2, 0.3, -0.2], [2.0, 1.0, 0.5]),
        ([1.0, 0.2, -0.1], [-1.0, -0.4, 0.1]),
        ([1.0, -0.4, -0.5], [-1.0, 1.26, 1.04]),
    ] {
        let (_, jacobian) = contact.equation(p, rhs);
        for col in 0..3 {
            let mut plus = p;
            let mut minus = p;
            plus[col] += 1e-6;
            minus[col] -= 1e-6;
            let fp = contact.equation(plus, rhs).0;
            let fm = contact.equation(minus, rhs).0;
            for row in 0..3 {
                assert!(((fp[row] - fm[row]) / 2e-6 - jacobian[row][col]).abs() < 1e-6);
            }
        }
    }
}

#[test]
fn projection_fallback_is_counted_and_cannot_replace_a_better_physical_iterate() {
    let problem = anisotropic();
    let contact = Contact::new(&problem, [0, 1, 2]);
    let rhs = [-1.0, 1.26, 1.04];
    let mut p = [0.0; 3];
    let original = contact.error(p, rhs);
    let mut work = Work::default();
    contact.solve_bounded(&mut p, rhs, 1e-10, 0, 20, &mut work);
    assert!(contact.error(p, rhs) < original);
    assert_eq!(work.newton, 0);
    assert!(work.projection > 0 && work.projection <= 20);
    let before = p;
    contact.solve_bounded(&mut p, rhs, 1e-10, 0, 0, &mut work);
    assert_eq!(before, p);
}

#[test]
fn local_precision_adapts_monotonically_without_relaxing_the_final_target() {
    for target in [1e-10, 1e-8, 1e-6] {
        let mut previous = f64::INFINITY;
        for error in [1.0, 0.1, 1e-3, 1e-5, 1e-7, 0.0] {
            let tolerance = nsgs::local_tolerance(error, target, true);
            assert!(tolerance >= target * 0.1 && tolerance <= previous);
            assert_eq!(nsgs::local_tolerance(error, target, false), target * 0.1);
            previous = tolerance;
        }
        assert_eq!(
            nsgs::local_tolerance(f64::INFINITY, target, true),
            target * 0.1
        );
    }
}

fn coupled() -> Problem {
    let one = tests::impact(0.8, [0.0; 3]);
    let mut limits = one.limits.clone();
    limits.extend([
        Limit::Pushing,
        Limit::Friction {
            normal: 3,
            partner: 5,
            mu: 0.8,
        },
        Limit::Friction {
            normal: 3,
            partner: 4,
            mu: 0.8,
        },
    ]);
    let matrix: Vec<_> = (0..36)
        .map(|i| if i / 6 == i % 6 { 2.0 } else { 0.2 })
        .collect();
    let exact = [1.0, -0.48, -0.64, 2.0, -0.96, -1.28];
    let velocity = [0.0, 0.3, 0.4, 0.0, 0.6, 0.8];
    let rhs = (0..6)
        .map(|i| velocity[i] - (0..6).map(|j| matrix[i * 6 + j] * exact[j]).sum::<f64>())
        .collect();
    Problem {
        matrix,
        rhs,
        limits,
        reference: vec![0.0; 6],
        response: vec![],
    }
}

#[test]
fn nonlinear_sweeps_keep_off_block_coupling_and_bound_all_local_work() {
    let problem = coupled();
    for method in [Method::Nsgs, Method::NsgsFixed] {
        let mut solver = Replay::new(&problem, method, &[None; 6]).unwrap();
        let capped = solver
            .solve(Settings {
                max_sweeps: 1,
                tolerance: 1e-10,
                ..Default::default()
            })
            .unwrap();
        assert!(!capped.accepted && capped.sweeps == 1);
        assert!(capped.newton_steps <= 40 && capped.projection_steps <= 40);
        let result = solver
            .solve(Settings {
                tolerance: 1e-9,
                ..Default::default()
            })
            .unwrap();
        assert!(result.accepted, "{result:?}");
        for (p, exact) in result
            .impulse
            .iter()
            .zip([1.0, -0.48, -0.64, 2.0, -0.96, -1.28])
        {
            assert!((p - exact).abs() < 1e-7, "{result:?}");
        }
    }
}

#[test]
fn normal_tangent_coupling_and_basis_rotation_preserve_manufactured_sliding_solutions() {
    // SPD matrix L L^T with normal/tangent coupling; expected p and u specify b independently.
    let base = [[2.0, 0.2, -0.1], [0.2, 1.5, 0.3], [-0.1, 0.3, 1.0]];
    let expected = [1.0, -0.48, -0.64];
    let velocity = [0.0, 0.3, 0.4];
    for angle in [0.0f64, 0.31, 1.2, 2.4] {
        let (s, c) = angle.sin_cos();
        let rotation = [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]];
        let rotate = |v: [f64; 3]| -> [f64; 3] {
            std::array::from_fn(|i| (0..3).map(|j| rotation[i][j] * v[j]).sum())
        };
        let mut problem = tests::impact(0.8, [0.0; 3]);
        for i in 0..3 {
            for j in 0..3 {
                problem.matrix[i * 3 + j] = (0..3)
                    .flat_map(|k| (0..3).map(move |l| rotation[i][k] * base[k][l] * rotation[j][l]))
                    .sum();
            }
        }
        let exact = rotate(expected);
        let v = rotate(velocity);
        problem.rhs = (0..3)
            .map(|i| {
                v[i] - (0..3)
                    .map(|j| problem.matrix[i * 3 + j] * exact[j])
                    .sum::<f64>()
            })
            .collect();
        let result = Replay::new(&problem, Method::Nsgs, &[None; 3])
            .unwrap()
            .solve(Settings {
                tolerance: 1e-9,
                ..Default::default()
            })
            .unwrap();
        assert!(result.accepted, "{angle} {result:?}");
        for (p, expected) in result.impulse.iter().zip(exact) {
            assert!((p - expected).abs() < 1e-7);
        }
    }
}

#[test]
fn empty_opening_immobile_and_nonfinite_cases_keep_the_shared_contract() {
    for method in [Method::Nsgs, Method::NsgsFixed] {
        let empty = Problem {
            matrix: vec![],
            rhs: vec![],
            limits: vec![],
            reference: vec![],
            response: vec![],
        };
        assert!(
            Replay::new(&empty, method, &[])
                .unwrap()
                .solve(Settings::default())
                .unwrap()
                .accepted
        );
        let mut p = tests::impact(0.8, [1.0, 2.0, -1.0]);
        let result = Replay::new(&p, method, &[None; 3])
            .unwrap()
            .solve(Settings::default())
            .unwrap();
        assert!(result.accepted && result.impulse == vec![0.0; 3]);
        p.matrix.fill(0.0);
        p.rhs[0] = -1.0;
        let result = Replay::new(&p, method, &[None; 3])
            .unwrap()
            .solve(Settings {
                max_sweeps: 2,
                ..Default::default()
            })
            .unwrap();
        assert!(!result.accepted && result.sweeps <= 2);
        assert!(result.newton_steps > 0 && result.projection_steps > 0);
        assert!(result.impulse.iter().all(|p| p.is_finite()));
        p.rhs[0] = f64::NAN;
        assert!(Replay::new(&p, method, &[None; 3]).is_err());
    }
}

#[test]
fn nonlinear_contact_solutions_survive_mobility_scaling_and_the_declared_row_cap() {
    for scale in [1e-6, 1.0, 1e6] {
        for mu in [0.0, 0.2, 0.8, 2.0] {
            let mut problem = tests::impact(mu, [-1.0, 2.0, 0.0]);
            for value in problem.matrix.iter_mut().chain(&mut problem.rhs) {
                *value *= scale;
            }
            let result = Replay::new(&problem, Method::Nsgs, &[None; 3])
                .unwrap()
                .solve(Settings {
                    tolerance: 1e-9,
                    ..Default::default()
                })
                .unwrap();
            assert!(result.accepted, "scale={scale} mu={mu} {result:?}");
            assert!((result.impulse[0] - 1.0).abs() < 1e-7);
            assert!((result.impulse[1] + mu.min(2.0)).abs() < 1e-7);
        }
    }
    let n = 512;
    let mut problem = Problem {
        matrix: vec![0.0; n * n],
        rhs: vec![-1.0; n],
        limits: vec![Limit::Pushing; n],
        reference: vec![0.0; n],
        response: vec![],
    };
    for k in 0..n {
        problem.matrix[k * n + k] = 1.0;
    }
    let result = Replay::new(&problem, Method::Nsgs, &vec![None; n])
        .unwrap()
        .solve(Settings {
            max_sweeps: 1,
            ..Default::default()
        })
        .unwrap();
    assert!(result.accepted && result.sweeps == 1 && result.newton_steps == 0);
}
