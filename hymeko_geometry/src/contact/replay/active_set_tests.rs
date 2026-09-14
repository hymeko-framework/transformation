//! Manufactured, independent oracles for the bounded active-set subspace correction.
use super::{
    active_set::{MAX_ATTEMPTS, Rejection, State, WINDOW},
    dense, *,
};

fn identity(n: usize) -> Vec<f64> {
    (0..n * n).map(|i| f64::from(i / n == i % n)).collect()
}

fn problem(matrix: Vec<f64>, rhs: Vec<f64>, limits: Vec<Limit>) -> Problem {
    let n = rhs.len();
    let problem = Problem {
        matrix,
        rhs,
        limits,
        response: vec![],
        reference: vec![0.0; n],
    };
    problem.validate().unwrap();
    problem
}

fn cone(normal: usize, first: usize, second: usize, mu: f64) -> [Limit; 3] {
    [
        Limit::Pushing,
        Limit::Friction {
            normal,
            partner: second,
            mu,
        },
        Limit::Friction {
            normal,
            partner: first,
            mu,
        },
    ]
}

fn counted(outcome: &Outcome) -> usize {
    outcome.corrections_accepted + outcome.corrections_rejected.iter().sum::<usize>()
}

#[test]
fn shared_elimination_pivots_and_rejects_singular_or_nonfinite_systems() {
    let mut matrix = [
        0.0, 2.0, 0.0, 1.0, 1.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 4.0, 5.0, 1.0, 0.0, 0.0,
    ];
    let mut rhs = [-1.5, 7.0, 2.0, 4.0];
    assert!(dense::solve_in_place(&mut matrix, &mut rhs, 4).is_some());
    for (x, expected) in rhs.iter().zip([1.0, -1.0, 2.0, 0.5]) {
        assert!((x - expected).abs() < 1e-14, "{rhs:?}");
    }
    assert!(dense::solve_in_place(&mut [1.0, 2.0, 2.0, 4.0], &mut [1.0, 2.0], 2).is_none());
    assert!(dense::solve_in_place(&mut [1.0, 0.0, 0.0, f64::NAN], &mut [1.0, 1.0], 2).is_none());
}

#[test]
fn stick_slip_correction_stops_on_the_analytic_cone_crossing() {
    // Identity operator: the sticking solution [1, 0.8, 0] leaves the mu = 0.5 cone. From
    // [1, 0.2, 0] the direction is [0, 0.6, 0]; |0.2 + 0.6 t| = 0.5 gives t = 0.5, and
    // [1, 0.5, 0] is the exact sliding solution with velocity [0, -0.3, 0].
    let p = problem(
        identity(3),
        vec![-1.0, -0.8, 0.0],
        cone(0, 1, 2, 0.5).to_vec(),
    );
    let mut impulse = vec![1.0, 0.2, 0.0];
    let before = p.residual(&impulse);
    let mut state = State::new();
    let after = state.correct(&p, &mut impulse, before).unwrap();
    let step = (impulse[1] - 0.2) / 0.6;
    assert!((step - 0.5).abs() < 1e-12, "{impulse:?}");
    assert!((impulse[0] - 1.0).abs() < 1e-12 && impulse[2] == 0.0);
    assert!(after < 1e-12 && after < before);
    assert_eq!(
        (
            state.work.unknowns,
            state.work.products,
            state.work.residuals
        ),
        (3, 2, 1)
    );
    assert_eq!((state.last.sticking, state.last.sliding), (1, 0));
}

#[test]
fn contact_and_joint_limit_rows_are_corrected_together_until_the_limit_releases() {
    // Normal row 0 couples to limit row 3 through A[0][3] = 1. From p0 = [1, 0, 0, 0.5] the
    // sticking/active solution p* = [1.5, 0, 0, -0.25] gives d = [0.5, 0, 0, -0.75]; the limit
    // multiplier reaches zero at t = 2/3, candidate [4/3, 0, 0, 0]. By hand, the only remaining
    // row error is the normal: |4/3 - (4/3 + 1/24)| / (4/3) = 1/32, down from 0.5.
    let mut matrix = identity(4);
    matrix[0] = 2.0;
    matrix[3] = 1.0;
    matrix[12] = 1.0;
    matrix[15] = 2.0;
    let mut limits = cone(0, 1, 2, 0.8).to_vec();
    limits.push(Limit::Pushing);
    let p = problem(matrix, vec![-2.75, 0.0, 0.0, -1.0], limits);
    let mut impulse = vec![1.0, 0.0, 0.0, 0.5];
    let before = p.residual(&impulse);
    assert!((before - 0.5).abs() < 1e-15);
    let mut state = State::new();
    let after = state.correct(&p, &mut impulse, before).unwrap();
    assert!((impulse[0] - 4.0 / 3.0).abs() < 1e-14, "{impulse:?}");
    assert!(impulse[3].abs() < 1e-15 && impulse[1] == 0.0 && impulse[2] == 0.0);
    assert!((after - 1.0 / 32.0).abs() < 1e-14, "{after}");
    assert_eq!((state.last.unknowns, state.last.unilateral), (4, 2));
}

#[test]
fn inactive_unilateral_rows_stop_the_step_at_zero_velocity() {
    // Free row 0 couples to inactive pushing row 1 (p = 0, v = 0.2). The free solve gives
    // d = [1, 0], whose image lowers row 1 velocity by 0.5 per unit: it closes at t = 0.4.
    let p = problem(
        vec![1.0, -0.5, -0.5, 1.0],
        vec![-1.0, 0.2],
        vec![Limit::Free, Limit::Pushing],
    );
    let mut impulse = vec![0.0; 2];
    let before = p.residual(&impulse);
    let mut state = State::new();
    let after = state.correct(&p, &mut impulse, before).unwrap();
    assert!(
        (impulse[0] - 0.4).abs() < 1e-15 && impulse[1] == 0.0,
        "{impulse:?}"
    );
    assert!(
        (before - 1.0).abs() < 1e-15 && (after - 0.6).abs() < 1e-15,
        "{after}"
    );
    assert_eq!(state.last.inactive, 1);
}

#[test]
fn rejected_corrections_leave_the_iterate_bitwise_unchanged() {
    let cases = [
        // Exact sticking solution: no strict decrease is possible.
        (
            tests::impact(0.8, [-1.0, 0.0, 0.0]),
            vec![1.0, 0.0, 0.0],
            Rejection::NoDecrease,
        ),
        // Rank-one free rows: the reduced system is singular.
        (
            problem(vec![1.0; 4], vec![-1.0, -2.0], vec![Limit::Free; 2]),
            vec![0.0; 2],
            Rejection::Singular,
        ),
        // An active unilateral row one rounding step above zero admits no step.
        (
            problem(vec![1.0], vec![1.0], vec![Limit::Pushing]),
            vec![1e-15],
            Rejection::Blocked,
        ),
    ];
    for (p, start, reason) in cases {
        let mut impulse = start.clone();
        let residual = p.residual(&impulse);
        let mut state = State::new();
        assert_eq!(state.correct(&p, &mut impulse, residual), Err(reason));
        assert_eq!(impulse, start);
    }
}

#[test]
fn a_fast_case_never_triggers_and_equals_plain_nsgs_bitwise() {
    let mut problem = tests::impact(0.8, [-1.0, 1.26, 1.04]);
    problem.matrix[4] = 2.0;
    let settings = Settings {
        tolerance: 1e-10,
        ..Default::default()
    };
    let solve = |method| {
        Replay::new(&problem, method, &[None; 3])
            .unwrap()
            .solve(settings)
            .unwrap()
    };
    let (plain, active) = (solve(Method::Nsgs), solve(Method::NsgsActiveSet));
    assert!(plain.accepted && plain.sweeps < WINDOW);
    assert_eq!(active.corrections_attempted, 0);
    assert_eq!(
        (
            plain.buffer_bytes,
            plain.newton_steps,
            plain.residual_evaluations
        ),
        (
            active.buffer_bytes,
            active.newton_steps,
            active.residual_evaluations
        )
    );
    assert_eq!(
        (plain.impulse, plain.residual, plain.sweeps),
        (active.impulse, active.residual, active.sweeps)
    );
}

#[test]
fn rejected_plateau_corrections_back_off_and_stop_at_the_attempt_cap() {
    // Inconsistent rank-one free rows: Gauss-Seidel grows the impulse by one per sweep, so the
    // normalized residual is 1/s and r_s >= 0.9 r_(s-32) first holds at s = 320 (321 if the
    // equality rounds the other way). Every reduced system is singular; waits of 64, 128, ...
    // give five attempts before 2048 sweeps and reach the eight-attempt cap before 65536.
    let p = problem(vec![1.0; 4], vec![-1.0, -2.0], vec![Limit::Free; 2]);
    for (max_sweeps, attempts) in [(2048, 5), (65_536, MAX_ATTEMPTS)] {
        let settings = Settings {
            max_sweeps,
            ..Default::default()
        };
        let mut replay = Replay::new(&p, Method::NsgsActiveSet, &[None; 2]).unwrap();
        let outcome = replay.solve(settings).unwrap();
        assert!(!outcome.accepted && outcome.sweeps == max_sweeps);
        assert_eq!(outcome.corrections_attempted, attempts);
        assert_eq!(outcome.corrections_rejected, [attempts, 0, 0]);
        assert_eq!(counted(&outcome), outcome.corrections_attempted);
        assert_eq!(outcome.residual_evaluations, max_sweeps + 1);
        assert_eq!(outcome.correction_products, attempts);
        let first = replay.subspace.events[0].0;
        assert!(first == 320 || first == 321, "{first}");
        let again = replay.solve(settings).unwrap();
        assert_eq!(outcome.impulse, again.impulse);
    }
}

fn householder_operator(eigenvalues: [f64; 7]) -> Vec<f64> {
    let w = [1.0, 2.0, -1.0, 0.5, 3.0, -2.0, 1.0];
    let norm = w.iter().map(|x| x * x).sum::<f64>();
    let q = |i: usize, j: usize| f64::from(i == j) - 2.0 * w[i] * w[j] / norm;
    let mut matrix = vec![0.0; 49];
    for i in 0..7 {
        for j in 0..=i {
            let value = (0..7)
                .map(|k| q(i, k) * eigenvalues[k] * q(j, k))
                .sum::<f64>();
            matrix[i * 7 + j] = value;
            matrix[j * 7 + i] = value;
        }
    }
    matrix
}

#[test]
fn ill_conditioned_contact_joint_system_reaches_the_manufactured_sticking_solution() {
    // Q diag(lambda) Q with condition 4e6: two sticking contacts and one active joint limit,
    // manufactured with zero constraint velocity, b = -A p*.
    let matrix = householder_operator([4.0, 1.0, 0.3, 0.1, 1e-3, 1e-5, 1e-6]);
    let exact = [1.0, 0.1, -0.05, 0.8, -0.1, 0.12, 0.5];
    let rhs = (0..7)
        .map(|i| -(0..7).map(|j| matrix[i * 7 + j] * exact[j]).sum::<f64>())
        .collect();
    let mut limits = cone(0, 1, 2, 0.8).to_vec();
    limits.extend(cone(3, 4, 5, 0.8));
    limits.push(Limit::Pushing);
    let p = problem(matrix, rhs, limits);
    let settings = Settings::default();
    let plain = Replay::new(&p, Method::Nsgs, &[None; 7])
        .unwrap()
        .solve(settings)
        .unwrap();
    let mut replay = Replay::new(&p, Method::NsgsActiveSet, &[None; 7]).unwrap();
    let active = replay.solve(settings).unwrap();
    assert!(
        active.accepted && active.corrections_accepted >= 1,
        "{active:?} {:?}",
        replay.subspace.events
    );
    assert!(active.corrections_attempted <= MAX_ATTEMPTS);
    assert_eq!(counted(&active), active.corrections_attempted);
    for (got, want) in active.impulse.iter().zip(exact) {
        assert!((got - want).abs() < 1e-6, "{active:?}");
    }
    assert!(
        !plain.accepted || plain.sweeps > active.sweeps,
        "plain {plain:?} active {active:?}"
    );
    let again = replay.solve(settings).unwrap();
    assert_eq!(active.impulse, again.impulse);
    assert_eq!(
        (
            active.sweeps,
            active.corrections_attempted,
            active.correction_operations
        ),
        (
            again.sweeps,
            again.corrections_attempted,
            again.correction_operations
        )
    );
}
