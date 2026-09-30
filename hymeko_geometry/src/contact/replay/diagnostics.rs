//! Test-only observation of the actual NSGS loop, without another solver implementation.
use super::*;

type Event = (usize, active_set::Attempt, Option<active_set::Rejection>);

const HARD_CAPTURES: [&str; 3] = [
    "mu0.8_j20_step960",
    "mu0.8_j20_step1440",
    "mu0.8_j20_step1920",
];

#[derive(Serialize)]
struct Snapshot {
    sweep: usize,
    residual: f64,
    next_local_tolerance: f64,
    impulse: Vec<f64>,
    velocity: Vec<f64>,
    row_error: Vec<f64>,
    newton: usize,
    projection: usize,
    discarded: usize,
}

impl Snapshot {
    fn take(sweep: usize, residual: f64, replay: &Replay, settings: Settings) -> Self {
        let p = &replay.solver.impulse;
        let mut velocity = vec![0.0; p.len()];
        replay.problem.velocity(p, &mut velocity);
        let scale = p.iter().fold(1.0f64, |a, x| a.max(x.abs()));
        let row_error = (0..p.len())
            .map(|k| replay.problem.row_residual(k, p, &velocity) / scale)
            .collect();
        Self {
            sweep,
            residual,
            next_local_tolerance: nsgs::local_tolerance(
                residual,
                settings.tolerance,
                replay.method != Method::NsgsFixed,
            ),
            impulse: p.clone(),
            velocity,
            row_error,
            newton: replay.nonlinear.stats.newton,
            projection: replay.nonlinear.stats.projection,
            discarded: replay.nonlinear.stats.discarded,
        }
    }
}

fn trace(
    problem: &Problem,
    method: Method,
    settings: Settings,
) -> (Outcome, Vec<Snapshot>, Vec<Event>) {
    let anchors = vec![None; problem.rhs.len()];
    let expected = Replay::new(problem, method, &anchors)
        .unwrap()
        .solve(settings)
        .unwrap();
    let mut observed = Replay::new(problem, method, &anchors).unwrap();
    let mut snapshots = Vec::new();
    let (sweeps, _) = observed.observe_nsgs(settings, |sweep, residual, replay| {
        snapshots.push(Snapshot::take(sweep, residual, replay, settings));
    });
    assert_eq!(expected.sweeps, sweeps);
    assert_eq!(expected.impulse, observed.solver.impulse);
    assert_eq!(expected.newton_steps, observed.nonlinear.stats.newton);
    assert_eq!(
        expected.projection_steps,
        observed.nonlinear.stats.projection
    );
    assert_eq!(
        expected.discarded_stages,
        observed.nonlinear.stats.discarded
    );
    let work = observed.subspace.work;
    assert_eq!(
        (
            expected.corrections_attempted,
            expected.corrections_accepted
        ),
        (work.attempted, work.accepted)
    );
    assert_eq!(
        expected.residual_evaluations,
        observed.nonlinear.residuals + work.residuals
    );
    assert_eq!(observed.subspace.events.len(), work.attempted);
    assert_eq!(snapshots.len(), sweeps + 1);
    assert_eq!(expected.residual, snapshots.last().unwrap().residual);
    for sample in &snapshots {
        assert_eq!(
            sample.residual,
            sample.row_error.iter().copied().fold(0.0, f64::max)
        );
    }
    (
        expected,
        snapshots,
        std::mem::take(&mut observed.subspace.events),
    )
}

#[test]
fn observation_preserves_empty_early_exit_and_budget_exhaustion() {
    let empty = Problem {
        matrix: vec![],
        rhs: vec![],
        limits: vec![],
        response: vec![],
        reference: vec![],
    };
    let (_, samples, _) = trace(&empty, Method::Nsgs, Settings::default());
    assert_eq!(samples.len(), 1);
    let coupled_free = Problem {
        matrix: vec![2.0, 1.0, 1.0, 2.0],
        rhs: vec![-3.0, -3.0],
        limits: vec![Limit::Free; 2],
        response: vec![],
        reference: vec![0.0; 2],
    };
    let (limited, samples, _) = trace(
        &coupled_free,
        Method::Nsgs,
        Settings {
            max_sweeps: 1,
            ..Default::default()
        },
    );
    assert!(!limited.accepted && limited.sweeps == 1 && samples.len() == 2);
    for scale in [1e-4, 1.0, 1e4] {
        let mut problem = tests::impact(0.8, [-1.0, 1.26, 1.04]);
        problem.matrix[4] = 2.0;
        for x in problem.matrix.iter_mut().chain(&mut problem.rhs) {
            *x *= scale;
        }
        for method in [Method::Nsgs, Method::NsgsFixed, Method::NsgsActiveSet] {
            for max_sweeps in [1, 64] {
                let (outcome, samples, _) = trace(
                    &problem,
                    method,
                    Settings {
                        max_sweeps,
                        tolerance: 1e-10,
                        ..Default::default()
                    },
                );
                assert!(outcome.sweeps <= max_sweeps);
                assert_eq!(samples[0].impulse, vec![0.0; 3]);
                assert_eq!(samples[0].velocity, problem.rhs);
                if max_sweeps == 64 {
                    assert!(outcome.accepted && outcome.sweeps < max_sweeps);
                    for (p, expected) in outcome.impulse.iter().zip([1.0, -0.48, -0.64]) {
                        assert!((p - expected).abs() < 1e-8);
                    }
                }
            }
        }
    }
    // Rank-one free rows: two singular correction attempts (sweeps 320/384) stay observable.
    let rank_one = Problem {
        matrix: vec![1.0; 4],
        rhs: vec![-1.0, -2.0],
        ..coupled_free
    };
    let (outcome, _, events) = trace(
        &rank_one,
        Method::NsgsActiveSet,
        Settings {
            max_sweeps: 400,
            ..Default::default()
        },
    );
    assert_eq!(outcome.corrections_rejected, [2, 0, 0]);
    assert!(
        events
            .iter()
            .all(|(_, _, reason)| *reason == Some(active_set::Rejection::Singular))
    );
    let immobile = Problem {
        matrix: vec![0.0],
        rhs: vec![-1.0],
        limits: vec![Limit::Pushing],
        response: vec![],
        reference: vec![0.0],
    };
    let (outcome, samples, _) = trace(&immobile, Method::Nsgs, Settings::default());
    assert!(!outcome.accepted && outcome.sweeps == 1);
    assert_eq!(samples.last().unwrap().residual, f64::INFINITY);
}

// Machine-written numeric ledger: one constraint row per line, no authored syntax.
// Columns: kind, normal, partner, mu, rhs, reference, matrix row, response row.
fn read_problem(path: &std::path::Path) -> Problem {
    let text = std::fs::read_to_string(path).unwrap();
    let rows: Vec<Vec<f64>> = text
        .lines()
        .map(|line| {
            line.split_whitespace()
                .map(|x| x.parse().unwrap())
                .collect()
        })
        .collect();
    let n = rows.len();
    let mut problem = Problem {
        matrix: vec![],
        rhs: vec![],
        limits: vec![],
        response: vec![],
        reference: vec![],
    };
    for row in rows {
        assert!(row.len() >= 6 + n);
        problem.limits.push(match row[0] {
            0.0 => Limit::Pushing,
            1.0 => Limit::Free,
            2.0 => Limit::Friction {
                normal: row[1] as usize,
                partner: row[2] as usize,
                mu: row[3],
            },
            _ => panic!("invalid machine ledger row kind"),
        });
        problem.rhs.push(row[4]);
        problem.reference.push(row[5]);
        problem.matrix.extend_from_slice(&row[6..6 + n]);
        problem.response.extend_from_slice(&row[6 + n..]);
    }
    problem.validate().unwrap();
    problem
}

fn write_trace(path: &std::path::Path, samples: &[Snapshot]) {
    use std::io::Write;
    let mut output = std::io::BufWriter::new(std::fs::File::create(path).unwrap());
    writeln!(output, "sweep,residual,next_local_tolerance,newton,projection,discarded,row,impulse,velocity,row_error").unwrap();
    for s in samples {
        for k in 0..s.impulse.len() {
            writeln!(
                output,
                "{},{:.17e},{:.17e},{},{},{},{},{:.17e},{:.17e},{:.17e}",
                s.sweep,
                s.residual,
                s.next_local_tolerance,
                s.newton,
                s.projection,
                s.discarded,
                k,
                s.impulse[k],
                s.velocity[k],
                s.row_error[k]
            )
            .unwrap();
        }
    }
    output.flush().unwrap();
}

/// One line per correction attempt; `bound` is -1 for a full step or a pre-step rejection.
fn write_corrections(path: &std::path::Path, events: &[Event]) {
    use std::io::Write;
    let mut output = std::io::BufWriter::new(std::fs::File::create(path).unwrap());
    writeln!(
        output,
        "sweep,unknowns,sticking,sliding,unilateral,inactive,step,bound,before,trial,outcome"
    )
    .unwrap();
    for (sweep, a, rejection) in events {
        let bound = a.bound.map_or(-1, |row| row as i64);
        let outcome = rejection.map_or_else(|| "Accepted".to_owned(), |r| format!("{r:?}"));
        writeln!(
            output,
            "{sweep},{},{},{},{},{},{:.17e},{bound},{:.17e},{:.17e},{outcome}",
            a.unknowns, a.sticking, a.sliding, a.unilateral, a.inactive, a.step, a.before, a.trial
        )
        .unwrap();
    }
    output.flush().unwrap();
}

fn directory(variable: &str) -> std::path::PathBuf {
    let path = std::path::PathBuf::from(std::env::var_os(variable).expect(variable));
    assert!(path.is_dir(), "{variable} must name an existing directory");
    path
}

/// Ledger input and trace output; writing next to the ledgers is refused.
fn separate_directories() -> (std::path::PathBuf, std::path::PathBuf) {
    let input = directory("NSGS_LEDGER_INPUT");
    let output = directory("ACTIVE_SET_OUTPUT");
    assert_ne!(
        input.canonicalize().unwrap(),
        output.canonicalize().unwrap()
    );
    (input, output)
}

fn capture(
    problem: &Problem,
    output: &std::path::Path,
    name: &str,
    method: Method,
    max_sweeps: usize,
) -> Outcome {
    eprintln!("NSGS trace start {name} {method:?} cap={max_sweeps}");
    let settings = Settings {
        max_sweeps,
        ..Default::default()
    };
    let (outcome, snapshots, events) = trace(problem, method, settings);
    eprintln!(
        "NSGS trace end {name} {method:?} sweeps={} residual={:.9e} accepted={} corrections={}/{}",
        outcome.sweeps,
        outcome.residual,
        outcome.accepted,
        outcome.corrections_accepted,
        outcome.corrections_attempted
    );
    let stem = format!("{name}_{method:?}_{max_sweeps}");
    write_trace(&output.join(format!("{stem}.csv")), &snapshots);
    if method == Method::NsgsActiveSet {
        write_corrections(&output.join(format!("{stem}_corrections.csv")), &events);
    }
    outcome
}

#[test]
#[ignore = "Explicit frozen-equation diagnostic; requires prepared numeric ledger directory"]
fn capture_nsgs_convergence() {
    let output = directory("NSGS_DIAGNOSTIC_OUTPUT");
    for name in HARD_CAPTURES {
        let problem = read_problem(&output.join(format!("{name}.txt")));
        for method in [Method::Nsgs, Method::NsgsFixed] {
            for max_sweeps in [2048, 8192] {
                capture(&problem, &output, name, method, max_sweeps);
            }
        }
    }
}

/// Reads the frozen ledgers without writing next to them; plain NSGS at the original cap is
/// repeated as the refactor parity trace. The larger cap only documents a failed 2048 run.
#[test]
#[ignore = "Explicit active-set diagnostic; reads frozen ledgers and writes a separate directory"]
fn capture_active_set_convergence() {
    let (input, output) = separate_directories();
    for name in HARD_CAPTURES {
        let problem = read_problem(&input.join(format!("{name}.txt")));
        capture(&problem, &output, name, Method::Nsgs, 2048);
        if !capture(&problem, &output, name, Method::NsgsActiveSet, 2048).accepted {
            capture(&problem, &output, name, Method::NsgsActiveSet, 8192);
        }
    }
}

/// Blind test (docs/plans/2026-09-14-contact-nsgs-blind): NSGS_LEDGER_NAMES come from plain Nsgs
/// results only. Each method is traced once, at the original cap or, if that fails, at 8192.
#[test]
#[ignore = "Explicit blind-test diagnostic; reads Rust-decoded ledgers and writes a separate directory"]
fn capture_blind_convergence() {
    let (input, output) = separate_directories();
    let names = std::env::var("NSGS_LEDGER_NAMES").expect("comma-separated capture names");
    for name in names.split(',') {
        let problem = read_problem(&input.join(format!("{name}.txt")));
        let anchors = vec![None; problem.rhs.len()];
        for method in [Method::Nsgs, Method::NsgsActiveSet] {
            let mut replay = Replay::new(&problem, method, &anchors).unwrap();
            let accepted = replay.solve(Settings::default()).unwrap().accepted;
            capture(
                &problem,
                &output,
                name,
                method,
                if accepted { 2048 } else { 8192 },
            );
        }
    }
}
