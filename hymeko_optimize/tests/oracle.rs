//! The numbers, against hand-computed oracles.
//!
//! Every value below was derived on paper from the model's own declarations before any of this code
//! ran, and none of it is a formula this crate contains. The whole point of the exercise is that
//! `hymeko_optimize` assembles nodal equations from a declared topology; if a voltage divider had
//! been written into the solver these would still pass, so the architectural claim is checked
//! separately in `authority.rs` and the numbers are checked here.
//!
//! # The oracles
//!
//! `Rb = Vb/Ib = 1.5/0.5 = 3` ohm. Connected, `R2` and `Rb` are in parallel at `Rp`; the operating
//! contract fixes `V_A = 1.5` on a `6 V` supply, so `R1 = 3 Rp`. On that manifold the resistor loss
//! is `2.25 + 9/R2` watt — falling in `R2` — so the design wants `R2` as large as the open-circuit
//! limit allows. `V_A_open = 6 R2/(R1+R2) <= 2` gives `R2 <= 1.5`, and the optimum sits ON that
//! inequality: `R2 = 1.5`, `Rp = 1`, `R1 = 3`.
//!
//! Everything else follows: `P_loss = 6.75 + 1.5 = 8.25 W`, `P_bulb = 1.5^2/3 = 0.75 W`,
//! `P_source = 6 x 1.5 = 9 W`, `efficiency = 0.75/9 = 1/12`, `V_A_open = 2.0 V` exactly.

mod common;

use hymeko_optimize::backend::{AugmentedLagrangian, Backend, External, multistart, starts};
use hymeko_optimize::problem::Problem;
use hymeko_optimize::read::project;

/// The optimum, on paper.
const R1: f64 = 3.0;
const R2: f64 = 1.5;

fn oracles() -> Vec<(&'static str, f64)> {
    vec![
        ("resistor_divider.bulb.Rb", 3.0),
        ("resistor_divider.V_A_connected", 1.5),
        ("resistor_divider.V_A_disconnected", 2.0),
        ("resistor_divider.P_R1_connected", 6.75),
        ("resistor_divider.P_R2_connected", 1.5),
        ("resistor_divider.P_bulb_connected", 0.75),
        ("resistor_divider.P_source_connected", 9.0),
        ("resistor_divider.P_resistor_loss_connected", 8.25),
        ("resistor_divider.efficiency_connected", 1.0 / 12.0),
    ]
}

fn model() -> Option<Problem> {
    let (ir, it) = common::compile("resistor_divider.hymeko")?;
    let problem = project(&ir, &it).expect("the model projects");
    problem.check().expect("the model is well posed");
    Some(problem)
}

macro_rules! problem {
    () => {
        match model() {
            Some(p) => p,
            None => {
                eprintln!("skipped: core/data/optimization is not in this checkout");
                return;
            }
        }
    };
}

/// The assembled network reproduces every oracle at the known optimum.
///
/// This separates the evaluation from the search: a fault here is in the nodal assembly, a fault in
/// the next test and not this one is in the solver.
#[test]
fn evaluation_matches_the_oracle() {
    let problem = problem!();
    let evaluation = problem.evaluate(&[R1, R2]).expect("it evaluates");

    for (name, expected) in oracles() {
        let i = problem
            .index_of_name(name)
            .unwrap_or_else(|| panic!("{name} is in the model"));
        let got = evaluation.values[i].value;
        assert!(
            (got - expected).abs() < 1e-12,
            "{name}: expected {expected}, assembled {got}"
        );
    }

    // The contract holds exactly, and the open-circuit limit is exactly met rather than cleared.
    assert!(evaluation.equalities[0].value.abs() < 1e-12);
    assert!(
        evaluation.inequalities[0].value.abs() < 1e-12,
        "the open-circuit inequality is ACTIVE at the optimum: {}",
        evaluation.inequalities[0].value
    );
}

/// The gradients the dual arithmetic carries through the linear solve are the real ones.
///
/// Central differences on the VALUES, compared against the gradient the same evaluation produced.
/// A finite difference is good to about `1e-9` relatively here, which is far short of what the duals
/// give and far more than enough to catch a wrong chain rule.
#[test]
fn gradients_survive_the_linear_solve() {
    let problem = problem!();
    let x = [2.7, 1.9];
    let evaluation = problem.evaluate(&x).expect("it evaluates");

    for k in 0..2 {
        let delta = 1e-6 * x[k];
        let mut up = x;
        let mut down = x;
        up[k] += delta;
        down[k] -= delta;
        let up = problem.evaluate(&up).expect("it evaluates");
        let down = problem.evaluate(&down).expect("it evaluates");

        for (name, _) in oracles() {
            let i = problem.index_of_name(name).expect("in the model");
            let differenced = (up.values[i].value - down.values[i].value) / (2.0 * delta);
            let exact = evaluation.values[i].grad[k];
            let scale = exact.abs().max(differenced.abs()).max(1e-6);
            assert!(
                (exact - differenced).abs() / scale < 1e-6,
                "d{name}/dx{k}: exact {exact}, differenced {differenced}"
            );
        }
    }
}

/// The in-process backend finds the optimum from the model's declared start.
#[test]
fn augmented_lagrangian_finds_the_optimum() {
    let problem = problem!();
    let backend = AugmentedLagrangian::default();
    let solved = backend
        .solve(&problem, &problem.start())
        .expect("it converges");

    assert!(solved.feasible(), "{:#?}", solved.equalities);
    assert!(
        (solved.decisions[0].value - R1).abs() < 1e-7,
        "{:?}",
        solved.decisions
    );
    assert!(
        (solved.decisions[1].value - R2).abs() < 1e-7,
        "{:?}",
        solved.decisions
    );
    for (name, expected) in oracles() {
        let got = solved.value_of(name).expect("in the result");
        assert!(
            (got - expected).abs() < 1e-6,
            "{name}: expected {expected}, solved {got}"
        );
    }
    assert!((solved.objective - 8.25).abs() < 1e-6);
}

/// A different backend, in a different language, on the same neutral problem.
///
/// Skipped rather than failed where SciPy is not installed: an absent interpreter is a fact about
/// the machine, and a test that turned it into a red build would be reporting the wrong thing.
#[test]
fn external_backend_agrees() {
    let problem = problem!();
    let script = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("backends/slsqp.py");
    let python = std::env::var("HYMEKO_PYTHON").unwrap_or_else(|_| "python".into());
    if std::process::Command::new(&python)
        .args(["-c", "import scipy"])
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .map(|s| !s.success())
        .unwrap_or(true)
    {
        eprintln!("skipped: no {python} with scipy on this machine");
        return;
    }

    let backend = External::slsqp(&python, script.to_str().expect("a path"));
    let solved = backend
        .solve(&problem, &problem.start())
        .expect("it converges");

    assert!(solved.feasible());
    assert!(
        (solved.decisions[0].value - R1).abs() < 1e-6,
        "{:?}",
        solved.decisions
    );
    assert!(
        (solved.decisions[1].value - R2).abs() < 1e-6,
        "{:?}",
        solved.decisions
    );

    // The two backends agree, which is the evidence that neither of them contains the design: a
    // leaked formula would have to have leaked into both, identically, in two languages.
    let mine = AugmentedLagrangian::default()
        .solve(&problem, &problem.start())
        .expect("it converges");
    for (name, _) in oracles() {
        let a = mine.value_of(name).expect("in the result");
        let b = solved.value_of(name).expect("in the result");
        assert!(
            (a - b).abs() / a.abs().max(1e-9) < 1e-5,
            "{name}: rust {a}, scipy {b}"
        );
    }
}

/// Many starting points, one answer, and the same answer twice.
///
/// The starting lattice is deterministic, so this test also checks reproducibility: the second call
/// to `starts` must produce the identical points, and the whole multistart must produce the identical
/// result. There is no seed to record because there is no randomness to seed.
#[test]
fn deterministic_multistart() {
    let problem = problem!();
    assert_eq!(starts(&problem, 7), starts(&problem, 7));

    let backend = AugmentedLagrangian::default();
    let first = multistart(&backend, &problem, 7);
    let converged: Vec<_> = first.iter().filter_map(|r| r.as_ref().ok()).collect();
    assert!(
        converged.len() >= 4,
        "at least most of the lattice should converge, got {}/{}",
        converged.len(),
        first.len()
    );
    for solved in &converged {
        assert!(solved.feasible());
        assert!(
            (solved.decisions[0].value - R1).abs() < 1e-5
                && (solved.decisions[1].value - R2).abs() < 1e-5,
            "start {:?} landed at {:?}",
            solved.start,
            solved.decisions.iter().map(|d| d.value).collect::<Vec<_>>()
        );
    }

    let second = multistart(&backend, &problem, 7);
    for (a, b) in first.iter().zip(&second) {
        match (a, b) {
            (Ok(a), Ok(b)) => {
                assert_eq!(a.start, b.start);
                for (p, q) in a.decisions.iter().zip(&b.decisions) {
                    assert_eq!(p.value.to_bits(), q.value.to_bits(), "{} moved", p.name);
                }
            }
            (Err(a), Err(b)) => assert_eq!(a, b),
            _ => panic!("the same start converged in one run and not the other"),
        }
    }
}

/// The result carries the declarations it came from, so a number can be traced back to its source.
#[test]
fn result_is_traceable_to_the_model() {
    let problem = problem!();
    let solved = AugmentedLagrangian::default()
        .solve(&problem, &problem.start())
        .expect("it converges");

    for resolved in &solved.quantities {
        let i = problem.index_of(resolved.decl).expect("a live declaration");
        assert_eq!(problem.quantities[i].name, resolved.name);
    }
    // The approximation travelled with its quantity rather than being lost at the boundary.
    let rb = solved
        .quantities
        .iter()
        .find(|q| q.name.ends_with(".Rb"))
        .expect("the linearised bulb resistance");
    assert!(rb.approximation.is_some());

    let json = solved.to_json();
    assert!(json.contains("\"decl\""));
    assert!(json.contains("operating-point linearisation"));
    assert!(json.contains("\"satisfied\": true"));
}
