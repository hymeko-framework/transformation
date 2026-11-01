//! `hymeko-optimize` — compile a model, project it, solve it, and write the evidence.
//!
//! ```text
//!   hymeko-optimize <model.hymeko> [--multistart <n>] [--out <file.json>] [--quiet]
//! ```
//!
//! The command takes a MODEL and a STRATEGY. It does not take the design: there is no flag for a
//! target voltage, no flag for a bound, no flag for which scenario to run — those are declarations,
//! and a command line that could override them would be a second place to edit them.

use std::path::PathBuf;
use std::process::ExitCode;

use hymeko::module_store::module_store::ModuleStore;
use hymeko::module_store::source_provider::StdFsProvider;
use hymeko::util::real_parser::RealParser;
use hymeko_optimize::backend::{AugmentedLagrangian, Backend, BackendError, starts};
use hymeko_optimize::read::project;
use hymeko_optimize::report::Solved;

fn main() -> ExitCode {
    let mut arguments = std::env::args().skip(1);
    let mut model: Option<PathBuf> = None;
    let mut multistart = 1usize;
    let mut out: Option<PathBuf> = None;
    let mut quiet = false;

    while let Some(argument) = arguments.next() {
        match argument.as_str() {
            "--multistart" => {
                multistart = arguments
                    .next()
                    .and_then(|v| v.parse().ok())
                    .unwrap_or(1)
                    .max(1)
            }
            "--out" => out = arguments.next().map(PathBuf::from),
            "--quiet" => quiet = true,
            "-h" | "--help" => {
                eprintln!("{}", include_str!("solve.usage"));
                return ExitCode::SUCCESS;
            }
            other if model.is_none() => model = Some(PathBuf::from(other)),
            other => {
                eprintln!("unexpected argument: {other}");
                return ExitCode::FAILURE;
            }
        }
    }
    let Some(model) = model else {
        eprintln!("{}", include_str!("solve.usage"));
        return ExitCode::FAILURE;
    };

    // ── compile and project ─────────────────────────────────────────────────
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = match store.compile(&model) {
        Ok(program) => program,
        Err(error) => {
            eprintln!("{}: {error:?}", model.display());
            return ExitCode::FAILURE;
        }
    };
    let problem = match project(&program.ir.clone(), &store.it) {
        Ok(problem) => problem,
        Err(error) => {
            eprintln!("projection: {error:?}");
            return ExitCode::FAILURE;
        }
    };
    if let Err(error) = problem.check() {
        eprintln!("the model is not well posed: {error:?}");
        return ExitCode::FAILURE;
    }
    if !quiet {
        eprintln!(
            "{}: {} quantities, {} decisions, {} branches over {} nets, {} scenarios, {} constraints",
            model.display(),
            problem.quantities.len(),
            problem.decisions.len(),
            problem.branches.len(),
            problem.nets.len(),
            problem.scenarios.len(),
            problem.constraints.len()
        );
    }

    // ── solve ───────────────────────────────────────────────────────────────
    let solver = AugmentedLagrangian::default();
    let strategy: &dyn Backend = &solver;

    let mut best: Option<Solved> = None;
    let mut failures: Vec<(Vec<f64>, BackendError)> = Vec::new();
    for start in starts(&problem, multistart) {
        match strategy.solve(&problem, &start) {
            Ok(solved) => {
                if !quiet {
                    eprintln!(
                        "  start {:?} -> objective {:.12} in {} evaluations",
                        start, solved.objective, solved.iterations
                    );
                }
                // Ties go to the earlier start, so the reported answer does not depend on the order
                // two equally good designs happened to be found in.
                if best.as_ref().is_none_or(|b| solved.objective < b.objective) {
                    best = Some(solved);
                }
            }
            Err(error) => {
                if !quiet {
                    eprintln!("  start {start:?} -> {error:?}");
                }
                failures.push((start, error));
            }
        }
    }

    let Some(best) = best else {
        eprintln!(
            "no start converged ({} tried); the model may be infeasible",
            failures.len()
        );
        return ExitCode::FAILURE;
    };

    // ── report ──────────────────────────────────────────────────────────────
    if !quiet {
        println!("backend      {}", best.backend);
        println!("objective    {:.12}", best.objective);
        for decision in &best.decisions {
            println!(
                "  {:<44} {:>18.12} {}",
                decision.name, decision.value, decision.unit
            );
        }
        println!("derived");
        for quantity in &best.quantities {
            if quantity.kind == hymeko_optimize::problem::Kind::Derived {
                println!(
                    "  {:<44} {:>18.12} {}{}",
                    quantity.name,
                    quantity.value,
                    quantity.unit,
                    if quantity.approximation.is_some() {
                        "  (approximation)"
                    } else {
                        ""
                    }
                );
            }
        }
        println!("constraints");
        for residual in best.equalities.iter().chain(&best.inequalities) {
            println!(
                "  {:<44} {:>18.3e} {}",
                residual.name,
                residual.value,
                if residual.satisfied {
                    "held"
                } else {
                    "VIOLATED"
                }
            );
        }
    }

    if let Some(path) = out {
        if let Err(error) = std::fs::write(&path, best.to_json()) {
            eprintln!("{}: {error}", path.display());
            return ExitCode::FAILURE;
        }
        if !quiet {
            eprintln!("evidence written to {}", path.display());
        }
    }
    if best.feasible() {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}
