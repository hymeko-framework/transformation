//! How accurate the contact solve is, and how much of that accuracy the iteration count buys.
//!
//! Everything else in the step is exact to rounding: the articulated sweeps are closed-form linear
//! algebra, and a plane contact is one support evaluation. **Projected Gauss-Seidel is the only
//! approximation in the chain**, so it is the only place where "accurate enough" is a question with
//! a number rather than an answer.
//!
//! The measure is the complementarity residual. The solution of
//!
//! ```text
//! A p + b >= 0,   p >= 0,   p . (A p + b) = 0
//! ```
//!
//! satisfies `min(p_k, r_k) = 0` for every `k`, with `r = A p + b`. That one expression covers all
//! three conditions: a negative impulse, a violated constraint, and a constraint carrying load while
//! separating all show up in it. So the largest `|min(p_k, r_k)|`, relative to the problem's own
//! scale, is the error of the whole solve.
//!
//! Run: `cargo run --release --example solver_accuracy -- <robot.hymeko> <root> [height]`

use std::time::Instant;

use hymeko_clifford::Se3;
use hymeko_geometry::articulated::AbaPlan;
use hymeko_geometry::collide::{Placement, colliders_of, place};
use hymeko_geometry::contact::{
    ConstraintSolver, ContactParameters, ground_constraints, ground_contacts,
};
use hymeko_geometry::forward::FkPlan;
use hymeko_geometry::kinematics::kinematic::extract_kinematic_model;
use hymeko_query::QueryEngine;

const GRAVITY: [f64; 3] = [0.0, 0.0, -9.80665];

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (path, root) = (&args[0], &args[1]);
    let ground: f64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(0.0);

    let mut store = hymeko::module_store::module_store::ModuleStore::new(
        hymeko::module_store::source_provider::StdFsProvider::new(),
        hymeko::util::real_parser::RealParser,
    );
    let program = store.compile(std::path::Path::new(path)).expect("compile");
    let engine = QueryEngine::new(&program.ir, &store.it);
    let model = extract_kinematic_model(&engine, root);

    let aba = AbaPlan::new(&model).expect("the topology resolves");
    let plan = FkPlan::new(&model).expect("the topology resolves");
    let colliders = colliders_of(&model);
    let mut parameters = ContactParameters::default();

    let n = aba.slots();
    let q: Vec<f64> = (0..n).map(|i| 0.35 * ((i as f64 * 0.7).sin())).collect();
    let qd: Vec<f64> = (0..n).map(|i| 0.4 * ((i as f64 * 1.3).cos())).collect();
    let tau = vec![0.0; n];
    let h = 1.0 / 240.0;

    let mut scratch = aba.scratch();
    let mut free = vec![0.0; n];
    let mut poses: Vec<Se3> = plan.identity_poses_se3();
    let mut placed: Vec<Placement> = Vec::new();
    let mut contacts = Vec::new();
    let mut constraints = Vec::new();
    let mut solver = ConstraintSolver::new();
    let mut velocity = vec![0.0; n];

    plan.resolve_se3(&model, &q, &mut poses);
    place(&colliders, &poses, &mut placed);
    ground_contacts(
        &colliders,
        &placed,
        ground,
        parameters.margin,
        &mut contacts,
    );
    let c = contacts.len();
    if c == 0 {
        println!("  {root}: nothing is touching at z = {ground}");
        return;
    }
    aba.accelerations(&model, &q, &qd, &tau, GRAVITY, &mut scratch, &mut free);
    ground_constraints(
        &aba,
        &poses,
        &colliders,
        &contacts,
        &qd,
        &parameters,
        h,
        &mut scratch,
        &mut constraints,
    );

    println!("  {root}: {n} joints, {c} contacts at z = {ground}");
    {
        let mut warm = ContactParameters {
            iterations: 30,
            ..parameters
        };
        warm.iterations = 30;
        aba.accelerations(&model, &q, &qd, &tau, GRAVITY, &mut scratch, &mut free);
        velocity.copy_from_slice(&qd);
        solver.solve(&aba, &constraints, &warm, &mut scratch, &mut velocity);
    }
    println!(
        "    {} constraints have no mobility behind them",
        solver.unsatisfiable()
    );
    println!("    iterations  unconverged  unsatisfiable      us");

    parameters.tolerance = 0.0; // measure the fixed-count behaviour first
    let mut reference = Vec::new();
    for iterations in [1usize, 3, 5, 10, 20, 30, 60, 120, 500, 4000] {
        parameters.iterations = iterations;
        aba.accelerations(&model, &q, &qd, &tau, GRAVITY, &mut scratch, &mut free);
        velocity.copy_from_slice(&qd);
        solver.solve(&aba, &constraints, &parameters, &mut scratch, &mut velocity);
        let (unconverged, unsatisfiable) = solver.complementarity_residual();
        let impulses: Vec<f64> = solver.impulses().to_vec();

        // and how far the impulses themselves are from the converged ones
        let drift = if reference.is_empty() {
            f64::NAN
        } else {
            let scale = reference
                .iter()
                .fold(0.0f64, |m: f64, v: &f64| m.max(v.abs()))
                .max(1e-30);
            impulses
                .iter()
                .zip(&reference)
                .fold(0.0f64, |m, (a, b): (&f64, &f64)| m.max((a - b).abs()))
                / scale
        };

        // time one solve at this iteration count
        let reps = 50_000;
        aba.accelerations(&model, &q, &qd, &tau, GRAVITY, &mut scratch, &mut free);
        for _ in 0..500 {
            velocity.copy_from_slice(&qd);
            solver.solve(&aba, &constraints, &parameters, &mut scratch, &mut velocity);
        }
        let t0 = Instant::now();
        for _ in 0..reps {
            velocity.copy_from_slice(&qd);
            solver.solve(&aba, &constraints, &parameters, &mut scratch, &mut velocity);
            std::hint::black_box(&velocity);
        }
        let us = t0.elapsed().as_secs_f64() * 1e6 / reps as f64;

        let _ = drift;
        println!(
            "    {iterations:>10}   {unconverged:>9.2e}   {unsatisfiable:>9.2e}   {us:>5.2}                ({} sweeps taken)",
            solver.sweeps()
        );
        if iterations == 4000 {
            reference = impulses;
        }
    }

    // the last row again, now that the reference exists
    parameters.iterations = 30;
    aba.accelerations(&model, &q, &qd, &tau, GRAVITY, &mut scratch, &mut free);
    velocity.copy_from_slice(&qd);
    solver.solve(&aba, &constraints, &parameters, &mut scratch, &mut velocity);
    let scale = reference
        .iter()
        .fold(0.0f64, |m: f64, v: &f64| m.max(v.abs()))
        .max(1e-30);
    let drift = solver
        .impulses()
        .iter()
        .zip(&reference)
        .fold(0.0f64, |m, (a, b): (&f64, &f64)| m.max((a - b).abs()))
        / scale;
    println!("    at a fixed 30 sweeps the impulses are within {drift:.2e} of the converged ones");

    // and what the shipped adaptive setting actually does
    let shipped = ContactParameters::default();
    aba.accelerations(&model, &q, &qd, &tau, GRAVITY, &mut scratch, &mut free);
    velocity.copy_from_slice(&qd);
    solver.solve(&aba, &constraints, &shipped, &mut scratch, &mut velocity);
    let (unconverged, _) = solver.complementarity_residual();
    let drift = solver
        .impulses()
        .iter()
        .zip(&reference)
        .fold(0.0f64, |m, (a, b): (&f64, &f64)| m.max((a - b).abs()))
        / scale;
    let reps = 50_000;
    for _ in 0..500 {
        velocity.copy_from_slice(&qd);
        solver.solve(&aba, &constraints, &shipped, &mut scratch, &mut velocity);
    }
    let t0 = Instant::now();
    for _ in 0..reps {
        velocity.copy_from_slice(&qd);
        solver.solve(&aba, &constraints, &shipped, &mut scratch, &mut velocity);
        std::hint::black_box(&velocity);
    }
    let us = t0.elapsed().as_secs_f64() * 1e6 / reps as f64;
    println!(
        "    SHIPPED (adaptive): {} sweeps, unconverged {unconverged:.2e},          within {drift:.2e} of converged, {us:.2} us",
        solver.sweeps()
    );
}
