//! What a whole physics step costs, phase by phase.
//!
//! The split follows the structure, and the structure is the point:
//!
//!   tree        articulated-body forward dynamics, `O(n)` in the joints
//!   detect      placements, bounding boxes, and the exact test on the survivors
//!   constrain   one articulated solve per contact to build `A = J M^-1 J^T`, then Gauss-Seidel
//!               on a dense system of size the CONTACT COUNT
//!
//! The comparison is MuJoCo's `mj_step` with contacts enabled: 11.90, 13.76 and 11.85 microseconds
//! on the same three models, from `mujoco_step.py`. That one is like-for-like in a way the
//! detection-only figure was not — both sides now detect, solve and integrate.
//!
//! Run: `cargo run --release --example engine_cost -- <robot.hymeko> <root> [height]`

use std::time::Instant;

use hymeko_clifford::Se3;
use hymeko_geometry::articulated::AbaPlan;
use hymeko_geometry::collide::{colliders_of, place, BroadPhase, Placement};
use hymeko_geometry::contact::{
    ground_constraints, ground_contacts, ConstraintSolver, ContactParameters,
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
    let broad = BroadPhase::new(&model, &colliders);
    let parameters = ContactParameters::default();

    let n = aba.slots();
    let q: Vec<f64> = (0..n).map(|i| 0.35 * ((i as f64 * 0.7).sin())).collect();
    let qd: Vec<f64> = (0..n).map(|i| 0.4 * ((i as f64 * 1.3).cos())).collect();
    let tau = vec![0.0; n];
    let h = 1.0 / 240.0;

    let mut scratch = aba.scratch();
    let mut free = vec![0.0; n];
    let mut poses: Vec<Se3> = plan.identity_poses_se3();
    let mut placed: Vec<Placement> = Vec::new();
    let mut boxes = Vec::new();
    let mut candidates = Vec::new();
    let mut contacts = Vec::new();
    let mut constraints = Vec::new();
    let mut solver = ConstraintSolver::new();
    let mut velocity = vec![0.0; n];

    // --- the tree ---------------------------------------------------------------------------------
    let tree_us = time(200_000, || {
        aba.accelerations(&model, &q, &qd, &tau, GRAVITY, &mut scratch, &mut free);
        std::hint::black_box(&free);
    });

    // --- detection --------------------------------------------------------------------------------
    let detect_us = time(100_000, || {
        plan.resolve_se3(&model, &q, &mut poses);
        place(&colliders, &poses, &mut placed);
        broad.overlapping(&colliders, &placed, &mut boxes, &mut candidates);
        ground_contacts(&colliders, &placed, ground, parameters.margin, &mut contacts);
        std::hint::black_box((&candidates, &contacts));
    });

    // --- the constraints --------------------------------------------------------------------------
    plan.resolve_se3(&model, &q, &mut poses);
    place(&colliders, &poses, &mut placed);
    ground_contacts(&colliders, &placed, ground, parameters.margin, &mut contacts);
    let c = contacts.len();

    let constrain_us = if c == 0 {
        0.0
    } else {
        // A step calls `accelerations` first for the free velocity, and that leaves the
        // factorisation and the local transforms the constraint phase needs. Establishing it ONCE
        // here rather than inside the loop is what keeps this measuring the constraint phase and
        // not the tree phase over again.
        aba.accelerations(&model, &q, &qd, &tau, GRAVITY, &mut scratch, &mut free);
        time(20_000, || {
            ground_constraints(
                &aba, &poses, &colliders, &contacts, &qd, &parameters, h, &mut scratch,
                &mut constraints,
            );
            velocity.copy_from_slice(&qd);
            solver.solve(
                &aba,
                &constraints,
                &parameters,
                &mut scratch,
                &mut velocity,
            );
            std::hint::black_box(&velocity);
        })
    };

    println!(
        "  {root:<12} {} joints, {} colliders, {c} ground contacts at z = {ground}",
        n,
        colliders.len()
    );
    println!("    tree        {tree_us:>8.3} us");
    println!(
        "    detect      {detect_us:>8.3} us   ({} broad-phase candidates)",
        candidates.len()
    );
    if c == 0 {
        println!("    constrain      (nothing is touching; raise the ground to exercise it)");
    } else {
        println!(
            "    constrain   {constrain_us:>8.3} us   {c} constraints -> a {c}x{c} dense solve \
             ({:.3} us each)",
            constrain_us / c as f64
        );
    }
    println!(
        "    TOTAL       {:>8.3} us per step",
        tree_us + detect_us + constrain_us
    );
}

fn time<F: FnMut()>(reps: usize, mut f: F) -> f64 {
    for _ in 0..200 {
        f();
    }
    let t0 = Instant::now();
    for _ in 0..reps {
        f();
    }
    t0.elapsed().as_secs_f64() * 1e6 / reps as f64
}
