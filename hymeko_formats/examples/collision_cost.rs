//! What collision detection costs on a declared mechanism, phase by phase.
//!
//! Three numbers, because they answer different questions:
//!
//!   pairs      how many ordered pairs survive the STRUCTURAL filter — the ones a joint does not
//!              already hold together. Fixed at load; it does not depend on the pose.
//!   broad      one pass of bounding boxes over those pairs, at a pose.
//!   narrow     the exact test (GJK) on whatever the broad phase let through.
//!
//! The comparison worth drawing is against MuJoCo on the same models, whose step costs about 9.5,
//! 11.1 and 7.7 microseconds MORE with contacts enabled than without — that difference is what this
//! has to fit inside.
//!
//! Run: `cargo run --release --example collision_cost -- <robot.hymeko> <root>`

use std::time::Instant;

use hymeko_geometry::collide::{BroadPhase, Placement, colliders_of, intersects, place};
use hymeko_geometry::forward::FkPlan;
use hymeko_geometry::kinematics::kinematic::extract_kinematic_model;
use hymeko_query::QueryEngine;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (path, root) = (&args[0], &args[1]);
    let mut store = hymeko::module_store::module_store::ModuleStore::new(
        hymeko::module_store::source_provider::StdFsProvider::new(),
        hymeko::util::real_parser::RealParser,
    );
    let program = store.compile(std::path::Path::new(path)).expect("compile");
    let engine = QueryEngine::new(&program.ir, &store.it);
    let model = extract_kinematic_model(&engine, root);

    let plan = FkPlan::new(&model).expect("the topology resolves");
    let colliders = colliders_of(&model);
    let broad = BroadPhase::new(&model, &colliders);

    let n = model.joints.len();
    let q: Vec<f64> = (0..n).map(|i| 0.35 * ((i as f64 * 0.7).sin())).collect();
    let mut poses = plan.identity_poses_se3();
    plan.resolve_se3(&model, &q, &mut poses);

    let bodies = colliders.len();
    let all_pairs = bodies * bodies.saturating_sub(1) / 2;
    let kept = broad.pairs().len();

    let mut boxes: Vec<([f64; 3], [f64; 3])> = Vec::with_capacity(bodies);
    let mut candidates: Vec<(usize, usize)> = Vec::with_capacity(kept);
    let mut placed: Vec<Placement> = Vec::with_capacity(bodies);

    // --- broad phase, including resolving the placements the narrow phase then reuses ------------
    for _ in 0..200 {
        place(&colliders, &poses, &mut placed);
        broad.overlapping(&colliders, &placed, &mut boxes, &mut candidates);
    }
    let survivors = candidates.len();
    let reps = 200_000;
    let t0 = Instant::now();
    for _ in 0..reps {
        place(&colliders, &poses, &mut placed);
        broad.overlapping(&colliders, &placed, &mut boxes, &mut candidates);
        std::hint::black_box(&candidates);
    }
    let broad_us = t0.elapsed().as_secs_f64() * 1e6 / reps as f64;

    // --- narrow phase, on whatever survived ------------------------------------------------------
    let narrow_us = time_narrow(&colliders, &placed, &candidates, 200_000);
    // --- and on EVERY structural pair, which is what a broad phase has to beat -------------------
    let all_narrow_us = time_narrow(&colliders, &placed, broad.pairs(), 100_000);

    let hits = candidates
        .iter()
        .filter(|&&(i, j)| {
            intersects(
                &colliders[i].shape,
                &placed[i],
                &colliders[j].shape,
                &placed[j],
            )
        })
        .count();

    println!("  {root}: {} links, {bodies} colliders", model.links.len());
    println!(
        "    pairs      {all_pairs:>5} total -> {kept:>5} after the structural filter \
         ({:.0}% removed by the joints alone)",
        100.0 * (all_pairs - kept) as f64 / all_pairs.max(1) as f64
    );
    println!(
        "    broad      {broad_us:>8.3} us   {kept} boxes -> {survivors} candidates, {hits} touching"
    );
    println!(
        "    narrow     {narrow_us:>8.3} us   on the {survivors} survivors \
         ({:.3} us each)",
        narrow_us / survivors.max(1) as f64
    );
    println!("    (no broad) {all_narrow_us:>8.3} us   GJK on all {kept} pairs, for comparison");
    println!("    TOTAL      {:>8.3} us per step", broad_us + narrow_us);
}

fn time_narrow(
    colliders: &[hymeko_geometry::collide::Collider],
    placed: &[Placement],
    pairs: &[(usize, usize)],
    reps: usize,
) -> f64 {
    if pairs.is_empty() {
        return 0.0;
    }
    let run = || {
        let mut hits = 0usize;
        for &(i, j) in pairs {
            if intersects(
                &colliders[i].shape,
                &placed[i],
                &colliders[j].shape,
                &placed[j],
            ) {
                hits += 1;
            }
        }
        hits
    };
    for _ in 0..200 {
        std::hint::black_box(run());
    }
    let t0 = Instant::now();
    for _ in 0..reps {
        std::hint::black_box(run());
    }
    t0.elapsed().as_secs_f64() * 1e6 / reps as f64
}
