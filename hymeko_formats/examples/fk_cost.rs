//! Where the forward-kinematics time actually goes.
//!
//! Three variants over the same model and pose:
//!   A  the shipped `forward_kinematics`  — HashMaps + String keys, rebuilt every call
//!   B  topology prepared once            — flat index arrays, same Se arithmetic
//!   C  arithmetic only                   — the Se compositions B does, and nothing else
use hymeko_clifford::Se;
use hymeko_geometry::forward::{forward_kinematics, joint_motion, joint_origin, FkPlan, JointValues};
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

    let mut q = JointValues::new();
    for (i, j) in model.joints.iter().enumerate() {
        q.insert(j.name.clone(), 0.35 * ((i as f64 * 0.7).sin()));
    }

    // ── prepared topology: link index -> (joint index, parent link index), in evaluation order
    let index: std::collections::HashMap<&str, usize> = model
        .links
        .iter()
        .enumerate()
        .map(|(i, l)| (l.name.as_str(), i))
        .collect();
    let mut parent_of: Vec<Option<(usize, usize)>> = vec![None; model.links.len()];
    for j in &model.joints {
        if let (Some(&c), Some(&p)) = (index.get(j.child_link.as_str()), index.get(j.parent_link.as_str())) {
            parent_of[c] = Some((model.joints.iter().position(|x| x.name == j.name).unwrap(), p));
        }
    }
    // evaluation order: repeatedly take links whose parent is already placed
    let mut order = Vec::new();
    let mut placed = vec![false; model.links.len()];
    for (i, p) in parent_of.iter().enumerate() {
        if p.is_none() { order.push(i); placed[i] = true; }
    }
    while order.len() < model.links.len() {
        let before = order.len();
        for i in 0..model.links.len() {
            if placed[i] { continue; }
            if let Some((_, p)) = parent_of[i] {
                if placed[p] { order.push(i); placed[i] = true; }
            }
        }
        if order.len() == before { break; }
    }
    let values: Vec<f64> = model.joints.iter().map(|j| q[&j.name]).collect();
    eprintln!("{root}: {} links, {} joints, {} ordered", model.links.len(), model.joints.len(), order.len());

    let reps = 20_000;
    let time = |name: &str, f: &mut dyn FnMut()| {
        f();
        let t0 = std::time::Instant::now();
        for _ in 0..reps { f(); }
        let us = t0.elapsed().as_secs_f64() * 1e6 / reps as f64;
        println!("  {name:<34} {us:>7.2} us");
        us
    };

    let a = time("A  shipped forward_kinematics", &mut || {
        std::hint::black_box(forward_kinematics(&model, &q).unwrap());
    });
    let mut poses: Vec<Se> = (0..model.links.len()).map(|_| Se::identity(3)).collect();
    let b = time("B  prepared topology, flat", &mut || {
        for &i in &order {
            poses[i] = match parent_of[i] {
                None => Se::identity(3),
                Some((ji, p)) => {
                    let j = &model.joints[ji];
                    poses[p]
                        .compose(&joint_origin(j))
                        .compose(&joint_motion(j, values[ji]))
                }
            };
        }
        std::hint::black_box(&poses);
    });
    let plan = FkPlan::new(&model).expect("plan");
    let mut pp = plan.identity_poses();
    let mut flat = plan.zero_values();
    for j in &model.joints { flat[plan.slot(&j.name).unwrap()] = q[&j.name]; }
    let d = time("D  FkPlan::resolve", &mut || {
        plan.resolve(&model, &flat, &mut pp);
        std::hint::black_box(&pp);
    });
    let _ = d;
    let c = time("C  Se arithmetic only", &mut || {
        let mut acc = Se::identity(3);
        for &i in &order {
            if let Some((ji, _)) = parent_of[i] {
                let j = &model.joints[ji];
                acc = acc.compose(&joint_origin(j)).compose(&joint_motion(j, values[ji]));
            }
        }
        std::hint::black_box(&acc);
    });
    println!("\n  bookkeeping (A - B): {:.2} us   {:.0}% of A", a - b, 100.0 * (a - b) / a);
    println!("  Se allocation+math (C): {c:.2} us   {:.0}% of A", 100.0 * c / a);
}
