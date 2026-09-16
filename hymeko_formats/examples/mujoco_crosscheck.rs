//! Emit MJCF from a HyMeKo robot and dump our own forward kinematics for the same pose.
//!
//! One source, two representations: the MJCF the emitter writes, and the link poses this
//! workspace resolves. A Python companion loads the first into MuJoCo and compares.
//!
//! Usage: `cargo run --example mujoco_crosscheck -- <robot.hymeko> <root_decl> <out_dir>`
//!
//! # There are two MJCF emitters, and only one of them emits a robot
//!
//! Measured 2026-08-31. `codegen::generate_description(.., Mjcf)` — the documented entry point —
//! dispatches to the templates under `transforms/mjcf/`, and on `arm_gripper.hymeko` it produced a
//! FLAT list of bodies directly under `<worldbody>`, with no `pos` on any of them and no `<joint>`
//! anywhere: geometry and mass, and no articulation at all. On `quadruped.hymeko` it returned an
//! error instead. `MjcfTransform::emit` walks the body stack and produces the real thing — nesting,
//! joints, positions, euler mounts, materials, inertials.
//!
//! So this example calls the transform directly, and says why: a cross-check run through the first
//! path would have compared our kinematics against a model that has none, and agreed with it.
use hymeko_formats::transforms::MjcfTransform;
use hymeko_geometry::forward::{JointValues, forward_kinematics};
use hymeko_geometry::kinematics::kinematic::extract_kinematic_model;
use hymeko_geometry::transforms::{DomainTransform, ModelView, TransformConfig};
use hymeko_query::QueryEngine;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (path, root, out) = (&args[0], &args[1], &args[2]);

    let mut store = hymeko::module_store::module_store::ModuleStore::new(
        hymeko::module_store::source_provider::StdFsProvider::new(),
        hymeko::util::real_parser::RealParser,
    );
    let program = store.compile(std::path::Path::new(path)).expect("compile");
    let engine = QueryEngine::new(&program.ir, &store.it);

    let model = extract_kinematic_model(&engine, root);

    // The RUST emitter, not the template one. `generate_description(.., Mjcf)` dispatches to
    // `transforms/mjcf/` and produced a flat list of bodies at the origin with no joints and no
    // positions — geometry and mass, no articulation. `MjcfTransform::emit` is the path that walks
    // the body stack.
    let cfg = TransformConfig::default().with_name(root);
    let xml = MjcfTransform
        .emit(&ModelView::Kinematic(model.clone()), &cfg)
        .expect("mjcf");
    std::fs::write(format!("{out}/{root}.xml"), &xml).expect("write mjcf");
    eprintln!(
        "{root}: {} links, {} joints",
        model.links.len(),
        model.joints.len()
    );

    // A deterministic, non-trivial pose: every revolute joint gets a distinct angle.
    let mut q = JointValues::new();
    for (i, j) in model.joints.iter().enumerate() {
        let v = 0.35 * ((i as f64 * 0.7).sin() + 0.25 * (i as f64 * 1.9).cos());
        q.insert(j.name.clone(), v);
    }

    let poses = forward_kinematics(&model, &q).expect("fk");
    let mut rows = Vec::new();
    for l in &model.links {
        if let Some(se) = poses.get(&l.name) {
            let r = se.rotation_matrix();
            let t = &se.translation;
            rows.push(format!(
                "{{\"link\":\"{}\",\"pos\":[{},{},{}],\"mat\":[{}]}}",
                l.name,
                t[0],
                t[1],
                t[2],
                r.iter()
                    .map(|v| v.to_string())
                    .collect::<Vec<_>>()
                    .join(",")
            ));
        }
    }
    let qrows: Vec<String> = model
        .joints
        .iter()
        .map(|j| format!("\"{}\":{}", j.name, q[&j.name]))
        .collect();
    std::fs::write(
        format!("{out}/{root}_fk.json"),
        format!(
            "{{\"q\":{{{}}},\"poses\":[{}]}}",
            qrows.join(","),
            rows.join(",")
        ),
    )
    .expect("write fk");

    // timing: full FK resolves, hot
    let t0 = std::time::Instant::now();
    let reps = 20_000;
    for _ in 0..reps {
        std::hint::black_box(forward_kinematics(&model, &q).unwrap());
    }
    let us = t0.elapsed().as_secs_f64() * 1e6 / reps as f64;
    eprintln!("hymeko FK: {us:.2} us/solve  ({:.0} solves/s)", 1e6 / us);
    std::fs::write(format!("{out}/{root}_timing.txt"), format!("{us}")).expect("write timing");
}
