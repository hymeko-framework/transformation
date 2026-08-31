//! The prepared plan: same answers, no per-call bookkeeping, and the topology taken from the arena.
//!
//! The allocation claims live next door in `fk_allocation.rs`, in their own binary: a counting
//! global allocator sees every thread, and the tests in one binary run in parallel, so a counter
//! switched on here would also record what these tests are doing.

use std::collections::HashMap;
use std::path::PathBuf;

use hymeko::module_store::module_store::ModuleStore;
use hymeko::module_store::source_provider::StdFsProvider;
use hymeko::util::real_parser::RealParser;
use hymeko_geometry::forward::{forward_kinematics, FkPlan, JointValues};
use hymeko_geometry::kinematics::kinematic::{extract_kinematic_model, KinematicModel};
use hymeko_query::QueryEngine;

// ─── models ─────────────────────────────────────────────────────────────────────────────────────

fn compile(path: PathBuf, root: &str) -> KinematicModel {
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = store.compile(&path).expect("the fixture compiles");
    let engine = QueryEngine::new(&program.ir, &store.it);
    extract_kinematic_model(&engine, root)
}

fn fixture() -> KinematicModel {
    compile(
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/two_link_arm.hymeko"),
        "arm",
    )
}

/// A robot from `core`'s library, if this checkout has it beside us. `None` rather than a failure:
/// the sibling repository is not this crate's to require.
fn robot(file: &str, root: &str) -> Option<KinematicModel> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../core/data/robotics/robots")
        .join(file);
    path.exists().then(|| compile(path, root))
}

fn pose(model: &KinematicModel) -> (JointValues, Vec<f64>, FkPlan) {
    let plan = FkPlan::new(model).expect("the topology resolves");
    let mut named = JointValues::new();
    let mut flat = plan.zero_values();
    for (i, j) in model.joints.iter().enumerate() {
        let v = 0.4 * ((i as f64 * 0.9).sin() + 0.3 * (i as f64 * 2.1).cos());
        named.insert(j.name.clone(), v);
        flat[plan.slot(&j.name).expect("every joint has a slot")] = v;
    }
    assert_eq!(flat.len(), model.joints.len());
    (named, flat, plan)
}

// ─── the same answers ───────────────────────────────────────────────────────────────────────────

#[test]
fn the_plan_reproduces_the_shipped_solver_exactly() {
    // Bit-for-bit, not approximately. Both paths perform the same compositions in the same order,
    // so anything short of equality would mean the plan reordered the arithmetic — which is a
    // different function, however close its answers.
    let mut checked = 0;
    for model in [
        Some(fixture()),
        robot("arm_gripper.hymeko", "robot"),
        robot("humanoid.hymeko", "humanoid"),
        robot("quadruped.hymeko", "quadruped"),
    ]
    .into_iter()
    .flatten()
    {
        let (named, flat, plan) = pose(&model);
        let reference = forward_kinematics(&model, &named).expect("the shipped solver resolves");
        let mut poses = plan.identity_poses();
        plan.resolve(&model, &flat, &mut poses);

        for (i, l) in model.links.iter().enumerate() {
            let want = reference.get(&l.name).expect("the reference poses every link");
            assert_eq!(
                poses[i].translation, want.translation,
                "{}: translation differs",
                l.name
            );
            assert_eq!(
                poses[i].rotation_matrix(),
                want.rotation_matrix(),
                "{}: rotation differs",
                l.name
            );
        }
        assert_eq!(model.links[plan.root()].name, reference.root);
        checked += 1;
    }
    assert!(checked >= 1, "at least the local fixture must have been checked");
}

// ─── and no bookkeeping ─────────────────────────────────────────────────────────────────────────


// ─── the topology comes from the arena ──────────────────────────────────────────────────────────

#[test]
fn the_handles_and_the_names_name_the_same_links() {
    // `JointInfo` carries the topology twice: once as `DeclId` for the machine and once as `String`
    // for a person. Two spellings of one fact drift unless something checks them, and if they drift
    // the plan and the shipped solver would quietly disagree about the shape of the robot.
    let mut checked = 0;
    for model in [
        Some(fixture()),
        robot("arm_gripper.hymeko", "robot"),
        robot("quadruped.hymeko", "quadruped"),
    ]
    .into_iter()
    .flatten()
    {
        let by_handle: HashMap<_, _> = model.links.iter().map(|l| (l.did, l.name.as_str())).collect();
        let mut resolved = 0;
        for j in &model.joints {
            if let Some(name) = by_handle.get(&j.child) {
                assert_eq!(*name, j.child_link, "joint {}: child handle and name disagree", j.name);
                resolved += 1;
            }
            if let Some(name) = by_handle.get(&j.parent) {
                assert_eq!(*name, j.parent_link, "joint {}: parent handle and name disagree", j.name);
            }
        }
        assert_eq!(
            resolved,
            model.joints.len(),
            "every joint extracted from an arena must carry its child's handle, not only its name"
        );
        checked += 1;
    }
    assert!(checked >= 1);
}

#[test]
fn a_plan_built_without_handles_still_resolves() {
    // Hand-built fixtures have no arena behind them, so every handle is absent and the plan has to
    // fall back to matching by name. Removing that fallback would break every test in this crate
    // that builds a model literally, which is most of them.
    let mut model = fixture();
    for j in &mut model.joints {
        j.parent = hymeko::common::ids::DeclId::NONE;
        j.child = hymeko::common::ids::DeclId::NONE;
    }
    for l in &mut model.links {
        l.did = hymeko::common::ids::DeclId::NONE;
    }
    let (_, flat, plan) = pose(&model);
    let mut poses = plan.identity_poses();
    plan.resolve(&model, &flat, &mut poses);
    assert_eq!(poses.len(), model.links.len());
    assert!(poses.iter().all(|p| p.translation.len() == 3));
}

#[test]
fn a_plan_reports_the_same_structural_failures() {
    use hymeko_geometry::forward::FkError;
    let empty = KinematicModel {
        name: "empty".into(),
        links: Vec::new(),
        joints: Vec::new(),
        frames: Vec::new(),
    };
    assert!(matches!(FkPlan::new(&empty), Err(FkError::NoLinks)));

    // two links, nothing joining them: two roots, and the plan must say so rather than pick one
    let mut two = fixture();
    two.joints.clear();
    assert!(matches!(FkPlan::new(&two), Err(FkError::MultipleRoots(_))));
}

// ─── the stack-allocated walk ───────────────────────────────────────────────────────────────────

#[test]
fn the_se3_walk_is_bit_identical_to_the_heap_one() {
    // `SeFixed` mirrors `Se`'s arithmetic exactly — same bounds, same skip-if-zero guards, same
    // accumulation order — so this must be equality and not a tolerance. Anything less would mean
    // the fast path is a different function, and a caller holding a recorded pose would watch it
    // move for a reason no tolerance-based test could name.
    let mut checked = 0;
    for model in [
        Some(fixture()),
        robot("arm_gripper.hymeko", "robot"),
        robot("humanoid.hymeko", "humanoid"),
        robot("quadruped.hymeko", "quadruped"),
    ]
    .into_iter()
    .flatten()
    {
        let (_, flat, plan) = pose(&model);
        let mut heap = plan.identity_poses();
        let mut stack = plan.identity_poses_se3();
        plan.resolve(&model, &flat, &mut heap);
        plan.resolve_se3(&model, &flat, &mut stack);

        for (i, l) in model.links.iter().enumerate() {
            assert_eq!(
                stack[i].translation.as_slice(),
                heap[i].translation.as_slice(),
                "{}: translation differs",
                l.name
            );
            assert_eq!(
                stack[i].rotor.as_slice(),
                heap[i].rotor.components.as_slice(),
                "{}: rotor differs",
                l.name
            );
        }
        checked += 1;
    }
    assert!(checked >= 1);
}

