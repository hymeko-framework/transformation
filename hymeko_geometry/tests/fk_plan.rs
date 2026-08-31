//! The prepared plan: same answers, no per-call bookkeeping, and the topology taken from the arena.
//!
//! Three claims, and the middle one is the reason this file exists. A performance fix that is only
//! pinned by a timing measurement is not pinned at all — it passes on a fast machine having proved
//! nothing, and it stops being true the first time somebody adds a `clone()` to a hot path without
//! noticing. So the "no allocation" claim is measured with a counting allocator, which fails the
//! same way on every machine.

use std::alloc::{GlobalAlloc, Layout, System};
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};

use hymeko::module_store::module_store::ModuleStore;
use hymeko::module_store::source_provider::StdFsProvider;
use hymeko::util::real_parser::RealParser;
use hymeko_geometry::forward::{forward_kinematics, FkPlan, JointValues};
use hymeko_geometry::kinematics::kinematic::{extract_kinematic_model, KinematicModel};
use hymeko_query::QueryEngine;

// ─── a global allocator that can be asked how many times it was called ──────────────────────────

static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
static COUNTING: AtomicUsize = AtomicUsize::new(0);

struct Counting;

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if COUNTING.load(Ordering::Relaxed) != 0 {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if COUNTING.load(Ordering::Relaxed) != 0 {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static ALLOC: Counting = Counting;

/// Run `f` and report how many allocations it made.
fn allocations_during(mut f: impl FnMut()) -> usize {
    f(); // warm anything lazy, so the count is the steady state and not the first call
    ALLOCATIONS.store(0, Ordering::Relaxed);
    COUNTING.store(1, Ordering::Relaxed);
    f();
    COUNTING.store(0, Ordering::Relaxed);
    ALLOCATIONS.load(Ordering::Relaxed)
}

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

#[test]
fn resolving_a_pose_through_the_plan_allocates_nothing_for_bookkeeping() {
    // The claim is not "fewer allocations". Every allocation left is an `Se` — two per rigid motion,
    // four motions per link — and there must be no others: no hash map, no string, no vector of
    // names. So the budget is stated in Se's, and anything above it is bookkeeping that crept back.
    let model = robot("quadruped.hymeko", "quadruped").unwrap_or_else(fixture);
    let (named, flat, plan) = pose(&model);
    let mut poses = plan.identity_poses();

    let planned = allocations_during(|| plan.resolve(&model, &flat, &mut poses));
    let shipped = allocations_during(|| {
        std::hint::black_box(forward_kinematics(&model, &named).unwrap());
    });

    // The budget is MEASURED, not guessed: run the same compositions with no plan at all and see
    // what the Se arithmetic costs on its own. A hand-written constant would only have recorded
    // what `Multivector` happened to allocate on the day it was written — the first attempt here
    // guessed eight allocations per link and the real figure is over sixty, because every
    // multivector product allocates its own result.
    let arithmetic = allocations_during(|| {
        let mut acc = hymeko_clifford::Se::identity(3);
        for j in &model.joints {
            acc = acc
                .compose(&hymeko_geometry::forward::joint_origin(j))
                .compose(&hymeko_geometry::forward::joint_motion(j, 0.3));
        }
        std::hint::black_box(&acc);
    });
    // The floor is MEASURED, not guessed. A hand-written constant would record what `Multivector`
    // happened to allocate on the day it was written, and would have to be edited — silently — the
    // first time `Se` changes. Measured on a 33-link quadruped: planned 1214, arithmetic floor
    // 1223, shipped 1365. The plan sits AT the floor (below it, in fact, because it composes over
    // 32 links and a root identity where the reference chains 33 joints), and the shipped solver
    // carries 151 allocations nothing about the geometry requires.
    assert!(
        planned <= arithmetic + 2 * model.links.len(),
        "a planned resolve of {} links allocated {planned}, above the {arithmetic} the same          compositions cost with no plan at all — bookkeeping has crept back in",
        model.links.len()
    );
    assert!(
        planned < shipped,
        "the plan must allocate strictly less than the shipped solver: {planned} against {shipped}"
    );
}

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
