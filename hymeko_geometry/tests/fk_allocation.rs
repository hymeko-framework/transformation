//! Allocation, measured — in its own binary, and serialised.
//!
//! A counting global allocator sees EVERY thread. Rust runs the tests inside one binary in parallel,
//! so a counter switched on for one of them also records what the others are doing: the first
//! version of this lived beside the correctness tests and reported 48, then 743, then 822, then 984
//! allocations for the same allocation-free walk. Nothing was wrong with the walk.
//!
//! So the measurement lives here, alone, with the two tests serialised against each other. What is
//! being counted then really is what the closure did.

use std::alloc::{GlobalAlloc, Layout, System};
use std::path::PathBuf;
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

use hymeko::module_store::module_store::ModuleStore;
use hymeko::module_store::source_provider::StdFsProvider;
use hymeko::util::real_parser::RealParser;
use hymeko_geometry::forward::{FkPlan, JointValues, forward_kinematics};
use hymeko_geometry::kinematics::kinematic::{KinematicModel, extract_kinematic_model};
use hymeko_query::QueryEngine;

/// Only one measurement at a time, so no other test in this binary is allocating meanwhile.
static SERIAL: Mutex<()> = Mutex::new(());

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

/// Run `f` and report how many allocations its steady state makes.
fn allocations_during(mut f: impl FnMut()) -> usize {
    for _ in 0..4 {
        f(); // settle anything lazy, so the count is the steady state and not a first call
    }
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

#[test]
fn resolving_a_pose_through_the_plan_allocates_nothing_for_bookkeeping() {
    let _serial = SERIAL.lock().expect("the measurement lock");
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

#[test]
fn the_se3_walk_allocates_nothing_at_all() {
    let _serial = SERIAL.lock().expect("the measurement lock");
    // Not "less". Nothing. A rigid motion is now eight coefficients and three coordinates on the
    // stack, the pose buffer belongs to the caller, and the topology was resolved when the plan was
    // built — so there is nothing left for a pose to allocate, and the number is zero rather than a
    // budget somebody has to justify.
    let model = robot("quadruped.hymeko", "quadruped").unwrap_or_else(fixture);
    let (_, flat, plan) = pose(&model);
    let mut poses = plan.identity_poses_se3();
    let allocations = allocations_during(|| plan.resolve_se3(&model, &flat, &mut poses));
    assert_eq!(
        allocations,
        0,
        "a stack-allocated pose walk over {} links allocated {allocations} times",
        model.links.len()
    );
}
