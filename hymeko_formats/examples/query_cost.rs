//! Where a robot's load time goes: compile, query, extract, plan.
use hymeko_geometry::forward::FkPlan;
use hymeko_geometry::kinematics::kinematic::extract_kinematic_model;
use hymeko_query::{Predicate, QueryEngine};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (path, root) = (&args[0], &args[1]);
    let p = std::path::Path::new(path);

    let reps = 20;
    let t0 = std::time::Instant::now();
    for _ in 0..reps {
        let mut store = hymeko::module_store::module_store::ModuleStore::new(
            hymeko::module_store::source_provider::StdFsProvider::new(),
            hymeko::util::real_parser::RealParser,
        );
        std::hint::black_box(store.compile(p).expect("compile"));
    }
    let compile_ms = t0.elapsed().as_secs_f64() * 1e3 / reps as f64;

    let mut store = hymeko::module_store::module_store::ModuleStore::new(
        hymeko::module_store::source_provider::StdFsProvider::new(),
        hymeko::util::real_parser::RealParser,
    );
    let program = store.compile(p).expect("compile");
    let engine = QueryEngine::new(&program.ir, &store.it);
    println!(
        "{root}: {} declarations in the arena",
        program.ir.decl_nodes.len()
    );

    let time = |name: &str, n: usize, f: &mut dyn FnMut()| {
        f();
        let t = std::time::Instant::now();
        for _ in 0..n {
            f();
        }
        let us = t.elapsed().as_secs_f64() * 1e6 / n as f64;
        println!("  {name:<40} {us:>9.2} us");
        us
    };

    println!(
        "  {:<40} {compile_ms:>9.2} ms",
        "compile (parse + resolve + lower)"
    );
    let q_link = time("query inherits(link)", 200, &mut || {
        std::hint::black_box(engine.query(&Predicate::node().and(Predicate::inherits("link"))));
    });
    let q_joint = time("query inherits(rev_joint)", 200, &mut || {
        std::hint::black_box(
            engine.query(&Predicate::edge().and(Predicate::inherits("rev_joint"))),
        );
    });
    let extract = time("extract_kinematic_model (6 queries)", 50, &mut || {
        std::hint::black_box(extract_kinematic_model(&engine, root));
    });
    let model = extract_kinematic_model(&engine, root);
    let plan = time("FkPlan::new", 200, &mut || {
        std::hint::black_box(FkPlan::new(&model).unwrap());
    });
    let flat = FkPlan::new(&model).unwrap().zero_values();
    let pl = FkPlan::new(&model).unwrap();
    let mut poses = pl.identity_poses_se3();
    let fk = time("FkPlan::resolve_se3", 20_000, &mut || {
        pl.resolve_se3(&model, &flat, &mut poses);
        std::hint::black_box(&poses);
    });
    let _ = (q_link, q_joint, plan);
    println!(
        "\n  one load = {:.2} ms compile + {extract:.0} us extract; then {fk:.2} us per pose",
        compile_ms
    );
}
