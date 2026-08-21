//! End-to-end workflow benchmark.
//!
//! Measures wall-clock time for the complete `.hymeko` -> target-format
//! workflow on the real robotics fixtures under `data/robotics/`. Each
//! fixture is run `REPEATS` times; per-stage timings are collected and
//! written as CSV to `<workspace>/target/benchmarks/workflow_benchmark.csv`.
//!
//! Stages measured:
//!   - compile:  parse + intern + resolve + lower + apply_usings
//!               (the full `ModuleStore::compile` pipeline)
//!   - urdf:     `hymeko_formats::urdf::generate_urdf`
//!   - sdf:      `hymeko_formats::sdf::generate_sdf`
//!   - gazebo:   `hymeko_formats::gazebo::generate_gazebo_world`
//!   - mjcf:     `TransformRegistry::emit("mjcf", ModelView::Kinematic(model))`
//!   - dot:      `TransformRegistry::emit("dot", ...)`
//!   - mermaid:  `TransformRegistry::emit("mermaid", ...)`
//!
//! Each stage logs the byte-size of its emitted artefact alongside the
//! wall-clock time, so per-format throughput (MiB/s) can be derived
//! post-hoc from the CSV.

#[cfg(test)]
mod bench_workflow {
    use std::fs;
    use std::path::PathBuf;
    use std::sync::OnceLock;
    use std::time::Instant;

    use log::info;

    static BENCH_LOGGER: OnceLock<()> = OnceLock::new();

    fn init_bench_logger() {
        BENCH_LOGGER.get_or_init(|| {
            let _ =
                env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
                    .is_test(true)
                    .try_init();
        });
    }

    use hymeko_formats::gazebo::generate_gazebo_world;
    use hymeko_formats::sdf::generate_sdf;
    use hymeko_formats::urdf::generate_urdf;
    use hymeko_query::engine::QueryEngine;
    use hymeko_query::kinematics::kinematic::extract_kinematic_model;
    use hymeko_query::transforms::{
        DomainTransform, ModelView, TransformConfig, TransformRegistry,
    };

    use crate::test_helpers::load_and_lower;

    const REPEATS: usize = 30;

    #[derive(Clone, Copy, Debug)]
    struct Fixture {
        label: &'static str,
        path: &'static str,
        robot_name: &'static str,
    }

    const FIXTURES: &[Fixture] = &[
        Fixture {
            label: "mini_arm",
            path: "../data/robotics/mini_arm.hymeko",
            robot_name: "mini_arm",
        },
        Fixture {
            label: "anthropomorphic_arm",
            path: "../data/robotics/anthropomorphic_arm.hymeko",
            robot_name: "moveo",
        },
        Fixture {
            label: "anthropomorphic_using",
            path: "../data/robotics/anthropomorphic_arm_using.hymeko",
            robot_name: "moveo",
        },
        Fixture {
            label: "robot_4wh",
            path: "../data/robotics/robot_4wh.hymeko",
            robot_name: "diff_robot",
        },
        Fixture {
            label: "robot_4wh_using",
            path: "../data/robotics/robot_4wh_using.hymeko",
            robot_name: "diff_robot",
        },
    ];

    #[derive(Debug)]
    struct WorkflowRow {
        fixture: &'static str,
        run_idx: usize,
        source_bytes: u64,
        compile_ms: f64,
        urdf_ms: f64,
        urdf_bytes: usize,
        sdf_ms: f64,
        sdf_bytes: usize,
        gazebo_ms: f64,
        gazebo_bytes: usize,
        mjcf_ms: f64,
        mjcf_bytes: usize,
        dot_ms: f64,
        dot_bytes: usize,
        mermaid_ms: f64,
        mermaid_bytes: usize,
        total_emit_ms: f64,
    }

    fn csv_out() -> PathBuf {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let ws = manifest.parent().expect("workspace root").to_path_buf();
        ws.join("target")
            .join("benchmarks")
            .join("workflow_benchmark.csv")
    }

    fn source_size(path: &str) -> u64 {
        fs::metadata(path).map(|m| m.len()).unwrap_or(0)
    }

    fn time_ms<F, T>(mut f: F) -> (T, f64)
    where
        F: FnMut() -> T,
    {
        let t = Instant::now();
        let out = f();
        (out, t.elapsed().as_secs_f64() * 1_000.0)
    }

    fn run_fixture(fx: Fixture, reg: &TransformRegistry) -> Vec<WorkflowRow> {
        let mut rows = Vec::with_capacity(REPEATS);
        let src_bytes = source_size(fx.path);

        for run_idx in 1..=REPEATS {
            // ---- compile ---------------------------------------------------
            let ((store, compiled), compile_ms) =
                time_ms(|| load_and_lower(fx.path).expect("fixture should compile"));

            // ---- prepare model view for registry-driven transforms --------
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, fx.robot_name);
            let cfg = TransformConfig::default().with_name(fx.robot_name);
            let view = ModelView::Kinematic(model);

            // ---- URDF -----------------------------------------------------
            let (urdf, urdf_ms) = time_ms(|| generate_urdf(&compiled.ir, &store.it, fx.robot_name));
            let urdf_bytes = urdf.len();

            // ---- SDF ------------------------------------------------------
            let (sdf, sdf_ms) = time_ms(|| generate_sdf(&compiled.ir, &store.it, fx.robot_name));
            let sdf_bytes = sdf.len();

            // ---- Gazebo world --------------------------------------------
            let (gazebo, gazebo_ms) =
                time_ms(|| generate_gazebo_world(&compiled.ir, &store.it, fx.robot_name, "empty"));
            let gazebo_bytes = gazebo.len();

            // ---- MJCF / DOT / Mermaid via registry -----------------------
            let (mjcf_out, mjcf_ms) = time_ms(|| reg.get("mjcf").and_then(|t| t.emit(&view, &cfg)));
            let mjcf_bytes = mjcf_out.as_deref().map(|s| s.len()).unwrap_or(0);

            let (dot_out, dot_ms) = time_ms(|| reg.get("dot").and_then(|t| t.emit(&view, &cfg)));
            let dot_bytes = dot_out.as_deref().map(|s| s.len()).unwrap_or(0);

            let (merm_out, merm_ms) =
                time_ms(|| reg.get("mermaid").and_then(|t| t.emit(&view, &cfg)));
            let merm_bytes = merm_out.as_deref().map(|s| s.len()).unwrap_or(0);

            let total_emit = urdf_ms + sdf_ms + gazebo_ms + mjcf_ms + dot_ms + merm_ms;

            info!(
                "[workflow] {} run {}/{}: compile={:.2}ms, urdf={:.2}ms({}B), sdf={:.2}ms({}B), gazebo={:.2}ms({}B), mjcf={:.2}ms({}B), dot={:.2}ms({}B), mermaid={:.2}ms({}B), total_emit={:.2}ms",
                fx.label,
                run_idx,
                REPEATS,
                compile_ms,
                urdf_ms,
                urdf_bytes,
                sdf_ms,
                sdf_bytes,
                gazebo_ms,
                gazebo_bytes,
                mjcf_ms,
                mjcf_bytes,
                dot_ms,
                dot_bytes,
                merm_ms,
                merm_bytes,
                total_emit,
            );

            rows.push(WorkflowRow {
                fixture: fx.label,
                run_idx,
                source_bytes: src_bytes,
                compile_ms,
                urdf_ms,
                urdf_bytes,
                sdf_ms,
                sdf_bytes,
                gazebo_ms,
                gazebo_bytes,
                mjcf_ms,
                mjcf_bytes,
                dot_ms,
                dot_bytes,
                mermaid_ms: merm_ms,
                mermaid_bytes: merm_bytes,
                total_emit_ms: total_emit,
            });
        }

        rows
    }

    fn write_csv(rows: &[WorkflowRow]) {
        let path = csv_out();
        if let Some(p) = path.parent() {
            let _ = fs::create_dir_all(p);
        }
        let header = "fixture,run_idx,source_bytes,compile_ms,urdf_ms,urdf_bytes,sdf_ms,sdf_bytes,gazebo_ms,gazebo_bytes,mjcf_ms,mjcf_bytes,dot_ms,dot_bytes,mermaid_ms,mermaid_bytes,total_emit_ms";
        let body = rows
            .iter()
            .map(|r| {
                format!(
                    "{},{},{},{:.4},{:.4},{},{:.4},{},{:.4},{},{:.4},{},{:.4},{},{:.4},{},{:.4}",
                    r.fixture,
                    r.run_idx,
                    r.source_bytes,
                    r.compile_ms,
                    r.urdf_ms,
                    r.urdf_bytes,
                    r.sdf_ms,
                    r.sdf_bytes,
                    r.gazebo_ms,
                    r.gazebo_bytes,
                    r.mjcf_ms,
                    r.mjcf_bytes,
                    r.dot_ms,
                    r.dot_bytes,
                    r.mermaid_ms,
                    r.mermaid_bytes,
                    r.total_emit_ms,
                )
            })
            .collect::<Vec<_>>()
            .join("\n");
        let payload = format!("{}\n{}\n", header, body);
        fs::write(&path, payload).expect("write workflow CSV");
        info!(
            "[workflow] wrote CSV: {} (rows={})",
            path.display(),
            rows.len()
        );
    }

    #[test]
    fn bench_end_to_end_workflow() {
        init_bench_logger();
        let reg = hymeko_formats::default_registry();
        let mut all = Vec::new();
        for fx in FIXTURES {
            let mut rows = run_fixture(*fx, &reg);
            all.append(&mut rows);
        }
        write_csv(&all);
        assert!(!all.is_empty());
        assert!(
            all.iter().all(|r| r.urdf_bytes > 0),
            "every fixture must emit URDF"
        );
    }
}
