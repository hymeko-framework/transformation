//! End-to-end workflow benchmark.
//!
//! Measures wall-clock time for the complete `.hymeko` -> target-format
//! workflow on the real robotics fixtures under `data/robotics/`. Each
//! fixture is run `REPEATS` times; per-stage timings are collected and
//! written as CSV to `<workspace>/target/benchmarks/workflow_benchmark.csv`.
//!
//! Stages measured:
//!   - compile:  parse + intern + module-local aliases + resolve + lower
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
    use hymeko_mechanism::kinematics::kinematic::extract_kinematic_model;
    use hymeko_formats::plugin::{
        DomainTransform, ModelView, TransformConfig, TransformRegistry,
    };
    use hymeko_query::engine::QueryEngine;

    use crate::test_helpers::load_and_lower;

    const REPEATS: usize = 30;

    #[derive(Clone, Copy, Debug)]
    struct Fixture<'a> {
        label: &'a str,
        path: &'a str,
        robot_name: &'a str,
    }

    const FIXTURES: &[Fixture<'static>] = &[
        Fixture {
            label: "mini_arm",
            path: "../data/robotics/robots/mini_arm.hymeko",
            robot_name: "mini_arm",
        },
        Fixture {
            label: "anthropomorphic_arm",
            path: "../data/robotics/robots/anthropomorphic_arm.hymeko",
            robot_name: "moveo",
        },
        Fixture {
            label: "anthropomorphic_using",
            path: "../data/robotics/robots/anthropomorphic_arm_using.hymeko",
            robot_name: "moveo",
        },
        Fixture {
            label: "robot_4wh",
            path: "../data/robotics/robots/robot_4wh.hymeko",
            robot_name: "diff_robot",
        },
        Fixture {
            label: "robot_4wh_using",
            path: "../data/robotics/robots/robot_4wh_using.hymeko",
            robot_name: "diff_robot",
        },
    ];

    /// Relative input paths are resolved beside the TSV, independent of cwd.
    struct WorkflowSuite {
        cases: Vec<(String, String, String)>,
    }

    impl WorkflowSuite {
        fn parse(text: &str, directory: &std::path::Path) -> Result<Self, String> {
            let mut lines = text.lines();
            if text.len() > 64 * 1024 || lines.next() != Some("label\tpath\trobot_name") {
                return Err("expected a bounded label/path/robot_name TSV manifest".into());
            }
            let mut cases = Vec::new();
            let mut labels = std::collections::BTreeSet::new();
            for line in lines.filter(|line| !line.trim().is_empty()) {
                let fields: Vec<_> = line.split('\t').collect();
                if fields.len() != 3 || fields.iter().any(|field| field.is_empty()) {
                    return Err("three nonempty fixture fields required".into());
                }
                if [fields[0], fields[2]].iter().any(|name| {
                    name.as_bytes()[0].is_ascii_digit()
                        || !name.bytes().all(|c| c.is_ascii_alphanumeric() || c == b'_')
                }) || !labels.insert(fields[0]) {
                    return Err("fixture labels must be unique; labels and roots must be identifiers".into());
                }
                let path = directory.join(fields[1]);
                if !path.is_file() || cases.len() >= 64 {
                    return Err("missing input file or too many fixtures".into());
                }
                cases.push((fields[0].to_owned(), path.to_str().ok_or("UTF-8 path required")?.to_owned(), fields[2].to_owned()));
            }
            if cases.is_empty() { return Err("empty fixture corpus".into()); }
            Ok(Self { cases })
        }

        fn selected() -> Result<Self, String> {
            let mut suite = if let Some(path) = std::env::var_os("HYMEKO_WORKFLOW_MANIFEST") {
                let path = fs::canonicalize(PathBuf::from(path)).map_err(|e| e.to_string())?;
                if fs::metadata(&path).map_err(|e| e.to_string())?.len() > 64 * 1024 {
                    return Err("manifest exceeds 64 KiB".into());
                }
                Self::parse(&fs::read_to_string(&path).map_err(|e| e.to_string())?, path.parent().ok_or("manifest parent")?)?
            } else {
                Self { cases: FIXTURES.iter().map(|f| (f.label.into(), f.path.into(), f.robot_name.into())).collect() }
            };
            if let Ok(label) = std::env::var("HYMEKO_WORKFLOW_CASE") {
                suite.cases.retain(|case| case.0 == label);
                if suite.cases.len() != 1 { return Err("selected fixture absent".into()); }
            }
            Ok(suite)
        }

        fn fixtures(&self) -> impl Iterator<Item = Fixture<'_>> {
            self.cases.iter().map(|(label, path, robot_name)| Fixture { label, path, robot_name })
        }
    }

    #[test]
    fn workflow_manifest_is_bounded_and_resolves_real_input_paths() {
        let directory = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let text = "label\tpath\trobot_name\narm\t../data/robotics/robots/mini_arm.hymeko\tmini_arm\n";
        let suite = WorkflowSuite::parse(text, &directory).unwrap();
        let fixture = suite.fixtures().next().unwrap();
        assert_eq!(fixture.label, "arm");
        assert_eq!(fixture.robot_name, "mini_arm");
        assert!(std::path::Path::new(fixture.path).is_file());
        for bad in ["", "label\tpath\trobot_name\n", "bad header", "label\tpath\trobot_name\na\tmissing\tr\n"] {
            assert!(WorkflowSuite::parse(bad, &directory).is_err());
        }
        for bad in [text.replace("arm\t", "bad,label\t"), text.replace("\tmini_arm\n", "\t\n"),
                    text.to_owned() + "arm\t../data/robotics/robots/mini_arm.hymeko\tmini_arm\n"] {
            assert!(WorkflowSuite::parse(&bad, &directory).is_err());
        }
        assert!(WorkflowSuite::parse(&"x".repeat(65537), &directory).is_err());
        let many = "label\tpath\trobot_name\n".to_owned() + &(0..65)
            .map(|n| format!("arm_{n}\t../data/robotics/robots/mini_arm.hymeko\tmini_arm\n"))
            .collect::<String>();
        assert!(WorkflowSuite::parse(&many, &directory).is_err());
    }

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

    fn run_fixture(fx: Fixture<'static>, reg: &TransformRegistry) -> Vec<WorkflowRow> {
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
    #[ignore = "explicit HYMEKO_WORKFLOW_BENCH=1; historical per-stage measurement"]
    fn bench_end_to_end_workflow() {
        assert_eq!(std::env::var("HYMEKO_WORKFLOW_BENCH").as_deref(), Ok("1"));
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

    const MATCHED_FORMATS: &[&str] = &["urdf", "sdf", "gazebo", "mjcf", "dot", "mermaid"];

    #[derive(Clone, Copy)]
    enum CompilationPolicy {
        Shared,
        PerFormat,
    }

    struct EmissionBundle {
        artifacts: Vec<(&'static str, String)>,
        compile_requests: usize,
        declarations: usize,
        links: usize,
        joints: usize,
    }
    impl EmissionBundle {
        fn bytes(&self) -> usize {
            self.artifacts.iter().map(|(_, text)| text.len()).sum()
        }
    }

    impl Fixture<'_> {
        /// One production ModuleStore compilation and one shared domain view.
        /// Both policies call this exact same registry-backed implementation.
        fn emit_subset(
            &self,
            registry: &TransformRegistry,
            formats: &[&'static str],
        ) -> EmissionBundle {
            assert!(!formats.is_empty());
            let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(self.path);
            let (store, compiled) = load_and_lower(path).expect("matched fixture must compile");
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, self.robot_name);
            assert!(!model.links.is_empty(), "selected robot root has no links");
            let (links, joints) = (model.links.len(), model.joints.len());
            let view = ModelView::Kinematic(model);
            let config = TransformConfig::default().with_name(self.robot_name);
            let artifacts = formats
                .iter()
                .map(|&name| {
                    let text = registry
                        .get(name)
                        .expect("registered format")
                        .emit(&view, &config)
                        .expect("fixture accepted by the registered emitter");
                    assert!(!text.is_empty(), "empty {name} output");
                    (name, text)
                })
                .collect();
            EmissionBundle {
                artifacts,
                compile_requests: 1,
                declarations: compiled.ir.decl_nodes.len(),
                links,
                joints,
            }
        }

        fn emit_matched(
            &self,
            registry: &TransformRegistry,
            policy: CompilationPolicy,
        ) -> EmissionBundle {
            match policy {
                CompilationPolicy::Shared => self.emit_subset(registry, MATCHED_FORMATS),
                CompilationPolicy::PerFormat => {
                    let mut result = EmissionBundle {
                        artifacts: Vec::with_capacity(MATCHED_FORMATS.len()),
                        compile_requests: 0,
                        declarations: 0,
                        links: 0,
                        joints: 0,
                    };
                    for &name in MATCHED_FORMATS {
                        let next = self.emit_subset(registry, &[name]);
                        result.compile_requests += next.compile_requests;
                        result.declarations = next.declarations;
                        result.links = next.links;
                        result.joints = next.joints;
                        result.artifacts.extend(next.artifacts);
                    }
                    result
                }
            }
        }

        fn verify_matched(&self, registry: &TransformRegistry) -> EmissionBundle {
            let shared = self.emit_matched(registry, CompilationPolicy::Shared);
            let repeated = self.emit_matched(registry, CompilationPolicy::PerFormat);
            assert_eq!(shared.compile_requests, 1);
            assert_eq!(repeated.compile_requests, MATCHED_FORMATS.len());
            assert_eq!(shared.artifacts.len(), MATCHED_FORMATS.len());
            assert_eq!(
                shared.artifacts, repeated.artifacts,
                "complete artifact mismatch for {}",
                self.label
            );
            shared
        }
    }

    #[test]
    fn shared_compile_matches_per_format_byte_for_byte() {
        let registry = hymeko_formats::default_registry();
        let suite = WorkflowSuite::selected().expect("valid selected workflow corpus");
        for fixture in suite.fixtures() {
            let first = fixture.verify_matched(&registry);
            let repeated = fixture.emit_matched(&registry, CompilationPolicy::Shared);
            assert_eq!(
                first.artifacts, repeated.artifacts,
                "repeat determinism: {}",
                fixture.label
            );
            assert!(first.bytes() > 0);
            println!("WORKFLOW_CHECK fixture={} declarations={} links={} joints={} bytes={} formats={} byte_equivalent=true",
                fixture.label, first.declarations, first.links, first.joints, first.bytes(), MATCHED_FORMATS.len());
        }
    }

    /// Paired control of compilation/domain-view reuse, not a hypergraph-vs-graph
    /// representation ablation. File reads use the warm OS cache; registry setup
    /// and exact output comparisons stay outside the measured complete calls.
    #[test]
    #[ignore = "explicit HYMEKO_WORKFLOW_BENCH=1; matched complete-call control"]
    fn bench_matched_compilation() {
        use hymeko::util::measurement::MeasurementRunner;
        use std::io::Write;

        assert_eq!(std::env::var("HYMEKO_WORKFLOW_BENCH").as_deref(), Ok("1"));
        let destination =
            PathBuf::from(std::env::var_os("HYMEKO_WORKFLOW_CSV").expect("fresh output CSV path"));
        assert!(
            !destination.exists(),
            "refuse to overwrite an existing measurement"
        );
        let run: usize = std::env::var("HYMEKO_WORKFLOW_RUN")
            .expect("process run index")
            .parse()
            .unwrap();
        assert!(run > 0);
        let registry = hymeko_formats::default_registry();
        let mut csv = String::from(
            "fixture,run,pair,order,shared_us,per_format_us,bytes,formats,shared_compile_requests,per_format_compile_requests,ir_declarations,links,joints\n",
        );
        let suite = WorkflowSuite::selected().expect("valid selected workflow corpus");
        for fixture in suite.fixtures() {
            let reference = fixture.verify_matched(&registry);
            let bytes = reference.bytes();
            let operation = |policy| {
                let output = fixture.emit_matched(&registry, policy);
                assert_eq!(output.bytes(), bytes);
                std::hint::black_box(&output.artifacts);
                drop(output); // Complete artifact and temporary destruction is timed.
                bytes
            };
            let mut runner = MeasurementRunner::new(3, 30);
            let (shared, repeated) = runner.compare(
                || operation(CompilationPolicy::Shared),
                || operation(CompilationPolicy::PerFormat),
            );
            assert_eq!(
                fixture.verify_matched(&registry).artifacts,
                reference.artifacts
            );
            for (ordinal, (a, b)) in runner
                .samples()
                .iter()
                .zip(runner.comparison_samples())
                .enumerate()
            {
                csv.push_str(&format!(
                    "{},{run},{ordinal},{},{a:.9},{b:.9},{bytes},{},1,{},{},{},{}\n",
                    fixture.label,
                    if ordinal % 2 == 0 { "AB" } else { "BA" },
                    MATCHED_FORMATS.len(),
                    MATCHED_FORMATS.len(), reference.declarations, reference.links, reference.joints
                ));
            }
            println!(
                "WORKFLOW_CONTROL fixture={} run={run} unit=us scope=warm_file_load_compile_prepare_emit_six_outputs_and_drop shared_median={} repeated_median={} byte_equivalent=true samples=30 warmups=3 quantiles=upper_middle paired_order=alternating_AB_BA",
                fixture.label, shared.median, repeated.median
            );
        }
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(destination)
            .unwrap();
        file.write_all(csv.as_bytes()).unwrap();
    }

    /// One policy in one fresh child for an external process-memory observer.
    /// Full-output equivalence is checked separately, never inside this child.
    #[test]
    #[ignore = "explicit HYMEKO_WORKFLOW_BENCH=1; isolated process-memory workload"]
    fn bench_matched_memory() {
        assert_eq!(std::env::var("HYMEKO_WORKFLOW_BENCH").as_deref(), Ok("1"));
        let suite = WorkflowSuite::selected().expect("valid selected workflow corpus");
        assert_eq!(suite.cases.len(), 1, "memory children require exactly one fixture");
        let policy = std::env::var("HYMEKO_WORKFLOW_POLICY").expect("shared, per_format or baseline");
        if policy == "baseline" { println!("WORKFLOW_MEMORY baseline=true"); return; }
        let policy = match policy.as_str() {
            "shared" => CompilationPolicy::Shared,
            "per_format" => CompilationPolicy::PerFormat,
            _ => panic!("unknown memory policy"),
        };
        let fixture = suite.fixtures().next().unwrap();
        let registry = hymeko_formats::default_registry();
        let output = fixture.emit_matched(&registry, policy);
        println!("WORKFLOW_MEMORY fixture={} bytes={} formats={} declarations={} links={} joints={}",
            fixture.label, output.bytes(), output.artifacts.len(), output.declarations, output.links, output.joints);
        std::hint::black_box(&output.artifacts);
        drop(output);
    }
}
