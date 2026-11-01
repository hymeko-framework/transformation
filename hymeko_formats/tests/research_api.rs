//! Public callers exercise the same domain APIs as the research binaries.
use hymeko::module_store::{
    execution::CompiledModel,
    source_provider::{MemProvider, StdFsProvider},
};
use hymeko::util::real_parser::RealParser;
use hymeko_formats::{
    OutputFormat, default_registry,
    environment::{
        CompiledWorld, Layout, Point,
        maze::{Profile, Topology},
    },
    generate_description, metaheuristic,
    robot_projection::RobotProjection,
};
use hymeko_geometry::transforms::{TransformConfig, TransformEngine};
use std::path::{Path, PathBuf};

fn robot() -> CompiledModel {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../data/robotics/sim/dual_fanuc/fanuc_lrmate200id.hymeko");
    CompiledModel::compile(StdFsProvider::new(), RealParser, &path).unwrap()
}

const FIDELITY_SOURCE: &str = r#"
        fidelity { @"meta_kinematics.hymeko"; }
        robot: meta_kinematics.kinematics.elements {
            world: meta_kinematics.kinematics.elements.frame {}
            base: meta_kinematics.kinematics.elements.link { mass 2; }
            tip: meta_kinematics.kinematics.elements.link {
                mass 1;
                origin [0.1, 0.2, 0.3];
                link_geometry: meta_kinematics.kinematics.geometry.box { dimension [0.1, 0.2, 0.3]; }
            }
            diagonal: meta_kinematics.kinematics.axes.axis_definition { ax [0.6, 0.8, 0.0]; }
            @anchor: meta_kinematics.kinematics.fixed_joint {
                (+ world [[1, 2, 3], [0, 0, 0]], - base);
            }
            @hinge: meta_kinematics.kinematics.conti_joint {
                (+ base [[0.2, 0, 0.4], [0, 0, 90]], - tip, - diagonal);
            }
        }
    "#;

fn fidelity_model_from_source(source: &str) -> CompiledModel {
    let mut files = MemProvider::default();
    files.insert_file(
        "/meta_kinematics.hymeko",
        include_str!("../../data/robotics/meta_kinematics.hymeko"),
    );
    files.insert_file("/model.hymeko", source);
    CompiledModel::compile(files, RealParser, Path::new("/model.hymeko")).unwrap()
}

fn fidelity_model() -> CompiledModel {
    fidelity_model_from_source(FIDELITY_SOURCE)
}

fn emitted_model_views(model: &CompiledModel) -> [String; 3] {
    [OutputFormat::Urdf, OutputFormat::Sdf17, OutputFormat::Mjcf].map(|format| {
        generate_description(&model.program().ir, model.interner(), "robot", format).unwrap()
    })
}

#[test]
fn model_emitters_preserve_oblique_axis_components() {
    let model = fidelity_model();
    let urdf = hymeko_formats::urdf::generate_urdf(&model.program().ir, model.interner(), "robot");
    let sdf = hymeko_formats::sdf::generate_sdf(&model.program().ir, model.interner(), "robot");
    assert!(urdf.contains("<axis xyz=\"0.6 0.8 0\"/>"), "{urdf}");
    assert!(sdf.contains("<xyz>0.6 0.8 0</xyz>"), "{sdf}");
}

#[test]
fn mjcf_declares_the_urdf_rpy_rotation_convention() {
    let source = FIDELITY_SOURCE.replace("[0, 0, 90]", "[30, 40, 50]");
    assert_ne!(source, FIDELITY_SOURCE);
    let model = fidelity_model_from_source(&source);
    let xml = generate_description(
        &model.program().ir,
        model.interner(),
        "robot",
        OutputFormat::Mjcf,
    )
    .unwrap();
    assert!(xml.contains("euler=\"0.5236 0.6981 0.8727\""), "{xml}");
    assert!(xml.contains("eulerseq=\"XYZ\""), "{xml}");
}

#[test]
fn sdf_retains_link_joint_and_geometry_frames() {
    let model = fidelity_model();
    let sdf = hymeko_formats::sdf::generate_sdf(&model.program().ir, model.interner(), "robot");
    assert!(
        sdf.contains("<link name=\"tip\">\n      <pose relative_to=\"hinge\">0 0 0 0 0 0</pose>"),
        "{sdf}"
    );
    assert!(
        sdf.contains("<pose relative_to=\"__model__\">1 2 3 0.0000 0.0000 0.0000</pose>"),
        "{sdf}"
    );
    assert!(
        sdf.contains("<pose relative_to=\"base\">0.2 0 0.4 0.0000 0.0000 1.5708</pose>"),
        "{sdf}"
    );
    for tag in ["visual", "collision"] {
        assert!(
            sdf.contains(&format!(
                "<{tag} name=\"tip_{tag}\">\n        <pose>0.1 0.2 0.3 0 0 0</pose>"
            )),
            "{sdf}"
        );
    }
}

#[test]
fn all_kinematic_codegen_routes_use_the_same_model_emitter() {
    let model = fidelity_model();
    let query = hymeko_query::QueryEngine::new(&model.program().ir, model.interner());
    let config = TransformConfig::default().with_name("robot");
    let engine = TransformEngine {
        registry: default_registry(),
        ..Default::default()
    };
    for (name, format) in [
        ("urdf", OutputFormat::Urdf),
        ("sdf", OutputFormat::Sdf17),
        ("mjcf", OutputFormat::Mjcf),
    ] {
        let typed = engine
            .emit_with_config(&model.program().ir, model.interner(), &config, name)
            .unwrap();
        let dispatched =
            generate_description(&model.program().ir, model.interner(), "robot", format).unwrap();
        assert_eq!(typed, dispatched, "{name}");
        assert_eq!(
            Some(Ok(typed)),
            engine
                .registry
                .render(name, &query, &config, Path::new("absent")),
            "{name}"
        );
        assert!(
            engine
                .registry
                .render_from_templates(
                    name,
                    &model.program().ir,
                    model.interner(),
                    &config,
                    Path::new("absent")
                )
                .is_none()
        );
    }
}

#[test]
fn declared_parent_frames_are_accepted_but_unknown_parents_are_rejected() {
    use hymeko_geometry::transforms::ModelView;
    let document = fidelity_model();
    let projection = RobotProjection::new(&document, "robot").unwrap();
    let registry = default_registry();
    for name in ["urdf", "sdf"] {
        let transform = registry.get(name).unwrap();
        let mut model = projection.model().clone();
        assert!(
            transform
                .validate(&ModelView::Kinematic(model.clone()))
                .is_empty()
        );
        model.frames.clear();
        assert!(
            transform
                .validate(&ModelView::Kinematic(model))
                .iter()
                .any(|d| d.is_error())
        );
    }
}

#[test]
fn checked_value_update_refreshes_all_views_and_preserves_the_old_snapshot() {
    use hymeko::common::ids::DeclId;
    use hymeko::ir::ir::ValueR;
    use hymeko::module_store::execution::{ExecutionState, PublicationError, ValueUpdate};
    let state = ExecutionState::new(fidelity_model());
    let old = state.snapshot().unwrap();
    let before = emitted_model_views(old.model());
    let ir_before = serde_json::to_vec(&old.model().program().ir).unwrap();
    let query = hymeko_query::QueryEngine::new(&old.model().program().ir, old.model().interner());
    let tip = old
        .model()
        .program()
        .ir
        .decls()
        .find(|(_, d)| old.model().interner().resolve(d.name) == "tip")
        .unwrap()
        .0;
    let mass = query.own_child(tip, "mass").unwrap();
    assert!(
        old.model()
            .with_value_updates(&[ValueUpdate::Clear {
                declaration: DeclId::NONE
            }])
            .is_err()
    );
    assert_eq!(state.snapshot().unwrap().revision(), 0);
    let updated = old
        .model()
        .with_value_updates(&[ValueUpdate::Set {
            declaration: mass,
            value: ValueR::Num(3.0),
        }])
        .unwrap();
    assert_eq!(
        old.model().program().canon.structure(),
        updated.program().canon.structure()
    );
    assert_ne!(old.model().semantic_digest(), updated.semantic_digest());
    let published = state
        .commit(state.prepare_model_at(&old, updated).unwrap())
        .unwrap();
    assert_eq!(published.revision(), 1);
    let after = emitted_model_views(published.model());
    for (i, marker) in [
        "<mass value=\"3\"/>",
        "<mass>3</mass>",
        "<inertial mass=\"3\"",
    ]
    .into_iter()
    .enumerate()
    {
        assert_ne!(before[i], after[i]);
        assert!(after[i].contains(marker), "{}", after[i]);
    }
    let stale = old.model().with_value_updates(&[]).unwrap();
    assert!(matches!(
        state.prepare_model_at(&old, stale),
        Err(PublicationError::StaleBase { .. })
    ));
    assert_eq!(before, emitted_model_views(old.model()));
    assert_eq!(
        ir_before,
        serde_json::to_vec(&old.model().program().ir).unwrap()
    );
}

#[test]
fn structural_recompile_publishes_new_views_without_replacing_retained_readers() {
    use hymeko::module_store::execution::ExecutionState;
    let state = ExecutionState::new(fidelity_model());
    let old = state.snapshot().unwrap();
    let before = emitted_model_views(old.model());
    let extended = fidelity_model_from_source(&format!(
        r#"{FIDELITY_SOURCE}
        extra_link: meta_kinematics.kinematics.elements.link {{ mass 0.5; }}
        @extension: meta_kinematics.kinematics.fixed_joint {{ (+ robot.tip [[0,0,0.1],[0,0,0]], - extra_link); }}
    "#
    ));
    assert_ne!(
        old.model().program().canon.structure(),
        extended.program().canon.structure()
    );
    let published = state
        .commit(state.prepare_model_at(&old, extended).unwrap())
        .unwrap();
    for xml in emitted_model_views(published.model()) {
        assert!(xml.contains("name=\"extra_link\""), "{xml}");
    }
    assert_eq!(
        RobotProjection::new(published.model(), "robot")
            .unwrap()
            .model()
            .links
            .len(),
        3
    );
    assert_eq!(before, emitted_model_views(old.model()));
    assert!(before.iter().all(|xml| !xml.contains("extra_link")));
}

#[test]
fn public_codegen_and_robot_projection_preserve_articulation() {
    let document = robot();
    let projection = RobotProjection::new(&document, "robot").unwrap();
    let config = TransformConfig::default().with_name("robot");
    let registry = default_registry();
    let xml = generate_description(
        &document.program().ir,
        document.interner(),
        "robot",
        OutputFormat::Mjcf,
    )
    .unwrap();
    assert_eq!(xml, projection.mjcf().unwrap());
    let engine = TransformEngine {
        registry,
        ..Default::default()
    };
    assert_eq!(
        xml,
        engine
            .emit_with_config(&document.program().ir, document.interner(), &config, "mjcf")
            .unwrap()
    );
    assert_eq!(xml.matches("type=\"hinge\"").count(), 6);
    assert!(xml.contains("<actuator>"));
    assert_eq!(projection.model().links.len(), 7);
    assert_eq!(projection.metadata()["schema"], "hymeko.robot-projection/1");
    let poses = projection
        .forward_poses(&projection.diagnostic_pose())
        .unwrap();
    assert_eq!(poses["poses"].as_array().unwrap().len(), 7);
    let empty = CompiledModel::compile(
        MemProvider::default().with_file("/empty.hymeko", "empty {}"),
        RealParser,
        Path::new("/empty.hymeko"),
    )
    .unwrap();
    assert!(RobotProjection::new(&empty, "empty").is_err());
    assert!(
        engine
            .registry
            .render_from_templates(
                "mjcf",
                &document.program().ir,
                document.interner(),
                &config,
                Path::new("unused")
            )
            .is_none()
    );
}

#[test]
fn template_errors_do_not_fall_back_to_a_different_backend() {
    let document = robot();
    let registry = default_registry();
    let query = hymeko_query::QueryEngine::new(&document.program().ir, document.interner());
    let config = TransformConfig::default().with_name("robot");
    let absent = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("__missing_templates__");
    assert!(
        registry
            .render("dot", &query, &config, &absent)
            .unwrap()
            .is_err()
    );
    assert!(
        registry
            .render("unknown", &query, &config, &absent)
            .is_none()
    );
    assert!(
        registry
            .render("mjcf", &query, &config, &absent)
            .unwrap()
            .is_ok()
    );
}

#[test]
fn public_grid_projection_rejects_illegal_moves_before_search() {
    let layout = Layout {
        width: 2,
        height: 2,
        start: Point { x: 0, y: 0 },
        goal: Point { x: 1, y: 1 },
        walls: vec![Point { x: 1, y: 0 }],
    };
    let source = layout.source().unwrap();
    let (world, _) = CompiledWorld::canonical(&source).unwrap();
    let projection = world.project().unwrap();
    assert_eq!(
        projection.evaluate(false).unwrap().shortest_path_length,
        Some(2)
    );
    let bad = format!("{source}\n@illegal {{ (~ cell_0_0, ~ cell_1_0); }}");
    assert!(CompiledWorld::compile(&bad).unwrap().project().is_err());
}

#[test]
fn public_maze_priority_projection_keeps_fixed_encoding_and_bounds() {
    for (profile, edges, cells) in [(Profile::Hex, 42, 19), (Profile::Polar, 18, 12)] {
        let topology = Topology::new(profile);
        assert_eq!(topology.priority_pairs().len(), edges);
        let genes: Vec<_> = (0..edges).map(|i| i as f64 / edges as f64).collect();
        let (world, source, geometry, checks) = topology.generate_priorities(0, &genes).unwrap();
        assert_eq!(geometry.cells().len(), cells);
        assert_eq!(geometry.passages().len(), cells - 1);
        assert!(checks.iter().all(|check| check["passed"] == true));
        assert_eq!(
            world.digest(),
            CompiledWorld::compile(&source).unwrap().digest()
        );
        assert!(
            topology
                .generate_priorities(0, &vec![f64::NAN; edges])
                .is_err()
        );
    }
}

#[test]
fn public_profile_validation_preserves_frozen_identities() {
    for (source, digest, operators) in [
        (
            include_str!("../data/metaheuristics/de_rand_1_bin.hymeko"),
            "dde322cf659e60d60a3f3a20297988ac4727e8aa1d7b262561b9ce577f9b7770",
            10,
        ),
        (
            include_str!("../data/metaheuristics/ga_tournament_uniform_reset.hymeko"),
            "5b8f53e30a60314bebd4f15f1c506445f035b7442e7fae8c717f0a96cf748868",
            8,
        ),
    ] {
        let model = metaheuristic::compile(source).unwrap();
        let contract = metaheuristic::validate(&model).unwrap();
        assert_eq!(contract["model_semantic_digest"], digest);
        assert_eq!(contract["operators"].as_object().unwrap().len(), operators);
    }
}
