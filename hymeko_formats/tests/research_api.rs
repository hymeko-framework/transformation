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
            .render("urdf", &query, &config, &absent)
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
