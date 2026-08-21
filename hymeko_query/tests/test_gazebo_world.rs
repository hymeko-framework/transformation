//! Paper 2 / T11 — Gazebo world transform tests.
//!
//! Exercises `hymeko_formats::gazebo::generate_gazebo_world` and
//! the `GazeboWorldTransform` registry entry, plus the
//! `extract_gazebo_plugins` helper in
//! `hymeko_query::kinematics::gazebo_plugins`. The fixtures
//! `anthropomorphic_arm.hymeko` and `robot_4wh.hymeko` both declare
//! `sim_plugin` + `control_plugin` hyperedges with concrete `plugin` /
//! `filename` / `parameters` children, so each should produce a world
//! file with fully-populated `<plugin>` tags.

#[cfg(test)]
mod test_gazebo_world {
    use hymeko_formats::gazebo::generate_gazebo_world;
    use hymeko_query::engine::QueryEngine;
    use hymeko_query::kinematics::gazebo_plugins::{GazeboPluginKind, extract_gazebo_plugins};
    use hymeko_query::kinematics::kinematic::extract_kinematic_model;
    use hymeko_query::transforms::{
        DomainTransform, ModelView, TransformConfig, TransformRegistry,
    };

    use crate::test_helpers::load_and_lower;

    const MOVEO: &str = "../data/robotics/anthropomorphic_arm.hymeko";
    const DIFF_ROBOT: &str = "../data/robotics/robot_4wh.hymeko";

    // ---- plugin extractor ---------------------------------------------------

    #[test]
    fn plugin_extractor_finds_sim_and_control_plugins_on_moveo() {
        let (store, compiled) = load_and_lower(MOVEO).expect("moveo should compile");
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let plugins = extract_gazebo_plugins(&engine);

        let sim_count = plugins
            .iter()
            .filter(|p| p.kind == GazeboPluginKind::Sim)
            .count();
        let ctrl_count = plugins
            .iter()
            .filter(|p| p.kind == GazeboPluginKind::Control)
            .count();

        assert!(sim_count >= 1, "expected ≥1 sim_plugin, got {sim_count}");
        assert!(
            ctrl_count >= 1,
            "expected ≥1 control_plugin, got {ctrl_count}"
        );
    }

    #[test]
    fn plugin_extractor_populates_plugin_class_and_filename() {
        let (store, compiled) = load_and_lower(MOVEO).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let plugins = extract_gazebo_plugins(&engine);

        // `@sim_control_plugin` declares plugin + filename (+ parameters).
        let sim = plugins
            .iter()
            .find(|p| p.edge_name == "sim_control_plugin")
            .expect("sim_control_plugin edge missing");
        assert_eq!(sim.kind, GazeboPluginKind::Sim);
        assert_eq!(
            sim.plugin_class.as_deref(),
            Some("gz_ros2_control::GazeboSimROS2ControlPlugin")
        );
        assert_eq!(sim.filename.as_deref(), Some("gz_ros2_control-system"));
        assert!(sim.is_complete());

        // `@gazebo_sim_system` declares plugin only — filename may be None.
        let ctrl = plugins
            .iter()
            .find(|p| p.edge_name == "gazebo_sim_system")
            .expect("gazebo_sim_system edge missing");
        assert_eq!(ctrl.kind, GazeboPluginKind::Control);
        assert_eq!(
            ctrl.plugin_class.as_deref(),
            Some("gz_ros2_control/GazeboSimSystem")
        );
    }

    #[test]
    fn plugin_extractor_works_on_diff_robot_too() {
        let (store, compiled) = load_and_lower(DIFF_ROBOT).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let plugins = extract_gazebo_plugins(&engine);
        assert!(
            plugins.iter().any(|p| p.kind == GazeboPluginKind::Sim),
            "diff_robot should declare ≥1 sim_plugin"
        );
        assert!(
            plugins.iter().any(|p| p.kind == GazeboPluginKind::Control),
            "diff_robot should declare ≥1 control_plugin"
        );
    }

    // ---- full world emitter -------------------------------------------------

    fn moveo_world() -> String {
        let (store, compiled) = load_and_lower(MOVEO).unwrap();
        generate_gazebo_world(&compiled.ir, &store.it, "moveo", "empty")
    }

    #[test]
    fn gazebo_world_is_sdf_1_8() {
        let w = moveo_world();
        assert!(w.starts_with("<?xml"));
        assert!(w.contains("<sdf version=\"1.8\">"));
        assert!(w.trim_end().ends_with("</sdf>"));
    }

    #[test]
    fn gazebo_world_wraps_named_world_tag() {
        let w = moveo_world();
        assert!(w.contains("<world name=\"empty\">"));
        assert!(w.contains("</world>"));
    }

    #[test]
    fn gazebo_world_includes_standard_physics_plugin_triple() {
        let w = moveo_world();
        for plugin in [
            "gz-sim-physics-system",
            "gz-sim-user-commands-system",
            "gz-sim-scene-broadcaster-system",
        ] {
            assert!(w.contains(plugin), "missing standard plugin `{plugin}`");
        }
    }

    #[test]
    fn gazebo_world_includes_ground_plane() {
        let w = moveo_world();
        assert!(w.contains("<model name=\"ground_plane\">"));
        assert!(w.contains("<static>true</static>"));
        assert!(w.contains("<plane>"));
    }

    #[test]
    fn gazebo_world_does_not_inline_robot_model() {
        // Regression: the world.sdf emitter must emit a world-stage only
        // (physics, gz-sim plugin triple, ground_plane, light, extracted
        // sim_plugin/control_plugin entries at world scope). The robot
        // itself is the URDF's responsibility and is spawned at launch
        // time via `ros_gz_sim::create`; inlining it here would (a)
        // duplicate the kinematic definition between URDF and world.sdf
        // with drift risk, and (b) collide on name with the runtime
        // spawn. Prior to 2026-05-23 the template embedded `<model
        // name="{robot_name}">` with full link/joint/inertia geometry;
        // this test would have failed against that revision.
        let w = moveo_world();
        assert!(
            !w.contains("<model name=\"moveo\">"),
            "world.sdf must not inline `<model name=\"moveo\">` — the robot \
             is spawned from URDF at launch time, not embedded in the world"
        );
        // The world's only `<model>` should be the ground_plane.
        assert_eq!(
            w.matches("<model name=").count(),
            1,
            "expected exactly one `<model name=` (ground_plane); inline \
             robot model has resurfaced"
        );
    }

    #[test]
    fn gazebo_world_injects_extracted_sim_plugin() {
        let w = moveo_world();
        assert!(
            w.contains("gz_ros2_control::GazeboSimROS2ControlPlugin"),
            "sim_plugin class missing from world output:\n{w}"
        );
        assert!(
            w.contains("gz_ros2_control-system"),
            "sim_plugin filename missing"
        );
    }

    #[test]
    fn gazebo_world_injects_extracted_control_plugin() {
        let w = moveo_world();
        assert!(
            w.contains("gz_ros2_control/GazeboSimSystem"),
            "control_plugin class missing from world output:\n{w}"
        );
    }

    #[test]
    fn gazebo_world_is_deterministic_across_runs() {
        let a = moveo_world();
        let b = moveo_world();
        assert_eq!(a, b, "Gazebo world output must be byte-stable");
    }

    #[test]
    fn gazebo_world_for_diff_robot_generates_non_empty_content() {
        let (store, compiled) = load_and_lower(DIFF_ROBOT).unwrap();
        let w = generate_gazebo_world(&compiled.ir, &store.it, "diff_robot", "factory");
        assert!(w.contains("<world name=\"factory\">"));
        // robot_4wh declares its sim_plugin `@sim_control_plugin`; confirms
        // the fixture's plugin hyperedges flow through the emitter even
        // though the robot model itself lives in URDF (post-2026-05-23 the
        // robot name no longer appears in world.sdf — the world is a stage
        // only, the robot is spawned at launch).
        assert!(w.contains("gz_ros2_control-system"));
    }

    // ---- registry integration -----------------------------------------------

    #[test]
    fn registry_exposes_gazebo_transform() {
        let reg = hymeko_formats::default_registry();
        let names = reg.available();
        assert!(names.contains(&"gazebo"));
        assert_eq!(reg.get("gazebo").unwrap().extension(), "world.sdf");
    }

    #[test]
    fn registry_emit_returns_stub_with_physics_plugins() {
        // The registry stub can't reach `sim_plugin` edges (ModelView
        // doesn't carry the raw IR), but it must still emit a launchable
        // world skeleton.
        let (store, compiled) = load_and_lower(MOVEO).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let model = extract_kinematic_model(&engine, "moveo");
        let reg = hymeko_formats::default_registry();
        let t = reg.get("gazebo").unwrap();
        let out = t
            .emit(
                &ModelView::Kinematic(model),
                &TransformConfig::default().with_name("moveo"),
            )
            .expect("stub emit succeeds");
        assert!(out.contains("<sdf version=\"1.8\">"));
        assert!(out.contains("gz-sim-physics-system"));
        // Stub is explicit about the gap.
        assert!(
            out.contains("TODO: delegate to formats::gazebo"),
            "stub should flag its incomplete state"
        );
    }

    #[test]
    fn registry_gazebo_validator_passes_for_serial_chain() {
        let (store, compiled) = load_and_lower(MOVEO).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let model = extract_kinematic_model(&engine, "moveo");
        let reg = hymeko_formats::default_registry();
        let t = reg.get("gazebo").unwrap();
        let diags = t.validate(&ModelView::Kinematic(model));
        let errors: Vec<_> = diags.iter().filter(|d| d.is_error()).collect();
        assert!(errors.is_empty(), "validator: {errors:?}");
    }
}
