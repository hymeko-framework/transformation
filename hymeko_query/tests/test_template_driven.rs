//! End-to-end tests for the template-driven transform pipeline.
//!
//! Every format with a `template_dir()` override (urdf, sdf, mjcf, dot,
//! gazebo, mermaid) can be rendered through
//! [`TransformRegistry::render_from_templates`] without any Rust
//! `push_str` in the critical path: the output comes from
//! `transforms/<name>/template.*` rendered against queries in
//! `transforms/<name>/queries.hymeko` via
//! `hymeko_query::rewrite::template::execute_transform`.

#[cfg(test)]
mod test_template_driven {
    use std::path::PathBuf;

    use hymeko_query::transforms::{TransformConfig, TransformRegistry};

    use crate::test_helpers::load_and_lower;

    const MOVEO: &str = "../data/robotics/anthropomorphic_arm.hymeko";
    const MINI_ARM: &str = "../data/robotics/mini_arm.hymeko";

    /// `<workspace_root>/transforms/` — one level up from the crate
    /// manifest (`hymeko_query/`).
    fn transforms_root() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("hymeko_query has a parent (workspace root)")
            .join("transforms")
    }

    fn render(transform: &str, fixture: &str, name: &str) -> String {
        let (store, compiled) = load_and_lower(fixture).expect("compile");
        let reg = hymeko_formats::default_registry();
        let cfg = TransformConfig::default().with_name(name);
        reg.render_from_templates(transform, &compiled.ir, &store.it, &cfg, &transforms_root())
            .expect("transform registered")
            .expect("template render succeeded")
    }

    // ---- presence / surface ------------------------------------------------

    #[test]
    fn every_shipped_transform_exposes_a_template_dir() {
        let reg = hymeko_formats::default_registry();
        for name in ["urdf", "sdf", "mjcf", "dot", "gazebo", "mermaid"] {
            let t = reg.get(name).expect(name);
            assert_eq!(
                t.template_dir(),
                Some(name),
                "{name}.template_dir() should equal its name — that's the subdirectory lookup"
            );
        }
    }

    #[test]
    fn render_from_templates_returns_none_for_unknown_transform() {
        let reg = hymeko_formats::default_registry();
        let (store, compiled) = load_and_lower(MINI_ARM).unwrap();
        let out = reg.render_from_templates(
            "nonexistent",
            &compiled.ir,
            &store.it,
            &TransformConfig::default(),
            &transforms_root(),
        );
        assert!(out.is_none());
    }

    // ---- per-format renders -----------------------------------------------

    #[test]
    fn urdf_template_renders_robot_header() {
        let out = render("urdf", MOVEO, "moveo");
        assert!(out.contains("<robot name=\"moveo\""));
        assert!(out.contains("</robot>"));
    }

    #[test]
    fn urdf_template_renders_each_link() {
        let out = render("urdf", MOVEO, "moveo");
        for link in ["base_link", "link_0", "link_1", "tool"] {
            assert!(
                out.contains(&format!("<link name=\"{link}\"")),
                "URDF template output missing `<link name=\"{link}\"`:\n{out}"
            );
        }
    }

    #[test]
    fn sdf_template_wraps_model() {
        let out = render("sdf", MOVEO, "moveo");
        assert!(out.contains("<sdf version=\"1.7\">") || out.contains("<sdf version=\"1.8\">"));
        assert!(out.contains("<model name=\"moveo\""));
    }

    #[test]
    fn mjcf_template_wraps_mujoco_model() {
        let out = render("mjcf", MOVEO, "moveo");
        assert!(out.contains("<mujoco model=\"moveo\""));
        assert!(out.contains("<worldbody>"));
    }

    #[test]
    fn dot_template_opens_digraph_and_has_arrows() {
        let out = render("dot", MOVEO, "moveo");
        assert!(out.starts_with("digraph"));
        assert!(out.contains(" -> "));
        assert!(out.contains("moveo"));
    }

    #[test]
    fn mermaid_template_opens_flowchart() {
        let out = render("mermaid", MOVEO, "moveo");
        assert!(out.contains("flowchart TD"));
        assert!(out.contains("classDef link"));
        assert!(
            out.contains("base_link"),
            "Mermaid template should emit base_link node"
        );
    }

    #[test]
    fn mermaid_template_emits_dashed_arrow_for_fixed_joint() {
        let out = render("mermaid", MOVEO, "moveo");
        assert!(
            out.contains(" -.->"),
            "Mermaid template should emit dashed arrow for j_fix:\n{out}"
        );
    }

    #[test]
    fn gazebo_template_wraps_world_with_plugins() {
        let out = render("gazebo", MOVEO, "moveo");
        assert!(out.contains("<sdf version=\"1.8\">"));
        assert!(out.contains("<world name=\"empty\">"));
        // Standard plugin triple is part of the template skeleton.
        assert!(out.contains("gz-sim-physics-system"));
        assert!(out.contains("gz-sim-scene-broadcaster-system"));
        assert!(out.contains("<model name=\"ground_plane\""));
        // Post-2026-05-23 the world is a stage only; the robot model is
        // not inlined (URDF + launch-time `ros_gz_sim::create` own it).
        assert!(!out.contains("<model name=\"moveo\""));
    }

    #[test]
    fn gazebo_template_picks_up_sim_plugin_from_fixture() {
        // `@sim_control_plugin` in the fixture declares plugin +
        // filename + parameters. The template renders it via
        // `{{#each sim_plugins}}`.
        let out = render("gazebo", MOVEO, "moveo");
        assert!(
            out.contains("gz_ros2_control::GazeboSimROS2ControlPlugin"),
            "Gazebo template did not expand sim_plugin:\n{out}"
        );
        assert!(
            out.contains("gz_ros2_control-system"),
            "Gazebo template did not expand sim_plugin filename"
        );
    }

    // ---- determinism + hand-rolled parity -------------------------------

    #[test]
    fn render_from_templates_is_deterministic() {
        let a = render("mermaid", MOVEO, "moveo");
        let b = render("mermaid", MOVEO, "moveo");
        assert_eq!(a, b, "template rendering must be byte-stable");
    }

    #[test]
    fn mini_arm_renders_every_format_non_empty() {
        // The minimal 2-link / 1-joint fixture should produce non-empty
        // output from every registered template-backed transform.
        //
        // `gazebo` is excluded from the robot-name substring check: its
        // world.sdf is a stage only (physics + plugins + ground_plane),
        // not a robot description, so the robot name never appears there
        // (post-2026-05-23). It still has to produce non-empty output.
        for name in ["urdf", "sdf", "mjcf", "dot", "mermaid"] {
            let out = render(name, MINI_ARM, "mini_arm");
            assert!(!out.trim().is_empty(), "{name} produced empty output");
            assert!(out.contains("mini_arm"), "{name} output missing robot name");
        }
        let gazebo_out = render("gazebo", MINI_ARM, "mini_arm");
        assert!(!gazebo_out.trim().is_empty(), "gazebo produced empty output");
    }
}
