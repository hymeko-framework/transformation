//! Tests for the `MermaidTransform` — renders a kinematic chain as a
//! Mermaid `flowchart TD` that GitHub, docs sites, and the VS Code
//! Mermaid preview can display without a Graphviz toolchain.

#[cfg(test)]
mod test_mermaid {
    use hymeko_query::engine::QueryEngine;
    use hymeko_query::kinematics::kinematic::extract_kinematic_model;
    use hymeko_query::transforms::{
        DomainTransform, ModelView, TransformConfig, TransformRegistry,
    };

    use crate::test_helpers::load_and_lower;

    const MOVEO: &str = "../data/robotics/anthropomorphic_arm.hymeko";
    const DIFF_ROBOT: &str = "../data/robotics/robot_4wh.hymeko";
    const MINI_ARM: &str = "../data/robotics/mini_arm.hymeko";

    fn emit(path: &str, robot_name: &str) -> String {
        let (store, compiled) = load_and_lower(path).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let model = extract_kinematic_model(&engine, robot_name);
        let reg = hymeko_formats::default_registry();
        let t = reg.get("mermaid").expect("mermaid transform registered");
        let cfg = TransformConfig::default().with_name(robot_name);
        t.emit(&ModelView::Kinematic(model), &cfg)
            .expect("mermaid emit succeeds")
    }

    #[test]
    fn mermaid_registered_with_mmd_extension() {
        let reg = hymeko_formats::default_registry();
        assert!(reg.available().contains(&"mermaid"));
        assert_eq!(reg.get("mermaid").unwrap().extension(), "mmd");
    }

    #[test]
    fn mermaid_output_opens_with_flowchart_directive() {
        let out = emit(MOVEO, "moveo");
        assert!(
            out.contains("flowchart TD"),
            "expected `flowchart TD` directive in Mermaid output:\n{out}"
        );
    }

    #[test]
    fn mermaid_declares_classdef_for_links_and_roots() {
        let out = emit(MOVEO, "moveo");
        assert!(out.contains("classDef link"));
        assert!(out.contains("classDef root"));
    }

    #[test]
    fn mermaid_emits_one_node_per_link_with_mass_label() {
        let out = emit(MOVEO, "moveo");
        for name in ["base_link", "link_0", "link_1", "link_4", "tool"] {
            assert!(
                out.contains(&format!("{name}[")),
                "Mermaid output missing `{name}[` node declaration"
            );
        }
        // Base link mass is 25.0 kg in the fixture.
        assert!(
            out.contains("25.00 kg"),
            "Mermaid output missing base_link mass label"
        );
    }

    #[test]
    fn mermaid_emits_world_as_a_root_frame_for_moveo() {
        // world is declared as a `frame` in anthropomorphic_arm.hymeko,
        // not a `link`, so the extractor won't list it in model.links.
        // Our emitter recovers it via `find_roots`.
        let out = emit(MOVEO, "moveo");
        assert!(
            out.contains("world(["),
            "Mermaid output should declare `world` as a root-style node:\n{out}"
        );
    }

    #[test]
    fn mermaid_emits_dashed_arrow_for_fixed_joint() {
        let out = emit(MOVEO, "moveo");
        // j_fix is the world→base_link fixed joint.
        assert!(
            out.contains("world -.->"),
            "expected dashed arrow for fixed joint:\n{out}"
        );
    }

    #[test]
    fn mermaid_emits_solid_arrow_for_revolute_joint() {
        let out = emit(MOVEO, "moveo");
        assert!(
            out.contains("base_link -->"),
            "expected solid arrow for revolute joint:\n{out}"
        );
    }

    #[test]
    fn mermaid_emits_one_arrow_per_joint() {
        let out = emit(MOVEO, "moveo");
        let revolute = out.matches(" -->|\"").count();
        let fixed = out.matches(" -.->|\"").count();
        // moveo has 6 revolute + 1 fixed = 7 joints.
        assert_eq!(
            revolute + fixed,
            7,
            "expected 7 total arrows, got {}+{}",
            revolute,
            fixed
        );
        assert_eq!(fixed, 1);
        assert_eq!(revolute, 6);
    }

    #[test]
    fn mermaid_axis_letter_appears_in_joint_label() {
        let out = emit(MOVEO, "moveo");
        // j0 is about Z, j1 is about X per the fixture's anthropomorphic signature.
        assert!(
            out.contains("j0 (rev, Z)"),
            "expected `j0 (rev, Z)` label:\n{out}"
        );
        assert!(out.contains("j1 (rev, X)"));
    }

    #[test]
    fn mermaid_emit_is_deterministic() {
        let a = emit(MOVEO, "moveo");
        let b = emit(MOVEO, "moveo");
        assert_eq!(a, b, "Mermaid output must be byte-stable");
    }

    #[test]
    fn mermaid_diff_robot_emits_continuous_label() {
        // robot_4wh has wheel joints declared as `conti_joint` (continuous).
        let out = emit(DIFF_ROBOT, "diff_robot");
        assert!(
            out.contains("(cont,") || out.contains("(rev,"),
            "expected continuous- or revolute-labelled arrows on diff_robot:\n{out}"
        );
    }

    #[test]
    fn mermaid_mini_arm_has_single_continuous_joint_arrow() {
        let out = emit(MINI_ARM, "mini_arm");
        // mini_arm: base_link -> spin_joint (continuous) -> spinner.
        let arrows = out.matches(" -->|\"").count();
        let dashed = out.matches(" -.->|\"").count();
        assert_eq!(arrows + dashed, 1);
    }
}
