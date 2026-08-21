#[cfg(test)]
mod test_transform_ecosystem {
    use hymeko_query::engine::QueryEngine;
    use hymeko_query::kinematics::kinematic::*;
    use hymeko_query::transforms::ModelView;

    const MOVEO_ARM: &str = "../data/robotics/anthropomorphic_arm.hymeko";
    const MOVEO_ARM_USING: &str = "../data/robotics/anthropomorphic_arm_using.hymeko";
    const DIFF_ROBOT: &str = "../data/robotics/robot_4wh.hymeko";
    const DIFF_ROBOT_USING: &str = "../data/robotics/robot_4wh_using.hymeko";

    // ============================================================
    // Registry tests
    // ============================================================

    mod registry {
        use super::*;
        use crate::test_helpers::load_and_lower;
        use hymeko_query::transforms::{TransformConfig, TransformRegistry};

        #[test]
        fn default_registry_has_all_formats() {
            let reg = hymeko_formats::default_registry();
            let available = reg.available();
            assert!(available.contains(&"urdf"), "Missing URDF");
            assert!(available.contains(&"sdf"), "Missing SDF");
            assert!(available.contains(&"mjcf"), "Missing MJCF");
            assert!(available.contains(&"dot"), "Missing DOT");
        }

        #[test]
        fn lookup_by_name() {
            let reg = hymeko_formats::default_registry();
            assert!(reg.get("urdf").is_some());
            assert!(reg.get("nonexistent").is_none());
        }

        #[test]
        fn lookup_by_extension() {
            let reg = hymeko_formats::default_registry();
            assert_eq!(reg.by_extension("urdf").unwrap().name(), "urdf");
            assert_eq!(reg.by_extension("sdf").unwrap().name(), "sdf");
            assert_eq!(reg.by_extension("dot").unwrap().name(), "dot");
        }

        #[test]
        fn generate_all_formats() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let reg = hymeko_formats::default_registry();
            let config = TransformConfig::default().with_name("moveo");

            let model = hymeko_query::transforms::extract(
                &compiled.ir,
                &store.it,
                &config.robot_name,
                hymeko_query::transforms::ModelKind::Kinematic,
            );
            let results = reg.emit_all(&model, &config);

            // Default registrations producing output for a Kinematic model:
            // urdf, sdf, mjcf, dot, gazebo, mermaid, sysml.
            // (torch_dataflow is registered but accepts a non-Kinematic model
            // kind so it doesn't appear in this kinematic-only emit_all set.)
            // Gazebo joined with Paper 2 T11 and Mermaid with the docs-friendly
            // diagram path on 2026-04-19; SysML 2 added on 2026-05-06.
            assert_eq!(results.len(), 7, "Should generate 7 formats");
            for (filename, content) in &results {
                assert!(!content.is_empty(), "Empty output for {}", filename);
                println!("Generated {} ({} bytes)", filename, content.len());
            }
        }

        #[test]
        fn generate_all_for_both_robots() {
            let reg = hymeko_formats::default_registry();

            for (path, name) in &[(MOVEO_ARM, "moveo"), (DIFF_ROBOT, "diff_robot")] {
                let (store, compiled) = load_and_lower(path).unwrap();
                let config = TransformConfig::default().with_name(name);
                let model = hymeko_query::transforms::extract(
                    &compiled.ir,
                    &store.it,
                    &config.robot_name,
                    hymeko_query::transforms::ModelKind::Kinematic,
                );
                let results = reg.emit_all(&model, &config);

                for (filename, content) in &results {
                    assert!(
                        !content.is_empty(),
                        "{}: empty output for {}",
                        name,
                        filename
                    );
                }
            }
        }
    }

    // ============================================================
    // Validation tests
    // ============================================================

    mod validation {
        use super::*;
        use crate::test_helpers::load_and_lower;
        use hymeko_formats::MjcfTransform;
        use hymeko_query::transforms::{DomainTransform, ModelView, TransformRegistry};

        #[test]
        fn moveo_validates_clean_for_all_formats() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");

            let reg = hymeko_formats::default_registry();
            for name in reg.available() {
                let transform = reg.get(name).unwrap();
                let diags = transform.validate(&ModelView::Kinematic(model.clone()));
                let errors: Vec<_> = diags
                    .iter()
                    .filter(|d| d.is_error())
                    .filter(|d| !d.message.contains("world"))
                    .collect();
                assert!(
                    errors.is_empty(),
                    "{} validation failed: {:?}",
                    name,
                    errors.iter().map(|e| &e.message).collect::<Vec<_>>()
                );
            }
        }

        #[test]
        fn mjcf_validates_tree_topology() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");

            let mjcf = MjcfTransform;
            let diags = mjcf.validate(&ModelView::Kinematic(model.clone()));
            let errors: Vec<_> = diags.iter().filter(|d| d.is_error()).collect();

            // Moveo is a serial chain — should pass tree check
            assert!(
                errors.is_empty(),
                "MJCF tree validation failed: {:?}",
                errors.iter().map(|e| &e.message).collect::<Vec<_>>()
            );
        }
    }

    // ============================================================
    // MJCF generation tests
    // ============================================================

    mod mjcf {
        use super::*;
        use crate::test_helpers::load_and_lower;
        use hymeko_formats::MjcfTransform;
        use hymeko_query::transforms::{DomainTransform, ModelView, TransformConfig};

        #[test]
        fn moveo_mjcf_wellformed() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");
            let config = TransformConfig::default().with_name("moveo");

            let mjcf = MjcfTransform;
            let output = mjcf.emit(&ModelView::Kinematic(model), &config).unwrap();
            println!("--- Moveo MJCF ---\n{}", output);

            assert!(output.contains("<mujoco model=\"moveo\""));
            assert!(output.ends_with("</mujoco>\n"));
        }

        #[test]
        fn moveo_mjcf_has_bodies() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");
            let config = TransformConfig::default().with_name("moveo");

            let mjcf = MjcfTransform;
            let output = mjcf.emit(&ModelView::Kinematic(model), &config).unwrap();

            // MJCF uses <body> elements for links
            for name in &["base_link", "link_0", "link_1", "link_4", "tool"] {
                assert!(
                    output.contains(&format!("<body name=\"{}\"", name)),
                    "MJCF missing body '{}'",
                    name
                );
            }
        }

        #[test]
        fn moveo_mjcf_has_joints() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");
            let config = TransformConfig::default().with_name("moveo");

            let mjcf = MjcfTransform;
            let output = mjcf.emit(&ModelView::Kinematic(model), &config).unwrap();

            // MJCF uses <joint> inside <body>
            let hinge_count = output.matches("type=\"hinge\"").count();
            assert_eq!(
                hinge_count, 6,
                "Expected 6 hinge joints, got {}",
                hinge_count
            );
        }

        #[test]
        fn moveo_mjcf_has_actuators() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");
            let config = TransformConfig::default().with_name("moveo");

            let mjcf = MjcfTransform;
            let output = mjcf.emit(&ModelView::Kinematic(model), &config).unwrap();

            assert!(output.contains("<actuator>"));
            let motor_count = output.matches("<motor").count();
            assert_eq!(
                motor_count, 6,
                "Expected 6 actuators (one per revolute joint), got {}",
                motor_count
            );
        }

        #[test]
        fn moveo_mjcf_half_extents_for_box() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");
            let config = TransformConfig::default().with_name("moveo");

            let mjcf = MjcfTransform;
            let output = mjcf.emit(&ModelView::Kinematic(model), &config).unwrap();

            // tool has box [0.075, 0.15, 0.1]
            // MJCF uses half-extents: [0.0375, 0.075, 0.05]
            assert!(
                output.contains("0.0375") || output.contains("0.075"),
                "MJCF should use half-extents for box geometry"
            );
        }

        #[test]
        fn moveo_mjcf_has_materials() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");
            let config = TransformConfig::default().with_name("moveo");

            let mjcf = MjcfTransform;
            let output = mjcf.emit(&ModelView::Kinematic(model), &config).unwrap();

            assert!(output.contains("<asset>"));
            assert!(output.contains("<material name="));
        }

        #[test]
        fn moveo_mjcf_radians_not_degrees() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");
            let config = TransformConfig::default().with_name("moveo");

            let mjcf = MjcfTransform;
            let output = mjcf.emit(&ModelView::Kinematic(model), &config).unwrap();

            assert!(
                output.contains("angle=\"radian\""),
                "MJCF compiler should specify radian mode"
            );
        }

        #[test]
        fn moveo_mjcf_nested_body_hierarchy() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");
            let config = TransformConfig::default().with_name("moveo");

            let mjcf = MjcfTransform;
            let output = mjcf.emit(&ModelView::Kinematic(model), &config).unwrap();

            // MJCF bodies must be nested (not flat like URDF)
            // Count indentation levels to verify nesting
            let lines: Vec<&str> = output.lines().collect();
            let max_indent = lines
                .iter()
                .filter(|l| l.contains("<body"))
                .map(|l| l.len() - l.trim_start().len())
                .max()
                .unwrap_or(0);

            // With 7 links in a serial chain, deepest nesting should be significant
            assert!(
                max_indent >= 12,
                "MJCF bodies should be deeply nested for serial chain, max indent = {}",
                max_indent
            );
        }
    }

    // ============================================================
    // DOT generation tests
    // ============================================================

    mod dot {
        use super::*;
        use crate::test_helpers::load_and_lower;
        use hymeko_formats::DotTransform;
        use hymeko_query::transforms::{DomainTransform, ModelView, TransformConfig};

        #[test]
        fn moveo_dot_wellformed() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");
            let config = TransformConfig::default().with_name("moveo");

            let dot = DotTransform;
            let output = dot.emit(&ModelView::Kinematic(model), &config).unwrap();
            println!("--- Moveo DOT ---\n{}", output);

            assert!(output.starts_with("digraph"));
            assert!(output.ends_with("}\n"));
        }

        #[test]
        fn moveo_dot_has_all_links_as_nodes() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");
            let config = TransformConfig::default().with_name("moveo");

            let dot = DotTransform;
            let output = dot.emit(&ModelView::Kinematic(model), &config).unwrap();

            for name in &["base_link", "link_0", "link_4", "tool"] {
                assert!(
                    output.contains(&format!("\"{}\"", name)),
                    "DOT missing node for '{}'",
                    name
                );
            }
        }

        #[test]
        fn moveo_dot_has_edges_with_joint_names() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");
            let config = TransformConfig::default().with_name("moveo");

            let dot = DotTransform;
            let output = dot.emit(&ModelView::Kinematic(model), &config).unwrap();

            // Check directed edges exist
            assert!(output.contains("\"base_link\" -> \"link_0\""));
            assert!(output.contains("\"link_4\" -> \"tool\""));
        }

        #[test]
        fn moveo_dot_fixed_joints_dashed() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");
            let config = TransformConfig::default().with_name("moveo");

            let dot = DotTransform;
            let output = dot.emit(&ModelView::Kinematic(model), &config).unwrap();

            // Fixed joints should be dashed
            assert!(
                output.contains("style=dashed"),
                "Fixed joints should have dashed style in DOT"
            );
        }

        #[test]
        fn moveo_dot_revolute_joints_bold() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");
            let config = TransformConfig::default().with_name("moveo");

            let dot = DotTransform;
            let output = dot.emit(&ModelView::Kinematic(model), &config).unwrap();

            let bold_count = output.matches("style=bold").count();
            assert_eq!(
                bold_count, 6,
                "Expected 6 bold edges (revolute), got {}",
                bold_count
            );
        }

        #[test]
        fn moveo_dot_axis_labels() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");
            let config = TransformConfig::default().with_name("moveo");

            let dot = DotTransform;
            let output = dot.emit(&ModelView::Kinematic(model), &config).unwrap();

            // Joint labels should include axis letter. The canonical anthropomorphic arm uses
            // only local X and Z tokens (world Y comes from j1's 90° twist), so DOT shows (X)/(Z)
            // and no (Y).
            assert!(output.contains("(Z)"), "DOT should show Z axis joints");
            assert!(output.contains("(X)"), "DOT should show X axis joints");
            assert!(!output.contains("(Y)"), "corrected arm has no local Y-axis joint label");
        }

        #[test]
        fn diff_robot_dot_generates() {
            let (store, compiled) = load_and_lower(DIFF_ROBOT).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "diff_robot");
            let config = TransformConfig::default().with_name("diff_robot");

            let dot = DotTransform;
            let output = dot.emit(&ModelView::Kinematic(model), &config).unwrap();
            println!("--- Diff Robot DOT ---\n{}", output);

            assert!(output.contains("\"base_link\""));
            assert!(output.contains("\"wheel_fr\""));
        }
    }

    // ============================================================
    // Alias-parity tests
    //
    // Fixtures ending in `_using.hymeko` exercise the `using <path> as <alias>;`
    // namespace-alias syntax (grammar in parser/src/hymeko.lalrpop, lowered via
    // ModuleStore::compile() → apply_usings()). These tests guarantee that the
    // aliased sources produce the same kinematic model and the same transform
    // output as their non-alias counterparts — i.e. alias resolution is a pure
    // desugaring step with no semantic drift.
    //
    // Coverage promised in changelog_20260407.md; first landed 2026-04-18 as
    // part of the namespace-alias audit follow-up.
    // ============================================================

    mod alias_parity {
        use super::*;
        use crate::test_helpers::load_and_lower;
        use hymeko_formats::DotTransform;
        use hymeko_query::transforms::{
            DomainTransform, ModelView, TransformConfig, TransformRegistry,
        };
        use std::collections::BTreeSet;

        fn load_model(path: &str, name: &str) -> KinematicModel {
            let (store, compiled) = load_and_lower(path).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            extract_kinematic_model(&engine, name)
        }

        fn link_name_set(model: &KinematicModel) -> BTreeSet<String> {
            model.links.iter().map(|l| l.name.clone()).collect()
        }

        fn joint_name_set(model: &KinematicModel) -> BTreeSet<String> {
            model.joints.iter().map(|j| j.name.clone()).collect()
        }

        // --- moveo (anthropomorphic arm) -------------------------------------

        #[test]
        fn moveo_using_has_same_link_count_as_baseline() {
            let baseline = load_model(MOVEO_ARM, "moveo");
            let aliased = load_model(MOVEO_ARM_USING, "robot");
            assert_eq!(
                aliased.links.len(),
                baseline.links.len(),
                "aliased fixture lost or gained links during alias resolution"
            );
        }

        #[test]
        fn moveo_using_has_same_joint_count_as_baseline() {
            let baseline = load_model(MOVEO_ARM, "moveo");
            let aliased = load_model(MOVEO_ARM_USING, "robot");
            assert_eq!(
                aliased.joints.len(),
                baseline.joints.len(),
                "aliased fixture lost or gained joints during alias resolution"
            );
        }

        #[test]
        fn moveo_using_link_names_match_baseline() {
            let baseline = load_model(MOVEO_ARM, "moveo");
            let aliased = load_model(MOVEO_ARM_USING, "robot");
            assert_eq!(
                link_name_set(&aliased),
                link_name_set(&baseline),
                "aliased fixture produced a different set of link names"
            );
        }

        #[test]
        fn moveo_using_joint_names_match_baseline() {
            let baseline = load_model(MOVEO_ARM, "moveo");
            let aliased = load_model(MOVEO_ARM_USING, "robot");
            assert_eq!(
                joint_name_set(&aliased),
                joint_name_set(&baseline),
                "aliased fixture produced a different set of joint names"
            );
        }

        #[test]
        fn moveo_using_urdf_structure_matches_baseline() {
            let baseline = load_model(MOVEO_ARM, "moveo");
            let aliased = load_model(MOVEO_ARM_USING, "robot");
            let reg = hymeko_formats::default_registry();
            let urdf = reg.get("urdf").expect("URDF transform registered");

            let cfg = TransformConfig::default().with_name("moveo");
            let out_baseline = urdf.emit(&ModelView::Kinematic(baseline), &cfg).unwrap();
            let out_aliased = urdf.emit(&ModelView::Kinematic(aliased), &cfg).unwrap();

            let link_tag_count = |s: &str| s.matches("<link name=").count();
            let joint_tag_count = |s: &str| s.matches("<joint name=").count();

            assert_eq!(
                link_tag_count(&out_aliased),
                link_tag_count(&out_baseline),
                "aliased URDF has a different <link> count"
            );
            assert_eq!(
                joint_tag_count(&out_aliased),
                joint_tag_count(&out_baseline),
                "aliased URDF has a different <joint> count"
            );
        }

        #[test]
        fn moveo_using_dot_edge_count_matches_baseline() {
            let baseline = load_model(MOVEO_ARM, "moveo");
            let aliased = load_model(MOVEO_ARM_USING, "robot");
            let cfg = TransformConfig::default().with_name("moveo");
            let dot = DotTransform;
            let out_baseline = dot.emit(&ModelView::Kinematic(baseline), &cfg).unwrap();
            let out_aliased = dot.emit(&ModelView::Kinematic(aliased), &cfg).unwrap();
            assert_eq!(
                out_aliased.matches(" -> ").count(),
                out_baseline.matches(" -> ").count(),
                "aliased DOT has a different edge count"
            );
        }

        // --- diff_robot (4-wheel differential drive) -------------------------

        #[test]
        fn diff_robot_using_has_same_link_count_as_baseline() {
            let baseline = load_model(DIFF_ROBOT, "diff_robot");
            let aliased = load_model(DIFF_ROBOT_USING, "diff_robot");
            assert_eq!(aliased.links.len(), baseline.links.len());
        }

        #[test]
        fn diff_robot_using_has_same_joint_count_as_baseline() {
            let baseline = load_model(DIFF_ROBOT, "diff_robot");
            let aliased = load_model(DIFF_ROBOT_USING, "diff_robot");
            assert_eq!(aliased.joints.len(), baseline.joints.len());
        }

        #[test]
        fn diff_robot_using_link_names_match_baseline() {
            let baseline = load_model(DIFF_ROBOT, "diff_robot");
            let aliased = load_model(DIFF_ROBOT_USING, "diff_robot");
            assert_eq!(link_name_set(&aliased), link_name_set(&baseline));
        }

        #[test]
        fn diff_robot_using_joint_names_match_baseline() {
            let baseline = load_model(DIFF_ROBOT, "diff_robot");
            let aliased = load_model(DIFF_ROBOT_USING, "diff_robot");
            assert_eq!(joint_name_set(&aliased), joint_name_set(&baseline));
        }

        #[test]
        fn diff_robot_using_urdf_structure_matches_baseline() {
            let baseline = load_model(DIFF_ROBOT, "diff_robot");
            let aliased = load_model(DIFF_ROBOT_USING, "diff_robot");
            let reg = hymeko_formats::default_registry();
            let urdf = reg.get("urdf").expect("URDF transform registered");

            let cfg = TransformConfig::default().with_name("diff_robot");
            let out_baseline = urdf.emit(&ModelView::Kinematic(baseline), &cfg).unwrap();
            let out_aliased = urdf.emit(&ModelView::Kinematic(aliased), &cfg).unwrap();

            assert_eq!(
                out_aliased.matches("<link name=").count(),
                out_baseline.matches("<link name=").count()
            );
            assert_eq!(
                out_aliased.matches("<joint name=").count(),
                out_baseline.matches("<joint name=").count()
            );
        }

        // --- cross-format parity --------------------------------------------

        #[test]
        fn moveo_using_sdf_link_count_matches_baseline() {
            let baseline = load_model(MOVEO_ARM, "moveo");
            let aliased = load_model(MOVEO_ARM_USING, "robot");
            let reg = hymeko_formats::default_registry();
            let sdf = reg.get("sdf").expect("SDF transform registered");

            let cfg = TransformConfig::default().with_name("moveo");
            let out_baseline = sdf.emit(&ModelView::Kinematic(baseline), &cfg).unwrap();
            let out_aliased = sdf.emit(&ModelView::Kinematic(aliased), &cfg).unwrap();
            assert_eq!(
                out_aliased.matches("<link name=").count(),
                out_baseline.matches("<link name=").count(),
                "SDF <link> count must be invariant under aliasing"
            );
        }

        #[test]
        fn moveo_using_mjcf_body_count_matches_baseline() {
            let baseline = load_model(MOVEO_ARM, "moveo");
            let aliased = load_model(MOVEO_ARM_USING, "robot");
            let reg = hymeko_formats::default_registry();
            let mjcf = reg.get("mjcf").expect("MJCF transform registered");

            let cfg = TransformConfig::default().with_name("moveo");
            let out_baseline = mjcf.emit(&ModelView::Kinematic(baseline), &cfg).unwrap();
            let out_aliased = mjcf.emit(&ModelView::Kinematic(aliased), &cfg).unwrap();
            assert_eq!(
                out_aliased.matches("<body name=").count(),
                out_baseline.matches("<body name=").count(),
                "MJCF <body> count must be invariant under aliasing"
            );
        }

        #[test]
        fn moveo_using_mjcf_hinge_count_matches_baseline() {
            let baseline = load_model(MOVEO_ARM, "moveo");
            let aliased = load_model(MOVEO_ARM_USING, "robot");
            let reg = hymeko_formats::default_registry();
            let mjcf = reg.get("mjcf").expect("MJCF transform registered");

            let cfg = TransformConfig::default().with_name("moveo");
            let out_baseline = mjcf.emit(&ModelView::Kinematic(baseline), &cfg).unwrap();
            let out_aliased = mjcf.emit(&ModelView::Kinematic(aliased), &cfg).unwrap();
            assert_eq!(
                out_aliased.matches("type=\"hinge\"").count(),
                out_baseline.matches("type=\"hinge\"").count(),
                "MJCF hinge-joint count must be invariant under aliasing"
            );
        }

        #[test]
        fn diff_robot_using_sdf_link_count_matches_baseline() {
            let baseline = load_model(DIFF_ROBOT, "diff_robot");
            let aliased = load_model(DIFF_ROBOT_USING, "diff_robot");
            let reg = hymeko_formats::default_registry();
            let sdf = reg.get("sdf").expect("SDF transform registered");

            let cfg = TransformConfig::default().with_name("diff_robot");
            let out_baseline = sdf.emit(&ModelView::Kinematic(baseline), &cfg).unwrap();
            let out_aliased = sdf.emit(&ModelView::Kinematic(aliased), &cfg).unwrap();
            assert_eq!(
                out_aliased.matches("<link name=").count(),
                out_baseline.matches("<link name=").count()
            );
        }

        #[test]
        fn moveo_using_link_name_masses_match_baseline() {
            // Stronger check: link masses keyed by name must be identical.
            let baseline = load_model(MOVEO_ARM, "moveo");
            let aliased = load_model(MOVEO_ARM_USING, "robot");

            let mut map_baseline: std::collections::BTreeMap<_, _> = baseline
                .links
                .iter()
                .map(|l| (l.name.clone(), l.mass))
                .collect();
            let map_aliased: std::collections::BTreeMap<_, _> = aliased
                .links
                .iter()
                .map(|l| (l.name.clone(), l.mass))
                .collect();
            assert_eq!(
                map_aliased, map_baseline,
                "per-link mass must survive alias expansion"
            );
            // Drop the mutable one to silence unused-mut warning.
            map_baseline.clear();
        }
    }

    // ============================================================
    // SMC #5 — requirements-trace witness → SysML v2 + DOT
    // (template-only transforms; no kinematic model, no core edit)
    // ============================================================
    mod requirements_trace {
        use crate::test_helpers::load_and_lower;
        use hymeko_query::transforms::TransformConfig;
        use std::path::PathBuf;

        const WITNESS: &str = "../data/paper/traceability_smc.hymeko";

        fn transforms_root() -> PathBuf {
            PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("..")
                .join("transforms")
        }

        fn render(name: &str) -> String {
            let (store, compiled) = load_and_lower(WITNESS).expect("witness should compile");
            let reg = hymeko_formats::default_registry();
            let config = TransformConfig::default().with_name("pick_place_cell");
            reg.render_from_templates(name, &compiled.ir, &store.it, &config, &transforms_root())
                .expect("transform is registered")
                .expect("template render succeeds")
        }

        #[test]
        fn registry_has_requirements_transforms_without_regressing_others() {
            let reg = hymeko_formats::default_registry();
            let av = reg.available();
            assert!(av.contains(&"requirements_sysml"), "missing requirements_sysml");
            assert!(av.contains(&"requirements_dot"), "missing requirements_dot");
            for f in ["urdf", "sdf", "mjcf", "dot", "mermaid", "sysml"] {
                assert!(av.contains(&f), "registration regressed: missing {f}");
            }
        }

        #[test]
        fn sysml_emits_requirement_defs_satisfy_allocate_derive() {
            let s = render("requirements_sysml");
            assert!(s.contains("requirement def R1_safe_stop"), "requirement def\n{s}");
            assert!(s.contains("part def SafetyController"), "block -> part def");
            assert!(s.contains("satisfy R1_safe_stop by SafetyController;"), "satisfy");
            // R2 has two realizers -> two satisfy statements (binary projection).
            assert!(s.contains("satisfy R2_pos_accuracy by MotionPlanner;"));
            assert!(s.contains("satisfy R2_pos_accuracy by Arm;"));
            assert!(s.contains("allocate R2a_repeatability to Arm;"), "allocate");
            assert!(s.contains("derive_der_rep"), "derive connection");
        }

        #[test]
        fn dot_emits_trace_nodes_and_stereotyped_edges() {
            let s = render("requirements_dot");
            assert!(s.contains("digraph"), "dot graph header");
            assert!(s.contains("\"R1_safe_stop\""), "requirement node");
            assert!(
                s.contains("\"SafetyController\" -> \"R1_safe_stop\""),
                "satisfy edge direction"
            );
            assert!(s.contains("<<satisfy>>"), "satisfy label");
            assert!(s.contains("<<allocate>>"), "allocate label");
            assert!(s.contains("<<deriveReqt>>"), "derive label");
        }
    }
}
