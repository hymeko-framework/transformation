#[cfg(test)]
mod test_transform_ecosystem {
    use hymeko_mechanism::kinematics::kinematic::*;
    use hymeko_formats::plugin::ModelView;
    use hymeko_query::engine::QueryEngine;

    const MOVEO_ARM: &str = "../data/robotics/robots/anthropomorphic_arm.hymeko";
    const MOVEO_ARM_USING: &str = "../data/robotics/robots/anthropomorphic_arm_using.hymeko";
    const DIFF_ROBOT: &str = "../data/robotics/robots/robot_4wh.hymeko";
    const DIFF_ROBOT_USING: &str = "../data/robotics/robots/robot_4wh_using.hymeko";

    // ============================================================
    // Registry tests
    // ============================================================

    mod registry {
        use super::*;
        use crate::test_helpers::load_and_lower;
        use hymeko_formats::plugin::{TransformConfig, TransformRegistry};

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

            let model = hymeko_formats::plugin::extract(
                &compiled.ir,
                &store.it,
                &config.robot_name,
                hymeko_formats::plugin::ModelKind::Kinematic,
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
                let model = hymeko_formats::plugin::extract(
                    &compiled.ir,
                    &store.it,
                    &config.robot_name,
                    hymeko_formats::plugin::ModelKind::Kinematic,
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
        use hymeko_formats::plugin::{DomainTransform, ModelView, TransformRegistry};

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
        use hymeko_formats::plugin::{DomainTransform, ModelView, TransformConfig};

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
        use hymeko_formats::plugin::{DomainTransform, ModelView, TransformConfig};

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
            assert!(
                !output.contains("(Y)"),
                "corrected arm has no local Y-axis joint label"
            );
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
    // ModuleStore::compile() and its module-local alias environments). These tests guarantee that the
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
        use hymeko_formats::plugin::{
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
        use hymeko_formats::plugin::TransformConfig;
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
            assert!(
                av.contains(&"requirements_sysml"),
                "missing requirements_sysml"
            );
            assert!(av.contains(&"requirements_dot"), "missing requirements_dot");
            for f in ["urdf", "sdf", "mjcf", "dot", "mermaid", "sysml"] {
                assert!(av.contains(&f), "registration regressed: missing {f}");
            }
        }

        #[test]
        fn sysml_emits_requirement_defs_satisfy_allocate_derive() {
            let s = render("requirements_sysml");
            assert!(
                s.contains("requirement def R1_safe_stop"),
                "requirement def\n{s}"
            );
            assert!(s.contains("part def SafetyController"), "block -> part def");
            assert!(
                s.contains("satisfy requirement sat_safe references hymeko_req_R1_safe_stop by hymeko_part_SafetyController;"),
                "satisfy"
            );
            // R2 has two realizers -> two satisfy statements (binary projection).
            assert!(s.contains("satisfy requirement sat_pos_mp references hymeko_req_R2_pos_accuracy by hymeko_part_MotionPlanner;"));
            assert!(s.contains("satisfy requirement sat_pos_arm references hymeko_req_R2_pos_accuracy by hymeko_part_Arm;"));
            assert!(
                s.contains(
                    "allocation alloc_rep allocate hymeko_req_R2a_repeatability to hymeko_part_Arm;"
                ),
                "allocate"
            );
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

    // ============================================================
    // SysML v2 structural view (BDD / IBD / deployment), template path
    // (plan docs/plans/2026-10-06-sysml-structure-template)
    // ============================================================
    mod sysml_structure {
        use crate::test_helpers::load_and_lower;
        use hymeko_formats::plugin::TransformConfig;
        use std::path::PathBuf;

        const WITNESS: &str = "../data/sysml/sysml_structure_witness.hymeko";

        fn render(name: &str) -> String {
            let (store, compiled) = load_and_lower(WITNESS).expect("witness should compile");
            let reg = hymeko_formats::default_registry();
            let config = TransformConfig::default().with_name("cell");
            let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("..")
                .join("transforms");
            reg.render_from_templates(name, &compiled.ir, &store.it, &config, &root)
                .expect("transform is registered")
                .expect("template render succeeds")
        }

        #[test]
        fn registry_lists_sysml_structure_without_regressing_others() {
            let reg = hymeko_formats::default_registry();
            let av = reg.available();
            assert!(av.contains(&"sysml_structure"), "missing sysml_structure");
            for f in [
                "requirements_sysml",
                "requirements_dot",
                "sysml",
                "dot",
                "urdf",
            ] {
                assert!(av.contains(&f), "registration regressed: missing {f}");
            }
        }

        #[test]
        fn blocks_specialize_only_blocks_and_ports_are_owner_qualified() {
            let s = render("sysml_structure");
            assert!(
                s.contains("part def IPCamera :> Camera;"),
                "specialization\n{s}"
            );
            assert!(
                s.contains("part def Camera;"),
                "a vocabulary base is not emitted as :>\n{s}"
            );
            assert!(s.contains("port def IPCamera_eth;") && s.contains("port def Switch_access;"));
            assert!(
                !s.contains("part def execution_environment") && !s.contains("part def block"),
                "vocabulary metaclasses must not be projected\n{s}"
            );
        }

        #[test]
        fn composition_multiplicity_from_constants_and_shared_definitions() {
            let s = render("sysml_structure");
            assert!(
                s.contains("end cameras : IPCamera[2..2];"),
                "constant arithmetic\n{s}"
            );
            assert!(
                s.contains("end switch : Switch[1..1];"),
                "shared `one`\n{s}"
            );
            assert!(
                s.contains("end edge : Edge[0..1];"),
                "shared `optional`\n{s}"
            );
            assert!(s.contains("end whole : Cell[1];"));
        }

        #[test]
        fn connector_keeps_ports_and_all_neutral_participants() {
            let s = render("sysml_structure");
            assert!(s.contains("end source : IPCamera_eth;"), "{s}");
            assert!(s.contains("end target : Switch_access;"), "{s}");
            assert!(
                s.contains("conveys: Video, Cat6"),
                "n-ary item + medium\n{s}"
            );
        }

        #[test]
        fn deployment_allocations_and_nary_deploys() {
            let s = render("sysml_structure");
            assert!(s.contains("part def runtime;") && s.contains("part def host;"));
            assert!(s.contains("allocate Edge to host;"), "realizes\n{s}");
            assert!(s.contains("allocate runtime to host;"), "hosts\n{s}");
            assert!(
                s.contains("deploys pipeline, database to runtime"),
                "n-ary deploys\n{s}"
            );
        }

        #[test]
        fn requirements_transform_is_not_affected_by_the_structural_profile() {
            // Counter-test: the same witness through requirements_sysml carries no
            // structural projection, so the two contracts stay separate.
            let s = render("requirements_sysml");
            assert!(
                s.contains("part def IPCamera;"),
                "blocks are still components there\n{s}"
            );
            assert!(!s.contains("port def") && !s.contains(":>"), "{s}");
        }
    }

    // ============================================================
    // Risk assessment and decision-option vocabulary
    // (plan docs/plans/2026-10-07-risk-decision)
    // ============================================================
    mod sysml_risk {
        use crate::test_helpers::load_and_lower;
        use hymeko_query::rewrite::template::{TransformSpec, execute_transform};
        use std::collections::HashMap;

        const WITNESS: &str = "../data/sysml/sysml_risk_witness.hymeko";
        const STRUCTURE_WITNESS: &str = "../data/sysml/sysml_structure_witness.hymeko";

        const QUERIES: &str = "risk_rows {}
context
{
    hazards:    hazard {}
    risks:      risk {}
    categories: risk_category {}
    cells:      risk_cell {}
    decisions:  decision {}
    options:    option {}
    @mitigates_edges:  mitigates {}
    @leads_edges:      leads_to {}
    @considers_edges:  considers {}
    @selects_edges:    selects {}
    @adds_edges:       adds {}
    @omits_edges:      omits {}
    @overrides_edges:  overrides {}
    @introduces_edges: introduces {}
}
";

        const TEMPLATE: &str = "{{#each hazards}}HAZ {{name}}
{{/each}}{{#each risks}}RISK {{name}} {{field:category.label}} {{field:likelihood_initial.rank}} {{field:impact_residual.label}}
{{/each}}{{#each categories}}CAT {{name}}
{{/each}}{{#each cells}}CELL {{field:likelihood.label}} {{field:impact.label}} {{field:level.label}}
{{/each}}{{#each decisions}}DEC {{name}}
{{/each}}{{#each options}}OPT {{name}}
{{/each}}{{#each mitigates_edges}}MIT {{bind:+:all}} {{bind:-:0}}
{{/each}}{{#each leads_edges}}LEAD {{bind:+:0}} {{bind:-:0}}
{{/each}}{{#each considers_edges}}CONS {{bind:+:0}} {{bind:-:all}}
{{/each}}{{#each selects_edges}}SEL {{bind:+:0}} {{bind:-:0}}
{{/each}}{{#each adds_edges}}ADD {{bind:+:0}} {{bind:-:0}} {{bind:~:0}}
{{/each}}{{#each omits_edges}}OMIT {{bind:+:0}} {{bind:-:0}}
{{/each}}{{#each overrides_edges}}OVR {{bind:+:0}} {{bind:-:0}} {{field:value}} {{field:basis}}
{{/each}}{{#each introduces_edges}}INTRO {{bind:+:0}} {{bind:-:0}}
{{/each}}";

        /// Render an inline query/template pair against a model (shared by the vocabulary witnesses).
        pub(super) fn render_inline(model: &str, queries: &str, template: &str) -> String {
            let (store, compiled) = load_and_lower(model).expect("witness should compile");
            let spec = TransformSpec {
                name: "inline_rows".to_string(),
                query_source: queries.to_string(),
                template_source: template.to_string(),
            };
            execute_transform(&compiled.ir, &store.it, &spec, &HashMap::new()).expect("render")
        }

        fn rows(model: &str) -> String {
            render_inline(model, QUERIES, TEMPLATE)
        }

        #[test]
        fn every_kind_and_edge_of_the_vocabulary_resolves() {
            let s = rows(WITNESS);
            for line in [
                "HAZ data_loss",
                "RISK condensation environment 2 high",
                "RISK sensor_fault environment 2 low",
                "CAT environment",
                "CELL high high high",
                "CELL low high low",
                "DEC enclosure",
                "MIT keep_dry condensation",
                "LEAD condensation sensor_fault",
                "LEAD sensor_fault data_loss",
                "CONS enclosure better_sensor heated_box nothing",
                "SEL better_sensor S2",
                "ADD heated_box H1 Heater",
                "OMIT nothing S1",
                "OVR heated_box rh 70 enclosure kept below 70 % RH",
                "INTRO heated_box heater_on",
            ] {
                assert!(s.lines().any(|l| l == line), "missing `{line}`\n{s}");
            }
        }

        #[test]
        fn kinds_match_exactly_and_not_their_neighbours() {
            let s = rows(WITNESS);
            let count = |p: &str| s.lines().filter(|l| l.starts_with(p)).count();
            assert_eq!(
                count("RISK "),
                2,
                "risk must not match risk_cell or risk_category\n{s}"
            );
            assert_eq!(count("CELL "), 4, "a full 2 x 2 table\n{s}");
            assert_eq!(count("OPT "), 3, "{s}");
        }

        #[test]
        fn a_model_without_the_vocabulary_yields_no_rows() {
            let s = rows(STRUCTURE_WITNESS);
            assert!(
                s.trim().is_empty(),
                "structure witness has no risk rows\n{s}"
            );
        }
    }

    // ============================================================
    // Price quotes, currencies and priced_by
    // (plan docs/plans/2026-10-07-price-quotes)
    // ============================================================
    mod sysml_price {
        use super::sysml_risk::render_inline;

        const WITNESS: &str = "../data/sysml/sysml_price_witness.hymeko";
        const STRUCTURE_WITNESS: &str = "../data/sysml/sysml_structure_witness.hymeko";

        const QUERIES: &str = "price_rows {}
context
{
    quotes:     price_quote {}
    currencies: currency {}
    @priced:    priced_by {}
}
";

        const TEMPLATE: &str = "{{#each quotes}}Q {{name}} {{field:amount}} {{field:currency.label}} {{field:currency.rate}} {{field:vat_included}} {{field:pack}}
{{/each}}{{#each currencies}}C {{name}}
{{/each}}{{#each priced}}P {{bind:+:0}} {{bind:-:0}} {{field:per_unit}}
{{/each}}";

        #[test]
        fn quotes_resolve_with_currency_vat_pack_and_per_unit() {
            let s = render_inline(WITNESS, QUERIES, TEMPLATE);
            for line in [
                "Q q_box 93510 HUF 1 0.27 305",
                "Q q_flange 40 EUR 400 0 1",
                "Q q_tube 180 EUR 400 0 1",
                "C EUR",
                "P q_box cable_line 1",
                "P q_flange housing_kit 2",
                "P q_tube housing_kit 1",
            ] {
                assert!(s.lines().any(|l| l == line), "missing `{line}`\n{s}");
            }
        }

        #[test]
        fn a_model_without_quotes_yields_no_rows() {
            let s = render_inline(STRUCTURE_WITNESS, QUERIES, TEMPLATE);
            assert!(s.trim().is_empty(), "structure witness has no quotes\n{s}");
        }
    }
}
