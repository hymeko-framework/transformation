#[cfg(test)]
mod test_generation_engine {
    use hymeko_formats::sdf::generate_sdf;
    use hymeko_formats::urdf::{generate_urdf, validate_robot_schema};
    use hymeko_query::engine::QueryEngine;
    use hymeko_query::kinematics::kinematic::*;
    use hymeko_query::{Predicate, QueryMatch, ValuePredicate};

    // ============================================================
    // Test fixtures
    // ============================================================
    const DIFF_ROBOT: &str = "../data/robotics/robot_4wh.hymeko";
    const MOVEO_ARM: &str = "../data/robotics/anthropomorphic_arm.hymeko";

    /// Helper: extract names from Vec<QueryMatch>
    fn names(matches: &[QueryMatch]) -> Vec<&str> {
        matches.iter().map(|m| m.name.as_str()).collect()
    }

    // ============================================================
    // SECTION 1: Kinematic model extraction
    // ============================================================

    mod kinematic_extraction {
        use super::*;
        use crate::test_helpers::load_and_lower;

        #[test]
        fn diff_robot_links_count() {
            let (store, compiled) = load_and_lower(DIFF_ROBOT).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let r = engine.query(&Predicate::node().and(Predicate::inherits("link")));
            assert_eq!(r.len(), 6, "4 wheels + base + camera = 6 links");
        }

        #[test]
        fn diff_robot_joints_count() {
            let (store, compiled) = load_and_lower(DIFF_ROBOT).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let r = engine.query(&Predicate::edge().and(Predicate::inherits("joint")));
            assert!(
                r.len() >= 5,
                "At least 5 joints (instances + meta templates), got {}",
                r.len()
            );
        }

        #[test]
        fn moveo_links_count() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let r = engine.query(&Predicate::node().and(Predicate::inherits("link")));
            let names = names(&r);
            println!("Moveo links: {:?}", names);
            // world + base_link + link_0..4 + tool = 8
            assert_eq!(
                r.len(),
                7,
                "7 links (world is frame, not link), got {}: {:?}",
                r.len(),
                names
            );
        }

        #[test]
        fn moveo_joint_types() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);

            let fixed = engine.query(&Predicate::edge().and(Predicate::inherits("fixed_joint")));
            let revolute = engine.query(&Predicate::edge().and(Predicate::inherits("rev_joint")));
            println!("Fixed joints: {:?}", names(&fixed));
            println!("Revolute joints: {:?}", names(&revolute));

            assert_eq!(fixed.len(), 1, "1 fixed joint (j_fix)");
            assert_eq!(revolute.len(), 6, "6 revolute joints (j0..j4 + jtool)");
        }

        #[test]
        fn moveo_kinematic_chain_topology() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");

            // Verify kinematic chain: world→base→link0→link1→link2→link3→link4→tool
            let expected_chain = vec![
                ("j_fix", "world", "base_link"),
                ("j0", "base_link", "link_0"),
                ("j1", "link_0", "link_1"),
                ("j2", "link_1", "link_2"),
                ("j3", "link_2", "link_3"),
                ("j4", "link_3", "link_4"),
                ("jtool", "link_4", "tool"),
            ];

            for (jname, expected_parent, expected_child) in &expected_chain {
                let joint = model.joints.iter().find(|j| j.name == *jname);
                assert!(joint.is_some(), "Joint '{}' not found in model", jname);
                let joint = joint.unwrap();
                assert_eq!(
                    joint.parent_link, *expected_parent,
                    "Joint '{}' parent: expected '{}', got '{}'",
                    jname, expected_parent, joint.parent_link
                );
                assert_eq!(
                    joint.child_link, *expected_child,
                    "Joint '{}' child: expected '{}', got '{}'",
                    jname, expected_child, joint.child_link
                );
            }
        }

        #[test]
        fn moveo_joint_axes() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");

            // Canonical anthropomorphic arm: world axes Z,Y,Y,Z,Y,Z (shoulder ∥ elbow). The
            // local AXIS_* tokens are Z,X,X,Z,X,Z — link_1..tool carry j1's 90°-about-Z frame
            // twist, so local X reads as world Y for j2/j4 and local Z stays world Z for j3.
            let expected_axes: Vec<(&str, [f64; 3])> = vec![
                ("j0", [0.0, 0.0, 1.0]),    // Z  (waist)
                ("j1", [1.0, 0.0, 0.0]),    // X  -> world Y (shoulder pitch)
                ("j2", [1.0, 0.0, 0.0]),    // X  -> world Y (elbow pitch, ∥ shoulder)
                ("j3", [0.0, 0.0, 1.0]),    // Z  -> world Z (forearm roll)
                ("j4", [1.0, 0.0, 0.0]),    // X  -> world Y (wrist pitch)
                ("jtool", [0.0, 0.0, 1.0]), // Z  (tool roll)
            ];

            for (jname, expected_axis) in &expected_axes {
                let joint = model.joints.iter().find(|j| j.name == *jname).unwrap();
                assert!(joint.axis.is_some(), "Joint '{}' missing axis", jname);
                let axis = joint.axis.unwrap();
                for d in 0..3 {
                    assert!(
                        (axis[d] - expected_axis[d]).abs() < 1e-9,
                        "Joint '{}' axis[{}]: expected {}, got {}",
                        jname,
                        d,
                        expected_axis[d],
                        axis[d]
                    );
                }
            }
        }

        #[test]
        fn moveo_joint_origins() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");

            // j1 has origin [[0.0, 0.15, 0.15], [0.0, 0.0, 90.0]]
            let j1 = model.joints.iter().find(|j| j.name == "j1").unwrap();
            assert!(j1.origin_xyz.is_some(), "j1 missing origin xyz");
            let xyz = j1.origin_xyz.unwrap();
            assert!((xyz[0] - 0.0).abs() < 1e-9);
            assert!((xyz[1] - 0.15).abs() < 1e-9);
            assert!((xyz[2] - 0.15).abs() < 1e-9);

            // RPY in degrees: [0, 0, 90]
            assert!(j1.origin_rpy_deg.is_some(), "j1 missing origin rpy");
            let rpy = j1.origin_rpy_deg.unwrap();
            assert!((rpy[2] - 90.0).abs() < 1e-9, "j1 rpy yaw should be 90°");
        }

        #[test]
        fn moveo_link_masses() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");

            let expected_masses: Vec<(&str, f64)> = vec![
                ("base_link", 25.0),
                ("link_0", 5.0),
                ("link_1", 2.0),
                ("link_2", 2.0),
                ("link_3", 2.0),
                ("link_4", 2.0),
                ("tool", 0.5),
            ];

            for (lname, expected_mass) in &expected_masses {
                let link = model.links.iter().find(|l| l.name == *lname);
                assert!(link.is_some(), "Link '{}' not found", lname);
                let link = link.unwrap();
                assert!(link.mass.is_some(), "Link '{}' missing mass", lname);
                assert!(
                    (link.mass.unwrap() - expected_mass).abs() < 1e-9,
                    "Link '{}' mass: expected {}, got {:?}",
                    lname,
                    expected_mass,
                    link.mass
                );
            }
        }

        #[test]
        fn moveo_link_geometries() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let model = extract_kinematic_model(&engine, "moveo");

            // base_link: cylinder [0.13, 0.05]
            let base = model.links.iter().find(|l| l.name == "base_link").unwrap();
            assert!(base.geometry.is_some());
            let geom = base.geometry.as_ref().unwrap();
            assert_eq!(geom.shape, GeometryShape::Cylinder);
            assert!((geom.dimensions[0] - 0.13).abs() < 1e-9);
            assert!((geom.dimensions[1] - 0.05).abs() < 1e-9);

            // tool: box [0.075, 0.15, 0.1]
            let tool = model.links.iter().find(|l| l.name == "tool").unwrap();
            assert!(tool.geometry.is_some());
            let geom = tool.geometry.as_ref().unwrap();
            assert_eq!(geom.shape, GeometryShape::Box);
            assert_eq!(geom.dimensions.len(), 3);
        }
    }

    // ============================================================
    // SECTION 2: URDF generation
    // ============================================================

    mod urdf_generation {
        use super::*;
        use crate::test_helpers::load_and_lower;

        #[test]
        fn moveo_urdf_xml_wellformed() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let urdf = generate_urdf(&compiled.ir, &store.it, "moveo");
            println!("--- Generated Moveo URDF ---\n{}", urdf);

            assert!(urdf.starts_with("<?xml"));
            assert!(urdf.ends_with("</robot>\n"));
            assert!(urdf.contains("<robot name=\"moveo\""));
        }

        #[test]
        fn moveo_urdf_all_links_present() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let urdf = generate_urdf(&compiled.ir, &store.it, "moveo");

            for name in &[
                "base_link",
                "link_0",
                "link_1",
                "link_2",
                "link_3",
                "link_4",
                "tool",
            ] {
                assert!(
                    urdf.contains(&format!("<link name=\"{}\"", name)),
                    "URDF missing link '{}'",
                    name
                );
            }
        }

        #[test]
        fn moveo_urdf_all_joints_present() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let urdf = generate_urdf(&compiled.ir, &store.it, "moveo");

            for name in &["j_fix", "j0", "j1", "j2", "j3", "j4", "jtool"] {
                assert!(
                    urdf.contains(&format!("<joint name=\"{}\"", name)),
                    "URDF missing joint '{}'",
                    name
                );
            }
        }

        #[test]
        fn moveo_urdf_joint_types_correct() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let urdf = generate_urdf(&compiled.ir, &store.it, "moveo");

            // j_fix should be fixed
            assert!(
                urdf.contains("joint name=\"j_fix\" type=\"fixed\""),
                "j_fix should be type='fixed'"
            );

            // j0..jtool should be revolute
            let revolute_count = urdf.matches("type=\"revolute\"").count();
            assert_eq!(
                revolute_count, 6,
                "Expected 6 revolute joints, got {}",
                revolute_count
            );
        }

        #[test]
        fn moveo_urdf_parent_child_correct() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let urdf = generate_urdf(&compiled.ir, &store.it, "moveo");

            // j0: parent=base_link, child=link_0
            assert!(urdf.contains("<parent link=\"base_link\"/>"));
            assert!(urdf.contains("<child link=\"link_0\"/>"));

            // jtool: parent=link_4, child=tool
            assert!(urdf.contains("<parent link=\"link_4\"/>"));
            assert!(urdf.contains("<child link=\"tool\"/>"));
        }

        #[test]
        fn moveo_urdf_origin_with_rpy_conversion() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let urdf = generate_urdf(&compiled.ir, &store.it, "moveo");

            // j1 has rpy = [0, 0, 90°] → [0, 0, 1.5708 rad]
            // Check that radian conversion happened
            assert!(
                urdf.contains("1.5708") || urdf.contains("1.571"),
                "Expected j1 yaw 90° → ~1.5708 rad in URDF"
            );
        }

        #[test]
        fn moveo_urdf_axes_present() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let urdf = generate_urdf(&compiled.ir, &store.it, "moveo");

            let axis_count = urdf.matches("<axis xyz=").count();
            assert_eq!(
                axis_count, 6,
                "6 revolute joints should have axes, got {}",
                axis_count
            );

            // Z axis for j0/j3/jtool
            assert!(urdf.contains("<axis xyz=\"0 0 1\"/>"));
            // X axis for j1/j2/j4
            assert!(urdf.contains("<axis xyz=\"1 0 0\"/>"));
            // the canonical anthropomorphic arm (shoulder ∥ elbow) uses only X and Z locally —
            // no Y axis token (world Y is produced by j1's 90° twist acting on local X).
            assert!(!urdf.contains("<axis xyz=\"0 1 0\"/>"));
        }

        #[test]
        fn moveo_urdf_geometry_correct() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let urdf = generate_urdf(&compiled.ir, &store.it, "moveo");

            // base_link: cylinder radius=0.13 length=0.05
            assert!(
                urdf.contains("<cylinder radius=\"0.13\" length=\"0.05\""),
                "base_link geometry missing or wrong"
            );

            // tool: box
            assert!(urdf.contains("<box size="), "tool box geometry missing");
        }

        #[test]
        fn moveo_urdf_schema_valid() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let errors = validate_robot_schema(&compiled.ir, &store.it);
            if !errors.is_empty() {
                for e in &errors {
                    eprintln!("  Schema error: {}", e);
                }
            }
            // j_fix parent "world" is a frame, not in links list
            let real_errors: Vec<_> = errors.iter().filter(|e| !e.contains("world")).collect();
            assert!(
                real_errors.is_empty(),
                "Schema validation failed: {:?}",
                real_errors
            );
        }

        #[test]
        fn moveo_urdf_link_count_matches_joint_refs() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let urdf = generate_urdf(&compiled.ir, &store.it, "moveo");

            let link_count = urdf.matches("<link name=").count();
            let joint_count = urdf.matches("<joint name=").count();

            // For a serial chain: joints = links - 1
            // (including world frame: 8 links, 7 joints)
            println!("Links: {}, Joints: {}", link_count, joint_count);
            assert!(
                joint_count == link_count - 1 || joint_count == link_count,
                "Joint/link count mismatch: {} joints, {} links",
                joint_count,
                link_count
            );
        }

        #[test]
        fn moveo_urdf_mass_elements() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let urdf = generate_urdf(&compiled.ir, &store.it, "moveo");

            assert!(
                urdf.contains("<mass value=\"25\"/>"),
                "base_link mass should be 25"
            );
            assert!(
                urdf.contains("<mass value=\"5\"/>"),
                "link_0 mass should be 5"
            );
            assert!(
                urdf.contains("<mass value=\"0.5\"/>"),
                "tool mass should be 0.5"
            );
        }

        #[test]
        fn moveo_urdf_color_material() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let urdf = generate_urdf(&compiled.ir, &store.it, "moveo");

            // Should have material/color elements
            assert!(
                urdf.contains("<material name=\"color\">"),
                "Color material missing"
            );
            assert!(urdf.contains("<color rgba="), "Color rgba missing");
        }
    }

    // ============================================================
    // SECTION 3: SDF generation
    // ============================================================

    mod sdf_generation {
        use super::*;
        use crate::test_helpers::load_and_lower;

        #[test]
        fn moveo_sdf_wellformed() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let sdf = generate_sdf(&compiled.ir, &store.it, "moveo");
            println!("--- Generated Moveo SDF ---\n{}", sdf);

            assert!(sdf.starts_with("<?xml"));
            assert!(sdf.contains("<sdf version=\"1.7\">"));
            assert!(sdf.contains("<model name=\"moveo\">"));
            assert!(sdf.ends_with("</sdf>\n"));
        }

        #[test]
        fn moveo_sdf_links_present() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let sdf = generate_sdf(&compiled.ir, &store.it, "moveo");

            for name in &[
                "base_link",
                "link_0",
                "link_1",
                "link_2",
                "link_3",
                "link_4",
                "tool",
            ] {
                assert!(
                    sdf.contains(&format!("<link name=\"{}\"", name)),
                    "SDF missing link '{}'",
                    name
                );
            }
        }

        #[test]
        fn moveo_sdf_joints_present() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let sdf = generate_sdf(&compiled.ir, &store.it, "moveo");

            for name in &["j0", "j1", "j2", "j3", "j4", "jtool"] {
                assert!(
                    sdf.contains(&format!("<joint name=\"{}\"", name)),
                    "SDF missing joint '{}'",
                    name
                );
            }
        }

        #[test]
        fn moveo_sdf_inertial_diagonal() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let sdf = generate_sdf(&compiled.ir, &store.it, "moveo");

            // SDF should have inertial elements with diagonal approximation
            assert!(sdf.contains("<inertial>"), "SDF missing inertial elements");
            assert!(
                sdf.contains("<mass>25</mass>"),
                "SDF missing base_link mass"
            );
        }

        #[test]
        fn moveo_sdf_geometry_types() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let sdf = generate_sdf(&compiled.ir, &store.it, "moveo");

            assert!(sdf.contains("<cylinder>"), "SDF missing cylinder geometry");
            assert!(sdf.contains("<box>"), "SDF missing box geometry");
        }

        #[test]
        fn moveo_sdf_revolute_type() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let sdf = generate_sdf(&compiled.ir, &store.it, "moveo");

            // SDF uses "revolute" for both revolute and continuous
            let revolute_count = sdf.matches("type=\"revolute\"").count();
            assert!(
                revolute_count >= 6,
                "Expected >=6 revolute joints in SDF, got {}",
                revolute_count
            );
        }

        #[test]
        fn moveo_sdf_pose_relative_to() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let sdf = generate_sdf(&compiled.ir, &store.it, "moveo");

            // SDF poses should have relative_to attribute
            assert!(
                sdf.contains("relative_to="),
                "SDF poses should have relative_to attribute"
            );
        }
    }

    // ============================================================
    // SECTION 4: Cross-format consistency
    // ============================================================

    mod cross_format {
        use super::*;
        use crate::test_helpers::load_and_lower;

        #[test]
        fn urdf_and_sdf_same_link_count() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let urdf = generate_urdf(&compiled.ir, &store.it, "moveo");
            let sdf = generate_sdf(&compiled.ir, &store.it, "moveo");

            // URDF may emit a `<link name="world"/>` stub as a spec-mandated
            // anchor for fixed joints to the world frame; SDF expresses the
            // world attachment via `<pose>` and has no such stub. Exclude it
            // so the count reflects the kinematic topology of the robot.
            let urdf_links =
                urdf.matches("<link name=").count() - urdf.matches("<link name=\"world\"").count();
            let sdf_links = sdf.matches("<link name=").count();

            assert_eq!(
                urdf_links, sdf_links,
                "URDF has {} links but SDF has {}",
                urdf_links, sdf_links
            );
        }

        #[test]
        fn urdf_and_sdf_same_joint_count() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let urdf = generate_urdf(&compiled.ir, &store.it, "moveo");
            let sdf = generate_sdf(&compiled.ir, &store.it, "moveo");

            let urdf_joints = urdf.matches("<joint name=").count();
            let sdf_joints = sdf.matches("<joint name=").count();

            assert_eq!(
                urdf_joints, sdf_joints,
                "URDF has {} joints but SDF has {}",
                urdf_joints, sdf_joints
            );
        }

        #[test]
        fn both_robots_generate_without_panic() {
            for (path, name) in &[(DIFF_ROBOT, "diff_robot"), (MOVEO_ARM, "moveo")] {
                let (store, compiled) = load_and_lower(path).unwrap();
                let urdf = generate_urdf(&compiled.ir, &store.it, name);
                let sdf = generate_sdf(&compiled.ir, &store.it, name);
                assert!(!urdf.is_empty(), "{} URDF empty", name);
                assert!(!sdf.is_empty(), "{} SDF empty", name);
            }
        }

        #[test]
        fn kinematic_model_identical_source() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let m1 = extract_kinematic_model(&engine, "moveo");
            let m2 = extract_kinematic_model(&engine, "moveo");

            assert_eq!(m1.links.len(), m2.links.len());
            assert_eq!(m1.joints.len(), m2.joints.len());
            for (a, b) in m1.joints.iter().zip(m2.joints.iter()) {
                assert_eq!(a.name, b.name);
                assert_eq!(a.parent_link, b.parent_link);
                assert_eq!(a.child_link, b.child_link);
            }
        }
    }

    // ============================================================
    // SECTION 5: Query engine edge cases
    // ============================================================

    mod query_edge_cases {
        use super::*;
        use crate::test_helpers::load_and_lower;

        #[test]
        fn query_nonexistent_type_returns_empty() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);
            let r = engine.query(&Predicate::edge().and(Predicate::inherits("prismatic_joint")));
            assert_eq!(r.len(), 0, "No prismatic joints in Moveo");
        }

        #[test]
        fn query_combined_predicates() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);

            // Heavy links that inherit from link
            let r = engine.query(&Predicate::node().and(Predicate::inherits("link")).and(
                Predicate::ChildValue("mass".into(), ValuePredicate::NumGt(10.0)),
            ));
            assert_eq!(r.len(), 1, "Only base_link > 10kg");
            assert!(names(&r).contains(&"base_link"));
        }

        #[test]
        fn query_signed_refs_on_joints() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);

            // All revolute joints must have both + and - refs to links
            let rev_joints = engine.query(&Predicate::edge().and(Predicate::inherits("rev_joint")));

            for m in &rev_joints {
                let has_plus_link = m.arc_bindings.iter().any(|b| b.sign == 1);
                let has_minus_link = m.arc_bindings.iter().any(|b| b.sign == -1);

                assert!(has_plus_link, "Joint '{}' missing +link (parent)", m.name);
                assert!(has_minus_link, "Joint '{}' missing -link (child)", m.name);
            }
        }

        #[test]
        fn query_or_predicate() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);

            let r = engine.query(
                &Predicate::edge()
                    .and(Predicate::inherits("rev_joint").or(Predicate::inherits("fixed_joint"))),
            );

            assert_eq!(r.len(), 7, "6 revolute + 1 fixed = 7");
        }

        #[test]
        fn query_not_predicate() {
            let (store, compiled) = load_and_lower(MOVEO_ARM).unwrap();
            let engine = QueryEngine::new(&compiled.ir, &store.it);

            // Links that are NOT base_link
            let r = engine.query(
                &Predicate::node()
                    .and(Predicate::inherits("link"))
                    .and(Predicate::named("base_link").not()),
            );

            assert!(!names(&r).contains(&"base_link"));
            assert!(r.len() >= 6, "Should have link_0..4, tool, world");
        }
    }
}
