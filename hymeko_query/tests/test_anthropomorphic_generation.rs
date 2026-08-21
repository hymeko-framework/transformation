//! Dedicated generation suite for `data/robotics/anthropomorphic_arm.hymeko`.
//!
//! Where `test_generation_engine.rs` spreads moveo coverage across
//! kinematic-extraction / URDF / SDF sections, this file is organised
//! around the **robot itself** — asserts its specific 6-DoF anthropomorphic
//! signature, its serial-chain topology, its per-joint axis pattern, the
//! joint-limit presence in the IR, and the control/simulation hyperedges
//! the fixture declares. Those hyperedges (`gazebo_sim_system`,
//! `sim_control_plugin`, `joint_trajectory_controller`) are not exercised
//! by any existing test.
//!
//! URDF and SDF emission goes via `hymeko_query::formats::{urdf,sdf}`
//! directly; the `TransformRegistry` entries for URDF/SDF are currently
//! stubs (TODO comments in `transforms/mod.rs`), so tests that assert
//! link/joint content use the full generators. MJCF and DOT emission go
//! via the registry since those are complete.

#[cfg(test)]
mod test_anthropomorphic_generation {
    use hymeko_formats::sdf::generate_sdf;
    use hymeko_formats::urdf::generate_urdf;
    use hymeko_query::engine::QueryEngine;
    use hymeko_query::kinematics::joints::{JointInfo, JointType};
    use hymeko_query::kinematics::kinematic::*;
    use hymeko_query::transforms::{
        DomainTransform, ModelView, TransformConfig, TransformRegistry,
    };
    use hymeko_query::{Predicate, ValuePredicate};
    use log::info;
    use std::time::Instant;

    use crate::test_helpers::{load_and_lower, log_test_footer, log_test_header};

    const MOVEO: &str = "../data/robotics/anthropomorphic_arm.hymeko";
    const ROBOT_NAME: &str = "moveo";

    /// Links the kinematic extractor should produce. `world` is declared
    /// as a `frame` in the fixture, not a `link`, so it is NOT in this set.
    const EXPECTED_LINKS: &[&str] = &[
        "base_link",
        "link_0",
        "link_1",
        "link_2",
        "link_3",
        "link_4",
        "tool",
    ];

    /// Joints in fixture order, including the fixed world→base.
    const EXPECTED_JOINTS: &[&str] = &["j_fix", "j0", "j1", "j2", "j3", "j4", "jtool"];

    /// Centralised loader — every test in this file uses the same setup.
    fn model() -> KinematicModel {
        let (store, compiled) = load_and_lower(MOVEO).expect("moveo should compile");
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        extract_kinematic_model(&engine, ROBOT_NAME)
    }

    fn find_joint<'a>(model: &'a KinematicModel, name: &str) -> &'a JointInfo {
        model
            .joints
            .iter()
            .find(|j| j.name == name)
            .unwrap_or_else(|| panic!("joint {name} not found"))
    }

    /// Regression for B-005 (joint-axis extraction).
    ///
    /// The fixture declares **mixed** per-joint axes via `- ...axes.AXIS_*` arcs. After the
    /// 2026-06-19 canonical-axis correction the local tokens are j0=Z, j1=X, j2=X, j3=Z, j4=X,
    /// jtool=Z (which, through j1's 90°-about-Z twist on link_1..tool, give the canonical world
    /// pattern Z,Y,Y,Z,Y,Z — shoulder ∥ elbow). The extractor must read each joint's axis from
    /// its `axis_definition` arc — *not* default everything to Z, which collapses the arm to a
    /// colinear spinner with zero end-effector workspace (the symptom that surfaced building
    /// hymeko_rl). The bug emitted `axis="0 0 1"` for every joint.
    #[test]
    fn per_joint_axes_match_the_source_b005() {
        let m = model();
        let expected: &[(&str, [f64; 3])] = &[
            ("j0", [0.0, 0.0, 1.0]),
            ("j1", [1.0, 0.0, 0.0]),
            ("j2", [1.0, 0.0, 0.0]),
            ("j3", [0.0, 0.0, 1.0]),
            ("j4", [1.0, 0.0, 0.0]),
            ("jtool", [0.0, 0.0, 1.0]),
        ];
        for (name, axis) in expected {
            let j = find_joint(&m, name);
            assert_eq!(
                j.axis,
                Some(*axis),
                "joint {name}: axis must be read from the AXIS_* arc, got {:?}",
                j.axis
            );
        }
        // Degeneracy guard: the revolute joints must NOT all share one axis.
        let distinct: std::collections::HashSet<String> = m
            .joints
            .iter()
            .filter_map(|j| j.axis.map(|a| format!("{a:?}")))
            .collect();
        assert!(
            distinct.len() >= 2,
            "all joint axes collapsed onto a single direction (B-005)"
        );
    }

    /// Regression for joint-limit extraction through the `limit -> …` reference.
    ///
    /// `@j0`/`@j1` declare `limit -> joint0_limit` (effort 500 N·m); `@j2..@jtool` inherit
    /// `limit -> joint_rev_limit` (effort 50) from `rev_joint`. The extractor must follow
    /// the ref — declared *or* inherited — not return `None` (which left the emitted joints
    /// unlimited with no actuator ctrlrange, making torque-control reaching unlearnable).
    #[test]
    fn per_joint_limits_resolved_through_the_limit_ref() {
        let m = model();
        let expected_effort: &[(&str, f64)] = &[
            ("j0", 500.0),
            ("j1", 500.0),
            ("j2", 50.0),
            ("j3", 50.0),
            ("j4", 50.0),
            ("jtool", 50.0),
        ];
        for (name, effort) in expected_effort {
            let j = find_joint(&m, name);
            let lim = j.limits.as_ref().unwrap_or_else(|| {
                panic!("joint {name}: no limits — the `limit` ref was not followed")
            });
            assert_eq!(lim.effort, *effort, "joint {name} effort (N·m)");
            assert_eq!(lim.lower, -180.0, "joint {name} lower (deg)");
            assert_eq!(lim.upper, 180.0, "joint {name} upper (deg)");
        }
    }

    /// Regression for B-004 / B-005 / world-collision / inertial-pos at the **MJCF
    /// emit** layer — the end-to-end product the CLI `emit -f mjcf` now ships.
    ///
    /// Exercises the exact path the CLI uses (registry transform `emit()` over a
    /// `ModelView::Kinematic`), guarding the four defects that made the emitted arm
    /// unusable in MuJoCo:
    ///   - B-004: the static template emitted **no `<joint>`** at all;
    ///   - B-005: axes hardcoded to Z (`emit` must carry the mixed axes);
    ///   - the root `world` frame collided with MuJoCo's implicit `<worldbody>`;
    ///   - `<inertial>` lacked the required `pos` attribute (load error).
    #[test]
    fn mjcf_emit_is_loadable_and_articulable_b004_b005() {
        use hymeko_query::transforms::{ModelView, TransformConfig};

        let reg = hymeko_formats::default_registry();
        let t = reg.get("mjcf").expect("mjcf transform must be registered");
        let view = ModelView::Kinematic(model());
        let cfg = TransformConfig::default().with_name(ROBOT_NAME);
        let mjcf = t
            .emit(&view, &cfg)
            .expect("mjcf emit must produce output for a kinematic model");

        // B-004: the six revolute joints must be present (the template emitted none).
        let n_joints = mjcf.matches("<joint ").count();
        assert_eq!(n_joints, 6, "expected 6 hinge joints in MJCF, got {n_joints}");

        // B-005: mixed per-joint axes survive to the emitted scene. The canonical arm uses only
        // local X and Z (no Y token) — both must be present (mixedness), proving axes are not
        // collapsed onto a single direction.
        assert!(mjcf.contains("axis=\"1 0 0\""), "MJCF missing an X-axis joint (B-005)");
        assert!(mjcf.contains("axis=\"0 0 1\""), "MJCF missing a Z-axis joint (B-005)");

        // world frame maps onto the implicit worldbody — no colliding `<body name="world">`.
        assert!(
            !mjcf.contains("<body name=\"world\""),
            "world frame must not be emitted as a body (MuJoCo name collision)"
        );

        // every `<inertial>` carries the required `pos` attribute.
        for line in mjcf.lines().filter(|l| l.contains("<inertial")) {
            assert!(line.contains("pos="), "<inertial> missing required pos: {line}");
        }
    }

    // =====================================================================
    // SECTION 1: Structural signature — serial chain, 6 DoF + fixed base
    // =====================================================================

    #[test]
    fn fixture_declares_seven_robot_links() {
        let title = "fixture_declares_seven_robot_links";
        log_test_header(
            title,
            "Confirms kinematic extractor returns the expected 7 link-typed nodes.",
        );
        let start = Instant::now();

        let m = model();
        info!("extracted {} links from moveo", m.links.len());
        for l in &m.links {
            info!("  link `{}` mass={:?}", l.name, l.mass);
        }

        for expected in EXPECTED_LINKS {
            assert!(
                m.links.iter().any(|l| l.name == *expected),
                "missing link `{expected}`"
            );
        }
        assert_eq!(m.links.len(), EXPECTED_LINKS.len());

        log_test_footer(
            title,
            Some(start.elapsed()),
            "all 7 expected links present.",
        );
    }

    #[test]
    fn fixture_declares_seven_joints_one_fixed_six_revolute() {
        let title = "fixture_declares_seven_joints_one_fixed_six_revolute";
        log_test_header(
            title,
            "Counts fixed vs revolute joints — moveo is 1 fixed + 6 revolute.",
        );
        let start = Instant::now();

        let m = model();
        let fixed = m
            .joints
            .iter()
            .filter(|j| j.joint_type == JointType::Fixed)
            .count();
        let revolute = m
            .joints
            .iter()
            .filter(|j| j.joint_type == JointType::Revolute)
            .count();
        info!(
            "joint-type census: fixed={fixed}, revolute={revolute}, total={}",
            m.joints.len()
        );

        assert_eq!(fixed, 1);
        assert_eq!(revolute, 6);
        assert_eq!(m.joints.len(), EXPECTED_JOINTS.len());

        log_test_footer(
            title,
            Some(start.elapsed()),
            "joint counts match anthropomorphic signature.",
        );
    }

    #[test]
    fn j_fix_connects_world_to_base() {
        let title = "j_fix_connects_world_to_base";
        log_test_header(title, "Validates the world→base_link fixed joint.");
        let start = Instant::now();

        let m = model();
        let j = find_joint(&m, "j_fix");
        info!(
            "j_fix: parent=`{}`, child=`{}`, type={:?}",
            j.parent_link, j.child_link, j.joint_type
        );

        assert_eq!(j.joint_type, JointType::Fixed);
        assert_eq!(j.parent_link, "world");
        assert_eq!(j.child_link, "base_link");

        log_test_footer(title, Some(start.elapsed()), "j_fix parent/child correct.");
    }

    #[test]
    fn serial_chain_topology_parent_child_adjacency() {
        let title = "serial_chain_topology_parent_child_adjacency";
        log_test_header(
            title,
            "Validates the full world → base_link → link_0..4 → tool chain.",
        );
        let start = Instant::now();

        let m = model();
        let expected = [
            ("j_fix", "world", "base_link"),
            ("j0", "base_link", "link_0"),
            ("j1", "link_0", "link_1"),
            ("j2", "link_1", "link_2"),
            ("j3", "link_2", "link_3"),
            ("j4", "link_3", "link_4"),
            ("jtool", "link_4", "tool"),
        ];
        for (name, parent, child) in expected {
            let j = find_joint(&m, name);
            info!("  {:<6} : {:<10} -> {}", name, parent, child);
            assert_eq!(j.parent_link, parent, "{name} parent mismatch");
            assert_eq!(j.child_link, child, "{name} child mismatch");
        }

        log_test_footer(
            title,
            Some(start.elapsed()),
            "serial chain intact end-to-end.",
        );
    }

    #[test]
    fn each_link_appears_as_child_at_most_once() {
        // Serial-chain invariant: every rigid body (other than the root)
        // has exactly one parent, so each link name appears as a joint's
        // `child_link` at most once.
        let m = model();
        let mut seen = std::collections::BTreeMap::<String, usize>::new();
        for j in &m.joints {
            *seen.entry(j.child_link.clone()).or_default() += 1;
        }
        for (child, n) in &seen {
            assert!(
                *n <= 1,
                "link `{child}` is the child of {n} joints — not a tree"
            );
        }
    }

    // =====================================================================
    // SECTION 2: Kinematic specifics — axis pattern, mass, limits
    // =====================================================================

    /// Axis letter ("X"/"Y"/"Z") from a unit-axis vector.
    fn axis_letter(a: [f64; 3]) -> Option<char> {
        if (a[0].abs() - 1.0).abs() < 1e-6 {
            return Some('X');
        }
        if (a[1].abs() - 1.0).abs() < 1e-6 {
            return Some('Y');
        }
        if (a[2].abs() - 1.0).abs() < 1e-6 {
            return Some('Z');
        }
        None
    }

    #[test]
    fn revolute_joints_use_only_canonical_axes() {
        let m = model();
        for j in m
            .joints
            .iter()
            .filter(|j| j.joint_type == JointType::Revolute)
        {
            let axis = j
                .axis
                .unwrap_or_else(|| panic!("revolute {} missing axis", j.name));
            assert!(
                axis_letter(axis).is_some(),
                "joint {} has non-canonical axis {:?}",
                j.name,
                axis
            );
        }
    }

    #[test]
    fn six_dof_axis_signature_matches_fixture() {
        let title = "six_dof_axis_signature_matches_fixture";
        log_test_header(
            title,
            "Verifies the j0=Z, j1=X, j2=X, j3=Z, j4=X, jtool=Z local-axis pattern.",
        );
        let start = Instant::now();

        let m = model();
        let expected = [
            ("j0", 'Z'),
            ("j1", 'X'),
            ("j2", 'X'),
            ("j3", 'Z'),
            ("j4", 'X'),
            ("jtool", 'Z'),
        ];
        let mut pattern = String::new();
        for (name, letter) in expected {
            let j = find_joint(&m, name);
            let got = axis_letter(j.axis.unwrap()).unwrap();
            pattern.push(got);
            assert_eq!(got, letter, "{name} axis mismatch: got {got} want {letter}");
        }
        // Local tokens ZXXZXZ -> canonical world axes Z,Y,Y,Z,Y,Z via j1's 90° twist.
        info!("6-DoF local-axis signature: {pattern}  (expected: ZXXZXZ)");

        log_test_footer(
            title,
            Some(start.elapsed()),
            "axis pattern matches canonical anthropomorphic-arm signature.",
        );
    }

    #[test]
    fn j1_has_ninety_degree_twist_about_z() {
        // The fixture writes `j1` with rpy `[0.0, 0.0, 90.0]` (degrees) —
        // this captures the shoulder's orthogonal offset.
        let m = model();
        let j1 = find_joint(&m, "j1");
        let rpy_deg = j1.origin_rpy_deg.expect("j1 should have rpy");
        assert!(
            (rpy_deg[2] - 90.0).abs() < 1e-6,
            "j1 yaw should be 90°, got {}",
            rpy_deg[2]
        );
    }

    #[test]
    fn base_link_is_heaviest_link() {
        let title = "base_link_is_heaviest_link";
        log_test_header(
            title,
            "base_link should dominate the mass budget for the arm.",
        );
        let start = Instant::now();

        let m = model();
        let mut masses: Vec<(String, f64)> = m
            .links
            .iter()
            .filter_map(|l| l.mass.map(|mm| (l.name.clone(), mm)))
            .collect();
        masses.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        info!("mass ranking:");
        for (n, m) in &masses {
            info!("  {n:<12} {m:>6.2} kg");
        }
        let total: f64 = masses.iter().map(|(_, m)| m).sum();
        info!(
            "total robot mass: {total:.2} kg across {} links",
            masses.len()
        );

        let (name, mass) = masses.first().cloned().expect("some link must have mass");
        assert_eq!(name, "base_link");
        assert!((mass - 25.0).abs() < 1e-6);

        log_test_footer(title, Some(start.elapsed()), "base_link = 25 kg, heaviest.");
    }

    #[test]
    fn joint0_limit_node_carries_expected_values() {
        // The fixture declares a shared `joint0_limit { lower -180.0;
        // upper 180.0; effort 500.0; velocity 4.0; }` node; `j0` and `j1`
        // reference it via `limit -> joint0_limit;`. The kinematic
        // extractor does not (yet) dereference that link into
        // `JointInfo.limits` — see the TODO in
        // `hymeko_query/src/kinematics/kinematic.rs::extract_joint_limits`.
        // For now we assert the *source of truth* is wired by querying the
        // joint0_limit node directly with a child-value predicate.
        let (store, compiled) = load_and_lower(MOVEO).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);

        for (child, expected) in [
            ("lower", -180.0_f64),
            ("upper", 180.0_f64),
            ("effort", 500.0_f64),
            ("velocity", 4.0_f64),
        ] {
            let r = engine.query(&Predicate::And(vec![
                Predicate::node(),
                Predicate::Named(child.to_string()),
                Predicate::HasValue(ValuePredicate::NumEq(expected)),
            ]));
            assert!(
                !r.is_empty(),
                "joint0_limit child `{child}` = {expected} not found"
            );
        }
    }

    // =====================================================================
    // SECTION 3: Control / simulation hyperedges
    // =====================================================================

    #[test]
    fn gazebo_sim_system_edge_is_queryable() {
        let (store, compiled) = load_and_lower(MOVEO).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::And(vec![
            Predicate::edge(),
            Predicate::Named("gazebo_sim_system".to_string()),
        ]));
        assert_eq!(
            r.len(),
            1,
            "exactly one gazebo_sim_system hyperedge expected"
        );
    }

    #[test]
    fn joint_trajectory_controller_node_is_queryable() {
        let (store, compiled) = load_and_lower(MOVEO).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine
            .query(&Predicate::node().and(Predicate::inherits("joint_trajectory_controller")));
        assert!(!r.is_empty(), "joint_trajectory_controller node expected");
    }

    #[test]
    fn sim_plugin_has_expected_filename_value() {
        // `sim_control_plugin` declares `filename "gz_ros2_control-system";`
        let (store, compiled) = load_and_lower(MOVEO).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::And(vec![
            Predicate::node(),
            Predicate::Named("filename".to_string()),
            Predicate::HasValue(ValuePredicate::StrEq("gz_ros2_control-system".to_string())),
        ]));
        assert!(
            !r.is_empty(),
            "expected filename=\"gz_ros2_control-system\" somewhere in sim_control_plugin"
        );
    }

    // =====================================================================
    // SECTION 4: URDF / SDF generation via formats:: (full impl)
    // =====================================================================

    fn urdf_text() -> String {
        let (store, compiled) = load_and_lower(MOVEO).unwrap();
        generate_urdf(&compiled.ir, &store.it, ROBOT_NAME)
    }

    fn sdf_text() -> String {
        let (store, compiled) = load_and_lower(MOVEO).unwrap();
        generate_sdf(&compiled.ir, &store.it, ROBOT_NAME)
    }

    #[test]
    fn urdf_contains_every_expected_link() {
        let urdf = urdf_text();
        for link in EXPECTED_LINKS {
            assert!(
                urdf.contains(&format!("<link name=\"{link}\"")),
                "URDF missing link `{link}`"
            );
        }
    }

    #[test]
    fn urdf_contains_every_expected_joint() {
        let title = "urdf_contains_every_expected_joint";
        log_test_header(title, "All 7 joints appear in generate_urdf output.");
        let start = Instant::now();

        let urdf = urdf_text();
        info!(
            "URDF output: {} bytes, {} joint tags",
            urdf.len(),
            urdf.matches("<joint name=").count()
        );
        for joint in EXPECTED_JOINTS {
            assert!(
                urdf.contains(&format!("<joint name=\"{joint}\"")),
                "URDF missing joint `{joint}`"
            );
        }

        log_test_footer(
            title,
            Some(start.elapsed()),
            "all 7 expected joints emitted.",
        );
    }

    #[test]
    fn urdf_joint_types_split_fixed_vs_revolute() {
        let title = "urdf_joint_types_split_fixed_vs_revolute";
        log_test_header(
            title,
            "Counts type=\"fixed\" vs type=\"revolute\" in URDF output.",
        );
        let start = Instant::now();

        let urdf = urdf_text();
        let fixed = urdf.matches("type=\"fixed\"").count();
        let rev = urdf.matches("type=\"revolute\"").count();
        info!("URDF joint-type census: fixed={fixed}, revolute={rev}");

        assert_eq!(fixed, 1);
        assert_eq!(rev, 6);

        log_test_footer(
            title,
            Some(start.elapsed()),
            "1 fixed + 6 revolute in URDF.",
        );
    }

    #[test]
    fn sdf_reuses_revolute_type_for_all_non_fixed_joints() {
        // SDF 1.7 collapses continuous into revolute, but moveo has no
        // continuous joints — so the revolute count should be exactly 6.
        let sdf = sdf_text();
        let revolute_count = sdf.matches("type=\"revolute\"").count();
        assert_eq!(revolute_count, 6);
    }

    #[test]
    fn sdf_contains_every_expected_link() {
        let sdf = sdf_text();
        for link in EXPECTED_LINKS {
            assert!(
                sdf.contains(&format!("<link name=\"{link}\"")),
                "SDF missing link `{link}`"
            );
        }
    }

    #[test]
    fn urdf_and_sdf_agree_on_joint_count() {
        let urdf = urdf_text();
        let sdf = sdf_text();
        let urdf_j = urdf.matches("<joint name=").count();
        let sdf_j = sdf.matches("<joint name=").count();
        assert_eq!(urdf_j, sdf_j, "urdf={urdf_j} sdf={sdf_j}");
        assert_eq!(urdf_j, EXPECTED_JOINTS.len());
    }

    // =====================================================================
    // SECTION 5: MJCF + DOT generation via TransformRegistry (full impl)
    // =====================================================================

    fn emit_via_registry(name: &str) -> String {
        let m = model();
        let reg = hymeko_formats::default_registry();
        let t = reg
            .get(name)
            .unwrap_or_else(|| panic!("{name} not registered"));
        let cfg = TransformConfig::default().with_name(ROBOT_NAME);
        t.emit(&ModelView::Kinematic(m), &cfg)
            .expect("emit succeeds")
    }

    #[test]
    fn mjcf_body_hierarchy_matches_chain_depth() {
        let title = "mjcf_body_hierarchy_matches_chain_depth";
        log_test_header(
            title,
            "MJCF emits one <body> per link (±1 for optional world wrapper) and 6 hinge joints.",
        );
        let start = Instant::now();

        let mjcf = emit_via_registry("mjcf");
        info!(
            "MJCF: {} bytes, body-count={}, hinge-count={}, motor-count={}",
            mjcf.len(),
            mjcf.matches("<body name=").count(),
            mjcf.matches("type=\"hinge\"").count(),
            mjcf.matches("<motor").count(),
        );
        let body_count = mjcf.matches("<body name=").count();
        // The MJCF emitter may prepend a `<body name="world">` wrapper to
        // materialise the root frame into a body (MuJoCo does not have a
        // first-class "frame" primitive). We accept either `num_links` or
        // `num_links + 1` — the strict invariant is "≥ one body per link".
        assert!(
            body_count >= EXPECTED_LINKS.len(),
            "MJCF body count {} below expected floor {}",
            body_count,
            EXPECTED_LINKS.len()
        );
        assert!(
            body_count <= EXPECTED_LINKS.len() + 1,
            "MJCF body count {} above expected ceiling {} (world wrapper only)",
            body_count,
            EXPECTED_LINKS.len() + 1
        );
        let hinge_count = mjcf.matches("type=\"hinge\"").count();
        assert_eq!(hinge_count, 6, "expected 6 hinge joints");

        log_test_footer(
            title,
            Some(start.elapsed()),
            "MJCF body/hinge counts match chain depth.",
        );
    }

    #[test]
    fn mjcf_has_one_actuator_per_revolute_joint() {
        let mjcf = emit_via_registry("mjcf");
        let motor_count = mjcf.matches("<motor").count();
        assert_eq!(
            motor_count, 6,
            "expected 6 actuators (1 per revolute joint)"
        );
    }

    #[test]
    fn dot_has_one_edge_per_joint() {
        let title = "dot_has_one_edge_per_joint";
        log_test_header(
            title,
            "DOT output has one arrow per joint, with j_fix dashed.",
        );
        let start = Instant::now();

        let dot = emit_via_registry("dot");
        let arrow_count = dot.matches(" -> ").count();
        let dashed = dot.matches("style=dashed").count();
        info!(
            "DOT output: {} bytes, arrows={arrow_count}, dashed={dashed}",
            dot.len()
        );

        assert_eq!(arrow_count, EXPECTED_JOINTS.len());
        assert!(
            dashed >= 1,
            "j_fix (fixed joint) should have dashed styling"
        );

        log_test_footer(title, Some(start.elapsed()), "DOT graph topology correct.");
    }

    #[test]
    fn mjcf_validator_passes_for_moveo_serial_chain() {
        let m = model();
        let reg = hymeko_formats::default_registry();
        let mjcf = reg.get("mjcf").unwrap();
        let diags = mjcf.validate(&ModelView::Kinematic(m));
        let errors: Vec<_> = diags.iter().filter(|d| d.is_error()).collect();
        assert!(
            errors.is_empty(),
            "MJCF validation failed: {:?}",
            errors.iter().map(|d| &d.message).collect::<Vec<_>>()
        );
    }

    // =====================================================================
    // SECTION 6: Determinism — two runs produce identical output
    // =====================================================================

    #[test]
    fn urdf_generation_is_deterministic_across_runs() {
        let a = urdf_text();
        let b = urdf_text();
        assert_eq!(a, b, "URDF output must be byte-identical across two runs");
    }

    #[test]
    fn model_extraction_is_deterministic_across_runs() {
        let m1 = model();
        let m2 = model();
        assert_eq!(m1.links.len(), m2.links.len());
        assert_eq!(m1.joints.len(), m2.joints.len());
        for (a, b) in m1.joints.iter().zip(m2.joints.iter()) {
            assert_eq!(a.name, b.name);
            assert_eq!(a.joint_type, b.joint_type);
        }
    }
}
