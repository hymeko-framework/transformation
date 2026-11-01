#[path = "assembly/fixture.rs"]
mod fixture;

use fixture::{Fixture, SOURCE};
use hymeko::{ir::ir::ValueR, module_store::execution::ValueUpdate};
use hymeko_clifford::Se3;
use hymeko_geometry::kinematics::{
    assembly::{AssemblyError, StateError},
    kinematic::scope::{ScopedModelError, extract_scoped_model},
};

struct Poses;
impl Poses {
    fn near(actual: [f64; 3], expected: [f64; 3]) {
        assert!(
            actual
                .iter()
                .zip(expected)
                .all(|(a, b)| (*a - b).abs() < 1e-10),
            "{actual:?} != {expected:?}"
        );
    }
}

#[test]
fn one_ir_binds_arm_door_and_slider_through_the_same_facade() {
    let fixture = Fixture::new(SOURCE);
    let original = fixture.hash();
    let mut arm = fixture.read("room.arm").unwrap();
    let mut door = fixture.read("room.door").unwrap();
    let mut slider = fixture.read("room.slider").unwrap();
    assert_eq!(slider.scope(), fixture.id("room.slider"));
    assert_eq!(
        (
            arm.model().links.len(),
            door.model().links.len(),
            slider.model().links.len()
        ),
        (3, 2, 2)
    );
    assert_eq!(
        (
            arm.coordinates().len(),
            door.coordinates().len(),
            slider.coordinates().len()
        ),
        (3, 2, 2)
    );
    assert_eq!(
        arm.model()
            .frames
            .iter()
            .filter(|name| name.as_str() == "world")
            .count(),
        1,
        "an external world frame is an anchor, not another body"
    );
    let mut q = arm.coordinates().to_vec();
    q[arm.joint_index(fixture.id("room.arm.joint1")).unwrap()] = std::f64::consts::FRAC_PI_2;
    arm.set_state(&q, Se3::identity()).unwrap();
    Poses::near(
        arm.link_pose(fixture.id("room.arm.tip"))
            .unwrap()
            .translation,
        [1., 1., 0.],
    );
    let tool = arm.attachment_pose(fixture.id("room.arm.tool")).unwrap();
    Poses::near(tool.translation, [1., 1.25, 0.]);
    Poses::near(tool.rotate(&[1., 0., 0.]), [-1., 0., 0.]);
    Poses::near(
        door.attachment_pose(fixture.id("room.door.tool"))
            .unwrap()
            .translation,
        [1., 0., 0.],
    );
    Poses::near(
        slider
            .link_pose(fixture.id("room.slider.tip"))
            .unwrap()
            .translation,
        [0.; 3],
    );

    for (group, assembly, value) in [
        ("door", &mut door, std::f64::consts::FRAC_PI_2),
        ("slider", &mut slider, 0.4),
    ] {
        let mut q = assembly.coordinates().to_vec();
        q[assembly
            .joint_index(fixture.id(&format!("room.{group}.joint1")))
            .unwrap()] = value;
        assembly.set_state(&q, Se3::identity()).unwrap();
    }
    Poses::near(
        door.attachment_pose(fixture.id("room.door.tool"))
            .unwrap()
            .translation,
        [0., 1., 0.],
    );
    Poses::near(
        slider
            .attachment_pose(fixture.id("room.slider.tool"))
            .unwrap()
            .translation,
        [0.6, 0., 0.],
    );
    for (assembly, tip, expected) in [
        (&door, "room.door.tip", [0., 0.5, 0.]),
        (&slider, "room.slider.tip", [0.5, 0., 0.]),
    ] {
        let tip = fixture.id(tip);
        let visual = assembly
            .visuals()
            .iter()
            .position(|visual| visual.did == tip)
            .unwrap();
        Poses::near(assembly.visual_poses()[visual].translation, expected);
        let body = assembly
            .model()
            .links
            .iter()
            .position(|link| link.did == tip)
            .unwrap();
        let collider = assembly
            .colliders()
            .iter()
            .position(|collider| collider.link == body)
            .unwrap();
        Poses::near(assembly.collider_placements()[collider].centre, expected);
    }
    assert!(
        door.link_pose(fixture.id("room.arm.tip")).is_none(),
        "equal local names never merge canonical identities"
    );
    assert_eq!(
        fixture.hash(),
        original,
        "posing does not mutate or replace canonical IR"
    );
}

#[test]
fn scoped_model_and_initial_coordinate_references_use_canonical_identity() {
    let fixture = Fixture::new(&SOURCE.replace("slider 0;", "slider 0.35;"));
    let scope = fixture.id("room.slider");
    let model = extract_scoped_model(&fixture.query(), scope).unwrap();
    assert_eq!(model.links.len(), 2);
    assert!(model.links.iter().all(|link| {
        [
            fixture.id("room.slider.base"),
            fixture.id("room.slider.tip"),
        ]
        .contains(&link.did)
    }));
    let assembly = fixture.read("room.slider").unwrap();
    assert!(
        assembly
            .joint_index(fixture.id("room.arm.joint1"))
            .is_none()
    );
    assert_eq!(
        assembly.coordinates()[assembly
            .joint_index(fixture.id("room.slider.joint1"))
            .unwrap()],
        0.35
    );
    Poses::near(
        assembly
            .attachment_pose(fixture.id("room.slider.tool"))
            .unwrap()
            .translation,
        [0.55, 0., 0.],
    );
    assert!(
        assembly
            .attachment_pose(fixture.id("room.arm.tool"))
            .is_none()
    );
}

#[test]
fn root_placement_and_numeric_leaf_deltas_keep_all_bindings_aligned() {
    let fixture = Fixture::new(SOURCE);
    let mut assembly = fixture.read("room.arm").unwrap();
    let visual_buffer = assembly.visual_poses().as_ptr();
    let collider_buffer = assembly.collider_placements().as_ptr();
    let update = ValueUpdate::Set {
        declaration: fixture.id("room.samples.arm1"),
        value: ValueR::Num(std::f64::consts::FRAC_PI_2),
    };
    assert!(
        assembly
            .apply_values(std::slice::from_ref(&update))
            .unwrap()
    );
    let q = assembly.coordinates().to_vec();
    let root = Se3::rpy_xyz([0., 0., std::f64::consts::FRAC_PI_2], [5., 2., 0.]);
    assembly.set_state(&q, root).unwrap();
    Poses::near(
        assembly
            .link_pose(fixture.id("room.arm.tip"))
            .unwrap()
            .translation,
        [4., 3., 0.],
    );
    Poses::near(
        assembly
            .attachment_pose(fixture.id("room.arm.tool"))
            .unwrap()
            .translation,
        [3.75, 3., 0.],
    );
    assert!(
        !assembly
            .apply_values(std::slice::from_ref(&update))
            .unwrap(),
        "unchanged checkpoint values do not reset the pose"
    );
    assert!(
        !assembly
            .apply_values(&[ValueUpdate::Set {
                declaration: fixture.id("room.samples.unrelated"),
                value: ValueR::Num(33.)
            }])
            .unwrap()
    );
    Poses::near(
        assembly
            .attachment_pose(fixture.id("room.arm.tool"))
            .unwrap()
            .translation,
        [3.75, 3., 0.],
    );
    assert_eq!(assembly.visual_poses().as_ptr(), visual_buffer);
    assert_eq!(assembly.collider_placements().as_ptr(), collider_buffer);
}

#[test]
fn refused_samples_leave_the_accepted_pose_unchanged() {
    let fixture = Fixture::new(SOURCE);
    let mut assembly = fixture.read("room.arm").unwrap();
    let original = assembly
        .attachment_pose(fixture.id("room.arm.tool"))
        .unwrap()
        .translation;
    assert!(matches!(
        assembly.set_state(&[], Se3::identity()),
        Err(StateError::JointCount { .. })
    ));
    let mut q = assembly.coordinates().to_vec();
    q[1] = f64::NAN;
    assert!(matches!(
        assembly.set_state(&q, Se3::identity()),
        Err(StateError::NonFiniteCoordinate { .. })
    ));
    let mut root = Se3::identity();
    root.translation[0] = f64::INFINITY;
    assert_eq!(
        assembly.set_state(&[0.; 3], root),
        Err(StateError::InvalidPlacement)
    );
    let bad = fixture.id("room.samples.arm2");
    assert_eq!(
        assembly.apply_values(&[
            ValueUpdate::Set {
                declaration: fixture.id("room.samples.arm1"),
                value: ValueR::Num(1.)
            },
            ValueUpdate::Text {
                declaration: bad,
                value: "bad"
            },
        ]),
        Err(StateError::InvalidCoordinateSample { declaration: bad })
    );
    Poses::near(
        assembly
            .attachment_pose(fixture.id("room.arm.tool"))
            .unwrap()
            .translation,
        original,
    );
    assert!(assembly.coordinates().iter().all(|q| *q == 0.));
}

#[test]
fn group_boundaries_and_invalid_attachment_metadata_are_refused() {
    let crossed = Fixture::new(&SOURCE.replace(
        "(+ base, - tip, - k.axes.AXIS_X)",
        "(+ room.arm.base, - tip, - k.axes.AXIS_X)",
    ));
    assert!(matches!(
        crossed.read("room.slider"),
        Err(AssemblyError::Model(ScopedModelError::OutsideGroup { .. }))
    ));
    for (from, to) in [
        (
            "(+ tip, - tool); origin [0.25,0,0]",
            "(+ tip, - room.door.tool); origin [0.25,0,0]",
        ),
        ("origin [0.25,0,0]", "origin [0.25,0]"),
        (
            "attachment_kind -> k.elements.frame_attachment",
            "attachment_kind -> world",
        ),
    ] {
        let fixture = Fixture::new(&SOURCE.replace(from, to));
        assert!(matches!(
            fixture.read("room.arm"),
            Err(AssemblyError::Binding(_))
        ));
    }
}

#[test]
fn assembly_binding_caps_are_enforced() {
    let mut bodies = String::from("description {} types { link {} } group {");
    for index in 0..1025 {
        bodies.push_str(&format!("b{index}: types.link {{}}\n"));
    }
    bodies.push('}');
    assert!(
        matches!(Fixture::new(&bodies).read("group"), Err(AssemblyError::Binding(message)) if message.contains("1024 links/joints"))
    );

    let mut frames = String::from(
        "description {} types { link {} frame {} @frame_attachment {} } group { attachment_kind -> types.frame_attachment; body: types.link {}",
    );
    for index in 0..1025 {
        frames.push_str(&format!(
            "f{index}: types.frame {{}} @m{index}: types.frame_attachment {{ (+ body, - f{index}); }}\n"
        ));
    }
    frames.push('}');
    let error = Fixture::new(&frames)
        .read("group")
        .err()
        .expect("over-cap frame bindings are refused");
    assert!(
        matches!(&error, AssemblyError::Binding(message) if message.contains("1024 frame attachments")),
        "{error}"
    );
}
