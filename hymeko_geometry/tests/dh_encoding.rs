//! Joint-local DH contract, independent of the source projection and origin/axis FK.
use std::f64::consts::{FRAC_PI_2, PI};

use hymeko_clifford::Se3;
use hymeko_geometry::kinematics::joints::{DhError, JointEncoding, JointType};

fn close(actual: &[f64], expected: &[f64]) {
    assert_eq!(actual.len(), expected.len());
    for (a, e) in actual.iter().zip(expected) {
        assert!((a - e).abs() < 1e-12, "{actual:?} != {expected:?}");
    }
}

fn matrix(pose: &Se3) -> [f64; 16] {
    let mut rotation = [0.0; 9];
    pose.rotation_matrix_into(&mut rotation);
    let [x, y, z] = pose.translation;
    [
        rotation[0],
        rotation[1],
        rotation[2],
        x,
        rotation[3],
        rotation[4],
        rotation[5],
        y,
        rotation[6],
        rotation[7],
        rotation[8],
        z,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

// Closed-form homogeneous matrix, not a second product of the implementation's
// Clifford factors. Standard-DH convention: Robotics Toolbox DHLink.A reference.
fn closed_form(theta: f64, d: f64, a: f64, alpha: f64) -> [f64; 16] {
    let (st, ct) = theta.sin_cos();
    let (sa, ca) = alpha.sin_cos();
    [
        ct,
        -st * ca,
        st * sa,
        a * ct,
        st,
        ct * ca,
        -ct * sa,
        a * st,
        0.0,
        sa,
        ca,
        d,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

fn unit_link() -> JointEncoding {
    JointEncoding::Dh {
        theta: 0.0,
        d: 0.0,
        a: 1.0,
        alpha: 0.0,
    }
}

#[test]
fn spatial_offsets_match_the_homogeneous_matrix_for_every_joint_kind() {
    let (theta, d, a, alpha) = (0.37, 0.27, 0.81, -0.63);
    let encoding = JointEncoding::Dh { theta, d, a, alpha };
    for q in [-0.91, 0.0, 0.42] {
        for kind in [
            JointType::Fixed,
            JointType::Revolute,
            JointType::Continuous,
            JointType::Prismatic,
        ] {
            let (t, offset) = match kind {
                JointType::Fixed => (theta, d),
                JointType::Revolute | JointType::Continuous => (theta + q, d),
                JointType::Prismatic => (theta, d + q),
            };
            let pose = encoding.standard_dh_transform(kind, q).unwrap();
            close(&matrix(&pose), &closed_form(t, offset, a, alpha));
            // Apply to a non-origin point as well: orientation and translation
            // jointly obey p_parent = T_parent_child * p_child.
            let expected = closed_form(t, offset, a, alpha);
            let p = [0.2, -0.4, 0.7];
            let target = [0, 4, 8].map(|r| {
                expected[r] * p[0]
                    + expected[r + 1] * p[1]
                    + expected[r + 2] * p[2]
                    + expected[r + 3]
            });
            close(&pose.apply(&p), &target);
        }
    }
}

#[test]
fn rotation_precedes_link_translation_in_the_planar_arm() {
    // These are the Robotics acceptance configurations. In particular q1=pi/2
    // must rotate both translations, which origin(Tx(a)) * motion(Rz(q)) cannot do.
    for (q, position, angle) in [
        ([0.0, 0.0], [2.0, 0.0, 0.0], 0.0),
        ([FRAC_PI_2, 0.0], [0.0, 2.0, 0.0], FRAC_PI_2),
        ([0.0, FRAC_PI_2], [1.0, 1.0, 0.0], FRAC_PI_2),
        ([FRAC_PI_2, -FRAC_PI_2], [1.0, 1.0, 0.0], 0.0),
    ] {
        let first = unit_link()
            .standard_dh_transform(JointType::Revolute, q[0])
            .unwrap();
        let second = unit_link()
            .standard_dh_transform(JointType::Revolute, q[1])
            .unwrap();
        let pose = first.compose(&second);
        close(&pose.translation, &position);
        let mut want = closed_form(angle, 0.0, 0.0, 0.0);
        want[3] = position[0];
        want[7] = position[1];
        want[11] = position[2];
        close(&matrix(&pose), &want);
    }
}

#[test]
fn prismatic_coordinate_translates_along_parent_z_before_the_twist() {
    let encoding = JointEncoding::Dh {
        theta: FRAC_PI_2,
        d: 0.2,
        a: 0.5,
        alpha: FRAC_PI_2,
    };
    let pose = encoding
        .standard_dh_transform(JointType::Prismatic, 0.3)
        .unwrap();
    close(&pose.translation, &[0.0, 0.5, 0.5]);
    // Child z points along parent x. Sliding along child z would give the wrong pose.
    close(&pose.apply(&[0.0, 0.0, 1.0]), &[1.0, 0.5, 0.5]);
}

#[test]
fn base_and_tool_frames_compose_without_changing_dh_parameters() {
    let base = Se3::rpy_xyz([0.0, 0.0, FRAC_PI_2], [1.0, 2.0, 3.0]);
    let tool = Se3::rpy_xyz([0.0, FRAC_PI_2, 0.0], [0.25, 0.0, 0.0]);
    let first = unit_link()
        .standard_dh_transform(JointType::Revolute, FRAC_PI_2)
        .unwrap();
    let second = unit_link()
        .standard_dh_transform(JointType::Revolute, 0.0)
        .unwrap();
    let pose = base.compose(&first).compose(&second).compose(&tool);
    close(
        &matrix(&pose),
        &[
            0.0, 0.0, -1.0, -1.25, 0.0, -1.0, 0.0, 2.0, -1.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0,
        ],
    );
}

#[test]
fn fixed_joint_keeps_offsets_and_input_encoding_is_unchanged() {
    let encoding = JointEncoding::Dh {
        theta: 0.3,
        d: 0.2,
        a: -0.7,
        alpha: PI,
    };
    let pose = encoding
        .standard_dh_transform(JointType::Fixed, 4.0)
        .unwrap();
    close(&matrix(&pose), &closed_form(0.3, 0.2, -0.7, PI));
    assert!(matches!(
        encoding,
        JointEncoding::Dh {
            theta: 0.3,
            d: 0.2,
            a: -0.7,
            alpha: PI
        }
    ));
}

#[test]
fn invalid_inputs_and_coordinate_overflow_are_rejected() {
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for (slot, name) in ["theta", "d", "a", "alpha"].into_iter().enumerate() {
            let mut parameters = [0.0; 4];
            parameters[slot] = bad;
            let [theta, d, a, alpha] = parameters;
            let error = JointEncoding::Dh { theta, d, a, alpha }
                .standard_dh_transform(JointType::Revolute, 0.0)
                .unwrap_err();
            assert_eq!(error, DhError::NonFiniteParameter(name));
        }
        for kind in [JointType::Revolute, JointType::Prismatic, JointType::Fixed] {
            assert_eq!(
                unit_link().standard_dh_transform(kind, bad).unwrap_err(),
                DhError::NonFiniteCoordinate
            );
        }
    }
    let angular = JointEncoding::Dh {
        theta: f64::MAX,
        d: 0.0,
        a: 0.0,
        alpha: 0.0,
    };
    assert_eq!(
        angular
            .standard_dh_transform(JointType::Revolute, f64::MAX)
            .unwrap_err(),
        DhError::NonFiniteParameter("theta")
    );
    let linear = JointEncoding::Dh {
        theta: 0.0,
        d: f64::MAX,
        a: 0.0,
        alpha: 0.0,
    };
    assert_eq!(
        linear
            .standard_dh_transform(JointType::Prismatic, f64::MAX)
            .unwrap_err(),
        DhError::NonFiniteParameter("d")
    );
}

#[test]
fn other_parameterizations_never_silently_become_standard_dh() {
    for encoding in [
        JointEncoding::Rpy {
            xyz: [0.0; 3],
            rpy: [0.0; 3],
        },
        JointEncoding::Hayati {
            theta: 0.0,
            beta: 0.0,
            a: 1.0,
            alpha: 0.0,
            d: 0.0,
        },
        JointEncoding::Poe { twist: [0.0; 6] },
    ] {
        assert_eq!(
            encoding
                .standard_dh_transform(JointType::Revolute, 0.0)
                .unwrap_err(),
            DhError::NotStandardDh
        );
    }
}
