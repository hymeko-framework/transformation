use hymeko_geometry::kinematics::{
    dh::{DhChainError, standard_dh_chain},
    joints::{DhError, JointEncoding, JointType},
};
use std::f64::consts::FRAC_PI_2;

fn dh(theta: f64, d: f64, a: f64, alpha: f64, kind: JointType) -> (JointEncoding, JointType) {
    (JointEncoding::Dh { theta, d, a, alpha }, kind)
}
fn close(actual: &[f64], expected: &[f64]) {
    for (a, e) in actual.iter().zip(expected) {
        assert!((a - e).abs() < 1e-12, "{actual:?} != {expected:?}");
    }
}

#[test]
fn planar_chain_returns_every_frame_in_base_coordinates() {
    let chain = [
        dh(0., 0., 1., 0., JointType::Revolute),
        dh(0., 0., 1., 0., JointType::Continuous),
    ];
    for (q, end) in [
        ([0., 0.], [2., 0., 0.]),
        ([FRAC_PI_2, 0.], [0., 2., 0.]),
        ([0., FRAC_PI_2], [1., 1., 0.]),
        ([FRAC_PI_2, -FRAC_PI_2], [1., 1., 0.]),
    ] {
        let poses = standard_dh_chain(&chain, &q).unwrap();
        assert_eq!(poses.len(), 3);
        close(&poses[0].translation, &[0.; 3]);
        close(&poses[1].translation, &[q[0].cos(), q[0].sin(), 0.]);
        close(&poses[2].translation, &end);
    }
    assert_eq!(standard_dh_chain(&[], &[]).unwrap().len(), 1);
}

#[test]
fn spatial_chain_matches_independent_homogeneous_matrix_products() {
    // Closed-form DH matrices and generic 4x4 products, independent of Clifford composition.
    let parameters = [
        (0.37, 0.27, -0.81, -0.63, JointType::Revolute),
        (-0.1, 0.2, 0.4, FRAC_PI_2, JointType::Prismatic),
        (0.6, -0.3, 0.2, -FRAC_PI_2, JointType::Fixed),
    ];
    let chain: Vec<_> = parameters
        .iter()
        .map(|&(t, d, a, r, k)| dh(t, d, a, r, k))
        .collect();
    let q = [0.42, 0.15, 123.];
    let poses = standard_dh_chain(&chain, &q).unwrap();
    let mut expected = [[0.; 4]; 4];
    for (i, row) in expected.iter_mut().enumerate() {
        row[i] = 1.;
    }
    for (index, &(t, d, a, alpha, kind)) in parameters.iter().enumerate() {
        let theta = t + if kind == JointType::Revolute {
            q[index]
        } else {
            0.
        };
        let d = d + if kind == JointType::Prismatic {
            q[index]
        } else {
            0.
        };
        let (s, c) = theta.sin_cos();
        let (sa, ca) = alpha.sin_cos();
        let local = [
            [c, -s * ca, s * sa, a * c],
            [s, c * ca, -c * sa, a * s],
            [0., sa, ca, d],
            [0., 0., 0., 1.],
        ];
        let mut product = [[0.; 4]; 4];
        for (r, row) in product.iter_mut().enumerate() {
            for (c, v) in row.iter_mut().enumerate() {
                *v = (0..4).map(|k| expected[r][k] * local[k][c]).sum();
            }
        }
        expected = product;
        for point in [[0., 0., 0.], [0.2, -0.4, 0.7]] {
            let want: [f64; 3] = std::array::from_fn(|r| {
                expected[r][3] + (0..3).map(|c| expected[r][c] * point[c]).sum::<f64>()
            });
            close(&poses[index + 1].apply(&point), &want);
        }
    }
}

#[test]
fn invalid_cardinality_joint_and_accumulation_are_checked() {
    let chain = [dh(0., 0., 1., 0., JointType::Revolute)];
    assert_eq!(
        standard_dh_chain(&chain, &[]).unwrap_err(),
        DhChainError::CoordinateCount {
            joints: 1,
            coordinates: 0
        }
    );
    assert!(matches!(
        standard_dh_chain(&chain, &[f64::NAN]),
        Err(DhChainError::Joint {
            index: 0,
            source: DhError::NonFiniteCoordinate
        })
    ));
    let unsupported = [(JointEncoding::Poe { twist: [0.; 6] }, JointType::Fixed)];
    assert!(matches!(
        standard_dh_chain(&unsupported, &[0.]),
        Err(DhChainError::Joint {
            index: 0,
            source: DhError::NotStandardDh
        })
    ));
    let huge = [
        dh(0., 0., f64::MAX * 0.75, 0., JointType::Fixed),
        dh(0., 0., f64::MAX * 0.75, 0., JointType::Fixed),
    ];
    assert_eq!(
        standard_dh_chain(&huge, &[0., 0.]).unwrap_err(),
        DhChainError::NonFiniteComposition { index: 1 }
    );
}
