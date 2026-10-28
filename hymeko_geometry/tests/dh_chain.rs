use hymeko_clifford::Se3;
use hymeko_geometry::kinematics::{
    dh::{DhChainError, collapse_parallel_axis_offsets, standard_dh_chain},
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

/// UR5e DH columns `(theta, d, a, alpha)`: nominal, and a controller's calibrated set
/// (xrobotassist's UR5e on 2026-10-28, Primary KinematicsInfo, calibration_status 2) with
/// the joint positions and flange pose its RTDE reported at the same time.
const NOMINAL: [[f64; 4]; 6] = [
    [0., 0.1625, 0., FRAC_PI_2],
    [0., 0., -0.425, 0.],
    [0., 0., -0.3922, 0.],
    [0., 0.1333, 0., FRAC_PI_2],
    [0., 0.0997, 0., -FRAC_PI_2],
    [0., 0.0996, 0., 0.],
];
const CALIBRATED: [[f64; 4]; 6] = [
    [
        4.2085548974599152e-08,
        0.16261989814306138,
        9.0783842152673187e-05,
        1.5693752622876662,
    ],
    [
        0.80794890416645548,
        128.90512876239504,
        -0.29414778590159774,
        0.0023872201154912088,
    ],
    [
        5.1496627883043233,
        -157.91966810293744,
        -0.3718926669748438,
        0.0043269737651047309,
    ],
    [
        0.32557472321549807,
        29.148025977020971,
        -5.7820578328018618e-05,
        1.5704894333087369,
    ],
    [
        1.5656742338374108e-06,
        0.099831356352270348,
        7.412683600236342e-05,
        -1.5694640836671627,
    ],
    [7.7788076592935629e-08, 0.099790117186186036, 0., 0.],
];
const Q: [f64; 6] = [
    -1.6232636610614222,
    -2.1971785030760707,
    -0.86497622728347778,
    -1.0553791087916871,
    1.5313825607299805,
    0.014752333983778954,
];
const FLANGE: [f64; 3] = [
    -0.1790182062238761,
    -0.77111191392562639,
    0.51274749427834598,
];

fn ur5e(columns: &[[f64; 4]; 6]) -> Vec<Se3> {
    let chain: Vec<_> = columns
        .iter()
        .map(|&[t, d, a, r]| dh(t, d, a, r, JointType::Revolute))
        .collect();
    standard_dh_chain(&chain, &Q).unwrap()
}
fn axis(pose: &Se3, i: usize) -> [f64; 3] {
    let mut e = [0.; 3];
    e[i] = 1.;
    pose.rotate(&e)
}
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|i| (a[i] - b[i]).powi(2)).sum::<f64>().sqrt()
}

#[test]
fn collapse_returns_chains_without_offsets_in_parallel_runs_as_they_were() {
    let nominal = ur5e(&NOMINAL);
    let mut collapsed = nominal.clone();
    collapse_parallel_axis_offsets(&mut collapsed);
    for (before, after) in nominal.iter().zip(&collapsed) {
        close(&after.translation, &before.translation);
        for i in 0..3 {
            close(&axis(after, i), &axis(before, i));
        }
    }
    let planar = [
        dh(0.3, 0., 1., 0., JointType::Revolute),
        dh(-0.2, 0., 0.5, 0., JointType::Revolute),
    ];
    let poses = standard_dh_chain(&planar, &[0.1, 0.4]).unwrap();
    let mut collapsed = poses.clone();
    collapse_parallel_axis_offsets(&mut collapsed);
    for (before, after) in poses.iter().zip(&collapsed) {
        close(&after.translation, &before.translation);
        close(&axis(after, 0), &axis(before, 0));
    }
    for mut short in [vec![], vec![Se3::identity()], vec![Se3::identity(); 2]] {
        collapse_parallel_axis_offsets(&mut short);
        assert!(short.iter().all(|p| p.translation == [0.; 3]));
    }
}

#[test]
fn collapse_places_a_calibrated_ur5e_at_its_joints() {
    let raw = ur5e(&CALIBRATED);
    // The controller's own flange pose: the calibrated chain itself is exact at its end ...
    assert!(
        distance(raw[6].translation, FLANGE) < 1e-6,
        "{:?}",
        raw[6].translation
    );
    // ... while its elbow frame lies about 129 m down the parallel shoulder axis.
    assert!(raw[2].translation.iter().any(|v| v.abs() > 10.));
    let mut placed = raw.clone();
    collapse_parallel_axis_offsets(&mut placed);
    let nominal = ur5e(&NOMINAL);
    for k in 0..raw.len() {
        // Every axis line is kept: the origin moves along the frame's own z, z itself is kept.
        let shift: [f64; 3] =
            std::array::from_fn(|i| placed[k].translation[i] - raw[k].translation[i]);
        let z = axis(&raw[k], 2);
        let off_axis = [
            shift[1] * z[2] - shift[2] * z[1],
            shift[2] * z[0] - shift[0] * z[2],
            shift[0] * z[1] - shift[1] * z[0],
        ];
        assert!(
            off_axis.iter().all(|v| v.abs() < 1e-9),
            "frame {k} left its axis: {off_axis:?}"
        );
        assert!(
            distance(axis(&placed[k], 2), z) < 1e-12,
            "frame {k} z turned"
        );
        // And the joints sit where the nominal arm has them, to the calibration's millimetres.
        assert!(
            distance(placed[k].translation, nominal[k].translation) < 5e-3,
            "frame {k}: {:?} vs nominal {:?}",
            placed[k].translation,
            nominal[k].translation
        );
        assert!(
            distance(axis(&placed[k], 0), axis(&nominal[k], 0)) < 1e-2,
            "frame {k} x"
        );
    }
    for k in [0, 1, 4, 5, 6] {
        assert_eq!(
            placed[k].translation, raw[k].translation,
            "frame {k} is outside a parallel run"
        );
        assert_eq!(placed[k].rotor, raw[k].rotor);
    }
}
