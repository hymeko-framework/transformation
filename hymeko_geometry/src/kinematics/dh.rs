//! Serial standard-DH evaluation; topology/source projection belongs to the caller.
use super::joints::{DhError, JointEncoding, JointType};
use hymeko_clifford::Se3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DhChainError {
    CoordinateCount { joints: usize, coordinates: usize },
    Joint { index: usize, source: DhError },
    NonFiniteComposition { index: usize },
}

impl std::fmt::Display for DhChainError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::CoordinateCount {
                joints,
                coordinates,
            } => write!(
                f,
                "DH chain has {joints} joints but {coordinates} coordinates"
            ),
            Self::Joint { index, source } => write!(f, "DH joint {index}: {source}"),
            Self::NonFiniteComposition { index } => {
                write!(f, "DH chain composition at joint {index} is not finite")
            }
        }
    }
}

impl std::error::Error for DhChainError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Joint { source, .. } => Some(source),
            _ => None,
        }
    }
}

/// Evaluate a serial chain in base coordinates, returning `[base, child_1, ..., child_n]`.
///
/// Each local transform follows [`JointEncoding::standard_dh_transform`]:
/// `Rz(theta) Tz(d) Tx(a) Rx(alpha)` with column vectors, metres and radians.
/// Exactly one coordinate is required per joint, including fixed joints. An empty
/// chain returns only identity. Errors identify the zero-based failing joint;
/// no partial result escapes. Work/storage are linear in the caller's joint count.
///
/// Base placement and any tool offset remain separate transforms. This does not
/// reinterpret `KinematicModel`'s origin/axis joints or alter `FkPlan` semantics.
pub fn standard_dh_chain(
    joints: &[(JointEncoding, JointType)],
    coordinates: &[f64],
) -> Result<Vec<Se3>, DhChainError> {
    if joints.len() != coordinates.len() {
        return Err(DhChainError::CoordinateCount {
            joints: joints.len(),
            coordinates: coordinates.len(),
        });
    }
    let mut poses = Vec::with_capacity(joints.len() + 1);
    let mut parent = Se3::identity();
    poses.push(parent);
    for (index, ((encoding, kind), &q)) in joints.iter().zip(coordinates).enumerate() {
        let local = encoding
            .standard_dh_transform(*kind, q)
            .map_err(|source| DhChainError::Joint { index, source })?;
        parent = parent.compose(&local);
        let mut rotation = [0.; 9];
        parent.rotation_matrix_into(&mut rotation);
        if !parent
            .translation
            .iter()
            .chain(rotation.iter())
            .all(|v| v.is_finite())
        {
            return Err(DhChainError::NonFiniteComposition { index });
        }
        poses.push(parent);
    }
    Ok(poses)
}

/// Consecutive joint axes at least this close to parallel (about 1.1°), but not exactly
/// parallel, are collapsed by [`collapse_parallel_axis_offsets`]: a calibration tilts them
/// by milliradians, a designed skew by far more.
pub const PARALLEL_AXES_COS: f64 = 0.9998;

/// Where the joints of a [`standard_dh_chain`] result physically are, when consecutive joint
/// axes are nearly but not exactly parallel.
///
/// Standard DH puts frame `k` where the common normal of joint axes `k` and `k + 1` meets
/// axis `k + 1`. For exactly parallel axes that normal is free and the author's `d` places
/// it. For two nearly parallel axes it is forced arbitrarily far away: a controller's
/// calibrated DH (Universal Robots' among them) carries metre-scale `d` offsets on its
/// parallel shoulder, elbow and wrist axes, and `theta` offsets, that cancel along the
/// chain. The chain stays exact on every axis line and at its end, but its intermediate
/// frames lie far from the joints they belong to.
///
/// Every joint frame `k` in `1..poses.len() - 1` (the end frame is never moved) whose z
/// axis is within [`PARALLEL_AXES_COS`] of frame `k - 1`'s, and not parallel to it to 1e-12,
/// slides along its own z axis onto the plane through the already placed frame `k - 1`
/// origin normal to that frame's z axis. It then turns about its z axis so that x points
/// along the link from that origin again, with the sign of the DH `a` it was built with.
/// Axis lines, z axes and every other frame are unchanged, so a nominal chain (exactly
/// parallel axes, the author's `d`) comes back as it was. The cancelling `d` offsets of a
/// collapsed run collect at its last frame.
///
/// Kinematics (FK, IK, Jacobians) keep the chain as evaluated; this is where its joints are.
pub fn collapse_parallel_axis_offsets(poses: &mut [Se3]) {
    const X: [f64; 3] = [1., 0., 0.];
    const Z: [f64; 3] = [0., 0., 1.];
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let along =
        |a: [f64; 3], s: f64, b: [f64; 3]| [a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]];
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    for k in 1..poses.len().saturating_sub(1) {
        let (previous, pose) = (poses[k - 1], poses[k]);
        let (axis, z) = (previous.rotate(&Z), pose.rotate(&Z));
        let cos = dot(axis, z);
        if cos.abs() < PARALLEL_AXES_COS || 1. - cos.abs() <= 1e-12 {
            continue;
        }
        let base = previous.translation;
        // The link from the predecessor's origin; its part normal to the predecessor's axis
        // is `a x` of this frame (sliding that origin along its own axis does not change it).
        let link = along(pose.translation, -1., base);
        let x = pose.rotate(&X);
        let a = dot(along(link, -dot(link, axis), axis), x);
        let origin = along(pose.translation, -dot(link, axis) / cos, z);
        let placed = along(origin, -1., base);
        let towards = along(placed, -dot(placed, z), z);
        let length = dot(towards, towards).sqrt();
        let mut rotor = pose.rotor;
        // Coincident axes (no link) keep their orientation.
        if length > 1e-12 && a.abs() > 1e-12 {
            let target = towards.map(|v| a.signum() * v / length);
            let angle = dot(cross(x, target), z).atan2(dot(x, target));
            let rotation = Se3 {
                rotor,
                translation: [0.; 3],
            };
            rotor = Se3::axis_angle(z, angle).compose(&rotation).rotor;
        }
        poses[k] = Se3 {
            rotor,
            translation: origin,
        };
    }
}
