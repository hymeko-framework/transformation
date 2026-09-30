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
