use hymeko::common::ids::DeclId;
use hymeko_clifford::Se3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointType {
    Fixed,
    Continuous,
    Revolute,
    Prismatic,
}

impl JointType {
    pub fn urdf_str(&self) -> &'static str {
        match self {
            Self::Fixed => "fixed",
            Self::Continuous => "continuous",
            Self::Revolute => "revolute",
            Self::Prismatic => "prismatic",
        }
    }

    pub fn sdf_str(&self) -> &'static str {
        match self {
            Self::Fixed => "fixed",
            Self::Continuous => "revolute", // SDF 1.7 has no continuous
            Self::Revolute => "revolute",
            Self::Prismatic => "prismatic",
        }
    }
}

#[derive(Debug, Clone)]
pub struct JointLimits {
    pub lower: f64,
    pub upper: f64,
    pub effort: f64,
    pub velocity: f64,
}

#[derive(Debug, Clone)]
pub struct JointInfo {
    pub did: DeclId,
    pub name: String,
    pub joint_type: JointType,
    /// The parent link's declaration, as the IR names it.
    ///
    /// This is the TOPOLOGY. `parent_link` beside it is the same fact spelled for a human, and the
    /// two are checked against each other by `tests/handles_match_names.rs` so they cannot drift.
    /// The distinction matters because the IR is a unified arena: `DeclId` is an index into it,
    /// resolving a name costs an interner lookup and an allocation, and doing that per edge on
    /// every kinematic query is how a zero-copy representation stops being one.
    pub parent: DeclId,
    /// The child link's declaration. See [`Self::parent`].
    pub child: DeclId,
    pub parent_link: String,
    pub child_link: String,
    pub axis: Option<[f64; 3]>,
    pub origin_xyz: Option<[f64; 3]>,
    pub origin_rpy_deg: Option<[f64; 3]>, // degrees from source
    pub limits: Option<JointLimits>,
}

impl JointInfo {
    /// Convert origin RPY from degrees to radians.
    pub fn origin_rpy_rad(&self) -> Option<[f64; 3]> {
        self.origin_rpy_deg.map(|rpy| {
            let deg2rad = std::f64::consts::PI / 180.0;
            [rpy[0] * deg2rad, rpy[1] * deg2rad, rpy[2] * deg2rad]
        })
    }
}

pub enum JointEncoding {
    /// RPY origin transform (current default, URDF-compatible)
    Rpy { xyz: [f64; 3], rpy: [f64; 3] },
    /// Standard Denavit-Hartenberg: theta/alpha in radians, d/a in metres.
    /// `theta` and `d` are the zero-coordinate offsets, before adding q.
    Dh {
        theta: f64,
        d: f64,
        a: f64,
        alpha: f64,
    },
    /// Reserved Hayati-Roberts parameter record; its convention is not yet evaluated.
    Hayati {
        theta: f64,
        beta: f64,
        a: f64,
        alpha: f64,
        d: f64,
    },
    /// Product of Exponentials / Screw theory (6-vector twist)
    Poe { twist: [f64; 6] },
}

/// An invalid input to the standard-DH numerical contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DhError {
    /// Only `JointEncoding::Dh` has the factor ordering this entry point evaluates.
    NotStandardDh,
    /// A geometric parameter or the resulting offset-plus-coordinate is not finite.
    NonFiniteParameter(&'static str),
    /// Runtime coordinates must be finite, including for a fixed joint.
    NonFiniteCoordinate,
    /// Finite inputs exceeded the numerical range of the rigid-transform operations.
    NonFiniteTransform,
}

impl std::fmt::Display for DhError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotStandardDh => write!(f, "expected the standard DH encoding"),
            Self::NonFiniteParameter(name) => write!(f, "DH parameter {name} must be finite"),
            Self::NonFiniteCoordinate => write!(f, "DH joint coordinate must be finite"),
            Self::NonFiniteTransform => {
                write!(f, "DH transform exceeded the finite numerical range")
            }
        }
    }
}

impl std::error::Error for DhError {}

impl JointEncoding {
    /// Evaluate standard DH as `Rz(theta) Tz(d) Tx(a) Rx(alpha)` using Clifford SE(3).
    ///
    /// Column-vector convention: `p_parent = T_parent_child.apply(p_child)`.
    /// Revolute/continuous adds `q` (radians) to theta; prismatic adds `q` (metres)
    /// to d. Fixed uses the stored offsets, ignoring a finite q. Angles are never
    /// interpreted as degrees. The source projection is responsible for unit conversion.
    ///
    /// This joint-local primitive does not read source, build topology, or change
    /// the origin/axis model used by `FkPlan`, dynamics and format exporters.
    /// Other encoding variants are rejected rather than reinterpreted as DH.
    pub fn standard_dh_transform(&self, kind: JointType, q: f64) -> Result<Se3, DhError> {
        let Self::Dh { theta, d, a, alpha } = *self else {
            return Err(DhError::NotStandardDh);
        };
        for (name, value) in [("theta", theta), ("d", d), ("a", a), ("alpha", alpha)] {
            if !value.is_finite() {
                return Err(DhError::NonFiniteParameter(name));
            }
        }
        if !q.is_finite() {
            return Err(DhError::NonFiniteCoordinate);
        }
        let theta = theta
            + if matches!(kind, JointType::Revolute | JointType::Continuous) {
                q
            } else {
                0.0
            };
        let d = d + if kind == JointType::Prismatic { q } else { 0.0 };
        for (name, value) in [("theta", theta), ("d", d)] {
            if !value.is_finite() {
                return Err(DhError::NonFiniteParameter(name));
            }
        }
        let pose = Se3::axis_angle([0.0, 0.0, 1.0], theta)
            .compose(&Se3::axis_translation([0.0, 0.0, 1.0], d))
            .compose(&Se3::axis_translation([1.0, 0.0, 0.0], a))
            .compose(&Se3::axis_angle([1.0, 0.0, 0.0], alpha));
        let mut rotation = [0.0; 9];
        pose.rotation_matrix_into(&mut rotation);
        if !pose
            .translation
            .iter()
            .chain(rotation.iter())
            .all(|value| value.is_finite())
        {
            return Err(DhError::NonFiniteTransform);
        }
        Ok(pose)
    }
}

// ============================================================
// Joint topology extraction
// ============================================================

pub struct JointTopology {
    parent_link: String,
    child_link: String,
    axis: Option<[f64; 3]>,
    origin_xyz: Option<[f64; 3]>,
    origin_rpy: Option<[f64; 3]>,
}
