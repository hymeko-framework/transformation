use hymeko::common::ids::DeclId;


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
            Self::Fixed      => "fixed",
            Self::Continuous => "continuous",
            Self::Revolute   => "revolute",
            Self::Prismatic  => "prismatic",
        }
    }

    pub fn sdf_str(&self) -> &'static str {
        match self {
            Self::Fixed      => "fixed",
            Self::Continuous => "revolute", // SDF 1.7 has no continuous
            Self::Revolute   => "revolute",
            Self::Prismatic  => "prismatic",
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
    pub parent_link: String,
    pub child_link: String,
    pub axis: Option<[f64; 3]>,
    pub origin_xyz: Option<[f64; 3]>,
    pub origin_rpy_deg: Option<[f64; 3]>,  // degrees from source
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
    /// Denavit-Hartenberg (4 params)
    Dh { theta: f64, d: f64, a: f64, alpha: f64 },
    /// Hayati-Roberts (5 params, parallel-axis safe)
    Hayati { theta: f64, beta: f64, a: f64, alpha: f64, d: f64 },
    /// Product of Exponentials / Screw theory (6-vector twist)
    Poe { twist: [f64; 6] },
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