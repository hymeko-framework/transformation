use super::super::{Identity, Refusal, RefusalCode};
use hymeko::common::ids::DeclId;
use serde::{Deserialize, Serialize};

/// Metric pose in the single authored observation frame; XYZW quaternion.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Pose {
    pub position_m: [f64; 3],
    pub quaternion_xyzw: [f64; 4],
}
impl Pose {
    pub(super) fn validate(&self) -> Result<(), Refusal> {
        let norm = self.quaternion_xyzw.iter().map(|x| x * x).sum::<f64>();
        if self
            .position_m
            .iter()
            .any(|x| !x.is_finite() || x.abs() > 100.)
            || self.quaternion_xyzw.iter().any(|x| !x.is_finite())
            || (norm - 1.).abs() > 1e-6
        {
            return Err(Refusal::new(
                RefusalCode::InvalidRequest,
                "finite bounded metres and a unit XYZW quaternion required",
            ));
        }
        Ok(())
    }
    pub(super) fn distance(&self, other: &Self) -> f64 {
        self.position_m
            .iter()
            .zip(other.position_m)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt()
    }
    pub(super) fn angle(&self, other: &Self) -> f64 {
        let dot = self
            .quaternion_xyzw
            .iter()
            .zip(other.quaternion_xyzw)
            .map(|(a, b)| a * b)
            .sum::<f64>();
        let [x, y, z, w] = self.quaternion_xyzw;
        let [a, b, c, d] = other.quaternion_xyzw;
        // Vector part of q * conjugate(other). atan2 preserves tiny rotations
        // that acos(dot) rounds to zero. Both terms share any norm scaling;
        // abs(dot) also makes q and -q represent the same orientation.
        let relative = [
            d * x - w * a - y * c + z * b,
            d * y - w * b - z * a + x * c,
            d * z - w * c - x * b + y * a,
        ];
        2. * relative
            .iter()
            .map(|v| v * v)
            .sum::<f64>()
            .sqrt()
            .atan2(dot.abs())
    }
}

/// One acquisition. A duplicate or different epoch cannot advance this run.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PoseObservation {
    pub source: String,
    pub epoch: String,
    pub frame: String,
    pub sequence: u64,
    pub acquired_ns: u64,
    pub pose: Pose,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Criteria {
    pub position_m: f64,
    pub orientation_rad: f64,
    pub speed_m_s: f64,
    pub dwell_ns: u64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Station {
    pub identity: Identity,
    pub pose: Pose,
    pub criteria: Criteria,
}

/// Immutable derived configuration with canonical declaration identities.
#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct InspectionModel {
    pub identity: Identity,
    pub source: String,
    pub frame: String,
    pub stations: Vec<Station>,
    pub home: Station,
    pub cycles: usize,
    pub max_gap_ns: u64,
    pub expiry_ns: u64,
    #[serde(skip)]
    pub(super) workflow: Bindings,
    #[serde(skip)]
    pub(super) output: OutputBindings,
}
impl InspectionModel {
    /// Number of authored station visits, excluding the terminal home check.
    pub fn required_visits(&self) -> usize {
        self.stations.len() * self.cycles
    }
    /// Authored target for the observed progress count; the home visit follows
    /// all station visits, and completed/out-of-range progress has no target.
    pub fn target_at(&self, completed: usize) -> Option<&Station> {
        let required = self.stations.len().checked_mul(self.cycles)?;
        if completed < required {
            self.stations.get(completed % self.stations.len())
        } else if completed == required {
            Some(&self.home)
        } else {
            None
        }
    }
}
#[derive(Clone, Debug)]
pub(super) struct Bindings {
    pub completed: usize,
    pub blocked: usize,
    pub advance: String,
    pub finish: String,
    pub block: String,
    pub resume: String,
}

#[derive(Clone, Debug)]
pub(super) struct OutputBindings {
    pub phase: DeclId,
    pub target_path: DeclId,
    pub evaluated_target_path: DeclId,
    pub target: DeclId,
    pub evaluated_target: DeclId,
    pub numbers: [DeclId; 17],
}
impl OutputBindings {
    pub const FIELDS: [&'static str; 17] = [
        "completed_visits",
        "required_visits",
        "fresh",
        "returned_home",
        "target_valid",
        "evidence_valid",
        "qualified",
        "position_error_m",
        "orientation_error_rad",
        "speed_valid",
        "speed_m_s",
        "dwell_s",
        "sample_present",
        "sequence_high",
        "sequence_low",
        "acquired_seconds",
        "acquired_nanoseconds",
    ];
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "camelCase")]
pub enum InspectionPhase {
    Missing,
    Approaching,
    Qualifying,
    Returning,
    Blocked,
    Complete,
}

/// Evidence explicitly names the evaluated target, which may just have completed.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Evidence {
    pub target: Identity,
    pub position_error_m: f64,
    pub orientation_error_rad: f64,
    pub speed_m_s: Option<f64>,
    pub qualified: bool,
    pub dwell_ns: u64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct InspectionSnapshot {
    pub task: Identity,
    pub phase: InspectionPhase,
    pub completed_visits: usize,
    pub required_visits: usize,
    pub target: Option<Identity>,
    pub returned_home: bool,
    pub fresh: bool,
    pub sample_sequence: Option<u64>,
    pub acquired_ns: Option<u64>,
    pub evidence: Option<Evidence>,
    pub revision: u32,
}
