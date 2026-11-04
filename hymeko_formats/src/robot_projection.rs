//! Robot export and FK diagnostics over a retained Core compilation.
//! Physical execution and scenario assembly remain consumer responsibilities.
use crate::plugin::{ModelKind, ModelView, extract};
use hymeko::{ir::canonical_hash::Canon, module_store::execution::CompiledModel};
use hymeko_mechanism::{
    forward::{FkPlan, JointValues, forward_kinematics},
    kinematics::{
        joints::{JointLimits, JointType},
        kinematic::KinematicModel,
    },
};
use serde_json::{Value, json};

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// An export view, borrowing the authoritative compilation throughout its lifetime.
pub struct RobotProjection<'a> {
    document: &'a CompiledModel,
    root: &'a str,
    view: ModelView,
}

impl<'a> RobotProjection<'a> {
    /// `root` labels the export, following Geometry's existing extraction contract.
    /// The compiled document must contain the intended single robot.
    pub fn new(document: &'a CompiledModel, root: &'a str) -> Result<Self> {
        let view = extract(
            &document.program().ir,
            document.interner(),
            root,
            ModelKind::Kinematic,
        );
        let model = view.as_kinematic().ok_or("expected a kinematic model")?;
        if model.links.is_empty() {
            return Err("robot projection requires at least one link".into());
        }
        FkPlan::new(model)?;
        Ok(Self {
            document,
            root,
            view,
        })
    }

    pub fn model(&self) -> &KinematicModel {
        // The constructor only accepts the kinematic variant.
        self.view.as_kinematic().expect("kinematic projection")
    }

    pub fn mjcf(&self) -> Result<String> {
        crate::generate_description(
            &self.document.program().ir,
            self.document.interner(),
            self.root,
            crate::OutputFormat::Mjcf,
        )
        .map_err(|error| error.to_string().into())
    }

    pub fn metadata(&self) -> Value {
        let canonical_hash: String =
            Canon::new(&self.document.program().ir, self.document.interner())
                .hash()
                .0
                .0
                .iter()
                .map(|byte| format!("{byte:02x}"))
                .collect();
        json!({
            "schema": "hymeko.robot-projection/1", "canonical_hash": canonical_hash,
            "root": self.root,
            "links": self.model().links.iter().map(|link| &link.name).collect::<Vec<_>>(),
            "joints": self.model().joints.iter().map(|joint| &joint.name).collect::<Vec<_>>()
        })
    }

    /// Deterministic diagnostic pose, respecting finite, ordered position limits.
    /// Authored revolute limits are degrees; FK coordinates are radians.
    /// This is a display probe, not a trajectory or collision certificate.
    pub fn diagnostic_pose(&self) -> JointValues {
        self.model()
            .joints
            .iter()
            .enumerate()
            .map(|(i, joint)| {
                (
                    joint.name.clone(),
                    Self::diagnostic_coordinate(i, joint.joint_type, joint.limits.as_ref()),
                )
            })
            .collect()
    }

    fn diagnostic_coordinate(index: usize, kind: JointType, limits: Option<&JointLimits>) -> f64 {
        if kind == JointType::Fixed {
            return 0.0;
        }
        let value = 0.35 * ((index as f64 * 0.7).sin() + 0.25 * (index as f64 * 1.9).cos());
        if matches!(kind, JointType::Revolute | JointType::Prismatic)
            && let Some(limits) = limits
            && limits.lower.is_finite()
            && limits.upper.is_finite()
            && limits.lower <= limits.upper
        {
            let (lower, upper) = if kind == JointType::Revolute {
                (limits.lower.to_radians(), limits.upper.to_radians())
            } else {
                (limits.lower, limits.upper)
            };
            value.clamp(lower, upper)
        } else {
            // Invalid declarations are the domain validator's responsibility.
            value
        }
    }

    pub fn forward_poses(&self, q: &JointValues) -> Result<Value> {
        let poses = forward_kinematics(self.model(), q)?;
        let rows: Vec<_> = self
            .model()
            .links
            .iter()
            .filter_map(|link| {
                poses.get(&link.name).map(|pose| {
                    json!({
                        "link": link.name, "pos": pose.translation,
                        "mat": pose.rotation_matrix().to_vec()
                    })
                })
            })
            .collect();
        Ok(json!({"q": q, "poses": rows}))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diagnostic_coordinates_respect_joint_units_and_stops() {
        let mut limits = JointLimits {
            lower: -120.0,
            upper: 0.0,
            effort: 40.0,
            velocity: 8.0,
        };
        for index in 0..64 {
            let q =
                RobotProjection::diagnostic_coordinate(index, JointType::Revolute, Some(&limits));
            assert!((limits.lower.to_radians()..=0.0).contains(&q));
        }
        limits.lower = 1.0;
        limits.upper = 2.0;
        let angle = RobotProjection::diagnostic_coordinate(0, JointType::Revolute, Some(&limits));
        assert_eq!(angle, 2.0_f64.to_radians());
        assert_eq!(
            RobotProjection::diagnostic_coordinate(0, JointType::Prismatic, Some(&limits)),
            1.0
        );
        assert_eq!(
            RobotProjection::diagnostic_coordinate(0, JointType::Fixed, Some(&limits)),
            0.0
        );
        assert_eq!(
            RobotProjection::diagnostic_coordinate(0, JointType::Continuous, Some(&limits)),
            0.0875
        );
        assert_eq!(
            RobotProjection::diagnostic_coordinate(0, JointType::Revolute, None),
            0.0875
        );
        limits.lower = 3.0;
        assert_eq!(
            RobotProjection::diagnostic_coordinate(0, JointType::Revolute, Some(&limits)),
            0.0875
        );
    }
}
