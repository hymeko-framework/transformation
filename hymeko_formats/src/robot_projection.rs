//! Robot export and FK diagnostics over a retained Core compilation.
//! Physical execution and scenario assembly remain consumer responsibilities.
use hymeko::{ir::canonical_hash::Canon, module_store::execution::CompiledModel};
use hymeko_geometry::{
    forward::{FkPlan, JointValues, forward_kinematics},
    kinematics::kinematic::KinematicModel,
    transforms::{ModelKind, ModelView, extract},
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

    /// Deterministic diagnostic pose retained from the original cross-check.
    pub fn diagnostic_pose(&self) -> JointValues {
        self.model()
            .joints
            .iter()
            .enumerate()
            .map(|(i, joint)| {
                (
                    joint.name.clone(),
                    0.35 * ((i as f64 * 0.7).sin() + 0.25 * (i as f64 * 1.9).cos()),
                )
            })
            .collect()
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
