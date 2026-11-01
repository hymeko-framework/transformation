//! A declared locomotion profile bound to an owner-projected legged robot.
//!
//! Core compiles both sources, Geometry supplies kinematics and colliders, and
//! `RobotProjection` emits the MJCF. This module only checks the declaration
//! against measured dimensions and derives the terrain bounds a generator may use.
//! Capability values (step height, slope, friction) are claims of the declared
//! controller profile; the simulator's physical pretests are their evidence.
use super::*;
use crate::robot_projection::RobotProjection;
use hymeko::module_store::source_provider::StdFsProvider;
use hymeko_geometry::{
    collide::{Convex, colliders_of},
    forward::{FkPlan, JointValues, forward_kinematics},
    kinematics::{
        actuation::DriveLaw,
        ik::{IkOptions, IkSolution, ModelChain, solve_serial},
        joints::JointType,
        kinematic::KinematicModel,
    },
};
use std::collections::BTreeMap;

/// Declared fields; units are metres, seconds and degrees as written in the source.
#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct LocomotionProfile {
    pub robot_root: String,
    pub base_joint: String,
    pub base_link: String,
    pub morphology: String,
    pub controller: String,
    pub stance_height_m: f64,
    pub nominal_speed_m_s: f64,
    pub step_length_m: f64,
    pub swing_height_m: f64,
    pub max_step_up_m: f64,
    pub max_slope_deg: f64,
    pub min_friction: f64,
    pub lateral_margin_m: f64,
    pub feet: Vec<String>,
}

/// Values measured on the projected model at its zero joint pose.
#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct MeasuredBody {
    pub mass_kg: f64,
    /// Base-link origin height above the lowest foot point.
    pub zero_pose_height_m: f64,
    /// Longest first-movable-joint to foot-contact distance.
    pub leg_length_m: f64,
    pub foot_length_m: f64,
    pub foot_width_m: f64,
    pub body_length_m: f64,
    pub body_width_m: f64,
    pub body_height_m: f64,
    /// Contact point of each foot in its link frame (lowest collider point at zero pose).
    pub foot_contact_local_m: Vec<[f64; 3]>,
    /// The same contact points in the zero-pose model frame.
    pub foot_contact_m: Vec<[f64; 3]>,
}

/// Admissible terrain parameters for this robot; a generator may exceed them only
/// to produce worlds that the geometric check classifies as infeasible.
#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct TerrainBounds {
    pub max_step_up_m: f64,
    pub max_slope_rad: f64,
    pub min_friction: f64,
    pub hurdle_height_m: f64,
    pub foothold_length_m: f64,
    pub foothold_width_m: f64,
    pub max_gap_m: f64,
    pub min_passage_width_m: f64,
    pub roughness_m: f64,
    pub nominal_speed_m_s: f64,
}

pub struct LeggedRobot {
    pub profile: LocomotionProfile,
    pub measured: MeasuredBody,
    pub bounds: TerrainBounds,
    pub profile_digest: String,
    pub robot_digest: String,
    mjcf: String,
    metadata: Value,
    /// The projected kinematic model the MJCF and the binding come from (absent in fixtures).
    model: Option<KinematicModel>,
}

fn digest_of(document: &CompiledModel) -> String {
    Canon::new(&document.program().ir, document.interner())
        .hash()
        .0
        .0
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

impl LeggedRobot {
    pub fn read(profile_path: &Path) -> Result<Self> {
        let document = CompiledModel::compile(StdFsProvider::new(), RealParser, profile_path)
            .map_err(|e| format!("locomotion profile compile: {e:?}"))?;
        let query = QueryEngine::new(&document.program().ir, document.interner());
        let profiles =
            query.query(&Predicate::node().and(Predicate::inherits("locomotion_profile")));
        let [record] = profiles.as_slice() else {
            return Err("expected exactly one locomotion_profile record".into());
        };
        let text = |id: DeclId, key: &str| -> Result<String> {
            match query.effective_value(id, key) {
                Some(ValueR::Str(s)) => Ok(document.interner().resolve(*s).to_owned()),
                _ => Err(format!("locomotion profile lacks string {key}").into()),
            }
        };
        let number = |key: &str| -> Result<f64> {
            match query.effective_value(record.id, key) {
                Some(ValueR::Num(v)) if v.is_finite() && *v >= 0.0 => Ok(*v),
                _ => Err(format!("locomotion profile lacks non-negative {key}").into()),
            }
        };
        let mut feet: Vec<_> = query
            .query(&Predicate::node().and(Predicate::inherits("locomotion_foot")))
            .into_iter()
            .map(|m| text(m.id, "link"))
            .collect::<Result<_>>()?;
        feet.sort();
        let profile = LocomotionProfile {
            robot_root: text(record.id, "robot_root")?,
            base_joint: text(record.id, "base_joint")?,
            base_link: text(record.id, "base_link")?,
            morphology: text(record.id, "morphology")?,
            controller: text(record.id, "controller")?,
            stance_height_m: number("stance_height_m")?,
            nominal_speed_m_s: number("nominal_speed_m_s")?,
            step_length_m: number("step_length_m")?,
            swing_height_m: number("swing_height_m")?,
            max_step_up_m: number("max_step_up_m")?,
            max_slope_deg: number("max_slope_deg")?,
            min_friction: number("min_friction")?,
            lateral_margin_m: number("lateral_margin_m")?,
            feet,
        };
        let expected_feet = match profile.morphology.as_str() {
            "quadruped" => 4,
            "biped" => 2,
            _ => return Err("morphology must be quadruped or biped".into()),
        };
        if profile.feet.len() != expected_feet {
            return Err("declared foot count does not match the morphology".into());
        }
        let source = text(record.id, "robot_source")?;
        let robot_path = profile_path
            .parent()
            .ok_or("profile has no directory")?
            .join(source);
        let robot = CompiledModel::compile(StdFsProvider::new(), RealParser, &robot_path)
            .map_err(|e| format!("robot compile: {e:?}"))?;
        let projection = RobotProjection::new(&robot, &profile.robot_root)?;
        let measured = Self::measure(&profile, projection.model())?;
        let bounds = Self::bind(&profile, &measured)?;
        let mut metadata = projection.metadata();
        metadata["drives"] = Self::drives(projection.model())?;
        metadata["joint_limits"] = Self::limits(&profile, projection.model())?;
        Ok(Self {
            profile_digest: digest_of(&document),
            robot_digest: digest_of(&robot),
            mjcf: projection.mjcf()?,
            metadata,
            model: Some(projection.model().clone()),
            profile,
            measured,
            bounds,
        })
    }

    fn kinematics(&self) -> Result<&KinematicModel> {
        self.model.as_ref().ok_or_else(|| "this legged robot carries no kinematic model".into())
    }

    /// Movable joints from the base link down to foot `index`, base to foot.
    pub fn leg_joints(&self, index: usize) -> Result<Vec<String>> {
        let foot = self.profile.feet.get(index).ok_or("no such foot")?;
        let names = ModelChain::path_names(self.kinematics()?, Some(&self.profile.base_link), foot);
        if names.is_empty() {
            return Err(format!("foot {foot} has no movable joint below {}", self.profile.base_link).into());
        }
        Ok(names.into_iter().map(str::to_owned).collect())
    }

    /// Inverse kinematics of one leg on the projected model: coordinates (in [`leg_joints`]
    /// order, from `start`) that place the foot link's frame at `position` with the row-major
    /// `rotation`, both relative to the base link, within the authored limits narrowed by
    /// `margin` on each side. This is Geometry's damped least squares (`ModelChain::below`,
    /// `solve_serial`); the simulator's legged controllers call it through their bridge.
    ///
    /// [`leg_joints`]: Self::leg_joints
    pub fn solve_leg(
        &self,
        index: usize,
        start: &[f64],
        position: [f64; 3],
        rotation: [f64; 9],
        margin: f64,
        options: &IkOptions,
    ) -> Result<IkSolution> {
        let model = self.kinematics()?;
        let joints = self.leg_joints(index)?;
        let names: Vec<&str> = joints.iter().map(String::as_str).collect();
        let chain = ModelChain::below(model, &self.profile.base_link, &names, &self.profile.feet[index])
            .map_err(|e| e.to_string())?;
        let limits: Vec<[f64; 2]> = names
            .iter()
            .map(|name| {
                let joint = model.joints.iter().find(|j| j.name == *name).expect("chain joints exist");
                match &joint.limits {
                    Some(l) if l.upper - l.lower > 2.0 * margin => [l.lower + margin, l.upper - margin],
                    Some(l) => [0.5 * (l.lower + l.upper); 2],
                    None => [f64::NEG_INFINITY, f64::INFINITY],
                }
            })
            .collect();
        // The chain lives in the model root's frame with the base joint at zero.
        let poses = forward_kinematics(model, &JointValues::new()).map_err(|e| format!("{e:?}"))?;
        let base = poses.get(&self.profile.base_link).ok_or("base link is not posed")?;
        let r = base.rotation_matrix();
        let t = &base.translation;
        let root_position = std::array::from_fn(|i| t[i] + (0..3).map(|k| r[i * 3 + k] * position[k]).sum::<f64>());
        let root_rotation: [f64; 9] =
            std::array::from_fn(|n| (0..3).map(|k| r[(n / 3) * 3 + k] * rotation[k * 3 + n % 3]).sum());
        solve_serial(&chain, &limits, start, &root_position, Some(&root_rotation), options).map_err(|e| e.to_string().into())
    }

    /// Every movable joint other than the base needs authored limits and effort.
    fn limits(profile: &LocomotionProfile, model: &KinematicModel) -> Result<Value> {
        let mut rows = BTreeMap::new();
        for joint in &model.joints {
            if joint.joint_type == JointType::Fixed || joint.name == profile.base_joint {
                continue;
            }
            let limits = joint
                .limits
                .as_ref()
                .ok_or_else(|| format!("joint {} has no declared limits", joint.name))?;
            if !(limits.lower < limits.upper && limits.effort > 0.0)
                || limits.lower <= -179.0
                || limits.upper >= 179.0
            {
                return Err(format!(
                    "joint {} keeps the unbounded default range or zero effort",
                    joint.name
                )
                .into());
            }
            rows.insert(
                joint.name.clone(),
                json!({"lower_deg":limits.lower,"upper_deg":limits.upper,"effort":limits.effort}),
            );
        }
        Ok(serde_json::to_value(rows)?)
    }

    fn drives(model: &KinematicModel) -> Result<Value> {
        let joint_name = |did: DeclId| {
            model
                .joints
                .iter()
                .find(|j| j.did == did)
                .map(|j| j.name.clone())
                .ok_or("drive control coordinate is not a joint")
        };
        model
            .actuation
            .drives
            .iter()
            .map(|d| -> Result<Value> {
                let (kind, area) = match d.law {
                    DriveLaw::Muscle { .. } => ("muscle", 0.0),
                    DriveLaw::Hydraulic { area_m2, .. } => ("hydraulic", area_m2),
                    DriveLaw::HydraulicMuscle { .. } => ("hydraulic_muscle", 0.0),
                };
                Ok(json!({"name":d.name,"actuator":format!("act_{}",d.name),
                    "joint":joint_name(d.joint)?,"kind":kind,"area_m2":area}))
            })
            .collect::<Result<Vec<_>>>()
            .map(Value::from)
    }

    fn measure(profile: &LocomotionProfile, model: &KinematicModel) -> Result<MeasuredBody> {
        let base = model
            .joints
            .iter()
            .find(|j| j.name == profile.base_joint)
            .ok_or("declared base joint is absent")?;
        if base.child_link != profile.base_link
            || !model.frames.contains(&base.parent_link)
            || !matches!(base.joint_type, JointType::Continuous | JointType::Revolute)
        {
            return Err("base joint must mount the base link on a world frame".into());
        }
        let index = |name: &str| {
            model
                .links
                .iter()
                .position(|l| l.name == name)
                .ok_or_else(|| format!("missing link {name}"))
        };
        for link in &model.links {
            let movable = link.mass.is_some_and(|m| m > 0.0);
            if movable
                && link
                    .inertia_com
                    .is_none_or(|i| i.iter().any(|v| !v.is_finite()) || i[..3].iter().any(|v| *v <= 0.0))
            {
                return Err(format!("link {} lacks an explicit inertia tensor", link.name).into());
            }
        }
        let plan = FkPlan::new(model)?;
        let q = plan.zero_values();
        let mut poses = plan.identity_poses_se3();
        plan.resolve_se3(model, &q, &mut poses);
        let colliders = colliders_of(model);
        // Extreme points of each collider in its link frame; cylinders are sampled on both rims.
        let points = |shape: &Convex, offset: [f64; 3]| -> Vec<[f64; 3]> {
            let add = |p: [f64; 3]| [p[0] + offset[0], p[1] + offset[1], p[2] + offset[2]];
            match *shape {
                Convex::Sphere { radius } => [
                    [radius, 0., 0.],
                    [-radius, 0., 0.],
                    [0., radius, 0.],
                    [0., -radius, 0.],
                    [0., 0., radius],
                    [0., 0., -radius],
                ]
                .map(add)
                .to_vec(),
                Convex::Cuboid { half } => (0..8)
                    .map(|i| {
                        add([
                            if i & 1 == 0 { -half[0] } else { half[0] },
                            if i & 2 == 0 { -half[1] } else { half[1] },
                            if i & 4 == 0 { -half[2] } else { half[2] },
                        ])
                    })
                    .collect(),
                Convex::Cylinder {
                    radius,
                    half_height,
                }
                | Convex::Capsule {
                    radius,
                    half_height,
                } => (0..16)
                    .flat_map(|k| {
                        let a = k as f64 * std::f64::consts::PI / 8.0;
                        [-half_height, half_height]
                            .map(|z| add([radius * a.cos(), radius * a.sin(), z]))
                    })
                    .collect(),
            }
        };
        let world_points: Vec<(usize, [f64; 3])> = colliders
            .iter()
            .flat_map(|c| {
                points(&c.shape, c.offset)
                    .into_iter()
                    .map(move |p| (c.link, p))
            })
            .map(|(link, p)| (link, poses[link].apply(&p)))
            .collect();
        let feet: Vec<usize> = profile
            .feet
            .iter()
            .map(|f| index(f))
            .collect::<std::result::Result<_, _>>()?;
        // A foot is its link and the whole subtree below it (the humanoid toe, held at zero).
        let subtree = |root: usize| -> Vec<usize> {
            let mut set = vec![root];
            let mut next = 0;
            while next < set.len() {
                let parent = model.links[set[next]].did;
                for joint in model.joints.iter().filter(|j| j.parent == parent) {
                    if let Some(c) = model.links.iter().position(|l| l.did == joint.child)
                        && !set.contains(&c)
                    {
                        set.push(c);
                    }
                }
                next += 1;
            }
            set
        };
        let ground = world_points
            .iter()
            .map(|(_, p)| p[2])
            .fold(f64::INFINITY, f64::min);
        let mut foot_contacts = Vec::new();
        let mut foot_world = Vec::new();
        let mut foot_extent = [0.0_f64, 0.0_f64];
        let mut leg_length = 0.0_f64;
        for &foot in &feet {
            let members = subtree(foot);
            let own: Vec<[f64; 3]> = world_points
                .iter()
                .filter(|(l, _)| members.contains(l))
                .map(|(_, p)| *p)
                .collect();
            if own.is_empty() {
                return Err(format!("foot {} has no collider", model.links[foot].name).into());
            }
            let low = own.iter().map(|p| p[2]).fold(f64::INFINITY, f64::min);
            if (low - ground).abs() > 0.02 {
                return Err(format!(
                    "foot {} is not a lowest contact at zero pose",
                    model.links[foot].name
                )
                .into());
            }
            let sole: Vec<_> = own.iter().filter(|p| p[2] <= low + 1e-6).collect();
            let centre = [
                sole.iter().map(|p| p[0]).sum::<f64>() / sole.len() as f64,
                sole.iter().map(|p| p[1]).sum::<f64>() / sole.len() as f64,
                low,
            ];
            let span = |axis: usize| {
                own.iter().map(|p| p[axis]).fold(f64::NEG_INFINITY, f64::max)
                    - own.iter().map(|p| p[axis]).fold(f64::INFINITY, f64::min)
            };
            foot_extent[0] = foot_extent[0].max(span(0));
            foot_extent[1] = foot_extent[1].max(span(1));
            // The foot pose is a pure translation from the origin at zero pose for these models;
            // the contact point is expressed relative to the link origin in world axes.
            let origin = poses[foot].apply(&[0.0; 3]);
            foot_world.push(centre);
            foot_contacts.push([
                centre[0] - origin[0],
                centre[1] - origin[1],
                centre[2] - origin[2],
            ]);
            // Walk up the leg: its root (hip) is the joint whose parent carries more than one foot,
            // so a body joint such as the quadruped waist is not counted as leg length.
            let mut link = foot;
            let mut hip = None;
            while let Some(joint) = model.joints.iter().find(|j| j.child == model.links[link].did) {
                if joint.name == profile.base_joint {
                    break;
                }
                let parent = model
                    .links
                    .iter()
                    .position(|l| l.did == joint.parent)
                    .ok_or("joint parent is not a link")?;
                if joint.joint_type != JointType::Fixed {
                    hip = Some(joint);
                }
                if feet.iter().filter(|f| subtree(parent).contains(f)).count() > 1 {
                    break;
                }
                link = parent;
            }
            let hip = hip.ok_or("foot has no movable chain to the base")?;
            let hip_child = index(&hip.child_link)?;
            let h = poses[hip_child].apply(&[0.0; 3]);
            leg_length = leg_length.max(
                ((h[0] - centre[0]).powi(2) + (h[1] - centre[1]).powi(2) + (h[2] - centre[2]).powi(2))
                    .sqrt(),
            );
        }
        let base_index = index(&profile.base_link)?;
        let base_origin = poses[base_index].apply(&[0.0; 3]);
        let span = |axis: usize| {
            world_points
                .iter()
                .map(|(_, p)| p[axis])
                .fold(f64::NEG_INFINITY, f64::max)
                - world_points
                    .iter()
                    .map(|(_, p)| p[axis])
                    .fold(f64::INFINITY, f64::min)
        };
        Ok(MeasuredBody {
            mass_kg: model.links.iter().filter_map(|l| l.mass).sum(),
            zero_pose_height_m: base_origin[2] - ground,
            leg_length_m: leg_length,
            foot_length_m: foot_extent[0],
            foot_width_m: foot_extent[1],
            body_length_m: span(0),
            body_width_m: span(1),
            body_height_m: span(2),
            foot_contact_local_m: foot_contacts,
            foot_contact_m: foot_world,
        })
    }

    /// Reject declarations the body cannot satisfy and derive the terrain bounds.
    fn bind(profile: &LocomotionProfile, body: &MeasuredBody) -> Result<TerrainBounds> {
        let leg = body.leg_length_m;
        let checks = [
            (
                "stance height within the zero-pose reach",
                profile.stance_height_m > 0.5 * body.zero_pose_height_m
                    && profile.stance_height_m <= body.zero_pose_height_m,
            ),
            (
                "step length below the leg length",
                profile.step_length_m > 0.0 && profile.step_length_m < leg,
            ),
            (
                "swing clearance below a third of the leg",
                profile.swing_height_m > 0.0 && profile.swing_height_m < leg / 3.0,
            ),
            (
                "step-up height below the swing clearance",
                profile.max_step_up_m < profile.swing_height_m,
            ),
            (
                "slope below 30 degrees",
                profile.max_slope_deg > 0.0 && profile.max_slope_deg < 30.0,
            ),
            (
                "friction in (0,1.5]",
                profile.min_friction > 0.0 && profile.min_friction <= 1.5,
            ),
            (
                "positive nominal speed below one leg length per second",
                profile.nominal_speed_m_s > 0.0 && profile.nominal_speed_m_s < leg,
            ),
        ];
        if let Some((label, _)) = checks.iter().find(|(_, ok)| !ok) {
            return Err(format!("locomotion profile refused: {label}").into());
        }
        Ok(TerrainBounds {
            max_step_up_m: profile.max_step_up_m,
            max_slope_rad: profile.max_slope_deg.to_radians(),
            min_friction: profile.min_friction,
            hurdle_height_m: profile.max_step_up_m,
            foothold_length_m: body.foot_length_m * 1.25,
            foothold_width_m: body.foot_width_m * 1.25,
            max_gap_m: (profile.step_length_m - body.foot_length_m).max(0.0) * 0.5,
            min_passage_width_m: body.body_width_m + 2.0 * profile.lateral_margin_m,
            roughness_m: profile.max_step_up_m * 0.4,
            nominal_speed_m_s: profile.nominal_speed_m_s,
        })
    }

    pub fn mjcf(&self) -> &str {
        &self.mjcf
    }

    /// A projection-free robot for generator tests; never written to a physical scene.
    #[cfg(test)]
    pub(crate) fn fixture(morphology: &str) -> Self {
        let biped = morphology == "biped";
        let profile = LocomotionProfile {
            robot_root: "fixture".into(),
            base_joint: "base".into(),
            base_link: "body".into(),
            morphology: morphology.into(),
            controller: "fixture".into(),
            stance_height_m: if biped { 0.92 } else { 0.225 },
            nominal_speed_m_s: if biped { 0.06 } else { 0.18 },
            step_length_m: if biped { 0.15 } else { 0.06 },
            swing_height_m: if biped { 0.06 } else { 0.04 },
            max_step_up_m: if biped { 0.04 } else { 0.025 },
            max_slope_deg: if biped { 5.0 } else { 8.0 },
            min_friction: if biped { 0.5 } else { 0.45 },
            lateral_margin_m: if biped { 0.12 } else { 0.06 },
            feet: if biped {
                vec!["l".into(), "r".into()]
            } else {
                vec!["a".into(), "b".into(), "c".into(), "d".into()]
            },
        };
        let measured = MeasuredBody {
            mass_kg: if biped { 63.25 } else { 5.705 },
            zero_pose_height_m: if biped { 0.97 } else { 0.27 },
            leg_length_m: if biped { 0.89 } else { 0.22 },
            foot_length_m: if biped { 0.3 } else { 0.05 },
            foot_width_m: if biped { 0.11 } else { 0.052 },
            body_length_m: if biped { 0.355 } else { 0.61 },
            body_width_m: if biped { 0.552 } else { 0.224 },
            body_height_m: if biped { 1.7 } else { 0.5 },
            foot_contact_local_m: vec![],
            foot_contact_m: if biped {
                vec![[0.055, 0.105, 0.0], [0.055, -0.105, 0.0]]
            } else {
                vec![]
            },
        };
        let bounds = Self::bind(&profile, &measured).expect("fixture profile is admissible");
        Self {
            profile,
            measured,
            bounds,
            profile_digest: "fixture-profile".into(),
            robot_digest: "fixture-robot".into(),
            mjcf: String::new(),
            metadata: json!({}),
            model: None,
        }
    }

    /// The simulator binding: projection metadata plus declared, measured and derived values.
    pub fn binding(&self) -> Value {
        json!({
            "schema": "hymeko-legged-robot",
            "projection": self.metadata,
            "profile": self.profile,
            "measured": self.measured,
            "terrain_bounds": self.bounds,
            "profile_digest": self.profile_digest,
            "robot_digest": self.robot_digest,
            "units": "m, s, kg; JSON angles in rad unless named _deg; HyMeKo mount angles in degrees",
        })
    }

    pub fn write(&self, output: &Path) -> Result<()> {
        for name in ["robot.xml", "robot.json"] {
            if output.join(name).exists() {
                return Err(format!("refusing to replace {name}").into());
            }
        }
        fs::write(output.join("robot.xml"), &self.mjcf)?;
        fs::write(
            output.join("robot.json"),
            serde_json::to_vec_pretty(&self.binding())?,
        )?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn declarations_beyond_the_body_are_refused() {
        let robot = LeggedRobot::fixture("biped");
        let mut profile = robot.profile.clone();
        assert!(LeggedRobot::bind(&profile, &robot.measured).is_ok());
        profile.stance_height_m = 1.2;
        assert!(LeggedRobot::bind(&profile, &robot.measured).is_err());
        profile = robot.profile.clone();
        profile.max_step_up_m = profile.swing_height_m;
        assert!(LeggedRobot::bind(&profile, &robot.measured).is_err());
        profile = robot.profile.clone();
        profile.step_length_m = 1.0;
        assert!(LeggedRobot::bind(&profile, &robot.measured).is_err());
    }

    #[test]
    fn bounds_scale_with_the_measured_body() {
        let biped = LeggedRobot::fixture("biped").bounds;
        let quadruped = LeggedRobot::fixture("quadruped").bounds;
        assert!((biped.min_passage_width_m - (0.552 + 0.24)).abs() < 1e-12);
        assert!((biped.foothold_length_m - 0.375).abs() < 1e-12);
        assert!(quadruped.min_passage_width_m < biped.min_passage_width_m);
        assert!((quadruped.max_slope_rad - 8.0_f64.to_radians()).abs() < 1e-12);
    }

    /// Reads a real profile and robot (sibling simulator checkout); no simulation.
    #[test]
    #[ignore = "requires HYMEKO_LOCOMOTION_PROFILE pointing at a simulator locomotion profile"]
    fn real_profile_projects_limits_drives_and_feet() {
        let path = std::env::var("HYMEKO_LOCOMOTION_PROFILE").expect("profile path");
        let robot = LeggedRobot::read(Path::new(&path)).unwrap();
        let binding = robot.binding();
        assert_eq!(binding["schema"], "hymeko-legged-robot");
        assert!(robot.mjcf().contains("<mujoco"));
        assert_eq!(robot.measured.foot_contact_m.len(), robot.profile.feet.len());
        assert!(robot.measured.leg_length_m > robot.profile.step_length_m);
        assert!(!binding["projection"]["joint_limits"].as_object().unwrap().is_empty());
        // Leg IK on the projected model: a foot pose read off forward kinematics at a bent leg
        // is found again from a different start, relative to the base link.
        let model = robot.model.as_ref().unwrap();
        for index in 0..robot.profile.feet.len() {
            let joints = robot.leg_joints(index).unwrap();
            let truth: Vec<f64> = joints.iter().enumerate().map(|(k, _)| if k + 3 == joints.len() { 0.5 } else { 0.1 }).collect();
            let mut values: JointValues = joints.iter().cloned().zip(truth.iter().copied()).collect();
            values.insert(robot.profile.base_joint.clone(), 0.0);
            let poses = forward_kinematics(model, &values).unwrap();
            let (base, foot) = (poses.get(&robot.profile.base_link).unwrap(), poses.get(&robot.profile.feet[index]).unwrap());
            let (rb, rf) = (base.rotation_matrix(), foot.rotation_matrix());
            let d: [f64; 3] = std::array::from_fn(|i| foot.translation[i] - base.translation[i]);
            let position = std::array::from_fn(|i| (0..3).map(|k| rb[k * 3 + i] * d[k]).sum::<f64>());
            let rotation = std::array::from_fn(|n| (0..3).map(|k| rb[k * 3 + n / 3] * rf[k * 3 + n % 3]).sum::<f64>());
            let start: Vec<f64> = truth.iter().map(|q| q + 0.05).collect();
            let solution = robot.solve_leg(index, &start, position, rotation, 0.0, &IkOptions::default()).unwrap();
            assert!(solution.converged, "foot {index}: {solution:?}");
        }
    }
}
