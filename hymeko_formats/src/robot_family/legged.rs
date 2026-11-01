//! The `legged_pairs` family: a box body and `legs` legs in left/right pairs. The leg count is
//! the structural choice of a family value; the declared continuous parameters are shared by
//! every count. The leg layout follows the 2026-10-15 legged cases (hip abduction about X, hip
//! flexion and knee about Y, fixed foot), now with declared limits and density-derived masses.
use super::{DesignPoint, Expectation, FamilyError, JointKind, Limits, LinkSpec, ParameterSpace, Requirement, RobotFamily, RobotSource, Shape};
use std::collections::BTreeMap;

const HIP_RADIUS: f64 = 0.045;
const FOOT: [f64; 3] = [0.18, 0.08, 0.05];

pub struct LeggedPairs {
    space: ParameterSpace,
    legs: usize,
}

impl LeggedPairs {
    pub fn new(space: ParameterSpace, legs: usize) -> Result<Self, FamilyError> {
        if space.generator != "legged_pairs" {
            return Err(FamilyError::Declaration(format!("generator {} is not legged_pairs", space.generator)));
        }
        let counts = space.lists.get("leg_counts").ok_or_else(|| FamilyError::Declaration("leg_counts absent".into()))?;
        if !counts.contains(&(legs as f64)) {
            return Err(FamilyError::Rejected(format!("{legs} legs is not a declared leg count {counts:?}")));
        }
        for key in ["mount_height", "hip_drop", "flex_lower", "flex_upper", "knee_lower", "knee_upper", "abduct_lower", "abduct_upper", "effort", "velocity"] {
            space.constant(&format!("stance.{key}"))?;
        }
        Ok(Self { space, legs })
    }

    pub fn legs(&self) -> usize {
        self.legs
    }

    /// Pair positions along X, front first, centred on the body.
    fn pair_x(&self, d: &DesignPoint) -> Vec<f64> {
        let pairs = self.legs / 2;
        (0..pairs).map(|k| ((pairs as f64 - 1.) / 2. - k as f64) * d.get("pair_pitch")).collect()
    }

    fn limits(&self, joint: &str) -> Limits {
        let c = |k: &str| self.space.constant(&format!("stance.{k}")).expect("checked in new");
        Limits { lower_deg: c(&format!("{joint}_lower")), upper_deg: c(&format!("{joint}_upper")), effort: c("effort"), velocity: c("velocity") }
    }

    fn suffixes(&self) -> Vec<String> {
        (0..self.legs / 2).flat_map(|k| ["l", "r"].map(|side| format!("{side}{k}"))).collect()
    }
}

fn link(name: &str, shape: Shape, density: f64, origin: [f64; 3]) -> LinkSpec {
    let mass = density * shape.volume();
    LinkSpec { name: name.into(), mass, shape, origin, inertia: Some(shape.solid_inertia(mass)), color: None }
}

impl RobotFamily for LeggedPairs {
    fn space(&self) -> &ParameterSpace {
        &self.space
    }

    fn module(&self) -> String {
        format!("legged_{}", self.legs)
    }

    fn link_specs(&self, d: &DesignPoint) -> Result<Vec<LinkSpec>, FamilyError> {
        let (rho, r) = (d.get("leg_density"), d.get("leg_radius"));
        let mut links = vec![link("body", Shape::Box([d.get("body_length"), d.get("body_width"), d.get("body_height")]), d.get("body_density"), [0.; 3])];
        for suffix in self.suffixes() {
            links.push(link(&format!("hip_{suffix}"), Shape::Sphere(HIP_RADIUS), rho, [0.; 3]));
            links.push(link(&format!("upper_{suffix}"), Shape::Cylinder { radius: r, length: d.get("thigh_length") }, rho, [0., 0., -d.get("thigh_length") / 2.]));
            links.push(link(&format!("lower_{suffix}"), Shape::Cylinder { radius: r, length: d.get("shin_length") }, rho, [0., 0., -d.get("shin_length") / 2.]));
            links.push(link(&format!("foot_{suffix}"), Shape::Box(FOOT), rho, [0.06, 0., -FOOT[2] / 2.]));
        }
        Ok(links)
    }

    fn emit(&self, d: &DesignPoint) -> Result<String, FamilyError> {
        for x in self.pair_x(d) {
            if x.abs() > d.get("body_length") / 2. {
                return Err(FamilyError::Rejected(format!("a leg pair at x = {x} m lies beyond the body (length {})", d.get("body_length"))));
            }
        }
        let root = self.module();
        let mut s = RobotSource::new(&root, "../meta_kinematics.hymeko");
        s.frame("world");
        for l in self.link_specs(d)? {
            s.link(&l);
        }
        let mount = self.space.constant("stance.mount_height")?;
        s.joint("base", "world", "body", JointKind::Fixed, [0., 0., mount], [0.; 3]);
        let (abduct, flex, knee) = (self.limits("abduct"), self.limits("flex"), self.limits("knee"));
        let drop = self.space.constant("stance.hip_drop")?;
        for (k, x) in self.pair_x(d).into_iter().enumerate() {
            for (side, y) in [("l", d.get("hip_offset")), ("r", -d.get("hip_offset"))] {
                let sfx = format!("{side}{k}");
                let [hip, upper, lower, foot] = ["hip", "upper", "lower", "foot"].map(|p| format!("{p}_{sfx}"));
                s.joint(&format!("abduct_{sfx}"), "body", &hip, JointKind::Revolute { axis: "AXIS_X", limits: abduct }, [x, y, -drop], [0.; 3]);
                s.joint(&format!("flex_{sfx}"), &hip, &upper, JointKind::Revolute { axis: "AXIS_Y", limits: flex }, [0.; 3], [0.; 3]);
                s.joint(&format!("knee_{sfx}"), &upper, &lower, JointKind::Revolute { axis: "AXIS_Y", limits: knee }, [0., 0., -d.get("thigh_length")], [0.; 3]);
                s.joint(&format!("ankle_{sfx}"), &lower, &foot, JointKind::Fixed, [0., 0., -d.get("shin_length")], [0.; 3]);
            }
        }
        Ok(s.finish())
    }

    fn expectation(&self) -> Expectation {
        let sfx = self.suffixes();
        let mut links = vec!["body".to_string()];
        let mut movable = Vec::new();
        let mut axes = BTreeMap::new();
        for s in &sfx {
            links.extend(["hip", "upper", "lower", "foot"].map(|p| format!("{p}_{s}")));
            for (j, a) in [("abduct", [1., 0., 0.]), ("flex", [0., 1., 0.]), ("knee", [0., 1., 0.])] {
                movable.push(format!("{j}_{s}"));
                axes.insert(format!("{j}_{s}"), a);
            }
        }
        let pairs = self.legs / 2;
        Expectation {
            root: self.module(),
            links,
            movable,
            axes,
            above: sfx.iter().map(|s| ("body".to_string(), format!("foot_{s}"))).collect(),
            left_of: (0..pairs).map(|k| (format!("hip_l{k}"), format!("hip_r{k}"))).collect(),
            ahead_of: (1..pairs).map(|k| (format!("hip_l{}", k - 1), format!("hip_l{k}"))).collect(),
        }
    }

    /// Hanging at the declared mount height, the straight legs clear the floor; adjacent legs
    /// and the two sides do not intersect.
    fn requirements(&self, d: &DesignPoint) -> Result<Vec<Requirement>, FamilyError> {
        let mount = self.space.constant("stance.mount_height")?;
        let drop = self.space.constant("stance.hip_drop")?;
        let clearance = mount - drop - d.get("thigh_length") - d.get("shin_length") - FOOT[2];
        let r = d.get("leg_radius").max(HIP_RADIUS);
        let mut out = vec![Requirement { name: "floor_clearance".into(), value: clearance, bound: 0., unit: "m".into(), passed: clearance >= 0. }];
        out.push(Requirement { name: "side_gap".into(), value: 2. * d.get("hip_offset") - 2. * r, bound: 0., unit: "m".into(), passed: d.get("hip_offset") > r });
        if self.legs > 2 {
            out.push(Requirement { name: "pair_gap".into(), value: d.get("pair_pitch") - 2. * r, bound: 0., unit: "m".into(), passed: d.get("pair_pitch") > 2. * r });
        }
        Ok(out)
    }
}
