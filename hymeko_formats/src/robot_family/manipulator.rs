//! The `serial_six_axis` family: the cell's FANUC LR Mate 200iD-style arm with five continuous
//! geometry parameters. Names, joint order, axes and limits are the reference model's
//! (`data/robotics/sim/dual_fanuc/fanuc_lrmate200id.hymeko`), so the generated cell binds the
//! same `robot.j1..j6` and `robot.tool_flange`; the defaults reproduce its link frames and masses.
//! Every link declares the COM inertia of its mass model (the reference model declares none).
use super::{DesignPoint, Expectation, FamilyError, JointKind, Limits, LinkSpec, ParameterSpace, Requirement, RobotFamily, RobotSource, Shape};
use std::collections::BTreeMap;

const PEDESTAL_RADIUS: f64 = 0.10;
const SHOULDER: f64 = 0.075; // link_j1: J1 to J2
const ELBOW: f64 = 0.075; // link_elbow: J3 to J4
const WRIST: f64 = 0.05; // link_wrist: J5 to J6
const FLANGE: f64 = 0.02;
const YELLOW: [f64; 4] = [0.96, 0.86, 0.13, 1.0];
const BLACK: [f64; 4] = [0.18, 0.18, 0.18, 1.0];
/// Reference joints: name, parent, child, axis, limits (lower/upper degrees, N m, rad/s).
const JOINTS: [(&str, &str, &str, &str, [f64; 4]); 6] = [
    ("j1", "base_link", "link_j1", "AXIS_Z", [-170., 170., 60., 6.28]),
    ("j2", "link_j1", "link_upper_arm", "AXIS_Y", [-100., 145., 60., 5.06]),
    ("j3", "link_upper_arm", "link_elbow", "AXIS_Y", [-180., 213., 50., 6.28]),
    ("j4", "link_elbow", "link_forearm", "AXIS_Z", [-190., 190., 30., 7.85]),
    ("j5", "link_forearm", "link_wrist", "AXIS_Y", [-125., 125., 30., 7.85]),
    ("j6", "link_wrist", "tool_flange", "AXIS_Z", [-360., 360., 20., 12.56]),
];

pub struct SerialSixAxis {
    space: ParameterSpace,
}

/// An aluminium tube of outer radius `r` and the family's wall, with a solid equipment core of
/// fixed mass filling its bore. Mass and COM inertia (cylinder axis = local z).
struct Tube {
    radius: f64,
    length: f64,
    tube_mass: f64,
    equipment: f64,
    wall: f64,
}

impl Tube {
    fn mass(&self) -> f64 {
        self.tube_mass + self.equipment
    }
    fn inertia(&self) -> [f64; 6] {
        let (r, ri, l) = (self.radius, self.radius - self.wall, self.length);
        let (mt, me) = (self.tube_mass, self.equipment);
        let side = mt * (3. * (r * r + ri * ri) + l * l) / 12. + me * (3. * ri * ri + l * l) / 12.;
        [side, side, mt * (r * r + ri * ri) / 2. + me * ri * ri / 2., 0., 0., 0.]
    }
    /// Second moment of area of the tube section.
    fn area_moment(&self) -> f64 {
        let ri = self.radius - self.wall;
        std::f64::consts::PI / 4. * (self.radius.powi(4) - ri.powi(4))
    }
}

impl SerialSixAxis {
    pub fn new(space: ParameterSpace) -> Result<Self, FamilyError> {
        if space.generator != "serial_six_axis" {
            return Err(FamilyError::Declaration(format!("generator {} is not serial_six_axis", space.generator)));
        }
        let family = Self { space };
        let defaults = family.space.defaults();
        for key in ["tube.density", "tube.elastic_modulus", "tube.wall", "stiffness.payload", "stiffness.max_deflection", "stiffness.gravity"] {
            family.space.constant(key)?;
        }
        if family.space.constant("tube.wall")? >= family.space.parameters.iter().filter(|p| p.name.ends_with("radius")).map(|p| p.lower).fold(f64::INFINITY, f64::min) {
            return Err(FamilyError::Declaration("tube wall must be thinner than every admissible radius".into()));
        }
        family.tubes(&defaults)?;
        Ok(family)
    }

    fn tube(&self, radius: f64, length: f64, reference: &str, reference_radius: f64, reference_length: f64) -> Result<Tube, FamilyError> {
        let (rho, wall) = (self.space.constant("tube.density")?, self.space.constant("tube.wall")?);
        let shell = |r: f64, l: f64| rho * std::f64::consts::PI * (r * r - (r - wall) * (r - wall)) * l;
        let equipment = self.space.constant(reference)? - shell(reference_radius, reference_length);
        if !(equipment >= 0.) {
            return Err(FamilyError::Declaration(format!("{reference} is lighter than its tube")));
        }
        Ok(Tube { radius, length, tube_mass: shell(radius, length), equipment, wall })
    }

    /// Pedestal, upper arm and forearm tubes of a design.
    fn tubes(&self, d: &DesignPoint) -> Result<[Tube; 3], FamilyError> {
        let reference = self.space.defaults();
        Ok([
            self.tube(PEDESTAL_RADIUS, d.get("pedestal_height"), "tube.pedestal_mass", PEDESTAL_RADIUS, reference.get("pedestal_height"))?,
            self.tube(d.get("upper_arm_radius"), d.get("upper_arm_length"), "tube.upper_arm_mass", reference.get("upper_arm_radius"), reference.get("upper_arm_length"))?,
            self.tube(d.get("forearm_radius"), d.get("forearm_length"), "tube.forearm_mass", reference.get("forearm_radius"), reference.get("forearm_length"))?,
        ])
    }
}

fn solid(name: &str, mass: f64, radius: f64, length: f64, color: &str) -> LinkSpec {
    let shape = Shape::Cylinder { radius, length };
    LinkSpec { name: name.into(), mass, shape, origin: [0., 0., length / 2.], inertia: Some(shape.solid_inertia(mass)), color: Some(format!("robot.{color}")) }
}

fn tube_link(name: &str, tube: &Tube, color: &str) -> LinkSpec {
    LinkSpec {
        name: name.into(),
        mass: tube.mass(),
        shape: Shape::Cylinder { radius: tube.radius, length: tube.length },
        origin: [0., 0., tube.length / 2.],
        inertia: Some(tube.inertia()),
        color: Some(format!("robot.{color}")),
    }
}

impl RobotFamily for SerialSixAxis {
    fn space(&self) -> &ParameterSpace {
        &self.space
    }

    fn module(&self) -> String {
        "serial_six_axis_robot".into()
    }

    fn link_specs(&self, d: &DesignPoint) -> Result<Vec<LinkSpec>, FamilyError> {
        let [pedestal, upper, fore] = self.tubes(d)?;
        Ok(vec![
            tube_link("base_link", &pedestal, "fanuc_black"),
            solid("link_j1", 4.5, 0.085, 2. * SHOULDER, "fanuc_yellow"),
            tube_link("link_upper_arm", &upper, "fanuc_yellow"),
            solid("link_elbow", 2.0, 0.05, ELBOW, "fanuc_yellow"),
            tube_link("link_forearm", &fore, "fanuc_yellow"),
            solid("link_wrist", 0.8, 0.04, WRIST, "fanuc_black"),
            solid("tool_flange", 0.3, 0.025, FLANGE, "fanuc_black"),
        ])
    }

    fn emit(&self, d: &DesignPoint) -> Result<String, FamilyError> {
        let links = self.link_specs(d)?;
        let mut s = RobotSource::new("robot", "../meta_kinematics.hymeko");
        s.frame("world").constant("fanuc_yellow", &YELLOW).constant("fanuc_black", &BLACK);
        for link in &links {
            s.link(link);
        }
        s.joint("j_fix", "world", "base_link", JointKind::Fixed, [0.; 3], [0.; 3]);
        // Each joint sits at the far end of its parent link's frame extent along local z.
        let mounts = [d.get("pedestal_height"), SHOULDER, d.get("upper_arm_length"), ELBOW, d.get("forearm_length"), WRIST];
        for ((name, parent, child, axis, [lower_deg, upper_deg, effort, velocity]), z) in JOINTS.into_iter().zip(mounts) {
            let limits = Limits { lower_deg, upper_deg, effort, velocity };
            s.joint(name, parent, child, JointKind::Revolute { axis, limits }, [0., 0., z], [0.; 3]);
        }
        Ok(s.finish())
    }

    fn expectation(&self) -> Expectation {
        let axis = |a: &str| match a {
            "AXIS_Z" => [0., 0., 1.],
            _ => [0., 1., 0.],
        };
        Expectation {
            root: "robot".into(),
            links: ["base_link", "link_j1", "link_upper_arm", "link_elbow", "link_forearm", "link_wrist", "tool_flange"].map(String::from).to_vec(),
            movable: JOINTS.iter().map(|j| j.0.to_string()).collect(),
            axes: JOINTS.iter().map(|j| (j.0.to_string(), axis(j.3))).collect::<BTreeMap<_, _>>(),
            above: vec![("tool_flange".into(), "link_elbow".into()), ("link_elbow".into(), "link_j1".into())],
            left_of: vec![],
            ahead_of: vec![],
        }
    }

    /// With J2 at 90 degrees (arm horizontal, forward) and the elbow straight: the tool
    /// deflection from bending of both tubes under their own weight, the distal links, the cup
    /// (0.05 kg, the cell's) and the declared payload. Euler-Bernoulli cantilevers, superposed;
    /// the upper arm's end slope carries the rest of the arm.
    fn requirements(&self, d: &DesignPoint) -> Result<Vec<Requirement>, FamilyError> {
        let [_, upper, fore] = self.tubes(d)?;
        let e = self.space.constant("tube.elastic_modulus")?;
        let g = self.space.constant("stiffness.gravity")?;
        let payload = self.space.constant("stiffness.payload")? + 0.05;
        let l2 = fore.length;
        // Point masses beyond each tube and their lever arms from the tube's end.
        let beyond_fore = [(0.8, WRIST / 2.), (0.3, WRIST + FLANGE / 2.), (payload, WRIST + FLANGE + 0.05)];
        let beyond_upper = [(2.0, ELBOW / 2.), (fore.mass(), ELBOW + l2 / 2.), (0.8, ELBOW + l2 + WRIST / 2.), (0.3, ELBOW + l2 + WRIST + FLANGE / 2.), (payload, ELBOW + l2 + WRIST + FLANGE + 0.05)];
        let cantilever = |tube: &Tube, loads: &[(f64, f64)]| {
            let (l, ei) = (tube.length, e * tube.area_moment());
            let w = tube.mass() * g / l;
            let p: f64 = loads.iter().map(|(m, _)| m * g).sum();
            let m: f64 = loads.iter().map(|(m, x)| m * g * x).sum();
            let deflection = w * l.powi(4) / (8. * ei) + p * l.powi(3) / (3. * ei) + m * l * l / (2. * ei);
            let slope = w * l.powi(3) / (6. * ei) + p * l * l / (2. * ei) + m * l / ei;
            (deflection, slope)
        };
        let (d1, slope1) = cantilever(&upper, &beyond_upper);
        let (d2, slope2) = cantilever(&fore, &beyond_fore);
        let tool = d1 + slope1 * (ELBOW + l2 + WRIST + FLANGE + 0.05) + d2 + slope2 * (WRIST + FLANGE + 0.05);
        let bound = self.space.constant("stiffness.max_deflection")?;
        if !tool.is_finite() {
            return Err(FamilyError::Rejected("static deflection is not finite".into()));
        }
        Ok(vec![Requirement { name: "static_tool_deflection".into(), value: tool, bound, unit: "m".into(), passed: tool <= bound }])
    }
}
