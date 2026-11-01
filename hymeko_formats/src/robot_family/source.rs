//! HyMeKo robot source over the `meta_kinematics` vocabulary, written declaration by declaration.
//!
//! This is the generation primitive the retired Python scaling generator
//! (`hymeko_framework_rust/scripts/scaling/generate_fixtures.py`: `HEADER_TMPL`, `link_block`,
//! `revolute_joint_block`, `FOOTER`) and the 2026-10-15 robot-generation cases built on. The
//! surface form is the same; numbers are written at full round-trip precision instead of three
//! decimals, so a generated design is the design that was asked for.
use std::fmt::Write;
use hymeko::generators::GraphGenerator;
use super::FamilyError;

/// A deterministic rooted kinematic tree, emitted through the same source builder
/// as the declared robot families. Branching one gives a serial chain.
pub struct TreeRobotGenerator {
    root: String,
    vocabulary: String,
    links: usize,
    branching: usize,
}

impl TreeRobotGenerator {
    /// Admit 1..=5000 links and 1..=8 children per parent. Source identifiers and
    /// the quoted vocabulary path must be representable without escaping.
    pub fn new(root: &str, vocabulary: &str, links: usize, branching: usize) -> Result<Self, FamilyError> {
        let identifier = !root.is_empty()
            && root.bytes().all(|c| c.is_ascii_alphanumeric() || c == b'_')
            && !root.as_bytes()[0].is_ascii_digit();
        if !identifier || vocabulary.is_empty() || vocabulary.chars().any(|c| c == '"' || c.is_control())
            || !(1..=5000).contains(&links) || !(1..=8).contains(&branching) {
            return Err(FamilyError::Declaration("invalid tree name, vocabulary, link count or branching".into()));
        }
        Ok(Self { root: root.into(), vocabulary: vocabulary.into(), links, branching })
    }
}

impl GraphGenerator for TreeRobotGenerator {
    type Output = String;
    type Error = FamilyError;

    fn generate(&self) -> Result<String, FamilyError> {
        let mut source = RobotSource::new(&self.root, &self.vocabulary);
        source.frame("world");
        let shape = Shape::Box([0.1, 0.08, 0.08]);
        for index in 0..self.links {
            source.link(&LinkSpec { name: format!("link_{index}"), mass: 1., shape,
                origin: [0.; 3], inertia: Some(shape.solid_inertia(1.)), color: None });
        }
        source.joint("base", "world", "link_0", JointKind::Fixed, [0., 0., 0.2], [0.; 3]);
        let limits = Limits { lower_deg: -90., upper_deg: 90., effort: 40., velocity: 8. };
        for index in 1..self.links {
            let parent = (index - 1) / self.branching;
            let offset = ((index - 1) % self.branching) as f64 - (self.branching - 1) as f64 / 2.;
            source.joint(&format!("joint_{index}"), &format!("link_{parent}"), &format!("link_{index}"),
                JointKind::Revolute { axis: "AXIS_Y", limits }, [0.2, 0.15 * offset, 0.], [0.; 3]);
        }
        Ok(source.finish())
    }
}

/// A link primitive and its `dimension` list, in the vocabulary's order: box full extents
/// `[x, y, z]`, cylinder `[radius, length]` along local z, sphere `[radius]`, capsule
/// `[segment length, radius]` (Geometry's capsule contract).
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub enum Shape {
    Box([f64; 3]),
    Cylinder { radius: f64, length: f64 },
    Sphere(f64),
    Capsule { length: f64, radius: f64 },
}

impl Shape {
    fn keyword(&self) -> &'static str {
        match self {
            Self::Box(_) => "box",
            Self::Cylinder { .. } => "cylinder",
            Self::Sphere(_) => "sphere",
            Self::Capsule { .. } => "capsule",
        }
    }
    pub fn dimension(&self) -> Vec<f64> {
        match *self {
            Self::Box(d) => d.to_vec(),
            Self::Cylinder { radius, length } => vec![radius, length],
            Self::Sphere(r) => vec![r],
            Self::Capsule { length, radius } => vec![length, radius],
        }
    }
    pub fn volume(&self) -> f64 {
        match *self {
            Self::Box([x, y, z]) => x * y * z,
            Self::Cylinder { radius, length } => std::f64::consts::PI * radius * radius * length,
            Self::Sphere(r) => 4. / 3. * std::f64::consts::PI * r * r * r,
            Self::Capsule { length, radius: r } => std::f64::consts::PI * r * r * (length + 4. / 3. * r),
        }
    }
    /// COM inertia `[xx, yy, zz, xy, xz, yz]` of a uniform solid of mass `m`.
    pub fn solid_inertia(&self, m: f64) -> [f64; 6] {
        match *self {
            Self::Box([x, y, z]) => [m * (y * y + z * z) / 12., m * (x * x + z * z) / 12., m * (x * x + y * y) / 12., 0., 0., 0.],
            Self::Cylinder { radius: r, length: h } => {
                let side = m * (3. * r * r + h * h) / 12.;
                [side, side, m * r * r / 2., 0., 0., 0.]
            }
            Self::Sphere(r) => {
                let i = 0.4 * m * r * r;
                [i, i, i, 0., 0., 0.]
            }
            Self::Capsule { length: h, radius: r } => {
                // Uniform density: a cylinder and two hemispheres whose centres sit at the segment ends.
                let (cylinder, cap) = (h, 2. / 3. * r);
                let (mc, mh) = (m * cylinder / (cylinder + 2. * cap), m * cap / (cylinder + 2. * cap));
                let side = mc * (h * h / 12. + r * r / 4.) + 2. * mh * (0.4 * r * r + h * h / 4. + 3. * h * r / 8.);
                [side, side, mc * r * r / 2. + 2. * mh * 0.4 * r * r, 0., 0., 0.]
            }
        }
    }
}

/// One link as the vocabulary declares it. `inertia` is the COM inertia in link axes.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct LinkSpec {
    pub name: String,
    pub mass: f64,
    pub shape: Shape,
    pub origin: [f64; 3],
    pub inertia: Option<[f64; 6]>,
    pub color: Option<String>,
}

/// Position stops in degrees (the source convention), effort in N m, velocity in rad/s.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct Limits {
    pub lower_deg: f64,
    pub upper_deg: f64,
    pub effort: f64,
    pub velocity: f64,
}

/// `axis` names a `meta_kinematics.kinematics.axes` declaration, e.g. `AXIS_Y`.
#[derive(Debug, Clone, PartialEq)]
pub enum JointKind<'a> {
    Fixed,
    Continuous { axis: &'a str },
    Revolute { axis: &'a str, limits: Limits },
}

/// Builds one robot module. `finish` closes the root declaration.
pub struct RobotSource {
    text: String,
}

/// Shortest round-trip decimal; HyMeKo has no exponent notation and Rust's `Display` writes none.
pub fn number(x: f64) -> String {
    assert!(x.is_finite(), "robot source numbers are finite");
    if x == 0. { "0".into() } else { format!("{x}") }
}

fn list(values: &[f64]) -> String {
    values.iter().map(|v| number(*v)).collect::<Vec<_>>().join(", ")
}

impl RobotSource {
    /// The description header importing `vocabulary` and the root's three vocabulary bases.
    pub fn new(root: &str, vocabulary: &str) -> Self {
        Self {
            text: format!(
                "{root}_description {{\n    @\"{vocabulary}\";\n}}\n\n{root}: meta_kinematics.kinematics.elements,\n        meta_kinematics.kinematics.geometry,\n        meta_kinematics.kinematics.axes\n{{\n"
            ),
        }
    }

    pub fn frame(&mut self, name: &str) -> &mut Self {
        writeln!(self.text, "    {name}: meta_kinematics.kinematics.elements.frame {{}}").unwrap();
        self
    }

    pub fn constant(&mut self, name: &str, values: &[f64]) -> &mut Self {
        writeln!(self.text, "    {name} [{}];", list(values)).unwrap();
        self
    }

    pub fn link(&mut self, link: &LinkSpec) -> &mut Self {
        let t = &mut self.text;
        writeln!(t, "    {}: meta_kinematics.kinematics.elements.link {{", link.name).unwrap();
        writeln!(t, "        mass {};", number(link.mass)).unwrap();
        writeln!(t, "        link_geometry: meta_kinematics.kinematics.geometry.{} {{", link.shape.keyword()).unwrap();
        writeln!(t, "            dimension [{}];", list(&link.shape.dimension())).unwrap();
        t.push_str("        }\n        visual    -> link_geometry;\n        collision -> link_geometry;\n");
        writeln!(t, "        origin [{}];", list(&link.origin)).unwrap();
        if let Some(color) = &link.color {
            writeln!(t, "        color -> {color};").unwrap();
        }
        if let Some([xx, yy, zz, xy, xz, yz]) = link.inertia {
            let n = number;
            writeln!(t, "        inertia_com {{ xx {}; yy {}; zz {}; xy {}; xz {}; yz {}; }}", n(xx), n(yy), n(zz), n(xy), n(xz), n(yz)).unwrap();
        }
        t.push_str("    }\n");
        self
    }

    /// A joint mounted at `xyz` (m) and `rpy` (degrees) in the parent frame.
    pub fn joint(&mut self, name: &str, parent: &str, child: &str, kind: JointKind<'_>, xyz: [f64; 3], rpy: [f64; 3]) -> &mut Self {
        let t = &mut self.text;
        let mount = format!("[[{}], [{}]]", list(&xyz), list(&rpy));
        match kind {
            JointKind::Fixed => {
                writeln!(t, "    @{name}: meta_kinematics.kinematics.fixed_joint {{ (+ {parent} {mount}, - {child}); }}").unwrap();
            }
            JointKind::Continuous { axis } => {
                writeln!(t, "    @{name}: meta_kinematics.kinematics.conti_joint {{\n        (+ {parent} {mount},\n         - {child},\n         - meta_kinematics.kinematics.axes.{axis});\n    }}").unwrap();
            }
            JointKind::Revolute { axis, limits } => {
                writeln!(t, "    @{name}: meta_kinematics.kinematics.rev_joint {{\n        (+ {parent} {mount},\n         - {child},\n         - meta_kinematics.kinematics.axes.{axis});").unwrap();
                writeln!(
                    t,
                    "        limit_lower {}; limit_upper {}; limit_effort {}; limit_velocity {};\n    }}",
                    number(limits.lower_deg),
                    number(limits.upper_deg),
                    number(limits.effort),
                    number(limits.velocity)
                )
                .unwrap();
            }
        }
        self
    }

    pub fn finish(mut self) -> String {
        self.text.push_str("}\n");
        self.text
    }
}
