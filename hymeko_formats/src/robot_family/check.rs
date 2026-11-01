//! Compile a generated robot with Core and check it against its family's promises.
//!
//! The candidate is written with the checkout's vocabulary into its own snapshot directory and
//! compiled there by Core, then read back through Geometry and Formats exactly as an authored robot would be. A check that
//! fails is reported by name with its evidence; [`Candidate::valid`] is their conjunction.
use super::{Expectation, FamilyError, FamilyGenerator, GeneratedRobot, LinkSpec, Requirement, RobotFamily};
use crate::robot_projection::RobotProjection;
use hymeko::{generators::GraphGenerator, ir::ir::ValueR, module_store::execution::CompiledModel};
use hymeko_mechanism::{
    forward::{JointValues, forward_kinematics},
    kinematics::{joints::JointType, kinematic::GeometryShape},
};
use hymeko_query::{Predicate, QueryEngine};
use serde::Serialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    path::Path,
};

/// Engineering-material envelope for a link's mean density (kg/m^3): foam-filled shells to steel.
pub const DENSITY_RANGE: [f64; 2] = [100., 8000.];
const RELATIVE: f64 = 1e-9;

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Check {
    pub name: String,
    pub passed: bool,
    pub detail: String,
}

/// A compiled candidate and its evidence. The Core snapshot stays owned for downstream reads.
pub struct Candidate {
    pub generated: GeneratedRobot,
    pub model: CompiledModel,
    pub checks: Vec<Check>,
    pub requirements: Vec<Requirement>,
    pub links: Vec<LinkSpec>,
}

impl Candidate {
    pub fn valid(&self) -> bool {
        self.checks.iter().all(|c| c.passed) && self.requirements.iter().all(|r| r.passed)
    }
    pub fn digest(&self) -> String {
        format!("{:?}", self.model.semantic_digest())
    }
    pub fn structure(&self) -> String {
        format!("{:?}", self.model.program().canon.structure())
    }
    pub fn total_mass(&self) -> f64 {
        self.links.iter().map(|l| l.mass).sum()
    }
}

/// The vocabulary path a generated module imports, relative to `robotics/families`.
pub fn vocabulary(data: &Path) -> Result<String, FamilyError> {
    std::fs::read_to_string(data.join("robotics/meta_kinematics.hymeko")).map_err(|e| FamilyError::Declaration(format!("meta_kinematics.hymeko: {e}")))
}

/// Generate, compile and check one design. `data` is the checkout's `data` directory; the
/// compiled sources are kept under the new directory `workspace`.
pub fn candidate<F: RobotFamily>(family: &F, design: super::DesignPoint, data: &Path, workspace: &Path) -> Result<Candidate, FamilyError> {
    let generated = FamilyGenerator { family, design }.generate()?;
    let path = format!("robotics/families/{}.hymeko", generated.module);
    let files = BTreeMap::from([("robotics/meta_kinematics.hymeko".to_string(), vocabulary(data)?), (path.clone(), generated.source.clone())]);
    let model = super::compile_snapshot(workspace, &files, &path)?;
    let links = family.link_specs(&generated.design)?;
    let requirements = family.requirements(&generated.design)?;
    let checks = robot_checks(&model, &family.expectation(), &links);
    Ok(Candidate { generated, model, checks, requirements, links })
}

fn add(out: &mut Vec<Check>, name: &str, passed: bool, detail: String) {
    out.push(Check { name: name.into(), passed, detail });
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= RELATIVE * a.abs().max(b.abs()).max(1e-12)
}

/// Every check, on the model as Geometry and Formats project it.
pub fn robot_checks(model: &CompiledModel, expect: &Expectation, specs: &[LinkSpec]) -> Vec<Check> {
    let mut out = Vec::new();
    let projection = match RobotProjection::new(model, &expect.root) {
        Ok(p) => p,
        Err(e) => {
            add(&mut out, "kinematic_tree", false, e.to_string());
            return out;
        }
    };
    let k = projection.model();
    add(&mut out, "kinematic_tree", true, format!("{} links, {} joints, one rooted tree (FkPlan)", k.links.len(), k.joints.len()));

    let names: BTreeSet<_> = k.links.iter().map(|l| l.name.clone()).collect();
    let wanted: BTreeSet<_> = expect.links.iter().cloned().collect();
    let dangling: Vec<_> = k.joints.iter().filter(|j| j.parent_link == "unknown" || j.child_link == "unknown").map(|j| j.name.clone()).collect();
    add(&mut out, "references", names == wanted && dangling.is_empty(), format!("links match: {}; unresolved joint ends: {dangling:?}", names == wanted));

    let movable: Vec<_> = k.joints.iter().filter(|j| j.joint_type != JointType::Fixed).collect();
    let movable_names: BTreeSet<_> = movable.iter().map(|j| j.name.clone()).collect();
    add(&mut out, "movable_joints", movable_names == expect.movable.iter().cloned().collect(), format!("{} movable", movable.len()));

    let bad_axes: Vec<_> = movable.iter().filter(|j| j.axis != expect.axes.get(&j.name).copied()).map(|j| format!("{}: {:?}", j.name, j.axis)).collect();
    add(&mut out, "axes", bad_axes.is_empty(), format!("off-convention axes: {bad_axes:?}"));

    let bad_limits: Vec<_> = movable
        .iter()
        .filter(|j| j.limits.as_ref().is_none_or(|l| !(l.lower.is_finite() && l.upper.is_finite() && l.lower < l.upper && l.effort > 0. && l.velocity > 0.)))
        .map(|j| j.name.clone())
        .collect();
    add(&mut out, "joint_and_actuator_limits", bad_limits.is_empty(), format!("missing or invalid limits: {bad_limits:?}"));

    add(&mut out, "units", declared_units(model), "meta_kinematics.units: length m, angle degree, mass kg, time s".into());

    let mut mass = Vec::new();
    let mut inertia = Vec::new();
    let mut density = Vec::new();
    let mut geometry = Vec::new();
    for spec in specs {
        let Some(link) = k.links.iter().find(|l| l.name == spec.name) else {
            mass.push(format!("{} absent", spec.name));
            continue;
        };
        if !link.mass.is_some_and(|m| m > 0. && close(m, spec.mass)) {
            mass.push(format!("{}: {:?} vs {}", spec.name, link.mass, spec.mass));
        }
        let declared = link.inertia_com;
        if !link.valid_inertia() || !declared.zip(spec.inertia).is_some_and(|(a, b)| a.iter().zip(b).all(|(x, y)| close(*x, y))) {
            inertia.push(spec.name.clone());
        }
        let rho = spec.mass / spec.shape.volume();
        if !(DENSITY_RANGE[0]..=DENSITY_RANGE[1]).contains(&rho) {
            density.push(format!("{}: {rho:.1} kg/m^3", spec.name));
        }
        let shape_ok = link.geometry.as_ref().is_some_and(|g| {
            let kind = matches!(
                (g.shape, spec.shape),
                (GeometryShape::Box, super::Shape::Box(_))
                    | (GeometryShape::Cylinder, super::Shape::Cylinder { .. })
                    | (GeometryShape::Sphere, super::Shape::Sphere(_))
                    | (GeometryShape::Capsule, super::Shape::Capsule { .. })
            );
            kind && g.dimensions.len() == spec.shape.dimension().len() && g.dimensions.iter().zip(spec.shape.dimension()).all(|(a, b)| close(*a, b))
        }) && link.origin.as_ref().is_some_and(|o| o.len() == 3 && o.iter().zip(spec.origin).all(|(a, b)| close(*a, b)));
        if !shape_ok {
            geometry.push(spec.name.clone());
        }
    }
    add(&mut out, "mass_model", mass.is_empty(), format!("mismatches: {mass:?}"));
    add(&mut out, "inertia", inertia.is_empty(), format!("invalid or not the mass model's: {inertia:?}"));
    add(&mut out, "density", density.is_empty(), format!("outside {DENSITY_RANGE:?}: {density:?}"));
    add(&mut out, "geometry", geometry.is_empty(), format!("shape/extent/origin mismatches: {geometry:?}"));

    let zero: JointValues = k.joints.iter().map(|j| (j.name.clone(), 0.)).collect();
    match forward_kinematics(k, &zero) {
        Ok(poses) => {
            let at = |l: &str| poses.get(l).map(|p| [p.translation[0], p.translation[1], p.translation[2]]);
            let mut wrong = Vec::new();
            for (axis, label, pairs) in [(2, "above", &expect.above), (1, "left of", &expect.left_of), (0, "ahead of", &expect.ahead_of)] {
                for (a, b) in pairs {
                    if !at(a).zip(at(b)).is_some_and(|(pa, pb)| pa[axis] > pb[axis]) {
                        wrong.push(format!("{a} not {label} {b}"));
                    }
                }
            }
            add(&mut out, "frame_convention", wrong.is_empty(), format!("X forward, Y left, Z up at the zero pose: {wrong:?}"));
        }
        Err(e) => add(&mut out, "frame_convention", false, format!("{e:?}")),
    }

    match projection.mjcf() {
        Ok(xml) => {
            let missing: Vec<_> = expect.links.iter().filter(|l| !xml.contains(&format!("<body name=\"{l}\""))).chain(expect.movable.iter().filter(|j| !xml.contains(&format!("<joint name=\"{j}\"")))).cloned().collect();
            add(&mut out, "mjcf_export", missing.is_empty(), format!("{} bytes; missing bodies/joints: {missing:?}", xml.len()));
        }
        Err(e) => add(&mut out, "mjcf_export", false, e.to_string()),
    }
    out
}

/// The vocabulary's declared unit system, read from the compiled model.
fn declared_units(model: &CompiledModel) -> bool {
    let query = QueryEngine::new(&model.program().ir, model.interner());
    let rows = query.query(&Predicate::Named("units".into()));
    rows.len() == 1
        && [("length", "m"), ("angle", "degree"), ("mass", "kg"), ("time", "s")]
            .iter()
            .all(|(field, unit)| matches!(query.effective_value(rows[0].id, field), Some(ValueR::Str(s)) if query.resolver().resolve(*s) == *unit))
}
