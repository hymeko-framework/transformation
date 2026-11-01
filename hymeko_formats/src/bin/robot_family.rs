//! Generate, check and prepare robot-family candidates through the public Formats API.
//!
//!   robot_family check  DATA FAMILY_FILE FAMILY LEGS|- DESIGN.json|default NEW_OUTPUT
//!   robot_family sample DATA FAMILY_FILE FAMILY LEGS|- SEED
//!   robot_family cell   DATA DESIGN.json|default NEW_OUTPUT
//!   robot_family workflow-corpus DATA NEW_OUTPUT
//!
//! DATA is the checkout's `data` directory; FAMILY_FILE is relative to it. `check` writes the
//! generated module, its MJCF and `candidate.json`; `cell` also prepares the serial_six_axis
//! pick-and-place bundle (`robot.xml`, `bundle.json`, `model.cbor`) for the Simulator. A refused
//! design or failed candidate exits with status 2 after writing `candidate.json`.
use hymeko_formats::robot_family;

use hymeko::writers::cbor_writer;
use hymeko::generators::GraphGenerator;
use robot_family::{
    DesignPoint, FamilyError, FamilySampler, ParameterSpace, RobotFamily,
    cell::{DesignCell, TASK_INPUTS},
    check::{Candidate, candidate},
    legged::LeggedPairs,
    manipulator::SerialSixAxis,
};
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, path::Path, process::ExitCode};

/// Input preparation only: source models and a bounded TSV list consumed by the
/// existing matched-output runner. No compilation is timed here.
struct WorkflowCorpus;

impl WorkflowCorpus {
    fn prepare(data: &Path, output: &Path) -> Result<(), Box<dyn Error>> {
        use robot_family::source::TreeRobotGenerator;
        if output.exists() { return Err("corpus output must be new".into()); }
        let vocabulary = std::fs::read(data.join("robotics/meta_kinematics.hymeko"))?;
        let space = ParameterSpace::load(&data.join("robotics/families/legged_family.hymeko"), "legged")?;
        let mut models = Vec::new();
        for (kind, branching) in [("chain", 1), ("tree", 3)] {
            for links in [10, 100, 1000] {
                let name = format!("{kind}_{links}");
                let source = TreeRobotGenerator::new(&name, "../meta_kinematics.hymeko", links, branching)?.generate()?;
                models.push((name, source));
            }
        }
        for legs in [2, 4, 6, 8] {
            let family = LeggedPairs::new(space.clone(), legs)?;
            models.push((family.module(), family.emit(&family.space().defaults())?));
        }
        std::fs::create_dir(output)?;
        std::fs::create_dir(output.join("cases"))?;
        std::fs::write(output.join("meta_kinematics.hymeko"), vocabulary)?;
        let mut manifest = String::from("label\tpath\trobot_name\n");
        for (name, source) in models {
            let relative = format!("cases/{name}.hymeko");
            std::fs::write(output.join(&relative), source)?;
            manifest.push_str(&format!("{name}\t{relative}\t{name}\n"));
        }
        std::fs::write(output.join("manifest.tsv"), manifest)?;
        println!("prepared {} (10 source models; no measurements)", output.display());
        Ok(())
    }
}

const MANIPULATOR: &str = "robotics/families/manipulator_family.hymeko";

fn design(space: &ParameterSpace, arg: &str) -> Result<DesignPoint, FamilyError> {
    if arg == "default" {
        return Ok(space.defaults());
    }
    let text = std::fs::read_to_string(arg).map_err(|e| FamilyError::Rejected(format!("{arg}: {e}")))?;
    let values: BTreeMap<String, f64> = serde_json::from_str(&text).map_err(|e| FamilyError::Rejected(format!("design JSON: {e}")))?;
    space.admit(&values)
}

fn report(c: &Candidate, space: &ParameterSpace) -> Value {
    json!({
        "family": space.family, "generator": space.generator, "module": c.generated.module,
        "design": c.generated.design, "unit_design": space.to_unit(&c.generated.design),
        "valid": c.valid(), "checks": c.checks, "requirements": c.requirements,
        "robot_digest": c.digest(), "robot_structure": c.structure(),
        "total_mass_kg": c.total_mass(), "links": c.links, "space": space,
    })
}

fn rejected(output: &Path, space: &ParameterSpace, reason: &FamilyError, partial: Option<Value>) -> Result<ExitCode, Box<dyn Error>> {
    std::fs::create_dir_all(output)?;
    let mut value = partial.unwrap_or_else(|| json!({"family": space.family, "space": space}));
    value["valid"] = json!(false);
    value["rejection"] = json!(reason.to_string());
    std::fs::write(output.join("candidate.json"), serde_json::to_vec_pretty(&value)?)?;
    eprintln!("rejected: {reason}");
    Ok(ExitCode::from(2))
}

fn family(data: &Path, file: &str, name: &str, legs: &str) -> Result<Box<dyn RobotFamily>, FamilyError> {
    let space = ParameterSpace::load(&data.join(file), name)?;
    Ok(match space.generator.as_str() {
        "serial_six_axis" => Box::new(SerialSixAxis::new(space)?),
        "legged_pairs" => Box::new(LeggedPairs::new(space, legs.parse().map_err(|_| FamilyError::Rejected(format!("leg count {legs}")))?)?),
        other => return Err(FamilyError::Declaration(format!("no generator {other}"))),
    })
}

fn check(data: &Path, family: &dyn RobotFamily, design_arg: &str, output: &Path) -> Result<(ExitCode, Option<Candidate>), Box<dyn Error>> {
    if output.exists() {
        return Err("output must be new".into());
    }
    let space = family.space().clone();
    let point = match design(&space, design_arg) {
        Ok(p) => p,
        Err(e) => return Ok((rejected(output, &space, &e, None)?, None)),
    };
    let c = match candidate(&Dyn(family), point, data, &output.join("sources")) {
        Ok(c) => c,
        Err(e) => return Ok((rejected(output, &space, &e, None)?, None)),
    };
    std::fs::create_dir_all(output)?;
    std::fs::write(output.join(format!("{}.hymeko", c.generated.module)), &c.generated.source)?;
    if let Ok(projection) = hymeko_formats::robot_projection::RobotProjection::new(&c.model, &family.expectation().root) {
        if let Ok(xml) = projection.mjcf() {
            std::fs::write(output.join("robot_only.xml"), xml)?;
        }
    }
    let value = report(&c, &space);
    std::fs::write(output.join("candidate.json"), serde_json::to_vec_pretty(&value)?)?;
    if !c.valid() {
        let failed: Vec<_> = c.checks.iter().filter(|k| !k.passed).map(|k| k.name.clone()).chain(c.requirements.iter().filter(|r| !r.passed).map(|r| r.name.clone())).collect();
        return Ok((rejected(output, &space, &FamilyError::Rejected(format!("failed: {failed:?}")), Some(value))?, None));
    }
    Ok((ExitCode::SUCCESS, Some(c)))
}

/// Borrowed dynamic family, so the generic generator contracts accept a trait object.
struct Dyn<'a>(&'a dyn RobotFamily);
impl RobotFamily for Dyn<'_> {
    fn space(&self) -> &ParameterSpace {
        self.0.space()
    }
    fn module(&self) -> String {
        self.0.module()
    }
    fn emit(&self, d: &DesignPoint) -> Result<String, FamilyError> {
        self.0.emit(d)
    }
    fn expectation(&self) -> robot_family::Expectation {
        self.0.expectation()
    }
    fn requirements(&self, d: &DesignPoint) -> Result<Vec<robot_family::Requirement>, FamilyError> {
        self.0.requirements(d)
    }
    fn link_specs(&self, d: &DesignPoint) -> Result<Vec<robot_family::LinkSpec>, FamilyError> {
        self.0.link_specs(d)
    }
}

fn main() -> Result<ExitCode, Box<dyn Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let usage = "usage: robot_family check DATA FAMILY_FILE FAMILY LEGS|- DESIGN|default NEW_OUTPUT | sample DATA FAMILY_FILE FAMILY LEGS|- SEED | cell DATA DESIGN|default NEW_OUTPUT | workflow-corpus DATA NEW_OUTPUT";
    match args.first().map(String::as_str) {
        Some("workflow-corpus") if args.len() == 3 => {
            WorkflowCorpus::prepare(Path::new(&args[1]), Path::new(&args[2]))?;
            Ok(ExitCode::SUCCESS)
        }
        Some("check") if args.len() == 7 => {
            let data = Path::new(&args[1]);
            let f = family(data, &args[2], &args[3], &args[4])?;
            Ok(check(data, f.as_ref(), &args[5], Path::new(&args[6]))?.0)
        }
        Some("sample") if args.len() == 6 => {
            let f = family(Path::new(&args[1]), &args[2], &args[3], &args[4])?;
            let point = FamilySampler::new(&Dyn(f.as_ref()), args[5].parse()?).design()?;
            println!("{}", serde_json::to_string(point.values())?);
            Ok(ExitCode::SUCCESS)
        }
        Some("cell") if args.len() == 4 => {
            let data = Path::new(&args[1]);
            let output = Path::new(&args[3]);
            let f = family(data, MANIPULATOR, "serial_six_axis", "-")?;
            let (code, c) = check(data, f.as_ref(), &args[2], output)?;
            let Some(c) = c else { return Ok(code) };
            let space = f.space().clone();
            let cell = match DesignCell::prepare(&c, data, &output.join("cell_sources")).and_then(|cell| cell.artifacts().map(|a| (cell, a))) {
                Ok(v) => v,
                Err(e) => {
                    let mut value = report(&c, &space);
                    value["valid"] = json!(false);
                    value["rejection"] = json!(e.to_string());
                    std::fs::write(output.join("candidate.json"), serde_json::to_vec_pretty(&value)?)?;
                    eprintln!("rejected: {e}");
                    return Ok(ExitCode::from(2));
                }
            };
            let (cell, (xml, mut manifest)) = cell;
            // Installation envelope: the flange's far face above the floor with the arm stretched up.
            let projection = hymeko_formats::robot_projection::RobotProjection::new(&c.model, "robot")?;
            let zero = projection.model().joints.iter().map(|j| (j.name.clone(), 0.)).collect();
            let poses = hymeko_mechanism::forward::forward_kinematics(projection.model(), &zero)?;
            let envelope = poses.get("tool_flange").ok_or("tool_flange not posed")?.translation[2] + 0.02;
            manifest["design"] = json!({"family": space.family, "values": c.generated.design.values(), "envelope_m": envelope, "unit": space.to_unit(&c.generated.design), "robot_digest": c.digest(), "robot_structure": c.structure(), "total_mass_kg": c.total_mass(), "requirements": c.requirements});
            manifest["stations"] = json!(cell.stations);
            manifest["objective"] = json!(cell.objective()?);
            manifest["inputs"] = json!(TASK_INPUTS.iter().chain([&MANIPULATOR, &"robotics/families/meta_robot_family.hymeko"]).collect::<Vec<_>>());
            let program = cell.bundle.model().program();
            let cbor = cbor_writer::encode_compiled(&output.join("cell_sources"), program, cell.bundle.model().interner())?;
            std::fs::write(output.join("robot.xml"), xml)?;
            std::fs::write(output.join("bundle.json"), serde_json::to_vec_pretty(&manifest)?)?;
            std::fs::write(output.join("model.cbor"), cbor)?;
            println!("prepared {}", output.display());
            Ok(ExitCode::SUCCESS)
        }
        _ => Err(usage.into()),
    }
}
