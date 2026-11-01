//! The generated cell's pick-and-place task for a `serial_six_axis` candidate.
//!
//! The task (`robotics/families/serial_six_axis_cell.hymeko`) fixes the stations in Cartesian
//! space and imports the candidate robot module. For each candidate, Geometry's IK finds the
//! joint targets that put the TCP on the same stations, and Core's checked value update writes
//! them into the task's `joint_target.q` leaves; [`SimulationBundle`] then admits the case and
//! emits the robot and manifest for the Simulator, as for the authored cell.
use super::{FamilyError, check::Candidate};
use crate::{robot_projection::RobotProjection, traceability::simulation::SimulationBundle};
use hymeko::{
    common::ids::DeclId,
    ir::ir::ValueR,
    module_store::execution::{CompiledModel, ValueUpdate},
};
use hymeko_geometry::kinematics::ik::{IkOptions, ModelChain, solve_serial};
use hymeko_query::{Predicate, QueryEngine};
use serde::Serialize;
use serde_json::Value;
use std::{collections::BTreeMap, path::Path};

pub const TASK: &str = "robotics/families/serial_six_axis_cell.hymeko";
/// Files the task compiles from, besides the generated robot; paths relative to `data`.
pub const TASK_INPUTS: [&str; 5] = [TASK, "robotics/meta_kinematics.hymeko", "paper/meta_simulation_case.hymeko", "paper/traceability_verification.hymeko", "paper/meta_sysml_trace.hymeko"];
const JOINTS: [&str; 6] = ["j1", "j2", "j3", "j4", "j5", "j6"];
/// IK must place the TCP on the station this closely; MissionMotion then tracks these targets.
pub const STATION_TOLERANCE: [f64; 2] = [1e-7, 1e-6];

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct StationSolution {
    pub target: String,
    pub station: String,
    pub tcp: Vec<f64>,
    pub reference_q: Vec<f64>,
    pub q: Vec<f64>,
    pub position_error_m: f64,
    pub orientation_error_rad: f64,
    pub iterations: usize,
    pub converged: bool,
}

pub struct DesignCell {
    pub bundle: SimulationBundle,
    pub stations: Vec<StationSolution>,
    /// The robot module text and every task input, as compiled.
    pub sources: BTreeMap<String, String>,
}

impl DesignCell {
    /// Compile the task over a valid candidate in the new snapshot directory `workspace`, solve
    /// the stations and admit the case.
    pub fn prepare(candidate: &Candidate, data: &Path, workspace: &Path) -> Result<Self, FamilyError> {
        if !candidate.valid() {
            return Err(FamilyError::Rejected("candidate checks or static requirements failed".into()));
        }
        let mut sources = BTreeMap::new();
        for input in TASK_INPUTS {
            let text = std::fs::read_to_string(data.join(input)).map_err(|e| FamilyError::Declaration(format!("{input}: {e}")))?;
            sources.insert(input.to_string(), text);
        }
        sources.insert(format!("robotics/families/{}.hymeko", candidate.generated.module), candidate.generated.source.clone());
        let model = super::compile_snapshot(workspace, &sources, TASK)?;
        let (updated, stations) = solve_stations(&model)?;
        let bundle = SimulationBundle::new(updated, "experiment").map_err(FamilyError::Projection)?;
        Ok(Self { bundle, stations, sources })
    }

    /// The task's declared design objective (`design_types.objective`), field by field.
    pub fn objective(&self) -> Result<BTreeMap<String, f64>, FamilyError> {
        let model = self.bundle.model();
        let read = Reader { query: QueryEngine::new(&model.program().ir, model.interner()) };
        let id = read.named("design_objective")?;
        read.query
            .effective_fields(id)
            .iter()
            .map(|(name, _)| match read.query.effective_value(id, name) {
                Some(ValueR::Num(v)) if v.is_finite() && *v >= 0. => Ok((name.to_string(), *v)),
                _ => Err(FamilyError::Declaration(format!("objective field {name} must be a nonnegative number"))),
            })
            .collect()
    }

    /// The Simulator's bundle, with the TCP of every target re-derived through the bundle's own
    /// forward kinematics and compared with its station.
    pub fn artifacts(&self) -> Result<(String, Value), FamilyError> {
        let (xml, manifest) = self.bundle.artifacts().map_err(FamilyError::Projection)?;
        let offset: Vec<f64> = manifest["scene"]["tcp_offset"].as_array().into_iter().flatten().filter_map(Value::as_f64).collect();
        let targets = std::iter::once(&manifest["initial"]).chain(manifest["targets"].as_array().into_iter().flatten());
        for (target, station) in targets.zip(&self.stations) {
            let pose = &target["tool_pose"];
            let p: Vec<f64> = pose["pos"].as_array().into_iter().flatten().filter_map(Value::as_f64).collect();
            let r: Vec<f64> = pose["mat"].as_array().into_iter().flatten().filter_map(Value::as_f64).collect();
            if p.len() != 3 || r.len() != 9 || offset.len() != 3 || target["name"] != station.target.as_str() {
                return Err(FamilyError::Projection("bundle target poses are incomplete".into()));
            }
            let gap = (0..3).map(|i| (p[i] + (0..3).map(|j| r[i * 3 + j] * offset[j]).sum::<f64>() - station.tcp[i]).powi(2)).sum::<f64>().sqrt();
            if gap > 1e-6 {
                return Err(FamilyError::Projection(format!("target {} misses station {} by {gap:.3e} m", station.target, station.station)));
            }
        }
        Ok((xml, manifest))
    }
}

struct Reader<'a> {
    query: QueryEngine<'a, hymeko::resolution::interner::Interner>,
}

impl<'a> Reader<'a> {
    fn named(&self, name: &str) -> Result<DeclId, FamilyError> {
        match self.query.query(&Predicate::Named(name.into())).as_slice() {
            [row] => Ok(row.id),
            _ => Err(FamilyError::Declaration(format!("{name} absent or ambiguous"))),
        }
    }
    fn name(&self, id: DeclId) -> String {
        self.query.resolver().resolve(self.query.ir().decl_nodes[id.raw()].name).to_owned()
    }
    fn reference(&self, id: DeclId, field: &str) -> Result<DeclId, FamilyError> {
        match self.query.effective_value(id, field) {
            Some(ValueR::Ref(v)) => Ok(*v),
            _ => Err(FamilyError::Declaration(format!("{} needs a reference {field}", self.name(id)))),
        }
    }
    fn references(&self, id: DeclId, field: &str) -> Result<Vec<DeclId>, FamilyError> {
        match self.query.effective_value(id, field) {
            Some(ValueR::List(items)) => items.iter().map(|v| if let ValueR::Ref(d) = v { Ok(*d) } else { Err(FamilyError::Declaration(format!("{field} must list references"))) }).collect(),
            _ => Err(FamilyError::Declaration(format!("{} needs a reference list {field}", self.name(id)))),
        }
    }
    fn numbers(&self, id: DeclId, field: &str) -> Result<Vec<f64>, FamilyError> {
        match self.query.effective_value(id, field) {
            Some(ValueR::List(items)) => items
                .iter()
                .map(|v| match v {
                    ValueR::Num(x) if x.is_finite() => Ok(*x),
                    _ => Err(FamilyError::Declaration(format!("{field} must be finite numbers"))),
                })
                .collect(),
            _ => Err(FamilyError::Declaration(format!("{} needs a number list {field}", self.name(id)))),
        }
    }
}

/// IK for every target's station, written back through Core's checked value update.
fn solve_stations(model: &CompiledModel) -> Result<(CompiledModel, Vec<StationSolution>), FamilyError> {
    let read = Reader { query: QueryEngine::new(&model.program().ir, model.interner()) };
    let case = read.named("experiment")?;
    let scene = read.reference(case, "environment")?;
    let offset = read.numbers(scene, "tcp_offset")?;
    let targets: Vec<DeclId> = std::iter::once(read.reference(case, "initial")).chain(read.references(case, "targets")?.into_iter().map(Ok)).collect::<Result<_, _>>()?;
    let projection = RobotProjection::new(model, "robot").map_err(|e| FamilyError::Projection(e.to_string()))?;
    let kinematics = projection.model();
    let chain = ModelChain::new(kinematics, &JOINTS, "tool_flange").map_err(|e| FamilyError::Projection(e.to_string()))?;
    let limits: Vec<[f64; 2]> = JOINTS
        .iter()
        .map(|name| kinematics.joints.iter().find(|j| j.name == *name).and_then(|j| j.limits.as_ref()).map(|l| [l.lower, l.upper]))
        .collect::<Option<_>>()
        .ok_or_else(|| FamilyError::Projection("every arm joint needs position limits".into()))?;
    let options = IkOptions { iterations: 400, tolerance: STATION_TOLERANCE, ..IkOptions::default() };
    let mut updates = Vec::new();
    let mut solutions = Vec::new();
    let mut written = std::collections::HashSet::new();
    for target in targets {
        let station = read.reference(target, "station")?;
        let (tcp, axes) = (read.numbers(station, "tcp")?, read.numbers(station, "tool_axes")?);
        let reference_q = read.numbers(target, "q")?;
        if tcp.len() != 3 || axes.len() != 9 || offset.len() != 3 || reference_q.len() != 6 {
            return Err(FamilyError::Declaration("a station needs tcp[3] and tool_axes[9]; a target q[6]".into()));
        }
        let rotation: [f64; 9] = std::array::from_fn(|i| axes[i]);
        let flange: [f64; 3] = std::array::from_fn(|i| tcp[i] - (0..3).map(|j| rotation[i * 3 + j] * offset[j]).sum::<f64>());
        let solution = solve_serial(&chain, &limits, &reference_q, &flange, Some(&rotation), &options).map_err(|e| FamilyError::Projection(e.to_string()))?;
        let entry = StationSolution {
            target: read.name(target),
            station: read.name(station),
            tcp: tcp.clone(),
            reference_q,
            q: solution.coordinates.clone(),
            position_error_m: solution.position_error_m,
            orientation_error_rad: solution.orientation_error_rad,
            iterations: solution.iterations,
            converged: solution.converged,
        };
        if !solution.converged {
            return Err(FamilyError::Rejected(format!("station {} unreachable: {:.3e} m, {:.3e} rad after {} iterations", entry.station, entry.position_error_m, entry.orientation_error_rad, entry.iterations)));
        }
        if written.insert(target) {
            let slot = read.query.own_child(target, "q").ok_or_else(|| FamilyError::Declaration("joint_target.q is not an own leaf".into()))?;
            updates.push(ValueUpdate::Set { declaration: slot, value: ValueR::List(solution.coordinates.iter().map(|v| ValueR::Num(*v)).collect()) });
        }
        solutions.push(entry);
    }
    let updated = model.with_value_updates(&updates).map_err(|e| FamilyError::Projection(format!("checked joint-target update: {e}")))?;
    Ok((updated, solutions))
}
