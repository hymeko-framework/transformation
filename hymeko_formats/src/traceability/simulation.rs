//! A source-declared simulation/evidence bundle. Physics remains in the Simulator consumer.
use crate::{robot_projection::RobotProjection, traceability::TraceabilityExport};
use hymeko::{
    common::ids::DeclId,
    ir::ir::ValueR,
    module_store::execution::{CompiledModel, ValueUpdate},
};
use hymeko_query::{Predicate, QueryEngine};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::collections::{BTreeMap, HashSet};

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Measurement {
    StationErrorM,
    CycleTimeS,
    RepeatabilityM,
    StopLatencyS,
    IdealGripRetained,
    DiagnosticsComplete,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Comparison {
    Max,
    Min,
}

#[derive(Debug, Clone, Serialize)]
pub struct ObservationRule {
    pub name: String,
    pub test: String,
    pub metric: Measurement,
    pub comparison: Comparison,
    pub bound: f64,
    #[serde(skip)]
    slot: DeclId,
}

impl ObservationRule {
    fn outcome(&self, reading: Option<&Value>) -> Result<i8, String> {
        let Some(value) = reading else {
            return Ok(-1);
        };
        let finite = |value: &Value| {
            value
                .as_f64()
                .filter(|v| v.is_finite() && *v >= 0.)
                .ok_or_else(|| "measurement must be finite and nonnegative".to_string())
        };
        let (lower, upper) = if value.is_number() {
            let v = finite(value)?;
            (Some(v), Some(v))
        } else {
            let bounds = value
                .as_object()
                .filter(|v| v.len() == 1)
                .ok_or("one explicit censoring bound required")?;
            if let Some(v) = bounds.get("lower_bound") {
                (Some(finite(v)?), None)
            } else if let Some(v) = bounds.get("upper_bound") {
                (None, Some(finite(v)?))
            } else {
                return Err("unknown measurement bound".into());
            }
        };
        Ok(match self.comparison {
            Comparison::Max if upper.is_some_and(|v| v <= self.bound) => 1,
            Comparison::Max if lower.is_some_and(|v| v > self.bound) => 0,
            Comparison::Min if lower.is_some_and(|v| v >= self.bound) => 1,
            Comparison::Min if upper.is_some_and(|v| v < self.bound) => 0,
            _ => -1,
        })
    }
}

/// Owns a Core snapshot with all test outcomes explicitly reset to Unknown.
pub struct SimulationBundle {
    model: CompiledModel,
    case: DeclId,
    trace_scope: String,
    rules: Vec<ObservationRule>,
}

impl SimulationBundle {
    pub fn new(model: CompiledModel, case_name: &str) -> Result<Self, String> {
        let read = CaseReader::new(&model);
        let case = read.named(case_name)?;
        read.classified(case, "case_definition")?;
        let trace = read.reference(case, "trace_scope")?;
        let trace_scope = read.name(trace);
        let projection = TraceabilityExport::new(&model, &trace_scope)?;
        let valid_tests: HashSet<_> = projection
            .projection()
            .entities
            .iter()
            .filter(|e| e.kind == "test_case")
            .map(|e| e.declaration)
            .collect();
        let mut updates = Vec::new();
        for entity in &projection.projection().entities {
            if entity.kind == "test_case" {
                let slot = read
                    .query
                    .own_child(entity.declaration, "outcome")
                    .ok_or("test outcome slot missing")?;
                updates.push(ValueUpdate::Set {
                    declaration: slot,
                    value: ValueR::Num(-1.),
                });
            }
        }
        let model = model
            .with_value_updates(&updates)
            .map_err(|e| format!("unknown initialization: {e:?}"))?;
        let read = CaseReader::new(&model);
        let vocabulary = read.named("simulation_types")?;
        let base = read
            .query
            .own_child(vocabulary, "observes")
            .ok_or("observation vocabulary absent")?;
        let mut rules = Vec::new();
        let mut tests = HashSet::new();
        let mut metrics = HashSet::new();
        for edge in read
            .query
            .query(&Predicate::edge().and(Predicate::InheritsFromId(base)))
        {
            let edge_id = read
                .query
                .ir()
                .as_edge(edge.id)
                .ok_or("observation must be an edge")?;
            if read.query.ir().edges[edge_id.raw()].arcs.len() != 1 {
                return Err("observation requires one arc".into());
            }
            let source = edge
                .arc_bindings
                .iter()
                .filter(|b| b.sign == 1)
                .collect::<Vec<_>>();
            let target = edge
                .arc_bindings
                .iter()
                .filter(|b| b.sign == -1)
                .collect::<Vec<_>>();
            if edge.arc_bindings.len() != 2
                || source.len() != 1
                || target.len() != 1
                || target[0].target != case
                || edge.arc_bindings.iter().any(|b| b.weights.is_some())
            {
                return Err("observation requires (+ test, - case)".into());
            }
            let test = source[0].target;
            if !valid_tests.contains(&test)
                || read.query.ir().parent(test) != trace
                || !tests.insert(test)
            {
                return Err("duplicate or foreign observation test".into());
            }
            let slot = read
                .query
                .own_child(test, "outcome")
                .ok_or("observation target is not a test outcome")?;
            let metric: Measurement =
                serde_json::from_value(json!(read.string(edge.id, "metric")?))
                    .map_err(|_| "unknown measurement")?;
            let key = serde_json::to_string(&metric).unwrap();
            if !metrics.insert(key) {
                return Err("duplicate measurement binding".into());
            }
            let comparison = serde_json::from_value(json!(read.string(edge.id, "comparison")?))
                .map_err(|_| "unknown comparison")?;
            let bound = read.number(edge.id, "bound")?;
            if bound < 0. {
                return Err("negative measurement bound".into());
            }
            rules.push(ObservationRule {
                name: edge.name,
                test: read.name(test),
                metric,
                comparison,
                bound,
                slot,
            });
        }
        if rules.len() != 6 {
            return Err("six distinct measurement rules required".into());
        }
        Ok(Self {
            model,
            case,
            trace_scope,
            rules,
        })
    }

    pub fn model(&self) -> &CompiledModel {
        &self.model
    }
    pub fn trace_scope(&self) -> &str {
        &self.trace_scope
    }
    pub fn digest(&self) -> String {
        format!("{:?}", self.model.semantic_digest())
    }

    /// A checked control-leaf revision. The retained bundle remains immutable;
    /// the normal projection admission validates the new physical profile.
    /// An authored `control_period_s` can be revised independently of integration
    /// timestep; it must remain an integral number of steps, no longer than 10 ms.
    pub fn with_control(&self, field: &str, value: f64) -> Result<Self, String> {
        if ![
            "actuator_kp",
            "actuator_kd",
            "actuator_limit",
            "timestep_s",
            "control_period_s",
            "sample_hz",
            "episode_s",
        ]
        .contains(&field)
        {
            return Err("control field has no checked revision contract".into());
        }
        let read = CaseReader::new(&self.model);
        let control = read.reference(self.case, "control")?;
        let slot = read
            .query
            .own_child(control, field)
            .ok_or("unknown control field")?;
        read.number(control, field)?;
        let updated = self
            .model
            .with_value_updates(&[ValueUpdate::Set {
                declaration: slot,
                value: ValueR::Num(value),
            }])
            .map_err(|e| format!("control update: {e:?}"))?;
        let result = Self::new(updated, &read.name(self.case))?;
        result.artifacts()?;
        Ok(result)
    }

    /// Produces the robot through the existing MJCF writer and pose references through Geometry.
    pub fn artifacts(&self) -> Result<(String, Value), String> {
        let read = CaseReader::new(&self.model);
        let robot_root = read.name(read.reference(self.case, "robot_model")?);
        let robot = RobotProjection::new(&self.model, &robot_root).map_err(|e| e.to_string())?;
        let tool = read.name(read.reference(self.case, "tool")?);
        let cup = read.name(read.reference(self.case, "gripper")?);
        if !robot.model().links.iter().any(|l| l.name == tool)
            || !robot.model().links.iter().any(|l| l.name == cup)
        {
            return Err("simulation tool must be a projected link".into());
        }
        let joints = read.references(self.case, "joints")?;
        let names = joints.iter().map(|&id| read.name(id)).collect::<Vec<_>>();
        if names.len() != 6
            || names.iter().collect::<HashSet<_>>().len() != 6
            || names.iter().any(|name| {
                !robot.model().joints.iter().any(|j| {
                    &j.name == name
                        && matches!(
                            j.joint_type,
                            hymeko_geometry::kinematics::joints::JointType::Revolute
                                | hymeko_geometry::kinematics::joints::JointType::Continuous
                        )
                })
            })
        {
            return Err("six distinct projected revolute coordinates required".into());
        }
        let target = |id| -> Result<Value, String> {
            read.classified(id, "joint_target")?;
            let q = read.numbers(id, "q")?;
            if q.len() != 6 || q.iter().any(|x| x.abs() > std::f64::consts::TAU) {
                return Err("six bounded joint coordinates required".into());
            }
            let values = names.iter().cloned().zip(q.iter().copied()).collect();
            let poses = robot.forward_poses(&values).map_err(|e| e.to_string())?;
            let pose = poses["poses"]
                .as_array()
                .ok_or("poses absent")?
                .iter()
                .find(|p| p["link"] == tool)
                .ok_or("tool pose absent")?;
            Ok(json!({"name":read.name(id),"q":q,"tool_pose":pose}))
        };
        let initial = target(read.reference(self.case, "initial")?)?;
        let targets = read
            .references(self.case, "targets")?
            .into_iter()
            .map(target)
            .collect::<Result<Vec<_>, _>>()?;
        if targets.len() != 4 {
            return Err("pick/lift/place/retreat targets required".into());
        }
        let control_id = read.reference(self.case, "control")?;
        read.classified(control_id, "control_profile")?;
        let control = read.numeric_fields(control_id)?;
        for (name, lo, hi) in [
            ("timestep_s", 0.0005, 0.01),
            ("sample_hz", 10., 200.),
            ("episode_s", 1., 10.),
            ("actuator_limit", 1., 200.),
            ("actuator_kp", 1., 5000.),
            ("actuator_kd", 0., 200.),
        ] {
            let value = *control
                .get(name)
                .ok_or_else(|| format!("missing control {name}"))?;
            if value < lo || value > hi {
                return Err(format!("control out of range: {name}"));
            }
        }
        if let Some(&period) = control.get("control_period_s") {
            let dt = control["timestep_s"];
            let steps = period / dt;
            if !period.is_finite() || period < dt || period > 0.01
                || (steps - steps.round()).abs() > 1e-9 {
                return Err("control_period_s must be an integer multiple of timestep_s, at most 0.01 s".into());
            }
        }
        let scene = read.reference(self.case, "environment")?;
        read.classified(scene, "scene_profile")?;
        let mut environment = serde_json::Map::new();
        for name in [
            "payload_half_size",
            "table_half_size",
            "tcp_offset",
            "grasp_offset",
        ] {
            let values = read.numbers(scene, name)?;
            if values.len() != 3 || values.iter().any(|v| v.abs() > 2.) {
                return Err(format!("invalid scene vector: {name}"));
            }
            if name.ends_with("half_size") && values.iter().any(|v| *v <= 0.) {
                return Err(format!("nonpositive geometry: {name}"));
            }
            environment.insert(name.into(), json!(values));
        }
        for name in [
            "payload_mass_kg",
            "payload_friction",
            "grasp_distance_m",
            "retained_distance_m",
            "required_lift_m",
            "stop_at_s",
            "stop_speed_m_s",
            "stop_joint_speed_rad_s",
            "stop_dwell_s",
            "station_window_s",
        ] {
            let value = read.number(scene, name)?;
            if value <= 0. || value > 10. {
                return Err(format!("invalid scene value: {name}"));
            }
            environment.insert(name.into(), json!(value));
        }
        let bindings = json!({"bodies":robot.model().links.iter().map(|l|json!({"name":l.name,"declaration":l.did.raw()})).collect::<Vec<_>>(),"joints":robot.model().joints.iter().map(|j|json!({"name":j.name,"declaration":j.did.raw()})).collect::<Vec<_>>(),"environment":scene.raw(),"control":control_id.raw()});
        let manifest = json!({"model_digest":self.digest(),"structure_hash":format!("{:?}",self.model.program().canon.structure()),"bindings":bindings,"robot":robot.metadata(),"joint_names":names,"tool":tool,"gripper":cup,"initial":initial,"targets":targets,"controls":control,"scene":environment,"rules":self.rules,"grasp_model":"ideal_vacuum_weld","motion_policy":"MissionMotion: 0.75 rad/s, ramp >= 0.35 s, settle 0.05 s, dwell 0.3 s"});
        Ok((robot.mjcf().map_err(|e| e.to_string())?, manifest))
    }

    /// Missing evidence remains Unknown; finite measurements use the source-declared bound.
    pub fn record(&self, evidence: &Value) -> Result<(CompiledModel, Value), String> {
        if evidence["model_digest"] != self.digest() {
            return Err("evidence belongs to a different model snapshot".into());
        }
        let measurements = evidence["measurements"]
            .as_object()
            .ok_or("measurement record absent")?;
        let mut updates = Vec::new();
        let mut verdicts = Vec::new();
        for rule in &self.rules {
            let name = serde_json::to_value(rule.metric)
                .unwrap()
                .as_str()
                .unwrap()
                .to_owned();
            let measured = measurements.get(&name).filter(|v| !v.is_null());
            let outcome = rule.outcome(measured)?;
            updates.push(ValueUpdate::Set {
                declaration: rule.slot,
                value: ValueR::Num(f64::from(outcome)),
            });
            verdicts.push(json!({"rule":rule.name,"test":rule.test,"metric":rule.metric,"measured":measured,"bound":rule.bound,"outcome":outcome,"scope":"simulated declared model; ideal vacuum attachment"}));
        }
        let model = self
            .model
            .with_value_updates(&updates)
            .map_err(|e| format!("recording outcomes: {e:?}"))?;
        Ok((model, json!(verdicts)))
    }
}

struct CaseReader<'a> {
    query: QueryEngine<'a, hymeko::resolution::interner::Interner>,
}
impl<'a> CaseReader<'a> {
    fn new(model: &'a CompiledModel) -> Self {
        Self {
            query: QueryEngine::new(&model.program().ir, model.interner()),
        }
    }
    fn classified(&self, id: DeclId, kind: &str) -> Result<(), String> {
        let root = self.named("simulation_types")?;
        let base = self
            .query
            .own_child(root, kind)
            .ok_or("simulation vocabulary kind missing")?;
        if self
            .query
            .query_decl(id, &Predicate::node().and(Predicate::InheritsFromId(base)))
            .is_none()
        {
            return Err(format!("expected {kind}: {}", self.name(id)));
        }
        Ok(())
    }
    fn name(&self, id: DeclId) -> String {
        self.query
            .resolver()
            .resolve(self.query.ir().decl_nodes[id.raw()].name)
            .to_owned()
    }
    fn named(&self, name: &str) -> Result<DeclId, String> {
        let rows = self.query.query(&Predicate::Named(name.into()));
        if rows.len() != 1 {
            return Err(format!("ambiguous or absent declaration {name}"));
        }
        Ok(rows[0].id)
    }
    fn value(&self, id: DeclId, field: &str) -> Result<&'a ValueR, String> {
        self.query
            .effective_value(id, field)
            .ok_or_else(|| format!("missing {field} on {}", self.name(id)))
    }
    fn reference(&self, id: DeclId, field: &str) -> Result<DeclId, String> {
        match self.value(id, field)? {
            ValueR::Ref(v) if self.query.ir().decl_node(*v).is_some() => Ok(*v),
            _ => Err(format!("{field} must reference a declaration")),
        }
    }
    fn references(&self, id: DeclId, field: &str) -> Result<Vec<DeclId>, String> {
        match self.value(id, field)? {
            ValueR::List(v) => v
                .iter()
                .map(|x| match x {
                    ValueR::Ref(d) if self.query.ir().decl_node(*d).is_some() => Ok(*d),
                    _ => Err("reference list required".into()),
                })
                .collect(),
            _ => Err("reference list required".into()),
        }
    }
    fn number(&self, id: DeclId, field: &str) -> Result<f64, String> {
        match self.value(id, field)? {
            ValueR::Num(v) if v.is_finite() => Ok(*v),
            _ => Err(format!("finite number required: {field}")),
        }
    }
    fn numbers(&self, id: DeclId, field: &str) -> Result<Vec<f64>, String> {
        match self.value(id, field)? {
            ValueR::List(v) => v
                .iter()
                .map(|x| match x {
                    ValueR::Num(v) if v.is_finite() => Ok(*v),
                    _ => Err("finite numeric list required".into()),
                })
                .collect(),
            _ => Err("numeric list required".into()),
        }
    }
    fn string(&self, id: DeclId, field: &str) -> Result<String, String> {
        match self.value(id, field)? {
            ValueR::Str(v) => Ok(self.query.resolver().resolve(*v).into()),
            _ => Err(format!("string required: {field}")),
        }
    }
    fn numeric_fields(&self, id: DeclId) -> Result<BTreeMap<String, f64>, String> {
        self.query
            .effective_fields(id)
            .iter()
            .map(|(name, _)| Ok((name.to_string(), self.number(id, name)?)))
            .collect()
    }
}
