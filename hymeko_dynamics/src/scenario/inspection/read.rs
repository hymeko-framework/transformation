use super::super::{
    Action, Comparison, Effect, EffectKind, Guard, Identity, Refusal, RefusalCode, ScenarioModel,
    read::Projection,
};
use super::model::{Bindings, Criteria, InspectionModel, OutputBindings, Pose, Station};
use crate::read::Reader;
use hymeko::{
    common::ids::DeclId,
    ir::{
        canonical_hash::{DerivedPathIndex, build_path_index},
        ir::{Ir, ValueR},
    },
    resolution::interner::Interner,
};
use std::collections::BTreeSet;

pub(super) struct InspectionReader<'a>(Projection<'a>, DerivedPathIndex);
impl<'a> InspectionReader<'a> {
    pub fn new(ir: &'a Ir, it: &'a Interner) -> Result<Self, Refusal> {
        if ir.decl_nodes.len() > 10_000 {
            return Err(Refusal::new(
                RefusalCode::Limit,
                "inspection IR exceeds limit",
            ));
        }
        let paths = build_path_index(ir)
            .map_err(|e| Refusal::new(RefusalCode::InvalidModel, e.to_string()))?;
        Ok(Self(
            Projection {
                reader: Reader { ir, it },
            },
            paths,
        ))
    }
    fn identity(&self, id: DeclId) -> Result<Identity, Refusal> {
        let path = self
            .1
            .path_of(id)
            .ok_or_else(|| self.0.error(id, "canonical declaration path missing"))?;
        let name = path
            .0
            .iter()
            .map(|s| self.0.reader.it.resolve(*s))
            .collect::<Vec<_>>()
            .join(".");
        if name.len() > 256 {
            return Err(self.0.error(id, "inspection path exceeds 256 bytes"));
        }
        Ok(Identity { id: id.raw(), name })
    }
    fn reference(&self, at: DeclId, field: &str) -> Result<DeclId, Refusal> {
        self.0
            .reader
            .reference(at, field)
            .ok_or_else(|| self.0.error(at, &format!("missing reference {field}")))
    }
    fn scalar(&self, at: DeclId, field: &str, low: f64, high: f64) -> Result<f64, Refusal> {
        match self.0.reader.field(at, field) {
            Some(ValueR::Num(n)) if n.is_finite() && *n >= low && *n <= high => Ok(*n),
            _ => Err(self
                .0
                .error(at, &format!("{field} must be finite in [{low}, {high}]"))),
        }
    }
    fn duration(&self, at: DeclId, field: &str) -> Result<u64, Refusal> {
        Ok((self.scalar(at, field, 0.000001, 60.)? * 1e9).round() as u64)
    }
    fn vector<const N: usize>(&self, at: DeclId, field: &str) -> Result<[f64; N], Refusal> {
        let Some(ValueR::List(values)) = self.0.reader.field(at, field) else {
            return Err(self.0.error(at, &format!("missing list {field}")));
        };
        let mut result = [0.; N];
        if values.len() != N {
            return Err(self.0.error(at, "wrong vector dimension"));
        }
        for (out, value) in result.iter_mut().zip(values) {
            let ValueR::Num(n) = value else {
                return Err(self.0.error(at, "numeric vector required"));
            };
            *out = *n;
        }
        Ok(result)
    }
    fn classified(&self, at: DeclId, kind: &str) -> Result<(), Refusal> {
        if self.0.reader.inherits(at, kind) {
            Ok(())
        } else {
            Err(self.0.error(at, &format!("expected isa {kind}")))
        }
    }
    fn criteria(&self, at: DeclId) -> Result<Criteria, Refusal> {
        self.classified(at, "inspection_criteria")?;
        self.0.fields(
            at,
            &["position_m", "orientation_rad", "speed_m_s", "dwell_s"],
        )?;
        Ok(Criteria {
            position_m: self.scalar(at, "position_m", 1e-9, 1.)?,
            orientation_rad: self.scalar(at, "orientation_rad", 1e-9, std::f64::consts::PI)?,
            speed_m_s: self.scalar(at, "speed_m_s", 0., 10.)?,
            dwell_ns: self.duration(at, "dwell_s")?,
        })
    }
    fn station(&self, at: DeclId) -> Result<Station, Refusal> {
        self.classified(at, "inspection_station")?;
        self.0
            .fields(at, &["position_m", "quaternion_xyzw", "criteria"])?;
        let pose = Pose {
            position_m: self.vector(at, "position_m")?,
            quaternion_xyzw: self.vector(at, "quaternion_xyzw")?,
        };
        pose.validate().map_err(|e| self.0.error(at, &e.reason))?;
        Ok(Station {
            identity: self.identity(at)?,
            pose,
            criteria: self.criteria(self.reference(at, "criteria")?)?,
        })
    }
    pub fn read(&self, root: usize) -> Result<(InspectionModel, usize), Refusal> {
        if self.0.reader.ir.decl_nodes.len() > 10_000 || root >= self.0.reader.ir.decl_nodes.len() {
            return Err(Refusal::new(
                RefusalCode::Limit,
                "bounded valid inspection root required",
            ));
        }
        let at = DeclId::new(root);
        self.classified(at, "inspection_task")?;
        self.0.fields(
            at,
            &[
                "workflow",
                "output",
                "completed",
                "blocked",
                "advance",
                "finish",
                "block",
                "resume",
                "source",
                "frame",
                "cycles",
                "stations",
                "home",
                "max_gap_s",
                "expiry_s",
            ],
        )?;
        let workflow = self.reference(at, "workflow")?;
        let state = ScenarioModel::from_ir(self.0.reader.ir, self.0.reader.it, workflow.raw())?;
        let cycles = self.scalar(at, "cycles", 1., 16.)?;
        if cycles.fract() != 0. {
            return Err(self.0.error(at, "cycles must be integral"));
        }
        let Some(ValueR::List(stations)) = self.0.reader.field(at, "stations") else {
            return Err(self.0.error(at, "stations must be references"));
        };
        if stations.is_empty() || stations.len() > 16 {
            return Err(self.0.error(at, "expected 1..16 stations"));
        }
        let mut seen = BTreeSet::new();
        let mut targets = Vec::with_capacity(stations.len());
        for station in stations {
            let ValueR::Ref(id) = station else {
                return Err(self.0.error(at, "station reference required"));
            };
            if !seen.insert(*id) {
                return Err(self.0.error(at, "station references must be unique"));
            }
            targets.push(self.station(*id)?);
        }
        let required = targets.len() * cycles as usize;
        let bindings = self.bindings(at, &state, required)?;
        let model = InspectionModel {
            identity: self.identity(at)?,
            source: self.0.text(at, "source")?,
            frame: self.0.text(at, "frame")?,
            stations: targets,
            home: self.station(self.reference(at, "home")?)?,
            cycles: cycles as usize,
            max_gap_ns: self.duration(at, "max_gap_s")?,
            expiry_ns: self.duration(at, "expiry_s")?,
            workflow: bindings,
            output: self.output(self.reference(at, "output")?)?,
        };
        if model.source.is_empty() || model.frame.is_empty() || model.max_gap_ns > model.expiry_ns {
            return Err(self
                .0
                .error(at, "source/frame and max_gap <= expiry required"));
        }
        Ok((model, workflow.raw()))
    }
    fn output(&self, at: DeclId) -> Result<OutputBindings, Refusal> {
        self.classified(at, "inspection_observation")?;
        let fields = OutputBindings::FIELDS
            .into_iter()
            .chain([
                "phase",
                "target",
                "evaluated_target",
                "target_path",
                "evaluated_target_path",
            ])
            .collect::<Vec<_>>();
        self.0.fields(at, &fields)?;
        let children = self.0.children(at)?;
        let field = |name: &str| {
            children
                .get(name)
                .copied()
                .ok_or_else(|| self.0.error(at, &format!("missing output {name}")))
        };
        let mut numbers = [DeclId::new(0); 17];
        for (slot, name) in numbers.iter_mut().zip(OutputBindings::FIELDS) {
            *slot = field(name)?;
            if !matches!(self.0.reader.field(at, name), Some(ValueR::Num(n)) if n.is_finite()) {
                return Err(self.0.error(at, "numeric observation output required"));
            }
        }
        self.0.text(at, "phase")?;
        self.0.text(at, "target_path")?;
        self.0.text(at, "evaluated_target_path")?;
        for name in ["target", "evaluated_target"] {
            self.station(self.reference(at, name)?)?;
        }
        Ok(OutputBindings {
            phase: field("phase")?,
            target_path: field("target_path")?,
            evaluated_target_path: field("evaluated_target_path")?,
            target: field("target")?,
            evaluated_target: field("evaluated_target")?,
            numbers,
        })
    }
    fn bindings(
        &self,
        at: DeclId,
        model: &ScenarioModel,
        required: usize,
    ) -> Result<Bindings, Refusal> {
        let element = |field: &str, capacity: i64| -> Result<usize, Refusal> {
            let id = self.reference(at, field)?;
            model
                .elements
                .iter()
                .position(|e| e.identity.id == id.raw() && e.initial == 0 && e.capacity == capacity)
                .ok_or_else(|| {
                    self.0
                        .error(at, "workflow element identity/initial/capacity mismatch")
                })
        };
        if model.elements.len() != 2
            || model.transitions.len() != 2
            || model.interventions.len() != 2
        {
            return Err(self.0.error(
                at,
                "inspection workflow requires two counters, transitions and interventions",
            ));
        }
        let completed = element("completed", required as i64 + 1)?;
        let blocked = element("blocked", 1)?;
        if completed == blocked {
            return Err(self.0.error(at, "progress and block counters must differ"));
        }
        let guard = |target, op, value| Guard { target, op, value };
        let active = guard(blocked, Comparison::Eq, 0);
        let advance = self.action(
            at,
            "advance",
            &model.transitions,
            &[
                active.clone(),
                guard(completed, Comparison::Le, required as i64 - 1),
            ],
            Effect {
                target: completed,
                op: EffectKind::Add,
                value: 1,
                input: false,
            },
        )?;
        let finish = self.action(
            at,
            "finish",
            &model.transitions,
            &[
                active.clone(),
                guard(completed, Comparison::Eq, required as i64),
            ],
            Effect {
                target: completed,
                op: EffectKind::Add,
                value: 1,
                input: false,
            },
        )?;
        let block = self.action(
            at,
            "block",
            &model.interventions,
            &[active],
            Effect {
                target: blocked,
                op: EffectKind::Set,
                value: 1,
                input: false,
            },
        )?;
        let resume = self.action(
            at,
            "resume",
            &model.interventions,
            &[guard(blocked, Comparison::Eq, 1)],
            Effect {
                target: blocked,
                op: EffectKind::Set,
                value: 0,
                input: false,
            },
        )?;
        Ok(Bindings {
            completed,
            blocked,
            advance,
            finish,
            block,
            resume,
        })
    }
    fn action(
        &self,
        at: DeclId,
        field: &str,
        actions: &[Action],
        guards: &[Guard],
        effect: Effect,
    ) -> Result<String, Refusal> {
        let id = self.reference(at, field)?;
        actions
            .iter()
            .find(|a| {
                a.identity.id == id.raw()
                    && a.input_range.is_none()
                    && a.effects == [effect.clone()]
                    && a.guards.len() == guards.len()
                    && guards.iter().all(|g| a.guards.contains(g))
            })
            .map(|a| a.identity.name.clone())
            .ok_or_else(|| {
                self.0.error(
                    at,
                    &format!("{field}: workflow effect/guard contract mismatch"),
                )
            })
    }
}
