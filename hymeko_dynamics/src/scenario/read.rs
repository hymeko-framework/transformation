use super::model::*;
use crate::read::Reader;
use hymeko::{
    common::ids::DeclId,
    ir::ir::{Ir, ValueR},
    resolution::interner::Interner,
};
use serde::de::DeserializeOwned;
use std::collections::{BTreeMap, BTreeSet};

impl ScenarioModel {
    /// Project a bounded scenario declaration from an already compiled coherent IR.
    /// Rejects unknown fields, cross-scenario references and unsupported versions.
    pub fn from_ir(ir: &Ir, it: &Interner, root: usize) -> Result<Self, Refusal> {
        if ir.decl_nodes.len() > 10_000 || root >= ir.decl_nodes.len() {
            return Err(Refusal::new(
                RefusalCode::Limit,
                "scenario IR must contain at most 10000 declarations and a valid root",
            ));
        }
        Projection {
            reader: Reader { ir, it },
        }
        .project(DeclId::new(root))
    }
}

pub(super) struct Projection<'a> {
    pub(super) reader: Reader<'a>,
}
impl Projection<'_> {
    pub(super) fn error(&self, id: DeclId, text: &str) -> Refusal {
        Refusal::new(
            RefusalCode::InvalidModel,
            format!("{}: {text}", self.reader.qualified(id)),
        )
    }
    pub(super) fn identity(&self, id: DeclId) -> Identity {
        Identity {
            id: id.raw(),
            name: self.reader.qualified(id),
        }
    }
    pub(super) fn children(&self, id: DeclId) -> Result<BTreeMap<String, DeclId>, Refusal> {
        let mut children = BTreeMap::new();
        let mut child = self.reader.ir.decl_nodes[id.raw()].first_child;
        while child.is_some() {
            let node = self
                .reader
                .ir
                .decl_nodes
                .get(child.raw())
                .ok_or_else(|| self.error(id, "invalid child"))?;
            if children
                .insert(self.reader.it.resolve(node.name).into(), child)
                .is_some()
            {
                return Err(self.error(id, "duplicate field or child name"));
            }
            child = node.next_sibling;
        }
        Ok(children)
    }
    pub(super) fn fields(&self, id: DeclId, allowed: &[&str]) -> Result<(), Refusal> {
        for name in self.children(id)?.keys() {
            if !allowed.contains(&name.as_str()) {
                return Err(self.error(id, &format!("unknown field {name}")));
            }
        }
        Ok(())
    }
    fn group(&self, root: DeclId, name: &str, cap: usize) -> Result<Vec<DeclId>, Refusal> {
        let id = *self
            .children(root)?
            .get(name)
            .ok_or_else(|| self.error(root, &format!("missing {name}")))?;
        let children = self.children(id)?;
        if children.len() > cap {
            return Err(Refusal::new(
                RefusalCode::Limit,
                format!("{name} exceeds {cap}"),
            ));
        }
        // Canonical declaration order, independent of associative-container iteration.
        let mut result: Vec<_> = children.into_values().collect();
        result.sort_by_key(|d| d.raw());
        Ok(result)
    }
    pub(super) fn text(&self, id: DeclId, name: &str) -> Result<String, Refusal> {
        match self.reader.field(id, name) {
            Some(ValueR::Str(s)) if self.reader.it.resolve(*s).len() <= 256 => {
                Ok(self.reader.it.resolve(*s).into())
            }
            _ => Err(self.error(id, &format!("{name} must be text of at most 256 bytes"))),
        }
    }
    fn number(&self, id: DeclId, name: &str) -> Result<i64, Refusal> {
        match self.reader.field(id, name) {
            Some(ValueR::Num(n))
                if n.is_finite() && n.fract() == 0.0 && n.abs() <= MAX_VALUE as f64 =>
            {
                Ok(*n as i64)
            }
            _ => Err(self.error(id, &format!("{name} must be an integer in +/-{MAX_VALUE}"))),
        }
    }
    fn choice<T: DeserializeOwned>(&self, id: DeclId, name: &str) -> Result<T, Refusal> {
        serde_json::from_value(serde_json::Value::String(self.text(id, name)?))
            .map_err(|e| self.error(id, &format!("{name}: {e}")))
    }
    fn reference(
        &self,
        id: DeclId,
        field: &str,
        index: &BTreeMap<DeclId, usize>,
    ) -> Result<usize, Refusal> {
        self.reader
            .reference(id, field)
            .and_then(|d| index.get(&d).copied())
            .ok_or_else(|| {
                self.error(
                    id,
                    &format!("{field} must reference this scenario's declared group"),
                )
            })
    }
    fn targets(&self, id: DeclId, index: &BTreeMap<DeclId, usize>) -> Result<Vec<usize>, Refusal> {
        let Some(ValueR::List(items)) = self.reader.field(id, "targets") else {
            return Err(self.error(id, "targets must be a list of references"));
        };
        let targets = items
            .iter()
            .map(|value| match value {
                ValueR::Ref(target) => index
                    .get(target)
                    .copied()
                    .ok_or_else(|| self.error(id, "target outside scenario")),
                _ => Err(self.error(id, "target must be a reference")),
            })
            .collect::<Result<Vec<_>, _>>()?;
        if targets.len() > MAX_ELEMENTS
            || targets.iter().copied().collect::<BTreeSet<_>>().len() != targets.len()
        {
            return Err(self.error(id, "targets must be bounded and unique"));
        }
        Ok(targets)
    }
    fn index(ids: &[DeclId]) -> BTreeMap<DeclId, usize> {
        ids.iter().enumerate().map(|(i, id)| (*id, i)).collect()
    }
    fn action(
        &self,
        id: DeclId,
        elements: &BTreeMap<DeclId, usize>,
        causes: &BTreeMap<DeclId, usize>,
        intervention: bool,
    ) -> Result<Action, Refusal> {
        self.fields(id, &["cause", "guards", "effects", "inputMin", "inputMax"])?;
        let mut action = Action {
            identity: self.identity(id),
            cause: self.reference(id, "cause", causes)?,
            guards: Vec::new(),
            effects: Vec::new(),
            input_range: None,
        };
        if self.reader.field(id, "inputMin").is_some()
            || self.reader.field(id, "inputMax").is_some()
        {
            let range = [self.number(id, "inputMin")?, self.number(id, "inputMax")?];
            if !intervention || range[0] > range[1] {
                return Err(self.error(id, "invalid intervention input range"));
            }
            action.input_range = Some(range);
        }
        for guard in self.group(id, "guards", MAX_ELEMENTS)? {
            self.fields(guard, &["target", "op", "value"])?;
            action.guards.push(Guard {
                target: self.reference(guard, "target", elements)?,
                op: self.choice(guard, "op")?,
                value: self.number(guard, "value")?,
            });
        }
        for effect in self.group(id, "effects", MAX_ELEMENTS)? {
            self.fields(effect, &["target", "op", "value", "input"])?;
            let input = self.reader.field(effect, "input").is_some();
            if input
                && (self.text(effect, "input")? != "value"
                    || action.input_range.is_none()
                    || self.reader.field(effect, "value").is_some())
            {
                return Err(self.error(
                    effect,
                    "input requires a declared range and excludes literal value",
                ));
            }
            action.effects.push(Effect {
                target: self.reference(effect, "target", elements)?,
                op: self.choice(effect, "op")?,
                value: if input {
                    0
                } else {
                    self.number(effect, "value")?
                },
                input,
            });
        }
        if action.effects.is_empty()
            || action
                .effects
                .iter()
                .map(|e| e.target)
                .collect::<BTreeSet<_>>()
                .len()
                != action.effects.len()
        {
            return Err(self.error(id, "effects must have nonempty, unique targets"));
        }
        Ok(action)
    }
    fn project(&self, root: DeclId) -> Result<ScenarioModel, Refusal> {
        self.fields(
            root,
            &[
                "runtime",
                "timing",
                "policy",
                "resetCause",
                "causes",
                "elements",
                "transitions",
                "interventions",
                "observers",
                "invariants",
            ],
        )?;
        if !runtime_is(&self.text(root, "runtime")?) {
            return Err(Refusal::new(
                RefusalCode::SchemaMismatch,
                "unsupported scenario schema",
            ));
        }
        if self.text(root, "timing")? != "untimed" {
            return Err(self.error(root, "v1 timing must explicitly be untimed"));
        }
        let cause_ids = self.group(root, "causes", 64)?;
        let element_ids = self.group(root, "elements", MAX_ELEMENTS)?;
        if cause_ids.is_empty() || element_ids.is_empty() {
            return Err(self.error(root, "causes and elements must not be empty"));
        }
        let ci = Self::index(&cause_ids);
        let ei = Self::index(&element_ids);
        let causes = cause_ids
            .into_iter()
            .map(|id| {
                self.fields(id, &["label"])?;
                Ok(Cause {
                    identity: self.identity(id),
                    label: self.text(id, "label")?,
                })
            })
            .collect::<Result<_, Refusal>>()?;
        let elements = element_ids
            .into_iter()
            .map(|id| {
                self.fields(id, &["initial", "capacity"])?;
                let element = Element {
                    identity: self.identity(id),
                    initial: self.number(id, "initial")?,
                    capacity: self.number(id, "capacity")?,
                };
                if element.initial < 0 || element.initial > element.capacity {
                    return Err(self.error(id, "initial content must be between zero and capacity"));
                }
                Ok(element)
            })
            .collect::<Result<_, Refusal>>()?;
        let transitions = self
            .group(root, "transitions", MAX_ACTIONS)?
            .into_iter()
            .map(|id| self.action(id, &ei, &ci, false))
            .collect::<Result<Vec<_>, _>>()?;
        let interventions = self
            .group(root, "interventions", MAX_ACTIONS)?
            .into_iter()
            .map(|id| self.action(id, &ei, &ci, true))
            .collect::<Result<Vec<_>, _>>()?;
        if transitions.len() + interventions.len() > MAX_ACTIONS {
            return Err(Refusal::new(
                RefusalCode::Limit,
                "combined action limit exceeded",
            ));
        }
        let observers = self
            .group(root, "observers", MAX_OBSERVERS)?
            .into_iter()
            .map(|id| self.observer(id, &ei, &ci))
            .collect::<Result<_, _>>()?;
        let invariants = self
            .group(root, "invariants", MAX_OBSERVERS)?
            .into_iter()
            .map(|id| {
                self.fields(id, &["targets", "minimum", "maximum", "violation"])?;
                let invariant = Invariant {
                    identity: self.identity(id),
                    targets: self.targets(id, &ei)?,
                    minimum: self.number(id, "minimum")?,
                    maximum: self.number(id, "maximum")?,
                    violation: self.choice(id, "violation")?,
                };
                if invariant.minimum > invariant.maximum {
                    return Err(self.error(id, "invariant range is reversed"));
                }
                Ok(invariant)
            })
            .collect::<Result<_, Refusal>>()?;
        Ok(ScenarioModel {
            schema: SCHEMA.into(),
            identity: self.identity(root),
            policy: self.choice(root, "policy")?,
            reset_cause: self.reference(root, "resetCause", &ci)?,
            causes,
            elements,
            transitions,
            interventions,
            observers,
            invariants,
        })
    }
    fn observer(
        &self,
        id: DeclId,
        elements: &BTreeMap<DeclId, usize>,
        causes: &BTreeMap<DeclId, usize>,
    ) -> Result<Observer, Refusal> {
        self.fields(id, &["mode", "targets", "cause"])?;
        let kind = self.choice(id, "mode")?;
        let cause = self
            .reader
            .field(id, "cause")
            .map(|_| self.reference(id, "cause", causes))
            .transpose()?;
        let targets = self.targets(id, elements)?;
        if (kind == ObserverKind::Count && !targets.is_empty())
            || (kind != ObserverKind::Count && cause.is_some())
        {
            return Err(self.error(id, "count uses optional cause; sum/accumulate use targets"));
        }
        Ok(Observer {
            identity: self.identity(id),
            kind,
            targets,
            cause,
        })
    }
}
