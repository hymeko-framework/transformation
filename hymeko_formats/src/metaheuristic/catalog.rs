//! Descriptive operator-support projection, not an optimizer or equivalence checker.
//! Names and canonical identities stay in provenance, outside identification features.
use super::{Contract, Result};
use hymeko::{
    common::ids::DeclId,
    ir::{canonical_hash::Canon, ir::ValueR},
    module_store::execution::CompiledModel,
};
use hymeko_query::{Predicate, QueryEngine, QueryMatch};
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};

const SPACES: &[&str] = &[
    "population",
    "scores",
    "point",
    "scalar",
    "counter",
    "random",
    "bounds",
    "function",
    "vector",
    "indices",
    "mask",
    "memory",
];
const CATEGORIES: &[&str] = &[
    "initialize",
    "select",
    "vary",
    "evaluate",
    "accept",
    "adapt",
    "archive",
];
const GUARDS: &[&str] = &[
    "always",
    "improvement",
    "probability",
    "stagnation",
    "branch",
];
const SCHEDULES: &[&str] = &[
    "deferred",
    "sequential",
    "pairwise",
    "phased",
    "unspecified",
];
const FAMILIES: &[&str] = &[
    "uniform_initialization",
    "objective_evaluation",
    "minimum_archive",
    "difference_mutation",
    "coordinate_mix",
    "box_projection",
    "pairwise_greedy",
    "tournament_selection",
    "coordinate_reset",
    "rank_truncate",
    "velocity_attraction",
    "position_integration",
    "personal_best_update",
    "ranked_leaders",
    "coefficient_schedule",
    "distance_attraction",
    "mean_aggregation",
    "spiral_perturbation",
    "branch_choice",
    "levy_perturbation",
    "uniform_perturbation",
    "amplitude_schedule",
    "exponential_distance_weight",
    "fitness_proportional_selection",
    "coordinate_difference",
    "stagnation_update",
    "random_restart",
    "best_gate",
    "random_peer_selection",
    "neighbour_perturbation",
];

struct Operator {
    id: DeclId,
    stage: usize,
    category: String,
    family: String,
    guard: String,
    rule: String,
    reads: Vec<DeclId>,
    writes: Vec<DeclId>,
}

/// Validate one bounded descriptor through Core/Query and export two name-free projections.
/// The result intentionally carries no claim that its rule strings are executable.
pub fn project(model: &CompiledModel) -> Result<Value> {
    if model.program().ir.decl_nodes.len() > 4096 {
        return Err("declaration cap exceeded".into());
    }
    Catalog {
        contract: Contract::new(model)?,
    }
    .project()
}

struct Catalog<'a> {
    contract: Contract<'a>,
}

impl Catalog<'_> {
    fn field(&self, id: DeclId, name: &str, choices: &[&str]) -> Result<String> {
        let value = self.contract.field(id, name)?;
        if !choices.contains(&value.as_str()) {
            return Err(format!("unsupported {name}: {value}").into());
        }
        Ok(value)
    }

    fn group(&self, parent: DeclId, name: &str) -> Result<DeclId> {
        let query = QueryEngine::new(
            &self.contract.model.program().ir,
            self.contract.model.interner(),
        );
        let id = query
            .effective_child(parent, name)
            .ok_or("missing local group")?;
        if self.contract.model.program().ir.parent(id) != parent {
            return Err("group must be local".into());
        }
        Ok(id)
    }

    fn members(&mut self, kind: &str, group: DeclId) -> Result<Vec<QueryMatch>> {
        let type_id = self.contract.vocabulary(kind)?;
        let query = QueryEngine::new(
            &self.contract.model.program().ir,
            self.contract.model.interner(),
        );
        let predicate = Predicate::InheritsFromId(type_id);
        let members = query.query(&predicate);
        if !(1..=64).contains(&members.len()) {
            return Err("member count must be 1..64".into());
        }
        let expected_kind = if kind == "state_kind" {
            Predicate::node()
        } else {
            Predicate::edge()
        };
        if members
            .iter()
            .any(|m| query.query_decl(m.id, &expected_kind).is_none())
        {
            return Err("typed member has the wrong declaration kind".into());
        }
        if members
            .iter()
            .any(|m| self.contract.model.program().ir.parent(m.id) != group)
        {
            return Err("typed member lies outside its owning group".into());
        }
        self.contract.check(kind, predicate, members.len())?;
        self.contract
            .require_children(group, &members.iter().map(|m| m.id).collect())?;
        Ok(members)
    }

    fn states(&mut self, group: DeclId) -> Result<BTreeMap<DeclId, String>> {
        let members = self.members("state_kind", group)?;
        let mut states = BTreeMap::new();
        for member in members {
            if !member.arc_bindings.is_empty() {
                return Err("state must not carry incidences".into());
            }
            let fields = self.contract.require_fields(member.id, &["space"])?;
            self.contract.require_children(member.id, &fields)?;
            states.insert(member.id, self.field(member.id, "space", SPACES)?);
        }
        Ok(states)
    }

    fn stage(&self, id: DeclId) -> Result<usize> {
        let query = QueryEngine::new(
            &self.contract.model.program().ir,
            self.contract.model.interner(),
        );
        match query.effective_value(id, "stage") {
            Some(ValueR::Num(n)) if n.is_finite() && *n >= 0. && *n < 64. && n.fract() == 0. => {
                Ok(*n as usize)
            }
            _ => Err("stage must be an integer in 0..63".into()),
        }
    }

    fn operator(
        &mut self,
        member: QueryMatch,
        states: &BTreeMap<DeclId, String>,
        read: DeclId,
        write: DeclId,
    ) -> Result<Operator> {
        if !member.arc_bindings.is_empty() {
            return Err("operator has untyped direct incidences".into());
        }
        let id = member.id;
        let stage = self.stage(id)?;
        let mut children = self
            .contract
            .require_fields(id, &["category", "family", "guard", "rule"])?;
        // Stage is numeric; the common string-field helper cannot admit it.
        let stage_id = self.group(id, "stage")?;
        self.contract.require_children(stage_id, &BTreeSet::new())?;
        children.insert(stage_id);
        let predicate = Predicate::Named(member.name).and(Predicate::HasParent(Box::new(
            Predicate::named("operators"),
        )));
        // Query the scoped name, then verify the exact canonical parent for each returned role.
        let mut roles = Vec::new();
        for (label, kind, opposite) in [("reads", read, write), ("writes", write, read)] {
            let role = self.contract.dependency_role(
                &format!("stage-{stage}:{label}"),
                &predicate,
                kind,
                opposite,
            )?;
            if self.contract.model.program().ir.parent(role.id) != id {
                return Err("role belongs to another operator".into());
            }
            children.insert(role.id);
            let refs: Vec<_> = role.arc_bindings.iter().map(|b| b.target).collect();
            if refs.is_empty()
                || refs.len() != refs.iter().collect::<BTreeSet<_>>().len()
                || refs.iter().any(|r| !states.contains_key(r))
            {
                return Err("dependency targets must be distinct local states".into());
            }
            roles.push(refs);
        }
        self.contract.require_children(id, &children)?;
        Ok(Operator {
            id,
            stage,
            category: self.field(id, "category", CATEGORIES)?,
            family: self.field(id, "family", FAMILIES)?,
            guard: self.field(id, "guard", GUARDS)?,
            rule: self.contract.field(id, "rule")?,
            reads: roles.remove(0),
            writes: roles.remove(0),
        })
    }

    fn project(mut self) -> Result<Value> {
        let method_type = self.contract.vocabulary("method_kind")?;
        let method = self
            .contract
            .check(
                "one_method",
                Predicate::node().and(Predicate::InheritsFromId(method_type)),
                1,
            )?
            .remove(0)
            .id;
        let mut children = self
            .contract
            .require_fields(method, &["label", "source", "notes", "schedule"])?;
        let states_group = self.group(method, "states")?;
        let operators_group = self.group(method, "operators")?;
        children.extend([states_group, operators_group]);
        self.contract.require_children(method, &children)?;
        let states = self.states(states_group)?;
        let members = self.members("operator_kind", operators_group)?;
        let read = self.contract.vocabulary("read_role")?;
        let write = self.contract.vocabulary("write_role")?;
        self.contract.check(
            "dependency_roles",
            Predicate::InheritsFromId(read).or(Predicate::InheritsFromId(write)),
            members.len() * 2,
        )?;
        let mut operators = members
            .into_iter()
            .map(|op| self.operator(op, &states, read, write))
            .collect::<Result<Vec<_>>>()?;
        operators.sort_by_key(|op| op.stage);
        if operators.iter().enumerate().any(|(i, op)| i != op.stage) {
            return Err("stages must uniquely cover 0..operator_count".into());
        }
        let schedule = self.field(method, "schedule", SCHEDULES)?;
        let support = features(&states, &operators, &schedule, false)?;
        let mechanisms = features(&states, &operators, &schedule, true)?;
        let records = operators.iter().map(|op| Ok(json!({"identity":self.contract.identity(op.id)?,
            "stage":op.stage,"category":op.category,"family":op.family,"guard":op.guard,"rule":op.rule,
            "reads":op.reads.iter().map(|id|self.contract.identity(*id)).collect::<Result<Vec<_>>>()?,
            "writes":op.writes.iter().map(|id|self.contract.identity(*id)).collect::<Result<Vec<_>>>()?}))).collect::<Result<Vec<_>>>()?;
        let state_records = states
            .iter()
            .map(|(id, space)| Ok(json!({"identity":self.contract.identity(*id)?,"space":space})))
            .collect::<Result<Vec<_>>>()?;
        let hash: String = Canon::new(
            &self.contract.model.program().ir,
            self.contract.model.interner(),
        )
        .hash()
        .0
        .0
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
        Ok(
            json!({"schema":"hymeko.metaheuristic-structure/1", "label":self.contract.field(method,"label")?,
            "source":self.contract.field(method,"source")?,"notes":self.contract.field(method,"notes")?,
            "model_semantic_digest":hash,"schedule":schedule,"operators":records,"states":state_records,
            "features":{"support":support,"mechanisms":mechanisms},
            "query_checks":self.contract.checks.iter().map(|c|json!({"label":c.label,"passed":c.passed,"actual":c.actual,"witness_decl_ids":c.witnesses.iter().map(|id|id.raw()).collect::<Vec<_>>(),"witnesses_truncated":c.witnesses_truncated})).collect::<Vec<_>>(),
            "scope":"typed incidence-neighborhood projection; rule text not interpreted; no execution/equivalence/performance claim"}),
        )
    }
}

fn features(
    states: &BTreeMap<DeclId, String>,
    operators: &[Operator],
    schedule: &str,
    enriched: bool,
) -> Result<BTreeMap<String, usize>> {
    let mut counts = BTreeMap::new();
    let mut add = |v: Value| -> Result<()> {
        *counts.entry(serde_json::to_string(&v)?).or_insert(0) += 1;
        Ok(())
    };
    let label = |op: &Operator| {
        if enriched {
            json!([op.category, op.family, op.guard])
        } else {
            json!([op.category])
        }
    };
    for op in operators {
        let mut reads: Vec<_> = op.reads.iter().map(|id| &states[id]).collect();
        reads.sort();
        let mut writes: Vec<_> = op.writes.iter().map(|id| &states[id]).collect();
        writes.sort();
        add(json!(["operator", label(op), reads, writes]))?;
    }
    for (id, space) in states {
        let mut uses = Vec::new();
        for op in operators {
            if op.reads.contains(id) {
                uses.push(json!(["read", label(op)]).to_string());
            }
            if op.writes.contains(id) {
                uses.push(json!(["write", label(op)]).to_string());
            }
        }
        uses.sort();
        add(json!(["state", space, uses]))?;
    }
    if enriched {
        add(json!(["schedule", schedule]))?;
        for pair in operators.windows(2) {
            add(json!(["successor", label(&pair[0]), label(&pair[1])]))?;
        }
    }
    Ok(counts)
}
