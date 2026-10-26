//! IR to [`Problem`]: the typed projection.
//!
//! Everything is found by INHERITANCE, never by name. `R1` is a decision variable because its base
//! chain reaches `opt.kinds.decision`, not because it is called `R1` and not because it appears in
//! some list a consumer keeps. Rename it and it stays a decision variable; declare it a parameter
//! and it stops being one, without a line changing here.
//!
//! Stable identity is the [`DeclId`], and it survives into [`Quantity::decl`] and out the other side
//! into the result. Array positions appear for the first time in [`Problem::decisions`], which is
//! the numerical projection and the only place they are allowed.

use std::collections::{HashMap, HashSet};

use hymeko::common::ids::{DeclId, SymId};
use hymeko::ir::ir::{DeclKind, Ir, SignedRefR, ValueR};
use hymeko::resolution::interner::Interner;

use crate::network::Branch;
use crate::problem::{Constraint, Kind, Problem, Quantity, Relation, Scenario, Sense, Unit};

/// Why a model could not be projected.
#[derive(Debug, Clone, PartialEq)]
pub enum ReadError {
    /// A relation whose incidences do not match what its operator takes.
    Arity {
        relation: String,
        expected: &'static str,
        found: String,
    },
    /// A reference to something the projection needs and did not find.
    Unresolved { at: String, what: String },
    /// A unit declaration this projection does not know.
    UnknownUnit { quantity: String, unit: String },
    /// A quantity with no `unit` field at all.
    UnitMissing { quantity: String },
    /// Relations that cannot be ordered — a quantity defined in terms of itself.
    Cyclic { unresolved: Vec<String> },
    /// More than one net declared as ground, or none where the model needs one.
    Ground { found: usize },
}

/// Read a compiled IR into a solver-neutral problem.
///
/// The model is found wherever it is: nothing here takes a root name, because a decision variable is
/// a decision variable in any scope.
pub fn project(ir: &Ir, it: &Interner) -> Result<Problem, ReadError> {
    let reader = Reader { ir, it };
    reader.project()
}

struct Reader<'a> {
    ir: &'a Ir,
    it: &'a Interner,
}

impl Reader<'_> {
    fn name(&self, decl: DeclId) -> &str {
        self.ir
            .decl_nodes
            .get(decl.raw())
            .map(|n| self.it.resolve(n.name))
            .unwrap_or("?")
    }

    /// The dotted path from the outermost declaration down, which is what a human and a result file
    /// both want and what stays stable across a renumbering.
    fn qualified(&self, decl: DeclId) -> String {
        let mut parts = Vec::new();
        let mut here = decl;
        let mut guard = 0usize;
        while here.is_some() && guard <= self.ir.decl_nodes.len() {
            parts.push(self.name(here));
            here = self.ir.decl_nodes[here.raw()].parent;
            guard += 1;
        }
        parts.reverse();
        parts.join(".")
    }

    fn bases(&self, decl: DeclId) -> &[SignedRefR] {
        if let Some(node) = self.ir.as_node(decl) {
            return &self.ir[self.ir.nodes[node.raw()].bases];
        }
        if let Some(edge) = self.ir.as_edge(decl) {
            return &self.ir[self.ir.edges[edge.raw()].bases];
        }
        &[]
    }

    /// Does `decl`'s base chain reach a declaration of this name?
    fn inherits(&self, decl: DeclId, name: &str) -> bool {
        let mut seen = HashSet::new();
        let mut stack = vec![decl];
        while let Some(here) = stack.pop() {
            if !seen.insert(here) {
                continue;
            }
            for base in self.bases(here) {
                let target = base.target();
                if self.name(target) == name {
                    return true;
                }
                stack.push(target);
            }
        }
        false
    }

    /// A child declaration's value, by field name.
    fn field(&self, decl: DeclId, field: &str) -> Option<&ValueR> {
        let mut child = self.ir.decl_nodes.get(decl.raw())?.first_child;
        let mut guard = 0usize;
        while child.is_some() && guard <= self.ir.decl_nodes.len() {
            let node = &self.ir.decl_nodes[child.raw()];
            if self.it.resolve(node.name) == field {
                return self.ir[node.anno].value.as_ref();
            }
            child = node.next_sibling;
            guard += 1;
        }
        None
    }

    fn number(&self, decl: DeclId, field: &str) -> Option<f64> {
        match self.field(decl, field) {
            Some(ValueR::Num(n)) => Some(*n),
            _ => None,
        }
    }

    fn text(&self, decl: DeclId, field: &str) -> Option<String> {
        match self.field(decl, field) {
            Some(ValueR::Str(s)) => Some(self.it.resolve(*s).to_string()),
            _ => None,
        }
    }

    fn reference(&self, decl: DeclId, field: &str) -> Option<DeclId> {
        match self.field(decl, field) {
            Some(ValueR::Ref(d)) => Some(*d),
            _ => None,
        }
    }

    /// The incidences of an edge, flattened across its arcs.
    fn incidences(&self, decl: DeclId) -> Vec<(&SignedRefR, DeclId, Option<f64>)> {
        self.ir
            .arcs
            .iter()
            .filter(|arc| arc.in_edge == decl)
            .flat_map(|arc| arc.refs.iter())
            .map(|r| {
                let a = self.ir.reference_atom(r).expect("model-local reference");
                let weight = a.weights.as_ref().and_then(|w| match w.first() {
                    Some(ValueR::Num(n)) => Some(*n),
                    _ => None,
                });
                (r, a.target, weight)
            })
            .collect()
    }

    fn project(&self) -> Result<Problem, ReadError> {
        let mut problem = Problem::default();

        // ── quantities ───────────────────────────────────────────────────────────
        let mut quantity_of: HashMap<DeclId, usize> = HashMap::new();
        for raw in 0..self.ir.decl_nodes.len() {
            let decl = DeclId::new(raw);
            if self.ir.decl_nodes[raw].kind != DeclKind::Node {
                continue;
            }
            let kind = if self.inherits(decl, "decision") {
                Kind::Decision
            } else if self.inherits(decl, "parameter") {
                Kind::Parameter
            } else if self.inherits(decl, "derived") {
                Kind::Derived
            } else {
                continue;
            };
            let name = self.qualified(decl);
            let unit_decl = self.reference(decl, "unit").ok_or(ReadError::UnitMissing {
                quantity: name.clone(),
            })?;
            let unit_name = self.name(unit_decl);
            let unit = Unit::named(unit_name).ok_or_else(|| ReadError::UnknownUnit {
                quantity: name.clone(),
                unit: unit_name.to_string(),
            })?;
            quantity_of.insert(decl, problem.quantities.len());
            problem.quantities.push(Quantity {
                decl,
                name,
                kind,
                unit,
                value: self.number(decl, "value"),
                lower: self.number(decl, "lower"),
                upper: self.number(decl, "upper"),
                start: self.number(decl, "start"),
                approximation: self.text(decl, "approximation"),
                provenance: Vec::new(),
            });
        }
        problem.decisions = (0..problem.quantities.len())
            .filter(|&i| problem.quantities[i].kind == Kind::Decision)
            .collect();

        // ── nets ─────────────────────────────────────────────────────────────────
        let mut net_of: HashMap<DeclId, usize> = HashMap::new();
        for raw in 0..self.ir.decl_nodes.len() {
            let decl = DeclId::new(raw);
            if self.ir.decl_nodes[raw].kind == DeclKind::Node && self.inherits(decl, "net") {
                net_of.insert(decl, problem.nets.len());
                problem.nets.push(decl);
                problem.net_names.push(self.qualified(decl));
            }
        }

        // ── branches, ground, and the edge-borne relations ───────────────────────
        let mut branch_of: HashMap<DeclId, usize> = HashMap::new();
        let mut grounds = Vec::new();
        let mut scenario_of: HashMap<DeclId, usize> = HashMap::new();
        let mut participation: Vec<(DeclId, Vec<DeclId>)> = Vec::new();
        let mut provenance: Vec<(DeclId, Vec<DeclId>)> = Vec::new();
        let mut pending: Vec<(DeclId, &'static str)> = Vec::new();

        for raw in 0..self.ir.decl_nodes.len() {
            let decl = DeclId::new(raw);
            if self.ir.decl_nodes[raw].kind == DeclKind::Node && self.inherits(decl, "scenario") {
                scenario_of.insert(decl, problem.scenarios.len());
                problem.scenarios.push(Scenario {
                    decl,
                    name: self.qualified(decl),
                    branches: Vec::new(),
                });
            }
            if self.ir.decl_nodes[raw].kind != DeclKind::Edge {
                continue;
            }
            for op in [
                "resistor",
                "ideal_source",
                "ground",
                "net_potential",
                "branch_power",
                "add",
                "subtract",
                "multiply",
                "divide",
                "square",
                "equality",
                "at_most",
                "at_least",
                "minimise",
                "participation",
                "approximation",
            ] {
                if self.inherits(decl, op) {
                    pending.push((decl, op));
                    break;
                }
            }
        }

        // Branches first: a readout names one, so they must exist before the relations are read.
        for &(decl, op) in &pending {
            let name = self.qualified(decl);
            let inc = self.incidences(decl);
            match op {
                "resistor" | "ideal_source" => {
                    let plus = signed(&inc, Sign::Plus);
                    let minus = signed(&inc, Sign::Minus);
                    let neutral = signed(&inc, Sign::Neutral);
                    if plus.len() != 1 || minus.len() != 1 || neutral.len() != 1 {
                        return Err(ReadError::Arity {
                            relation: name,
                            expected: "(+ net_a, - net_b, ~ value)",
                            found: format!("{}+ {}- {}~", plus.len(), minus.len(), neutral.len()),
                        });
                    }
                    let a = *net_of.get(&plus[0]).ok_or_else(|| ReadError::Unresolved {
                        at: name.clone(),
                        what: self.qualified(plus[0]),
                    })?;
                    let b = *net_of.get(&minus[0]).ok_or_else(|| ReadError::Unresolved {
                        at: name.clone(),
                        what: self.qualified(minus[0]),
                    })?;
                    let value =
                        *quantity_of
                            .get(&neutral[0])
                            .ok_or_else(|| ReadError::Unresolved {
                                at: name.clone(),
                                what: self.qualified(neutral[0]),
                            })?;
                    branch_of.insert(decl, problem.branches.len());
                    problem.branches.push(if op == "resistor" {
                        Branch::Resistor {
                            a,
                            b,
                            resistance: value,
                        }
                    } else {
                        Branch::Source {
                            a,
                            b,
                            voltage: value,
                        }
                    });
                    problem.branch_names.push(name);
                }
                "ground" => {
                    let neutral = signed(&inc, Sign::Neutral);
                    if neutral.len() != 1 {
                        return Err(ReadError::Arity {
                            relation: name,
                            expected: "(~ net)",
                            found: format!("{}~", neutral.len()),
                        });
                    }
                    grounds.push(*net_of.get(&neutral[0]).ok_or_else(|| {
                        ReadError::Unresolved {
                            at: name.clone(),
                            what: self.qualified(neutral[0]),
                        }
                    })?);
                }
                _ => {}
            }
        }
        if grounds.len() != 1 {
            return Err(ReadError::Ground {
                found: grounds.len(),
            });
        }
        problem.ground = Some(grounds[0]);

        // ── the rest ─────────────────────────────────────────────────────────────
        for &(decl, op) in &pending {
            let name = self.qualified(decl);
            let inc = self.incidences(decl);
            let plus = signed(&inc, Sign::Plus);
            let minus = signed(&inc, Sign::Minus);
            let neutral = signed(&inc, Sign::Neutral);
            let need = |set: &[DeclId],
                        i: usize,
                        map: &HashMap<DeclId, usize>|
             -> Result<usize, ReadError> {
                let target = *set.get(i).ok_or_else(|| ReadError::Arity {
                    relation: name.clone(),
                    expected: "more incidences",
                    found: format!("{} of the wanted sign", set.len()),
                })?;
                map.get(&target)
                    .copied()
                    .ok_or_else(|| ReadError::Unresolved {
                        at: name.clone(),
                        what: self.qualified(target),
                    })
            };

            match op {
                "resistor" | "ideal_source" | "ground" => {}
                "add" => {
                    let result = need(&plus, 0, &quantity_of)?;
                    let mut terms = Vec::new();
                    for r in &inc {
                        if r.0.sign() == -1 {
                            let q =
                                *quantity_of.get(&r.1).ok_or_else(|| ReadError::Unresolved {
                                    at: name.clone(),
                                    what: self.qualified(r.1),
                                })?;
                            terms.push((q, r.2.unwrap_or(1.0)));
                        }
                    }
                    problem.relations.push(Relation::Add { result, terms });
                }
                "subtract" | "multiply" | "divide" => {
                    let result = need(&plus, 0, &quantity_of)?;
                    let a = need(&minus, 0, &quantity_of)?;
                    let b = need(&minus, 1, &quantity_of)?;
                    problem.relations.push(match op {
                        "subtract" => Relation::Subtract { result, a, b },
                        "multiply" => Relation::Multiply { result, a, b },
                        _ => Relation::Divide { result, a, b },
                    });
                }
                "square" => {
                    let result = need(&plus, 0, &quantity_of)?;
                    let a = need(&minus, 0, &quantity_of)?;
                    problem.relations.push(Relation::Square { result, a });
                }
                "net_potential" => {
                    let result = need(&plus, 0, &quantity_of)?;
                    let net = need(&minus, 0, &net_of)?;
                    let scenario = need(&neutral, 0, &scenario_of)?;
                    problem.relations.push(Relation::NetPotential {
                        result,
                        net,
                        scenario,
                    });
                }
                "branch_power" => {
                    let result = need(&plus, 0, &quantity_of)?;
                    let branch = need(&minus, 0, &branch_of)?;
                    let scenario = need(&neutral, 0, &scenario_of)?;
                    problem.relations.push(Relation::BranchPower {
                        result,
                        branch,
                        scenario,
                    });
                }
                "equality" | "at_most" | "at_least" => {
                    let left = need(&plus, 0, &quantity_of)?;
                    let right = need(&minus, 0, &quantity_of)?;
                    problem.constraints.push(Constraint {
                        decl,
                        name,
                        sense: match op {
                            "equality" => Sense::Equality,
                            "at_most" => Sense::AtMost,
                            _ => Sense::AtLeast,
                        },
                        left,
                        right,
                    });
                }
                "minimise" => {
                    for r in &inc {
                        if r.0.sign() == -1 {
                            let q =
                                *quantity_of.get(&r.1).ok_or_else(|| ReadError::Unresolved {
                                    at: name.clone(),
                                    what: self.qualified(r.1),
                                })?;
                            problem.objective.push((q, r.2.unwrap_or(1.0)));
                        }
                    }
                }
                "participation" => {
                    let scenario = *plus.first().ok_or_else(|| ReadError::Arity {
                        relation: name.clone(),
                        expected: "(+ scenario, - element, ...)",
                        found: "no scenario".into(),
                    })?;
                    participation.push((scenario, minus.clone()));
                }
                "approximation" => {
                    let quantity = *plus.first().ok_or_else(|| ReadError::Arity {
                        relation: name.clone(),
                        expected: "(+ quantity, - evidence, ...)",
                        found: "no quantity".into(),
                    })?;
                    provenance.push((quantity, minus.clone()));
                }
                _ => {}
            }
        }

        for (quantity, evidence) in provenance {
            if let Some(&i) = quantity_of.get(&quantity) {
                problem.quantities[i].provenance = evidence;
            }
        }

        // ── scenario membership ──────────────────────────────────────────────────
        // A branch named by no participation edge takes part in every scenario; one named by any
        // takes part only where it is named. That is what makes the base topology shared rather than
        // copied, and it is the whole of the scenario mechanism.
        let mut restricted: HashMap<usize, HashSet<usize>> = HashMap::new();
        for (scenario, elements) in &participation {
            let Some(&s) = scenario_of.get(scenario) else {
                continue;
            };
            for element in elements {
                if let Some(&b) = branch_of.get(element) {
                    restricted.entry(b).or_default().insert(s);
                }
            }
        }
        for (s, scenario) in problem.scenarios.iter_mut().enumerate() {
            scenario.branches = (0..problem.branches.len())
                .filter(|b| restricted.get(b).is_none_or(|only| only.contains(&s)))
                .collect();
        }

        problem.order = order_relations(&problem)?;
        Ok(problem)
    }
}

/// Relations in an order where every operand is defined before it is read.
///
/// A net readout depends on every element value of its scenario's branches, which is how a load
/// whose resistance is itself derived gets evaluated before the network that contains it.
fn order_relations(problem: &Problem) -> Result<Vec<usize>, ReadError> {
    let mut produced_by: HashMap<usize, usize> = HashMap::new();
    for (i, relation) in problem.relations.iter().enumerate() {
        produced_by.insert(relation.result(), i);
    }

    let needs = |relation: &Relation| -> Vec<usize> {
        match relation {
            Relation::Add { terms, .. } => terms.iter().map(|(q, _)| *q).collect(),
            Relation::Subtract { a, b, .. }
            | Relation::Multiply { a, b, .. }
            | Relation::Divide { a, b, .. } => vec![*a, *b],
            Relation::Square { a, .. } => vec![*a],
            Relation::NetPotential { scenario, .. } | Relation::BranchPower { scenario, .. } => {
                problem.scenarios[*scenario]
                    .branches
                    .iter()
                    .map(|&b| match problem.branches[b] {
                        Branch::Resistor { resistance, .. } => resistance,
                        Branch::Source { voltage, .. } => voltage,
                    })
                    .collect()
            }
        }
    };

    let mut done = vec![false; problem.relations.len()];
    let mut ready: HashSet<usize> = problem
        .quantities
        .iter()
        .enumerate()
        .filter(|(_, q)| q.kind != Kind::Derived)
        .map(|(i, _)| i)
        .collect();
    let mut order = Vec::new();
    loop {
        let mut moved = false;
        for (i, relation) in problem.relations.iter().enumerate() {
            if done[i] {
                continue;
            }
            if needs(relation).iter().all(|q| ready.contains(q)) {
                ready.insert(relation.result());
                order.push(i);
                done[i] = true;
                moved = true;
            }
        }
        if !moved {
            break;
        }
    }
    if order.len() != problem.relations.len() {
        return Err(ReadError::Cyclic {
            unresolved: problem
                .relations
                .iter()
                .enumerate()
                .filter(|(i, _)| !done[*i])
                .map(|(_, r)| problem.quantities[r.result()].name.clone())
                .collect(),
        });
    }
    Ok(order)
}

#[derive(Clone, Copy, PartialEq)]
enum Sign {
    Plus,
    Minus,
    Neutral,
}

fn signed(inc: &[(&SignedRefR, DeclId, Option<f64>)], want: Sign) -> Vec<DeclId> {
    inc.iter()
        .filter(|(r, _, _)| {
            matches!(
                (r.sign(), want),
                (1, Sign::Plus) | (-1, Sign::Minus) | (0, Sign::Neutral)
            )
        })
        .map(|(_, d, _)| *d)
        .collect()
}

/// Unused import guard: `SymId` is part of the resolver's signature in `hymeko_core`.
const _: Option<SymId> = None;
