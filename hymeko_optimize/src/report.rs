//! The typed result, and its serialisation as evidence.
//!
//! # Why the result carries declarations and not just numbers
//!
//! A backend returns an array. An array is anonymous: position 0 is `R1` only because somebody
//! remembered the order, and the moment the model gains a variable the memory is wrong and nothing
//! says so. Every value here therefore carries the [`DeclId`] it came from, which is the identity the
//! IR assigns and the only thing in this pipeline that does not move.
//!
//! JSON appears at the very end and only here. It is a record of what happened — inputs, outputs,
//! residuals, provenance — and never a thing to edit. Editing the JSON changes nothing, because the
//! next run reads the `.hymeko`. That is the test of which artefact is the authority.

use hymeko::common::ids::DeclId;

use crate::problem::{Evaluation, Kind, Problem, Sense};

/// A quantity as it came out.
#[derive(Debug, Clone, PartialEq)]
pub struct Resolved {
    /// The declaration — the identity that survives from the source file to here.
    pub decl: DeclId,
    pub name: String,
    pub kind: Kind,
    pub value: f64,
    pub unit: String,
    /// Present where the model declared the quantity an approximation.
    pub approximation: Option<String>,
}

/// A constraint as it came out.
#[derive(Debug, Clone, PartialEq)]
pub struct Residual {
    pub decl: DeclId,
    pub name: String,
    pub sense: Sense,
    /// `h(x)` for an equality, `g(x)` for an inequality.
    pub value: f64,
    /// Whether the constraint holds to [`Solved::tolerance`].
    pub satisfied: bool,
}

/// What a solve produced.
#[derive(Debug, Clone)]
pub struct Solved {
    /// Which backend ran, so a result file says how it was obtained.
    pub backend: String,
    pub converged: bool,
    pub iterations: usize,
    pub tolerance: f64,
    pub objective: f64,
    /// The decision variables, in the problem's own order.
    pub decisions: Vec<Resolved>,
    /// Every quantity, decision and parameter and derived alike.
    pub quantities: Vec<Resolved>,
    pub equalities: Vec<Residual>,
    pub inequalities: Vec<Residual>,
    /// The starting point this run used, so a multistart record is reproducible.
    pub start: Vec<f64>,
}

impl Solved {
    /// Build a typed result from a converged evaluation.
    pub fn of(
        problem: &Problem,
        evaluation: &Evaluation,
        backend: &str,
        converged: bool,
        iterations: usize,
        tolerance: f64,
        start: &[f64],
    ) -> Self {
        let resolve = |i: usize| {
            let q = &problem.quantities[i];
            Resolved {
                decl: q.decl,
                name: q.name.clone(),
                kind: q.kind,
                value: evaluation.values[i].value,
                unit: q.unit.label(),
                approximation: q.approximation.clone(),
            }
        };

        let mut equalities = Vec::new();
        let mut inequalities = Vec::new();
        let (mut e, mut g) = (0usize, 0usize);
        for constraint in &problem.constraints {
            match constraint.sense {
                Sense::Equality => {
                    let value = evaluation.equalities[e].value;
                    e += 1;
                    equalities.push(Residual {
                        decl: constraint.decl,
                        name: constraint.name.clone(),
                        sense: constraint.sense,
                        value,
                        satisfied: value.abs() <= tolerance,
                    });
                }
                _ => {
                    let value = evaluation.inequalities[g].value;
                    g += 1;
                    inequalities.push(Residual {
                        decl: constraint.decl,
                        name: constraint.name.clone(),
                        sense: constraint.sense,
                        value,
                        satisfied: value >= -tolerance,
                    });
                }
            }
        }

        Self {
            backend: backend.to_string(),
            converged,
            iterations,
            tolerance,
            objective: evaluation.objective.value,
            decisions: problem.decisions.iter().map(|&i| resolve(i)).collect(),
            quantities: (0..problem.quantities.len()).map(resolve).collect(),
            equalities,
            inequalities,
            start: start.to_vec(),
        }
    }

    /// Every constraint holds and the solve reported convergence.
    pub fn feasible(&self) -> bool {
        self.converged
            && self.equalities.iter().all(|r| r.satisfied)
            && self.inequalities.iter().all(|r| r.satisfied)
    }

    /// A quantity by its qualified name, or by the last segment of one.
    pub fn value_of(&self, name: &str) -> Option<f64> {
        self.quantities
            .iter()
            .find(|q| q.name == name || q.name.ends_with(&format!(".{name}")))
            .map(|q| q.value)
    }

    /// The result as JSON evidence.
    ///
    /// Written by hand rather than derived, because this is the one artefact that must not acquire a
    /// schema somebody could be tempted to edit and feed back in. It is a record.
    pub fn to_json(&self) -> String {
        let mut out = String::new();
        out.push_str("{\n");
        out.push_str(&format!("  \"backend\": {},\n", quote(&self.backend)));
        out.push_str(&format!("  \"converged\": {},\n", self.converged));
        out.push_str(&format!("  \"iterations\": {},\n", self.iterations));
        out.push_str(&format!("  \"tolerance\": {:e},\n", self.tolerance));
        out.push_str(&format!("  \"objective\": {},\n", number(self.objective)));
        out.push_str(&format!(
            "  \"start\": [{}],\n",
            self.start
                .iter()
                .map(|v| number(*v))
                .collect::<Vec<_>>()
                .join(", ")
        ));
        out.push_str("  \"quantities\": [\n");
        let rows: Vec<String> = self
            .quantities
            .iter()
            .map(|q| {
                let mut row = format!(
                    "    {{ \"decl\": {}, \"name\": {}, \"kind\": {}, \"unit\": {}, \"value\": {}",
                    q.decl.raw(),
                    quote(&q.name),
                    quote(match q.kind {
                        Kind::Decision => "decision",
                        Kind::Parameter => "parameter",
                        Kind::Derived => "derived",
                    }),
                    quote(&q.unit),
                    number(q.value)
                );
                if let Some(note) = &q.approximation {
                    row.push_str(&format!(", \"approximation\": {}", quote(note)));
                }
                row.push_str(" }");
                row
            })
            .collect();
        out.push_str(&rows.join(",\n"));
        out.push_str("\n  ],\n  \"constraints\": [\n");
        let rows: Vec<String> = self
            .equalities
            .iter()
            .chain(&self.inequalities)
            .map(|r| {
                format!(
                    "    {{ \"decl\": {}, \"name\": {}, \"sense\": {}, \"residual\": {}, \"satisfied\": {} }}",
                    r.decl.raw(),
                    quote(&r.name),
                    quote(match r.sense {
                        Sense::Equality => "equality",
                        Sense::AtMost => "at_most",
                        Sense::AtLeast => "at_least",
                    }),
                    number(r.value),
                    r.satisfied
                )
            })
            .collect();
        out.push_str(&rows.join(",\n"));
        out.push_str("\n  ]\n}\n");
        out
    }
}

fn quote(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

/// JSON has no infinity and no NaN, and silently writing `null` would lose the fact that a run
/// diverged. It is written as a string instead, which is invalid as a number and obvious as a fault.
fn number(v: f64) -> String {
    if v.is_finite() {
        format!("{v:.12}")
    } else {
        format!("\"{v}\"")
    }
}
