//! The solver-neutral constrained-optimisation problem, and its evaluation.
//!
//! ```text
//! minimise   f(x, p)
//! subject to h(x, p) = 0
//!            g(x, p) >= 0
//!            lower <= x <= upper
//! ```
//!
//! # What is deliberately not here
//!
//! No bulb, no `R1`, no voltage divider, no circuit. A [`Problem`] is decision variables, fixed
//! parameters, a graph of relations that define derived quantities, constraints, and an objective —
//! and a backend that receives one cannot tell what it is a design of.
//!
//! What IS here is the branch network, because a network is a general structure and its equations
//! must be assembled from a declared topology rather than written down. That is the difference the
//! architecture turns on: `hymeko_optimize` knows what a resistor network is because a meta-model
//! declared one; it does not know what THIS network is.
//!
//! # Units
//!
//! The language has no type system beyond inheritance, so `unit units.volt;` is a fact the IR
//! carries and nothing in the parser checks. The check happens HERE, at the typed projection, which
//! is the last point before the problem becomes an array of numbers. Adding volts to ohms, or
//! constraining a voltage to equal a resistance, is rejected with both units named.

use std::collections::HashMap;

use hymeko::common::ids::DeclId;

use crate::dual::Dual;
use crate::network::{self, Branch, NetworkError};

/// A physical dimension as exponents of (mass, length, time, current).
///
/// Enough for an electrical model and extended by adding a constant, not by adding a case: the
/// arithmetic below is dimensional analysis and does not know what the dimensions mean.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Unit {
    pub mass: i8,
    pub length: i8,
    pub time: i8,
    pub current: i8,
}

impl Unit {
    pub const DIMENSIONLESS: Self = Self {
        mass: 0,
        length: 0,
        time: 0,
        current: 0,
    };
    pub const AMPERE: Self = Self {
        mass: 0,
        length: 0,
        time: 0,
        current: 1,
    };
    pub const VOLT: Self = Self {
        mass: 1,
        length: 2,
        time: -3,
        current: -1,
    };
    pub const OHM: Self = Self {
        mass: 1,
        length: 2,
        time: -3,
        current: -2,
    };
    pub const WATT: Self = Self {
        mass: 1,
        length: 2,
        time: -3,
        current: 0,
    };

    /// The unit a meta-model declaration names, or `None` for one this projection does not know.
    ///
    /// By NAME, matching `opt.units`. A unit the meta-model adds and this does not is an explicit
    /// failure rather than a silent dimensionless.
    pub fn named(name: &str) -> Option<Self> {
        Some(match name {
            "dimensionless" => Self::DIMENSIONLESS,
            "volt" => Self::VOLT,
            "ampere" => Self::AMPERE,
            "ohm" => Self::OHM,
            "watt" => Self::WATT,
            _ => return None,
        })
    }

    pub fn mul(self, rhs: Self) -> Self {
        Self {
            mass: self.mass + rhs.mass,
            length: self.length + rhs.length,
            time: self.time + rhs.time,
            current: self.current + rhs.current,
        }
    }

    pub fn div(self, rhs: Self) -> Self {
        Self {
            mass: self.mass - rhs.mass,
            length: self.length - rhs.length,
            time: self.time - rhs.time,
            current: self.current - rhs.current,
        }
    }

    /// The name this projection would print, or a dimension vector when it has no name.
    pub fn label(self) -> String {
        for (name, unit) in [
            ("dimensionless", Self::DIMENSIONLESS),
            ("volt", Self::VOLT),
            ("ampere", Self::AMPERE),
            ("ohm", Self::OHM),
            ("watt", Self::WATT),
        ] {
            if unit == self {
                return name.to_string();
            }
        }
        format!(
            "M^{} L^{} T^{} I^{}",
            self.mass, self.length, self.time, self.current
        )
    }
}

/// What a quantity is to the solver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// The solver may choose it.
    Decision,
    /// The model fixes it.
    Parameter,
    /// A relation defines it.
    Derived,
}

/// A named scalar.
#[derive(Debug, Clone, PartialEq)]
pub struct Quantity {
    /// The declaration it came from — the stable identity, which survives every renumbering below.
    pub decl: DeclId,
    /// Its qualified name, for a human and for a result file.
    pub name: String,
    pub kind: Kind,
    pub unit: Unit,
    /// A parameter's value.
    pub value: Option<f64>,
    /// A decision variable's bounds and starting point.
    pub lower: Option<f64>,
    pub upper: Option<f64>,
    pub start: Option<f64>,
    /// Set where the declaration says this quantity stands in for something it is not.
    pub approximation: Option<String>,
    /// Declarations cited as evidence for that approximation.
    pub provenance: Vec<DeclId>,
}

/// A relation defining one quantity from others.
#[derive(Debug, Clone, PartialEq)]
pub enum Relation {
    /// `y = sum(coefficient * operand)`.
    Add {
        result: usize,
        terms: Vec<(usize, f64)>,
    },
    Subtract {
        result: usize,
        a: usize,
        b: usize,
    },
    Multiply {
        result: usize,
        a: usize,
        b: usize,
    },
    Divide {
        result: usize,
        a: usize,
        b: usize,
    },
    Square {
        result: usize,
        a: usize,
    },
    /// The potential of a net once the scenario's network is solved.
    NetPotential {
        result: usize,
        net: usize,
        scenario: usize,
    },
    /// The power a branch dissipates, `V * I`, once the scenario's network is solved.
    BranchPower {
        result: usize,
        branch: usize,
        scenario: usize,
    },
}

impl Relation {
    pub fn result(&self) -> usize {
        match *self {
            Self::Add { result, .. }
            | Self::Subtract { result, .. }
            | Self::Multiply { result, .. }
            | Self::Divide { result, .. }
            | Self::Square { result, .. }
            | Self::NetPotential { result, .. }
            | Self::BranchPower { result, .. } => result,
        }
    }
}

/// Which way a constraint faces.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sense {
    /// `left = right`, projected as `h = left - right`.
    Equality,
    /// `left <= right`, projected as `g = right - left >= 0`.
    AtMost,
    /// `left >= right`, projected as `g = left - right >= 0`.
    AtLeast,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Constraint {
    pub decl: DeclId,
    pub name: String,
    pub sense: Sense,
    pub left: usize,
    pub right: usize,
}

/// One variant of the base model: which branches take part.
#[derive(Debug, Clone, PartialEq)]
pub struct Scenario {
    pub decl: DeclId,
    pub name: String,
    /// Indices into [`Problem::branches`].
    pub branches: Vec<usize>,
}

/// Everything a numerical backend needs and nothing about what it is a design of.
#[derive(Debug, Clone, Default)]
pub struct Problem {
    pub quantities: Vec<Quantity>,
    /// Indices into `quantities`, in the order the decision vector uses.
    pub decisions: Vec<usize>,
    pub relations: Vec<Relation>,
    /// Relations in an order where every operand is defined before it is read.
    pub order: Vec<usize>,
    pub constraints: Vec<Constraint>,
    /// `(quantity, weight)`; a weight of zero is a carried and visible exclusion.
    pub objective: Vec<(usize, f64)>,
    pub branches: Vec<Branch>,
    pub branch_names: Vec<String>,
    pub nets: Vec<DeclId>,
    pub net_names: Vec<String>,
    pub ground: Option<usize>,
    pub scenarios: Vec<Scenario>,
}

/// Why a problem could not be evaluated or is not well posed.
#[derive(Debug, Clone, PartialEq)]
pub enum ProblemError {
    /// A relation reads a quantity nothing has defined yet, and no ordering fixes it.
    CyclicRelations { unresolved: Vec<String> },
    /// Two relations define the same quantity, so the model does not say which holds.
    DoublyDefined { quantity: String },
    /// A derived quantity no relation defines.
    Undefined { quantity: String },
    /// A parameter with no value.
    ValueMissing { quantity: String },
    /// Dimensions that cannot combine.
    UnitMismatch {
        at: String,
        left: String,
        right: String,
    },
    /// A quantity declaring itself an approximation with nothing cited for it.
    ApproximationWithoutProvenance { quantity: String },
    /// The network could not be solved.
    Network {
        scenario: String,
        error: NetworkError,
    },
}

/// A full evaluation: every quantity's value and gradient, plus what the constraints came to.
#[derive(Debug, Clone)]
pub struct Evaluation {
    pub values: Vec<Dual>,
    pub objective: Dual,
    /// `h(x)`, one per equality.
    pub equalities: Vec<Dual>,
    /// `g(x) >= 0`, one per inequality.
    pub inequalities: Vec<Dual>,
}

impl Problem {
    /// How many decision variables.
    pub fn dimension(&self) -> usize {
        self.decisions.len()
    }

    /// The quantity a stable declaration names, if the projection kept it.
    pub fn index_of(&self, decl: DeclId) -> Option<usize> {
        self.quantities.iter().position(|q| q.decl == decl)
    }

    /// The quantity a qualified name names.
    pub fn index_of_name(&self, name: &str) -> Option<usize> {
        self.quantities.iter().position(|q| q.name == name)
    }

    /// The starting point the model declares, one entry per decision variable.
    pub fn start(&self) -> Vec<f64> {
        self.decisions
            .iter()
            .map(|&q| {
                let quantity = &self.quantities[q];
                quantity.start.or(quantity.lower).unwrap_or(1.0)
            })
            .collect()
    }

    /// `(lower, upper)` per decision variable.
    pub fn bounds(&self) -> Vec<(f64, f64)> {
        self.decisions
            .iter()
            .map(|&q| {
                let quantity = &self.quantities[q];
                (
                    quantity.lower.unwrap_or(f64::NEG_INFINITY),
                    quantity.upper.unwrap_or(f64::INFINITY),
                )
            })
            .collect()
    }

    /// Every check that does not need a value: definedness, units, and provenance.
    ///
    /// Run once when the problem is built, so a malformed model fails before a solver is started
    /// rather than after it has wandered.
    pub fn check(&self) -> Result<(), ProblemError> {
        // exactly one relation per derived quantity
        let mut defined: HashMap<usize, usize> = HashMap::new();
        for relation in &self.relations {
            let result = relation.result();
            if defined.insert(result, 1).is_some() {
                return Err(ProblemError::DoublyDefined {
                    quantity: self.quantities[result].name.clone(),
                });
            }
        }
        for (i, quantity) in self.quantities.iter().enumerate() {
            match quantity.kind {
                Kind::Derived if !defined.contains_key(&i) => {
                    return Err(ProblemError::Undefined {
                        quantity: quantity.name.clone(),
                    });
                }
                Kind::Parameter if quantity.value.is_none() => {
                    return Err(ProblemError::ValueMissing {
                        quantity: quantity.name.clone(),
                    });
                }
                _ => {}
            }
            // An approximation is a claim, and a claim with nothing behind it does not ship.
            if quantity.approximation.is_some() && quantity.provenance.is_empty() {
                return Err(ProblemError::ApproximationWithoutProvenance {
                    quantity: quantity.name.clone(),
                });
            }
        }
        self.check_units()
    }

    /// Dimensional analysis over the relations and the constraints.
    ///
    /// The language cannot do this — it has no type system — so it happens here, which is the last
    /// point before the problem becomes an array of numbers.
    fn check_units(&self) -> Result<(), ProblemError> {
        let unit = |i: usize| self.quantities[i].unit;
        let name = |i: usize| self.quantities[i].name.clone();

        for relation in &self.relations {
            match relation {
                Relation::Add { result, terms } => {
                    for &(term, _) in terms {
                        if unit(term) != unit(*result) {
                            return Err(ProblemError::UnitMismatch {
                                at: format!("{} = ... + {}", name(*result), name(term)),
                                left: unit(*result).label(),
                                right: unit(term).label(),
                            });
                        }
                    }
                }
                Relation::Subtract { result, a, b } => {
                    if unit(*a) != unit(*b) || unit(*a) != unit(*result) {
                        return Err(ProblemError::UnitMismatch {
                            at: format!("{} = {} - {}", name(*result), name(*a), name(*b)),
                            left: unit(*a).label(),
                            right: unit(*b).label(),
                        });
                    }
                }
                Relation::Multiply { result, a, b } => {
                    let want = unit(*a).mul(unit(*b));
                    if want != unit(*result) {
                        return Err(ProblemError::UnitMismatch {
                            at: format!("{} = {} * {}", name(*result), name(*a), name(*b)),
                            left: unit(*result).label(),
                            right: want.label(),
                        });
                    }
                }
                Relation::Divide { result, a, b } => {
                    let want = unit(*a).div(unit(*b));
                    if want != unit(*result) {
                        return Err(ProblemError::UnitMismatch {
                            at: format!("{} = {} / {}", name(*result), name(*a), name(*b)),
                            left: unit(*result).label(),
                            right: want.label(),
                        });
                    }
                }
                Relation::Square { result, a } => {
                    let want = unit(*a).mul(unit(*a));
                    if want != unit(*result) {
                        return Err(ProblemError::UnitMismatch {
                            at: format!("{} = {}^2", name(*result), name(*a)),
                            left: unit(*result).label(),
                            right: want.label(),
                        });
                    }
                }
                Relation::NetPotential { result, .. } => {
                    if unit(*result) != Unit::VOLT {
                        return Err(ProblemError::UnitMismatch {
                            at: format!("{} reads a net potential", name(*result)),
                            left: unit(*result).label(),
                            right: Unit::VOLT.label(),
                        });
                    }
                }
                Relation::BranchPower { result, .. } => {
                    if unit(*result) != Unit::WATT {
                        return Err(ProblemError::UnitMismatch {
                            at: format!("{} reads a branch power", name(*result)),
                            left: unit(*result).label(),
                            right: Unit::WATT.label(),
                        });
                    }
                }
            }
        }

        // A constraint compares two quantities, so they must be the same kind of thing.
        for constraint in &self.constraints {
            if unit(constraint.left) != unit(constraint.right) {
                return Err(ProblemError::UnitMismatch {
                    at: constraint.name.clone(),
                    left: unit(constraint.left).label(),
                    right: unit(constraint.right).label(),
                });
            }
        }

        // Every term of the objective is added together, so they must agree.
        if let Some(&(first, _)) = self.objective.first() {
            for &(term, _) in &self.objective {
                if unit(term) != unit(first) {
                    return Err(ProblemError::UnitMismatch {
                        at: format!("objective term {}", name(term)),
                        left: unit(first).label(),
                        right: unit(term).label(),
                    });
                }
            }
        }
        Ok(())
    }

    /// Evaluate everything at one decision vector: values, gradients, objective and constraints.
    ///
    /// # Panics
    /// If `x` is not [`Self::dimension`] long.
    pub fn evaluate(&self, x: &[f64]) -> Result<Evaluation, ProblemError> {
        let n = self.dimension();
        assert_eq!(x.len(), n, "the decision vector must cover every variable");

        let mut values: Vec<Option<Dual>> = vec![None; self.quantities.len()];
        for (k, &q) in self.decisions.iter().enumerate() {
            values[q] = Some(Dual::variable(x[k], k, n));
        }
        for (i, quantity) in self.quantities.iter().enumerate() {
            if quantity.kind == Kind::Parameter {
                let value = quantity.value.ok_or_else(|| ProblemError::ValueMissing {
                    quantity: quantity.name.clone(),
                })?;
                values[i] = Some(Dual::constant(value, n));
            }
        }

        // One solved network per scenario, computed when a readout first asks for it — the element
        // values it needs are themselves quantities, and a derived one (a linearised load) has to be
        // evaluated before the network can be assembled. The relation order guarantees that.
        let mut solved: HashMap<usize, network::Solution> = HashMap::new();

        for &r in &self.order {
            let relation = &self.relations[r];
            let get = |i: usize, values: &Vec<Option<Dual>>| -> Dual {
                values[i].clone().unwrap_or_else(|| {
                    panic!("relation order left {} undefined", self.quantities[i].name)
                })
            };
            let out = match *relation {
                Relation::Add { result, ref terms } => {
                    let mut acc = Dual::zero(n);
                    for &(term, coefficient) in terms {
                        acc = acc.add(&get(term, &values).scale(coefficient));
                    }
                    (result, acc)
                }
                Relation::Subtract { result, a, b } => {
                    (result, get(a, &values).sub(&get(b, &values)))
                }
                Relation::Multiply { result, a, b } => {
                    (result, get(a, &values).mul(&get(b, &values)))
                }
                Relation::Divide { result, a, b } => {
                    (result, get(a, &values).div(&get(b, &values)))
                }
                Relation::Square { result, a } => {
                    let v = get(a, &values);
                    (result, v.mul(&v))
                }
                Relation::NetPotential {
                    result,
                    net,
                    scenario,
                } => {
                    let solution = self.solve_scenario(scenario, &values, &mut solved, n)?;
                    (result, solution.potential[net].clone())
                }
                Relation::BranchPower {
                    result,
                    branch,
                    scenario,
                } => {
                    let solution = self.solve_scenario(scenario, &values, &mut solved, n)?;
                    let (a, b) = self.branches[branch].nets();
                    let across = solution.potential[a].sub(&solution.potential[b]);
                    // `V * I`, and for a source that comes out negative because it delivers rather
                    // than dissipates — so the magnitude is what a power readout means.
                    let power = across.mul(&solution.current[branch]);
                    let signed = if power.value < 0.0 {
                        power.neg()
                    } else {
                        power
                    };
                    (result, signed)
                }
            };
            values[out.0] = Some(out.1);
        }

        let resolved: Vec<Dual> = values
            .into_iter()
            .enumerate()
            .map(|(i, v)| {
                v.unwrap_or_else(|| panic!("{} was never given a value", self.quantities[i].name))
            })
            .collect();

        let mut objective = Dual::zero(n);
        for &(term, weight) in &self.objective {
            objective = objective.add(&resolved[term].scale(weight));
        }

        let mut equalities = Vec::new();
        let mut inequalities = Vec::new();
        for constraint in &self.constraints {
            let (left, right) = (&resolved[constraint.left], &resolved[constraint.right]);
            match constraint.sense {
                Sense::Equality => equalities.push(left.sub(right)),
                Sense::AtMost => inequalities.push(right.sub(left)),
                Sense::AtLeast => inequalities.push(left.sub(right)),
            }
        }

        Ok(Evaluation {
            values: resolved,
            objective,
            equalities,
            inequalities,
        })
    }

    fn solve_scenario<'a>(
        &self,
        scenario: usize,
        values: &Vec<Option<Dual>>,
        cache: &'a mut HashMap<usize, network::Solution>,
        width: usize,
    ) -> Result<&'a network::Solution, ProblemError> {
        if !cache.contains_key(&scenario) {
            let taking = &self.scenarios[scenario].branches;
            let branches: Vec<Branch> = taking.iter().map(|&b| self.branches[b]).collect();
            // Element values are indexed by quantity, and a branch inside this scenario refers to
            // them by the same index, so the value table is passed through whole.
            let element: Vec<Dual> = self
                .quantities
                .iter()
                .enumerate()
                .map(|(i, q)| {
                    values[i]
                        .clone()
                        .unwrap_or_else(|| Dual::constant(q.value.unwrap_or(0.0), width))
                })
                .collect();
            let solution = network::solve(self.nets.len(), &branches, &element, self.ground, width)
                .map_err(|error| ProblemError::Network {
                    scenario: self.scenarios[scenario].name.clone(),
                    error,
                })?;
            // The solution is indexed by the scenario's own branch order; widen it back to the
            // problem's, so a readout can name any branch and get zero where it does not take part.
            let mut current = vec![Dual::zero(width); self.branches.len()];
            for (local, &global) in taking.iter().enumerate() {
                current[global] = solution.current[local].clone();
            }
            cache.insert(
                scenario,
                network::Solution {
                    potential: solution.potential,
                    current,
                },
            );
        }
        Ok(cache.get(&scenario).expect("just inserted"))
    }
}
