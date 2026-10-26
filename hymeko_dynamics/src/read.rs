//! IR to [`System`]: the dynamical projection.
//!
//! Everything is found by INHERITANCE. A declaration is a state coordinate because its base chain
//! reaches `ph.stores.effort_store` or `ph.stores.flow_store` — a capacitance or a spring, an
//! inductance or a mass — and not because it is called `charge`.
//!
//! # Where the array positions come from
//!
//! The state vector's layout is DECLARATION ORDER, and this is the only place in the pipeline where
//! an order matters. It is deterministic — the arena numbers declarations as the source file
//! declares them — and every coordinate keeps its [`DeclId`] in [`Coordinate::decl`], so a position
//! is never the thing anybody has to remember. Reordering the model reorders the state vector and
//! changes no result, because nothing downstream reads a position it did not get from here.
//!
//! # A duplication worth naming
//!
//! The inheritance walk and the field readers below are the same shape as the ones in
//! `hymeko_optimize::read`. They are IR *traversal*, not semantics — no vocabulary is duplicated,
//! and the two crates share no model between them — but the helper itself would be better lifted
//! into one place. It is written twice here rather than refactored across a shipped crate mid-flight,
//! and that is a debt rather than a design.

use std::collections::HashSet;

use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{DeclKind, Ir, SignedRefR, ValueR};
use hymeko::resolution::interner::Interner;
use hymeko_numerics::{Interconnection, Port, PortHamiltonian};

use crate::field::{Region, Shape};
use crate::system::{
    Blueprint, Coordinate, Declared, Drive, LossSource, Observable, PortSource, Readout, Sweep,
    System, Terminal, Wire,
};

/// Why a declaration could not be projected.
#[derive(Debug, Clone, PartialEq)]
pub enum ReadError {
    /// A coupling bypasses a sealed subsystem or its public port cannot be resolved.
    Boundary { at: String, why: String },
    /// A structure edge whose incidences do not match what it takes.
    Arity {
        at: String,
        expected: &'static str,
        found: String,
    },
    /// A reference to something the projection needs and did not find.
    Unresolved { at: String, what: String },
    /// A coordinate with no `capacity`, or one that is not positive.
    Capacity {
        coordinate: String,
        value: Option<f64>,
    },
    /// A dissipation that would create energy.
    NegativeDissipation { at: String, value: f64 },
    /// A waveform missing `amplitude` or `frequency`.
    Waveform { at: String, missing: &'static str },
    /// A declared sweep that does not say what it varies, or over what.
    Sweep { at: String, missing: &'static str },
    /// A mode shape missing one of its dimensions, or given one that is not positive.
    Shape {
        at: String,
        field: &'static str,
        value: Option<f64>,
    },
    /// A probe whose two leads are not joined by any run of conductors.
    ///
    /// Not a large reading: no reading. A potential difference is defined along a path, and between
    /// two separated pieces of circuit there is none.
    Disconnected {
        at: String,
        plus: String,
        minus: String,
    },
    /// More than one terminal declared as the reference, or a potential asked for without one.
    Reference { at: String, why: &'static str },
    /// No state coordinate at all — there is no system to project.
    Empty,
}

/// A readout as the first pass sees it.
///
/// Probes name TERMINALS and CONDUCTORS, and those are read after the coordinates are. Rather than
/// force the passes into an order the model does not have, a probe waits here and is resolved once
/// the circuit exists — which also keeps the observables in the order the model declares them.
enum Pending {
    Ready(Readout),
    Across { plus: DeclId, minus: DeclId },
    Potential { terminal: DeclId },
}

/// The concrete kinds this projection knows, by domain.
///
/// Named leaves rather than the roots they share, and not for want of trying: a declaration that
/// inherits `effort_store` includes the VOCABULARY's own `ph.stores.capacitive`, which is a type and
/// not a store and has no capacity to read. The language draws no distinction the reader can see
/// between declaring a subtype and instantiating one — both arrive as a base — so the reader names
/// what it is looking for. Adding a domain is a line here, which is the right size for the decision:
/// a projection ought to say which vocabulary it implements.
const EFFORT_STORES: [&str; 2] = ["capacitive", "kinetic"];
const FLOW_STORES: [&str; 2] = ["inductive", "elastic"];
const JUNCTIONS: [&str; 2] = ["terminal", "body"];
const BRANCHES: [&str; 2] = ["wire", "link"];

/// Read a compiled IR into a port-Hamiltonian system.
pub fn project(ir: &Ir, it: &Interner) -> Result<System, ReadError> {
    Reader { ir, it }.project()
}

pub(crate) struct Reader<'a> {
    pub(crate) ir: &'a Ir,
    pub(crate) it: &'a Interner,
}

impl Reader<'_> {
    fn name(&self, decl: DeclId) -> &str {
        self.ir
            .decl_nodes
            .get(decl.raw())
            .map(|n| self.it.resolve(n.name))
            .unwrap_or("?")
    }

    pub(crate) fn qualified(&self, decl: DeclId) -> String {
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

    pub(crate) fn inherits(&self, decl: DeclId, name: &str) -> bool {
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

    pub(crate) fn field(&self, decl: DeclId, field: &str) -> Option<&ValueR> {
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

    /// A three-component list field, for a placement.
    fn vector3(&self, decl: DeclId, field: &str) -> Option<[f64; 3]> {
        match self.field(decl, field) {
            Some(ValueR::List(items)) if items.len() >= 3 => {
                let mut out = [0.0; 3];
                for (i, item) in items.iter().take(3).enumerate() {
                    match item {
                        ValueR::Num(n) => out[i] = *n,
                        _ => return None,
                    }
                }
                Some(out)
            }
            _ => None,
        }
    }

    pub(crate) fn reference(&self, decl: DeclId, field: &str) -> Option<DeclId> {
        match self.field(decl, field) {
            Some(ValueR::Ref(d)) => Some(*d),
            _ => None,
        }
    }

    /// The unit a quantity or coordinate declares, as a label. Not checked — the language has no
    /// type system, and this projection carries the label for a human rather than enforcing it.
    fn unit(&self, decl: DeclId) -> String {
        self.reference(decl, "unit")
            .map(|u| self.name(u).to_string())
            .unwrap_or_default()
    }

    /// The numeric `value` of a declared parameter, followed through the reference that names it.
    fn value_of(&self, decl: DeclId) -> Option<f64> {
        match self.field(decl, "value") {
            Some(ValueR::Num(n)) => Some(*n),
            _ => None,
        }
    }

    fn incidences(&self, decl: DeclId) -> Vec<(&SignedRefR, DeclId)> {
        self.ir
            .arcs
            .iter()
            .filter(|arc| arc.in_edge == decl)
            .flat_map(|arc| arc.refs.iter())
            .map(|r| (r, r.target()))
            .collect()
    }

    fn project(&self) -> Result<System, ReadError> {
        // ── coordinates ─────────────────────────────────────────────────────────
        // Declaration order IS the state layout. See the module note.
        let mut coordinates = Vec::new();
        let mut capacity_from = Vec::new();
        let mut index_of = std::collections::HashMap::new();
        for raw in 0..self.ir.decl_nodes.len() {
            let decl = DeclId::new(raw);
            if self.ir.decl_nodes[raw].kind != DeclKind::Node {
                continue;
            }
            let Some(kind) = EFFORT_STORES
                .iter()
                .chain(FLOW_STORES.iter())
                .find(|k| self.inherits(decl, k))
            else {
                continue;
            };
            let is_flow = FLOW_STORES.contains(kind);
            let name = self.qualified(decl);
            let Some(capacity_decl) = self.reference(decl, "capacity") else {
                return Err(ReadError::Capacity {
                    coordinate: name,
                    value: None,
                });
            };
            let capacity = self.value_of(capacity_decl);
            let Some(capacity) = capacity.filter(|c| *c > 0.0) else {
                return Err(ReadError::Capacity {
                    coordinate: name,
                    value: capacity,
                });
            };
            capacity_from.push(capacity_decl);
            index_of.insert(decl, coordinates.len());
            coordinates.push(Coordinate {
                decl,
                name,
                unit: self.unit(decl),
                capacity,
                flow_store: is_flow,
                kind: (*kind).to_string(),
            });
        }
        if coordinates.is_empty() {
            return Err(ReadError::Empty);
        }
        let n = coordinates.len();
        let hierarchy = crate::hierarchy::Hierarchy::project(self, &coordinates)?;

        // ── J, R, G and the readouts ────────────────────────────────────────────
        let mut ports = Vec::new();
        let mut port_sources = Vec::new();
        let mut dissipation = vec![0.0; n];
        let mut losses = Vec::new();
        let mut drives = Vec::new();
        let mut drive_amplitude_from = Vec::new();
        let mut drive_frequency_from = Vec::new();
        let mut observables = Vec::new();
        let mut pending_readouts: Vec<(String, DeclId, Pending)> = Vec::new();
        let mut sweeps = Vec::new();

        for raw in 0..self.ir.decl_nodes.len() {
            let decl = DeclId::new(raw);
            if self.ir.decl_nodes[raw].kind != DeclKind::Edge {
                continue;
            }
            let at = self.qualified(decl);
            let inc = self.incidences(decl);
            let plus: Vec<DeclId> = signed(&inc, |r| r.sign() == 1);
            let minus: Vec<DeclId> = signed(&inc, |r| r.sign() == -1);
            let neutral: Vec<DeclId> = signed(&inc, |r| r.sign() == 0);

            let coordinate = |set: &[DeclId], i: usize| -> Result<usize, ReadError> {
                let target = *set.get(i).ok_or_else(|| ReadError::Arity {
                    at: at.clone(),
                    expected: "one more coordinate",
                    found: format!("{} of that sign", set.len()),
                })?;
                hierarchy.resolve(self, decl, target, &index_of)
            };
            // Returns the declaration as well as the number, so the blueprint can record which
            // parameter supplied which structural element and a sweep can move all of them at once.
            let weight = |set: &[DeclId], i: usize| -> Result<(DeclId, f64), ReadError> {
                let target = *set.get(i).ok_or_else(|| ReadError::Arity {
                    at: at.clone(),
                    expected: "a weight",
                    found: "none".into(),
                })?;
                let value = self.value_of(target).ok_or_else(|| ReadError::Unresolved {
                    at: at.clone(),
                    what: self.qualified(target),
                })?;
                Ok((target, value))
            };

            if self.inherits(decl, "port") {
                // `(+ target, - source, ~ weight)` -> `J[source][target] = +w`.
                let target = coordinate(&plus, 0)?;
                let source = coordinate(&minus, 0)?;
                let (weight_from, w) = weight(&neutral, 0)?;
                ports.push(Port::edge(source, target, w));
                port_sources.push(PortSource {
                    decl,
                    name: at,
                    source,
                    target,
                    weight_from,
                });
            } else if self.inherits(decl, "dissipative") {
                let k = coordinate(&plus, 0)?;
                let (value_from, value) = weight(&neutral, 0)?;
                if value < 0.0 {
                    return Err(ReadError::NegativeDissipation { at, value });
                }
                dissipation[k] += value;
                losses.push(LossSource {
                    decl,
                    name: at,
                    coordinate: k,
                    value_from,
                });
            } else if self.inherits(decl, "drive") {
                let k = coordinate(&plus, 0)?;
                let waveform = *neutral.first().ok_or_else(|| ReadError::Arity {
                    at: at.clone(),
                    expected: "(+ coordinate, ~ waveform)",
                    found: "no waveform".into(),
                })?;
                let amplitude_from =
                    self.reference(waveform, "amplitude")
                        .ok_or(ReadError::Waveform {
                            at: at.clone(),
                            missing: "amplitude",
                        })?;
                let frequency_from =
                    self.reference(waveform, "frequency")
                        .ok_or(ReadError::Waveform {
                            at: at.clone(),
                            missing: "frequency",
                        })?;
                let amplitude = self.value_of(amplitude_from).ok_or(ReadError::Waveform {
                    at: at.clone(),
                    missing: "amplitude",
                })?;
                let frequency_hz = self.value_of(frequency_from).ok_or(ReadError::Waveform {
                    at: at.clone(),
                    missing: "frequency",
                })?;
                drive_amplitude_from.push(amplitude_from);
                drive_frequency_from.push(frequency_from);
                drives.push(Drive {
                    decl,
                    name: at,
                    coordinate: k,
                    amplitude,
                    frequency_hz,
                });
            } else {
                let target = |set: &[DeclId], i: usize| -> Result<DeclId, ReadError> {
                    set.get(i).copied().ok_or_else(|| ReadError::Arity {
                        at: at.clone(),
                        expected: "(+ quantity, - terminal, ...)",
                        found: format!("{} on the minus side", set.len()),
                    })
                };
                let pending = if self.inherits(decl, "effort") {
                    Pending::Ready(Readout::Effort {
                        coordinate: coordinate(&minus, 0)?,
                    })
                } else if self.inherits(decl, "stored_energy") {
                    Pending::Ready(Readout::StoredEnergy {
                        coordinate: coordinate(&minus, 0)?,
                    })
                } else if self.inherits(decl, "store") {
                    Pending::Ready(Readout::Store {
                        coordinate: coordinate(&minus, 0)?,
                    })
                } else if self.inherits(decl, "product") {
                    Pending::Ready(Readout::Product {
                        a: coordinate(&minus, 0)?,
                        b: coordinate(&minus, 1)?,
                    })
                } else if self.inherits(decl, "across") {
                    Pending::Across {
                        plus: target(&minus, 0)?,
                        minus: target(&minus, 1)?,
                    }
                } else if self.inherits(decl, "potential") {
                    Pending::Potential {
                        terminal: target(&minus, 0)?,
                    }
                } else {
                    continue;
                };
                let quantity = *plus.first().ok_or_else(|| ReadError::Arity {
                    at: at.clone(),
                    expected: "(+ quantity, - coordinate, ...)",
                    found: "no quantity".into(),
                })?;
                pending_readouts.push((at, quantity, pending));
            }
        }

        // -- the declared mode geometry -----------------------------------------
        // A shape is read only where an `@occupies` edge attaches it to a coordinate: a shape nobody
        // occupies is a shape nothing has the energy of, and carrying it would put a field in the
        // projection that no coordinate drives.
        let mut regions = Vec::new();
        // A shape may decline to place itself, the same way a terminal may. Both are settled below,
        // once the circuit that decides where they belong has been read.
        let mut declared_centre: Vec<Option<[f64; 3]>> = Vec::new();
        let mut declared_axis: Vec<Option<[f64; 3]>> = Vec::new();
        for raw in 0..self.ir.decl_nodes.len() {
            let decl = DeclId::new(raw);
            if self.ir.decl_nodes[raw].kind != DeclKind::Edge || !self.inherits(decl, "occupies") {
                continue;
            }
            let at = self.qualified(decl);
            let inc = self.incidences(decl);
            let plus: Vec<DeclId> = signed(&inc, |r| r.sign() == 1);
            let neutral: Vec<DeclId> = signed(&inc, |r| r.sign() == 0);

            let target = *plus.first().ok_or_else(|| ReadError::Arity {
                at: at.clone(),
                expected: "(+ coordinate, ~ shape)",
                found: "no coordinate".into(),
            })?;
            let coordinate = hierarchy.resolve(self, decl, target, &index_of)?;
            let shape_decl = *neutral.first().ok_or_else(|| ReadError::Arity {
                at: at.clone(),
                expected: "(+ coordinate, ~ shape)",
                found: "no shape".into(),
            })?;
            let name = self.qualified(shape_decl);

            let positive = |field: &'static str| -> Result<f64, ReadError> {
                let value = match self.field(shape_decl, field) {
                    Some(ValueR::Num(n)) => Some(*n),
                    _ => None,
                };
                value.filter(|v| *v > 0.0).ok_or(ReadError::Shape {
                    at: name.clone(),
                    field,
                    value,
                })
            };
            let shape = if self.inherits(shape_decl, "parallel_plate") {
                Shape::ParallelPlate {
                    area: positive("area")?,
                    separation: positive("separation")?,
                    relative_permittivity: positive("relative_permittivity")?,
                }
            } else if self.inherits(shape_decl, "solenoid") {
                Shape::Solenoid {
                    turns: positive("turns")?,
                    area: positive("area")?,
                    length: positive("length")?,
                }
            } else {
                return Err(ReadError::Unresolved {
                    at,
                    what: format!("{name} is not a shape this projection knows"),
                });
            };

            declared_centre.push(self.vector3(shape_decl, "centre"));
            declared_axis.push(self.vector3(shape_decl, "axis"));
            regions.push(Region {
                decl: shape_decl,
                name,
                coordinate,
                shape,
                centre: self.vector3(shape_decl, "centre").unwrap_or([0.0; 3]),
                axis: self.vector3(shape_decl, "axis").unwrap_or([0.0, 1.0, 0.0]),
            });
        }

        // -- the circuit in space, where the model draws one ---------------------
        let mut terminals = Vec::new();
        let mut terminal_of = std::collections::HashMap::new();
        // A terminal that declares no `at` is not a terminal at the origin. The distinction is kept
        // here and settled below, once the conductors that decide where it belongs have been read.
        let mut declared_at: Vec<Option<[f64; 3]>> = Vec::new();
        for raw in 0..self.ir.decl_nodes.len() {
            let decl = DeclId::new(raw);
            if self.ir.decl_nodes[raw].kind != DeclKind::Node
                || !JUNCTIONS.iter().any(|k| self.inherits(decl, k))
            {
                continue;
            }
            let at = self.vector3(decl, "at");
            terminal_of.insert(decl, terminals.len());
            terminals.push(Terminal {
                decl,
                name: self.qualified(decl),
                at: at.unwrap_or([0.0; 3]),
                pinned: at.is_some(),
            });
            declared_at.push(at);
        }

        let mut wires = Vec::new();
        let mut reference: Option<usize> = None;
        let mut emitters = Vec::new();
        for raw in 0..self.ir.decl_nodes.len() {
            let decl = DeclId::new(raw);
            if self.ir.decl_nodes[raw].kind != DeclKind::Edge {
                continue;
            }
            let at = self.qualified(decl);
            let inc = self.incidences(decl);
            let plus: Vec<DeclId> = signed(&inc, |r| r.sign() == 1);
            let neutral: Vec<DeclId> = signed(&inc, |r| r.sign() == 0);
            let minus: Vec<DeclId> = signed(&inc, |r| r.sign() == -1);

            let is_supply = self.inherits(decl, "supply");
            if BRANCHES.iter().any(|k| self.inherits(decl, k)) || is_supply {
                let end = |set: &[DeclId], which: &'static str| -> Result<usize, ReadError> {
                    let target = *set.first().ok_or_else(|| ReadError::Arity {
                        at: at.clone(),
                        expected: "(+ to, - from, ~ coordinate)",
                        found: which.into(),
                    })?;
                    terminal_of
                        .get(&target)
                        .copied()
                        .ok_or_else(|| ReadError::Unresolved {
                            at: at.clone(),
                            what: self.qualified(target),
                        })
                };
                let to = end(&plus, "no destination")?;
                let from = end(&minus, "no origin")?;
                let carried = *neutral.first().ok_or_else(|| ReadError::Arity {
                    at: at.clone(),
                    expected: "(+ to, - from, ~ coordinate)",
                    found: "no coordinate".into(),
                })?;
                let coordinate = hierarchy.resolve(self, decl, carried, &index_of)?;
                wires.push(Wire {
                    decl,
                    name: at,
                    from,
                    to,
                    coordinate,
                    via: self.vector3(decl, "via"),
                    supply: is_supply,
                });
            } else if self.inherits(decl, "reference") {
                let target = *plus.first().ok_or_else(|| ReadError::Arity {
                    at: at.clone(),
                    expected: "(+ terminal)",
                    found: "nothing".into(),
                })?;
                let index = *terminal_of
                    .get(&target)
                    .ok_or_else(|| ReadError::Unresolved {
                        at: at.clone(),
                        what: self.qualified(target),
                    })?;
                if reference.is_some_and(|already| already != index) {
                    return Err(ReadError::Reference {
                        at,
                        why: "two different terminals are declared as the reference",
                    });
                }
                reference = Some(index);
            } else if self.inherits(decl, "emits") {
                let target = *plus.first().ok_or_else(|| ReadError::Arity {
                    at: at.clone(),
                    expected: "(+ coordinate)",
                    found: "nothing".into(),
                })?;
                emitters.push(hierarchy.resolve(self, decl, target, &index_of)?);
            }
        }

        // -- where the drawing goes, where the model did not say ----------------
        // The positions come from the incidence between terminals and conductors: the same structure
        // `J` is assembled from, read a second way. A model may pin some terminals and leave the
        // rest, and what it pins is honoured exactly.
        let edges: Vec<(usize, usize)> = wires.iter().map(|w| (w.from, w.to)).collect();
        for (terminal, at) in terminals.iter_mut().zip(crate::layout::arrange(
            declared_at.len(),
            &edges,
            &declared_at,
        )) {
            terminal.at = at;
        }
        let positions: Vec<[f64; 3]> = terminals.iter().map(|t| t.at).collect();
        let declared_via: Vec<Option<[f64; 3]>> = wires.iter().map(|w| w.via).collect();
        for (wire, via) in
            wires
                .iter_mut()
                .zip(crate::layout::bends(&positions, &edges, &declared_via))
        {
            wire.via = via;
        }

        // A region the model did not place goes where the circuit says its energy is: along the
        // conductor that carries its coordinate, or at the node a shunt store hangs from. Without
        // this an arranged circuit drags its parasitics along behind it, still sitting where a
        // drawing that no longer exists put them.
        let conductors: Vec<crate::layout::Conductor> = wires
            .iter()
            .map(|w| crate::layout::Conductor {
                from: w.from,
                to: w.to,
                coordinate: w.coordinate,
                supply: w.supply,
                middle: w.via.unwrap_or_else(|| {
                    let (a, b) = (positions[w.from], positions[w.to]);
                    [
                        0.5 * (a[0] + b[0]),
                        0.5 * (a[1] + b[1]),
                        0.5 * (a[2] + b[2]),
                    ]
                }),
            })
            .collect();
        for (r, region) in regions.iter_mut().enumerate() {
            if declared_centre[r].is_some() {
                continue;
            }
            let coupled: Vec<usize> = ports
                .iter()
                .filter(|p| {
                    p.sources.contains(&region.coordinate) || p.targets.contains(&region.coordinate)
                })
                .flat_map(|p| p.sources.iter().chain(p.targets.iter()).copied())
                .filter(|k| *k != region.coordinate)
                .collect();
            let Some((centre, axis)) = crate::layout::place_store(
                region.coordinate,
                &coupled,
                &conductors,
                &positions,
                reference,
            ) else {
                continue;
            };
            region.centre = centre;
            if let (None, Some(along)) = (declared_axis[r], axis) {
                region.axis = along;
            }
        }

        // -- the probes, now that there is a circuit to place them on -----------
        let component = crate::layout::component_of(terminals.len(), &edges);
        let resolve_terminal = |target: DeclId, at: &str| -> Result<usize, ReadError> {
            terminal_of
                .get(&target)
                .copied()
                .ok_or_else(|| ReadError::Unresolved {
                    at: at.to_string(),
                    what: self.qualified(target),
                })
        };
        for (at, quantity, pending) in pending_readouts {
            let readout = match pending {
                Pending::Ready(readout) => readout,
                Pending::Across { plus, minus } => Readout::Across {
                    plus: resolve_terminal(plus, &at)?,
                    minus: resolve_terminal(minus, &at)?,
                },
                Pending::Potential { terminal } => Readout::Across {
                    plus: resolve_terminal(terminal, &at)?,
                    minus: reference.ok_or(ReadError::Reference {
                        at: at.clone(),
                        why: "a potential is measured against something, and no terminal is declared as the reference",
                    })?,
                },
            };
            if let Readout::Across { plus, minus } = readout {
                if component[plus] != component[minus] {
                    return Err(ReadError::Disconnected {
                        at,
                        plus: terminals[plus].name.clone(),
                        minus: terminals[minus].name.clone(),
                    });
                }
            }
            observables.push(Observable {
                decl: quantity,
                name: self.qualified(quantity),
                unit: self.unit(quantity),
                readout,
            });
        }

        // -- the declared studies, and every number a sweep could move ----------
        for raw in 0..self.ir.decl_nodes.len() {
            let decl = DeclId::new(raw);
            if self.ir.decl_nodes[raw].kind != DeclKind::Node || !self.inherits(decl, "sweep") {
                continue;
            }
            let at = self.qualified(decl);
            let parameter = self.reference(decl, "over").ok_or(ReadError::Sweep {
                at: at.clone(),
                missing: "over",
            })?;
            let number = |field: &'static str| -> Result<f64, ReadError> {
                match self.field(decl, field) {
                    Some(ValueR::Num(n)) => Ok(*n),
                    _ => Err(ReadError::Sweep {
                        at: at.clone(),
                        missing: field,
                    }),
                }
            };
            sweeps.push(Sweep {
                decl,
                parameter,
                parameter_name: self.qualified(parameter),
                from: number("from")?,
                to: number("to")?,
                steps: number("steps")?.max(1.0) as usize,
                name: at,
            });
        }

        // Every declaration carrying a number, so `System::at` can re-evaluate at another one.
        let mut values = Vec::new();
        for raw in 0..self.ir.decl_nodes.len() {
            let decl = DeclId::new(raw);
            if self.ir.decl_nodes[raw].kind == DeclKind::Node {
                if let Some(value) = self.value_of(decl) {
                    values.push(Declared {
                        decl,
                        name: self.qualified(decl),
                        value,
                    });
                }
            }
        }

        // `Q = diag(1/capacity)`: the coordinate's energy is `x^2/(2c)`, so `grad H = x/c`.
        let energy = coordinates.iter().map(|c| 1.0 / c.capacity).collect();
        let name = coordinates[0]
            .name
            .rsplit_once('.')
            .map(|(root, _)| root.to_string())
            .unwrap_or_else(|| coordinates[0].name.clone());

        Ok(System {
            hierarchy,
            name,
            coordinates,
            port_hamiltonian: PortHamiltonian::new(
                Interconnection::new(n, ports),
                dissipation,
                energy,
            ),
            drives,
            observables,
            blueprint: Blueprint {
                capacity_from,
                ports: port_sources,
                losses,
                drive_amplitude_from,
                drive_frequency_from,
                values,
            },
            sweeps,
            regions,
            terminals,
            wires,
            reference,
            emitters,
        })
    }
}

fn signed(inc: &[(&SignedRefR, DeclId)], want: impl Fn(&SignedRefR) -> bool) -> Vec<DeclId> {
    inc.iter()
        .filter(|(r, _)| want(r))
        .map(|(_, d)| *d)
        .collect()
}

// Shared strict scalar/vector reads for continuum projections.
pub(crate) fn number(
    ir: &Ir,
    it: &Interner,
    decl: DeclId,
    name: &str,
    default: Option<f64>,
) -> Result<f64, String> {
    match hymeko_graph::octree::field(ir, it, decl, name) {
        Some(ValueR::Num(v)) if v.is_finite() => Ok(*v),
        None => default.ok_or_else(|| format!("missing {name}")),
        _ => Err(format!("{name} must be finite numeric")),
    }
}
pub(crate) fn vector(
    ir: &Ir,
    it: &Interner,
    decl: DeclId,
    name: &str,
    default: Option<[f64; 3]>,
) -> Result<[f64; 3], String> {
    match hymeko_graph::octree::field(ir, it, decl, name) {
        Some(ValueR::List(v)) if v.len() == 3 => {
            let mut out = [0.0; 3];
            for (i, x) in v.iter().enumerate() {
                match x {
                    ValueR::Num(n) if n.is_finite() => out[i] = *n,
                    _ => return Err(format!("{name} must contain finite numbers")),
                }
            }
            Ok(out)
        }
        None => default.ok_or_else(|| format!("missing {name}")),
        _ => Err(format!("{name} must be a three-vector")),
    }
}
