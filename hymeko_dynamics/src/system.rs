//! The projected dynamical system: what a declaration becomes.
//!
//! ```text
//! x' = (J - R) grad H(x) + G u(t),    H(x) = sum_k x_k^2 / (2 c_k)
//! ```
//!
//! `J` is a [`hymeko_numerics::Interconnection`] — a signed hypergraph, not a stored matrix — so
//! the skew-symmetry that makes the energy balance hold is a property of the declaration's sign
//! convention rather than something checked after assembly.
//!
//! Nothing here knows it is a circuit. A coordinate has a capacity, a port has a weight, a drive has
//! an amplitude and a frequency. That the third coordinate is an electric displacement flux is
//! recorded in its NAME and its UNIT, which travel from the declaration for a human's benefit and
//! are never read as instructions.

use hymeko::common::ids::DeclId;
use hymeko_numerics::{Interconnection, Port, PortHamiltonian};

use crate::field::Region;

/// One energy store.
#[derive(Debug, Clone, PartialEq)]
pub struct Coordinate {
    /// The declaration — the identity that survives from the source file into a result.
    pub decl: DeclId,
    pub name: String,
    pub unit: String,
    /// The denominator of the quadratic store: `H_k = x^2 / (2 capacity)`, `effort = x / capacity`.
    pub capacity: f64,
    /// Which of the two dual store kinds this is: `false` for an EFFORT store, `true` for a FLOW
    /// store. A capacitance and an inductance in the circuit reading; a spring and a mass in the
    /// mechanical one.
    ///
    /// The dynamics does not need it. A quadratic store is a quadratic store, and everything above
    /// treats the two alike. What needs it is the CIRCUIT: the drop a probe reads across a conductor
    /// is `xdot + R e` on a flow store, where the rate is the potential difference and `R` sits in
    /// series with it, and `e` alone on an effort store, where the effort IS the potential
    /// difference and the same `R` is a leakage across it rather than a drop along it. Taking that
    /// from the declaration is the difference between measuring and guessing.
    pub flow_store: bool,
    /// What the model called this kind of store: `capacitive`, `inductive`, `kinetic`, `elastic`.
    ///
    /// Provenance, and the only thing that says which DOMAIN a system is in. The projection does not
    /// care — a quadratic store is a quadratic store — but anything that puts a label on a number
    /// does: the effort of a flow store is a current on one and a force on the other, and calling a
    /// force a current is not a presentation detail.
    pub kind: String,
}

/// What an observation reads off the trajectory.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Readout {
    /// `x_k / c_k` — a voltage on a charge coordinate, a current on a flux one.
    Effort { coordinate: usize },
    /// `x_k` itself.
    Store { coordinate: usize },
    /// `x_k^2 / (2 c_k)`.
    StoredEnergy { coordinate: usize },
    /// `e_a e_b` — two efforts multiplied, which is what a Poynting flux is.
    Product { a: usize, b: usize },
    /// `V(plus) - V(minus)` — what a probe with two leads reads, between two TERMINALS.
    ///
    /// The only readout that is not a function of one coordinate. A potential is not a state
    /// variable of a port-Hamiltonian system: it lives on the nodes of the circuit, and the state
    /// lives on the branches. So this one is evaluated by walking the declared conductors from one
    /// terminal to the other and adding up what each drops, which is the definition of a potential
    /// difference and also, when two different walks are compared, Kirchhoff's voltage law.
    Across { plus: usize, minus: usize },
}

impl Readout {
    /// Is this readout a linear functional of the state?
    ///
    /// A frequency response is only defined for one that is. `Product` and `StoredEnergy` are
    /// quadratic, and asking for their transfer function is a category error rather than a hard
    /// case — so [`crate::analysis::response`] refuses instead of linearising silently.
    pub fn is_linear(&self) -> bool {
        matches!(self, Self::Effort { .. } | Self::Store { .. })
    }

    /// Does evaluating this readout need the rate of the state, and therefore the time?
    ///
    /// [`Readout::Across`] does: a flow store's contribution to a potential difference is `xdot`,
    /// and `xdot` depends on the drive, which depends on when you look.
    pub fn needs_rate(&self) -> bool {
        matches!(self, Self::Across { .. })
    }
}

/// A named observation of the trajectory.
#[derive(Debug, Clone, PartialEq)]
pub struct Observable {
    pub decl: DeclId,
    pub name: String,
    pub unit: String,
    pub readout: Readout,
}

/// An input entering one coordinate's rate equation.
#[derive(Debug, Clone, PartialEq)]
pub struct Drive {
    pub decl: DeclId,
    pub name: String,
    pub coordinate: usize,
    pub amplitude: f64,
    /// In hertz, as declared. The angular frequency is derived where it is used, once.
    pub frequency_hz: f64,
}

impl Drive {
    pub fn at(&self, time: f64) -> f64 {
        self.amplitude * (std::f64::consts::TAU * self.frequency_hz * time).cos()
    }
}

/// One port, and the declaration that supplied its weight.
#[derive(Debug, Clone, PartialEq)]
pub struct PortSource {
    pub decl: DeclId,
    pub name: String,
    pub source: usize,
    pub target: usize,
    pub weight_from: DeclId,
}

/// One dissipation term, and the declaration that supplied its value.
#[derive(Debug, Clone, PartialEq)]
pub struct LossSource {
    pub decl: DeclId,
    pub name: String,
    pub coordinate: usize,
    pub value_from: DeclId,
}

/// Which declaration supplied every number the structure was assembled from.
///
/// Kept so the projection can be re-evaluated at another value of a declared parameter WITHOUT
/// re-reading the file, and without anybody rewriting the model as text. A sweep is then a question
/// asked of one projection rather than forty projections of forty edited files.
#[derive(Debug, Clone, PartialEq)]
pub struct Blueprint {
    /// Per coordinate, the parameter its `capacity` referenced.
    pub capacity_from: Vec<DeclId>,
    pub ports: Vec<PortSource>,
    pub losses: Vec<LossSource>,
    /// Per drive, the parameters its waveform referenced.
    pub drive_amplitude_from: Vec<DeclId>,
    pub drive_frequency_from: Vec<DeclId>,
    /// Every declared parameter that carries a number.
    pub values: Vec<Declared>,
}

/// A declared number, and what the model calls it.
///
/// The name travels because a consumer that wants to CHANGE one has to say which: an inspector
/// offering "the resistance on this branch" has to name the parameter the model would be edited at,
/// and the edge that references it is not that parameter.
#[derive(Debug, Clone, PartialEq)]
pub struct Declared {
    pub decl: DeclId,
    pub name: String,
    pub value: f64,
}

impl Blueprint {
    pub fn value_of(&self, decl: DeclId) -> Option<f64> {
        self.values.iter().find(|d| d.decl == decl).map(|d| d.value)
    }

    /// What the model calls the declaration that supplied a number.
    pub fn name_of(&self, decl: DeclId) -> Option<&str> {
        self.values
            .iter()
            .find(|d| d.decl == decl)
            .map(|d| d.name.as_str())
    }
}

/// A declared investigation: one parameter, over a range.
#[derive(Debug, Clone, PartialEq)]
pub struct Sweep {
    pub decl: DeclId,
    pub name: String,
    /// The parameter being varied.
    pub parameter: DeclId,
    pub parameter_name: String,
    pub from: f64,
    pub to: f64,
    pub steps: usize,
}

impl Sweep {
    /// The values to visit, inclusive of both ends.
    pub fn points(&self) -> Vec<f64> {
        let steps = self.steps.max(1);
        if steps == 1 {
            return vec![self.from];
        }
        (0..steps)
            .map(|i| self.from + (self.to - self.from) * i as f64 / (steps - 1) as f64)
            .collect()
    }
}

/// A point the circuit occupies.
#[derive(Debug, Clone, PartialEq)]
pub struct Terminal {
    pub decl: DeclId,
    pub name: String,
    /// Where it sits in a drawing. Presentation: it does not enter the dynamics.
    ///
    /// Always a resolved position. A model that declares `at` gets exactly what it declared; a model
    /// that does not gets one derived by [`crate::layout::arrange`] from the same incidence the
    /// dynamics is assembled from, so a consumer never has to know which happened.
    pub at: [f64; 3],
    /// Whether the model placed this terminal itself, rather than the layout deriving it.
    ///
    /// Provenance, not presentation. It is what lets a view say which part of a drawing the model
    /// is responsible for, and what lets a re-arrangement leave the declared points alone.
    pub pinned: bool,
}

/// A conductor between two terminals, carrying one coordinate's effort as its current.
///
/// The current is not a separate declaration. A wire names the coordinate, and that coordinate's
/// effort IS the current through it — so a view shows the current without being told what a current
/// is, and a resistor is found on this wire because its dissipation is attached to the same
/// coordinate rather than because a name matched.
#[derive(Debug, Clone, PartialEq)]
pub struct Wire {
    pub decl: DeclId,
    pub name: String,
    pub from: usize,
    pub to: usize,
    pub coordinate: usize,
    /// A bend, so two branches between the same pair of terminals do not lie on top of each other.
    pub via: Option<[f64; 3]>,
    /// Whether this is the supply rather than a conductor.
    ///
    /// A source and the branch it drives are in series and therefore carry the same coordinate, so
    /// without this a consumer finds the same dissipation on both and draws the resistor twice.
    pub supply: bool,
}

/// A declared port-Hamiltonian system, ready to analyse or to run.
#[derive(Debug, Clone, PartialEq)]
pub struct System {
    /// Sealed source subsystems and their exported power ports.
    pub hierarchy: crate::hierarchy::Hierarchy,
    /// The declaration this came from, for a result file.
    pub name: String,
    pub coordinates: Vec<Coordinate>,
    /// `J`, `R` and `Q`, as the numerics crate wants them.
    pub port_hamiltonian: PortHamiltonian,
    pub drives: Vec<Drive>,
    pub observables: Vec<Observable>,
    /// Where every assembled number came from, so the system can be re-evaluated at another one.
    pub blueprint: Blueprint,
    /// The investigations the model declares.
    pub sweeps: Vec<Sweep>,
    /// The spatial mode each coordinate is the amplitude of, where the model declares one.
    ///
    /// Empty is legitimate: a system with no declared geometry is a perfectly good dynamical
    /// system that cannot be drawn in space, and saying so is better than drawing it anyway.
    pub regions: Vec<Region>,
    /// The circuit itself, where the model draws one: terminals and the conductors between them.
    ///
    /// Separate from `regions` because they answer different questions. A region says where a
    /// coordinate's ENERGY sits; on a resistor network every one of those is a parasitic, and the
    /// components the design is about store nothing and appear in none of them.
    pub terminals: Vec<Terminal>,
    pub wires: Vec<Wire>,
    /// The terminal every potential is measured against, where the model names one.
    ///
    /// Kirchhoff fixes node potentials only up to a constant, so "the voltage at A" is not a
    /// quantity until something says what it is a voltage above. Declaring the reference is how a
    /// model turns a difference into a number.
    pub reference: Option<usize>,
    /// Coordinates whose dissipation leaves as light rather than as heat alone.
    pub emitters: Vec<usize>,
}

impl System {
    pub fn dim(&self) -> usize {
        self.coordinates.len()
    }

    /// A coordinate by its qualified name, or by the last segment of one.
    pub fn coordinate_of(&self, name: &str) -> Option<usize> {
        self.coordinates
            .iter()
            .position(|c| c.name == name || c.name.ends_with(&format!(".{name}")))
    }

    /// An observable by its qualified name, or by the last segment of one.
    pub fn observable_of(&self, name: &str) -> Option<usize> {
        self.observables
            .iter()
            .position(|o| o.name == name || o.name.ends_with(&format!(".{name}")))
    }

    /// The input vector at a time, in state-rate units.
    pub fn input_at(&self, time: f64, out: &mut [f64]) {
        out.fill(0.0);
        for drive in &self.drives {
            out[drive.coordinate] += drive.at(time);
        }
    }

    /// `grad H(x) = Q x`.
    pub fn effort(&self, x: &[f64], out: &mut [f64]) {
        for (k, coordinate) in self.coordinates.iter().enumerate() {
            out[k] = x[k] / coordinate.capacity;
        }
    }

    /// Evaluate every observable at a state and a time.
    ///
    /// The time is not decoration. A probe reads a potential difference, a potential difference
    /// contains the rate of the flow stores along the path, and that rate contains the drive — so
    /// asking what a probe reads without saying when is not a question with an answer.
    pub fn observe_at(&self, x: &[f64], time: f64) -> Vec<f64> {
        let mut effort = vec![0.0; self.dim()];
        self.effort(x, &mut effort);
        // Walking the circuit costs a pass over the conductors, and most models have no probe on
        // them, so it happens only when something asks.
        let potentials = self
            .observables
            .iter()
            .any(|o| o.readout.needs_rate())
            .then(|| self.potentials(x, time));
        self.observables
            .iter()
            .map(|o| match o.readout {
                Readout::Effort { coordinate } => effort[coordinate],
                Readout::Store { coordinate } => x[coordinate],
                Readout::StoredEnergy { coordinate } => 0.5 * x[coordinate] * effort[coordinate],
                Readout::Product { a, b } => effort[a] * effort[b],
                // Unreachable through `read`, which refuses a probe whose two leads are not
                // connected. Not a number rather than zero if it ever is: a reading that cannot be
                // taken must not look like a reading of nothing.
                Readout::Across { plus, minus } => potentials
                    .as_ref()
                    .and_then(|p| Some(p[plus]? - p[minus]?))
                    .unwrap_or(f64::NAN),
            })
            .collect()
    }

    /// What each conductor drops, from its `from` terminal to its `to` terminal.
    ///
    /// A conductor carrying a flow store is an inductance in series with its dissipation and drops
    /// `xdot + R e` along the direction its effort flows. One carrying an effort store IS that
    /// store, and drops `e`; the dissipation on such a coordinate is a leakage across it and does
    /// not appear in the path. A supply is not a conductor at all — it RAISES the potential by the
    /// drive entering the coordinate it carries, which is why an ideal source needs no state.
    pub fn branch_drops(&self, x: &[f64], time: f64) -> Vec<f64> {
        let n = self.dim();
        let mut effort = vec![0.0; n];
        self.effort(x, &mut effort);
        let mut input = vec![0.0; n];
        self.input_at(time, &mut input);
        let mut rate = vec![0.0; n];
        self.port_hamiltonian.flow(x, &input, &mut rate);

        self.wires
            .iter()
            .map(|wire| {
                let k = wire.coordinate;
                if wire.supply {
                    -input[k]
                } else if self.coordinates[k].flow_store {
                    rate[k] + self.port_hamiltonian.dissipation[k] * effort[k]
                } else {
                    effort[k]
                }
            })
            .collect()
    }

    /// The potential of every terminal, relative to the declared reference.
    ///
    /// `None` where a terminal cannot be reached from the reference through the declared conductors.
    /// A potential difference across a gap is not a small number — it is not a number, and returning
    /// zero for it would be inventing a measurement.
    pub fn potentials(&self, x: &[f64], time: f64) -> Vec<Option<f64>> {
        self.walk(&self.branch_drops(x, time))
    }

    /// The STATE each branch carries, in the order the branches are declared.
    ///
    /// A branch's state is to its drop what a displacement is to a velocity — one integrates the
    /// other — so walking these instead gives the integral of what [`System::potentials`] gives. On
    /// a mechanism that is exactly the bodies' displacements from the extensions of the springs
    /// between them, which is the only way to know where a body IS: a port-Hamiltonian system of
    /// masses and springs has momenta and extensions in its state and no positions at all.
    pub fn branch_states(&self, x: &[f64]) -> Vec<f64> {
        self.wires.iter().map(|w| x[w.coordinate]).collect()
    }

    /// The walk of [`System::branch_states`] outward from the reference.
    ///
    /// `None` where a junction cannot be reached, for the same reason and with the same meaning as
    /// in [`System::potentials`].
    pub fn junction_states(&self, x: &[f64]) -> Vec<Option<f64>> {
        self.walk(&self.branch_states(x))
    }

    /// The worst loop residual of the declared circuit, in the unit its drops are in.
    ///
    /// Kirchhoff's voltage law says the drops around any loop sum to zero. Here that is not an
    /// assumption but a comparison between two independent declarations: the DRAWING — which
    /// terminals each conductor joins — and the DYNAMICS — the signed hypergraph `J` was assembled
    /// from. Nothing forces them to agree. A model whose layout does not match its interconnection
    /// returns a residual the size of its voltages, and one whose layout does match returns
    /// round-off, so a picture becomes something that can be wrong and therefore checked.
    ///
    /// `None` when the model draws no circuit.
    pub fn kirchhoff_residual(&self, x: &[f64], time: f64) -> Option<f64> {
        let drops = self.branch_drops(x, time);
        let potentials = self.walk(&drops);
        let mut worst: Option<f64> = None;
        for (w, wire) in self.wires.iter().enumerate() {
            let (Some(a), Some(b)) = (potentials[wire.from], potentials[wire.to]) else {
                continue;
            };
            let residual = (a - b - drops[w]).abs();
            worst = Some(worst.map_or(residual, |m: f64| m.max(residual)));
        }
        worst
    }

    /// Accumulate potentials outward from the reference along a spanning tree of the conductors.
    ///
    /// Which tree does not matter — that it does not matter is exactly what
    /// [`System::kirchhoff_residual`] measures.
    fn walk(&self, drops: &[f64]) -> Vec<Option<f64>> {
        let mut out = vec![None; self.terminals.len()];
        let Some(root) = self.reference.or(if self.terminals.is_empty() {
            None
        } else {
            Some(0)
        }) else {
            return out;
        };
        out[root] = Some(0.0);
        let mut queue = vec![root];
        while let Some(node) = queue.pop() {
            let here = out[node].expect("only settled terminals are queued");
            for (w, wire) in self.wires.iter().enumerate() {
                let (other, potential) = if wire.from == node {
                    (wire.to, here - drops[w])
                } else if wire.to == node {
                    (wire.from, here + drops[w])
                } else {
                    continue;
                };
                if out[other].is_none() {
                    out[other] = Some(potential);
                    queue.push(other);
                }
            }
        }
        out
    }

    /// `H(x)`.
    pub fn energy(&self, x: &[f64]) -> f64 {
        self.port_hamiltonian.hamiltonian(x)
    }

    /// The dense state matrix `A = (J - R) Q`, row-major.
    ///
    /// Built for ANALYSIS only. The time stepping never forms it — [`hymeko_numerics`] applies the
    /// hypergraph directly, at the cost of one pass over the incidences rather than `n^2`. A dense
    /// matrix is the right shape for an eigenvalue problem and the wrong shape for a trajectory, and
    /// the two are kept apart on purpose.
    pub fn state_matrix(&self) -> Vec<f64> {
        let n = self.dim();
        let j = self.port_hamiltonian.interconnection.dense();
        let mut a = vec![0.0; n * n];
        for row in 0..n {
            for column in 0..n {
                let structure = j[row * n + column]
                    - if row == column {
                        self.port_hamiltonian.dissipation[row]
                    } else {
                        0.0
                    };
                a[row * n + column] = structure / self.coordinates[column].capacity;
            }
        }
        a
    }

    /// `S = Q^(1/2) (J - R) Q^(1/2)`, row-major — SIMILAR to [`Self::state_matrix`], so identical
    /// eigenvalues, and far better conditioned.
    ///
    /// `A = (J - R) Q` and `S = T (J - R) T` with `T = Q^(1/2)` satisfy `S = T A T^(-1)`, which is a
    /// similarity and therefore spectrum-preserving. The difference is arithmetic, and on this model
    /// it is the difference between an answer and noise: `A`'s entries span `1/C ~ 2e7` down to
    /// `1/L_field ~ 2e2`, so its characteristic polynomial has a constant term near `1e19` while its
    /// roots are near `5e4`. `S`'s entries are all natural frequencies — `1/sqrt(LC)`, `k/sqrt(L Cf)`,
    /// `g/sqrt(Lf Cf)`, `R/L`, `sigma/Cf` — every one of them the same order as the eigenvalues.
    ///
    /// It is also the coordinate system `hymeko_numerics` already steps in, for the same reason.
    pub fn normalised_matrix(&self) -> Vec<f64> {
        let n = self.dim();
        let j = self.port_hamiltonian.interconnection.dense();
        let root: Vec<f64> = self
            .coordinates
            .iter()
            .map(|c| (1.0 / c.capacity).sqrt())
            .collect();
        let mut s = vec![0.0; n * n];
        for row in 0..n {
            for column in 0..n {
                let structure = j[row * n + column]
                    - if row == column {
                        self.port_hamiltonian.dissipation[row]
                    } else {
                        0.0
                    };
                s[row * n + column] = root[row] * structure * root[column];
            }
        }
        s
    }

    /// The same system with one declared parameter set to another value.
    ///
    /// Everything that parameter supplied moves together: if `k_coupling` weights two ports it
    /// re-weights both, and if it is also a capacity it changes that too. That is the point of
    /// keeping the blueprint — a sweep must vary the DECLARATION, not one place the declaration
    /// happened to reach.
    ///
    /// # Panics
    /// If the new value makes the system impossible — a non-positive store, or a negative
    /// dissipation. That is a refusal rather than a silently clamped answer.
    pub fn at(&self, parameter: DeclId, value: f64) -> Self {
        let look = |decl: DeclId| -> f64 {
            if decl == parameter {
                value
            } else {
                self.blueprint.value_of(decl).unwrap_or(0.0)
            }
        };

        let coordinates: Vec<Coordinate> = self
            .coordinates
            .iter()
            .enumerate()
            .map(|(k, c)| Coordinate {
                capacity: look(self.blueprint.capacity_from[k]),
                ..c.clone()
            })
            .collect();
        let n = coordinates.len();
        let ports: Vec<Port> = self
            .blueprint
            .ports
            .iter()
            .map(|p| Port::edge(p.source, p.target, look(p.weight_from)))
            .collect();
        let mut dissipation = vec![0.0; n];
        for loss in &self.blueprint.losses {
            dissipation[loss.coordinate] += look(loss.value_from);
        }
        let energy: Vec<f64> = coordinates.iter().map(|c| 1.0 / c.capacity).collect();
        let drives: Vec<Drive> = self
            .drives
            .iter()
            .enumerate()
            .map(|(i, d)| Drive {
                amplitude: look(self.blueprint.drive_amplitude_from[i]),
                frequency_hz: look(self.blueprint.drive_frequency_from[i]),
                ..d.clone()
            })
            .collect();

        Self {
            hierarchy: self.hierarchy.clone(),
            name: self.name.clone(),
            coordinates,
            port_hamiltonian: PortHamiltonian::new(
                Interconnection::new(n, ports),
                dissipation,
                energy,
            ),
            drives,
            observables: self.observables.clone(),
            blueprint: self.blueprint.clone(),
            sweeps: self.sweeps.clone(),
            regions: self.regions.clone(),
            terminals: self.terminals.clone(),
            wires: self.wires.clone(),
            reference: self.reference,
            emitters: self.emitters.clone(),
        }
    }

    /// The region holding one coordinate's energy, if the model declares one.
    pub fn region_of(&self, coordinate: usize) -> Option<&Region> {
        self.regions.iter().find(|r| r.coordinate == coordinate)
    }

    /// The efforts, and the field each declared region carries at them.
    ///
    /// The single place a renderer asks what the field is, so a native one and a browser one
    /// cannot drift apart in their answer.
    pub fn fields(&self, x: &[f64]) -> Vec<(usize, f64)> {
        let mut effort = vec![0.0; self.dim()];
        self.effort(x, &mut effort);
        self.regions
            .iter()
            .enumerate()
            .map(|(i, r)| (i, r.shape.field(effort[r.coordinate])))
            .collect()
    }

    /// A declared parameter by name.
    pub fn parameter_of(&self, name: &str) -> Option<DeclId> {
        self.sweeps
            .iter()
            .find(|s| s.parameter_name == name || s.parameter_name.ends_with(&format!(".{name}")))
            .map(|s| s.parameter)
    }

    /// The input matrix column for a drive: which coordinate it enters, at unit amplitude.
    pub fn drive_direction(&self, drive: usize) -> Vec<f64> {
        let mut b = vec![0.0; self.dim()];
        b[self.drives[drive].coordinate] = 1.0;
        b
    }
}
