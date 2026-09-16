//! Kirchhoff's laws, for a port-Hamiltonian system that need not be a circuit.
//!
//! The laws are usually stated about a network of wires, and the statement here is the same one with
//! the wires taken out of it.
//!
//! # What they are
//!
//! On a graph, Kirchhoff's two laws say that the flows lie in the CYCLE space and the efforts lie in
//! the CUT space, and those two are orthogonal complements — which is Tellegen's theorem, and which
//! is the same sentence as "the interconnection moves power and does not make it". A port-Hamiltonian
//! system says that directly: `J` is skew, so
//!
//! ```text
//! e^T J e = 0
//! ```
//!
//! identically, for every effort vector, whatever the system is. **That is Kirchhoff's laws, in the
//! form that survives when there is no circuit.** A mechanism, a coupled field, a chain of masses —
//! each has an interconnection, each moves power between its stores, and none of them has a wire.
//!
//! # Why measuring it is not measuring nothing
//!
//! `J` is never formed. [`hymeko_numerics::Interconnection`] is a signed hypergraph and `apply`
//! walks its incidences, so whether the thing that walk computes is skew is a property of the
//! ASSEMBLER and not a theorem about mathematics. `e^T (J e)` is the assembler asked whether it kept
//! its own convention, and it can answer no.
//!
//! # The two laws separately, where a model draws a graph
//!
//! `e^T J e = 0` is the two laws multiplied together, and a model that also declares a layout can be
//! asked each of them on its own:
//!
//! * **The node law.** What the branches bring a junction has to equal what the store at that
//!   junction absorbs — its rate, plus what leaks out of it, less what drives it. The left side comes
//!   from the DRAWING (which branches touch this junction) and the right side from `J`. Nothing makes
//!   them agree.
//! * **The loop law.** What the branches drop around any loop has to sum to zero — the same two
//!   declarations compared the other way round, which is [`crate::System::kirchhoff_residual`].
//!
//! A model with no layout has neither, and saying so is the honest answer rather than reporting a
//! zero: there are no junctions to balance at and no loops to close.

use crate::system::System;

/// What the laws come to, at one state and one instant.
#[derive(Debug, Clone, PartialEq)]
pub struct Laws {
    /// `|e^T J e|` over the largest power any single port moves.
    ///
    /// Dimensionless, and zero to round-off for an interconnection that is what it claims to be.
    /// Normalised by the largest single-port power rather than by the total, because the total is a
    /// sum that is supposed to cancel and dividing by it would be dividing by the answer.
    pub tellegen: f64,
    /// The worst node-law residual over the drawn junctions, relative to the largest branch effort.
    ///
    /// `None` when the model draws no layout.
    pub node: Option<f64>,
    /// The worst loop-law residual over the drawn branches, relative to the largest drop.
    ///
    /// `None` for the same reason.
    pub loops: Option<f64>,
    /// Directions the system cannot move along at all: `dim ker((J - R) Q)`.
    ///
    /// A conserved quantity, or a coordinate the model declared twice over without meaning to. The
    /// interconnection graph predicts how many there are, which is where matroids come in and where
    /// this stops being arithmetic.
    pub conserved: usize,
    /// Vertices, edges and independent cycles of the INTERCONNECTION's own graph — the one whose
    /// vertices are coordinates and whose edges are ports.
    pub graph: Graph,
}

/// The shape of an interconnection, with the numbers forgotten.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Graph {
    pub vertices: usize,
    pub edges: usize,
    pub components: usize,
    /// `|E| - |V| + c`: how many independent loops the interconnection has.
    ///
    /// Zero for every model in this repository so far, and that is a fact about them rather than
    /// about the projection: each port is a distinct pathway between two stores, and physical
    /// systems built that way come out as trees. A model whose ports close a loop — three bodies
    /// joined pairwise by springs — has one, and it is exactly the coordinate it declared twice.
    pub cycles: usize,
}

impl System {
    /// The interconnection as a graph: coordinates for vertices, ports for edges.
    ///
    /// A port with more than one source or target is a hyperedge and is skipped: it is not an edge
    /// and pretending it is one would be counting a structure this does not describe.
    pub fn interconnection_graph(&self) -> Graph {
        let edges: Vec<(usize, usize)> = self
            .port_hamiltonian
            .interconnection
            .ports()
            .iter()
            .filter(|p| p.sources.len() == 1 && p.targets.len() == 1)
            .map(|p| (p.sources[0], p.targets[0]))
            .collect();

        let n = self.dim();
        let mut parent: Vec<usize> = (0..n).collect();
        fn root(parent: &mut [usize], mut i: usize) -> usize {
            while parent[i] != i {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            i
        }
        let mut rank = 0;
        for &(a, b) in &edges {
            let (ra, rb) = (root(&mut parent, a), root(&mut parent, b));
            if ra != rb {
                parent[ra] = rb;
                rank += 1;
            }
        }
        Graph {
            vertices: n,
            edges: edges.len(),
            components: n - rank,
            cycles: edges.len() - rank,
        }
    }

    /// The junction a shunt store hangs from, if it hangs from exactly one.
    ///
    /// `None` for a coordinate a branch carries — that one is not a shunt — and for one whose claim
    /// is ambiguous. The rule is the one the layout already uses to PLACE such a store: count the
    /// junctions of every branch it exchanges power with, ignore the reference because every shunt
    /// store is tied to it and so it distinguishes nothing, and take the junction that wins outright.
    ///
    /// Ignoring the reference is not a detail. Without it the divider's node capacitance belongs
    /// equally to `node_a` and to `ground_rail` — three branches each — and assigning it to both
    /// makes the node law fail at the ground by exactly the current the source delivers.
    pub fn junction_of(&self, coordinate: usize) -> Option<usize> {
        if self.wires.iter().any(|w| w.coordinate == coordinate) {
            return None;
        }
        let coupled: Vec<usize> = self
            .port_hamiltonian
            .interconnection
            .ports()
            .iter()
            .filter(|p| p.sources.contains(&coordinate) || p.targets.contains(&coordinate))
            .flat_map(|p| p.sources.iter().chain(p.targets.iter()).copied())
            .filter(|c| *c != coordinate)
            .collect();

        let mut tally = vec![0usize; self.terminals.len()];
        for wire in self
            .wires
            .iter()
            .filter(|w| coupled.contains(&w.coordinate))
        {
            tally[wire.from] += 1;
            tally[wire.to] += 1;
        }
        if let Some(ground) = self.reference {
            tally[ground] = 0;
        }
        let most = tally.iter().copied().max().filter(|n| *n > 0)?;
        let mut winners = tally.iter().enumerate().filter(|(_, n)| **n == most);
        let (junction, _) = winners.next()?;
        winners.next().is_none().then_some(junction)
    }

    /// The shunt store at a junction, where the model puts one there.
    ///
    /// In a circuit it is the node's capacitance; in a mechanism it is the body's mass; in both it is
    /// what absorbs whatever the branches do not balance, and it is what turns an algebraic
    /// constraint into a state. A junction with none is ANCHORED — a wall, a ground, the end of an
    /// ideal source — and takes whatever arrives, which is why there is no law to check there.
    pub fn store_at(&self, junction: usize) -> Option<usize> {
        (0..self.dim()).find(|&k| self.junction_of(k) == Some(junction))
    }

    /// Kirchhoff's laws, at a state and an instant.
    pub fn kirchhoff(&self, x: &[f64], time: f64) -> Laws {
        let n = self.dim();
        let mut effort = vec![0.0; n];
        self.effort(x, &mut effort);
        let mut input = vec![0.0; n];
        self.input_at(time, &mut input);
        let mut rate = vec![0.0; n];
        self.port_hamiltonian.flow(x, &input, &mut rate);

        // ── Tellegen ──────────────────────────────────────────────────────────────────────
        // `J e` alone, without the dissipation or the drive: those are ports to the outside and are
        // not what the interconnection is being asked about.
        let mut moved = vec![0.0; n];
        self.port_hamiltonian
            .interconnection
            .apply(&effort, &mut moved);
        let total: f64 = (0..n).map(|k| effort[k] * moved[k]).sum();
        let largest = (0..n)
            .map(|k| (effort[k] * moved[k]).abs())
            .fold(f64::MIN_POSITIVE, f64::max);
        let tellegen = total.abs() / largest;

        // ── the two laws separately, where there is a drawing ─────────────────────────────
        let (node, loops) = if self.wires.is_empty() {
            (None, None)
        } else {
            let scale = self
                .wires
                .iter()
                .map(|w| effort[w.coordinate].abs())
                .fold(f64::MIN_POSITIVE, f64::max);
            let mut worst = 0.0f64;
            for junction in 0..self.terminals.len() {
                let Some(store) = self.store_at(junction) else {
                    // An anchored junction — a reference, a wall, the end of a source — takes
                    // whatever is left over. There is no law to check there, which is what being
                    // anchored means.
                    continue;
                };
                // What the branches bring in, signed by which way each runs.
                let brought: f64 = self
                    .wires
                    .iter()
                    .filter(|w| w.from == junction || w.to == junction)
                    .map(|w| {
                        let sign = if w.to == junction { 1.0 } else { -1.0 };
                        sign * effort[w.coordinate]
                    })
                    .sum();
                // What the store does with it: takes it in, leaks some, and is pushed by its own
                // drive.
                let absorbed = rate[store]
                    + self.port_hamiltonian.dissipation[store] * effort[store]
                    - input[store];
                worst = worst.max((brought - absorbed).abs());
            }
            (
                Some(worst / scale),
                self.kirchhoff_residual(x, time).map(|r| {
                    let drops = self.branch_drops(x, time);
                    let d = drops.iter().fold(f64::MIN_POSITIVE, |m, v| m.max(v.abs()));
                    r / d
                }),
            )
        };

        Laws {
            tellegen,
            node,
            loops,
            conserved: n - rank_of(&self.state_matrix(), n),
            graph: self.interconnection_graph(),
        }
    }
}

/// The rank of a small dense matrix, by Gaussian elimination with partial pivoting.
///
/// A tolerance relative to the largest pivot seen, because a rank is a question about which
/// directions the matrix reaches and an absolute threshold answers a different one on a matrix whose
/// entries span twenty orders of magnitude — which these do.
fn rank_of(a: &[f64], n: usize) -> usize {
    let mut m = a.to_vec();
    let mut rank = 0;
    let mut biggest = 0.0f64;
    let mut used = vec![false; n];
    for column in 0..n {
        let mut pivot = None;
        let mut best = 0.0;
        for row in 0..n {
            if used[row] {
                continue;
            }
            let v = m[row * n + column].abs();
            if v > best {
                best = v;
                pivot = Some(row);
            }
        }
        biggest = biggest.max(best);
        let Some(row) = pivot.filter(|_| best > 1e-11 * biggest.max(1e-300)) else {
            continue;
        };
        used[row] = true;
        rank += 1;
        for other in 0..n {
            if other == row {
                continue;
            }
            let factor = m[other * n + column] / m[row * n + column];
            if factor == 0.0 {
                continue;
            }
            for c in column..n {
                m[other * n + c] -= factor * m[row * n + c];
            }
        }
    }
    rank
}
