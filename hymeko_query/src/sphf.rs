//! The signed patch hypergraph a compiled IR already is.
//!
//! # This is a reading, not a construction
//!
//! A G-SPHF is the tuple `(V, E, B, sigma, P)`: vertices, hyperedges, an incidence relation, a sign
//! on each incidence, and a partition of the vertices into named patches. The HyMeKo IR carries all
//! five, and it carries them under those names:
//!
//! | G-SPHF  | IR                                                                     |
//! | ------- | ---------------------------------------------------------------------- |
//! | `V`     | declarations of [`DeclKind::Node`]                                     |
//! | `E`     | declarations of [`DeclKind::Edge`] and [`DeclKind::HyperArc`]          |
//! | `B`     | the `refs` of an arc and the `bases` of an edge                        |
//! | `sigma` | [`SignedRefR`] — `Plus`, `Minus`, `Neutral`                            |
//! | `P`     | the declaration tree: a vertex's patch is the declaration enclosing it  |
//!
//! So nothing here invents a structure to lay over the IR. It reads the one that is there, and the
//! only work it does is to flatten a tree of declarations into flat index arrays that an algorithm
//! can sweep — the same trade `FkPlan` makes for kinematics, for the same reason.
//!
//! # What it is for
//!
//! Two things, and they are the same thing seen twice.
//!
//! **The interconnection of a port-Hamiltonian system.** A port contributes
//! `w (e_u e_v^T - e_v e_u^T)` over its sources and targets, which is antisymmetric term by term —
//! so the structure matrix is skew whatever the hypergraph looks like, and `e^T J e = 0` identically.
//! Energy cannot be created by the interconnection, because the interconnection is a sign
//! convention. [`SignedHypergraph::ports`] hands the sets to whoever builds that matrix.
//!
//! **The size of the dense part of anything.** A spanning tree can be swept in linear time; what
//! cannot is the extra edges, and their number is the [cyclomatic
//! number](SignedHypergraph::cyclomatic_number). That is what a constraint solve costs.
//!
//! # The IR's hypergraph is not the domain's graph
//!
//! A joint in a robot reads out as a **ternary** edge, not a binary one:
//!
//! ```text
//! edge waist   patch quadruped   [ +torso  -rear_body  -AXIS_Z ]
//! ```
//!
//! parent, child, and the axis — and `AXIS_Z` is one declaration SHARED by every joint that turns
//! about it. So the joints of a kinematic tree are all connected through it, and the cyclomatic
//! number of the full reading counts those shared-parameter cycles along with any real ones. On the
//! library quadruped that is twenty, against a mechanism which is a tree and has none.
//!
//! That is not a defect in either the IR or the count. It is the difference between the structure as
//! declared and a domain's projection of it, and [`SignedHypergraph::restricted_to`] is how one
//! becomes the other: keep the vertices the domain calls bodies, drop the rest, and the quadruped's
//! cyclomatic number falls to zero because it really is a tree.

use std::collections::HashMap;

use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{DeclKind, Ir, SignedRefR};

/// The sign an incidence carries: `sigma(v, e)`.
///
/// Source and target are `-1` and `+1` by the G-SPHF convention. `Neutral` is the third thing the IR
/// can say and a plain signed graph cannot: a vertex that takes part in an edge without a direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Role {
    /// `sigma = -1`.
    Source,
    /// `sigma = +1`.
    Target,
    /// In the edge, with no direction.
    Neutral,
}

impl Role {
    /// `-1`, `+1` or `0`.
    pub fn sign(self) -> f64 {
        match self {
            Self::Source => -1.0,
            Self::Target => 1.0,
            Self::Neutral => 0.0,
        }
    }
}

/// One incidence: a vertex in a hyperedge, with its sign and its weight.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Incidence {
    /// Index into [`SignedHypergraph::vertices`].
    pub vertex: usize,
    pub role: Role,
    /// The first numeric weight the reference carries, or `1.0` where it carries none.
    ///
    /// One number and not the whole list, because that is what an interconnection needs and because
    /// a reference carrying several means something the IR has not decided yet. The full list stays
    /// in the IR for a caller that wants it.
    pub weight: f64,
}

/// A hyperedge and everything incident to it.
#[derive(Debug, Clone, PartialEq)]
pub struct Hyperedge {
    /// The declaration this edge came from.
    pub decl: DeclId,
    /// Which declaration encloses it — its own patch.
    pub patch: DeclId,
    pub incidences: Vec<Incidence>,
}

impl Hyperedge {
    /// Vertices incident with `sigma = -1`.
    pub fn sources(&self) -> impl Iterator<Item = usize> + '_ {
        self.incidences
            .iter()
            .filter(|i| i.role == Role::Source)
            .map(|i| i.vertex)
    }

    /// Vertices incident with `sigma = +1`.
    pub fn targets(&self) -> impl Iterator<Item = usize> + '_ {
        self.incidences
            .iter()
            .filter(|i| i.role == Role::Target)
            .map(|i| i.vertex)
    }

    /// How many vertices it touches. Two is an ordinary edge; more is a junction.
    pub fn arity(&self) -> usize {
        self.incidences.len()
    }
}

/// `(V, E, B, sigma, P)`, flattened.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SignedHypergraph {
    /// The vertices, by declaration, in arena order.
    pub vertices: Vec<DeclId>,
    /// The hyperedges.
    pub edges: Vec<Hyperedge>,
    /// Each vertex's patch — the declaration that encloses it.
    pub patch: Vec<DeclId>,
}

impl SignedHypergraph {
    /// Read the whole IR.
    pub fn of(ir: &Ir) -> Self {
        Self::within(ir, None)
    }

    /// Read one subtree of the declaration arena, or all of it when `scope` is `None`.
    ///
    /// A scope is a patch, so this is how a subsystem is taken out on its own — which is the point of
    /// `P` being in the tuple at all.
    pub fn within(ir: &Ir, scope: Option<DeclId>) -> Self {
        let inside = |decl: DeclId| match scope {
            None => true,
            Some(root) => encloses(ir, root, decl),
        };

        // Vertices first, so an incidence can name one by index.
        let mut vertices = Vec::new();
        let mut patch = Vec::new();
        let mut index: HashMap<DeclId, usize> = HashMap::new();
        for (raw, node) in ir.decl_nodes.iter().enumerate() {
            let decl = DeclId::new(raw);
            if node.kind != DeclKind::Node || !inside(decl) {
                continue;
            }
            index.insert(decl, vertices.len());
            vertices.push(decl);
            patch.push(node.parent);
        }

        // Then the edges. An arc's `refs` are its incidences; an edge with no arcs still carries its
        // own `bases`, which is how a binary edge is written.
        let mut edges = Vec::new();
        for record in &ir.arcs {
            let decl = record.in_edge;
            if !inside(decl) {
                continue;
            }
            let incidences = incidences_of(&record.refs, &index);
            if incidences.is_empty() {
                continue;
            }
            edges.push(Hyperedge {
                decl,
                patch: parent_of(ir, decl),
                incidences,
            });
        }
        for record in &ir.edges {
            if !record.arcs.is_empty() || !inside(record.decl) {
                continue; // its arcs have already been read
            }
            let incidences = incidences_of(&record.bases, &index);
            if incidences.is_empty() {
                continue;
            }
            edges.push(Hyperedge {
                decl: record.decl,
                patch: parent_of(ir, record.decl),
                incidences,
            });
        }

        Self {
            vertices,
            edges,
            patch,
        }
    }

    /// The patches, in order of first appearance.
    pub fn patches(&self) -> Vec<DeclId> {
        let mut seen = Vec::new();
        for &p in &self.patch {
            if !seen.contains(&p) {
                seen.push(p);
            }
        }
        seen
    }

    /// How many independent cycles the structure has: `|E| - |V| + c`, over the graph whose edges
    /// are the hyperedges.
    ///
    /// **This is the size of the dense part of anything solved over the structure.** A spanning tree
    /// is swept in linear time; the extra edges are what cannot be, and there are exactly this many
    /// of them.
    ///
    /// A hyperedge of arity `k` closes `k - 1` connections, so it counts as `k - 1` edges. An edge
    /// touching one vertex or none closes nothing.
    ///
    /// # It counts the cycles of THIS hypergraph, which may not be the ones you mean
    ///
    /// Measured on the library quadruped, the full reading gives twenty — and the mechanism is a
    /// tree with no closed loops at all. The difference is the axis declarations: a joint is a
    /// ternary edge over parent, child and axis, and one `AXIS_Z` is shared by every joint that
    /// turns about it, so they are all connected through it.
    ///
    /// [`Self::restricted_to`] is how a domain asks its own question. Kept to the links, the
    /// quadruped comes out at zero.
    pub fn cyclomatic_number(&self) -> usize {
        let connections: usize = self
            .edges
            .iter()
            .map(|e| e.arity().saturating_sub(1))
            .sum();
        let components = self.components();
        (connections + components).saturating_sub(self.vertices.len())
    }

    /// Connected components over the vertices, counting a hyperedge as joining everything it touches.
    pub fn components(&self) -> usize {
        let n = self.vertices.len();
        let mut parent: Vec<usize> = (0..n).collect();
        fn find(parent: &mut [usize], mut i: usize) -> usize {
            while parent[i] != i {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            i
        }
        for edge in &self.edges {
            let mut members = edge.incidences.iter().map(|i| i.vertex);
            let Some(first) = members.next() else { continue };
            for other in members {
                let (a, b) = (find(&mut parent, first), find(&mut parent, other));
                if a != b {
                    parent[a] = b;
                }
            }
        }
        (0..n).filter(|&i| find(&mut parent, i) == i).count()
    }

    /// The same structure over a subset of the vertices, with everything outside it dropped.
    ///
    /// The projection a domain needs to ask about its own graph rather than about the declaration's.
    /// A joint reads out as parent, child and axis; keep the bodies and drop the axes, and what is
    /// left is the kinematic graph, whose cyclomatic number really is its closed-loop count.
    ///
    /// Vertices are renumbered in their existing order. An edge left with no incidences is dropped;
    /// an edge left with one is kept, because a self-loop and an isolated vertex are different
    /// things and the count above already treats them so.
    pub fn restricted_to(&self, keep: impl Fn(usize) -> bool) -> Self {
        let mut remap = vec![usize::MAX; self.vertices.len()];
        let (mut vertices, mut patch) = (Vec::new(), Vec::new());
        for (i, &decl) in self.vertices.iter().enumerate() {
            if keep(i) {
                remap[i] = vertices.len();
                vertices.push(decl);
                patch.push(self.patch[i]);
            }
        }
        let edges = self
            .edges
            .iter()
            .filter_map(|edge| {
                let incidences: Vec<Incidence> = edge
                    .incidences
                    .iter()
                    .filter(|i| remap[i.vertex] != usize::MAX)
                    .map(|i| Incidence {
                        vertex: remap[i.vertex],
                        ..*i
                    })
                    .collect();
                if incidences.is_empty() {
                    return None;
                }
                Some(Hyperedge {
                    decl: edge.decl,
                    patch: edge.patch,
                    incidences,
                })
            })
            .collect();
        Self {
            vertices,
            edges,
            patch,
        }
    }

    /// Each edge as `(sources, targets, weight)` — what an interconnection is built from.
    ///
    /// The weight is the first one any of the edge's incidences carries, since a port has one
    /// coupling strength and the incidences of one port are two ends of it. An edge with no source or
    /// no target is skipped: it moves nothing, and a port with only one side is not a port.
    pub fn ports(&self) -> Vec<(Vec<usize>, Vec<usize>, f64)> {
        self.edges
            .iter()
            .filter_map(|edge| {
                let sources: Vec<usize> = edge.sources().collect();
                let targets: Vec<usize> = edge.targets().collect();
                if sources.is_empty() || targets.is_empty() {
                    return None;
                }
                let weight = edge
                    .incidences
                    .iter()
                    .map(|i| i.weight)
                    .find(|w| *w != 1.0)
                    .unwrap_or(1.0);
                Some((sources, targets, weight))
            })
            .collect()
    }

    /// The signed incidence matrix, row-major, `|V|` by `|E|`.
    ///
    /// For looking at and for tests; nothing that sweeps the structure needs it, which is the whole
    /// point of keeping the structure rather than a matrix of it.
    pub fn incidence_matrix(&self) -> Vec<f64> {
        let (v, e) = (self.vertices.len(), self.edges.len());
        let mut m = vec![0.0; v * e];
        for (column, edge) in self.edges.iter().enumerate() {
            for incidence in &edge.incidences {
                m[incidence.vertex * e + column] += incidence.role.sign() * incidence.weight;
            }
        }
        m
    }
}

/// The first numeric weight a reference carries, or `1.0`.
fn weight_of(atom: &hymeko::ir::ir::RefAtomR) -> f64 {
    use hymeko::ir::ir::ValueR;
    atom.weights
        .as_ref()
        .and_then(|w| w.first())
        .and_then(|v| match v {
            ValueR::Num(n) => Some(*n),
            _ => None,
        })
        .unwrap_or(1.0)
}

fn incidences_of(refs: &[SignedRefR], index: &HashMap<DeclId, usize>) -> Vec<Incidence> {
    refs.iter()
        .filter_map(|reference| {
            let (role, atom) = match reference {
                SignedRefR::Minus(a) => (Role::Source, a),
                SignedRefR::Plus(a) => (Role::Target, a),
                SignedRefR::Neutral(a) => (Role::Neutral, a),
            };
            Some(Incidence {
                vertex: *index.get(&atom.target)?,
                role,
                weight: weight_of(atom),
            })
        })
        .collect()
}

fn parent_of(ir: &Ir, decl: DeclId) -> DeclId {
    ir.decl_nodes
        .get(decl.raw())
        .map(|n| n.parent)
        .unwrap_or(DeclId::NONE)
}

/// Is `decl` inside the subtree rooted at `root`, or `root` itself?
fn encloses(ir: &Ir, root: DeclId, decl: DeclId) -> bool {
    let mut here = decl;
    let mut guard = 0usize;
    while here.is_some() && guard <= ir.decl_nodes.len() {
        if here == root {
            return true;
        }
        here = parent_of(ir, here);
        guard += 1;
    }
    false
}
