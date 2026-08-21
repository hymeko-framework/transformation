//! Split-layer rewrite proposer — step 3 of the 5-step entropy
//! hot-swap plan (see `docs/structural_entropy_ir.md` + the PyTorch
//! backend memory).
//!
//! Given a scope (a hypervertex body whose inner sub-hypergraph we want
//! to split), compute a deterministic k=2 clustering of that scope's
//! vertices by their incidence-row signatures across the scope's edges.
//! Emit a [`SplitProposal`] describing:
//!
//! - which vertices fall into each of the two clusters,
//! - which inner hyperedges belong to each cluster (by majority-target
//!   vote; edges whose targets span both clusters are flagged as
//!   cross-cluster),
//! - the k-means inertia (sum of squared distances from each point to
//!   its centroid) as a rough quality score — lower = more separable.
//!
//! The proposer is pure: no IR mutation, no codegen. Step 4 of the
//! plan consumes a `SplitProposal` to emit the rewritten `.hymeko`.
//!
//! **Determinism contract.** Same `Ir` + same scope → bit-identical
//! `SplitProposal`. Vertex iteration order follows `DeclId`; centroid
//! initialisation is seed-free (first centroid = first vertex by
//! DeclId, second = vertex with max squared distance from the first,
//! ties broken by lower DeclId).

use std::collections::HashMap;

use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{DeclKind, Ir, SignedRefR, ValueR};

use crate::entropy::compute_entropy_hierarchical;

/// Maximum k-means iterations. 50 is generous for k=2 over the small
/// feature spaces we see in HyMeKo hypervertex bodies (typically < 30
/// vertices × < 20 edges). Convergence is declared when no label
/// flipped in an iteration.
const KMEANS_MAX_ITER: usize = 50;

/// How to collapse an arc's weight vector (`Option<Vec<ValueR>>`) into
/// a single scalar participation strength for the k-means feature
/// space.
///
/// HyMeKo arc weights are genuinely polymorphic — kinematic chains use
/// 3-vector `axis` / `origin` weights, neural factors use single gain
/// scalars, and tagged meta-attributes can hold nested lists. Picking
/// one hard-coded scalar reduction risks hiding structural signal; an
/// enum lets the caller match the metric to the domain (e.g., L2 for
/// axis-like vectors, `FirstScalar` for "gain + flag tail" patterns).
///
/// All variants degenerate to `|w|` for single-scalar weights — the
/// default ([`WeightAggregation::L2Norm`]) is therefore
/// backward-compatible with the earlier "first scalar" behaviour while
/// generalising naturally to vector weights.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WeightAggregation {
    /// `|first scalar|`, or `1.0` if no scalar found. Earliest
    /// behaviour — kept for parity and for cases where only the
    /// leading component is meaningful (e.g., a gain scalar followed
    /// by a tagged flag tail).
    FirstScalar,
    /// `Σ |w_i|` over all scalar components — Manhattan magnitude.
    /// Each component contributes additively. Useful when the weight
    /// vector represents additive shares (e.g., per-channel fan-out).
    L1Norm,
    /// `sqrt(Σ w_i²)` over all scalar components — Euclidean
    /// magnitude. **Default.** Generalises "weight strength" to
    /// multi-component weights such as 3-vector axes, RPY rotations,
    /// or signed-channel vectors while degenerating to `|scalar|` on
    /// single-scalar weights.
    #[default]
    L2Norm,
    /// `max |w_i|` over all scalar components — preserves the
    /// dominant component while ignoring tails. Useful when weights
    /// mix heterogeneous units and summing them would conflate unlike
    /// quantities.
    LinfNorm,
}

/// Which half of the proposed split a vertex or edge falls into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cluster {
    /// Cluster 0 — anchored at the lowest-DeclId vertex in the scope.
    A,
    /// Cluster 1 — anchored at the vertex maximally distant from A's seed.
    B,
    /// Edge spans both clusters (mixed-target incidences). Not a
    /// vertex classification — vertices are always A or B.
    Cross,
}

/// The output of [`propose_split`]: a description of the proposed
/// k=2 split of a scope's inner sub-hypergraph.
#[derive(Debug, Clone)]
pub struct SplitProposal {
    /// DeclId of the scope that was split.
    pub target_scope: DeclId,
    /// Vertices in cluster A, sorted by DeclId.
    pub cluster_a: Vec<DeclId>,
    /// Vertices in cluster B, sorted by DeclId.
    pub cluster_b: Vec<DeclId>,
    /// For each edge in `E(scope)`, the cluster it belongs to. An edge
    /// is `Cross` when its signed-incidence targets span both clusters
    /// (i.e. at least one target in A and at least one in B).
    pub edge_assignments: Vec<(DeclId, Cluster)>,
    /// k-means inertia — sum of squared distances from each vertex
    /// signature to its assigned centroid. Lower = cleaner split. Zero
    /// means every vertex sits exactly on its centroid.
    pub inertia: f64,
    /// Number of edges with `Cluster::Cross` assignment. A proposal
    /// with zero cross edges is *directly actionable* — each cluster
    /// can become its own hypervertex body without rewiring. Nonzero
    /// means step 4's regen has to rewire the crossing edges.
    pub n_cross_edges: usize,
}

impl SplitProposal {
    /// The two non-cross edge lists, for convenience when step 4 is
    /// generating per-cluster hypervertex bodies.
    pub fn edges_in(&self, cluster: Cluster) -> impl Iterator<Item = DeclId> + '_ {
        self.edge_assignments.iter()
            .filter(move |(_, c)| *c == cluster)
            .map(|(d, _)| *d)
    }
}

/// Propose a 2-way split of `scope`'s inner sub-hypergraph using the
/// default weight aggregation ([`WeightAggregation::L2Norm`]). Returns
/// `None` when the scope cannot be meaningfully split:
/// - fewer than 2 vertices (nothing to partition), or
/// - zero edges (no incidence signal to drive clustering).
///
/// For a non-default weight metric, use [`propose_split_with`].
pub fn propose_split(ir: &Ir, scope: DeclId) -> Option<SplitProposal> {
    propose_split_with(ir, scope, WeightAggregation::default())
}

/// Like [`propose_split`] but with an explicit weight aggregation. All
/// four variants degenerate to `|scalar|` for single-scalar weights,
/// so the only observable difference is on arcs that carry
/// multi-component weight vectors (axes, RPY, per-channel gains).
pub fn propose_split_with(
    ir: &Ir,
    scope: DeclId,
    weight_agg: WeightAggregation,
) -> Option<SplitProposal> {
    let (vertices, edges) = scope_vertices_and_edges(ir, scope);
    if vertices.len() < 2 || edges.is_empty() {
        return None;
    }

    let sigs = incidence_signatures(ir, &vertices, &edges, weight_agg);
    let (labels, inertia) = kmeans_two(&sigs);

    let (cluster_a, cluster_b) = split_by_labels(&vertices, &labels);
    let edge_assignments = classify_edges(ir, &edges, &cluster_a, &cluster_b);
    let n_cross_edges = edge_assignments.iter()
        .filter(|(_, c)| *c == Cluster::Cross)
        .count();

    Some(SplitProposal {
        target_scope: scope,
        cluster_a,
        cluster_b,
        edge_assignments,
        inertia,
        n_cross_edges,
    })
}

/// Auto-pick the scope with the highest `H_sign` from
/// [`compute_entropy_hierarchical`] and propose a split of it using
/// the default weight aggregation. `H_sign` is the component that
/// best reflects role-mixing within hyperedges — the most productive
/// target for a split because refactoring it typically drops the
/// outermost `H_struct` the most.
pub fn propose_split_for_highest_h_sign(ir: &Ir) -> Option<SplitProposal> {
    propose_split_for_highest_h_sign_with(ir, WeightAggregation::default())
}

/// Like [`propose_split_for_highest_h_sign`] but with an explicit
/// weight aggregation.
pub fn propose_split_for_highest_h_sign_with(
    ir: &Ir,
    weight_agg: WeightAggregation,
) -> Option<SplitProposal> {
    let scopes = compute_entropy_hierarchical(ir);
    let best = scopes.into_iter()
        .filter(|(_, e)| e.n_edges > 0 && e.n_vertices >= 2)
        .max_by(|(_, a), (_, b)| a.h_sign.partial_cmp(&b.h_sign)
            .unwrap_or(std::cmp::Ordering::Equal))?;
    propose_split_with(ir, best.0, weight_agg)
}

// ─── Internals ───────────────────────────────────────────────────────

fn scope_vertices_and_edges(ir: &Ir, scope: DeclId) -> (Vec<DeclId>, Vec<DeclId>) {
    let mut vertices = Vec::new();
    let mut edges = Vec::new();
    let children: Vec<DeclId> = if scope.is_none() {
        (0..ir.decl_nodes.len()).map(DeclId::new)
            .filter(|d| ir.decl_nodes[d.raw()].parent.is_none())
            .collect()
    } else {
        ir.decl_children(scope).collect()
    };
    for child in children {
        match ir.decl_kind(child) {
            DeclKind::Node => vertices.push(child),
            DeclKind::Edge => edges.push(child),
            DeclKind::HyperArc => {}
        }
    }
    (vertices, edges)
}

/// Per-vertex signature: one f64 entry per edge, carrying the
/// weighted participation strength of that vertex in that edge
/// (summed across all arcs of the edge that reference the vertex).
///
/// Strength per arc is derived from the arc's weights via
/// [`WeightAggregation`] — L2 norm by default, degenerating to
/// `|scalar|` for single-scalar weights. Multiple arcs referencing
/// the same vertex inside the same edge accumulate.
///
/// **Why binary-on-sign (but magnitude-weighted).** Clustering on
/// *which* edges a vertex belongs to is the useful signal for layer
/// splitting; using the raw signed value `+1/-1/0` puts `+v` and `-v`
/// on opposite ends of the feature axis, which breaks the
/// disjoint-edges case into lopsided clusters. Weights add genuine
/// structural nuance (an arc with weight 5.0 is *more* participatory
/// than one with 0.1) without re-introducing the sign-symmetry trap.
/// Signed role information still drives [`classify_one_edge`], so no
/// structural detail is lost — only the feature space is weight-only.
fn incidence_signatures(
    ir: &Ir,
    vertices: &[DeclId],
    edges: &[DeclId],
    weight_agg: WeightAggregation,
) -> Vec<Vec<f64>> {
    let v_idx: HashMap<DeclId, usize> = vertices.iter().enumerate()
        .map(|(i, d)| (*d, i))
        .collect();
    let mut sigs = vec![vec![0.0; edges.len()]; vertices.len()];
    for (e_idx, e_did) in edges.iter().enumerate() {
        let Some(eid) = ir.as_edge(*e_did) else { continue };
        for &aid in &ir.edges[eid.0].arcs {
            for r in &ir.arcs[aid.0].refs {
                let Some(&row) = v_idx.get(&r.target()) else { continue };
                sigs[row][e_idx] += arc_weight_magnitude(r, weight_agg);
            }
        }
    }
    sigs
}

/// Collapse an arc's `Option<Vec<ValueR>>` weight vector to a single
/// scalar strength via the chosen aggregation. Scalar components are
/// gathered by a depth-first walk through the weight vector — nested
/// `ValueR::List` layers are flattened (HyMeKo's kinematic arc weights
/// use `[[xyz], [rpy]]` grouping, so nested lists are the norm, not
/// the exception). `Str` and `Ref` entries are skipped. If no scalar
/// survives, or the weights are absent entirely, returns `1.0` so the
/// arc still counts as a unit participation in the feature space.
fn arc_weight_magnitude(r: &SignedRefR, agg: WeightAggregation) -> f64 {
    let Some(ws) = r.atom().weights.as_ref() else { return 1.0 };
    let mut scalars = Vec::new();
    for v in ws {
        collect_scalars(v, &mut scalars);
    }
    if scalars.is_empty() { return 1.0; }
    match agg {
        WeightAggregation::FirstScalar => scalars[0].abs(),
        WeightAggregation::L1Norm      => scalars.iter().map(|n| n.abs()).sum(),
        WeightAggregation::L2Norm      => scalars.iter().map(|n| n * n).sum::<f64>().sqrt(),
        WeightAggregation::LinfNorm    => scalars.iter().map(|n| n.abs()).fold(0.0_f64, f64::max),
    }
}

fn collect_scalars(v: &ValueR, out: &mut Vec<f64>) {
    match v {
        ValueR::Num(n)  => out.push(*n),
        ValueR::List(xs) => xs.iter().for_each(|x| collect_scalars(x, out)),
        ValueR::Str(_) | ValueR::Ref(_) => {}
    }
}

/// Deterministic k=2 k-means. Returns `(labels, inertia)` where
/// `labels[i]` is 0 or 1 and `inertia` is the sum of squared distances
/// from each point to its assigned centroid.
fn kmeans_two(sigs: &[Vec<f64>]) -> (Vec<usize>, f64) {
    let n = sigs.len();
    if n < 2 {
        return (vec![0; n], 0.0);
    }
    let dim = sigs[0].len();
    let (i0, i1) = seed_indices(sigs);
    let mut centroids = [sigs[i0].clone(), sigs[i1].clone()];
    let mut labels = vec![0usize; n];

    for _ in 0..KMEANS_MAX_ITER {
        let mut changed = false;
        for (i, sig) in sigs.iter().enumerate() {
            let d0 = squared_distance(sig, &centroids[0]);
            let d1 = squared_distance(sig, &centroids[1]);
            let new_label = if d0 <= d1 { 0 } else { 1 };
            if labels[i] != new_label {
                labels[i] = new_label;
                changed = true;
            }
        }
        if !changed {
            break;
        }
        for k in 0..2 {
            recompute_centroid(&mut centroids[k], sigs, &labels, k, dim);
        }
    }

    let inertia: f64 = sigs.iter().zip(labels.iter())
        .map(|(sig, &l)| squared_distance(sig, &centroids[l]))
        .sum();
    (labels, inertia)
}

/// Pick two seed indices deterministically: `i0` is the lowest-index
/// vertex, `i1` is the vertex farthest from `sigs[i0]` (ties broken by
/// lower index). Guarantees same IR → same seeds → same clustering.
fn seed_indices(sigs: &[Vec<f64>]) -> (usize, usize) {
    let i0 = 0usize;
    let mut best_i1 = 1.min(sigs.len() - 1);
    let mut best_d = squared_distance(&sigs[i0], &sigs[best_i1]);
    for (i, sig) in sigs.iter().enumerate().skip(1) {
        let d = squared_distance(sig, &sigs[i0]);
        if d > best_d {
            best_d = d;
            best_i1 = i;
        }
    }
    (i0, best_i1)
}

fn recompute_centroid(
    centroid: &mut Vec<f64>,
    sigs: &[Vec<f64>],
    labels: &[usize],
    k: usize,
    dim: usize,
) {
    let mut sum = vec![0.0; dim];
    let mut count = 0usize;
    for (sig, &label) in sigs.iter().zip(labels.iter()) {
        if label != k {
            continue;
        }
        for d in 0..dim {
            sum[d] += sig[d];
        }
        count += 1;
    }
    if count == 0 {
        return; // leave centroid where it was (empty cluster edge case)
    }
    for d in 0..dim {
        sum[d] /= count as f64;
    }
    *centroid = sum;
}

fn squared_distance(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b.iter())
        .map(|(x, y)| (x - y).powi(2))
        .sum()
}

fn split_by_labels(vertices: &[DeclId], labels: &[usize]) -> (Vec<DeclId>, Vec<DeclId>) {
    let mut a = Vec::new();
    let mut b = Vec::new();
    for (v, &l) in vertices.iter().zip(labels.iter()) {
        if l == 0 { a.push(*v); } else { b.push(*v); }
    }
    a.sort_by_key(|d| d.raw());
    b.sort_by_key(|d| d.raw());
    (a, b)
}

/// Classify each edge by which cluster owns the majority of its
/// signed-incidence targets. An edge whose targets span both clusters
/// is flagged `Cluster::Cross` — those are the edges step 4 has to
/// rewire when it emits the split `.hymeko`.
fn classify_edges(
    ir: &Ir,
    edges: &[DeclId],
    cluster_a: &[DeclId],
    cluster_b: &[DeclId],
) -> Vec<(DeclId, Cluster)> {
    let in_a: HashMap<DeclId, ()> = cluster_a.iter().map(|d| (*d, ())).collect();
    let in_b: HashMap<DeclId, ()> = cluster_b.iter().map(|d| (*d, ())).collect();
    edges.iter()
        .map(|&e_did| (e_did, classify_one_edge(ir, e_did, &in_a, &in_b)))
        .collect()
}

fn classify_one_edge(
    ir: &Ir,
    e_did: DeclId,
    in_a: &HashMap<DeclId, ()>,
    in_b: &HashMap<DeclId, ()>,
) -> Cluster {
    let Some(eid) = ir.as_edge(e_did) else { return Cluster::Cross };
    let mut count_a = 0usize;
    let mut count_b = 0usize;
    for &aid in &ir.edges[eid.0].arcs {
        for r in &ir.arcs[aid.0].refs {
            if in_a.contains_key(&r.target()) { count_a += 1; }
            else if in_b.contains_key(&r.target()) { count_b += 1; }
        }
    }
    match (count_a, count_b) {
        (0, 0) => Cluster::Cross, // edge targets nothing in scope
        (_, 0) => Cluster::A,
        (0, _) => Cluster::B,
        _      => Cluster::Cross,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hymeko::common::ids::{EdgeId, HyperArcId, NodeId, SymId};
    use hymeko::ir::ir::{AnnoR, ArcRec, DeclNode, EdgeRec, NodeRec, RefAtomR};

    // Tiny IR builder (mirrors the one in entropy.rs tests).

    fn push_decl(ir: &mut Ir, parent: DeclId, kind: DeclKind) -> DeclId {
        let did = DeclId::new(ir.decl_nodes.len());
        ir.decl_nodes.push(DeclNode {
            kind,
            name: SymId::new(0),
            parent,
            first_child: DeclId::NONE,
            last_child: DeclId::NONE,
            next_sibling: DeclId::NONE,
            anno: AnnoR::default(),
        });
        ir.decl_to_node.push(None);
        ir.decl_to_edge.push(None);
        ir.decl_to_arc.push(None);
        ir.decl_hash.push(None);
        if parent.is_some() {
            let first = ir.decl_nodes[parent.0].first_child;
            if first.is_none() {
                ir.decl_nodes[parent.0].first_child = did;
                ir.decl_nodes[parent.0].last_child = did;
            } else {
                let last = ir.decl_nodes[parent.0].last_child;
                ir.decl_nodes[last.0].next_sibling = did;
                ir.decl_nodes[parent.0].last_child = did;
            }
        }
        did
    }

    fn push_node(ir: &mut Ir, parent: DeclId) -> DeclId {
        let did = push_decl(ir, parent, DeclKind::Node);
        let nid = NodeId::new(ir.nodes.len());
        ir.nodes.push(NodeRec::new(did, Vec::new()));
        ir.decl_to_node[did.0] = Some(nid);
        did
    }

    fn push_edge(ir: &mut Ir, parent: DeclId, incidences: &[(i8, DeclId)]) -> DeclId {
        let weighted: Vec<(i8, DeclId, Option<f64>)> = incidences.iter()
            .map(|(s, d)| (*s, *d, None))
            .collect();
        push_edge_weighted(ir, parent, &weighted)
    }

    fn push_edge_weighted(
        ir: &mut Ir,
        parent: DeclId,
        incidences: &[(i8, DeclId, Option<f64>)],
    ) -> DeclId {
        let e_did = push_decl(ir, parent, DeclKind::Edge);
        let eid = EdgeId::new(ir.edges.len());
        ir.edges.push(EdgeRec::new(e_did, Vec::new()));
        ir.decl_to_edge[e_did.0] = Some(eid);

        let arc_did = push_decl(ir, e_did, DeclKind::HyperArc);
        let aid = HyperArcId::new(ir.arcs.len());
        let refs = incidences.iter()
            .map(|&(sign, target, weight)| {
                let weights = weight.map(|w| vec![ValueR::Num(w)]);
                let atom = RefAtomR { target, anno: AnnoR::default(), weights };
                match sign {
                    1  => SignedRefR::Plus(atom),
                    -1 => SignedRefR::Minus(atom),
                    _  => SignedRefR::Neutral(atom),
                }
            })
            .collect();
        ir.arcs.push(ArcRec { anno: AnnoR::default(), in_edge: e_did, refs });
        ir.decl_to_arc[arc_did.0] = Some(aid);
        ir.edges[eid.0].arcs.push(aid);
        e_did
    }

    #[test]
    fn scope_with_one_vertex_rejected() {
        let mut ir = Ir::default();
        let root = push_node(&mut ir, DeclId::NONE);
        push_node(&mut ir, root);
        assert!(propose_split(&ir, root).is_none());
    }

    #[test]
    fn scope_with_no_edges_rejected() {
        let mut ir = Ir::default();
        let root = push_node(&mut ir, DeclId::NONE);
        push_node(&mut ir, root);
        push_node(&mut ir, root);
        assert!(propose_split(&ir, root).is_none());
    }

    #[test]
    fn two_disjoint_edges_split_cleanly() {
        // Two edges, each touching 2 vertices, no shared vertices →
        // k-means should place each vertex with its edge-mates.
        let mut ir = Ir::default();
        let root = push_node(&mut ir, DeclId::NONE);
        let v0 = push_node(&mut ir, root);
        let v1 = push_node(&mut ir, root);
        let v2 = push_node(&mut ir, root);
        let v3 = push_node(&mut ir, root);
        let e_left  = push_edge(&mut ir, root, &[(1, v0), (-1, v1)]);
        let e_right = push_edge(&mut ir, root, &[(1, v2), (-1, v3)]);

        let proposal = propose_split(&ir, root).expect("should propose");
        // Each cluster must contain exactly one edge's vertices.
        assert_eq!(proposal.cluster_a.len() + proposal.cluster_b.len(), 4);
        // Both edges should be non-cross (their targets are disjoint clusters).
        let cross: Vec<_> = proposal.edge_assignments.iter()
            .filter(|(_, c)| *c == Cluster::Cross)
            .collect();
        assert!(cross.is_empty(), "expected zero cross edges, got {}", cross.len());
        // Edge e_left and e_right must end up in different clusters.
        let e_left_cluster  = proposal.edge_assignments.iter()
            .find(|(d, _)| *d == e_left).unwrap().1;
        let e_right_cluster = proposal.edge_assignments.iter()
            .find(|(d, _)| *d == e_right).unwrap().1;
        assert_ne!(e_left_cluster, e_right_cluster);
        assert_eq!(proposal.n_cross_edges, 0);
    }

    #[test]
    fn shared_vertex_produces_cross_edge() {
        // Two edges both touching a shared vertex. Clusters will split
        // the non-shared vertices; the edges, whose targets span both,
        // are cross-cluster.
        let mut ir = Ir::default();
        let root = push_node(&mut ir, DeclId::NONE);
        let v0 = push_node(&mut ir, root);
        let v1 = push_node(&mut ir, root);
        let shared = push_node(&mut ir, root);
        push_edge(&mut ir, root, &[(1, v0), (-1, shared)]);
        push_edge(&mut ir, root, &[(1, v1), (-1, shared)]);

        let proposal = propose_split(&ir, root).expect("should propose");
        // At least one edge must be cross (the shared-vertex targets).
        assert!(proposal.n_cross_edges >= 1,
                "expected at least one cross edge on shared-vertex topology, got 0");
    }

    #[test]
    fn weight_aggregation_metrics_differ_on_vector_weights() {
        // Two arcs with multi-component weight vectors:
        //   arc0 weight = (3.0, 4.0)      — L1 = 7.0, L2 = 5.0, Linf = 4.0
        //   arc1 weight = (1.0, 1.0, 1.0) — L1 = 3.0, L2 ≈ 1.732, Linf = 1.0
        // L1 ranks arc0 2.33× heavier than arc1, L2 ranks it 2.89×,
        // Linf ranks it 4× — different metrics genuinely reorder the
        // relative strengths, so the resulting per-vertex signatures
        // (and thus the k-means inertia) should not match.
        let mut ir = Ir::default();
        let root = push_node(&mut ir, DeclId::NONE);
        let v0 = push_node(&mut ir, root);
        let v1 = push_node(&mut ir, root);
        let hub = push_node(&mut ir, root);

        // Manually push edges with multi-component weights — the test
        // helper only supports single-scalar weights.
        push_edge_with_raw_weights(&mut ir, root,
            &[(1, hub), (-1, v0)],
            &[None, Some(vec![3.0, 4.0])]);
        push_edge_with_raw_weights(&mut ir, root,
            &[(1, hub), (-1, v1)],
            &[None, Some(vec![1.0, 1.0, 1.0])]);

        let l1   = propose_split_with(&ir, root, WeightAggregation::L1Norm).unwrap();
        let l2   = propose_split_with(&ir, root, WeightAggregation::L2Norm).unwrap();
        let linf = propose_split_with(&ir, root, WeightAggregation::LinfNorm).unwrap();
        let first = propose_split_with(&ir, root, WeightAggregation::FirstScalar).unwrap();

        // All four metrics agree on topology (no edge can be non-cross
        // because `hub` bridges every edge), but their inertia values
        // differ because the coordinates themselves differ.
        let inertias = [l1.inertia, l2.inertia, linf.inertia, first.inertia];
        let unique: std::collections::BTreeSet<u64> = inertias
            .iter()
            .map(|x| x.to_bits())
            .collect();
        assert!(
            unique.len() >= 3,
            "expected ≥3 distinct inertias across L1/L2/Linf/FirstScalar, got {unique:?} from {inertias:?}"
        );
    }

    #[test]
    fn weight_aggregation_default_is_l2() {
        // Single-scalar weights → L1 = L2 = Linf = FirstScalar, so the
        // default should match every explicit metric choice on the
        // simple case that exercised the earlier weight-sensitivity
        // test.
        let mut ir = Ir::default();
        let root = push_node(&mut ir, DeclId::NONE);
        let hub = push_node(&mut ir, root);
        let v0 = push_node(&mut ir, root);
        let v1 = push_node(&mut ir, root);
        push_edge_weighted(&mut ir, root, &[(1, hub, Some(1.0)), (-1, v0, Some(10.0))]);
        push_edge_weighted(&mut ir, root, &[(1, hub, Some(1.0)), (-1, v1, Some(0.1))]);

        let default = propose_split(&ir, root).unwrap();
        let explicit_l2 = propose_split_with(&ir, root, WeightAggregation::L2Norm).unwrap();
        assert_eq!(default.inertia, explicit_l2.inertia,
                   "propose_split should default to L2Norm");
    }

    fn push_edge_with_raw_weights(
        ir: &mut Ir,
        parent: DeclId,
        incidences: &[(i8, DeclId)],
        weights: &[Option<Vec<f64>>],
    ) -> DeclId {
        assert_eq!(incidences.len(), weights.len(),
                   "incidence and weight slices must have matching length");
        let e_did = push_decl(ir, parent, DeclKind::Edge);
        let eid = EdgeId::new(ir.edges.len());
        ir.edges.push(EdgeRec::new(e_did, Vec::new()));
        ir.decl_to_edge[e_did.0] = Some(eid);

        let arc_did = push_decl(ir, e_did, DeclKind::HyperArc);
        let aid = HyperArcId::new(ir.arcs.len());
        let refs = incidences.iter().zip(weights.iter())
            .map(|(&(sign, target), w)| {
                let weights = w.as_ref().map(|ws| ws.iter().map(|n| ValueR::Num(*n)).collect());
                let atom = RefAtomR { target, anno: AnnoR::default(), weights };
                match sign {
                    1  => SignedRefR::Plus(atom),
                    -1 => SignedRefR::Minus(atom),
                    _  => SignedRefR::Neutral(atom),
                }
            })
            .collect();
        ir.arcs.push(ArcRec { anno: AnnoR::default(), in_edge: e_did, refs });
        ir.decl_to_arc[arc_did.0] = Some(aid);
        ir.edges[eid.0].arcs.push(aid);
        e_did
    }

    #[test]
    fn weights_affect_inertia() {
        // Three edges sharing a common hub vertex v_hub, plus two
        // leaves v0, v1 with *different* weights on their arcs.
        // Unweighted: v0 and v1 have identical participation patterns
        // and would cluster together. Weighted: v0's arcs are 10× v1's,
        // so their signatures differ enough to land in opposite
        // clusters (or at minimum produce different inertia).
        let mut ir_unweighted = Ir::default();
        {
            let root = push_node(&mut ir_unweighted, DeclId::NONE);
            let hub = push_node(&mut ir_unweighted, root);
            let v0  = push_node(&mut ir_unweighted, root);
            let v1  = push_node(&mut ir_unweighted, root);
            push_edge(&mut ir_unweighted, root, &[(1, hub), (-1, v0)]);
            push_edge(&mut ir_unweighted, root, &[(1, hub), (-1, v1)]);
        }
        let mut ir_weighted = Ir::default();
        {
            let root = push_node(&mut ir_weighted, DeclId::NONE);
            let hub = push_node(&mut ir_weighted, root);
            let v0  = push_node(&mut ir_weighted, root);
            let v1  = push_node(&mut ir_weighted, root);
            push_edge_weighted(&mut ir_weighted, root, &[
                (1, hub, Some(1.0)), (-1, v0, Some(10.0)),
            ]);
            push_edge_weighted(&mut ir_weighted, root, &[
                (1, hub, Some(1.0)), (-1, v1, Some(0.1)),
            ]);
        }
        let unweighted = propose_split(&ir_unweighted, DeclId::new(0)).expect("unweighted");
        let weighted   = propose_split(&ir_weighted,   DeclId::new(0)).expect("weighted");
        // Weights are supposed to carry into the feature space, so the
        // two proposals should not be structurally identical. Compare
        // inertia — weighted will have a different value because the
        // coordinates themselves are different.
        assert_ne!(
            unweighted.inertia, weighted.inertia,
            "weights should produce a different inertia (unweighted = weighted = {})",
            unweighted.inertia,
        );
    }

    #[test]
    fn determinism_two_calls_produce_identical_proposal() {
        // Same IR called twice → same labels, same inertia. Guards
        // against HashMap iteration order leaking into the result.
        let mut ir = Ir::default();
        let root = push_node(&mut ir, DeclId::NONE);
        let v0 = push_node(&mut ir, root);
        let v1 = push_node(&mut ir, root);
        let v2 = push_node(&mut ir, root);
        let v3 = push_node(&mut ir, root);
        push_edge(&mut ir, root, &[(1, v0), (-1, v1)]);
        push_edge(&mut ir, root, &[(1, v2), (-1, v3)]);

        let a = propose_split(&ir, root).unwrap();
        let b = propose_split(&ir, root).unwrap();
        assert_eq!(a.cluster_a, b.cluster_a);
        assert_eq!(a.cluster_b, b.cluster_b);
        assert_eq!(a.inertia, b.inertia);
        assert_eq!(a.n_cross_edges, b.n_cross_edges);
    }
}
