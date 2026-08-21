//! Structural-entropy walk over the hierarchical HyMeKo IR.
//!
//! See `docs/structural_entropy_ir.md` for the definition, worked
//! example, and contract. This module is step 1 of the 5-step entropy
//! hot-swap plan (`project_pytorch_backend.md`): a pure, deterministic
//! read-path that consumes a compiled `Ir` and emits per-scope Shannon
//! entropies computed from structural features only (arity, sign roles,
//! degree). No tensors, no Python, no codegen.

use std::collections::{BTreeMap, HashMap};

use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{DeclKind, Ir, SignedRefR};

/// Per-scope structural entropy, in nats. Components are returned
/// separately so callers can reweight without recomputing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StructuralEntropy {
    /// Shannon entropy of the hyperedge-arity distribution in this scope.
    pub h_arity: f64,
    /// Mean over edges of the per-edge sign-role entropy (alphabet `{+, -, ~}`).
    pub h_sign: f64,
    /// Shannon entropy of the vertex-degree distribution in this scope.
    pub h_degree: f64,
    /// `(h_arity + h_sign + h_degree) / 3.0`.
    pub h_total: f64,
    /// Number of `DeclKind::Node` direct children in this scope.
    pub n_vertices: usize,
    /// Number of `DeclKind::Edge` direct children in this scope.
    pub n_edges: usize,
}

impl StructuralEntropy {
    /// All-zeros value, used for empty scopes and as the numerical floor
    /// when either `V(S)` or `E(S)` is empty.
    pub const ZERO: Self = Self {
        h_arity: 0.0,
        h_sign: 0.0,
        h_degree: 0.0,
        h_total: 0.0,
        n_vertices: 0,
        n_edges: 0,
    };
}

/// Compute the structural entropy of one scope. The scope is
/// `DeclId::NONE` for the module root (iterate all decls whose parent
/// is NONE) or a concrete hypervertex `DeclId` (iterate its body).
///
/// Walks only *direct* children of the scope — hypervertex bodies are
/// not descended into. For the per-scope breakdown across the whole
/// hierarchy, use [`compute_entropy_hierarchical`].
pub fn compute_entropy(ir: &Ir, scope: DeclId) -> StructuralEntropy {
    let mut vertices: Vec<DeclId> = Vec::new();
    let mut edges: Vec<DeclId> = Vec::new();

    for child in scope_children(ir, scope) {
        match ir.decl_kind(child) {
            DeclKind::Node => vertices.push(child),
            DeclKind::Edge => edges.push(child),
            DeclKind::HyperArc => {}
        }
    }

    entropy_from_scope(ir, &vertices, &edges)
}

/// Compute structural entropy at every scope in the IR that has at
/// least one `Edge` direct child. Returns `(scope_did, entropy)` pairs
/// in ascending `DeclId` order. The module root (decls with
/// `parent.is_none()`) is emitted as `DeclId::NONE` first, and only
/// when it has at least one edge at top level.
pub fn compute_entropy_hierarchical(ir: &Ir) -> Vec<(DeclId, StructuralEntropy)> {
    let mut out: Vec<(DeclId, StructuralEntropy)> = Vec::new();

    let root = compute_entropy(ir, DeclId::NONE);
    if root.n_edges > 0 {
        out.push((DeclId::NONE, root));
    }

    for idx in 0..ir.decl_nodes.len() {
        let did = DeclId::new(idx);
        if ir.decl_kind(did) != DeclKind::Node {
            continue;
        }
        if ir.first_child(did).is_none() {
            continue;
        }
        let entropy = compute_entropy(ir, did);
        if entropy.n_edges > 0 {
            out.push((did, entropy));
        }
    }

    out
}

fn entropy_from_scope(ir: &Ir, vertices: &[DeclId], edges: &[DeclId]) -> StructuralEntropy {
    if vertices.is_empty() && edges.is_empty() {
        return StructuralEntropy::ZERO;
    }

    let (h_arity, h_sign, degree_by_vertex) = arity_sign_and_degrees(ir, edges);
    let h_degree = degree_entropy(vertices, &degree_by_vertex);
    let h_total = (h_arity + h_sign + h_degree) / 3.0;

    StructuralEntropy {
        h_arity,
        h_sign,
        h_degree,
        h_total,
        n_vertices: vertices.len(),
        n_edges: edges.len(),
    }
}

/// Single-pass accumulator: arity histogram + per-edge sign-role counts
/// + degree-by-target map. Returns `(H_arity, H_sign, deg)`.
fn arity_sign_and_degrees(
    ir: &Ir,
    edges: &[DeclId],
) -> (f64, f64, HashMap<DeclId, usize>) {
    if edges.is_empty() {
        return (0.0, 0.0, HashMap::new());
    }

    // BTreeMap so the value-iteration order (used as the summation
    // order inside `shannon_entropy_from_counts`) is deterministic.
    // HashMap's randomized iteration order shows up as last-ULP drift
    // when two compiles of the same IR are compared across processes.
    let mut arity_hist: BTreeMap<usize, usize> = BTreeMap::new();
    let mut deg: HashMap<DeclId, usize> = HashMap::new();
    let mut sign_entropy_sum = 0.0;

    for &e_did in edges {
        let Some(eid) = ir.as_edge(e_did) else { continue };
        let edge = &ir.edges[eid.0];

        let mut arity: usize = 0;
        let mut n_plus: usize = 0;
        let mut n_minus: usize = 0;
        let mut n_neutral: usize = 0;

        for &aid in &edge.arcs {
            let arc = &ir.arcs[aid.0];
            for r in &arc.refs {
                arity += 1;
                match r {
                    SignedRefR::Plus(_) => n_plus += 1,
                    SignedRefR::Minus(_) => n_minus += 1,
                    SignedRefR::Neutral(_) => n_neutral += 1,
                }
                let tgt = r.target();
                if tgt.is_some() {
                    *deg.entry(tgt).or_insert(0) += 1;
                }
            }
        }

        *arity_hist.entry(arity).or_insert(0) += 1;
        sign_entropy_sum += sign_entropy(n_plus, n_minus, n_neutral);
    }

    let h_arity = shannon_entropy_from_counts(arity_hist.values().copied(), edges.len());
    let h_sign = sign_entropy_sum / edges.len() as f64;

    (h_arity, h_sign, deg)
}

fn degree_entropy(vertices: &[DeclId], deg: &HashMap<DeclId, usize>) -> f64 {
    if vertices.is_empty() {
        return 0.0;
    }
    // BTreeMap for deterministic value iteration (see comment in
    // `arity_sign_and_degrees`).
    let mut deg_hist: BTreeMap<usize, usize> = BTreeMap::new();
    for v in vertices {
        let d = deg.get(v).copied().unwrap_or(0);
        *deg_hist.entry(d).or_insert(0) += 1;
    }
    shannon_entropy_from_counts(deg_hist.values().copied(), vertices.len())
}

/// `H = - Σ p_i ln p_i` over a distribution given as raw integer counts
/// with known total. Takes `0 · ln 0 = 0` everywhere.
fn shannon_entropy_from_counts<I>(counts: I, total: usize) -> f64
where
    I: IntoIterator<Item = usize>,
{
    if total == 0 {
        return 0.0;
    }
    let n = total as f64;
    let mut h = 0.0;
    for c in counts {
        if c == 0 {
            continue;
        }
        let p = c as f64 / n;
        h -= p * p.ln();
    }
    h
}

/// Entropy of the three-role sign distribution `{+, -, ~}` for one edge.
fn sign_entropy(n_plus: usize, n_minus: usize, n_neutral: usize) -> f64 {
    let total = n_plus + n_minus + n_neutral;
    shannon_entropy_from_counts([n_plus, n_minus, n_neutral], total)
}

/// Iterate direct children of `scope`. When `scope` is `DeclId::NONE`,
/// yields all top-level decls (those with `parent.is_none()`); this
/// scan is linear in `decl_nodes.len()` and is invoked once per
/// `compute_entropy_hierarchical` call. For concrete scopes, uses the
/// O(k) sibling-list walk.
fn scope_children<'a>(ir: &'a Ir, scope: DeclId) -> Box<dyn Iterator<Item = DeclId> + 'a> {
    if scope.is_none() {
        Box::new(
            (0..ir.decl_nodes.len())
                .map(DeclId::new)
                .filter(move |&d| ir.decl_nodes[d.0].parent.is_none()),
        )
    } else {
        Box::new(ir.decl_children(scope))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hymeko::common::ids::{EdgeId, HyperArcId, NodeId, SymId};
    use hymeko::ir::ir::{AnnoR, ArcRec, DeclNode, EdgeRec, NodeRec, RefAtomR};

    fn empty_ir() -> Ir {
        Ir::default()
    }

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

        // Chain into parent's child list (append).
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
        let e_did = push_decl(ir, parent, DeclKind::Edge);
        let eid = EdgeId::new(ir.edges.len());
        ir.edges.push(EdgeRec::new(e_did, Vec::new()));
        ir.decl_to_edge[e_did.0] = Some(eid);

        // One arc per edge, carrying all signed refs — matches the
        // `lower_arc` shape (an Edge contains HyperArc children each
        // holding a `Vec<SignedRefR>`).
        let arc_did = push_decl(ir, e_did, DeclKind::HyperArc);
        let aid = HyperArcId::new(ir.arcs.len());
        let refs = incidences
            .iter()
            .map(|&(sign, target)| {
                let atom = RefAtomR { target, anno: AnnoR::default(), weights: None };
                match sign {
                    1 => SignedRefR::Plus(atom),
                    -1 => SignedRefR::Minus(atom),
                    _ => SignedRefR::Neutral(atom),
                }
            })
            .collect();
        ir.arcs.push(ArcRec { anno: AnnoR::default(), in_edge: e_did, refs });
        ir.decl_to_arc[arc_did.0] = Some(aid);
        ir.edges[eid.0].arcs.push(aid);
        e_did
    }

    #[test]
    fn empty_ir_returns_zero() {
        let ir = empty_ir();
        let e = compute_entropy(&ir, DeclId::NONE);
        assert_eq!(e, StructuralEntropy::ZERO);
        assert!(compute_entropy_hierarchical(&ir).is_empty());
    }

    #[test]
    fn single_vertex_no_edges_is_zero() {
        let mut ir = empty_ir();
        let root = push_node(&mut ir, DeclId::NONE);
        push_node(&mut ir, root); // one child vertex, no edges

        let e = compute_entropy(&ir, root);
        assert_eq!(e.n_vertices, 1);
        assert_eq!(e.n_edges, 0);
        assert_eq!(e.h_arity, 0.0);
        assert_eq!(e.h_sign, 0.0);
        assert_eq!(e.h_degree, 0.0);
        assert_eq!(e.h_total, 0.0);

        // Hierarchical view: no edges anywhere → empty.
        assert!(compute_entropy_hierarchical(&ir).is_empty());
    }

    #[test]
    fn single_edge_all_plus_has_zero_sign_entropy() {
        // One scope, 3 vertices, 1 edge with three `+` incidences.
        let mut ir = empty_ir();
        let root = push_node(&mut ir, DeclId::NONE);
        let v0 = push_node(&mut ir, root);
        let v1 = push_node(&mut ir, root);
        let v2 = push_node(&mut ir, root);
        push_edge(&mut ir, root, &[(1, v0), (1, v1), (1, v2)]);

        let e = compute_entropy(&ir, root);
        assert_eq!(e.n_vertices, 3);
        assert_eq!(e.n_edges, 1);
        assert_eq!(e.h_arity, 0.0); // single edge → degenerate arity dist.
        assert_eq!(e.h_sign, 0.0);  // uniform sign → no entropy.
        // Every vertex has degree 1 → degenerate degree dist. → 0.
        assert_eq!(e.h_degree, 0.0);
        assert_eq!(e.h_total, 0.0);
    }

    #[test]
    fn three_way_mixed_sign_edge_is_ln_three() {
        // One edge with a single `+`, `-`, `~` incidence each:
        //   p_+ = p_- = p_0 = 1/3 → H = ln 3 ≈ 1.0986.
        let mut ir = empty_ir();
        let root = push_node(&mut ir, DeclId::NONE);
        let v0 = push_node(&mut ir, root);
        let v1 = push_node(&mut ir, root);
        let v2 = push_node(&mut ir, root);
        push_edge(&mut ir, root, &[(1, v0), (-1, v1), (0, v2)]);

        let e = compute_entropy(&ir, root);
        assert!(
            (e.h_sign - 3f64.ln()).abs() < 1e-12,
            "expected H_sign ≈ ln 3, got {}",
            e.h_sign
        );
    }

    #[test]
    fn outer_dataflow_mirrors_hand_calc() {
        // Reproduces the "Module scope" numbers from
        // docs/structural_entropy_ir.md on a minimal hand-built IR:
        // V={x, h, y, layer_0, layer_1}, E={flow_0, flow_1}
        // with (+ x, ~ layer_0, - h) and (+ h, ~ layer_1, - y).
        let mut ir = empty_ir();
        let root = push_node(&mut ir, DeclId::NONE);
        let x = push_node(&mut ir, root);
        let h = push_node(&mut ir, root);
        let y = push_node(&mut ir, root);
        let l0 = push_node(&mut ir, root);
        let l1 = push_node(&mut ir, root);
        push_edge(&mut ir, root, &[(1, x), (0, l0), (-1, h)]);
        push_edge(&mut ir, root, &[(1, h), (0, l1), (-1, y)]);

        let e = compute_entropy(&ir, root);
        assert_eq!(e.n_vertices, 5);
        assert_eq!(e.n_edges, 2);
        assert!(e.h_arity.abs() < 1e-12, "H_arity should be 0, got {}", e.h_arity);
        let expected_h_sign = 3f64.ln();
        assert!(
            (e.h_sign - expected_h_sign).abs() < 1e-12,
            "H_sign expected ln 3 ≈ {}, got {}",
            expected_h_sign,
            e.h_sign
        );
        // deg(x)=1, deg(h)=2, deg(y)=1, deg(l0)=1, deg(l1)=1
        // P_deg = {1: 4/5, 2: 1/5}; H = -(0.8 ln 0.8 + 0.2 ln 0.2)
        let expected_h_deg = -(0.8_f64 * 0.8_f64.ln() + 0.2_f64 * 0.2_f64.ln());
        assert!(
            (e.h_degree - expected_h_deg).abs() < 1e-12,
            "H_degree expected {}, got {}",
            expected_h_deg,
            e.h_degree
        );
        let expected_total = (0.0 + expected_h_sign + expected_h_deg) / 3.0;
        assert!(
            (e.h_total - expected_total).abs() < 1e-12,
            "H_total expected {}, got {}",
            expected_total,
            e.h_total
        );
    }

    #[test]
    fn hierarchical_visits_nested_scope_exactly_once() {
        // Two nested scopes each carrying a hyperedge; the outer edge
        // involves only the outer hypervertex, the inner edge only
        // inner vertices. The hierarchical walk must surface both, in
        // ascending DeclId order, with a single entry per scope.
        let mut ir = empty_ir();
        let outer = push_node(&mut ir, DeclId::NONE);
        let neighbour = push_node(&mut ir, DeclId::NONE);
        push_edge(&mut ir, DeclId::NONE, &[(1, outer), (-1, neighbour)]);

        let inner_v0 = push_node(&mut ir, outer);
        let inner_v1 = push_node(&mut ir, outer);
        push_edge(&mut ir, outer, &[(1, inner_v0), (-1, inner_v1)]);

        let scopes = compute_entropy_hierarchical(&ir);
        assert_eq!(scopes.len(), 2, "expected root + outer scope, got {scopes:?}");
        assert!(scopes[0].0.is_none(), "first scope should be the module root");
        assert_eq!(scopes[1].0, outer, "second scope should be the outer hypervertex");
        assert_eq!(scopes[0].1.n_edges, 1);
        assert_eq!(scopes[1].1.n_edges, 1);
    }
}
