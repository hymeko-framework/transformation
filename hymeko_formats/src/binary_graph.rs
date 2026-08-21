//! Binary-graph flattening of a signed hypergraph + a mock in-process emitter.
//!
//! Used by the SMC paper's in-process-vs-subprocess generation ablation
//! (`hymeko_bench --bin bench_ablation`). The signed hypergraph carries
//! hyperedge identity (one signed entry per incidence); flattening it to a
//! classical *binary* graph clique-expands every hyperedge of arity `k` into
//! `C(k, 2)` pairwise edges and discards that identity. [`BinaryGraph::mock_emit`]
//! is a representative in-process code generator over that flattened form — it
//! does the per-edge formatting work a real binary-graph emitter would, but has
//! no hyperedge structure to exploit, so it is a conservative lower bound on the
//! cost of generating from a binary representation.
//!
//! The clique-expansion arithmetic mirrors `binary_vs_hypergraph.rs` (the
//! representational-cost bench); lifting it here lets both share one definition.

/// One pairwise edge of the clique-expanded binary graph.
///
/// `sign` is carried from the originating hyperedge so the mock emitter does
/// polarity-dependent per-edge work (the binary *encoding* drops hyperedge
/// identity, not necessarily edge sign).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BinaryEdge {
    pub u: usize,
    pub v: usize,
    pub sign: i8,
}

/// A classical signed binary graph obtained by clique-expanding a hypergraph.
#[derive(Debug, Clone, Default)]
pub struct BinaryGraph {
    pub n_verts: usize,
    pub edges: Vec<BinaryEdge>,
}

impl BinaryGraph {
    /// Clique-expand `members` (per-hyperedge vertex-index lists) into pairwise
    /// edges, each tagged with its hyperedge's `signs` entry.
    ///
    /// # Preconditions
    /// - `members.len() == signs.len()`.
    /// - every index in every `members[e]` is `< n_verts`.
    ///
    /// # Postconditions
    /// - `edges.len() == sum_e C(members[e].len(), 2)` (== [`clique_edge_count`]).
    ///
    /// [`clique_edge_count`]: BinaryGraph::clique_edge_count
    pub fn from_hyperedges(n_verts: usize, members: &[Vec<usize>], signs: &[i8]) -> Self {
        debug_assert_eq!(
            members.len(),
            signs.len(),
            "members/signs length mismatch ({} vs {})",
            members.len(),
            signs.len()
        );
        let mut edges = Vec::with_capacity(Self::clique_edge_count(members));
        for (m, &s) in members.iter().zip(signs) {
            for i in 0..m.len() {
                for j in (i + 1)..m.len() {
                    debug_assert!(
                        m[i] < n_verts && m[j] < n_verts,
                        "member index out of range (n_verts={n_verts})"
                    );
                    edges.push(BinaryEdge {
                        u: m[i],
                        v: m[j],
                        sign: s,
                    });
                }
            }
        }
        Self { n_verts, edges }
    }

    /// `sum_e C(k_e, 2)` — the pairwise edge count of the clique expansion.
    /// Pure arithmetic; the same formula `binary_vs_hypergraph.rs` uses.
    pub fn clique_edge_count(members: &[Vec<usize>]) -> usize {
        members
            .iter()
            .map(|m| {
                let k = m.len();
                k * k.saturating_sub(1) / 2
            })
            .sum()
    }

    /// Mock in-process emitter: one textual record per edge.
    ///
    /// Reproduces the dominant cost of real template rendering (per-output-unit
    /// string formatting) without any hyperedge structure to amortise over.
    ///
    /// # Postconditions
    /// - output length is monotone non-decreasing in `self.edges.len()`.
    pub fn mock_emit(&self) -> String {
        let mut out = String::with_capacity(self.edges.len() * 24 + 32);
        out.push_str("# binary-graph emission (mock)\n");
        for (i, e) in self.edges.iter().enumerate() {
            let pol = if e.sign >= 0 { '+' } else { '-' };
            out.push_str(&format!("edge {i}: v{} {pol} v{}\n", e.u, e.v));
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn triangle_clique_has_three_edges() {
        let bg = BinaryGraph::from_hyperedges(3, &[vec![0, 1, 2]], &[1]);
        assert_eq!(bg.edges.len(), 3);
        assert_eq!(BinaryGraph::clique_edge_count(&[vec![0, 1, 2]]), 3);
    }

    #[test]
    fn arity_four_clique_has_six_edges() {
        let members = vec![vec![0, 1, 2, 3]];
        let bg = BinaryGraph::from_hyperedges(4, &members, &[-1]);
        assert_eq!(bg.edges.len(), 6);
        assert!(bg.edges.iter().all(|e| e.sign == -1));
    }

    #[test]
    fn arity_below_two_yields_no_edges() {
        // A unary (k=1) or nullary hyperedge contributes nothing to a binary graph.
        let members = vec![vec![0], vec![]];
        let bg = BinaryGraph::from_hyperedges(2, &members, &[1, 1]);
        assert_eq!(bg.edges.len(), 0);
        assert_eq!(bg.mock_emit().lines().count(), 1); // header only
    }

    #[test]
    fn mock_emit_length_monotone_in_edge_count() {
        let small = BinaryGraph::from_hyperedges(3, &[vec![0, 1, 2]], &[1]); // 3 edges
        let big = BinaryGraph::from_hyperedges(4, &[vec![0, 1, 2, 3]], &[1]); // 6 edges
        assert!(big.mock_emit().len() > small.mock_emit().len());
    }

    #[test]
    #[should_panic(expected = "member index out of range")]
    fn member_index_out_of_range_panics_in_debug() {
        // Precondition violation: index 5 >= n_verts 3.
        let _ = BinaryGraph::from_hyperedges(3, &[vec![0, 5]], &[1]);
    }

    #[test]
    #[should_panic(expected = "members/signs length mismatch")]
    fn members_signs_length_mismatch_panics_in_debug() {
        let _ = BinaryGraph::from_hyperedges(3, &[vec![0, 1], vec![1, 2]], &[1]);
    }
}
