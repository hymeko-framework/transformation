//! Integration test: structural entropy on the real compiled IR of
//! `data/nn/simple_net.hymeko`. Cross-checks the hand-calculated
//! numbers in `docs/structural_entropy_ir.md` against what the
//! `hymeko_query::entropy` walk actually produces on the output of
//! `ModuleStore::compile`.
//!
//! If either the design-note numbers or the walk shifts, this test
//! breaks — that is intended. The two are the spec ↔ implementation
//! pair for step 1 of the 5-step entropy hot-swap plan.

#[cfg(test)]
mod test_entropy_on_simple_net {
    use hymeko::common::ids::DeclId;
    use hymeko::ir::ir::DeclKind;
    use hymeko_query::{compute_entropy, compute_entropy_hierarchical};

    use crate::test_helpers::{find_decl, load_and_lower};

    const SIMPLE_NET: &str = "../data/nn/simple_net.hymeko";
    const TOL: f64 = 1e-9;

    fn approx(actual: f64, expected: f64, label: &str) {
        assert!(
            (actual - expected).abs() < TOL,
            "{label}: expected {expected}, got {actual} (|Δ| = {})",
            (actual - expected).abs()
        );
    }

    #[test]
    fn outer_scope_matches_design_note() {
        let (store, compiled) = load_and_lower(SIMPLE_NET).expect("compile simple_net");
        let simple_net = find_decl(&compiled.ir, &store.it, "simple_net", DeclKind::Node);

        let e = compute_entropy(&compiled.ir, simple_net);

        // V = {x, h, y, layer_0, layer_1}, E = {flow_0, flow_1}
        assert_eq!(e.n_vertices, 5, "V(simple_net) should be 5 hypervertices");
        assert_eq!(e.n_edges, 2, "E(simple_net) should be 2 dataflow edges");

        // Both edges arity 3 → H_arity = 0.
        approx(e.h_arity, 0.0, "H_arity");
        // Both edges have signs (+, -, ~) uniform → H_sign(e) = ln 3.
        approx(e.h_sign, 3f64.ln(), "H_sign");
        // deg distribution {1: 4/5, 2: 1/5}
        let expected_h_deg = -(0.8_f64 * 0.8_f64.ln() + 0.2_f64 * 0.2_f64.ln());
        approx(e.h_degree, expected_h_deg, "H_degree");
        approx(
            e.h_total,
            (0.0 + 3f64.ln() + expected_h_deg) / 3.0,
            "H_total",
        );
    }

    #[test]
    fn layer_0_scope_matches_design_note() {
        let (store, compiled) = load_and_lower(SIMPLE_NET).expect("compile simple_net");
        let layer_0 = find_decl(&compiled.ir, &store.it, "layer_0", DeclKind::Node);

        let e = compute_entropy(&compiled.ir, layer_0);

        // V = 3 input ports + 5 hidden neurons + 5 output ports
        //     + 3 attribute decls from `kernel: + <isa> ggk.bspline
        //       { degree 3; n_knots 8; }` (the kernel isa spec plus
        //       its two `field value;` statements each lower to Node
        //       decls) = 16. Matches docs/structural_entropy_ir.md.
        assert_eq!(e.n_vertices, 16, "V(layer_0) — see design note");
        assert_eq!(e.n_edges, 5, "E(layer_0) should be the 5 factor edges");
        // Every edge is arity 5 with (+, +, +, -, ~) → H_arity = 0.
        approx(e.h_arity, 0.0, "H_arity");
        // Per-edge sign dist: p_+ = 3/5, p_- = 1/5, p_0 = 1/5.
        let expected_h_sign = -((3.0_f64 / 5.0) * (3.0_f64 / 5.0).ln()
            + (1.0_f64 / 5.0) * (1.0_f64 / 5.0).ln()
            + (1.0_f64 / 5.0) * (1.0_f64 / 5.0).ln());
        approx(e.h_sign, expected_h_sign, "H_sign");

        // Degree distribution sanity: at least one vertex has degree 5
        // (the input ports appear in every factor).
        assert!(
            e.h_degree > 0.0,
            "H_degree should be positive (mixed 1-vs-5 degrees), got {}",
            e.h_degree
        );
    }

    #[test]
    fn hierarchical_walk_covers_expected_scopes() {
        let (store, compiled) = load_and_lower(SIMPLE_NET).expect("compile simple_net");
        let scopes = compute_entropy_hierarchical(&compiled.ir);

        // No module-root scope: the file has no decls with
        // parent == NONE that are Edges (only Node decls at top).
        assert!(
            scopes.iter().all(|(did, _)| !did.is_none()),
            "no module-root scope expected for simple_net (root has no top-level edges); got {:?}",
            scopes.iter().map(|(d, _)| d.raw()).collect::<Vec<_>>()
        );

        // Ascending DeclId order.
        for w in scopes.windows(2) {
            let (a, b) = (w[0].0, w[1].0);
            assert!(
                a.raw() < b.raw(),
                "scopes must be emitted in ascending DeclId"
            );
        }

        // The three user-authored scopes must all appear with the
        // expected (V, E) shape. Additional scopes may come from
        // imported library decls (meta_nn.hymeko type specs); we don't
        // pin those, only the ones written in simple_net.hymeko.
        let simple_net = find_decl(&compiled.ir, &store.it, "simple_net", DeclKind::Node);
        let layer_0 = find_decl(&compiled.ir, &store.it, "layer_0", DeclKind::Node);
        let layer_1 = find_decl(&compiled.ir, &store.it, "layer_1", DeclKind::Node);

        let by_did: std::collections::HashMap<DeclId, _> =
            scopes.iter().map(|(d, e)| (*d, *e)).collect();

        let s = by_did
            .get(&simple_net)
            .expect("simple_net scope should appear in hierarchical walk");
        assert_eq!(s.n_edges, 2, "simple_net scope edges");

        let l0 = by_did.get(&layer_0).expect("layer_0 scope should appear");
        assert_eq!(l0.n_edges, 5, "layer_0 factor count");

        let l1 = by_did.get(&layer_1).expect("layer_1 scope should appear");
        assert_eq!(l1.n_edges, 2, "layer_1 factor count");
    }

    #[test]
    fn determinism_two_compiles_match_bit_for_bit() {
        // Proposition-2 sanity: the same source compiled twice yields
        // identical entropy numbers (not just float-approximate, but
        // bit-identical — the IR is content-addressable and the walk
        // is order-deterministic).
        let (_store_a, compiled_a) = load_and_lower(SIMPLE_NET).expect("compile a");
        let (_store_b, compiled_b) = load_and_lower(SIMPLE_NET).expect("compile b");

        let a = compute_entropy(&compiled_a.ir, DeclId::NONE);
        let b = compute_entropy(&compiled_b.ir, DeclId::NONE);
        assert_eq!(a, b, "root-scope entropy drifted across recompile");

        let scopes_a = compute_entropy_hierarchical(&compiled_a.ir);
        let scopes_b = compute_entropy_hierarchical(&compiled_b.ir);
        assert_eq!(scopes_a.len(), scopes_b.len());
        for ((did_a, ent_a), (did_b, ent_b)) in scopes_a.iter().zip(scopes_b.iter()) {
            assert_eq!(did_a, did_b);
            assert_eq!(ent_a, ent_b);
        }
    }
}
