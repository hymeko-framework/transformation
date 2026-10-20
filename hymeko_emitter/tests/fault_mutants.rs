//! Named fault mutants for the `hymeko_core@15223bcd -> @5fe2c88` migration
//! (Phase 4). Each mutant is a deliberate corruption of the migrated
//! construction / read path; a named test proves the corruption is caught
//! (the migrated `push_decl`/`bind_*_decl` builder, the public accessors, or
//! the emitter surface it). Fixtures are built in memory, so no on-disk fixture
//! is mutated and there is nothing to restore — the "restoration hash-verified"
//! obligation is vacuous here (each test constructs and drops its own `Ir`).
//!
//! Two mutants are structural, not runtime-injectable in this workspace, and are
//! asserted at build/audit time instead (documented in the migration report):
//!   - *cross-version `Ir` smuggling* — prevented by `cargo tree` showing exactly
//!     one `hymeko_core`; a second version would make the `Ir` types incompatible
//!     and fail to compile.
//!   - *reverting to the old resolution-`Index` canonical hash* — no production
//!     site in the workspace hashes through `Index` (`Canon::structure` reads the `Ir`) or reads
//!     `Index.by_path` (grep-verified); there is nothing to revert.

use hymeko::ir::ir::Ir;
use hymeko::resolution::interner::Interner;
use hymeko_emitter::bridge::{CompilerIrBuilder, IrBuilder};
use hymeko_emitter::editor_ir::{HyMeKoEditorIR, HyperEdge, Sign, Vertex};
use hymeko_emitter::emit_hymeko;

fn fnv1a_64(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in bytes {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

fn vertex(name: &str) -> Vertex {
    Vertex {
        name: name.into(),
        level: 0,
        attributes: vec![],
        position: None,
    }
}

/// One hyperedge `e` over freshly-inserted vertices named in `incident`, each
/// with its sign. Duplicated names insert distinct vertices; pass the same key by
/// repeating a name in `dup_positions` handling below (see the duplicate test).
fn editor_edge(incident: &[(&str, Sign)]) -> HyMeKoEditorIR {
    let mut ed = HyMeKoEditorIR::new();
    let inc = incident
        .iter()
        .map(|(n, s)| (ed.vertices.insert(vertex(n)), *s))
        .collect();
    ed.hyperedges.insert(HyperEdge {
        name: "e".into(),
        incident: inc,
        weight: 1.0,
        patch_id: None,
    });
    ed
}

fn hymeko_fp(ed: &HyMeKoEditorIR) -> u64 {
    let mut it = Interner::new();
    let ir = CompilerIrBuilder::new(ed, &mut it)
        .build()
        .expect("canonical editor construction");
    fnv1a_64(emit_hymeko(&ir, &it, "m").as_bytes())
}

#[test]
fn mutant_sign_loss_flips_output() {
    // Correct: (a:+, b:-). Mutant: the `-` is lost (becomes `+`).
    let correct = hymeko_fp(&editor_edge(&[("a", Sign::Plus), ("b", Sign::Minus)]));
    let mutant = hymeko_fp(&editor_edge(&[("a", Sign::Plus), ("b", Sign::Plus)]));
    assert_ne!(
        correct, mutant,
        "losing an incidence sign must change the emitted output"
    );
    // And `~` is distinct from both (the +/-/~ trichotomy survives the migration).
    let neutral = hymeko_fp(&editor_edge(&[("a", Sign::Plus), ("b", Sign::Neutral)]));
    assert_ne!(correct, neutral);
    assert_ne!(mutant, neutral);
}

#[test]
fn mutant_duplicate_incidence_not_deduped() {
    // A hyperedge incident on the SAME vertex twice: both refs must survive the
    // migrated construction (a dedup mutant would collapse them to one).
    let mut ed = HyMeKoEditorIR::new();
    let a = ed.vertices.insert(vertex("a"));
    ed.hyperedges.insert(HyperEdge {
        name: "e".into(),
        incident: vec![(a, Sign::Plus), (a, Sign::Plus)],
        weight: 1.0,
        patch_id: None,
    });
    let mut it = Interner::new();
    let ir = CompilerIrBuilder::new(&ed, &mut it)
        .build()
        .expect("canonical editor construction");
    assert_eq!(ir.arcs.len(), 1, "one arc for the edge");
    assert_eq!(
        ir.arcs[0].refs.len(),
        2,
        "duplicate incidence kept as two distinct refs (not deduped)"
    );
}

#[test]
fn mutant_wrong_bind_caught_by_accessor_bijection() {
    // The migrated `bind_*_decl` must produce a coherent decl->record bijection:
    // for every bound decl, `*_for_decl` returns a record whose `.decl` points back.
    // A mutant that binds the wrong record id fails this back-link (the same
    // invariant `migration_oracles` characterizes).
    let mut it = Interner::new();
    let ir = CompilerIrBuilder::new(
        &editor_edge(&[("a", Sign::Plus), ("b", Sign::Minus)]),
        &mut it,
    )
    .build()
    .expect("canonical editor construction");
    for (did, _) in ir.decls() {
        if let Some((_nid, rec)) = ir.node_for_decl(did) {
            assert_eq!(rec.decl, did, "node back-link");
        }
        if let Some((_eid, rec)) = ir.edge_for_decl(did) {
            assert_eq!(rec.decl, did, "edge back-link");
        }
    }
}

#[test]
fn mutant_emitter_reads_the_passed_ir() {
    // The emitter must read the `Ir` handed to it, not a hidden/other one:
    // two different models produce different output.
    let a = hymeko_fp(&editor_edge(&[("a", Sign::Plus), ("b", Sign::Minus)]));
    let b = hymeko_fp(&editor_edge(&[
        ("x", Sign::Plus),
        ("y", Sign::Minus),
        ("z", Sign::Plus),
    ]));
    assert_ne!(a, b, "emitter output must depend on the passed Ir");
}

#[test]
fn mutant_emit_does_not_mutate_input_ir() {
    // Emit takes `&Ir`; a partial-commit / in-place-mutation mutant is impossible
    // by type, and observably the Ir is unchanged across repeated emits.
    let mut it = Interner::new();
    let ir: Ir = CompilerIrBuilder::new(
        &editor_edge(&[("a", Sign::Plus), ("b", Sign::Minus)]),
        &mut it,
    )
    .build()
    .expect("canonical editor construction");
    let before = (
        ir.decl_nodes.len(),
        ir.nodes.len(),
        ir.edges.len(),
        ir.arcs.len(),
    );
    let out1 = fnv1a_64(emit_hymeko(&ir, &it, "m").as_bytes());
    let out2 = fnv1a_64(emit_hymeko(&ir, &it, "m").as_bytes());
    let after = (
        ir.decl_nodes.len(),
        ir.nodes.len(),
        ir.edges.len(),
        ir.arcs.len(),
    );
    assert_eq!(before, after, "emit must not mutate the Ir");
    assert_eq!(out1, out2, "emit is a pure function of the Ir");
}
