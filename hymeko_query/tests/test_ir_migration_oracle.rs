//! Unified-IR M2b **pre-migration oracles** for `hymeko_query` and the
//! `hymeko_formats` wire DTO.
//!
//! APPROVED-TRANSFORMATION-TEST-EDIT: unified-ir-m2b-audit-oracles.
//!
//! * **Reader proof** — the production query paths (`compute_entropy_hierarchical`,
//!   `propose_split_for_highest_h_sign`, `emit_split_rewrite`) take `&Ir` and
//!   leave every arena length unchanged. This is the runtime half of the
//!   writer-list finding: in `hymeko_query`, *no production path writes the IR*
//!   (the `push_*` helpers all live in `#[cfg(test)]` modules).
//! * **Wire DTO freeze** — `snapshot_json` is byte-frozen against a committed
//!   fixture, and `ArcDto::target_id` is proven to be the raw `DeclId.0`. Both
//!   are the exact surface a stable/generational-ID migration would perturb, so
//!   they must be locked *before* it.

use std::path::PathBuf;

use hymeko::ir::ir::Ir;
use hymeko::resolution::string_table::StringTable;

use crate::test_helpers::load_and_lower;

const EDGE_FIXTURE: &str =
    "../data/minimal_examples/testing_edges/minimal_test_tensor_values_2nodes_1_edge.hymeko";

fn wire_fixture_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/wire/minimal_edge_snapshot.json")
}

fn arena_lengths(ir: &Ir) -> (usize, usize, usize, usize) {
    (
        ir.decl_nodes.len(),
        ir.nodes.len(),
        ir.edges.len(),
        ir.arcs.len(),
    )
}

fn normalize(s: &str) -> String {
    s.replace("\r\n", "\n").trim_end().to_string()
}

// -- reader proof ------------------------------------------------------------

#[test]
fn production_query_paths_do_not_mutate_the_ir() {
    let (store, compiled) = load_and_lower(EDGE_FIXTURE).expect("fixture compiles");
    let ir = &compiled.ir;
    let before = arena_lengths(ir);

    // Metric path.
    let _ = hymeko_query::compute_entropy_hierarchical(ir);

    // Rewrite path: propose (read) then emit source text (read).
    if let Some(proposal) = hymeko_query::rewrite::propose_split_for_highest_h_sign(ir) {
        let src = hymeko_query::rewrite::emit_split_rewrite(ir, &store.it, &proposal, "probe");
        assert!(!src.is_empty(), "rewrite emits source text");
    }

    assert_eq!(
        arena_lengths(ir),
        before,
        "a production query path mutated the canonical IR"
    );
}

// -- wire DTO freeze ---------------------------------------------------------

#[test]
fn wire_snapshot_json_is_frozen() {
    let (store, compiled) = load_and_lower(EDGE_FIXTURE).expect("fixture compiles");
    let st = StringTable::from_interner(&store.it);
    let json = hymeko_formats::snapshot::snapshot_json(&compiled.ir, &st).expect("json encodes");

    let frozen = std::fs::read_to_string(wire_fixture_path())
        .expect("wire fixture present — run `regenerate_wire_fixture` with --ignored to create it");
    assert_eq!(
        normalize(&json),
        normalize(&frozen),
        "snapshot_json wire shape drifted from the frozen fixture"
    );
}

#[test]
fn wire_snapshot_json_is_deterministic() {
    let (s1, c1) = load_and_lower(EDGE_FIXTURE).expect("compiles");
    let (s2, c2) = load_and_lower(EDGE_FIXTURE).expect("compiles");
    let j1 = hymeko_formats::snapshot::snapshot_json(&c1.ir, &StringTable::from_interner(&s1.it))
        .expect("json");
    let j2 = hymeko_formats::snapshot::snapshot_json(&c2.ir, &StringTable::from_interner(&s2.it))
        .expect("json");
    assert_eq!(j1, j2, "two compiles must yield byte-identical wire JSON");
}

#[test]
fn arc_dto_target_id_is_the_raw_declid() {
    // Locks the semantics `ArcDto::target_id == DeclId.0` (and `NodeDto::id`).
    // A stable/generational-ID migration changes exactly this; the test is its
    // tripwire.
    let (store, compiled) = load_and_lower(EDGE_FIXTURE).expect("compiles");
    let st = StringTable::from_interner(&store.it);
    let snap = hymeko_formats::snapshot::snapshot(&compiled.ir, &st);

    let mut checked = 0usize;
    for node in snap.nodes.iter().chain(snap.edges.iter()) {
        // NodeDto::id indexes the decl arena directly.
        assert_eq!(
            store.it.resolve(compiled.ir.decl_nodes[node.id].name),
            node.name,
            "NodeDto::id must be the raw DeclId index"
        );
        for arc in &node.arcs {
            assert!(
                arc.target_id < compiled.ir.decl_nodes.len(),
                "target_id in range"
            );
            assert_eq!(
                store.it.resolve(compiled.ir.decl_nodes[arc.target_id].name),
                arc.target_name,
                "ArcDto::target_id must index the named decl"
            );
            checked += 1;
        }
    }
    assert!(checked >= 1, "fixture must exercise at least one arc");
}

/// (Re)generate the frozen wire fixture. `#[ignore]` — a generator, not a PASS
/// test. Run: `cargo test -p hymeko_query --test integration -- --ignored
/// regenerate_wire_fixture`.
#[test]
#[ignore = "run with --ignored to (re)write tests/data/wire/minimal_edge_snapshot.json"]
fn regenerate_wire_fixture() {
    let (store, compiled) = load_and_lower(EDGE_FIXTURE).expect("compiles");
    let st = StringTable::from_interner(&store.it);
    let json = hymeko_formats::snapshot::snapshot_json(&compiled.ir, &st).expect("json");
    let path = wire_fixture_path();
    std::fs::create_dir_all(path.parent().unwrap()).expect("mkdir");
    std::fs::write(&path, json).expect("write fixture");
    eprintln!("wrote {}", path.display());
}
