//! Unified-IR M2b **pre-migration oracles** for `hymeko_emitter`.
//!
//! APPROVED-TRANSFORMATION-TEST-EDIT: unified-ir-m2b-audit-oracles.
//!
//! These tests lock the *current, pre-migration* behaviour of the emitter crate
//! against the `15223bc` canonical-IR accessor API, so the later mechanical
//! migration of raw `ir.<arena>[id.0]` reads to accessors is provably
//! behaviour-preserving. Nothing here changes production source.
//!
//! Four kinds of oracle:
//! 1. **Writer-list proof** (runtime corroboration of the grep finding):
//!    `bridge::to_compiler_ir` is the sole production *writer*; the four M2T
//!    emitters take `&Ir` and leave every arena length unchanged.
//! 2. **Accessor equivalence** on a well-formed IR: the total/checked accessors
//!    (`kind_of`, `parent_of`, `node_for_decl`, `edge_for_decl`) agree with the
//!    raw arena projections the migration will replace — for *every* decl.
//! 3. **Checked-composite value**: a corrupted `decl -> record` back-link is
//!    rejected by `node_for_decl` (returns `None`) where the raw
//!    `decl_to_node[..]` projection would hand back a silently-wrong record.
//! 4. **Frozen output fingerprints** (characterization goldens) + editor-IR
//!    **defect counterexamples** (`#[ignore]`, un-ignore when the named unified
//!    feature lands — never counted as PASS).

use std::path::Path;
use std::sync::Arc;

use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{DeclKind, Ir};
use hymeko::module_store::module_store::{
    CompiledProgram, HymekoParser, ModuleLoadError, ModuleStore,
};
use hymeko::module_store::source_provider::{MemProvider, StdFsProvider};
use hymeko::resolution::interner::Interner;
use parser::ast::AstStr;

use hymeko_emitter::bridge::{from_compiler_ir, to_compiler_ir};
use hymeko_emitter::editor_ir::{HyMeKoEditorIR, HyperEdge, IRDelta, Position, Sign, Vertex};
use hymeko_emitter::{emit_hymeko, emit_lean4, emit_rust_stubs, emit_sysml};

// -- compile-pipeline harness (test-only; matches the crate convention) ------

struct LalrpopParser;
impl HymekoParser for LalrpopParser {
    fn parse<'a>(&self, src: &'a str) -> Result<AstStr<'a>, String> {
        parser::parse_description(src).map_err(|e| format!("{e:?}"))
    }
}

fn load_and_lower(
    path: impl AsRef<Path>,
) -> Result<
    (
        ModuleStore<StdFsProvider, LalrpopParser>,
        Arc<CompiledProgram>,
    ),
    ModuleLoadError,
> {
    let mut store = ModuleStore::new(StdFsProvider::new(), LalrpopParser);
    let compiled = store.compile(path.as_ref())?;
    Ok((store, compiled))
}

const MINI_ARM: &str = "../data/robotics/robots/mini_arm.hymeko";

/// FNV-1a-64 — stable across Rust versions (never `DefaultHasher`, which is not
/// portable and not stable across toolchains).
fn fnv1a_64(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// A tiny editor IR: two vertices, one signed hyperedge. Self-contained — no
/// fixture, no parser.
fn tiny_editor() -> HyMeKoEditorIR {
    let mut editor = HyMeKoEditorIR::new();
    let a = editor.vertices.insert(Vertex {
        name: "a".into(),
        level: 0,
        attributes: vec![],
        position: None,
    });
    let b = editor.vertices.insert(Vertex {
        name: "b".into(),
        level: 0,
        attributes: vec![],
        position: None,
    });
    editor.hyperedges.insert(HyperEdge {
        name: "e".into(),
        incident: vec![(a, Sign::Plus), (b, Sign::Minus)],
        weight: 1.0,
        patch_id: None,
    });
    editor
}

fn arena_lengths(ir: &Ir) -> (usize, usize, usize, usize) {
    (
        ir.decl_nodes.len(),
        ir.nodes.len(),
        ir.edges.len(),
        ir.arcs.len(),
    )
}

// -- 1. writer-list proof (runtime corroboration) ----------------------------

#[test]
fn bridge_writes_and_the_four_emitters_only_read() {
    let mut interner = Interner::new();
    let ir = to_compiler_ir(&tiny_editor(), &mut interner);

    // `to_compiler_ir` is a writer: it populated the arenas.
    let before = arena_lengths(&ir);
    assert!(
        before.0 >= 3,
        "decls populated (2 nodes + 1 edge + 1 arc = 4)"
    );
    assert_eq!(before.1, 2, "two node records");
    assert_eq!(before.2, 1, "one edge record");
    assert_eq!(before.3, 1, "one arc record");

    // The four emitters are readers: none may mutate any arena.
    let _h = emit_hymeko(&ir, &interner, "probe");
    let _s = emit_sysml(&ir, &interner, "probe");
    let _r = emit_rust_stubs(&ir, &interner);
    let _l = emit_lean4(&ir, &interner);
    assert_eq!(arena_lengths(&ir), before, "an emitter mutated an arena");
}

// -- 2. accessor equivalence on a well-formed IR -----------------------------

#[test]
fn accessor_equivalence_holds_for_every_decl_on_a_real_ir() {
    let (store, compiled) = load_and_lower(MINI_ARM).expect("mini_arm compiles");
    let ir = &compiled.ir;
    let _ = &store; // keep the interner alive for symmetry with other tests

    assert_eq!(
        ir.decl_count(),
        ir.decl_nodes.len(),
        "decl_count == arena len"
    );

    for (did, decl) in ir.decls() {
        // kind_of == raw decl_nodes[did.0].kind
        assert_eq!(
            ir.kind_of(did),
            Some(decl.kind),
            "kind_of vs raw kind @ {did:?}"
        );

        // parent_of == raw parent (None when NONE)
        let raw_parent = ir.decl_nodes[did.0].parent;
        let expect_parent = raw_parent.is_some().then_some(raw_parent);
        assert_eq!(
            ir.parent_of(did),
            expect_parent,
            "parent_of vs raw parent @ {did:?}"
        );

        match decl.kind {
            DeclKind::Node => {
                let nid = ir.decl_to_node[did.0].expect("node mapping present");
                let (a_nid, a_rec) = ir.node_for_decl(did).expect("node_for_decl present");
                assert_eq!(a_nid, nid, "node id @ {did:?}");
                assert_eq!(a_rec.decl, did, "node back-link @ {did:?}");
                assert_eq!(ir.as_node(did), Some(nid), "as_node @ {did:?}");
            }
            DeclKind::Edge => {
                let eid = ir.decl_to_edge[did.0].expect("edge mapping present");
                let (a_eid, a_rec) = ir.edge_for_decl(did).expect("edge_for_decl present");
                assert_eq!(a_eid, eid, "edge id @ {did:?}");
                assert_eq!(a_rec.decl, did, "edge back-link @ {did:?}");
                assert_eq!(ir.as_edge(did), Some(eid), "as_edge @ {did:?}");
            }
            DeclKind::HyperArc => {
                assert_eq!(ir.as_edge(did), None, "an arc is not an edge @ {did:?}");
            }
        }
    }
}

// -- 3. checked-composite catches a corrupted back-link ----------------------

#[test]
fn node_for_decl_rejects_a_corrupted_backlink_where_raw_would_mislead() {
    let mut interner = Interner::new();
    let mut ir = to_compiler_ir(&tiny_editor(), &mut interner);

    // Find the first Node decl.
    let node_did = ir
        .decls()
        .find(|(_, d)| d.kind == DeclKind::Node)
        .map(|(id, _)| id)
        .expect("a node decl exists");
    let nid = ir.as_node(node_did).expect("node mapping");

    // Before corruption: both raw and checked-composite agree.
    assert_eq!(ir.node_for_decl(node_did).map(|(n, _)| n), Some(nid));

    // Inject a fault: point the record's back-link at a *different* decl.
    let wrong_did = DeclId::new(ir.decl_nodes.len() + 999);
    ir.nodes[nid.0].decl = wrong_did;

    // Raw projection is now a silently-wrong answer: the mapping still resolves,
    // the record is still there — it just describes the wrong declaration.
    assert!(
        ir.decl_to_node[node_did.0].is_some(),
        "raw mapping still present"
    );

    // The checked-composite accessor refuses it (back-link mismatch -> None).
    assert!(
        ir.node_for_decl(node_did).is_none(),
        "node_for_decl must reject a mismatched back-link"
    );
}

// -- characterization: bridge-built IR layout --------------------------------

#[test]
fn bridge_built_ir_has_flat_parent_hierarchy() {
    // Documented contract in bridge.rs: the editor IR does not track containment,
    // so every decl has parent == NONE. Locking this so the migration (and a
    // future containment-carrying editor IR) has a tripwire.
    let mut interner = Interner::new();
    let ir = to_compiler_ir(&tiny_editor(), &mut interner);
    for (did, _) in ir.decls() {
        assert_eq!(ir.parent_of(did), None, "bridge output is flat @ {did:?}");
    }
}

#[test]
fn bridge_makes_exactly_one_arc_per_edge() {
    // `to_compiler_ir` emits one ArcRec per hyperedge (the editor's "one atomic
    // hyperedge per named edge" model). Characterized so a migration that
    // changes arc cardinality is caught.
    let mut interner = Interner::new();
    let ir = to_compiler_ir(&tiny_editor(), &mut interner);
    assert_eq!(ir.edges.len(), 1);
    assert_eq!(ir.edges[0].arcs.len(), 1, "exactly one arc for the edge");
    assert_eq!(ir.arcs.len(), 1);
}

// -- frozen output fingerprints (characterization goldens) -------------------

#[test]
fn emitter_output_fingerprints_are_frozen() {
    let (store, compiled) = load_and_lower(MINI_ARM).expect("mini_arm compiles");
    let ir = &compiled.ir;
    let it = &store.it;

    // Frozen FNV-1a-64 of each emitter's output on mini_arm at core 15223bc.
    // A migration that changes any emitter output by a byte flips one of these.
    assert_eq!(
        fnv1a_64(emit_hymeko(ir, it, "mini_arm").as_bytes()),
        FP_HYMEKO,
        "emit_hymeko"
    );
    assert_eq!(
        fnv1a_64(emit_sysml(ir, it, "mini_arm").as_bytes()),
        FP_SYSML,
        "emit_sysml"
    );
    assert_eq!(
        fnv1a_64(emit_rust_stubs(ir, it).as_bytes()),
        FP_RUST,
        "emit_rust_stubs"
    );
    assert_eq!(
        fnv1a_64(emit_lean4(ir, it).as_bytes()),
        FP_LEAN,
        "emit_lean4"
    );
}

#[test]
fn fnv_known_answer_vector_is_stable() {
    // Frozen KAT: a mutation of `fnv1a_64` is caught here, so the fingerprint
    // oracle above cannot silently drift with the hash implementation.
    assert_eq!(fnv1a_64(b""), 0xcbf2_9ce4_8422_2325);
    assert_eq!(fnv1a_64(b"hymeko"), FP_KAT_HYMEKO);
}

// Frozen constants — harvested from `dump_oracle_constants` at core 15223bc.
const FP_HYMEKO: u64 = 0xe0bce0b5fac08061;
const FP_SYSML: u64 = 0xe52b4791df3c183d;
const FP_RUST: u64 = 0xbfcd6bfecd8bb267;
const FP_LEAN: u64 = 0xa5a456794b5bdb45;
const FP_KAT_HYMEKO: u64 = 0x5060c8138dfeaf9a;

/// Print the frozen constants. `#[ignore]` — a generator, not a PASS test.
#[test]
#[ignore = "run with --ignored to (re)harvest the frozen fingerprint constants"]
fn dump_oracle_constants() {
    let (store, compiled) = load_and_lower(MINI_ARM).expect("mini_arm compiles");
    let ir = &compiled.ir;
    let it = &store.it;
    println!(
        "FP_HYMEKO = {:#018x}",
        fnv1a_64(emit_hymeko(ir, it, "mini_arm").as_bytes())
    );
    println!(
        "FP_SYSML  = {:#018x}",
        fnv1a_64(emit_sysml(ir, it, "mini_arm").as_bytes())
    );
    println!(
        "FP_RUST   = {:#018x}",
        fnv1a_64(emit_rust_stubs(ir, it).as_bytes())
    );
    println!(
        "FP_LEAN   = {:#018x}",
        fnv1a_64(emit_lean4(ir, it).as_bytes())
    );
    println!("FP_KAT_HYMEKO = {:#018x}", fnv1a_64(b"hymeko"));
}

// -- editor-IR defect counterexamples (#[ignore]: un-ignore when fixed) ------

#[test]
#[ignore = "DEFECT: IRDelta::Batch doc claims 'succeeds-or-fails as one unit' but apply() is non-atomic (partial apply persists). Un-ignore when the unified-IR transaction API makes Batch atomic. See docs/unified-ir-construction-seam.md."]
fn batch_delta_should_be_atomic() {
    let mut ir = HyMeKoEditorIR::new();
    let bogus = slotmap::KeyData::from_ffi(0xDEAD_BEEF_u64).into();
    let _ = ir.apply(IRDelta::Batch {
        deltas: vec![
            IRDelta::AddVertex {
                data: Vertex {
                    name: "ok".into(),
                    level: 0,
                    attributes: vec![],
                    position: None,
                },
            },
            IRDelta::RemoveVertex { key: bogus }, // fails here
            IRDelta::AddVertex {
                data: Vertex {
                    name: "never".into(),
                    level: 0,
                    attributes: vec![],
                    position: None,
                },
            },
        ],
    });
    // Desired (atomic) end-state: a failed batch leaves the IR untouched.
    assert_eq!(
        ir.vertices.len(),
        0,
        "atomic batch must roll back the first AddVertex"
    );
}

#[test]
#[ignore = "LIMITATION: editor layout metadata (position, level, weight) has no arena-side home at 15223bc, so it is dropped on the editor->arena->editor round-trip. Un-ignore when the canonical IR carries editor attributes."]
fn editor_layout_metadata_survives_roundtrip() {
    let mut editor = HyMeKoEditorIR::new();
    let a = editor.vertices.insert(Vertex {
        name: "a".into(),
        level: 3,
        attributes: vec![],
        position: Some(Position { x: 12.0, y: 34.0 }),
    });
    let b = editor.vertices.insert(Vertex {
        name: "b".into(),
        level: 0,
        attributes: vec![],
        position: None,
    });
    editor.hyperedges.insert(HyperEdge {
        name: "e".into(),
        incident: vec![(a, Sign::Plus), (b, Sign::Minus)],
        weight: 7.5,
        patch_id: None,
    });

    let mut interner = Interner::new();
    let arena = to_compiler_ir(&editor, &mut interner);
    let back = from_compiler_ir(&arena, &interner);

    let va = back
        .vertices
        .values()
        .find(|v| v.name == "a")
        .expect("vertex a");
    assert_eq!(
        va.position,
        Some(Position { x: 12.0, y: 34.0 }),
        "position dropped"
    );
    assert_eq!(va.level, 3, "level dropped");
    let e = back.hyperedges.values().next().expect("edge");
    assert_eq!(e.weight, 7.5, "weight dropped");
}

// -- HyMeKo source is the resident (faithful) serialization of the IR --------

/// The sorted per-edge signed-incidence signature: for each edge, the sorted
/// multiset of `(sign, target_decl_name)` over all its arcs' refs. Name-based,
/// so it is invariant under re-indexing across a re-parse.
fn edge_signatures(ir: &Ir, it: &Interner) -> Vec<Vec<(i8, String)>> {
    let mut sigs: Vec<Vec<(i8, String)>> = Vec::new();
    for (did, decl) in ir.decls() {
        if decl.kind != DeclKind::Edge {
            continue;
        }
        let (_eid, rec) = ir.edge_for_decl(did).expect("edge record");
        let mut sig: Vec<(i8, String)> = Vec::new();
        for &aid in &rec.arcs {
            let arc = ir.arc(aid).expect("arc record");
            for r in &arc.refs {
                let tgt = r.target();
                if tgt.is_none() {
                    continue;
                }
                let name = it.resolve(ir.decl_nodes[tgt.0].name).to_string();
                sig.push((r.sign(), name));
            }
        }
        sig.sort();
        sigs.push(sig);
    }
    sigs.sort();
    sigs
}

#[test]
fn hymeko_source_is_the_resident_serialization() {
    // A self-contained fixture (no external imports), so the emitted source is
    // fully re-parsable in memory. This proves HyMeKo text is the faithful
    // round-trip serialization of the canonical IR: source -> Ir -> emit_hymeko
    // -> re-parse -> Ir', structurally equal. The framework's own format is the
    // resident serialization; JSON/DOT/URDF are lossy views by comparison.
    const FIXTURE: &str =
        "../data/minimal_examples/testing_edges/minimal_test_tensor_values_2nodes_1_edge.hymeko";
    let (store1, compiled1) = load_and_lower(FIXTURE).expect("fixture compiles");
    let ir1 = &compiled1.ir;

    let src2 = emit_hymeko(ir1, &store1.it, "minimal_test_2nodes_1_edge");

    // Re-parse the emitted HyMeKo entirely in memory.
    let mut provider = MemProvider::default();
    provider.insert_file(std::path::PathBuf::from("roundtrip.hymeko"), src2.clone());
    let mut store2 = ModuleStore::new(provider, LalrpopParser);
    let compiled2 = store2
        .compile(Path::new("roundtrip.hymeko"))
        .unwrap_or_else(|e| panic!("emitted HyMeKo must re-parse, got {e:?}\n---\n{src2}\n---"));
    let ir2 = &compiled2.ir;

    // Structural equivalence (name-based, index-independent).
    assert_eq!(ir2.nodes.len(), ir1.nodes.len(), "node count preserved");
    assert_eq!(ir2.edges.len(), ir1.edges.len(), "edge count preserved");
    assert_eq!(ir2.arcs.len(), ir1.arcs.len(), "arc count preserved");
    assert_eq!(
        edge_signatures(ir2, &store2.it),
        edge_signatures(ir1, &store1.it),
        "signed-incidence structure preserved across the HyMeKo round-trip"
    );
}
