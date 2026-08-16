//! Bridge round-trip property tests.
//!
//! Checks that `from_compiler_ir(to_compiler_ir(editor))` matches the
//! source editor IR under the structural-equality contract documented in
//! `docs/plans/06_wasm_editor/step1_ir_design.md` § "Risks".
//!
//! Also verifies the reverse direction: an arena IR produced by the full
//! `ModuleStore` pipeline (loading `data/robotics/mini_arm.hymeko`) can
//! be projected into an editor IR, then projected back, yielding a new
//! arena IR with the same node/edge counts.

use std::collections::BTreeSet;
use std::path::Path;
use std::sync::Arc;

use hymeko::module_store::module_store::{
    CompiledProgram, HymekoParser, ModuleLoadError, ModuleStore,
};
use hymeko::module_store::source_provider::StdFsProvider;
use hymeko::resolution::interner::Interner;
use parser::ast::AstStr;

use hymeko_emitter::bridge::{from_compiler_ir, to_compiler_ir};
use hymeko_emitter::editor_ir::{HyMeKoEditorIR, HyperEdge, Sign, Vertex};

// ---- mini compile-pipeline helper (private) -----------------------------

struct LalrpopParser;
impl HymekoParser for LalrpopParser {
    fn parse<'a>(&self, src: &'a str) -> Result<AstStr<'a>, String> {
        parser::parse_description(src).map_err(|e| format!("{e:?}"))
    }
}
fn load_and_lower(
    path: impl AsRef<Path>,
) -> Result<
    (ModuleStore<StdFsProvider, LalrpopParser>, Arc<CompiledProgram>),
    ModuleLoadError,
> {
    let fs = StdFsProvider::new();
    let mut store = ModuleStore::new(fs, LalrpopParser);
    let c = store.compile(path.as_ref())?;
    Ok((store, c))
}

// ---- structural extractors ----------------------------------------------

/// A canonical, slotmap-independent description of an editor IR: sorted
/// vertex names and sorted edge signatures.
#[derive(Debug, PartialEq, Eq)]
struct Structural {
    vertex_names: Vec<String>,
    /// Each tuple is `(edge_name, sorted_incident_descriptor)` where the
    /// incident descriptor is a sorted list of `"{sign}{vertex_name}"`.
    edge_signatures: Vec<(String, Vec<String>)>,
}

fn sign_glyph(s: Sign) -> char {
    match s {
        Sign::Plus => '+',
        Sign::Minus => '-',
        Sign::Neutral => '~',
    }
}

fn structural(editor: &HyMeKoEditorIR) -> Structural {
    let mut vertex_names: Vec<String> =
        editor.vertices.values().map(|v| v.name.clone()).collect();
    vertex_names.sort();

    let key_to_name: std::collections::BTreeMap<_, _> = editor
        .vertices
        .iter()
        .map(|(k, v)| (k, v.name.clone()))
        .collect();

    let mut edge_signatures: Vec<(String, Vec<String>)> = editor
        .hyperedges
        .values()
        .map(|e| {
            let mut parts: Vec<String> = e
                .incident
                .iter()
                .map(|(vk, s)| {
                    let name = key_to_name
                        .get(vk)
                        .cloned()
                        .unwrap_or_else(|| "<missing>".into());
                    format!("{}{}", sign_glyph(*s), name)
                })
                .collect();
            parts.sort();
            (e.name.clone(), parts)
        })
        .collect();
    edge_signatures.sort();
    Structural {
        vertex_names,
        edge_signatures,
    }
}

// ---- round-trip tests ---------------------------------------------------

#[test]
fn editor_roundtrips_through_compiler_ir() {
    let mut editor = HyMeKoEditorIR::new();
    let base = editor.vertices.insert(Vertex {
        name: "base_link".into(),
        level: 0,
        attributes: vec![],
        position: None,
    });
    let spinner = editor.vertices.insert(Vertex {
        name: "spinner".into(),
        level: 0,
        attributes: vec![],
        position: None,
    });
    let axis = editor.vertices.insert(Vertex {
        name: "AXIS_Z".into(),
        level: 0,
        attributes: vec![],
        position: None,
    });
    editor.hyperedges.insert(HyperEdge {
        name: "spin_joint".into(),
        incident: vec![(base, Sign::Plus), (spinner, Sign::Minus), (axis, Sign::Minus)],
        weight: 1.0,
        patch_id: None,
    });

    let mut interner = Interner::new();
    let arena = to_compiler_ir(&editor, &mut interner);
    let editor_back = from_compiler_ir(&arena, &interner);

    assert_eq!(structural(&editor_back), structural(&editor));
}

#[test]
fn empty_editor_roundtrips_to_empty_arena() {
    let editor = HyMeKoEditorIR::new();
    let mut interner = Interner::new();
    let arena = to_compiler_ir(&editor, &mut interner);
    assert_eq!(arena.nodes.len(), 0);
    assert_eq!(arena.edges.len(), 0);
    assert_eq!(arena.arcs.len(), 0);

    let back = from_compiler_ir(&arena, &interner);
    assert_eq!(structural(&back), structural(&editor));
}

#[test]
fn editor_with_multiple_edges_preserves_per_edge_signs() {
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
    // Edge 1: a→b with +/-
    editor.hyperedges.insert(HyperEdge {
        name: "e1".into(),
        incident: vec![(a, Sign::Plus), (b, Sign::Minus)],
        weight: 1.0,
        patch_id: None,
    });
    // Edge 2: a↔b with neutral/neutral
    editor.hyperedges.insert(HyperEdge {
        name: "e2".into(),
        incident: vec![(a, Sign::Neutral), (b, Sign::Neutral)],
        weight: 1.0,
        patch_id: None,
    });

    let mut interner = Interner::new();
    let arena = to_compiler_ir(&editor, &mut interner);
    let back = from_compiler_ir(&arena, &interner);

    // Every edge should still have exactly 2 incidents with the same sign mix.
    assert_eq!(back.hyperedges.len(), 2);
    for he in back.hyperedges.values() {
        assert_eq!(he.incident.len(), 2);
    }
    assert_eq!(structural(&back), structural(&editor));
}

#[test]
fn mini_arm_compile_ir_projects_into_editor_and_back() {
    // Start from a real `.hymeko` fixture so the arena IR has all the
    // quirks of a lowered source (imports, type namespaces, etc.). The
    // bridge should still produce an editor IR that preserves at least
    // the robot's visible nodes and the spin_joint edge.
    let (store, compiled) =
        load_and_lower("../data/robotics/mini_arm.hymeko").expect("compile");
    let editor = from_compiler_ir(&compiled.ir, &store.it);

    let names: BTreeSet<&str> = editor
        .vertices
        .values()
        .map(|v| v.name.as_str())
        .collect();
    assert!(names.contains("base_link"), "missing base_link");
    assert!(names.contains("spinner"), "missing spinner");

    let edge_names: BTreeSet<&str> = editor
        .hyperedges
        .values()
        .map(|e| e.name.as_str())
        .collect();
    assert!(edge_names.contains("spin_joint"), "missing spin_joint edge");
}

#[test]
fn arena_editor_arena_roundtrip_preserves_node_and_edge_counts() {
    let (store, compiled) =
        load_and_lower("../data/robotics/mini_arm.hymeko").expect("compile");
    let n_nodes = compiled.ir.nodes.len();
    let n_edges = compiled.ir.edges.len();

    let editor = from_compiler_ir(&compiled.ir, &store.it);

    // Now project back. We intern against a fresh interner — the names
    // will re-intern; since `to_compiler_ir` uses the `interner` param
    // mutably, we need a mut binding.
    let mut fresh_interner = Interner::new();
    let arena2 = to_compiler_ir(&editor, &mut fresh_interner);

    assert_eq!(arena2.nodes.len(), n_nodes);
    assert_eq!(arena2.edges.len(), n_edges);
}

#[test]
fn incident_vertices_that_are_edges_in_arena_are_filtered_on_reverse() {
    // Craft a synthetic arena that has an arc pointing to an Edge decl.
    // The editor IR can't express edge-to-edge refs (its `incident` is
    // `Vec<(VertexKey, Sign)>`), so the bridge's reverse pass must skip
    // such refs rather than insert bogus entries.
    use hymeko::common::ids::{DeclId, EdgeId, HyperArcId, NodeId, SymId};
    use hymeko::ir::ir::{
        AnnoR, ArcRec, DeclKind, DeclNode, EdgeRec, Ir, NodeRec, RefAtomR, SignedRefR,
    };

    let mut interner = Interner::new();
    let mut ir = Ir::default();

    // Push one Node + one Edge + one HyperArc.
    let node_sym = interner.intern("a");
    let edge1_sym = interner.intern("e1");
    let edge2_sym = interner.intern("e2_refs_e1");

    let fresh = |kind, name: SymId| DeclNode {
        kind,
        name,
        parent: DeclId::NONE,
        first_child: DeclId::NONE,
        last_child: DeclId::NONE,
        next_sibling: DeclId::NONE,
        anno: AnnoR::default(),
    };

    let push = |ir: &mut Ir, d: DeclNode| {
        let id = DeclId::new(ir.decl_nodes.len());
        ir.decl_nodes.push(d);
        ir.decl_to_node.push(None);
        ir.decl_to_edge.push(None);
        ir.decl_to_arc.push(None);
        ir.decl_hash.push(None);
        id
    };

    let node_did = push(&mut ir, fresh(DeclKind::Node, node_sym));
    ir.nodes.push(NodeRec::new(node_did, Vec::new()));
    ir.decl_to_node[node_did.0] = Some(NodeId::new(0));

    let edge1_did = push(&mut ir, fresh(DeclKind::Edge, edge1_sym));
    ir.edges.push(EdgeRec::new(edge1_did, Vec::new()));
    ir.decl_to_edge[edge1_did.0] = Some(EdgeId::new(0));

    let edge2_did = push(&mut ir, fresh(DeclKind::Edge, edge2_sym));
    let mut e2 = EdgeRec::new(edge2_did, Vec::new());

    // Arc references `a` (node, allowed) and `e1` (edge, should be filtered).
    let arc_did = push(&mut ir, fresh(DeclKind::HyperArc, SymId::new(0)));
    let atom = |t| RefAtomR {
        target: t,
        anno: AnnoR::default(),
        weights: None,
    };
    ir.arcs.push(ArcRec {
        anno: AnnoR::default(),
        in_edge: edge2_did,
        refs: vec![SignedRefR::Plus(atom(node_did)), SignedRefR::Plus(atom(edge1_did))],
    });
    let arc_id = HyperArcId::new(0);
    ir.decl_to_arc[arc_did.0] = Some(arc_id);
    e2.arcs.push(arc_id);
    ir.edges.push(e2);
    ir.decl_to_edge[edge2_did.0] = Some(EdgeId::new(1));

    let editor = from_compiler_ir(&ir, &interner);
    // Two edges on the arena side; each projects into the editor.
    assert_eq!(editor.hyperedges.len(), 2);
    // The second edge's arc refs should be filtered to just the node `a`.
    let e2_editor = editor
        .hyperedges
        .values()
        .find(|e| e.name == "e2_refs_e1")
        .unwrap();
    assert_eq!(e2_editor.incident.len(), 1, "edge→edge ref not filtered");
}
