//! Bridge between [`editor_ir::HyMeKoEditorIR`] (slotmap, mutation-first)
//! and `hymeko_core::ir::Ir` (arena, compile-time).
//!
//! See `docs/plans/06_wasm_editor/step1_ir_design.md` for the dual-IR
//! rationale. In short: the editor IR is optimized for small atomic
//! mutations (drag a vertex on the canvas, toggle a sign), while the
//! arena IR is the compilation target (canonical hashing, tensor
//! projection, URDF/SDF emission). The bridge lets the WASM editor hand
//! off to the full compiler for rendering / codegen and load existing
//! `.hymeko` sources back into canvas-editable form.
//!
//! # Contract
//!
//! Structural round-trip for the **editor-visible subset**:
//!
//! ```text
//! from_compiler_ir(to_compiler_ir(editor)) ≡structural editor
//! ```
//!
//! where `≡structural` means:
//! - same set of vertex names
//! - same set of hyperedge names
//! - same connectivity (per-edge multiset of `(vertex_name, sign)`)
//!
//! Slotmap keys are *not* preserved across the round-trip — the reverse
//! walk allocates fresh keys. The editor-only fields `Vertex.level`,
//! `Vertex.attributes`, `Vertex.position`, and `HyperEdge.patch_id` are
//! *not* round-trippable through the arena IR today (they have no
//! arena-side representation yet); the bridge sets them to defaults on
//! the reverse path.

use std::collections::BTreeMap;

use hymeko::common::ids::{DeclId, EdgeId, HyperArcId, NodeId, SymId};
use hymeko::ir::ir::{
    AnnoR, ArcRec, DeclKind, DeclNode, EdgeRec, Ir, NodeRec, RefAtomR, SignedRefR,
};
use hymeko::resolution::interner::Interner;

use crate::editor_ir::{EdgeKey, HyMeKoEditorIR, HyperEdge, Sign, Vertex, VertexKey};

// ---------------------------------------------------------------------------
// editor → arena
// ---------------------------------------------------------------------------

fn fresh_decl_node(kind: DeclKind, name: SymId) -> DeclNode {
    DeclNode {
        kind,
        name,
        parent: DeclId::NONE,
        first_child: DeclId::NONE,
        last_child: DeclId::NONE,
        next_sibling: DeclId::NONE,
        anno: AnnoR::default(),
    }
}

fn push_decl(ir: &mut Ir, decl: DeclNode) -> DeclId {
    let id = DeclId::new(ir.decl_nodes.len());
    ir.decl_nodes.push(decl);
    ir.decl_to_node.push(None);
    ir.decl_to_edge.push(None);
    ir.decl_to_arc.push(None);
    ir.decl_hash.push(None);
    id
}

fn signed_ref_for(target: DeclId, sign: Sign) -> SignedRefR {
    let atom = RefAtomR {
        target,
        anno: AnnoR::default(),
        weights: None,
    };
    match sign {
        Sign::Plus => SignedRefR::Plus(atom),
        Sign::Minus => SignedRefR::Minus(atom),
        Sign::Neutral => SignedRefR::Neutral(atom),
    }
}

/// Convert an editor IR snapshot into a fresh compile-time arena `Ir`.
///
/// Layout produced:
/// - One `Ir::decl_nodes` entry per vertex (kind `Node`) and per
///   hyperedge (kind `Edge`), plus one anonymous entry per arc (kind
///   `HyperArc`).
/// - `Ir::nodes`, `Ir::edges`, `Ir::arcs` populated in editor iteration
///   order.
/// - All `decl_nodes[i].parent == DeclId::NONE` (flat layout — editor IR
///   does not track containment yet).
///
/// Editor-only fields (level, attributes, position, patch_id) are
/// dropped. They have no arena-side home today; once the arena IR grows
/// value-attribute support beyond scalar `AnnoR.value`, this function
/// will thread them through.
pub fn to_compiler_ir(editor: &HyMeKoEditorIR, interner: &mut Interner) -> Ir {
    let mut ir = Ir::default();

    // Vertices → decl_nodes[kind=Node] + NodeRec.
    let mut vertex_to_decl: BTreeMap<VertexKey, DeclId> = BTreeMap::new();
    for (vkey, v) in editor.vertices.iter() {
        let sym = interner.intern(&v.name);
        let did = push_decl(&mut ir, fresh_decl_node(DeclKind::Node, sym));

        let nid = NodeId::new(ir.nodes.len());
        ir.nodes.push(NodeRec::new(did, Vec::new()));
        ir.decl_to_node[did.0] = Some(nid);

        vertex_to_decl.insert(vkey, did);
    }

    // Hyperedges → decl_nodes[kind=Edge] + EdgeRec + one ArcRec per edge.
    for (_ekey, he) in editor.hyperedges.iter() {
        let sym = interner.intern(&he.name);
        let edge_did = push_decl(&mut ir, fresh_decl_node(DeclKind::Edge, sym));

        let eid = EdgeId::new(ir.edges.len());
        let mut edge_rec = EdgeRec::new(edge_did, Vec::new());

        // Build the arc.
        let arc_did = push_decl(&mut ir, fresh_decl_node(DeclKind::HyperArc, SymId::new(0)));
        let refs: Vec<SignedRefR> = he
            .incident
            .iter()
            .filter_map(|(vkey, sign)| {
                vertex_to_decl.get(vkey).map(|&tgt| signed_ref_for(tgt, *sign))
            })
            .collect();
        let arc_id = HyperArcId::new(ir.arcs.len());
        ir.arcs.push(ArcRec {
            anno: AnnoR::default(),
            in_edge: edge_did,
            refs,
        });
        ir.decl_to_arc[arc_did.0] = Some(arc_id);
        edge_rec.arcs.push(arc_id);

        ir.edges.push(edge_rec);
        ir.decl_to_edge[edge_did.0] = Some(eid);
    }

    ir
}

// ---------------------------------------------------------------------------
// arena → editor
// ---------------------------------------------------------------------------

fn sign_from_ref(r: &SignedRefR) -> Sign {
    match r {
        SignedRefR::Plus(_) => Sign::Plus,
        SignedRefR::Minus(_) => Sign::Minus,
        SignedRefR::Neutral(_) => Sign::Neutral,
    }
}

/// Walk a compile-time arena `Ir` and populate a fresh editor IR.
///
/// Each `NodeRec` becomes an editor `Vertex` with default editor
/// metadata (level 0, no attributes, no layout position). Each
/// `EdgeRec`'s arcs are flattened into a single editor `HyperEdge` with
/// the concatenated `(VertexKey, Sign)` list — this matches the editor's
/// "one atomic hyperedge per named edge" model.
///
/// References that don't resolve to a `Node` (e.g. edge-to-edge refs
/// from the rewrite engine) are skipped, matching the editor's
/// vertex-centric surface.
pub fn from_compiler_ir(ir: &Ir, interner: &Interner) -> HyMeKoEditorIR {
    let mut editor = HyMeKoEditorIR::new();
    let mut node_decl_to_vkey: BTreeMap<DeclId, VertexKey> = BTreeMap::new();

    for node in &ir.nodes {
        let name = interner.resolve(ir.decl_nodes[node.decl.0].name).to_string();
        let vkey = editor.vertices.insert(Vertex {
            name,
            level: 0,
            attributes: Vec::new(),
            position: None,
        });
        node_decl_to_vkey.insert(node.decl, vkey);
    }

    for edge in &ir.edges {
        let name = interner.resolve(ir.decl_nodes[edge.decl.0].name).to_string();
        let mut incident: Vec<(VertexKey, Sign)> = Vec::new();

        for &arc_id in &edge.arcs {
            let arc = &ir.arcs[arc_id.0];
            for r in &arc.refs {
                if let Some(&vkey) = node_decl_to_vkey.get(&r.target()) {
                    incident.push((vkey, sign_from_ref(r)));
                }
            }
        }

        let _ekey: EdgeKey = editor.hyperedges.insert(HyperEdge {
            name,
            incident,
            weight: 1.0,
            patch_id: None,
        });
    }

    editor
}
