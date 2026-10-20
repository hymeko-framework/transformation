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
//! EditorIrBuilder(CompilerIrBuilder(editor).build()).build() ≡structural editor
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

use crate::editor_ir::{EdgeKey, HyMeKoEditorIR, HyperEdge, Sign, Vertex, VertexKey};
use hymeko::common::ids::DeclId;
pub use hymeko::ir::ir::IrBuilder;
use hymeko::ir::ir::{
    AnnoR, ArcPlacement, IncidenceSign, Ir, IrCapacities, IrConstruction, RefAtomR, ReferenceDraft,
    SignedRefR, dispatch,
};
use hymeko::resolution::interner::Interner;
use std::collections::BTreeMap;

/// Editor input adapter for Core's sole checked construction owner.
/// Named entities and anonymous arc declarations retain the editor's flat
/// hierarchy. Arc ownership is represented independently by the owning edge.
/// Editor-only layout/attributes remain outside the compiler-visible subset.
pub struct CompilerIrBuilder<'a> {
    editor: &'a HyMeKoEditorIR,
    interner: &'a mut Interner,
}
impl<'a> CompilerIrBuilder<'a> {
    pub fn new(editor: &'a HyMeKoEditorIR, interner: &'a mut Interner) -> Self {
        Self { editor, interner }
    }
    fn reference(target: DeclId, sign: Sign) -> ReferenceDraft {
        let atom = RefAtomR::plain(target);
        match sign {
            Sign::Plus => ReferenceDraft::new(IncidenceSign::Plus, atom),
            Sign::Minus => ReferenceDraft::new(IncidenceSign::Minus, atom),
            Sign::Neutral => ReferenceDraft::new(IncidenceSign::Neutral, atom),
        }
    }
}
impl IrBuilder for CompilerIrBuilder<'_> {
    type Output = Ir;
    type Error = dispatch::DispatchError;
    fn capacities(&self) -> IrCapacities {
        let nodes = self.editor.vertices.len();
        let edges = self.editor.hyperedges.len();
        IrCapacities {
            declarations: nodes + 2 * edges,
            nodes,
            edges,
            arcs: edges,
        }
    }
    fn populate(&mut self, construction: &mut IrConstruction<'_>) -> Result<(), Self::Error> {
        let mut vertex_to_decl = BTreeMap::new();
        for (key, vertex) in &self.editor.vertices {
            let name = self.interner.intern(&vertex.name);
            let id = construction.append_node(name, AnnoR::default(), Vec::new())?;
            vertex_to_decl.insert(key, id);
        }
        for (_, edge) in &self.editor.hyperedges {
            let name = self.interner.intern(&edge.name);
            let owner = construction.append_edge(name, AnnoR::default(), Vec::new())?;
            let references = edge
                .incident
                .iter()
                .filter_map(|(key, sign)| {
                    vertex_to_decl
                        .get(key)
                        .map(|&target| Self::reference(target, *sign))
                })
                .collect();
            construction.append_arc(
                ArcPlacement {
                    owner,
                    parent: DeclId::NONE,
                },
                AnnoR::default(),
                references,
            )?;
        }
        Ok(())
    }
    fn finish(self, ir: Ir) -> Ir {
        ir
    }
}

/// Read-only projection from canonical IR into the existing editor-facing DTO.
/// This owns no compiler-IR mutation or independent table-building algorithm.
pub struct EditorIrBuilder<'a> {
    ir: &'a Ir,
    interner: &'a Interner,
}
impl<'a> EditorIrBuilder<'a> {
    pub fn new(ir: &'a Ir, interner: &'a Interner) -> Self {
        Self { ir, interner }
    }
    fn sign(reference: &SignedRefR) -> Sign {
        match reference.sign() {
            1 => Sign::Plus,
            -1 => Sign::Minus,
            0 => Sign::Neutral,
            _ => unreachable!("closed incidence sign domain"),
        }
    }
    pub fn build(self) -> HyMeKoEditorIR {
        let Self { ir, interner } = self;
        let mut editor = HyMeKoEditorIR::new();
        let mut node_decl_to_vkey: BTreeMap<DeclId, VertexKey> = BTreeMap::new();

        for node in &ir.nodes {
            let name = interner
                .resolve(ir.decl_nodes[node.decl.raw()].name)
                .to_string();
            let vkey = editor.vertices.insert(Vertex {
                name,
                level: 0,
                attributes: Vec::new(),
                position: None,
            });
            node_decl_to_vkey.insert(node.decl, vkey);
        }

        for edge in &ir.edges {
            let name = interner
                .resolve(ir.decl_nodes[edge.decl.raw()].name)
                .to_string();
            let mut incident: Vec<(VertexKey, Sign)> = Vec::new();

            for &arc_id in &edge.arcs {
                let arc = &ir.arcs[arc_id.raw()];
                for r in &arc.refs {
                    if let Some(&vkey) = node_decl_to_vkey.get(&r.target()) {
                        incident.push((vkey, Self::sign(r)));
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
}
