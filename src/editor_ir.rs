//! Editor-facing IR: slotmap storage + atomic [`IRDelta`] mutations.
//!
//! This is **not** the compile IR in `hymeko_core::ir`. See
//! `docs/plans/06_wasm_editor/step1_ir_design.md` for why we keep the
//! two IRs separate and bridge between them via [`crate::bridge`].
//!
//! The key difference from `hymeko_core::ir::Ir`: this one is
//! *mutation-first*. Every change goes through an [`IRDelta`] so the
//! WASM editor can maintain an undo stack, and a P2P layer can gossip
//! the deltas verbatim (they `#[derive(Serialize, Deserialize)]`).

use serde::{Deserialize, Serialize};
use slotmap::{new_key_type, SlotMap};
use thiserror::Error;

new_key_type! {
    pub struct VertexKey;
    pub struct EdgeKey;
    pub struct PatchKey;
}

/// Canvas layout hint for a vertex. Not used by the emitter; carried
/// through round-trips so the frontend can preserve manual placement.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Position {
    pub x: f64,
    pub y: f64,
}

/// Signed-incidence discipline matching `hymeko_core::ir::SignedRefR`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Sign {
    Plus,
    Minus,
    Neutral,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum AttributeValue {
    Int(i64),
    Float(f64),
    Str(String),
    Bool(bool),
    List(Vec<AttributeValue>),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Attribute {
    pub key: String,
    pub value: AttributeValue,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Vertex {
    pub name: String,
    /// G-SPHF level, typically `-2..=8`.
    pub level: i8,
    pub attributes: Vec<Attribute>,
    pub position: Option<Position>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HyperEdge {
    pub name: String,
    /// Per-arc endpoints: which vertex, with what sign discipline.
    ///
    /// We store the sign alongside the key (unlike the spec) so the
    /// bridge to `SignedRefR::{Plus,Minus,Neutral}` is lossless.
    pub incident: Vec<(VertexKey, Sign)>,
    pub weight: f64,
    pub patch_id: Option<PatchKey>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Patch {
    pub name: String,
    pub level: i8,
    pub vertices: Vec<VertexKey>,
}

/// Atomic mutation unit. Also the P2P gossip unit for a future
/// `hymeko_wire`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum IRDelta {
    AddVertex { data: Vertex },
    RemoveVertex { key: VertexKey },
    AddHyperEdge { data: HyperEdge },
    RemoveEdge { key: EdgeKey },
    MoveVertex { key: VertexKey, position: Position },
    UpdateWeight { key: EdgeKey, weight: f64 },
    UpdateSign { key: EdgeKey, arc_index: usize, sign: Sign },
    AttachAttribute { key: VertexKey, attr: Attribute },
    DetachAttribute { key: VertexKey, name: String },
    AddPatch { data: Patch },
    /// Bulk apply — succeeds-or-fails as one unit; useful for CRDT
    /// gossip and for the bridge's "rebuild after n edits" pattern.
    Batch { deltas: Vec<IRDelta> },
}

#[derive(Debug, Error)]
pub enum IRError {
    #[error("entity not found")]
    NotFound,
    #[error("arc index {0} out of range for edge with {1} arc(s)")]
    ArcIndexOutOfRange(usize, usize),
    #[error("attribute `{0}` not attached to the given vertex")]
    AttributeNotFound(String),
    #[error("invalid delta: {0}")]
    Invalid(String),
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct HyMeKoEditorIR {
    pub vertices: SlotMap<VertexKey, Vertex>,
    pub hyperedges: SlotMap<EdgeKey, HyperEdge>,
    pub patches: SlotMap<PatchKey, Patch>,
}

impl HyMeKoEditorIR {
    pub fn new() -> Self {
        Self::default()
    }

    /// Apply a single delta. On [`IRDelta::Batch`] the deltas are applied
    /// in order; if any fails, the already-applied ones are **not**
    /// rolled back (callers can snapshot before a Batch if they need
    /// transactional semantics).
    pub fn apply(&mut self, delta: IRDelta) -> Result<(), IRError> {
        match delta {
            IRDelta::AddVertex { data } => {
                self.vertices.insert(data);
                Ok(())
            }
            IRDelta::RemoveVertex { key } => {
                self.vertices.remove(key).ok_or(IRError::NotFound)?;
                // Also purge dangling references from hyperedges' incident lists.
                for he in self.hyperedges.values_mut() {
                    he.incident.retain(|(v, _)| *v != key);
                }
                Ok(())
            }
            IRDelta::AddHyperEdge { data } => {
                self.hyperedges.insert(data);
                Ok(())
            }
            IRDelta::RemoveEdge { key } => {
                self.hyperedges.remove(key).ok_or(IRError::NotFound)?;
                Ok(())
            }
            IRDelta::MoveVertex { key, position } => {
                self.vertices.get_mut(key).ok_or(IRError::NotFound)?.position = Some(position);
                Ok(())
            }
            IRDelta::UpdateWeight { key, weight } => {
                self.hyperedges.get_mut(key).ok_or(IRError::NotFound)?.weight = weight;
                Ok(())
            }
            IRDelta::UpdateSign {
                key,
                arc_index,
                sign,
            } => {
                let he = self.hyperedges.get_mut(key).ok_or(IRError::NotFound)?;
                if arc_index >= he.incident.len() {
                    return Err(IRError::ArcIndexOutOfRange(arc_index, he.incident.len()));
                }
                he.incident[arc_index].1 = sign;
                Ok(())
            }
            IRDelta::AttachAttribute { key, attr } => {
                self.vertices
                    .get_mut(key)
                    .ok_or(IRError::NotFound)?
                    .attributes
                    .push(attr);
                Ok(())
            }
            IRDelta::DetachAttribute { key, name } => {
                let v = self.vertices.get_mut(key).ok_or(IRError::NotFound)?;
                let before = v.attributes.len();
                v.attributes.retain(|a| a.key != name);
                if v.attributes.len() == before {
                    Err(IRError::AttributeNotFound(name))
                } else {
                    Ok(())
                }
            }
            IRDelta::AddPatch { data } => {
                self.patches.insert(data);
                Ok(())
            }
            IRDelta::Batch { deltas } => {
                for d in deltas {
                    self.apply(d)?;
                }
                Ok(())
            }
        }
    }
}
