//! `hymeko_emitter` — editor-facing IR and M2T emitters.
//!
//! Two complementary pieces live here:
//!
//! - [`editor_ir`] — a slotmap-backed [`editor_ir::HyMeKoEditorIR`] with
//!   atomic [`editor_ir::IRDelta`] mutations, designed for the WASM editor
//!   (Plan 06, `steps/20260418/hymeko_claude_code_spec.md`). This IR is
//!   *not* the compile IR; see `docs/plans/06_wasm_editor/step1_ir_design.md`
//!   for the dual-IR rationale.
//! - Emitters (`emit_hymeko`, `emit_sysml`, `emit_rust_stubs`, `emit_lean4`)
//!   that consume the **arena compile IR** from `hymeko::ir::ir::Ir` and
//!   produce deterministic text. The editor converts to the arena IR via
//!   [`bridge`] before emitting.

pub mod editor_ir;
pub mod bridge;
pub mod emit_hymeko;
pub mod emit_sysml;
pub mod emit_rust_stubs;
pub mod emit_lean4;

pub use editor_ir::{
    Attribute, AttributeValue, HyMeKoEditorIR, IRDelta, IRError, Patch, Position, Sign, Vertex,
    HyperEdge,
};
pub use emit_hymeko::emit_hymeko;
pub use emit_sysml::emit_sysml;
pub use emit_rust_stubs::emit_rust_stubs;
pub use emit_lean4::emit_lean4;
