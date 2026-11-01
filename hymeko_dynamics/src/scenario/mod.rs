//! Source-declared, untimed discrete scenarios over the canonical IR.
//! State changes belong here; browser and renderer layers only consume evidence.
//!
//! Contract and consumer guide:
//! <https://github.com/hymeko-framework/hymeko_web/blob/feat/scenario-runtime/docs/scenario-runtime.md>
//! The executable source contract is exercised by `tests/scenario.rs` and its
//! `fixtures/reference-cell.hymeko` input. Open validates a coherent compiled IR;
//! mutations return typed refusals and never partially commit a single action.
pub mod inspection;
mod model;
mod read;
mod session;
mod wire;

pub use model::*;
pub use session::ScenarioSession;
