//! HyMeKo Query Engine — standalone crate for pattern matching on
//! compiled hypergraph IR.
//!
//! # Architecture
//!
//! ```text
//! Predicate tree  ──→  QueryEngine  ──→  Vec<QueryMatch>
//!       ↑                   ↑                   ↓
//!   interpret.rs        Ir + Resolver      domain transforms
//!  (from AST)       (from hymeko_core)    (URDF, SDF, etc.)
//! ```
//!
//! The engine is generic over `NameResolver` — works with both
//! `Interner` (daemon) and `StringTable` (Python bindings).

// Staged clippy hygiene for `cargo clippy -- -D warnings` (incremental cleanup).
#![allow(
    dead_code,
    clippy::collapsible_if,
    clippy::needless_range_loop,
    clippy::only_used_in_recursion,
    clippy::should_implement_trait,
    clippy::too_many_arguments,
    clippy::useless_conversion
)]

pub mod bases;
pub mod checks;
pub mod engine;
pub mod entropy;
pub mod fields;
pub mod predicate;
pub mod predicate_expr;

#[cfg(feature = "interpret")]
pub mod interpret;

#[cfg(feature = "interpret")]
pub mod rewrite;

pub mod traceability;
pub mod traits;

#[cfg(test)]
mod test_support;

// Re-exports for convenience
pub use bases::{BaseError, BasePolicy};
pub use checks::{MatchExpectation, QueryCheck};
pub use engine::{ArcBinding, QueryConfig, QueryEngine, QueryMatch};
pub use entropy::{StructuralEntropy, compute_entropy, compute_entropy_hierarchical};
pub use fields::FieldBinding;
pub use predicate::{NamedQuery, Predicate, ValuePredicate};
pub use traits::NameResolver;
