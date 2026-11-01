//! The optimisation projection: a compiled HyMeKo IR read as a constrained design problem.
//!
//! # Where the authority sits
//!
//! The `.hymeko` model is the only editable statement of what is being designed. This crate reads
//! it and produces a [`problem::Problem`] — decision variables, parameters, a relation graph,
//! constraints, an objective — which a numerical backend consumes and cannot edit. Nothing here
//! knows what a bulb is, what `R1` means, or that the model is a voltage divider; those live in
//! `core/data/optimization/resistor_divider.hymeko` and nowhere else.
//!
//! ```text
//!   .hymeko  ──parse──▶  IR  ──read::project──▶  Problem  ──backend──▶  Solved
//!   (authority)         (typed)   (projection)  (neutral)   (strategy)  (typed result)
//! ```
//!
//! The one direction that must never appear is an arrow back into the model: a backend that writes
//! semantics is a second authority, and two authorities is no authority.
//!
//! # The four layers
//!
//! - [`dual`] — forward-mode derivatives, so every gradient is exact rather than differenced.
//! - [`network`] — modified nodal analysis, so a topology becomes its own equations.
//! - [`problem`] — the solver-neutral problem, its unit check, and its evaluation.
//! - [`read`] — the IR to [`problem::Problem`] projection, which finds everything by inheritance.
//! - [`report`] — the typed result, carrying declaration identities back out.

pub mod backend;
/// Forward-mode duals, owned by Tensor; the projection and the network solve compute in them.
pub use hymeko_tensor::dual;
pub mod network;
pub mod problem;
pub mod read;
pub mod report;

/// The browser surface, behind the `web` feature. See [`web::solve`].
#[cfg(feature = "web")]
pub mod web;
