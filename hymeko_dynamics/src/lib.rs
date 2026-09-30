//! The dynamical projection: a compiled HyMeKo IR read as a port-Hamiltonian system.
//!
//! ```text
//!   .hymeko  ──parse──▶  IR  ──read::project──▶  System  ──┬─▶ analysis::modes    (what it will do)
//!   (authority)         (typed)  (projection)   (neutral)  └─▶ run::run           (what it does)
//! ```
//!
//! The two branches on the right are the reason this crate exists in this shape. `analysis` solves
//! an eigenvalue problem and a resolvent; `run` steps a trajectory through a structure-preserving
//! integrator. They share the assembled system and NOTHING else — no formula, no intermediate — so
//! their agreement is evidence rather than tautology.
//!
//! `J` is never written down. It is the signed incidence of the declared hypergraph, and the
//! skew-symmetry that makes the energy balance hold is a consequence of the sign convention.
//!
//! [`field`] adds the other half of a modal expansion: the spatial shape each coordinate is the
//! amplitude of. The dynamics has no space in it, so without that declaration there is no field
//! anywhere and a picture of one would be invented. With it, the geometry implies a capacity that
//! must equal the declared one — and does, to zero.

pub mod analysis;
pub mod diffusion;
pub mod field;
pub mod fluid;
pub mod hierarchy;
pub mod power;
pub mod kirchhoff;
pub mod layout;
pub mod read;
pub mod run;
pub mod scenario;
pub mod softbody;
pub mod sources;
pub mod system;

/// The browser surface, behind the `web` feature. See [`web::transient`].
#[cfg(feature = "web")]
pub mod web;

pub use read::project;
pub use sources::{MemoryProvider, SourceError, project_file, project_sources};
pub use system::System;
