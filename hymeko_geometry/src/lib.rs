//! # `hymeko_geometry` — the geometric reading of the HyMeKo IR
//!
//! [`hymeko_query`] answers *where* something is in the IR; this crate answers *what it is
//! geometrically*. Links and joints, shapes and sizes, and the poses those imply.
//!
//! ```text
//!   hymeko_core       the IR
//!   hymeko_query      pattern matching on the IR — generic, domain-free
//!   hymeko_geometry   shapes, links, joints, forward kinematics          ← this crate
//!   hymeko_formats    emitters that write it out (URDF, SDF, MJCF, …)
//! ```
//!
//! ## What is here
//!
//! - [`kinematics`] — the vocabulary ([`kinematics::kinematic::KinematicModel`],
//!   [`kinematics::joints::JointInfo`]) and its extraction from compiled IR.
//! - [`forward`] — forward kinematics: the world pose of every link, folded along the IR's own
//!   signed incidences. Rigid motions come from [`hymeko_clifford::Se`] — $SE(n)$ in the Tensor
//!   pillar's geometric algebra — so no transform type is defined here.
//! - [`transforms`] — the `DomainTransform` plugin API and the model view the emitters consume.
//!
//! ## Why it is not part of `hymeko_query`
//!
//! These modules used to live there. A crate that describes itself as a pattern-matching query
//! engine should not also own what a revolute joint is; the query core never referenced them, so
//! the seam was already there — this crate just makes it a boundary.

#![warn(missing_docs)]
#![allow(missing_docs)] // inherited from the moved modules; tightened per-module as they are revisited

#[cfg(feature = "simulation")]
pub mod articulated;
#[cfg(feature = "simulation")]
pub mod collide;
#[cfg(feature = "simulation")]
pub mod contact;
#[cfg(feature = "simulation")]
pub mod engine;
#[cfg(feature = "simulation")]
pub mod forward;
/// Triangle meshes and surface materials, owned by the Geometry repository's IR-free
/// `hymeko_mesh` and re-exported so existing paths keep one type identity.
pub use hymeko_mesh::{material, mesh};
/// HyMeKo-authored, bounded geometry processing recipes and surface execution.
pub mod pipeline;
/// Analytic two-bone limb IK, dependency-free (no `simulation` feature needed).
pub mod two_bone;
#[cfg(feature = "simulation")]
pub mod force_field;
#[cfg(feature = "simulation")]
pub mod kinematics;
#[cfg(feature = "simulation")]
pub mod transforms;
#[cfg(feature = "simulation")]
pub mod track;
#[cfg(feature = "simulation")]
pub mod vehicle;

#[cfg(feature = "simulation")]
pub use articulated::{AbaPlan, AbaScratch};
#[cfg(feature = "simulation")]
pub use engine::{Engine, FreeFlight};
#[cfg(feature = "simulation")]
pub use contact::{
    Limit,
    ground_constraints, ground_contacts, Constraint, ConstraintSolver, ContactParameters,
    GroundContact,
};
#[cfg(feature = "simulation")]
pub use collide::{
    colliders_of, intersects, intersects_at, place, BroadPhase, Collider, Contact, Convex,
    Placement,
};
#[cfg(feature = "simulation")]
pub use forward::{forward_kinematics, FkError, LinkPoses};
#[cfg(feature = "simulation")]
pub use kinematics::joints::{JointInfo, JointLimits, JointType};
#[cfg(feature = "simulation")]
pub use kinematics::kinematic::{
    extract_kinematic_model, GeometryInfo, GeometryShape, KinematicModel, LinkInfo,
};
#[cfg(feature = "simulation")]
pub use transforms::{extract, ModelKind, ModelView, TransformEngine};

/// $SE(n)$ re-exported from the Tensor pillar, so a consumer needs one geometry dependency.
#[cfg(feature = "simulation")]
pub use hymeko_clifford::Se;
