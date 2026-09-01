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

pub mod articulated;
pub mod collide;
pub mod forward;
pub mod kinematics;
pub mod transforms;

pub use articulated::{AbaPlan, AbaScratch};
pub use collide::{
    colliders_of, intersects, intersects_at, place, BroadPhase, Collider, Contact, Convex,
    Placement,
};
pub use forward::{forward_kinematics, FkError, LinkPoses};
pub use kinematics::joints::{JointInfo, JointLimits, JointType};
pub use kinematics::kinematic::{
    extract_kinematic_model, GeometryInfo, GeometryShape, KinematicModel, LinkInfo,
};
pub use transforms::{extract, ModelKind, ModelView, TransformEngine};

/// $SE(n)$ re-exported from the Tensor pillar, so a consumer needs one geometry dependency.
pub use hymeko_clifford::Se;
