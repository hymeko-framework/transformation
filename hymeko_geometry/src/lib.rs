//! # `hymeko_geometry` — the geometric reading of the HyMeKo IR
//!
//! [`hymeko_query`] answers *where* something is in the IR; this crate answers *what it is
//! geometrically*: it reads links, joints, shapes, frames and actuation out of compiled IR into a
//! kinematic model, and projects the views the emitters consume.
//!
//! ```text
//!   hymeko_core        the IR
//!   hymeko_query       pattern matching on the IR — generic, domain-free
//!   hymeko_geometry    reading the kinematic model out of the IR            ← this crate
//!   hymeko_formats     emitters that write it out (URDF, SDF, MJCF, …)
//! ```
//!
//! ## What is here
//!
//! - [`kinematics`] — extraction from compiled IR ([`extract_kinematic_model`], scoped groups,
//!   actuation, assemblies, cameras, visuals) and [`kinematics::IrHandles`], the Core
//!   declaration handles the extracted model carries.
//! - [`transforms`] — the `DomainTransform` plugin API and the model view the emitters consume.
//! - [`pipeline`], [`track`] — recipes and trajectories projected from IR.
//!
//! ## What is re-exported, not implemented here
//!
//! The computations no longer live in this repository. They are re-exported under their former
//! paths so consumers keep compiling while they move to the owners:
//!
//! - `mesh`, `material` — the Geometry repository's `hymeko_mesh`.
//! - `kinematics::{joints, kinematic, actuation, dh, ik, playout}` types, [`forward`],
//!   [`collide`], [`two_bone`] — the Geometry repository's `hymeko_kinematics`; the model types
//!   are aliases with [`kinematics::IrHandles`], so `did`, `parent` and `child` stay Core `DeclId`s.
//! - [`articulated`], [`contact`], [`engine`], [`force_field`], [`vehicle`] — the kawakern
//!   repository; [`Engine`] is `kawakern::Engine<IrHandles>`.
//!
//! ## Why it is not part of `hymeko_query`
//!
//! These modules used to live there. A crate that describes itself as a pattern-matching query
//! engine should not also own what a revolute joint is; the query core never referenced them, so
//! the seam was already there — this crate just makes it a boundary.

#![warn(missing_docs)]
#![allow(missing_docs)] // inherited from the moved modules; tightened per-module as they are revisited

#[cfg(feature = "simulation")]
pub use kawakern::articulated;
#[cfg(feature = "simulation")]
pub use hymeko_kinematics::collide;
#[cfg(feature = "simulation")]
pub use kawakern::contact;
/// Kawakern's stepping engine, with Core declaration handles.
#[cfg(feature = "simulation")]
pub mod engine {
    pub use kawakern::engine::*;

    /// The engine over a model read from IR.
    pub type Engine = kawakern::engine::Engine<crate::kinematics::IrHandles>;
}
#[cfg(feature = "simulation")]
pub use hymeko_kinematics::forward;
/// Triangle meshes and surface materials, owned by the Geometry repository's IR-free
/// `hymeko_mesh` and re-exported so existing paths keep one type identity.
pub use hymeko_mesh::{material, mesh};
/// HyMeKo-authored, bounded geometry processing recipes and surface execution.
pub mod pipeline;
/// Analytic two-bone limb IK, dependency-free (no `simulation` feature needed).
pub use hymeko_kinematics::two_bone;
#[cfg(feature = "simulation")]
pub use kawakern::force_field;
#[cfg(feature = "simulation")]
pub mod kinematics;
#[cfg(feature = "simulation")]
pub mod transforms;
#[cfg(feature = "simulation")]
pub mod track;
#[cfg(feature = "simulation")]
pub use kawakern::vehicle;

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
