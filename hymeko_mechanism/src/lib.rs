//! # `hymeko_mechanism` — mechanisms read out of HyMeKo IR
//!
//! The links, joints, frames, actuation, visuals and cameras of an authored mechanism — a robot, a
//! door, a slider — read out of compiled IR into the Geometry repository's kinematic model, carrying
//! Core declaration handles ([`kinematics::IrHandles`]); and geometry-processing recipes read out of
//! IR (`recipe`, `authoring` feature).
//!
//! ```text
//!   hymeko_core        the IR
//!   hymeko_query       pattern matching on the IR — generic, domain-free
//!   hymeko_mechanism   mechanisms (and recipes) read out of the IR          ← this crate
//!   hymeko_formats     emitters that write them out (URDF, SDF, MJCF, …)
//! ```
//!
//! Formerly `hymeko_geometry` (renamed 2026-11-01). The computations it used to carry live with
//! their owners: meshes, materials and recipe execution are `hymeko_mesh`; kinematic models,
//! FK/IK, DH chains, playout, two-bone limbs and collision are `hymeko_kinematics`; physics and
//! roads are `kawakern`; the emitter plugin API and Gazebo plugins are `hymeko_formats`. The mesh
//! and kinematics crates are re-exported here (the model types as aliases with `IrHandles`), so a
//! reader's output and its types have one identity; physics is used from `kawakern` directly.

#![warn(missing_docs)]
#![allow(missing_docs)] // inherited from the moved modules; tightened per-module as they are revisited

/// Triangle meshes and surface materials (`hymeko_mesh`), re-exported so paths keep one identity.
pub use hymeko_mesh::{material, mesh};
/// Analytic two-bone limb IK, dependency-free (no `simulation` feature needed).
pub use hymeko_kinematics::two_bone;
#[cfg(feature = "simulation")]
pub use hymeko_kinematics::{collide, forward};
#[cfg(feature = "simulation")]
pub mod kinematics;
/// Geometry-processing recipes read out of IR; `hymeko_mesh::recipe` executes them.
#[cfg(feature = "authoring")]
pub mod recipe;

#[cfg(feature = "simulation")]
pub use collide::{
    colliders_of, intersects, intersects_at, place, BroadPhase, Collider, Contact, Convex,
    Placement,
};
#[cfg(feature = "simulation")]
pub use forward::{forward_kinematics, FkError, LinkPoses};
#[cfg(feature = "simulation")]
pub use kinematics::actuation::Fields;
#[cfg(feature = "simulation")]
pub use kinematics::joints::{JointInfo, JointLimits, JointType};
#[cfg(feature = "simulation")]
pub use kinematics::kinematic::{
    extract_kinematic_model, GeometryInfo, GeometryShape, KinematicModel, LinkInfo,
};

/// $SE(n)$ re-exported from the Tensor pillar, so a consumer needs one geometry dependency.
#[cfg(feature = "simulation")]
pub use hymeko_clifford::Se;
