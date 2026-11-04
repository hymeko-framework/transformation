//! The kinematic reading of a HyMeKo IR.
//!
//! The model types and the computations on them are the Geometry repository's
//! `hymeko_kinematics`; this module reads them out of compiled IR and re-exports them with
//! [`IrHandles`], so `did`, `parent`, `child` and the actuation references are Core
//! [`DeclId`]s as before.

use hymeko::common::ids::DeclId;

pub mod actuation;
pub mod assembly;
pub mod cameras;
pub mod ik;
pub mod joints;
pub mod kinematic;
pub mod tp;
pub mod visual;

pub use hymeko_kinematics::{dh, playout};

/// Core declaration handles: what a model read from a HyMeKo IR carries on its parts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct IrHandles;

impl hymeko_kinematics::Handles for IrHandles {
    type Id = DeclId;
    const NONE: DeclId = DeclId::NONE;
}
