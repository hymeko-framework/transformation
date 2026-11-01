//! Inverse kinematics, owned by `hymeko_kinematics`, over models read from IR.

pub use hymeko_kinematics::ik::*;

/// A root-to-end path of a model read from IR.
pub type ModelChain<'m> = hymeko_kinematics::ik::ModelChain<'m, super::IrHandles>;
