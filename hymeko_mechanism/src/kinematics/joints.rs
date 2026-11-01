//! Joint data, owned by `hymeko_kinematics`, with Core declaration handles.

pub use hymeko_kinematics::joints::*;

/// A joint read from IR: `did`, `parent` and `child` are Core declarations.
pub type JointInfo = hymeko_kinematics::joints::JointInfo<super::IrHandles>;
