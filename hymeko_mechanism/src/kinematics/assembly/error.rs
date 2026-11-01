use crate::{forward::FkError, kinematics::kinematic::scope::ScopedModelError};
use hymeko::common::ids::DeclId;

/// An authored assembly cannot be bound to the existing mechanical projections.
#[derive(Debug, Clone, PartialEq)]
pub enum AssemblyError {
    Model(ScopedModelError),
    Topology(FkError),
    Binding(String),
    State(StateError),
}
impl std::fmt::Display for AssemblyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Model(error) => error.fmt(f),
            Self::Topology(error) => error.fmt(f),
            Self::Binding(error) => f.write_str(error),
            Self::State(error) => error.fmt(f),
        }
    }
}
impl std::error::Error for AssemblyError {}
impl From<ScopedModelError> for AssemblyError {
    fn from(e: ScopedModelError) -> Self {
        Self::Model(e)
    }
}
impl From<FkError> for AssemblyError {
    fn from(e: FkError) -> Self {
        Self::Topology(e)
    }
}
impl From<String> for AssemblyError {
    fn from(e: String) -> Self {
        Self::Binding(e)
    }
}
impl From<StateError> for AssemblyError {
    fn from(e: StateError) -> Self {
        Self::State(e)
    }
}

/// A sample is refused before replacing the assembly's accepted numeric pose.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StateError {
    JointCount { expected: usize, received: usize },
    NonFiniteCoordinate { index: usize },
    InvalidCoordinateSample { declaration: DeclId },
    InvalidPlacement,
    NonFinitePose { link: usize },
}
impl std::fmt::Display for StateError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::JointCount { expected, received } => {
                write!(f, "expected {expected} coordinates, received {received}")
            }
            Self::NonFiniteCoordinate { index } => write!(f, "coordinate {index} is nonfinite"),
            Self::InvalidCoordinateSample { declaration } => write!(
                f,
                "coordinate leaf {} needs a finite numeric value",
                declaration.raw()
            ),
            Self::InvalidPlacement => {
                f.write_str("assembly placement must be a finite unit rigid transform")
            }
            Self::NonFinitePose { link } => write!(f, "link {link} produced an invalid rigid pose"),
        }
    }
}
impl std::error::Error for StateError {}
