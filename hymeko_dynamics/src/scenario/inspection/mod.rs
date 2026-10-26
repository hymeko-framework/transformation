//! Observed pose qualification over the existing discrete scenario engine.
//!
//! The caller supplies acquisition/observation time in one monotonic clock domain.
//! Source loss blocks task progress, not a claimed physical robot stop. Explicit
//! resume admits progress only; execution permissions remain with the executor.
mod model;
mod read;
mod run;

pub use model::{
    Criteria, Evidence, InspectionModel, InspectionPhase, InspectionSnapshot, Pose,
    PoseObservation, Station,
};
pub use run::InspectionSession;
