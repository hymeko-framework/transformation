//! Numerical backends: execution strategy, and nothing else.
//!
//! # The seam
//!
//! A backend receives a [`Problem`] and returns a [`Solved`]. It is handed decision variables with
//! bounds, a callable that returns the objective and the constraints with their gradients, and no
//! way to ask what any of it means. There is no resistor here, no net, no scenario, and no divider
//! formula — a backend that could see one would be a second place where the design lives.
//!
//! The solver itself is Tensor's: [`AugmentedLagrangian`] is `hymeko_tensor::optimize`'s
//! augmented Lagrangian with a bound-projected Newton inner solve, reached through the
//! [`Constrained`](hymeko_tensor::optimize::Constrained) view that [`Problem`] implements. This
//! module keeps what belongs to the projection: the strategy seam, the evidence report built from
//! the converged point, and the starting lattice chosen for this projection's variables.

use hymeko_tensor::optimize::SolveError;

use crate::problem::{Problem, ProblemError};
use crate::report::Solved;

pub use hymeko_tensor::optimize::AugmentedLagrangian;

/// Why a backend did not return an answer.
#[derive(Debug, Clone, PartialEq)]
pub enum BackendError {
    /// The problem could not be evaluated at a point the search asked about.
    Evaluation(ProblemError),
    /// The search ran out of iterations with the constraints or stationarity unmet.
    NotConverged {
        iterations: usize,
        violation: f64,
        stationarity: f64,
    },
}

impl From<ProblemError> for BackendError {
    fn from(error: ProblemError) -> Self {
        Self::Evaluation(error)
    }
}

impl From<SolveError<ProblemError>> for BackendError {
    fn from(error: SolveError<ProblemError>) -> Self {
        match error {
            SolveError::Evaluation(error) => Self::Evaluation(error),
            SolveError::NotConverged {
                iterations,
                violation,
                stationarity,
            } => Self::NotConverged {
                iterations,
                violation,
                stationarity,
            },
        }
    }
}

/// A numerical strategy for a constrained problem.
pub trait Backend {
    /// What to record in the result, so a result file says how it was obtained.
    fn name(&self) -> String;

    /// Search from `start`.
    fn solve(&self, problem: &Problem, start: &[f64]) -> Result<Solved, BackendError>;
}

impl Backend for AugmentedLagrangian {
    fn name(&self) -> String {
        "augmented-lagrangian/projected-newton".into()
    }

    fn solve(&self, problem: &Problem, start: &[f64]) -> Result<Solved, BackendError> {
        let solution = AugmentedLagrangian::solve(self, problem, start)?;
        let evaluation = problem.evaluate(&solution.x)?;
        Ok(Solved::of(
            problem,
            &evaluation,
            &Backend::name(self),
            true,
            solution.iterations,
            self.tolerance,
            start,
        ))
    }
}

// ─── multistart ─────────────────────────────────────────────────────────────────────────────────

/// Solve from several starting points and keep the best feasible one.
///
/// # Why the starts are a lattice and not a random draw
///
/// A random multistart makes a result that cannot be reproduced without also recording the seed, and
/// a recorded seed is a piece of state living outside the model. The starts here are a deterministic
/// geometric lattice inside the declared bounds, so two runs of the same model on two machines try
/// the same points in the same order and reach the same answer — and if they do not, that is a
/// finding rather than a re-roll.
pub fn multistart(
    backend: &dyn Backend,
    problem: &Problem,
    points: usize,
) -> Vec<Result<Solved, BackendError>> {
    starts(problem, points)
        .into_iter()
        .map(|start| backend.solve(problem, &start))
        .collect()
}

/// The deterministic starting lattice: the model's own start, then a geometric sweep of the box.
///
/// Geometric rather than linear because these are resistances — a decade is the meaningful step, and
/// a linear sweep of `[1e-6, 1e6]` would put every point but the first at the top of the range.
pub fn starts(problem: &Problem, points: usize) -> Vec<Vec<f64>> {
    let declared = problem.start();
    let bounds = problem.bounds();
    let mut out = vec![declared];
    for i in 0..points.saturating_sub(1) {
        // A different fraction per variable, so the lattice is not a diagonal: the k-th variable
        // advances by a shifted stride and the points spread across the box rather than along it.
        let point = bounds
            .iter()
            .enumerate()
            .map(|(k, &(lower, upper))| {
                let steps = points.max(2) as f64;
                let t = ((i as f64 + 1.0) + k as f64 * 0.5) % steps / steps;
                let (lower, upper) = (lower.max(1e-9), upper.min(1e9));
                (lower.ln() + t * (upper.ln() - lower.ln())).exp()
            })
            .collect();
        out.push(point);
    }
    out
}
