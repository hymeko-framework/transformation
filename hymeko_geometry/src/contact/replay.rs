//! Bounded, offline comparison on the same assembled contact equations.
//!
//! The candidate is deliberately not selected by the runtime engine. ADMM solves
//! shifted cone subproblems; a De Saxce outer fixed point is required to recover
//! the original non-associated Coulomb law. Acceptance uses that original law,
//! not the cost of the convex subproblem or the legacy saturated-row residual.
use super::{ConstraintSolver, ContactParameters, Limit, MOBILITY_FLOOR};
use serde::{Deserialize, Serialize};

mod active_set;
#[cfg(test)]
mod active_set_tests;
mod admm;
// Dense f64 elimination is the Tensor pillar's (`hymeko_tensor::linalg::dense`, moved there
// 2026-10-21 without changing an operation); the replay code keeps calling it `dense`.
pub(crate) use hymeko_tensor::linalg::dense;
#[cfg(test)]
mod diagnostics;
mod local;
mod nsgs;
#[cfg(test)]
mod nsgs_tests;
mod partition;
#[cfg(test)]
mod tests;

/// Immutable capture of one already assembled velocity-level contact problem.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Problem {
    /// Row-major Delassus operator, c by c. Must be symmetric positive semidefinite.
    pub matrix: Vec<f64>,
    /// Free constraint velocities, including the engine's original bias.
    pub rhs: Vec<f64>,
    /// Original unilateral, bilateral and coupled friction conditions.
    pub limits: Vec<Limit>,
    /// Mobility columns, c by generalized-coordinate count; observation only.
    pub response: Vec<f64>,
    /// Original runtime impulses; never used to initialize a comparison.
    pub reference: Vec<f64>,
}

impl Problem {
    /// Validate shape, finite values, cone references and the experiment's 512-row cap.
    /// Positive semidefiniteness is a producer precondition (the engine builds J M^-1 J^T).
    pub fn validate(&self) -> Result<(), String> {
        let n = self.rhs.len();
        if n > 512
            || self.matrix.len() != n * n
            || self.limits.len() != n
            || self.reference.len() != n
            || (n == 0 && !self.response.is_empty())
            || (n > 0 && !self.response.len().is_multiple_of(n))
        {
            return Err("contact replay shape or 512-row cap violated".into());
        }
        if self
            .matrix
            .iter()
            .chain(&self.rhs)
            .chain(&self.response)
            .chain(&self.reference)
            .any(|x| !x.is_finite())
        {
            return Err("contact replay contains a nonfinite value".into());
        }
        self.validate_rows()
    }

    fn validate_rows(&self) -> Result<(), String> {
        let n = self.rhs.len();
        let mut cone_normals = vec![false; n];
        for (k, limit) in self.limits.iter().enumerate() {
            if self.matrix[k * n + k] < 0.0 {
                return Err("negative mobility diagonal".into());
            }
            if let Limit::Friction {
                normal,
                partner,
                mu,
            } = *limit
            {
                if !self.valid_pair(k, normal, partner, mu) {
                    return Err("invalid friction pair or coefficient".into());
                }
                if k < partner && std::mem::replace(&mut cone_normals[normal], true) {
                    return Err("multiple friction cones share one normal".into());
                }
            }
            for j in 0..k {
                let a = self.matrix[k * n + j];
                let b = self.matrix[j * n + k];
                if (a - b).abs() > 1e-9 * (1.0 + a.abs().max(b.abs())) {
                    return Err("nonsymmetric contact operator".into());
                }
            }
        }
        Ok(())
    }

    fn valid_pair(&self, k: usize, normal: usize, partner: usize, mu: f64) -> bool {
        mu.is_finite()
            && mu >= 0.0
            && normal < k
            && partner < self.limits.len()
            && partner != k
            && normal < partner
            && matches!(self.limits[normal], Limit::Pushing)
            && self.limits[partner]
                == (Limit::Friction {
                    normal,
                    partner: k,
                    mu,
                })
    }

    fn velocity(&self, impulse: &[f64], out: &mut [f64]) {
        let n = self.rhs.len();
        for (k, value) in out.iter_mut().enumerate() {
            *value = self.rhs[k]
                + self.matrix[k * n..(k + 1) * n]
                    .iter()
                    .zip(impulse)
                    .map(|(a, p)| a * p)
                    .sum::<f64>();
        }
    }

    /// Original-law projected residual, including sliding direction on saturated cones.
    /// Returns infinity on malformed/nonfinite impulses or an immobile violated row.
    /// Row velocities are converted to impulse units using local diagonal mobility.
    pub fn residual(&self, impulse: &[f64]) -> f64 {
        if impulse.len() != self.rhs.len() || impulse.iter().any(|p| !p.is_finite()) {
            return f64::INFINITY;
        }
        let n = impulse.len();
        let mut velocity = vec![0.0; n];
        self.velocity(impulse, &mut velocity);
        if velocity.iter().any(|v| !v.is_finite()) {
            return f64::INFINITY;
        }
        let mut error = 0.0f64;
        for k in 0..n {
            let row = self.row_residual(k, impulse, &velocity);
            if !row.is_finite() {
                return f64::INFINITY;
            }
            error = error.max(row);
        }
        error / impulse.iter().fold(1.0f64, |a, p| a.max(p.abs()))
    }

    fn row_residual(&self, k: usize, p: &[f64], v: &[f64]) -> f64 {
        let n = p.len();
        let d = self.matrix[k * n + k];
        if d <= MOBILITY_FLOOR {
            let satisfied = match self.limits[k] {
                Limit::Pushing => v[k] >= 0.0 && p[k] == 0.0,
                _ => v[k].abs() <= 1e-12,
            };
            return if satisfied { 0.0 } else { f64::INFINITY };
        }
        match self.limits[k] {
            Limit::Pushing => (p[k] - (p[k] - v[k] / d).max(0.0)).abs(),
            Limit::Free => (v[k] / d).abs(),
            Limit::Friction {
                normal,
                partner,
                mu,
            } => {
                let step = d.max(self.matrix[partner * n + partner]);
                let x = p[k] - v[k] / step;
                let y = p[partner] - v[partner] / step;
                let length = x.hypot(y);
                let radius = mu * p[normal].max(0.0);
                let scale = if length > radius {
                    radius / length
                } else {
                    1.0
                };
                (p[k] - scale * x).abs()
            }
        }
    }
}

/// Explicit experimental alternatives; no automatic runtime promotion.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Method {
    /// Original PGS iteration, starting from zero.
    Pgs,
    /// ADMM with PGS quadratic subsolves, original contact order.
    Admm,
    /// The same candidate, visiting contact blocks in Graph SBSH leaf order.
    AdmmSbsh,
    /// Try at most 16 PGS passes, certify, then use ADMM within the remaining budget.
    Hybrid,
    /// Nonlinear block Gauss-Seidel, local Newton/projection hybrid, adaptive precision.
    Nsgs,
    /// The same block solver with fixed tight local precision (adaptation ablation).
    NsgsFixed,
    /// Adaptive NSGS plus a bounded, plateau-triggered active-set subspace correction.
    /// Experimental; the sweep cap and the original-law acceptance are unchanged.
    NsgsActiveSet,
}

/// Common bounded work and acceptance settings for all replay variants.
#[derive(Clone, Copy, Debug)]
pub struct Settings {
    /// Total PGS passes, including all inner ADMM solves.
    pub max_sweeps: usize,
    /// Original-law projected residual required for acceptance.
    pub tolerance: f64,
    /// Apply a common mobility scale per cone (exact variable change); false is the ablation.
    pub equilibrate: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            max_sweeps: 2048,
            tolerance: 1e-6,
            equilibrate: true,
        }
    }
}

/// Solver result; a failed certificate is retained and must not count as a fast success.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Outcome {
    /// Chosen experimental algorithm.
    pub method: Method,
    /// Final candidate impulses.
    pub impulse: Vec<f64>,
    /// Original-law error, not the ADMM subproblem cost.
    pub residual: f64,
    /// Whether the physical equation tolerance was met.
    pub accepted: bool,
    /// Total inner passes, bounded by Settings::max_sweeps.
    pub sweeps: usize,
    /// Number of ADMM updates (zero for plain PGS).
    pub iterations: usize,
    /// Number of nonempty spatial patches; zero for original order.
    pub patches: usize,
    /// Owned numerical buffer capacities, excluding allocator overhead and input Problem.
    pub buffer_bytes: usize,
    /// Local Newton attempts in the NSGS variant, distinct from global sweeps.
    #[serde(default)]
    pub newton_steps: usize,
    /// Local projected fallback iterations in the NSGS variant.
    #[serde(default)]
    pub projection_steps: usize,
    /// Local stage results rejected because their physical error grew or was nonfinite.
    #[serde(default)]
    pub discarded_stages: usize,
    /// Active-set corrections attempted (NsgsActiveSet only), separate from sweeps.
    #[serde(default)]
    pub corrections_attempted: usize,
    /// Corrections kept because the original-law residual decreased.
    #[serde(default)]
    pub corrections_accepted: usize,
    /// Rejected corrections: singular reduced system, zero admissible step, no strict decrease.
    #[serde(default)]
    pub corrections_rejected: [usize; 3],
    /// Sum of reduced unknowns over all correction attempts.
    #[serde(default)]
    pub correction_unknowns: usize,
    /// Arithmetic operations of the reduced dense eliminations.
    #[serde(default)]
    pub correction_operations: usize,
    /// Dense Delassus products made by corrections (velocity and direction image).
    #[serde(default)]
    pub correction_products: usize,
    /// Original-law residual evaluations inside NSGS variants, including correction candidates.
    #[serde(default)]
    pub residual_evaluations: usize,
}

/// Prepared comparison state. Construction/partition cost can be timed separately.
pub struct Replay {
    problem: Problem,
    method: Method,
    solver: ConstraintSolver,
    order: Vec<usize>,
    patches: usize,
    state: admm::State,
    nonlinear: nsgs::State,
    subspace: active_set::State,
}

impl Replay {
    /// Prepare a replay. Anchors have one entry per row; None keeps nonspatial rows global.
    /// Paired tangents remain in their normal's block. Invalid input returns an error.
    pub fn new(
        problem: &Problem,
        method: Method,
        anchors: &[Option<[f64; 3]>],
    ) -> Result<Self, String> {
        problem.validate()?;
        if anchors.len() != problem.rhs.len()
            || anchors.iter().flatten().flatten().any(|x| !x.is_finite())
        {
            return Err("invalid contact row anchors".into());
        }
        let (order, patches) = partition::order(problem, anchors, method);
        let solver = ConstraintSolver {
            a: problem.matrix.clone(),
            b: problem.rhs.clone(),
            limits: problem.limits.clone(),
            impulse: vec![0.0; problem.rhs.len()],
            ..Default::default()
        };
        let state = admm::State::new(
            if matches!(method, Method::Admm | Method::AdmmSbsh | Method::Hybrid) {
                problem.rhs.len()
            } else {
                0
            },
        );
        let nonlinear = nsgs::State::new(problem, method);
        Ok(Self {
            problem: problem.clone(),
            method,
            solver,
            order,
            patches,
            state,
            nonlinear,
            subspace: active_set::State::new(),
        })
    }

    /// Solve from zero, preserving the captured physical problem and all iteration caps.
    /// This never advances an engine or silently falls back to a different method.
    pub fn solve(&mut self, settings: Settings) -> Result<Outcome, String> {
        if settings.max_sweeps == 0
            || settings.max_sweeps > 65_536
            || !settings.tolerance.is_finite()
            || settings.tolerance <= 0.0
        {
            return Err("invalid replay work/accuracy settings".into());
        }
        self.solver.a.clone_from(&self.problem.matrix);
        self.solver.b.clone_from(&self.problem.rhs);
        self.solver.limits.clone_from(&self.problem.limits);
        self.solver.impulse.fill(0.0);
        self.nonlinear.reset();
        self.subspace.reset();
        let (sweeps, iterations) = match self.method {
            Method::Pgs => self.solve_pgs(settings),
            Method::Hybrid => self.solve_hybrid(settings),
            Method::Nsgs | Method::NsgsFixed | Method::NsgsActiveSet => self.solve_nsgs(settings),
            _ => self.solve_admm(settings),
        };
        let residual = self.problem.residual(&self.solver.impulse);
        let corrections = self.subspace.work;
        Ok(Outcome {
            method: self.method,
            impulse: self.solver.impulse.clone(),
            residual,
            accepted: residual.is_finite() && residual <= settings.tolerance,
            sweeps,
            iterations,
            patches: self.patches,
            buffer_bytes: self.buffer_bytes(),
            newton_steps: self.nonlinear.stats.newton,
            projection_steps: self.nonlinear.stats.projection,
            discarded_stages: self.nonlinear.stats.discarded,
            corrections_attempted: corrections.attempted,
            corrections_accepted: corrections.accepted,
            corrections_rejected: [
                corrections.singular,
                corrections.blocked,
                corrections.no_decrease,
            ],
            correction_unknowns: corrections.unknowns,
            correction_operations: corrections.eliminations,
            correction_products: corrections.products,
            residual_evaluations: self.nonlinear.residuals + corrections.residuals,
        })
    }

    fn solve_hybrid(&mut self, settings: Settings) -> (usize, usize) {
        let (sweeps, _) = self.solve_pgs(Settings {
            max_sweeps: settings.max_sweeps.min(16),
            ..settings
        });
        if sweeps == settings.max_sweeps
            || self.problem.residual(&self.solver.impulse) <= settings.tolerance
        {
            return (sweeps, 0);
        }
        let (more, iterations) = self.solve_admm(Settings {
            max_sweeps: settings.max_sweeps - sweeps,
            ..settings
        });
        (sweeps + more, iterations)
    }

    fn solve_pgs(&mut self, settings: Settings) -> (usize, usize) {
        if self.problem.rhs.is_empty() {
            return (0, 0);
        }
        let mut sweeps = 0;
        while sweeps < settings.max_sweeps {
            let count = 8.min(settings.max_sweeps - sweeps);
            let params = ContactParameters {
                iterations: count,
                tolerance: 0.0,
                ..Default::default()
            };
            self.solver.iterate(&params, Some(&self.order));
            sweeps += self.solver.sweeps;
            if self.problem.residual(&self.solver.impulse) <= settings.tolerance {
                break;
            }
        }
        (sweeps, 0)
    }

    fn buffer_bytes(&self) -> usize {
        let p = &self.problem;
        let s = &self.solver;
        let floats = p.matrix.capacity()
            + p.rhs.capacity()
            + p.response.capacity()
            + p.reference.capacity()
            + s.a.capacity()
            + s.b.capacity()
            + s.impulse.capacity()
            + self.state.capacity();
        floats * std::mem::size_of::<f64>()
            + self.order.capacity() * std::mem::size_of::<usize>()
            + (p.limits.capacity() + s.limits.capacity()) * std::mem::size_of::<Limit>()
            + self.nonlinear.capacity_bytes()
            + self.subspace.capacity_bytes()
    }
}
