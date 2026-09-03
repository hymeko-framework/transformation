//! Numerical backends: execution strategy, and nothing else.
//!
//! # The seam
//!
//! A backend receives a [`Problem`] and returns a [`Solved`]. It is handed decision variables with
//! bounds, a callable that returns the objective and the constraints with their gradients, and no
//! way to ask what any of it means. There is no resistor here, no net, no scenario, and no divider
//! formula — a backend that could see one would be a second place where the design lives.
//!
//! Two implementations ship, and they are deliberately different in kind:
//!
//! - [`AugmentedLagrangian`] runs in this process, in Rust, on exact forward-mode gradients.
//! - [`External`] hands the same neutral problem to another process over a pipe. The child owns the
//!   search and nothing else: it asks for `f`, `h`, `g` and their gradients at a point, and this side
//!   answers. `backends/slsqp.py` is such a child, and it contains no circuit vocabulary at all.
//!
//! That the two agree to the tolerance is the evidence that the seam is real. If the semantics had
//! leaked into either one, they would have to leak into both identically for the answers to match.

use std::io::{BufRead, BufReader, Write};
use std::process::{Child, Command, Stdio};

use crate::problem::{Evaluation, Problem, ProblemError};
use crate::report::Solved;

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
    /// An external backend could not be started, or spoke out of turn.
    Channel(String),
}

impl From<ProblemError> for BackendError {
    fn from(error: ProblemError) -> Self {
        Self::Evaluation(error)
    }
}

/// A numerical strategy for a constrained problem.
pub trait Backend {
    /// What to record in the result, so a result file says how it was obtained.
    fn name(&self) -> String;

    /// Search from `start`.
    fn solve(&self, problem: &Problem, start: &[f64]) -> Result<Solved, BackendError>;
}

// ─── the in-process backend ─────────────────────────────────────────────────────────────────────

/// Augmented Lagrangian with a bound-projected Newton inner solve.
///
/// # Why not a plain penalty
///
/// A quadratic penalty reaches the constrained optimum only as its coefficient goes to infinity, and
/// long before that the inner problem is too ill-conditioned to solve. The augmented Lagrangian adds
/// a multiplier estimate that absorbs the constraint's own gradient, so the penalty stays moderate
/// and the residual falls with the multiplier rather than with the conditioning. For this model that
/// matters: the optimum sits ON the open-circuit inequality, so a method that only approaches
/// feasibility from inside would never find it and one that only approaches from outside would
/// report an infeasible design as the answer.
#[derive(Debug, Clone)]
pub struct AugmentedLagrangian {
    /// Constraint violation and stationarity both below this counts as converged.
    pub tolerance: f64,
    pub outer_iterations: usize,
    pub inner_iterations: usize,
    pub penalty: f64,
    pub penalty_max: f64,
}

impl Default for AugmentedLagrangian {
    fn default() -> Self {
        Self {
            tolerance: 1e-10,
            outer_iterations: 200,
            inner_iterations: 200,
            penalty: 10.0,
            penalty_max: 1e10,
        }
    }
}

impl Backend for AugmentedLagrangian {
    fn name(&self) -> String {
        "augmented-lagrangian/projected-newton".into()
    }

    fn solve(&self, problem: &Problem, start: &[f64]) -> Result<Solved, BackendError> {
        let n = problem.dimension();
        let bounds = problem.bounds();
        let mut x: Vec<f64> = start
            .iter()
            .enumerate()
            .map(|(k, v)| v.clamp(bounds[k].0, bounds[k].1))
            .collect();

        let probe = problem.evaluate(&x)?;
        let mut state = Augmented {
            problem,
            lambda: vec![0.0; probe.equalities.len()],
            mu: vec![0.0; probe.inequalities.len()],
            rho: self.penalty,
        };

        let mut previous = f64::INFINITY;
        let mut iterations = 0usize;
        let mut converged = false;
        let mut stationarity = f64::INFINITY;

        for _ in 0..self.outer_iterations {
            let inner = self.minimise(&state, &mut x, &bounds, n)?;
            iterations += inner;

            let (_, grad, evaluation) = state.at(&x)?;
            stationarity = free_norm(&x, &grad, &bounds);
            let violation = violation_of(&evaluation);

            if violation <= self.tolerance && stationarity <= self.tolerance {
                converged = true;
                break;
            }

            for (i, h) in evaluation.equalities.iter().enumerate() {
                state.lambda[i] -= state.rho * h.value;
            }
            for (j, g) in evaluation.inequalities.iter().enumerate() {
                state.mu[j] = (state.mu[j] - state.rho * g.value).max(0.0);
            }
            // Only tighten when the last round did not buy enough feasibility; raising the penalty
            // on a round that IS converging just spends conditioning for nothing.
            if violation > 0.25 * previous {
                state.rho = (state.rho * 10.0).min(self.penalty_max);
            }
            previous = violation;
        }

        let evaluation = problem.evaluate(&x)?;
        if !converged {
            return Err(BackendError::NotConverged {
                iterations,
                violation: violation_of(&evaluation),
                stationarity,
            });
        }
        Ok(Solved::of(
            problem,
            &evaluation,
            &self.name(),
            true,
            iterations,
            self.tolerance,
            start,
        ))
    }
}

impl AugmentedLagrangian {
    /// Minimise the augmented Lagrangian at fixed multipliers, inside the bounds.
    fn minimise(
        &self,
        state: &Augmented<'_>,
        x: &mut Vec<f64>,
        bounds: &[(f64, f64)],
        n: usize,
    ) -> Result<usize, BackendError> {
        let mut damping = 1e-8;
        for iteration in 0..self.inner_iterations {
            let (value, grad, _) = state.at(x)?;
            let free: Vec<usize> = (0..n).filter(|&k| is_free(x[k], grad[k], bounds[k])).collect();
            if free.is_empty() || free_norm(x, &grad, bounds) <= self.tolerance * 1e-2 {
                return Ok(iteration);
            }

            // Newton on the free block, with the Hessian from central differences of the EXACT
            // gradient. The gradient is exact to rounding, so a differenced Hessian is accurate to
            // about `sqrt(eps)` relatively — plenty to steer a step whose acceptance is judged on
            // the exact value and whose stopping test is the exact gradient.
            let hessian = state.hessian(x, &free, bounds)?;
            let mut direction = vec![0.0; n];
            let mut solved = false;
            for _ in 0..12 {
                let mut a = hessian.clone();
                for i in 0..free.len() {
                    a[i * free.len() + i] += damping;
                }
                let rhs: Vec<f64> = free.iter().map(|&k| -grad[k]).collect();
                if let Some(step) = dense_solve(&mut a, rhs, free.len()) {
                    let slope: f64 = free.iter().zip(&step).map(|(&k, s)| grad[k] * s).sum();
                    if slope < 0.0 && step.iter().all(|s| s.is_finite()) {
                        for (i, &k) in free.iter().enumerate() {
                            direction[k] = step[i];
                        }
                        solved = true;
                        break;
                    }
                }
                damping *= 10.0;
            }
            if !solved {
                // Steepest descent is always available and always a descent direction.
                for &k in &free {
                    direction[k] = -grad[k];
                }
            }

            // Armijo backtracking, projecting onto the bounds at every trial so a step that would
            // leave the box is shortened rather than rejected.
            let mut step = 1.0;
            let mut accepted = false;
            for _ in 0..60 {
                let trial: Vec<f64> = (0..n)
                    .map(|k| (x[k] + step * direction[k]).clamp(bounds[k].0, bounds[k].1))
                    .collect();
                let moved: f64 = trial
                    .iter()
                    .zip(x.iter())
                    .map(|(t, o)| (t - o).abs())
                    .fold(0.0, f64::max);
                if moved == 0.0 {
                    break;
                }
                if let Ok((candidate, _, _)) = state.at(&trial) {
                    let predicted: f64 = (0..n).map(|k| grad[k] * (trial[k] - x[k])).sum();
                    if candidate.is_finite() && candidate <= value + 1e-4 * predicted {
                        *x = trial;
                        accepted = true;
                        break;
                    }
                }
                step *= 0.5;
            }
            if !accepted {
                return Ok(iteration);
            }
            damping = (damping * 0.1).max(1e-12);
        }
        Ok(self.inner_iterations)
    }
}

/// The augmented Lagrangian at a set of multipliers.
struct Augmented<'a> {
    problem: &'a Problem,
    /// One per equality; free in sign.
    lambda: Vec<f64>,
    /// One per inequality; non-negative.
    mu: Vec<f64>,
    rho: f64,
}

impl Augmented<'_> {
    /// `(L, dL, the underlying evaluation)`.
    fn at(&self, x: &[f64]) -> Result<(f64, Vec<f64>, Evaluation), ProblemError> {
        let evaluation = self.problem.evaluate(x)?;
        let n = x.len();
        let mut value = evaluation.objective.value;
        let mut grad = evaluation.objective.grad.clone();

        for (i, h) in evaluation.equalities.iter().enumerate() {
            value += -self.lambda[i] * h.value + 0.5 * self.rho * h.value * h.value;
            let coefficient = self.rho * h.value - self.lambda[i];
            for k in 0..n {
                grad[k] += coefficient * h.grad[k];
            }
        }
        // `(1/2rho)(max(0, mu - rho g)^2 - mu^2)`: zero and flat where the constraint is slack with
        // a settled multiplier, quadratic where it is violated, and once differentiable at the join —
        // which is what lets a Newton step approach an active constraint instead of chattering on it.
        for (j, g) in evaluation.inequalities.iter().enumerate() {
            let active = (self.mu[j] - self.rho * g.value).max(0.0);
            value += (active * active - self.mu[j] * self.mu[j]) / (2.0 * self.rho);
            for k in 0..n {
                grad[k] -= active * g.grad[k];
            }
        }
        Ok((value, grad, evaluation))
    }

    /// The Hessian of the free block, by central differences of the exact gradient.
    fn hessian(
        &self,
        x: &[f64],
        free: &[usize],
        bounds: &[(f64, f64)],
    ) -> Result<Vec<f64>, ProblemError> {
        let m = free.len();
        let mut h = vec![0.0; m * m];
        for (c, &k) in free.iter().enumerate() {
            let delta = 1e-6 * x[k].abs().max(1.0);
            let mut up = x.to_vec();
            let mut down = x.to_vec();
            up[k] = (x[k] + delta).min(bounds[k].1);
            down[k] = (x[k] - delta).max(bounds[k].0);
            let span = up[k] - down[k];
            if span == 0.0 {
                h[c * m + c] = 1.0;
                continue;
            }
            let (_, gu, _) = self.at(&up)?;
            let (_, gd, _) = self.at(&down)?;
            for (r, &j) in free.iter().enumerate() {
                h[r * m + c] = (gu[j] - gd[j]) / span;
            }
        }
        // The true Hessian is symmetric; differencing is not, and the asymmetry is pure noise.
        for r in 0..m {
            for c in (r + 1)..m {
                let mean = 0.5 * (h[r * m + c] + h[c * m + r]);
                h[r * m + c] = mean;
                h[c * m + r] = mean;
            }
        }
        Ok(h)
    }
}

/// Is this variable free to move, or pinned at a bound the gradient pushes it through?
fn is_free(x: f64, grad: f64, (lower, upper): (f64, f64)) -> bool {
    let at_lower = x <= lower && grad > 0.0;
    let at_upper = x >= upper && grad < 0.0;
    !at_lower && !at_upper
}

/// The stationarity measure: the gradient, with the pinned directions taken out.
fn free_norm(x: &[f64], grad: &[f64], bounds: &[(f64, f64)]) -> f64 {
    (0..x.len())
        .filter(|&k| is_free(x[k], grad[k], bounds[k]))
        .map(|k| grad[k].abs())
        .fold(0.0, f64::max)
}

/// How far outside the feasible set a point is.
fn violation_of(evaluation: &Evaluation) -> f64 {
    let equalities = evaluation
        .equalities
        .iter()
        .map(|h| h.value.abs())
        .fold(0.0, f64::max);
    let inequalities = evaluation
        .inequalities
        .iter()
        .map(|g| (-g.value).max(0.0))
        .fold(0.0, f64::max);
    equalities.max(inequalities)
}

/// Gaussian elimination with partial pivoting; `None` when the matrix is singular.
fn dense_solve(a: &mut [f64], mut b: Vec<f64>, n: usize) -> Option<Vec<f64>> {
    for col in 0..n {
        let mut best = col;
        for row in col + 1..n {
            if a[row * n + col].abs() > a[best * n + col].abs() {
                best = row;
            }
        }
        if a[best * n + col].abs() < 1e-300 {
            return None;
        }
        if best != col {
            for c in 0..n {
                a.swap(col * n + c, best * n + c);
            }
            b.swap(col, best);
        }
        for row in col + 1..n {
            let factor = a[row * n + col] / a[col * n + col];
            if factor == 0.0 {
                continue;
            }
            for c in col..n {
                a[row * n + c] -= factor * a[col * n + c];
            }
            b[row] -= factor * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for col in (0..n).rev() {
        let mut acc = b[col];
        for c in col + 1..n {
            acc -= a[col * n + c] * x[c];
        }
        x[col] = acc / a[col * n + col];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

// ─── the out-of-process backend ─────────────────────────────────────────────────────────────────

/// A backend running in another process, speaking a line protocol that carries no semantics.
///
/// # The protocol
///
/// ```text
///   parent -> child   setup <n> <equalities> <inequalities> <tolerance> <start...> <lower...> <upper...>
///   child  -> parent  eval <x...>
///   parent -> child   <f> <df...> <h...> <dh row-major...> <g...> <dg row-major...>
///   child  -> parent  done <0|1> <iterations> <x...>
/// ```
///
/// Whitespace-separated numbers, one message per line. Deliberately not JSON: this is a private wire
/// between two halves of one run, not an artefact anybody keeps, and a format nobody can mistake for
/// a stored model is the point.
///
/// The child never learns what `x` is. It cannot: the protocol has no names in it.
#[derive(Debug, Clone)]
pub struct External {
    /// The interpreter or executable to run.
    pub program: String,
    /// Its arguments, ending in the driver script.
    pub arguments: Vec<String>,
    /// What the result file should call this backend.
    pub label: String,
    pub tolerance: f64,
}

impl External {
    /// SciPy's SLSQP, driven by `backends/slsqp.py`.
    pub fn slsqp(python: &str, script: &str) -> Self {
        Self {
            program: python.into(),
            arguments: vec![script.into()],
            label: "scipy/slsqp".into(),
            tolerance: 1e-10,
        }
    }
}

impl Backend for External {
    fn name(&self) -> String {
        self.label.clone()
    }

    fn solve(&self, problem: &Problem, start: &[f64]) -> Result<Solved, BackendError> {
        let n = problem.dimension();
        let bounds = problem.bounds();
        let probe = problem.evaluate(start)?;

        let mut child: Child = Command::new(&self.program)
            .args(&self.arguments)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .map_err(|e| BackendError::Channel(format!("{}: {e}", self.program)))?;
        let mut sink = child.stdin.take().expect("piped");
        let mut source = BufReader::new(child.stdout.take().expect("piped"));

        let mut setup = format!(
            "setup {n} {} {} {:e}",
            probe.equalities.len(),
            probe.inequalities.len(),
            self.tolerance
        );
        for value in start.iter() {
            setup.push_str(&format!(" {value:e}"));
        }
        for &(lower, _) in &bounds {
            setup.push_str(&format!(" {lower:e}"));
        }
        for &(_, upper) in &bounds {
            setup.push_str(&format!(" {upper:e}"));
        }
        writeln!(sink, "{setup}").map_err(|e| BackendError::Channel(e.to_string()))?;
        sink.flush().map_err(|e| BackendError::Channel(e.to_string()))?;

        let mut iterations = 0usize;
        let mut line = String::new();
        let (success, x) = loop {
            line.clear();
            let read = source
                .read_line(&mut line)
                .map_err(|e| BackendError::Channel(e.to_string()))?;
            if read == 0 {
                return Err(BackendError::Channel("the backend closed early".into()));
            }
            let mut words = line.split_whitespace();
            match words.next() {
                Some("eval") => {
                    let point: Vec<f64> = words.filter_map(|w| w.parse().ok()).collect();
                    if point.len() != n {
                        return Err(BackendError::Channel(format!(
                            "the backend asked for {} coordinates, the problem has {n}",
                            point.len()
                        )));
                    }
                    let evaluation = problem.evaluate(&point)?;
                    let mut reply = format!("{:e}", evaluation.objective.value);
                    let push = |values: &[f64], reply: &mut String| {
                        for v in values {
                            reply.push_str(&format!(" {v:e}"));
                        }
                    };
                    push(&evaluation.objective.grad, &mut reply);
                    let values: Vec<f64> =
                        evaluation.equalities.iter().map(|h| h.value).collect();
                    push(&values, &mut reply);
                    for h in &evaluation.equalities {
                        push(&h.grad, &mut reply);
                    }
                    let values: Vec<f64> =
                        evaluation.inequalities.iter().map(|g| g.value).collect();
                    push(&values, &mut reply);
                    for g in &evaluation.inequalities {
                        push(&g.grad, &mut reply);
                    }
                    writeln!(sink, "{reply}").map_err(|e| BackendError::Channel(e.to_string()))?;
                    sink.flush()
                        .map_err(|e| BackendError::Channel(e.to_string()))?;
                    iterations += 1;
                }
                Some("done") => {
                    let rest: Vec<&str> = words.collect();
                    if rest.len() < 2 + n {
                        return Err(BackendError::Channel(format!("short result: {line:?}")));
                    }
                    iterations = rest[1].parse().unwrap_or(iterations);
                    let point: Vec<f64> =
                        rest[2..2 + n].iter().filter_map(|w| w.parse().ok()).collect();
                    break (rest[0] == "1", point);
                }
                _ => return Err(BackendError::Channel(format!("unexpected: {line:?}"))),
            }
        };
        drop(sink);
        let _ = child.wait();

        if x.len() != n {
            return Err(BackendError::Channel("the result was not a point".into()));
        }
        let evaluation = problem.evaluate(&x)?;
        let violation = violation_of(&evaluation);
        if !success || violation > self.tolerance {
            return Err(BackendError::NotConverged {
                iterations,
                violation,
                stationarity: f64::NAN,
            });
        }
        Ok(Solved::of(
            problem,
            &evaluation,
            &self.name(),
            true,
            iterations,
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
