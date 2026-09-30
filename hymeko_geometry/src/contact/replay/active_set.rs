//! Bounded active-set subspace correction for NSGS residual plateaus (experimental).
//!
//! A Gauss-Seidel phase estimates the index sets and one reduced linear solve on the active set
//! gives a direction: Silcowitz, Niebe and Erleben, "Projected Gauss-Seidel Subspace Minimization
//! Method for Interactive Rigid Body Dynamics", GRAPP 2010; More and Toraldo, SIAM J. Optim. 1
//! (1991) 93-113. Deviations: Coulomb disc cones with non-associated sliding instead of boxes, a
//! ratio test to the first boundary instead of clamping, and direct pivoted elimination instead
//! of CG. A candidate is kept only when the original-law residual strictly decreases; otherwise
//! the Gauss-Seidel iterate continues unchanged.
use super::{Limit, MOBILITY_FLOOR, Problem, Replay, admm::project_cone, dense};

/// Pre-registered in docs/plans/2026-09-13-contact-nsgs (21:36 +02:00). The only revision
/// (22:00 +02:00) dropped a 1e-3 relative decrease margin after candidate 1 rejected every attempt.
pub(super) const WINDOW: usize = 32;
const STALL_RATIO: f64 = 0.9;
pub(super) const MAX_ATTEMPTS: usize = 8;
const STICK_SLACK: f64 = 1e-9;
const MIN_STEP: f64 = 1e-12;

/// Why a candidate was not kept; the iterate is then unchanged.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Rejection {
    Singular,
    Blocked,
    NoDecrease,
}

/// Correction work, counted apart from Gauss-Seidel sweeps and local Newton work.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) struct Work {
    pub attempted: usize,
    pub accepted: usize,
    pub singular: usize,
    pub blocked: usize,
    pub no_decrease: usize,
    /// Sum of reduced unknowns over all attempts.
    pub unknowns: usize,
    /// Arithmetic operations of the reduced dense eliminations.
    pub eliminations: usize,
    /// Dense Delassus products: the velocity and the direction image.
    pub products: usize,
    /// Original-law residual evaluations of candidates.
    pub residuals: usize,
}

/// What one attempt saw; test diagnostics keep a log of these.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(super) struct Attempt {
    pub unknowns: usize,
    pub sticking: usize,
    pub sliding: usize,
    pub unilateral: usize,
    pub inactive: usize,
    pub step: f64,
    /// Row whose boundary limited the step (the normal row for a cone); `None` for a full step.
    pub bound: Option<usize>,
    pub before: f64,
    pub trial: f64,
}

/// One reduced unknown: its equation row and its column of the variable map `E`.
#[derive(Clone, Copy)]
struct Unknown {
    row: usize,
    unilateral: bool,
    column: [(usize, f64); 3],
    entries: usize,
}

impl Unknown {
    fn single(row: usize, unilateral: bool) -> Self {
        Self {
            row,
            unilateral,
            column: [(row, 1.0), (row, 0.0), (row, 0.0)],
            entries: 1,
        }
    }
}

/// A loaded cone; `sliding` holds the fixed unit tangential direction of a saturated cone.
#[derive(Clone, Copy)]
struct Cone {
    rows: [usize; 3],
    mu: f64,
    sliding: Option<[f64; 2]>,
}

pub(super) struct State {
    ring: [f64; WINDOW],
    restart: usize,
    next_allowed: usize,
    rejections: usize,
    unknowns: Vec<Unknown>,
    cones: Vec<Cone>,
    inactive: Vec<usize>,
    handled: Vec<bool>,
    matrix: Vec<f64>,
    solution: Vec<f64>,
    scale: Vec<f64>,
    velocity: Vec<f64>,
    direction: Vec<f64>,
    image: Vec<f64>,
    candidate: Vec<f64>,
    pub work: Work,
    pub last: Attempt,
    #[cfg(test)]
    pub events: Vec<(usize, Attempt, Option<Rejection>)>,
}

impl State {
    /// No allocation until a correction is attempted, so other methods report unchanged buffers.
    pub(super) fn new() -> Self {
        Self {
            ring: [0.0; WINDOW],
            restart: 0,
            next_allowed: 0,
            rejections: 0,
            unknowns: Vec::new(),
            cones: Vec::new(),
            inactive: Vec::new(),
            handled: Vec::new(),
            matrix: Vec::new(),
            solution: Vec::new(),
            scale: Vec::new(),
            velocity: Vec::new(),
            direction: Vec::new(),
            image: Vec::new(),
            candidate: Vec::new(),
            work: Work::default(),
            last: Attempt::default(),
            #[cfg(test)]
            events: Vec::new(),
        }
    }

    pub(super) fn reset(&mut self) {
        self.ring = [0.0; WINDOW];
        self.restart = 0;
        self.next_allowed = 0;
        self.rejections = 0;
        self.work = Work::default();
        self.last = Attempt::default();
        #[cfg(test)]
        self.events.clear();
    }

    /// Record the residual of sweep zero, the first entry of the plateau window.
    pub(super) fn start(&mut self, residual: f64) {
        self.ring[0] = residual;
    }

    pub(super) fn capacity_bytes(&self) -> usize {
        use std::mem::size_of;
        let floats = self.matrix.capacity()
            + self.solution.capacity()
            + self.scale.capacity()
            + self.velocity.capacity()
            + self.direction.capacity()
            + self.image.capacity()
            + self.candidate.capacity();
        floats * size_of::<f64>()
            + self.unknowns.capacity() * size_of::<Unknown>()
            + self.cones.capacity() * size_of::<Cone>()
            + self.inactive.capacity() * size_of::<usize>()
            + self.handled.capacity()
    }

    fn eligible(&self, sweep: usize, residual: f64, past: f64) -> bool {
        sweep >= self.restart + WINDOW
            && sweep >= self.next_allowed
            && self.work.attempted < MAX_ATTEMPTS
            && residual >= STALL_RATIO * past
    }

    /// One bounded correction from the current iterate; on success `impulse` holds the kept
    /// candidate and its original-law residual is returned.
    ///
    /// # Preconditions
    /// `impulse.len() == problem.rhs.len()` and `residual == problem.residual(impulse)`.
    pub(super) fn correct(
        &mut self,
        problem: &Problem,
        impulse: &mut [f64],
        residual: f64,
    ) -> Result<f64, Rejection> {
        debug_assert_eq!(impulse.len(), problem.rhs.len());
        let n = impulse.len();
        self.last = Attempt {
            before: residual,
            trial: f64::NAN,
            step: f64::NAN,
            ..Attempt::default()
        };
        self.velocity.clear();
        self.velocity.resize(n, 0.0);
        problem.velocity(impulse, &mut self.velocity);
        self.work.products += 1;
        self.classify(problem, impulse);
        self.work.unknowns += self.unknowns.len();
        if self.unknowns.is_empty() {
            return Err(Rejection::Blocked);
        }
        self.solve_reduced(problem)?;
        self.direction(problem);
        let (step, bound) = self.ratio(impulse);
        self.last.step = step;
        self.last.bound = bound;
        if !(step.is_finite() && step > MIN_STEP) {
            return Err(Rejection::Blocked);
        }
        self.snap(impulse, step);
        self.work.residuals += 1;
        let trial = problem.residual(&self.candidate);
        self.last.trial = trial;
        if trial.is_finite() && trial < residual {
            impulse.copy_from_slice(&self.candidate);
            Ok(trial)
        } else {
            Err(Rejection::NoDecrease)
        }
    }

    fn classify(&mut self, problem: &Problem, p: &[f64]) {
        let n = p.len();
        self.unknowns.clear();
        self.cones.clear();
        self.inactive.clear();
        self.handled.clear();
        self.handled.resize(n, false);
        for (k, limit) in problem.limits.iter().enumerate() {
            if let Limit::Friction {
                normal,
                partner,
                mu,
            } = *limit
                && k < partner
            {
                for row in [normal, k, partner] {
                    self.handled[row] = true;
                }
                self.classify_cone(problem, [normal, k, partner], mu, p);
            }
        }
        for (k, limit) in problem.limits.iter().enumerate() {
            if self.handled[k] || problem.matrix[k * n + k] <= MOBILITY_FLOOR {
                continue;
            }
            match limit {
                Limit::Free => self.unknowns.push(Unknown::single(k, false)),
                Limit::Pushing if p[k] > 0.0 => self.unknowns.push(Unknown::single(k, true)),
                Limit::Pushing => self.inactive.push(k),
                Limit::Friction { .. } => {}
            }
        }
        self.last.unknowns = self.unknowns.len();
        self.last.unilateral = self.unknowns.iter().filter(|u| u.unilateral).count();
        self.last.inactive = self.inactive.len();
        self.last.sliding = self.cones.iter().filter(|c| c.sliding.is_some()).count();
        self.last.sticking = self.cones.len() - self.last.sliding;
    }

    fn classify_cone(&mut self, problem: &Problem, rows: [usize; 3], mu: f64, p: &[f64]) {
        let n = p.len();
        let mobile = |row: usize| problem.matrix[row * n + row] > MOBILITY_FLOOR;
        let [normal, first, second] = rows;
        if !mobile(normal) {
            return;
        }
        if p[normal] <= 0.0 {
            self.inactive.push(normal);
            return;
        }
        let tangential = mobile(first) && mobile(second);
        let length = p[first].hypot(p[second]);
        let radius = mu * p[normal];
        if tangential && radius - length > STICK_SLACK * radius {
            self.unknowns.push(Unknown::single(normal, true));
            self.unknowns.push(Unknown::single(first, false));
            self.unknowns.push(Unknown::single(second, false));
            self.cones.push(Cone {
                rows,
                mu,
                sliding: None,
            });
        } else if tangential && length > 0.0 {
            let unit = [p[first] / length, p[second] / length];
            self.unknowns.push(Unknown {
                row: normal,
                unilateral: true,
                column: [(normal, 1.0), (first, mu * unit[0]), (second, mu * unit[1])],
                entries: 3,
            });
            self.cones.push(Cone {
                rows,
                mu,
                sliding: Some(unit),
            });
        } else {
            // Immobile or unloaded tangents stay fixed; the normal is still solved with the rest.
            self.unknowns.push(Unknown::single(normal, true));
        }
    }

    /// Jacobi-scaled square system `R A E x = -v_R`, solved by the shared pivoted elimination.
    fn solve_reduced(&mut self, problem: &Problem) -> Result<(), Rejection> {
        let n = problem.rhs.len();
        let m = self.unknowns.len();
        self.scale.clear();
        self.scale.extend(
            self.unknowns
                .iter()
                .map(|u| problem.matrix[u.row * n + u.row].sqrt().recip()),
        );
        self.matrix.clear();
        self.matrix.resize(m * m, 0.0);
        self.solution.clear();
        for (i, equation) in self.unknowns.iter().enumerate() {
            for (j, unknown) in self.unknowns.iter().enumerate() {
                let entry: f64 = unknown.column[..unknown.entries]
                    .iter()
                    .map(|&(column, weight)| problem.matrix[equation.row * n + column] * weight)
                    .sum();
                self.matrix[i * m + j] = entry * self.scale[i] * self.scale[j];
            }
            self.solution
                .push(-self.velocity[equation.row] * self.scale[i]);
        }
        self.work.eliminations += elimination_operations(m);
        dense::solve_in_place(&mut self.matrix, &mut self.solution, m)
            .ok_or(Rejection::Singular)?;
        for (value, scale) in self.solution.iter_mut().zip(&self.scale) {
            *value *= scale;
        }
        Ok(())
    }

    fn direction(&mut self, problem: &Problem) {
        let n = problem.rhs.len();
        self.direction.clear();
        self.direction.resize(n, 0.0);
        for (x, unknown) in self.solution.iter().zip(&self.unknowns) {
            for &(column, weight) in &unknown.column[..unknown.entries] {
                self.direction[column] += x * weight;
            }
        }
        self.image.clear();
        self.image.extend((0..n).map(|k| {
            problem.matrix[k * n..(k + 1) * n]
                .iter()
                .zip(&self.direction)
                .map(|(a, d)| a * d)
                .sum::<f64>()
        }));
        self.work.products += 1;
    }

    /// Largest step in (0, 1] keeping active unilateral rows, sticking cones and inactive
    /// unilateral velocities admissible; the first boundary reached stops the step.
    fn ratio(&self, p: &[f64]) -> (f64, Option<usize>) {
        let d = &self.direction;
        let mut bound = (1.0f64, None);
        for unknown in self.unknowns.iter().filter(|u| u.unilateral) {
            if d[unknown.row] < 0.0 {
                tighten(&mut bound, -p[unknown.row] / d[unknown.row], unknown.row);
            }
        }
        for &k in &self.inactive {
            if self.velocity[k] >= 0.0 && self.image[k] < 0.0 {
                tighten(&mut bound, -self.velocity[k] / self.image[k], k);
            }
        }
        for cone in self.cones.iter().filter(|c| c.sliding.is_none()) {
            tighten(&mut bound, cone_crossing(cone, p, d), cone.rows[0]);
        }
        bound
    }

    /// Candidate `p + t d`; unilateral rows are clamped and cones projected, removing rounding.
    fn snap(&mut self, p: &[f64], step: f64) {
        self.candidate.clear();
        self.candidate
            .extend(p.iter().zip(&self.direction).map(|(p, d)| p + step * d));
        for unknown in self.unknowns.iter().filter(|u| u.unilateral) {
            self.candidate[unknown.row] = self.candidate[unknown.row].max(0.0);
        }
        for cone in &self.cones {
            let [normal, first, second] = cone.rows;
            let block = [
                self.candidate[normal],
                self.candidate[first],
                self.candidate[second],
            ];
            let snapped = match cone.sliding {
                None => project_cone(block, cone.mu),
                Some(unit) => {
                    let radius = cone.mu * block[0];
                    [block[0], radius * unit[0], radius * unit[1]]
                }
            };
            for (row, value) in cone.rows.into_iter().zip(snapped) {
                self.candidate[row] = value;
            }
        }
    }
}

/// Keep the smaller step and the row that imposes it; a NaN ratio never tightens.
fn tighten(bound: &mut (f64, Option<usize>), step: f64, row: usize) {
    if step < bound.0 {
        *bound = (step, Some(row));
    }
}

/// First positive `t` where `||p_t + t d_t|| = mu (p_n + t d_n)` for a cone starting strictly
/// inside; infinity if the line never leaves it.
fn cone_crossing(cone: &Cone, p: &[f64], d: &[f64]) -> f64 {
    let [normal, first, second] = cone.rows;
    let mu2 = cone.mu * cone.mu;
    let quadratic = mu2 * d[normal] * d[normal] - d[first] * d[first] - d[second] * d[second];
    let linear = 2.0 * (mu2 * p[normal] * d[normal] - p[first] * d[first] - p[second] * d[second]);
    let constant = mu2 * p[normal] * p[normal] - p[first] * p[first] - p[second] * p[second];
    first_positive_root(quadratic, linear, constant)
}

/// Smallest positive root of `a t^2 + b t + c` with `c > 0`, using the cancellation-free form.
fn first_positive_root(a: f64, b: f64, c: f64) -> f64 {
    let scale = a.abs().max(b.abs()).max(c.abs());
    if a.abs() <= f64::EPSILON * scale {
        return if b < 0.0 { -c / b } else { f64::INFINITY };
    }
    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return f64::INFINITY;
    }
    let q = -0.5 * (b + b.signum() * discriminant.sqrt());
    let other = if q == 0.0 { f64::INFINITY } else { c / q };
    [q / a, other]
        .into_iter()
        .filter(|t| *t > 0.0)
        .fold(f64::INFINITY, f64::min)
}

/// Divisions and multiply-subtracts of `dense::solve_in_place` for an `m`×`m` system.
fn elimination_operations(m: usize) -> usize {
    (0..m).map(|k| (m - 1 - k) * (m - k + 2)).sum::<usize>() + m * (m + 1) / 2
}

impl Replay {
    /// Record this sweep in the plateau window and, when eligible, attempt one correction.
    /// A rejection doubles the wait; an acceptance restarts the window.
    pub(super) fn plateau_correction(&mut self, sweep: usize, residual: f64) -> f64 {
        let state = &mut self.subspace;
        let past = std::mem::replace(&mut state.ring[sweep % WINDOW], residual);
        if !state.eligible(sweep, residual, past) {
            return residual;
        }
        state.work.attempted += 1;
        let result = state.correct(&self.problem, &mut self.solver.impulse, residual);
        #[cfg(test)]
        state.events.push((sweep, state.last, result.err()));
        match result {
            Ok(corrected) => {
                state.work.accepted += 1;
                state.restart = sweep;
                state.ring[sweep % WINDOW] = corrected;
                corrected
            }
            Err(reason) => {
                match reason {
                    Rejection::Singular => state.work.singular += 1,
                    Rejection::Blocked => state.work.blocked += 1,
                    Rejection::NoDecrease => state.work.no_decrease += 1,
                }
                state.rejections += 1;
                state.next_allowed = sweep + (WINDOW << state.rejections.min(16));
                residual
            }
        }
    }
}
