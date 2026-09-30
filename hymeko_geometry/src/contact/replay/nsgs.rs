//! Nonlinear Gauss-Seidel with Siconos-style local solver handoff and error budget.
use super::{
    Limit, Method, Problem, Replay, Settings,
    local::{Contact, Work},
};

pub(super) struct State {
    contacts: Vec<Contact>,
    pub stats: Work,
    /// Original-law residual evaluations of the sweep loop itself.
    pub residuals: usize,
}

impl State {
    pub(super) fn new(problem: &Problem, method: Method) -> Self {
        let mut contacts = Vec::new();
        if matches!(
            method,
            Method::Nsgs | Method::NsgsFixed | Method::NsgsActiveSet
        ) {
            for (k, limit) in problem.limits.iter().enumerate() {
                if let Limit::Friction {
                    normal, partner, ..
                } = *limit
                    && k < partner
                {
                    contacts.push(Contact::new(problem, [normal, k, partner]));
                }
            }
            contacts.sort_by_key(|c| c.rows[0]);
        }
        Self {
            contacts,
            stats: Work::default(),
            residuals: 0,
        }
    }

    pub(super) fn reset(&mut self) {
        self.stats = Work::default();
        self.residuals = 0;
    }

    pub(super) fn capacity_bytes(&self) -> usize {
        self.contacts.capacity() * std::mem::size_of::<Contact>()
    }
}

/// Inexact local solve: avoid demanding final precision while global coupling
/// still changes. The final physical acceptance tolerance is never relaxed.
pub(super) fn local_tolerance(global: f64, target: f64, adaptive: bool) -> f64 {
    let floor = target * 0.1;
    if adaptive && global.is_finite() {
        (global * 0.1).min(1e-2).max(floor)
    } else {
        floor
    }
}

impl Replay {
    pub(super) fn solve_nsgs(&mut self, settings: Settings) -> (usize, usize) {
        self.observe_nsgs(settings, |_, _, _| {})
    }

    /// Read-only diagnostic seam; callers prepare/reset the replay before entry.
    /// The production specialization has a zero-state no-op observer. The active-set variant
    /// corrects before observation, so a trace shows the residual the next sweep starts from.
    pub(super) fn observe_nsgs(
        &mut self,
        settings: Settings,
        mut observe: impl FnMut(usize, f64, &Self),
    ) -> (usize, usize) {
        if self.problem.rhs.is_empty() {
            observe(0, 0.0, self);
            return (0, 0);
        }
        let mut residual = self.global_residual();
        self.subspace.start(residual);
        observe(0, residual, self);
        for sweep in 1..=settings.max_sweeps {
            let tolerance = local_tolerance(
                residual,
                settings.tolerance,
                self.method != Method::NsgsFixed,
            );
            self.nonlinear_sweep(tolerance);
            residual = self.global_residual();
            if self.method == Method::NsgsActiveSet
                && residual.is_finite()
                && residual > settings.tolerance
            {
                residual = self.plateau_correction(sweep, residual);
            }
            observe(sweep, residual, self);
            if residual <= settings.tolerance || !residual.is_finite() {
                return (sweep, 0);
            }
        }
        (settings.max_sweeps, 0)
    }

    fn global_residual(&mut self) -> f64 {
        self.nonlinear.residuals += 1;
        self.problem.residual(&self.solver.impulse)
    }

    fn nonlinear_sweep(&mut self, tolerance: f64) {
        let mut contact_index = 0;
        for k in 0..self.problem.rhs.len() {
            if let Some(contact) = self.nonlinear.contacts.get(contact_index)
                && contact.rows[0] == k
            {
                let rhs = contact.rhs(&self.problem, &self.solver.impulse);
                let mut impulse = contact.rows.map(|i| self.solver.impulse[i]);
                contact.solve(&mut impulse, rhs, tolerance, &mut self.nonlinear.stats);
                for (i, p) in contact.rows.into_iter().zip(impulse) {
                    self.solver.impulse[i] = p;
                }
                contact_index += 1;
            } else if !matches!(self.problem.limits[k], Limit::Friction { .. }) {
                self.solver.update_row(k);
            }
        }
    }
}
