//! Local Alart-Curnier contact equation and safeguarded NSN/projection/NSN solve.
//! Algorithm reference: Siconos 56747c5e9a813db430bce2635159ea650faac98e,
//! fc3d_onecontact_nonsmooth_Newton_solvers.c and fc3d_AlartCurnier_functions.c.
//! Independent Rust implementation: bounded Armijo backtracking and fixed-step
//! De Saxce cone fallback; not a bit-equivalent port of Siconos' Goldstein/PLI.
use super::{Limit, MOBILITY_FLOOR, Problem, admm::project_cone, dense};

#[derive(Default)]
pub(super) struct Work {
    pub newton: usize,
    pub projection: usize,
    pub discarded: usize,
}

pub(super) struct Contact {
    pub rows: [usize; 3],
    matrix: [[f64; 3]; 3],
    mu: f64,
    rho: [f64; 3],
    projection_step: f64,
}

impl Contact {
    pub(super) fn new(problem: &Problem, rows: [usize; 3]) -> Self {
        let n = problem.rhs.len();
        let matrix = rows.map(|i| rows.map(|j| problem.matrix[i * n + j]));
        let mu = match problem.limits[rows[1]] {
            Limit::Friction { mu, .. } => mu,
            _ => 0.0,
        };
        let tangent = 0.5
            * (matrix[1][1]
                + matrix[2][2]
                + (matrix[1][1] - matrix[2][2]).hypot(2.0 * matrix[1][2]));
        let inverse = |d: f64| if d > MOBILITY_FLOOR { 1.0 / d } else { 1.0 };
        let norm = matrix
            .iter()
            .map(|row| row.iter().map(|x| x.abs()).sum::<f64>())
            .fold(MOBILITY_FLOOR, f64::max);
        Self {
            rows,
            matrix,
            mu,
            rho: [inverse(matrix[0][0]), inverse(tangent), inverse(tangent)],
            projection_step: 0.5 / norm / (1.0 + mu),
        }
    }

    pub(super) fn rhs(&self, problem: &Problem, impulse: &[f64]) -> [f64; 3] {
        let n = problem.rhs.len();
        self.rows.map(|k| {
            problem.rhs[k]
                + problem.matrix[k * n..(k + 1) * n]
                    .iter()
                    .zip(impulse)
                    .enumerate()
                    .filter(|(j, _)| !self.rows.contains(j))
                    .map(|(_, (a, p))| a * p)
                    .sum::<f64>()
        })
    }

    fn velocity(&self, p: [f64; 3], rhs: [f64; 3]) -> [f64; 3] {
        std::array::from_fn(|i| {
            rhs[i]
                + self.matrix[i]
                    .iter()
                    .zip(p)
                    .map(|(a, b)| a * b)
                    .sum::<f64>()
        })
    }

    /// Alart-Curnier mapping and one generalized Jacobian. The radius uses
    /// the projected normal; roots satisfy the original non-associated law.
    pub(super) fn equation(&self, p: [f64; 3], rhs: [f64; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
        let v = self.velocity(p, rhs);
        let x: [f64; 3] = std::array::from_fn(|i| p[i] - self.rho[i] * v[i]);
        let dx: [[f64; 3]; 3] = std::array::from_fn(|i| {
            std::array::from_fn(|j| f64::from(i == j) - self.rho[i] * self.matrix[i][j])
        });
        let normal = x[0].max(0.0);
        let dn = if x[0] > 0.0 { dx[0] } else { [0.0; 3] };
        let radius = self.mu * normal;
        let length = x[1].hypot(x[2]);
        let mut f = [p[0] - normal, 0.0, 0.0];
        let mut jacobian = [[0.0; 3]; 3];
        jacobian[0] = std::array::from_fn(|j| f64::from(j == 0) - dn[j]);
        if length <= radius {
            for i in 1..3 {
                f[i] = self.rho[i] * v[i];
                jacobian[i] = self.matrix[i].map(|a| self.rho[i] * a);
            }
        } else {
            let unit = [x[1] / length, x[2] / length];
            for i in 1..3 {
                f[i] = p[i] - radius * unit[i - 1];
                for j in 0..3 {
                    let radial = unit[0] * dx[1][j] + unit[1] * dx[2][j];
                    let projected = self.mu * dn[j] * unit[i - 1]
                        + radius / length * (dx[i][j] - unit[i - 1] * radial);
                    jacobian[i][j] = f64::from(i == j) - projected;
                }
            }
        }
        (f, jacobian)
    }

    /// Physical certificate uses the actual normal impulse, independently of
    /// the projected-normal radius employed by the Newton equation.
    pub(super) fn error(&self, p: [f64; 3], rhs: [f64; 3]) -> f64 {
        if p.iter().chain(&rhs).any(|v| !v.is_finite()) {
            return f64::INFINITY;
        }
        let v = self.velocity(p, rhs);
        if v.iter().any(|v| !v.is_finite()) {
            return f64::INFINITY;
        }
        let x: [f64; 3] = std::array::from_fn(|i| p[i] - self.rho[i] * v[i]);
        let radius = self.mu * p[0].max(0.0);
        let length = x[1].hypot(x[2]);
        let scale = if length > radius {
            radius / length
        } else {
            1.0
        };
        let f = [
            p[0] - x[0].max(0.0),
            p[1] - scale * x[1],
            p[2] - scale * x[2],
        ];
        if f.iter().any(|v| !v.is_finite()) {
            return f64::INFINITY;
        }
        f.iter().fold(0.0f64, |a, v| a.max(v.abs())) / p.iter().fold(1.0f64, |a, v| a.max(v.abs()))
    }

    pub(super) fn solve(&self, p: &mut [f64; 3], rhs: [f64; 3], tolerance: f64, work: &mut Work) {
        self.solve_bounded(p, rhs, tolerance, 10, 20, work);
    }

    pub(super) fn solve_bounded(
        &self,
        p: &mut [f64; 3],
        rhs: [f64; 3],
        tolerance: f64,
        newton_cap: usize,
        projection_cap: usize,
        work: &mut Work,
    ) {
        if rhs[0] >= 0.0 {
            *p = [0.0; 3];
            return;
        }
        self.newton_stage(p, rhs, tolerance, newton_cap, work);
        if self.error(*p, rhs) <= tolerance {
            return;
        }
        self.projection_stage(p, rhs, tolerance, projection_cap, work);
        if self.error(*p, rhs) > tolerance {
            self.newton_stage(p, rhs, tolerance, newton_cap, work);
        }
    }

    fn newton_stage(
        &self,
        p: &mut [f64; 3],
        rhs: [f64; 3],
        tolerance: f64,
        cap: usize,
        work: &mut Work,
    ) {
        let mut trial = *p;
        for _ in 0..cap {
            if self.error(trial, rhs) <= tolerance {
                break;
            }
            work.newton += 1;
            let (f, jacobian) = self.equation(trial, rhs);
            let Some(direction) = Self::newton_direction(jacobian, f) else {
                break;
            };
            let Some(next) = self.line_search(trial, rhs, direction, f) else {
                break;
            };
            trial = next;
        }
        self.keep_improvement(p, trial, rhs, work);
    }

    fn line_search(
        &self,
        p: [f64; 3],
        rhs: [f64; 3],
        direction: [f64; 3],
        f: [f64; 3],
    ) -> Option<[f64; 3]> {
        let merit = f.iter().map(|x| x * x).sum::<f64>();
        if !merit.is_finite() {
            return None;
        }
        let mut alpha = 1.0;
        for _ in 0..12 {
            let trial = std::array::from_fn(|i| p[i] + alpha * direction[i]);
            let value = self
                .equation(trial, rhs)
                .0
                .iter()
                .map(|x| x * x)
                .sum::<f64>();
            if value.is_finite() && value <= (1.0 - 1e-4 * alpha) * merit {
                return Some(trial);
            }
            alpha *= 0.5;
        }
        None
    }

    fn projection_stage(
        &self,
        p: &mut [f64; 3],
        rhs: [f64; 3],
        tolerance: f64,
        cap: usize,
        work: &mut Work,
    ) {
        let mut trial = *p;
        for _ in 0..cap {
            if self.error(trial, rhs) <= tolerance {
                break;
            }
            work.projection += 1;
            let mut v = self.velocity(trial, rhs);
            v[0] += self.mu * v[1].hypot(v[2]);
            trial = project_cone(
                std::array::from_fn(|i| trial[i] - self.projection_step * v[i]),
                self.mu,
            );
            self.keep_improvement(p, trial, rhs, work);
        }
    }

    fn keep_improvement(&self, p: &mut [f64; 3], trial: [f64; 3], rhs: [f64; 3], work: &mut Work) {
        let error = self.error(trial, rhs);
        if error.is_finite() && error <= self.error(*p, rhs) {
            *p = trial;
        } else {
            work.discarded += 1;
        }
    }

    /// Eliminate the three contact Jacobian equations with partial pivoting.
    /// A singular/nonfinite Newton direction triggers the projected fallback.
    fn newton_direction(j: [[f64; 3]; 3], f: [f64; 3]) -> Option<[f64; 3]> {
        let mut matrix = [0.0; 9];
        for (slot, value) in matrix.iter_mut().zip(j.iter().flatten()) {
            *slot = *value;
        }
        let mut direction = f.map(|x| -x);
        dense::solve_in_place(&mut matrix, &mut direction, 3)?;
        Some(direction)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stage_filter_rejects_worse_and_nonfinite_candidates() {
        let problem = super::super::tests::impact(0.8, [-1.0, 0.0, 0.0]);
        let contact = Contact::new(&problem, [0, 1, 2]);
        let rhs = [-1.0, 0.0, 0.0];
        let mut p = [1.0, 0.0, 0.0];
        let mut work = Work::default();
        contact.keep_improvement(&mut p, [3.0, 2.0, 1.0], rhs, &mut work);
        contact.keep_improvement(&mut p, [f64::NAN, 0.0, 0.0], rhs, &mut work);
        assert_eq!(p, [1.0, 0.0, 0.0]);
        assert_eq!(work.discarded, 2);
    }

    #[test]
    fn contact_newton_direction_pivots_and_rejects_singular_or_nonfinite_equations() {
        let jacobian = [[0.0, 2.0, 0.0], [3.0, 0.0, 0.0], [0.0, 0.0, 4.0]];
        assert_eq!(
            Contact::newton_direction(jacobian, [-4.0, -3.0, -12.0]),
            Some([1.0, 2.0, 3.0])
        );
        assert_eq!(Contact::newton_direction([[0.0; 3]; 3], [1.0; 3]), None);
        assert_eq!(
            Contact::newton_direction(jacobian, [f64::INFINITY; 3]),
            None
        );
    }
}
