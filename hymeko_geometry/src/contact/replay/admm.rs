use super::*;

pub(super) struct State {
    z: Vec<f64>,
    dual: Vec<f64>,
    previous: Vec<f64>,
    velocity: Vec<f64>,
    shift: Vec<f64>,
    scale: Vec<f64>,
    physical: Vec<f64>,
}

impl State {
    pub(super) fn new(n: usize) -> Self {
        Self {
            z: vec![0.0; n],
            dual: vec![0.0; n],
            previous: vec![0.0; n],
            velocity: vec![0.0; n],
            shift: vec![0.0; n],
            scale: vec![1.0; n],
            physical: vec![0.0; n],
        }
    }

    pub(super) fn capacity(&self) -> usize {
        self.z.capacity()
            + self.dual.capacity()
            + self.previous.capacity()
            + self.velocity.capacity()
            + self.shift.capacity()
            + self.scale.capacity()
            + self.physical.capacity()
    }

    fn reset(&mut self) {
        self.z.fill(0.0);
        self.dual.fill(0.0);
        self.previous.fill(0.0);
        self.velocity.fill(0.0);
        self.shift.fill(0.0);
        self.physical.fill(0.0);
    }

    fn correction(&mut self, problem: &Problem) {
        problem.velocity(&self.physical, &mut self.velocity);
        self.shift.fill(0.0);
        for (k, &limit) in problem.limits.iter().enumerate() {
            if let Limit::Friction {
                normal,
                partner,
                mu,
            } = limit
                && k < partner
            {
                self.shift[normal] = mu * self.velocity[k].hypot(self.velocity[partner]);
            }
        }
    }

    fn equilibrate(&mut self, problem: &Problem, enabled: bool) {
        self.scale.fill(1.0);
        if !enabled {
            return;
        }
        let n = self.scale.len();
        for k in 0..n {
            let d = problem.matrix[k * n + k];
            self.scale[k] = if d > MOBILITY_FLOOR {
                1.0 / d.sqrt()
            } else {
                1.0
            };
        }
        for (k, &limit) in problem.limits.iter().enumerate() {
            if let Limit::Friction {
                normal, partner, ..
            } = limit
                && k < partner
            {
                let scale = self.scale[normal]
                    .min(self.scale[k])
                    .min(self.scale[partner]);
                self.scale[normal] = scale;
                self.scale[k] = scale;
                self.scale[partner] = scale;
            }
        }
    }

    fn unscale(&mut self) {
        for k in 0..self.z.len() {
            self.physical[k] = self.z[k] * self.scale[k];
        }
    }

    fn project(&mut self, problem: &Problem) {
        for (k, &limit) in problem.limits.iter().enumerate() {
            match limit {
                Limit::Pushing => self.z[k] = self.z[k].max(0.0),
                Limit::Friction {
                    normal,
                    partner,
                    mu,
                } if k < partner => {
                    // Read the original normal from `previous`: normal-row projection above
                    // must not change the vector whose Euclidean cone projection is requested.
                    let p = project_cone(
                        [
                            self.previous[normal],
                            self.previous[k],
                            self.previous[partner],
                        ],
                        mu,
                    );
                    self.z[normal] = p[0];
                    self.z[k] = p[1];
                    self.z[partner] = p[2];
                }
                _ => {}
            }
        }
    }
}

/// Euclidean projection on ||p_t|| <= mu p_n, p_n >= 0.
pub(super) fn project_cone(p: [f64; 3], mu: f64) -> [f64; 3] {
    let t = p[1].hypot(p[2]);
    if p[0] >= 0.0 && t <= mu * p[0] {
        return p;
    }
    if p[0] + mu * t <= 0.0 {
        return [0.0; 3];
    }
    let normal = (p[0] + mu * t) / (1.0 + mu * mu);
    let scale = if t > 0.0 { mu * normal / t } else { 0.0 };
    [normal, scale * p[1], scale * p[2]]
}

impl Replay {
    pub(super) fn solve_admm(&mut self, settings: Settings) -> (usize, usize) {
        let n = self.problem.rhs.len();
        if n == 0 {
            return (0, 0);
        }
        self.state.reset();
        self.state.equilibrate(&self.problem, settings.equilibrate);
        for k in 0..n {
            // Zero for pure ADMM, the explicitly counted partial PGS result for Hybrid.
            self.state.physical[k] = self.solver.impulse[k];
            self.solver.impulse[k] /= self.state.scale[k];
            self.state.z[k] = self.solver.impulse[k];
        }
        for k in 0..n {
            for j in 0..n {
                self.solver.a[k * n + j] *= self.state.scale[k] * self.state.scale[j];
            }
        }
        let rho = (0..n).map(|i| self.solver.a[i * n + i]).sum::<f64>() / n as f64;
        if rho <= MOBILITY_FLOOR {
            return (0, 0);
        }
        for k in 0..n {
            self.solver.a[k * n + k] += rho;
            if matches!(self.solver.limits[k], Limit::Friction { .. }) {
                self.solver.limits[k] = Limit::Free;
            }
        }
        let (mut sweeps, mut iterations) = (0, 0);
        while sweeps < settings.max_sweeps {
            self.state.correction(&self.problem);
            for k in 0..n {
                self.solver.b[k] = self.state.scale[k]
                    * (self.problem.rhs[k] + self.state.shift[k])
                    - rho * (self.state.z[k] - self.state.dual[k]);
            }
            let params = ContactParameters {
                iterations: 16.min(settings.max_sweeps - sweeps),
                tolerance: 1e-10,
                ..Default::default()
            };
            self.solver.iterate(&params, Some(&self.order));
            sweeps += self.solver.sweeps;
            iterations += 1;
            self.consensus_step();
            self.state.unscale();
            if self.problem.residual(&self.state.physical) <= settings.tolerance {
                break;
            }
        }
        self.solver.impulse.clone_from(&self.state.physical);
        (sweeps, iterations)
    }

    fn consensus_step(&mut self) {
        for k in 0..self.state.z.len() {
            self.state.previous[k] = self.solver.impulse[k] + self.state.dual[k];
        }
        self.state.z.clone_from(&self.state.previous);
        self.state.project(&self.problem);
        for k in 0..self.state.z.len() {
            self.state.dual[k] += self.solver.impulse[k] - self.state.z[k];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::project_cone;

    #[test]
    fn cone_projection_is_feasible_idempotent_and_tangent_rotation_equivariant() {
        for mu in [0.0, 0.2, 0.8, 2.0] {
            for normal in [-3.0, 0.0, 1.0, 4.0] {
                for tangential in [-5.0, -0.1, 0.0, 0.3, 2.0] {
                    let p = [normal, tangential, tangential * 0.7];
                    let q = project_cone(p, mu);
                    assert!(q[0] >= 0.0 && q[1].hypot(q[2]) <= mu * q[0] + 1e-12);
                    let again = project_cone(q, mu);
                    let rotated = project_cone([p[0], -p[2], p[1]], mu);
                    for (a, b) in again.iter().zip(q) {
                        assert!((a - b).abs() < 1e-12);
                    }
                    assert!((rotated[0] - q[0]).abs() < 1e-12);
                    assert!((rotated[1] + q[2]).abs() < 1e-12);
                    assert!((rotated[2] - q[1]).abs() < 1e-12);
                }
            }
        }
    }
}
