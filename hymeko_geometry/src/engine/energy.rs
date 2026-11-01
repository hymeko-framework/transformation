//! Opt-in observation of the existing impulse step; it never supplies forces.
use super::*;

/// Additive impulse-work channels; positive work enters the physical mechanism.
#[derive(Clone, Debug, Default, serde::Serialize)]
pub struct ConstraintWork {
    pub normal_work: f64,
    pub friction_work: f64,
    pub limit_work: f64,
    pub attachment_work: f64,
    /// Removes the h*D numerical armature from the solver's work metric.
    pub implicit_metric_work: f64,
}

/// Cumulative energy accounting since explicit activation, in joules.
/// Work is positive into the mechanism. Viscous work is a nonnegative end-point
/// quadrature. Constraint work uses impulse times mean pre/post-impulse velocity.
/// `residual` includes time-discretization error and is never called dissipation.
#[derive(Clone, Debug, Default, serde::Serialize)]
pub struct EnergyAudit {
    pub complete: bool,
    pub steps: usize,
    pub unobserved_steps: usize,
    pub elapsed: f64,
    pub initial_energy: f64,
    pub energy: f64,
    pub drive_work: f64,
    pub viscous_work: f64,
    #[serde(flatten)]
    pub constraints: ConstraintWork,
    pub impulse_kinetic_change: f64,
    pub maximum_impulse_residual: f64,
    pub residual: f64,
    /// Independent quadratic kinetic increment of the unconstrained Euler step:
    /// sum 1/2 * delta_v_free^T M_physical(q) delta_v_free. Numerical, not friction.
    pub free_velocity_numerical_loss: f64,
    /// Largest raw coordinate-rate increment. Joint units may differ; this is
    /// a diagnostic maximum, not a frame-invariant physical velocity norm.
    pub maximum_free_velocity_increment: f64,
    pub contact_steps: usize,
    pub attachment_steps: usize,
}

pub(super) struct EnergyProbe {
    h: f64,
    before: f64,
    free_velocity: Vec<f64>,
    free_energy: f64,
}

pub(super) struct EnergyStep {
    probe: EnergyProbe,
    drive: f64,
    viscous: f64,
    work: [f64; 4],
    correction: f64,
    impulse_change: f64,
    contact: bool,
    attachment: bool,
    numerical_loss: f64,
    velocity_increment: f64,
}

impl Engine {
    /// Physical H(q,v), including reflected rotor inertia, excluding numerical
    /// h*D armature. Rejects malformed/nonfinite inputs; leaves the engine intact.
    pub fn energy_at(&self, q: &[f64], velocity: &[f64]) -> Option<f64> {
        if q.len() != self.aba.slots()
            || velocity.len() != q.len()
            || q.iter()
                .chain(velocity)
                .chain(&self.gravity)
                .any(|v| !v.is_finite())
        {
            return None;
        }
        let kinetic: f64 = self
            .aba
            .kinetic_energy_by_link(&self.model, q, velocity)
            .iter()
            .sum();
        let rotor: f64 = self
            .rotor_inertia
            .iter()
            .zip(velocity)
            .map(|(i, v)| 0.5 * i * v * v)
            .sum();
        let potential: f64 = self
            .aba
            .potential_energy_by_link(&self.model, q, self.gravity)
            .iter()
            .sum();
        let value = kinetic + rotor + potential;
        value.is_finite().then_some(value)
    }

    /// Start a bounded, observation-only energy ledger at the current state.
    /// Semi-implicit steps are covered. Accepted free-flight steps invalidate
    /// coverage rather than silently omitting their work. No history is retained.
    pub fn enable_energy_audit(&mut self) -> Result<(), String> {
        if self.q.len() > 64 || (0..self.q.len()).any(|i| self.damping_at(i).is_none()) {
            return Err("energy audit requires at most 64 valid joint slots".into());
        }
        let energy = self
            .energy_at(&self.q, &self.qd)
            .ok_or("invalid initial energy")?;
        self.energy_audit = Some(EnergyAudit {
            complete: true,
            initial_energy: energy,
            energy,
            ..Default::default()
        });
        Ok(())
    }

    /// The last cumulative observation, or None if auditing was not enabled.
    pub fn energy_audit(&self) -> Option<&EnergyAudit> {
        self.energy_audit.as_ref()
    }

    pub(super) fn energy_probe(&mut self, h: f64) -> Option<EnergyProbe> {
        self.energy_audit.as_ref()?;
        let before = self.energy_at(&self.q, &self.qd);
        let free_energy = self.energy_at(&self.q, &self.velocity);
        match (before, free_energy) {
            (Some(before), Some(free_energy)) => Some(EnergyProbe {
                h,
                before,
                free_energy,
                free_velocity: self.velocity.clone(),
            }),
            _ => {
                let audit = self.energy_audit.as_mut().unwrap();
                audit.complete = false;
                audit.unobserved_steps += 1;
                None
            }
        }
    }

    pub(super) fn finish_energy_probe(
        &self,
        probe: EnergyProbe,
        limits_start: usize,
        attachment_start: usize,
    ) -> EnergyStep {
        let mut work = [0.; 4];
        for (i, (row, impulse)) in self
            .constraints
            .iter()
            .zip(self.solver.impulses())
            .enumerate()
        {
            let category = if i >= attachment_start {
                3
            } else if i >= limits_start {
                2
            } else if matches!(row.limit, Limit::Friction { .. }) {
                1
            } else {
                0
            };
            work[category] += impulse
                * row
                    .force
                    .iter()
                    .zip(&probe.free_velocity)
                    .zip(&self.velocity)
                    .map(|((force, before), after)| force * 0.5 * (before + after))
                    .sum::<f64>();
        }
        let mut drive = 0.;
        let mut viscous = 0.;
        let mut correction = 0.;
        for (i, (&before, &after)) in probe.free_velocity.iter().zip(&self.velocity).enumerate() {
            let damping = self.damping_at(i).unwrap_or(f64::NAN);
            drive += probe.h * self.torque[i] * after;
            viscous += probe.h * damping * after * after;
            correction -= 0.5 * probe.h * damping * (after * after - before * before);
        }
        let impulse_change =
            self.energy_at(&self.q, &self.velocity).unwrap_or(f64::NAN) - probe.free_energy;
        let increment: Vec<_> = probe
            .free_velocity
            .iter()
            .zip(&self.qd)
            .map(|(a, b)| a - b)
            .collect();
        let zero = vec![0.; increment.len()];
        let mut momentum = zero.clone();
        self.aba.inverse_dynamics(
            &self.model,
            &self.q,
            &zero,
            &increment,
            [0.; 3],
            &mut self.aba.scratch(),
            &mut momentum,
        );
        let numerical_loss = increment
            .iter()
            .zip(momentum)
            .enumerate()
            .map(|(i, (&v, p))| {
                0.5 * v * (p - probe.h * self.damping_at(i).unwrap_or(f64::NAN) * v)
            })
            .sum();
        let velocity_increment = increment.iter().map(|v| v.abs()).fold(0., f64::max);
        EnergyStep {
            probe,
            drive,
            viscous,
            work,
            correction,
            impulse_change,
            contact: limits_start > 0,
            attachment: self.attachment.is_some(),
            numerical_loss,
            velocity_increment,
        }
    }

    pub(super) fn record_energy_step(&mut self, step: EnergyStep) {
        let energy = self.energy_at(&self.q, &self.qd);
        let audit = self.energy_audit.as_mut().unwrap();
        let Some(energy) = energy else {
            audit.complete = false;
            audit.unobserved_steps += 1;
            return;
        };
        if [
            step.drive,
            step.viscous,
            step.correction,
            step.impulse_change,
            step.numerical_loss,
            step.velocity_increment,
        ]
        .iter()
        .chain(&step.work)
        .any(|v| !v.is_finite())
        {
            audit.complete = false;
            audit.unobserved_steps += 1;
            return;
        }
        if (step.probe.before - audit.energy).abs() > 1e-9 * (1. + audit.energy.abs()) {
            audit.complete = false;
        }
        audit.steps += 1;
        audit.elapsed += step.probe.h;
        audit.energy = energy;
        audit.drive_work += step.drive;
        audit.viscous_work += step.viscous;
        audit.constraints.normal_work += step.work[0];
        audit.constraints.friction_work += step.work[1];
        audit.constraints.limit_work += step.work[2];
        audit.constraints.attachment_work += step.work[3];
        audit.constraints.implicit_metric_work += step.correction;
        audit.impulse_kinetic_change += step.impulse_change;
        audit.maximum_impulse_residual = audit
            .maximum_impulse_residual
            .max((step.impulse_change - step.work.iter().sum::<f64>() - step.correction).abs());
        audit.residual = energy - audit.initial_energy - audit.drive_work + audit.viscous_work
            - audit.constraints.normal_work
            - audit.constraints.friction_work
            - audit.constraints.limit_work
            - audit.constraints.attachment_work
            - audit.constraints.implicit_metric_work;
        audit.contact_steps += usize::from(step.contact);
        audit.attachment_steps += usize::from(step.attachment);
        audit.free_velocity_numerical_loss += step.numerical_loss;
        audit.maximum_free_velocity_increment = audit
            .maximum_free_velocity_increment
            .max(step.velocity_increment);
    }
}
