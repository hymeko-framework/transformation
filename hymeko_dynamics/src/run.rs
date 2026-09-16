//! Running a declared system, and what a run records.
//!
//! Two integrators, kept side by side because the difference between them is the whole claim the
//! port-Hamiltonian structure makes:
//!
//! * [`Integrator::DiscreteGradient`] — for a quadratic Hamiltonian this IS the implicit midpoint
//!   rule, and the energy balance holds to machine precision at EVERY step. Not approximately, and
//!   not with a drift that shrinks with the step: the skew part of `J` cancels in the discrete
//!   equation for the same reason it cancels in the continuous one.
//! * [`Integrator::Rk4`] — accurate and structure-blind. It has no conservation law of its own, so
//!   the energy error accumulates with the number of steps.
//!
//! Running both and comparing the balance residual is how the structure is checked rather than
//! asserted. A correct implementation shows one flat line and one that grows.

use crate::system::System;

/// How the state is stepped. Not *what* the system is — that is the declaration — but how its
/// trajectory is computed, which is a separate question with a separate answer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Integrator {
    /// Structure preserving. One linear solve per step.
    #[default]
    DiscreteGradient,
    /// Classical explicit Runge-Kutta, fourth order. Four flow evaluations per step.
    Rk4,
}

/// What a run recorded.
#[derive(Debug, Clone, PartialEq)]
pub struct Trace {
    pub times: Vec<f64>,
    /// One column per observable, in the system's own order.
    pub columns: Vec<Vec<f64>>,
    /// `H(x)` at each recorded time.
    pub energy: Vec<f64>,
    /// The largest and smallest each observable reached, over EVERY step rather than every recorded
    /// one.
    ///
    /// A peak read off the decimated columns is the largest SAMPLE, which is a different number: the
    /// same run reported a 59% overshoot at one stride and 54.6% at another, and only one of those is
    /// the trajectory's. Tracking it as the run goes costs one evaluation per step and makes the
    /// answer independent of how often anybody chose to write a row down.
    pub peak: Vec<f64>,
    pub trough: Vec<f64>,
    /// The worst power-balance residual over the whole run, relative to the run's characteristic
    /// power. On a structure-preserving step this is rounding and STAYS rounding.
    pub worst_balance: f64,
    /// The final state, so a run can be continued.
    pub final_state: Vec<f64>,
}

impl Trace {
    /// The largest and smallest one observable reached over the whole run.
    pub fn extent(&self, observable: usize) -> (f64, f64) {
        (self.trough[observable], self.peak[observable])
    }

    /// The amplitude of one observable over the last fraction of the run.
    ///
    /// Half the peak-to-peak rather than a maximum of the absolute value: a signal with a DC offset
    /// has a larger maximum than amplitude, and a steady state driven through a resonance can carry
    /// one. Measured over the tail so the transient has decayed.
    pub fn amplitude(&self, observable: usize, tail: f64) -> f64 {
        let column = &self.columns[observable];
        let from = ((1.0 - tail.clamp(0.0, 1.0)) * column.len() as f64) as usize;
        let slice = &column[from.min(column.len().saturating_sub(1))..];
        let high = slice.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let low = slice.iter().copied().fold(f64::INFINITY, f64::min);
        0.5 * (high - low)
    }

    /// How far the energy wandered, relative to its own scale over the run.
    pub fn energy_drift(&self) -> f64 {
        let high = self
            .energy
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        let low = self.energy.iter().copied().fold(f64::INFINITY, f64::min);
        (high - low) / high.abs().max(f64::MIN_POSITIVE)
    }
}

/// Step a declared system and record the observables.
///
/// `stride` records every n-th step; the balance residual is measured at EVERY step regardless,
/// because a scheme that loses energy once in a thousand steps must not be able to hide between two
/// recorded samples.
pub fn run(
    system: &System,
    initial: &[f64],
    duration: f64,
    step: f64,
    stride: usize,
    integrator: Integrator,
) -> Trace {
    let n = system.dim();
    let steps = (duration / step).ceil().max(0.0) as usize;
    let stride = stride.max(1);

    let mut x = initial.to_vec();
    let mut before = vec![0.0; n];
    let mut input = vec![0.0; n];

    let mut trace = Trace {
        times: Vec::with_capacity(steps / stride + 1),
        columns: vec![Vec::with_capacity(steps / stride + 1); system.observables.len()],
        energy: Vec::with_capacity(steps / stride + 1),
        peak: vec![f64::NEG_INFINITY; system.observables.len()],
        trough: vec![f64::INFINITY; system.observables.len()],
        worst_balance: 0.0,
        final_state: Vec::new(),
    };

    // The scale the balance residual is measured against: the energy the drive could put in over one
    // cycle, or the energy already present, whichever is larger. A residual normalised by a system
    // that happens to start at rest would be dividing by zero.
    let scale = system
        .energy(initial)
        .max(drive_energy_scale(system, duration))
        .max(f64::MIN_POSITIVE);

    let record = |trace: &mut Trace, time: f64, x: &[f64]| {
        trace.times.push(time);
        for (column, value) in trace.columns.iter_mut().zip(system.observe_at(x, time)) {
            column.push(value);
        }
        trace.energy.push(system.energy(x));
    };
    let extremes = |trace: &mut Trace, time: f64, x: &[f64]| {
        for (k, value) in system.observe_at(x, time).into_iter().enumerate() {
            trace.peak[k] = trace.peak[k].max(value);
            trace.trough[k] = trace.trough[k].min(value);
        }
    };
    record(&mut trace, 0.0, &x);
    extremes(&mut trace, 0.0, &x);

    for k in 0..steps {
        let time = k as f64 * step;
        before.copy_from_slice(&x);
        match integrator {
            Integrator::DiscreteGradient => {
                // Held at the midpoint, which is what makes the balance exact rather than
                // first-order in the input's own variation across the step.
                system.input_at(time + 0.5 * step, &mut input);
                system.port_hamiltonian.step(&mut x, &input, step, 1e-14);
            }
            Integrator::Rk4 => rk4(system, &mut x, time, step),
        }
        system.input_at(time + 0.5 * step, &mut input);
        let residual = system
            .port_hamiltonian
            .balance_residual(&before, &x, &input, step, scale);
        trace.worst_balance = trace.worst_balance.max(residual);
        // `x` is the state AFTER the step, so the readouts that depend on the drive are asked for
        // the time it now stands at rather than the one it came from.
        extremes(&mut trace, time + step, &x);

        if (k + 1) % stride == 0 {
            record(&mut trace, time + step, &x);
        }
    }
    trace.final_state = x;
    trace
}

/// Classical RK4 on `x' = (J - R) Q x + u(t)`.
fn rk4(system: &System, x: &mut [f64], time: f64, step: f64) {
    let n = system.dim();
    let mut input = vec![0.0; n];
    let mut work = vec![0.0; n];

    let mut derivative = |t: f64, state: &[f64], out: &mut [f64]| {
        system.input_at(t, &mut input);
        system.port_hamiltonian.flow(state, &input, out);
    };

    let mut k1 = vec![0.0; n];
    derivative(time, x, &mut k1);
    for i in 0..n {
        work[i] = x[i] + 0.5 * step * k1[i];
    }
    let mut k2 = vec![0.0; n];
    derivative(time + 0.5 * step, &work, &mut k2);
    for i in 0..n {
        work[i] = x[i] + 0.5 * step * k2[i];
    }
    let mut k3 = vec![0.0; n];
    derivative(time + 0.5 * step, &work, &mut k3);
    for i in 0..n {
        work[i] = x[i] + step * k3[i];
    }
    let mut k4 = vec![0.0; n];
    derivative(time + step, &work, &mut k4);

    for i in 0..n {
        x[i] += step / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
}

/// A characteristic energy for a driven system that may start at rest.
///
/// A quarter period of the drive is the natural yardstick — except at DC, where there is no period.
/// Dividing by a zero frequency there gives an infinite scale and therefore a balance residual that
/// is identically zero however wrong the step is, which silently disables the one check the
/// structure-preserving claim rests on. So the frequency is floored at `1/duration`: over a run of
/// length `T` the slowest thing that can happen is one cycle of it.
fn drive_energy_scale(system: &System, duration: f64) -> f64 {
    let slowest = 1.0 / duration.max(f64::MIN_POSITIVE);
    system
        .drives
        .iter()
        .map(|d| {
            let capacity = system.coordinates[d.coordinate].capacity;
            // The state a drive of this amplitude reaches in a quarter period, squared over the
            // store: an order of magnitude, not a bound, and only ever a denominator.
            let reach = d.amplitude / (4.0 * d.frequency_hz.max(slowest));
            0.5 * reach * reach / capacity
        })
        .fold(0.0, f64::max)
}
