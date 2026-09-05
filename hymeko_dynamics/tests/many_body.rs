//! Many bodies, and an answer that was known before the code was written.
//!
//! `dynamics/lattice_chain.hymeko` declares six masses on seven springs between two walls: thirteen
//! coordinates assembled from twelve ports. Nothing in the projection knows what a chain is. It
//! reads signed incidences, builds `J`, and solves an eigenvalue problem.
//!
//! A chain of `N` equal masses `m` on equal springs `k`, fixed at both ends, has
//!
//! ```text
//! omega_n = 2 sqrt(k/m) sin( n pi / (2(N+1)) ),      n = 1 .. N
//! ```
//!
//! which is arithmetic on paper. Comparing the two checks the assembly, the eigen solver and the
//! sign convention at once: get one incidence backwards and the spectrum is not a chain's.
//!
//! The mechanical model uses the SAME vocabulary as the circuits, because it is the same structure —
//! a mass sits at a junction the way a node's capacitance does, a spring sits on a branch the way an
//! inductance does — so these tests are also a check that generalising the vocabulary did not
//! quietly specialise the projection to circuits.

mod common;

use hymeko_dynamics::analysis::{Complex, eigenvalues, modes};
use hymeko_dynamics::run::{run, Integrator};
use hymeko_dynamics::System;

const BODIES: usize = 6;
const MASS: f64 = 0.5;
const STIFFNESS: f64 = 200.0; // 1 / 0.005

/// The chain as shipped, or `None` where `core` is not in this checkout.
fn chain() -> Option<System> {
    let path = common::model_path("lattice_chain.hymeko");
    if !path.exists() {
        return None;
    }
    let (system, _, _) = hymeko_dynamics::project_file(&path).expect("it compiles and projects");
    Some(system)
}

/// The same, with the drag taken out, so a frequency is a frequency and not a damped one.
fn lossless() -> Option<System> {
    common::variant_of(
        "lattice_chain.hymeko",
        &[(
            "damping:    kind.parameter { unit pu.newton_second_per_metre; value 0.35; }",
            "damping:    kind.parameter { unit pu.newton_second_per_metre; value 0.0; }",
        )],
    )
}

/// `omega_n = 2 sqrt(k/m) sin(n pi / (2(N+1)))`, in hertz, ascending.
fn closed_form(stiffness: f64, mass: f64, bodies: usize) -> Vec<f64> {
    let scale = 2.0 * (stiffness / mass).sqrt();
    (1..=bodies)
        .map(|n| {
            scale * (n as f64 * std::f64::consts::PI / (2.0 * (bodies + 1) as f64)).sin()
                / std::f64::consts::TAU
        })
        .collect()
}

fn frequencies(system: &System) -> Vec<f64> {
    let mut found: Vec<f64> = modes(system)
        .expect("the chain has modes")
        .iter()
        .map(|m| m.frequency_hz)
        .collect();
    found.sort_by(f64::total_cmp);
    found
}

// ─── the spectrum ───────────────────────────────────────────────────────────────────────────────

#[test]
fn the_chain_reproduces_the_closed_form_spectrum() {
    let Some(system) = lossless() else { return };
    assert_eq!(system.dim(), 2 * BODIES + 1, "six momenta and seven extensions");

    let found = frequencies(&system);
    let want = closed_form(STIFFNESS, MASS, BODIES);
    assert_eq!(found.len(), want.len(), "one mode per body: {found:?}");

    let mut worst = 0.0f64;
    for (got, expected) in found.iter().zip(&want) {
        worst = worst.max((got - expected).abs() / expected);
        println!("  {got:10.5} Hz   closed form {expected:10.5} Hz");
    }
    assert!(
        worst < 1e-9,
        "the assembled spectrum differs from the chain's by {worst:e} relative"
    );
}

/// Independent matrices with the exact spectrum `0, ..., 0, +/- i omega_j`.
/// These checks need neither a source fixture nor the dynamics projection.
struct OscillatorSpectrum {
    angular_frequencies: Vec<f64>,
    zero_roots: usize,
}

impl OscillatorSpectrum {
    fn new(angular_frequencies: &[f64], zero_roots: usize) -> Self {
        Self {
            angular_frequencies: angular_frequencies.to_vec(),
            zero_roots,
        }
    }

    fn dim(&self) -> usize {
        2 * self.angular_frequencies.len() + self.zero_roots
    }

    fn verify(&self, scale: f64, order: &[usize]) {
        let n = self.dim();
        let mut matrix = vec![0.0; n * n];
        for (k, &omega) in self.angular_frequencies.iter().enumerate() {
            matrix[(2 * k) * n + 2 * k + 1] = -omega;
            matrix[(2 * k + 1) * n + 2 * k] = omega;
        }
        let mut permuted = vec![0.0; n * n];
        for row in 0..n {
            for column in 0..n {
                permuted[row * n + column] = scale * matrix[order[row] * n + order[column]];
            }
        }
        let roots = eigenvalues(&permuted, n).expect("the analytic spectrum is trustworthy");
        assert_eq!(roots.len(), n, "algebraic multiplicity is preserved");
        assert!(
            roots
                .iter()
                .all(|root| root.re.is_finite() && root.im.is_finite()),
            "finite analytic roots at scale {scale:e}: {roots:?}"
        );
        assert_eq!(
            roots.iter().filter(|&&root| root == Complex::ZERO).count(),
            self.zero_roots,
            "exact stationary roots at scale {scale:e}, order {order:?}: {roots:?}"
        );
        let mut nonzero: Vec<_> = roots
            .into_iter()
            .filter(|&root| root != Complex::ZERO)
            .collect();
        nonzero.sort_by(|a, b| a.im.total_cmp(&b.im));
        let mut expected: Vec<_> = self
            .angular_frequencies
            .iter()
            .flat_map(|&omega| [-scale * omega, scale * omega])
            .collect();
        expected.sort_by(f64::total_cmp);
        assert_eq!(
            nonzero.len(),
            expected.len(),
            "one conjugate pair per oscillator"
        );
        for (root, omega) in nonzero.iter().zip(expected) {
            assert!(
                root.re.abs() / omega.abs() < 1e-9 && (root.im - omega).abs() / omega.abs() < 1e-9,
                "expected 0 + {omega:e}i at scale {scale:e}, order {order:?}; got {root:?}"
            );
        }
    }
}

#[test]
fn analytic_zero_matrices_have_only_exact_zero_roots() {
    for n in [1, 2, 5] {
        OscillatorSpectrum::new(&[], n).verify(1.0, &(0..n).collect::<Vec<_>>());
    }
}

#[test]
fn analytic_oscillators_preserve_zero_multiplicity() {
    for zero_roots in [1, 3] {
        let spectrum = OscillatorSpectrum::new(&[2.0], zero_roots);
        spectrum.verify(1.0, &(0..spectrum.dim()).collect::<Vec<_>>());
    }
}

#[test]
fn analytic_tiny_nonzero_frequency_is_not_a_zero_root() {
    for zero_roots in [0, 2] {
        let spectrum = OscillatorSpectrum::new(&[2.0_f64.powi(-40)], zero_roots);
        spectrum.verify(1.0, &(0..spectrum.dim()).collect::<Vec<_>>());
    }
}

#[test]
fn analytic_spectra_scale_and_permute_without_changing_modes() {
    let spectrum = OscillatorSpectrum::new(&[1.0, 2.0, 4.0], 3);
    let n = spectrum.dim();
    let orders = [
        (0..n).collect::<Vec<_>>(),
        (0..n).rev().collect(),
        (0..n).map(|k| (k + 3) % n).collect(),
    ];
    for scale in [2.0_f64.powi(-20), 1.0, 2.0_f64.powi(20)] {
        for order in &orders {
            spectrum.verify(scale, order);
        }
    }
}

/// Stiffening every spring together scales the whole spectrum and changes nothing about its shape.
///
/// The ratios between the modes are set by the chain's LENGTH alone — they are `sin(n pi/14)` — so a
/// sweep over the compliance may move every frequency and no ratio. A projection that had quietly
/// tied a frequency to something else would break the ratios, not the scale.
#[test]
fn stiffening_moves_the_spectrum_and_not_its_shape() {
    let Some(system) = lossless() else { return };
    let base = frequencies(&system);

    for factor in [0.25_f64, 4.0] {
        // `compliance` is `1/k`, so a stiffer chain is a SMALLER compliance.
        let stiffer = system.at(
            system
                .sweeps
                .first()
                .expect("the model declares a sweep")
                .parameter,
            0.005 / factor,
        );
        let moved = frequencies(&stiffer);
        for (got, was) in moved.iter().zip(&base) {
            assert!(
                (got / was - factor.sqrt()).abs() < 1e-9,
                "every frequency should scale as sqrt(k): {got} vs {was} at factor {factor}"
            );
        }
        // And the shape: each ratio to the lowest mode is untouched.
        for (got, was) in moved.iter().zip(&base) {
            let (a, b) = (got / moved[0], was / base[0]);
            assert!((a - b).abs() < 1e-9, "the mode SHAPE moved: {a} vs {b}");
        }
    }
}

// ─── the walk, on a mechanism ───────────────────────────────────────────────────────────────────

/// A probe on a body reads the velocity that body's own momentum already holds.
///
/// The probe walks the springs from the reference wall and adds up what each carries; the readout
/// beside it divides a momentum by a mass. Different declarations, different arithmetic, one
/// velocity — and the walk is the circuit's, unchanged.
#[test]
fn a_probe_on_a_body_reads_what_its_momentum_holds() {
    let Some(system) = chain() else { return };
    let column = |name: &str| {
        system
            .observables
            .iter()
            .position(|o| o.name.ends_with(name))
            .unwrap_or_else(|| panic!("the model declares {name}"))
    };

    let trace = run(
        &system,
        &vec![0.0; system.dim()],
        2.0,
        1.0e-4,
        4,
        Integrator::DiscreteGradient,
    );
    let scale = trace.columns[column("v_3")]
        .iter()
        .fold(0.0f64, |m, v| m.max(v.abs()));
    let worst = trace.columns[column("v_probe")]
        .iter()
        .zip(&trace.columns[column("v_3")])
        .fold(0.0f64, |m, (a, b)| m.max((a - b).abs()));

    assert!(scale > 1e-3, "the chain actually moves: {scale} m/s");
    assert!(
        worst < 1e-11 * scale,
        "the probe and the momentum disagree by {worst} on a {scale} m/s body"
    );
}

/// The far wall comes out at rest, by walking seven springs to reach it.
///
/// Only ONE wall is declared as the reference. That the other is also stationary is not an
/// assumption anywhere in the model — it is what the extensions have to add up to, and it is the
/// mechanical reading of the loops closing.
#[test]
fn the_far_wall_is_still() {
    let Some(system) = chain() else { return };
    let far = system
        .terminals
        .iter()
        .position(|t| t.name.ends_with("wall_b"))
        .expect("the model declares the far wall");

    let trace = run(
        &system,
        &vec![0.0; system.dim()],
        1.0,
        1.0e-4,
        4096,
        Integrator::DiscreteGradient,
    );
    let state = trace.final_state;
    let potentials = system.potentials(&state, 1.0);
    let moving = system
        .potentials(&state, 1.0)
        .iter()
        .flatten()
        .fold(0.0f64, |m, v| m.max(v.abs()));

    let wall = potentials[far].expect("the walls are joined by the chain");
    assert!(moving > 1e-3, "something is moving: {moving} m/s");
    assert!(
        wall.abs() < 1e-9 * moving,
        "the far wall moves at {wall} m/s while the chain moves at {moving}"
    );
}

/// A probe with two leads reads the difference, and the leads are seven springs apart.
#[test]
fn a_probe_across_the_chain_is_the_difference_of_its_ends() {
    let Some(system) = chain() else { return };
    let column = |name: &str| {
        system
            .observables
            .iter()
            .position(|o| o.name.ends_with(name))
            .expect(name)
    };
    let trace = run(
        &system,
        &vec![0.0; system.dim()],
        2.0,
        1.0e-4,
        4,
        Integrator::DiscreteGradient,
    );
    let (across, first, last) = (column("v_across"), column("v_1"), column("v_6"));
    let mut worst = 0.0f64;
    let mut scale = 0.0f64;
    for k in 0..trace.times.len() {
        let want = trace.columns[last][k] - trace.columns[first][k];
        worst = worst.max((trace.columns[across][k] - want).abs());
        scale = scale.max(want.abs());
    }
    assert!(scale > 1e-3, "the two ends move differently: {scale} m/s");
    assert!(worst < 1e-11 * scale, "the span reads {worst} off the difference");
}

// ─── and it is still a port-Hamiltonian system ──────────────────────────────────────────────────

/// The energy balance holds on a mechanism exactly as it does on a circuit, because it is the same
/// property of the same `J`.
#[test]
fn the_energy_balance_holds_on_a_mechanism() {
    let Some(system) = chain() else { return };
    let trace = run(
        &system,
        &vec![0.0; system.dim()],
        2.0,
        1.0e-4,
        4096,
        Integrator::DiscreteGradient,
    );
    println!("worst balance residual over two seconds: {:e}", trace.worst_balance);
    assert!(
        trace.worst_balance < 1e-18,
        "the discrete gradient should keep the balance to round-off, not {:e}",
        trace.worst_balance
    );
}

/// A lossless chain, left alone, keeps its energy.
#[test]
fn a_lossless_chain_left_alone_keeps_its_energy() {
    let Some(system) = common::variant_of(
        "lattice_chain.hymeko",
        &[
            (
                "damping:    kind.parameter { unit pu.newton_second_per_metre; value 0.35; }",
                "damping:    kind.parameter { unit pu.newton_second_per_metre; value 0.0; }",
            ),
            ("shake_amplitude: kind.parameter { unit pu.newton; value 6.0; }",
             "shake_amplitude: kind.parameter { unit pu.newton; value 0.0; }"),
        ],
    ) else {
        return;
    };

    // Plucked: one body pushed, everything else at rest.
    let mut start = vec![0.0; system.dim()];
    start[2] = 0.4; // p_3, a momentum
    let before = system.energy(&start);

    let trace = run(&system, &start, 4.0, 1.0e-4, 4096, Integrator::DiscreteGradient);
    let after = system.energy(&trace.final_state);
    let drift = (after - before).abs() / before;
    println!("energy {before:.9} J -> {after:.9} J, drift {drift:e}");
    assert!(before > 0.0);
    // Forty thousand steps of round-off, which is what this is: the BALANCE — what the integrator
    // actually preserves — holds to 1e-18 per step, and the drift is that accumulated. A tighter
    // bound here would be a bound on floating point, not on the scheme.
    assert!(
        drift < 1e-10,
        "a chain with no drag and no shaker drifted {drift:e} in energy"
    );
}
