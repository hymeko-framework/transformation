//! The declared Maxwell-RLC system, checked against three independent things.
//!
//! 1. **A hand-written implementation.** `hymeko_simulator::electro::port_hamiltonian` transcribes
//!    the same system as a literal 4x4 and a hand-written rate function. The assembled `J - R` must
//!    reproduce it entry for entry — not merely have the same symmetry.
//! 2. **Closed-form physics.** The lossless normal modes solve a biquadratic, and the coupling
//!    enters its sum but not its product. Both are exact and neither is in the code under test.
//! 3. **Itself, along a different path.** The eigenvalue analysis and the time stepping share the
//!    assembled system and nothing else. If the assembly were wrong they would disagree, because
//!    nothing in either path could compensate for a mistake in the other.

mod common;

use common::Parameters;
use hymeko_dynamics::analysis::{modes, response};
use hymeko_dynamics::run::{Integrator, run};

macro_rules! system {
    () => {
        match common::maxwell() {
            Some(s) => s,
            None => {
                eprintln!("skipped: core/data/dynamics is not in this checkout");
                return;
            }
        }
    };
}

macro_rules! variant {
    ($edits:expr) => {
        match common::variant($edits) {
            Some(s) => s,
            None => {
                eprintln!("skipped: core/data/dynamics is not in this checkout");
                return;
            }
        }
    };
}

/// Zeroing every loss and the drive: what the closed-form oracles are derived for.
const LOSSLESS: &[(&str, &str)] = &[
    (
        "R: kind.parameter { unit u.ohm;    value 12.0; }",
        "R: kind.parameter { unit u.ohm;    value 0.0; }",
    ),
    (
        "sigma:   kind.parameter { unit pu.siemens; value 0.00025; }",
        "sigma:   kind.parameter { unit pu.siemens; value 0.0; }",
    ),
    (
        "source_amplitude: kind.parameter { unit u.volt;   value 10.0; }",
        "source_amplitude: kind.parameter { unit u.volt;   value 0.0; }",
    ),
];

// ─── 1. against the hand-written implementation ─────────────────────────────────────────────────

/// The assembled `J - R` reproduces the simulator's literal 4x4.
///
/// ```text
///     [  0    1    0    0 ]        R = diag(0, resistance, conductance, 0)
///     [ -1   -R    k    0 ]
///     [  0   -k   -s    g ]
///     [  0    0   -g    0 ]
/// ```
///
/// Written out in `hymeko_simulator::electro::port_hamiltonian::structure_matrix` and assembled here
/// from three declared ports and two declared losses. Reproducing the SYMMETRY would not be enough —
/// any skew matrix has it — so this compares every entry.
#[test]
fn the_assembled_structure_reproduces_the_hand_written_matrix() {
    let system = system!();
    let p = Parameters::DECLARED;
    let (r, sigma) = (12.0, 0.00025);

    let j = system.port_hamiltonian.interconnection.dense();
    let want = [
        0.0, 1.0, 0.0, 0.0, //
        -1.0, 0.0, p.k, 0.0, //
        0.0, -p.k, 0.0, p.g, //
        0.0, 0.0, -p.g, 0.0,
    ];
    assert_eq!(j, want, "the declared chain is not the hand-written J");
    assert_eq!(
        system.port_hamiltonian.dissipation,
        vec![0.0, r, sigma, 0.0]
    );

    // `Q = diag(1/C, 1/L, 1/Cf, 1/Lf)` — the hand-written `hessian()`, from the declared capacities.
    let energy = &system.port_hamiltonian.energy;
    for (k, capacity) in [p.c, p.l, p.c_field, p.l_field].iter().enumerate() {
        assert!(
            (energy[k] - 1.0 / capacity).abs() / (1.0 / capacity) < 1e-15,
            "Q[{k}]: declared {} vs hand-written {}",
            energy[k],
            1.0 / capacity
        );
    }

    // And the state layout is the one the hand-written implementation uses.
    let names: Vec<&str> = system
        .coordinates
        .iter()
        .map(|c| c.name.rsplit('.').next().unwrap())
        .collect();
    assert_eq!(
        names,
        vec![
            "charge",
            "circuit_flux",
            "displacement_flux",
            "magnetic_flux"
        ]
    );
}

/// The interconnection is skew, so the interconnection cannot create energy.
///
/// Not asserted of the declaration — checked of what the projection built from it.
#[test]
fn the_assembled_interconnection_is_skew() {
    let system = system!();
    let n = system.dim();
    let j = system.port_hamiltonian.interconnection.dense();
    for row in 0..n {
        for column in 0..n {
            assert_eq!(
                j[row * n + column],
                -j[column * n + row],
                "J[{row}][{column}] is not the negative of its transpose"
            );
        }
    }
}

// ─── 2. against closed-form physics ─────────────────────────────────────────────────────────────

/// The lossless normal modes are the roots of the biquadratic.
#[test]
fn the_normal_modes_match_the_closed_form() {
    let system = variant!(LOSSLESS);
    let (low, high) = Parameters::DECLARED.normal_modes();
    let found = modes(&system).expect("the analysis is trustworthy");

    assert_eq!(found.len(), 2, "two conjugate pairs: {found:#?}");
    for (mode, want) in found.iter().zip([low, high]) {
        let got = mode.frequency_hz * std::f64::consts::TAU;
        assert!(
            (got - want).abs() / want < 1e-9,
            "closed form {want} rad/s, assembled {got} rad/s"
        );
        assert!(
            mode.damping_per_s.abs() < 1e-6 * want,
            "a lossless mode must not decay: {} /s",
            mode.damping_per_s
        );
    }
    println!(
        "lossless modes: {:.3} Hz and {:.3} Hz",
        found[0].frequency_hz, found[1].frequency_hz
    );
}

/// The avoided crossing: the coupling pushes the two modes apart and leaves their product alone.
///
/// `w1 w2 = wc wf` for every coupling, which follows from the biquadratic's constant term not
/// containing `k`. Two oscillators that merely happened to be near each other would not do this;
/// hybridised normal modes do, and it is the cleanest signature there is that the coupling was
/// assembled into `J` rather than into the energy.
#[test]
fn the_coupling_splits_the_modes_and_preserves_their_product() {
    let (wc, wf) = Parameters::DECLARED.uncoupled();
    let mut previous_gap = f64::NEG_INFINITY;

    for k in [0.0, 0.1, 0.2, 0.36, 0.6, 1.0] {
        let mut edits = LOSSLESS.to_vec();
        let from = "k_coupling: kind.parameter { unit u.dimensionless; value 0.36; }";
        let to = format!("k_coupling: kind.parameter {{ unit u.dimensionless; value {k}; }}");
        edits.push((from, &to));
        let system = variant!(&edits);

        let found = modes(&system).expect("the analysis is trustworthy");
        assert_eq!(found.len(), 2);
        let (w1, w2) = (
            found[0].frequency_hz * std::f64::consts::TAU,
            found[1].frequency_hz * std::f64::consts::TAU,
        );

        // The product is coupling-invariant.
        assert!(
            (w1 * w2 - wc * wf).abs() / (wc * wf) < 1e-9,
            "k={k}: the mode product moved from {} to {}",
            wc * wf,
            w1 * w2
        );
        // And the gap grows with the coupling, which is what "avoided" means.
        let gap = w2 - w1;
        assert!(
            gap > previous_gap,
            "k={k}: the gap did not widen ({gap} after {previous_gap})"
        );
        previous_gap = gap;

        println!(
            "k={k:<4} modes {:>9.2} and {:>9.2} Hz   gap {:>9.2} Hz   product {:.6e}",
            w1 / std::f64::consts::TAU,
            w2 / std::f64::consts::TAU,
            gap / std::f64::consts::TAU,
            w1 * w2
        );
    }
    // At zero coupling the two are exactly the uncoupled frequencies.
    let system = variant!(
        &[
            LOSSLESS,
            &[(
                "k_coupling: kind.parameter { unit u.dimensionless; value 0.36; }",
                "k_coupling: kind.parameter { unit u.dimensionless; value 0.0; }",
            )]
        ]
        .concat()
    );
    let found = modes(&system).expect("the analysis is trustworthy");
    let uncoupled = [wc.min(wf), wc.max(wf)];
    for (mode, want) in found.iter().zip(uncoupled) {
        let got = mode.frequency_hz * std::f64::consts::TAU;
        assert!((got - want).abs() / want < 1e-9, "{got} vs {want}");
    }
}

// ─── 3. the analysis against the trajectory ─────────────────────────────────────────────────────

/// A lossless, undriven system holds its Hamiltonian.
#[test]
fn the_lossless_system_conserves_its_energy() {
    let system = variant!(LOSSLESS);
    // Start with charge on the capacitor and nothing else: one store full, three empty.
    let initial = [1.0e-6, 0.0, 0.0, 0.0];
    let trace = run(
        &system,
        &initial,
        4.0e-3,
        2.0e-7,
        50,
        Integrator::DiscreteGradient,
    );

    assert!(
        trace.energy_drift() < 1e-12,
        "the energy wandered by {:e} over {} recorded steps",
        trace.energy_drift(),
        trace.energy.len()
    );
    println!(
        "lossless drift over 20000 steps: {:e}",
        trace.energy_drift()
    );
}

/// The power balance holds at every step, and RK4's does not.
///
/// This is the structure's whole claim: the skew part cancels in the DISCRETE equation for the same
/// reason it cancels in the continuous one, so the residual is rounding and stays rounding. RK4 is
/// the control — accurate, structure-blind, and its residual grows with the run.
#[test]
fn the_balance_is_exact_for_the_structure_preserving_step() {
    let system = system!();
    let initial = [0.0; 4];

    let structured = run(
        &system,
        &initial,
        2.0e-3,
        2.0e-7,
        100,
        Integrator::DiscreteGradient,
    );
    let blind = run(&system, &initial, 2.0e-3, 2.0e-7, 100, Integrator::Rk4);

    println!(
        "worst balance residual — discrete gradient {:e}, RK4 {:e}",
        structured.worst_balance, blind.worst_balance
    );
    assert!(
        structured.worst_balance < 1e-10,
        "the structure-preserving step lost the balance: {:e}",
        structured.worst_balance
    );
    assert!(
        blind.worst_balance > structured.worst_balance * 100.0,
        "RK4 kept the balance as well as the structured step, which would mean the residual is not \
         measuring what it claims: {:e} vs {:e}",
        blind.worst_balance,
        structured.worst_balance
    );
}

/// **The analysis predicts and the simulation confirms.**
///
/// For each observable, the resolvent `(i w I - A)^-1 b` gives the steady-state amplitude in closed
/// form. The transient run, stepped through an entirely different code path, must settle to it. The
/// drive is swept across both normal modes so the comparison is made where the response is largest
/// and where it is small — a match only at resonance would be a coincidence of scale.
#[test]
fn the_analysis_predicts_the_transient() {
    let base = system!();
    let found = modes(&base).expect("the analysis is trustworthy");
    let observable = base
        .observable_of("capacitor_voltage")
        .expect("the capacitor voltage is declared");

    // Around the two damped modes, plus a point well below and one well between.
    let mut probes: Vec<f64> = found.iter().map(|m| m.frequency_hz).collect();
    probes.push(2000.0);
    probes.push(0.5 * (found[0].frequency_hz + found[1].frequency_hz));
    probes.sort_by(f64::total_cmp);

    println!(
        "  {:>10}  {:>14}  {:>14}  {:>10}",
        "drive Hz", "predicted V", "simulated V", "rel.err"
    );
    for probe in probes {
        let system = variant!(&[(
            "source_frequency: kind.parameter { unit pu.hertz; value 6000.0; }",
            &format!("source_frequency: kind.parameter {{ unit pu.hertz; value {probe}; }}"),
        )]);
        let omega = std::f64::consts::TAU * probe;
        let predicted = response(&system, 0, observable, omega)
            .expect("a linear readout has a transfer function")
            .abs();

        // Long enough for the transient to die: the slowest mode decays at a few hundred per
        // second, so ten milliseconds is dozens of time constants.
        let trace = run(
            &system,
            &[0.0; 4],
            1.2e-2,
            1.0e-7,
            20,
            Integrator::DiscreteGradient,
        );
        let simulated = trace.amplitude(observable, 0.15);

        let error = (simulated - predicted).abs() / predicted.max(1e-12);
        println!("  {probe:>10.1}  {predicted:>14.6}  {simulated:>14.6}  {error:>10.2e}");
        assert!(
            error < 2.0e-3,
            "at {probe} Hz the resolvent says {predicted} V and the trajectory says {simulated} V"
        );
    }
}

/// The declared Poynting readout is the power the curl port carries.
///
/// `@maxwell_curl` moves energy between the electric and magnetic stores at `g E H` watts — the
/// weight times the product of the two efforts. The model declares `poynting = E H` and nothing
/// else, so the port's power is `g` times the readout, and that identity is checked against the
/// trajectory rather than assumed.
#[test]
fn the_poynting_readout_is_the_power_crossing_the_curl_port() {
    let system = system!();
    let g = Parameters::DECLARED.g;
    let poynting = system.observable_of("poynting").expect("declared");
    let electric = system.observable_of("electric_effort").expect("declared");
    let magnetic = system.observable_of("magnetic_effort").expect("declared");

    let trace = run(
        &system,
        &[0.0; 4],
        1.0e-3,
        2.0e-7,
        20,
        Integrator::DiscreteGradient,
    );
    let mut worst: f64 = 0.0;
    let mut scale: f64 = 0.0;
    for k in 0..trace.times.len() {
        let (e, h) = (trace.columns[electric][k], trace.columns[magnetic][k]);
        // What the port delivers into the magnetic store, from the assembled structure.
        let delivered = g * e * h;
        worst = worst.max((delivered - g * trace.columns[poynting][k]).abs());
        scale = scale.max(delivered.abs());
    }
    assert!(
        worst <= 1e-12 * scale.max(1e-12),
        "the readout and the port disagree by {worst} W against a scale of {scale} W"
    );
    println!("curl port carries up to {scale:.4} W; readout agrees to {worst:e} W");
}

// ─── the projection refuses what it should ──────────────────────────────────────────────────────

/// A dissipation that would create energy is refused rather than assembled.
#[test]
fn negative_dissipation_is_refused() {
    let source = common::model_path("maxwell_rlc.hymeko");
    if !source.exists() {
        eprintln!("skipped: core/data/dynamics is not in this checkout");
        return;
    }
    let result = std::panic::catch_unwind(|| {
        common::variant(&[(
            "R: kind.parameter { unit u.ohm;    value 12.0; }",
            "R: kind.parameter { unit u.ohm;    value -1.0; }",
        )])
    });
    assert!(
        result.is_err(),
        "a negative resistance must not project into a port-Hamiltonian system"
    );
}

/// A coordinate whose store is not positive is refused: it is not a state.
#[test]
fn a_coordinate_with_no_store_is_refused() {
    let source = common::model_path("maxwell_rlc.hymeko");
    if !source.exists() {
        eprintln!("skipped: core/data/dynamics is not in this checkout");
        return;
    }
    let result = std::panic::catch_unwind(|| {
        common::variant(&[(
            "L: kind.parameter { unit pu.henry; value 0.008; }",
            "L: kind.parameter { unit pu.henry; value 0.0; }",
        )])
    });
    assert!(result.is_err(), "a zero inductance is not an energy store");
}

// ─── the declared study ─────────────────────────────────────────────────────────────────────────

/// Re-evaluating the projection at another parameter value gives what editing the model gives.
///
/// `System::at` exists so a sweep is a question asked of one projection rather than forty
/// projections of forty edited files. That shortcut is only legitimate if it produces the same
/// answer, so here it is checked against the long way round — recompiling an edited declaration —
/// at four couplings including both ends of the declared range.
#[test]
fn the_declared_sweep_matches_recompiling_an_edited_model() {
    let base = variant!(LOSSLESS);
    let sweep = base
        .sweeps
        .iter()
        .find(|s| s.parameter_name.ends_with(".k_coupling"))
        .expect("the model declares a coupling sweep");
    assert_eq!(sweep.points().len(), 41);
    assert_eq!(sweep.points()[0], 0.0);
    assert_eq!(*sweep.points().last().unwrap(), 1.0);

    for k in [0.0, 0.2, 0.5, 1.0] {
        let mut edits = LOSSLESS.to_vec();
        let from = "k_coupling: kind.parameter { unit u.dimensionless; value 0.36; }";
        let to = format!("k_coupling: kind.parameter {{ unit u.dimensionless; value {k}; }}");
        edits.push((from, &to));
        let recompiled = variant!(&edits);

        let quick = modes(&base.at(sweep.parameter, k)).expect("trustworthy");
        let slow = modes(&recompiled).expect("trustworthy");
        assert_eq!(quick.len(), slow.len());
        for (a, b) in quick.iter().zip(&slow) {
            assert!(
                (a.frequency_hz - b.frequency_hz).abs() / b.frequency_hz < 1e-12,
                "k={k}: re-evaluated {} Hz, recompiled {} Hz",
                a.frequency_hz,
                b.frequency_hz
            );
        }
    }
}

/// The sweep is declared, and the projection carries it rather than a command line.
#[test]
fn the_study_is_part_of_the_model() {
    let system = system!();
    assert_eq!(system.sweeps.len(), 1);
    let sweep = &system.sweeps[0];
    assert!(sweep.name.ends_with(".coupling_sweep"));
    assert!(sweep.parameter_name.ends_with(".k_coupling"));
    assert_eq!((sweep.from, sweep.to, sweep.steps), (0.0, 1.0, 41));

    // And the blueprint knows which structural element that parameter supplies, which is what makes
    // re-evaluating it possible at all.
    let supplied: Vec<&str> = system
        .blueprint
        .ports
        .iter()
        .filter(|p| p.weight_from == sweep.parameter)
        .map(|p| p.name.as_str())
        .collect();
    assert_eq!(supplied.len(), 1);
    assert!(supplied[0].ends_with(".field_pickup"));
}

// ─── the declared mode geometry ─────────────────────────────────────────────────────────────────

/// Every coordinate is given a spatial mode, and the geometry implies the capacity that was declared.
///
/// This is the whole licence to draw a field at all. The dynamics is lumped and has no space in it;
/// a shape supplies the basis function the coordinate multiplies. If the geometry implied a different
/// capacity, the drawn field would carry a different energy from the simulated coordinate and the
/// picture would be an illustration standing beside the physics rather than a projection of it.
#[test]
fn the_geometry_implies_the_declared_capacity() {
    let system = system!();
    assert_eq!(
        system.regions.len(),
        system.dim(),
        "every coordinate needs a mode before a field can be drawn"
    );

    println!(
        "  {:<34} {:>16} {:>16} {:>10}",
        "region", "declared", "implied", "rel.err"
    );
    for region in &system.regions {
        let declared = system.coordinates[region.coordinate].capacity;
        let implied = region.shape.implied_capacity();
        let error = region.calibration_error(declared);
        println!(
            "  {:<34} {declared:>16.6e} {implied:>16.6e} {error:>10.2e}",
            region.name.rsplit('.').next().unwrap()
        );
        assert!(
            error < 1e-13,
            "{}: the geometry implies {implied} and the model declares {declared}",
            region.name
        );
    }
}

/// The energy in the drawn field is the energy in the Hamiltonian.
///
/// ```text
/// integral over the region of the energy density  ==  x^2 / (2c)  ==  H_k(x)
/// ```
///
/// The left side is geometry and vacuum constants; the right side is the term the integrator
/// conserves. Checked along a real trajectory rather than at one convenient point, because a
/// calibration that only holds at rest is not a calibration.
#[test]
fn the_field_energy_is_the_stored_energy() {
    let system = system!();
    let trace = run(
        &system,
        &[0.0; 4],
        2.0e-3,
        2.0e-7,
        40,
        Integrator::DiscreteGradient,
    );

    // Re-run recording the state, since the trace records observables.
    let mut x = vec![0.0; system.dim()];
    let mut input = vec![0.0; system.dim()];
    let mut worst: f64 = 0.0;
    let mut peak: f64 = 0.0;
    let step = 2.0e-7;
    for k in 0..10_000 {
        let time = k as f64 * step;
        system.input_at(time + 0.5 * step, &mut input);
        system.port_hamiltonian.step(&mut x, &input, step, 1e-14);

        let mut effort = vec![0.0; system.dim()];
        system.effort(&x, &mut effort);
        let from_geometry: f64 = system
            .regions
            .iter()
            .map(|r| r.shape.energy(effort[r.coordinate]))
            .sum();
        let hamiltonian = system.energy(&x);
        peak = peak.max(hamiltonian);
        if hamiltonian > 1e-12 {
            worst = worst.max((from_geometry - hamiltonian).abs() / hamiltonian);
        }
    }
    println!("peak stored energy {peak:.6e} J; worst field-vs-Hamiltonian disagreement {worst:e}");
    assert!(
        worst < 1e-13,
        "the drawn field and the simulated coordinate hold different energies: {worst:e}"
    );
    assert!(peak > 0.0 && trace.times.len() > 10);
}

/// A region produces field samples a renderer can draw, and they carry the field's own sign.
#[test]
fn the_regions_sample_a_field_a_renderer_can_draw() {
    let system = system!();
    let electric = system
        .regions
        .iter()
        .find(|r| r.name.ends_with(".field_gap"))
        .expect("the field mode's electric region");

    // At a hundred volts across a millimetre: 1e5 V/m, pointing along the declared axis.
    let samples = electric.sample(100.0, 5);
    assert_eq!(samples.len(), 25);
    for sample in &samples {
        assert!((sample.magnitude - 1.0e5).abs() / 1.0e5 < 1e-12);
        assert_eq!(sample.direction, [0.0, 1.0, 0.0]);
        // The offset is dimensionless, so it stays inside the unit face however big the plate is.
        assert!(sample.across.iter().all(|c| c.abs() <= 1.0 + 1e-12));
    }
    // The true extent is reported separately, in metres.
    let (width, depth) = electric.span();
    assert!((width - 0.0372704992233096_f64.sqrt()).abs() < 1e-12);
    assert!((depth - 0.001).abs() < 1e-15);
    for sample in &samples {}
    // Reversing the effort reverses the field rather than the geometry.
    let reversed = electric.sample(-100.0, 5);
    assert_eq!(reversed[0].direction, samples[0].direction);
    assert!((reversed[0].magnitude + samples[0].magnitude).abs() < 1e-9);

    // A solenoid drops the corners its circular cross-section does not have.
    let magnetic = system
        .regions
        .iter()
        .find(|r| r.name.ends_with(".field_coil"))
        .expect("the field mode's magnetic region");
    let round = magnetic.sample(1.0, 5);
    assert!(round.len() < 25, "a round region is not a square one");
    assert!(round.len() >= 13);
    // 400 turns over 113.7 mm at one ampere. Taken from the shape rather than from a literal: the
    // declared length carries fifteen decimals because the calibration needs them, and a rounded
    // copy of it here would be a second, worse statement of the same number.
    let hymeko_dynamics::field::Shape::Solenoid { turns, length, .. } = magnetic.shape else {
        panic!("the field coil is a solenoid");
    };
    assert!((round[0].magnitude - turns / length).abs() / round[0].magnitude < 1e-15);
    assert!(
        (round[0].magnitude - 3518.1).abs() < 1.0,
        "{} A/m",
        round[0].magnitude
    );
}

// ─── compiling without a file system ────────────────────────────────────────────────────────────

/// The same model, compiled from strings, gives the same system.
///
/// This is what lets the model itself be the input on a target with no `std::fs` — a browser. Without
/// it the only thing that could cross into a web page is somebody's pre-baked export, and the page
/// would be a viewer of numbers rather than a consumer of the declaration.
#[test]
fn compiling_from_memory_gives_the_same_system() {
    let from_disk = system!();

    let read = |file: &str| {
        std::fs::read_to_string(common::model_path(file)).expect("the model is readable")
    };
    let (model, dynamics, optimization) = (
        read("maxwell_rlc.hymeko"),
        read("meta_dynamics.hymeko"),
        read("../optimization/meta_optimization.hymeko"),
    );
    let (from_memory, _, _) = hymeko_dynamics::project_sources(
        "dynamics/maxwell_rlc.hymeko",
        &[
            ("dynamics/maxwell_rlc.hymeko", &model),
            ("dynamics/meta_dynamics.hymeko", &dynamics),
            ("optimization/meta_optimization.hymeko", &optimization),
        ],
    )
    .expect("the in-memory sources compile and project");

    assert_eq!(from_memory.coordinates, from_disk.coordinates);
    assert_eq!(from_memory.port_hamiltonian, from_disk.port_hamiltonian);
    assert_eq!(from_memory.drives, from_disk.drives);
    assert_eq!(from_memory.observables, from_disk.observables);
    assert_eq!(from_memory.regions, from_disk.regions);
    assert_eq!(from_memory.sweeps, from_disk.sweeps);
}

/// A source the model imports and the caller forgot to supply is named, not guessed at.
#[test]
fn a_missing_source_is_reported_with_what_was_provided() {
    let read = |file: &str| {
        std::fs::read_to_string(common::model_path(file)).expect("the model is readable")
    };
    let path = common::model_path("maxwell_rlc.hymeko");
    if !path.exists() {
        eprintln!("skipped: core/data/dynamics is not in this checkout");
        return;
    }
    let model = read("maxwell_rlc.hymeko");
    // `Interner` carries no `Debug`, so the success side cannot be unwrapped by `expect_err`.
    let Err(error) = hymeko_dynamics::project_sources(
        "dynamics/maxwell_rlc.hymeko",
        &[("dynamics/maxwell_rlc.hymeko", &model)],
    ) else {
        panic!("a model compiled without the meta-models it imports");
    };
    let text = format!("{error:?}");
    assert!(text.contains("meta_dynamics"), "{text}");
}
