//! Kirchhoff as the quasi-static limit of Maxwell, checked on one circuit.
//!
//! `optimization/resistor_divider.hymeko` declares a resistor network and `hymeko_optimize` solves it
//! with Kirchhoff's current law and Ohm's law: one linear system, no state, `V_A = 1.5 V`.
//! `dynamics/divider_maxwell.hymeko` declares the SAME circuit as a port-Hamiltonian system that
//! declines to neglect the node's capacitance or the branches' inductance: four state coordinates, a
//! trajectory, ringing at 35.6 MHz.
//!
//! Setting every derivative to zero turns the second into the first, term for term, with the
//! parasitics nowhere in the result. So:
//!
//! * the settled trajectory must equal the nodal solution **exactly**, and
//! * it must do so for **any** parasitic values, which is what "quasi-static limit" means.
//!
//! Two projections, two vocabularies, two pieces of numerical machinery — a nodal solve over dual
//! numbers and a structure-preserving integrator — and one circuit. Neither can compensate for a
//! mistake in the other, which is what makes the agreement worth measuring.

mod common;

use hymeko_dynamics::analysis::modes;
use hymeko_dynamics::run::{run, Integrator};
use hymeko_dynamics::System;

/// The dynamical reading of the divider.
fn maxwell() -> Option<System> {
    let path = common::model_path("divider_maxwell.hymeko");
    if !path.exists() {
        return None;
    }
    let (system, _, _) = hymeko_dynamics::project_file(&path).expect("it compiles and projects");
    Some(system)
}

/// The network reading, evaluated at the design the optimisation slice solved for.
fn kirchhoff() -> Option<Vec<(String, f64)>> {
    let path = common::model_path("../optimization/resistor_divider.hymeko");
    if !path.exists() {
        return None;
    }
    let mut store = hymeko::module_store::module_store::ModuleStore::new(
        hymeko::module_store::source_provider::StdFsProvider::new(),
        hymeko::util::real_parser::RealParser,
    );
    let program = store.compile(&path).expect("the network model compiles");
    let problem =
        hymeko_optimize::read::project(&program.ir.clone(), &store.it).expect("it projects");
    problem.check().expect("it is well posed");

    // The solved design: R1 = 3 ohm, R2 = 1.5 ohm.
    let evaluation = problem.evaluate(&[3.0, 1.5]).expect("it evaluates");
    Some(
        problem
            .quantities
            .iter()
            .enumerate()
            .map(|(i, q)| (q.name.clone(), evaluation.values[i].value))
            .collect(),
    )
}

fn look(table: &[(String, f64)], suffix: &str) -> f64 {
    table
        .iter()
        .find(|(name, _)| name.ends_with(suffix))
        .unwrap_or_else(|| panic!("{suffix} is not in the network model"))
        .1
}

/// Run a system to its steady state and read one observable.
fn settled(system: &System, name: &str, duration: f64, step: f64) -> f64 {
    let trace = run(
        system,
        &vec![0.0; system.dim()],
        duration,
        step,
        ((duration / step) as usize / 400).max(1),
        Integrator::DiscreteGradient,
    );
    let column = system.observable_of(name).expect("a declared readout");
    *trace.columns[column].last().expect("a recorded sample")
}

macro_rules! pair {
    () => {
        match (maxwell(), kirchhoff()) {
            (Some(a), Some(b)) => (a, b),
            _ => {
                eprintln!("skipped: core/data is not in this checkout");
                return;
            }
        }
    };
}

/// **The settled trajectory is the nodal answer.**
///
/// Twenty microseconds is some forty time constants of the slowest mode, so what remains of the
/// transient is below `1e-18`. The agreement that remains is between two independent solvers.
#[test]
fn the_steady_state_is_the_kirchhoff_answer() {
    let (system, network) = pair!();

    let step = 1.0e-9;
    let duration = 2.0e-5;
    println!("  {:>18}  {:>16}  {:>16}  {:>10}", "quantity", "Kirchhoff", "Maxwell settled", "rel.err");
    for (dynamic, nodal) in [
        ("V_A", "V_A_connected"),
        ("P_R2", "P_R2_connected"),
        ("P_bulb", "P_bulb_connected"),
    ] {
        let want = look(&network, nodal);
        let got = settled(&system, dynamic, duration, step);
        let error = (got - want).abs() / want.abs().max(f64::MIN_POSITIVE);
        println!("  {dynamic:>18}  {want:>16.9}  {got:>16.9}  {error:>10.2e}");
        assert!(
            error < 1e-9,
            "{dynamic}: the network says {want} and the settled trajectory says {got}"
        );
    }

    // And the currents obey Kirchhoff's law at the node, which is what the charge coordinate is.
    let (i1, i2, ib) = (
        settled(&system, "i_1", duration, step),
        settled(&system, "i_2", duration, step),
        settled(&system, "i_bulb", duration, step),
    );
    println!("  node current balance: {i1:.9} - {i2:.9} - {ib:.9} = {:.3e}", i1 - i2 - ib);
    assert!((i1 - i2 - ib).abs() < 1e-9 * i1.abs());
    assert!((i1 - 1.5).abs() < 1e-9 && (i2 - 1.0).abs() < 1e-9 && (ib - 0.5).abs() < 1e-9);
}

/// The parasitics move the ringing and leave the answer alone.
///
/// The declared sweep runs the node capacitance over two decades. The resonance goes as
/// `1/sqrt(LC)`, so it should move by a factor of ten across that; the settled voltage may not move
/// at all. A hundredfold change in what Kirchhoff neglects is not allowed to touch what Kirchhoff
/// says — that invariance is the quasi-static limit, and it is the reason the network model is
/// entitled to have no state in it.
#[test]
fn what_kirchhoff_neglects_does_not_change_what_kirchhoff_says() {
    let (system, network) = pair!();
    let sweep = system
        .sweeps
        .iter()
        .find(|s| s.parameter_name.ends_with(".C_node"))
        .expect("the model declares a parasitic sweep");
    let want = look(&network, "V_A_connected");

    println!("  {:>12}  {:>14}  {:>16}  {:>10}", "C_node F", "ringing Hz", "settled V_A", "rel.err");
    let mut lowest = f64::INFINITY;
    let mut highest: f64 = 0.0;
    for point in [sweep.from, 1.0e-10, sweep.to] {
        let at = system.at(sweep.parameter, point);
        let ringing = modes(&at)
            .expect("the analysis is trustworthy")
            .first()
            .map(|m| m.frequency_hz)
            .expect("an oscillatory mode");
        lowest = lowest.min(ringing);
        highest = highest.max(ringing);

        // The settling time goes as the capacitance too, so the run is sized from the mode itself.
        let settle = 40.0 / modes(&at).unwrap()[0].damping_per_s;
        let got = settled(&at, "V_A", settle, 1.0 / (ringing * 30.0));
        let error = (got - want).abs() / want;
        println!("  {point:>12.3e}  {ringing:>14.1}  {got:>16.9}  {error:>10.2e}");
        assert!(
            error < 1e-9,
            "at C_node = {point} the settled voltage moved to {got} from {want}"
        );
    }
    assert!(
        highest / lowest > 8.0,
        "the ringing barely moved ({lowest:.0} to {highest:.0} Hz); the sweep is not exercising anything"
    );
}

/// What Maxwell adds that Kirchhoff cannot see.
///
/// The network model has no state and therefore no transient: switch it on and the answer is simply
/// there. The dynamical one rings at 35.6 MHz with a quality factor above fifty, and overshoots the
/// settled voltage on the way — which is a real thing a real board does, and a thing a nodal solve is
/// structurally incapable of reporting.
#[test]
fn the_transient_overshoots_what_the_network_predicts() {
    let (system, network) = pair!();
    let want = look(&network, "V_A_connected");

    let found = modes(&system).expect("the analysis is trustworthy");
    assert_eq!(found.len(), 1, "one oscillatory mode: {found:#?}");
    let ringing = found[0];
    println!(
        "ringing {:.3} MHz, Q {:.1}, decaying at {:.3e} /s",
        ringing.frequency_hz / 1e6,
        ringing.quality,
        ringing.damping_per_s
    );
    assert!(ringing.quality > 10.0, "too damped to ring: Q {}", ringing.quality);

    let trace = run(
        &system,
        &vec![0.0; system.dim()],
        4.0e-6,
        5.0e-10,
        4,
        Integrator::DiscreteGradient,
    );
    let column = system.observable_of("V_A").expect("declared");
    // The tracked extreme, not the largest recorded sample: the two differ by several per cent at
    // this stride, and only one of them is the trajectory's.
    let (_, peak) = trace.extent(column);
    let overshoot = peak / want - 1.0;
    println!("peak {peak:.6} V against a settled {want:.6} V — {:.1}% overshoot", 100.0 * overshoot);
    assert!(
        overshoot > 0.2,
        "a Q of {:.0} should overshoot substantially; it reached {peak}",
        ringing.quality
    );

    // The balance still holds while all of that is happening.
    println!("worst power-balance residual through the transient: {:e}", trace.worst_balance);
    assert!(trace.worst_balance < 1e-10);
}

/// The circuit is declared in space, and the components are findable from it.
///
/// A region says where a coordinate's ENERGY sits, and on this circuit every one of those is a
/// parasitic — the node's capacitance, the loops' inductance. The three components the design is
/// about are resistors, which store nothing and appear in no region at all. A view built from
/// regions alone therefore draws three coils and a capacitor, and a reader concludes the circuit has
/// three inductors in it. It has none.
///
/// So the model declares the circuit as well: terminals, and the conductor between each pair. Each
/// wire names the coordinate whose effort is its current, which is also how its resistor is found —
/// structurally, through the dissipation attached to that same coordinate, and not by matching names.
#[test]
fn the_circuit_is_declared_in_space() {
    let (system, _) = pair!();

    assert_eq!(system.terminals.len(), 3, "supply rail, node A, ground");
    assert_eq!(system.wires.len(), 4, "the source and the three branches");
    assert_eq!(system.emitters.len(), 1, "one lamp");

    // Every wire carries a coordinate that a resistance is attached to: that is what makes a
    // component findable without a name.
    for wire in &system.wires {
        let resistance = system.port_hamiltonian.dissipation[wire.coordinate];
        assert!(
            resistance > 0.0,
            "{} carries {} and nothing dissipates on it",
            wire.name,
            system.coordinates[wire.coordinate].name
        );
        assert_ne!(wire.from, wire.to, "{} goes nowhere", wire.name);
    }

    // The two branches that share a pair of terminals are bent apart, or they would be one line.
    let parallel: Vec<&hymeko_dynamics::system::Wire> = system
        .wires
        .iter()
        .filter(|w| w.name.ends_with(".wire_r2") || w.name.ends_with(".wire_bulb"))
        .collect();
    assert_eq!(parallel.len(), 2);
    assert_eq!(parallel[0].from, parallel[1].from);
    assert_eq!(parallel[0].to, parallel[1].to);
    assert!(
        parallel.iter().all(|w| w.via.is_some()),
        "two branches between one pair of terminals need bending apart"
    );

    // The lamp is the bulb branch, and it is the branch the network model called the bulb.
    let lamp = system.emitters[0];
    assert!(system.coordinates[lamp].name.ends_with(".flux_bulb"));
    assert!(
        (system.port_hamiltonian.dissipation[lamp] - 3.0).abs() < 1e-12,
        "the lamp's resistance is the linearised bulb, 3 ohm"
    );
    println!(
        "{} terminals, {} wires, lamp on {} at {} ohm",
        system.terminals.len(),
        system.wires.len(),
        system.coordinates[lamp].name.rsplit('.').next().unwrap(),
        system.port_hamiltonian.dissipation[lamp]
    );
}
