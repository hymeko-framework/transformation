//! Two things a model can now say about its circuit, and what makes them checkable.
//!
//! **A probe** is a readout that names TERMINALS instead of a coordinate. What it reads is found by
//! walking the declared conductors from one lead to the other and adding up what each drops. That
//! walk uses only the drawing — which terminals each conductor joins — while everything it adds up
//! comes from the interconnection `J` was assembled from. The two are separate declarations and
//! nothing forces them to agree, so:
//!
//! * a probe on the node must return the voltage the node's own state variable already holds, by a
//!   route that never looks at that variable, and
//! * the drops around every loop must sum to zero, which is Kirchhoff's voltage law.
//!
//! Neither is an identity of the code. Flip one conductor's ends in the model and both break, which
//! is the negative control below and the reason the checks are worth running.
//!
//! **A layout** is what the model gets when it declines to place its terminals. The positions come
//! from the same incidence, read a second way: Hall's spectral drawing, the eigenvectors of the
//! terminal graph's Laplacian. A drawing derived that way is reproducible and cannot disagree with
//! the circuit, because it is made of it.

mod common;

use hymeko_dynamics::layout::{arrange, bends};
use hymeko_dynamics::read::ReadError;
use hymeko_dynamics::run::{run, Integrator};
use hymeko_dynamics::{SourceError, System};

/// The divider as a port-Hamiltonian system, or `None` where `core` is not in this checkout.
fn divider() -> Option<System> {
    let path = common::model_path("divider_maxwell.hymeko");
    if !path.exists() {
        return None;
    }
    let (system, _, _) = hymeko_dynamics::project_file(&path).expect("it compiles and projects");
    Some(system)
}

fn column(system: &System, name: &str) -> usize {
    system
        .observables
        .iter()
        .position(|o| o.name.ends_with(name))
        .unwrap_or_else(|| panic!("the model declares a readout called {name}"))
}

/// Long enough to cover the switch-on and several periods of the ringing, which is where a probe
/// and the state it is compared against have the most room to differ. Not long enough to settle —
/// the envelope is still a percent or so wide at the end — and nothing here needs it to be.
const SPAN_S: f64 = 2.0e-6;
const STEP_S: f64 = 1.0e-9;

/// The divider with every hand-placed coordinate taken out of it: what the studio's Arrange button
/// writes, and what a model that never placed anything would have said in the first place.
const STRIPPED: [(&str, &str); 12] = [
    ("supply_rail: layout.terminal { at [-2.6,  1.5, 0.0]; }", "supply_rail: layout.terminal {}"),
    ("node_a:      layout.terminal { at [ 1.4,  1.5, 0.0]; }", "node_a:      layout.terminal {}"),
    ("ground_rail: layout.terminal { at [-2.6, -1.6, 0.0]; }", "ground_rail: layout.terminal {}"),
    ("via [ 1.4, -1.6, 0.0];", ""),
    ("via [ 3.1, -0.2, 0.0];", ""),
    // Every `centre` and `axis`, including the two shapes that share a line with another.
    ("centre [-0.6, -0.1, 0.0];", ""),
    ("centre [-0.6, 1.5, 0.0];", ""),
    ("centre [1.4, 0.3, 0.0];", ""),
    ("centre [2.25, 0.65, 0.0];", ""),
    ("axis   [0.0, 1.0, 0.0];", ""),
    ("axis   [1.0, 0.0, 0.0];", ""),
    ("axis   [0.7071, -0.7071, 0.0];", ""),
];

/// States the identities have to hold at, whether or not a trajectory ever visits them.
///
/// Kirchhoff's voltage law is algebraic: it relates the drawing to `J`, not to a solution. Checking
/// it only where the system happens to go would let a drawing be right about the answer and wrong
/// about the circuit.
fn arbitrary_states(n: usize) -> Vec<Vec<f64>> {
    (1..=6)
        .map(|seed| {
            (0..n)
                .map(|k| {
                    let phase = (seed * 13 + k * 7) as f64;
                    // Spread across the scales the coordinates actually live at: a charge near a
                    // nanocoulomb and a flux near a microweber must both be exercised.
                    phase.sin() * 10f64.powi(-6 - (k % 3) as i32)
                })
                .collect()
        })
        .collect()
}

// ─── the probe against the state it never reads ─────────────────────────────────────────────────

#[test]
fn a_probe_reads_what_the_state_already_holds() {
    let Some(system) = divider() else { return };
    let probe = column(&system, "V_probe");
    let state = column(&system, "V_A");

    let trace = run(
        &system,
        &vec![0.0; system.dim()],
        SPAN_S,
        STEP_S,
        4,
        Integrator::DiscreteGradient,
    );

    // `V_A` is the charge on the node capacitance divided by it — one state variable. `V_probe` is
    // the sum of what the conductors between the node and the reference drop, which contains the
    // RATES of the branch fluxes and the drive, and no charge at all. Different declarations,
    // different arithmetic, one voltage.
    let scale = trace.columns[state]
        .iter()
        .fold(0.0f64, |m, v| m.max(v.abs()));
    let worst = trace.columns[probe]
        .iter()
        .zip(&trace.columns[state])
        .fold(0.0f64, |m, (a, b)| m.max((a - b).abs()));

    assert!(scale > 1.0, "the node reaches a volt or more: {scale}");
    assert!(
        worst < 1e-12 * scale,
        "the probe and the state disagree by {worst} on a {scale} volt node"
    );
}

#[test]
fn a_probe_across_the_supply_reads_the_source() {
    let Some(system) = divider() else { return };
    let supply = column(&system, "V_supply");
    let declared = system.drives[0].amplitude;

    let trace = run(
        &system,
        &vec![0.0; system.dim()],
        SPAN_S,
        STEP_S,
        4,
        Integrator::DiscreteGradient,
    );

    // Nothing in the walk is settling towards this. The supply rail sits at the source voltage from
    // the first step, because the only conductor between it and the reference IS the source.
    let worst = trace.columns[supply]
        .iter()
        .fold(0.0f64, |m, v| m.max((v - declared).abs()));
    assert!(declared > 0.0, "the model drives the circuit");
    assert!(
        worst < 1e-12 * declared,
        "the rail wanders {worst} V from the declared {declared} V source"
    );
}

// ─── Kirchhoff's voltage law, as a comparison of two declarations ───────────────────────────────

#[test]
fn the_drops_around_every_loop_sum_to_zero() {
    let Some(system) = divider() else { return };

    let mut worst_relative = 0.0f64;
    for state in arbitrary_states(system.dim()) {
        for &time in &[0.0, 1.0e-7, 3.3e-7] {
            let drops = system.branch_drops(&state, time);
            let scale = drops
                .iter()
                .fold(f64::MIN_POSITIVE, |m, v| m.max(v.abs()));
            let residual = system
                .kirchhoff_residual(&state, time)
                .expect("the model draws a circuit");
            worst_relative = worst_relative.max(residual / scale);
        }
    }
    assert!(
        worst_relative < 1e-13,
        "the loops close only to {worst_relative:e} of the largest drop"
    );
}

/// The negative control: a check nothing can fail is not a check.
#[test]
fn a_drawing_that_contradicts_the_dynamics_is_caught() {
    // One conductor's ends, swapped. The dynamics is untouched — `J` comes from the coupling edges,
    // not from the routes — so this is a drawing that is wrong about a circuit that is right.
    let Some(flipped) = common::variant_of(
        "divider_maxwell.hymeko",
        &[(
            "@wire_r2: route.wire {\n        (+ ground_rail, - node_a, ~ flux_2);",
            "@wire_r2: route.wire {\n        (+ node_a, - ground_rail, ~ flux_2);",
        )],
    ) else {
        return;
    };
    let Some(sound) = divider() else { return };

    let settled = run(
        &sound,
        &vec![0.0; sound.dim()],
        SPAN_S,
        STEP_S,
        4096,
        Integrator::DiscreteGradient,
    )
    .final_state;

    let honest = sound
        .kirchhoff_residual(&settled, SPAN_S)
        .expect("a circuit");
    let broken = flipped
        .kirchhoff_residual(&settled, SPAN_S)
        .expect("a circuit");

    // Not "larger": the size of the voltage that was double-counted. The residual reports what is
    // wrong, and how wrong, rather than only that something is.
    let node = sound.observe_at(&settled, SPAN_S)[column(&sound, "V_A")];
    assert!(honest < 1e-12, "the shipped drawing closes: {honest:e}");
    assert!(
        (broken - 2.0 * node.abs()).abs() < 1e-6 * node.abs(),
        "the flipped drawing should be out by twice the node voltage ({node} V), and is out by \
         {broken} V"
    );

    // And the physics is untouched, which is what makes the failure a drawing's failure.
    let a = sound.observe_at(&settled, SPAN_S)[column(&sound, "V_A")];
    let b = flipped.observe_at(&settled, SPAN_S)[column(&flipped, "V_A")];
    assert_eq!(a, b, "flipping a route may not move the answer");
}

// ─── what a model may not declare ───────────────────────────────────────────────────────────────

#[test]
fn a_potential_without_a_reference_is_refused() {
    let Some(result) = common::try_variant_of(
        "divider_maxwell.hymeko",
        &[("@ground: layout.reference { (+ ground_rail); }", "")],
    ) else {
        return;
    };
    match result {
        Err(SourceError::Projection(ReadError::Reference { at, .. })) => {
            assert!(at.contains("probe_node"), "it names the probe: {at}")
        }
        other => panic!("a potential above nothing should be refused, got {other:?}"),
    }
}

#[test]
fn a_probe_whose_leads_are_not_joined_is_refused() {
    let Some(result) = common::try_variant_of(
        "divider_maxwell.hymeko",
        &[(
            "    @ground: layout.reference { (+ ground_rail); }",
            "    @ground: layout.reference { (+ ground_rail); }\n\
             \n    island: layout.terminal { at [6.0, 6.0, 0.0]; }\n\
             \n    V_island: kind.derived { unit u.volt; }\n\
             \n    @probe_island: read.across { (+ V_island, - island, - ground_rail); }",
        )],
    ) else {
        return;
    };
    match result {
        Err(SourceError::Projection(ReadError::Disconnected { plus, minus, .. })) => {
            assert!(plus.ends_with("island"), "it names the loose lead: {plus}");
            assert!(minus.ends_with("ground_rail"), "and the other: {minus}");
        }
        other => panic!("a probe across a gap should be refused, got {other:?}"),
    }
}

// ─── the drawing the model did not draw ─────────────────────────────────────────────────────────

/// The shipped divider places all three of its terminals, and gets exactly what it asked for.
#[test]
fn declared_positions_are_kept_exactly() {
    let Some(system) = divider() else { return };
    let expected = [
        ("supply_rail", [-2.6, 1.5, 0.0]),
        ("node_a", [1.4, 1.5, 0.0]),
        ("ground_rail", [-2.6, -1.6, 0.0]),
    ];
    for (name, at) in expected {
        let terminal = system
            .terminals
            .iter()
            .find(|t| t.name.ends_with(name))
            .unwrap_or_else(|| panic!("the model declares {name}"));
        assert!(terminal.pinned, "{name} was placed by the model");
        assert_eq!(terminal.at, at, "{name} is where the model put it");
    }
}

/// Removing every `at` from the model changes the picture and not the circuit.
#[test]
fn a_model_with_no_positions_still_draws_the_same_circuit() {
    let Some(derived) = common::variant_of("divider_maxwell.hymeko", &STRIPPED) else {
        return;
    };
    let Some(declared) = divider() else { return };

    assert!(
        derived.terminals.iter().all(|t| !t.pinned),
        "nothing is pinned once the positions are gone"
    );
    // A derived position is not the origin, and no two terminals share one — a picture in which
    // every node is the same node cannot be clicked on.
    for (i, a) in derived.terminals.iter().enumerate() {
        assert!(
            a.at.iter().any(|v| v.abs() > 1e-6),
            "{} was placed somewhere",
            a.name
        );
        for b in &derived.terminals[i + 1..] {
            let gap = (0..3)
                .map(|k| (a.at[k] - b.at[k]).powi(2))
                .sum::<f64>()
                .sqrt();
            assert!(gap > 0.2, "{} and {} are {gap} apart", a.name, b.name);
        }
    }

    // The layout is presentation, so the answer may not move by a single bit.
    let initial = vec![0.0; declared.dim()];
    let one = run(&declared, &initial, SPAN_S, STEP_S, 4096, Integrator::DiscreteGradient);
    let two = run(&derived, &initial, SPAN_S, STEP_S, 4096, Integrator::DiscreteGradient);
    assert_eq!(
        one.columns[column(&declared, "V_A")],
        two.columns[column(&derived, "V_A")],
        "where the terminals are drawn may not change what the circuit does"
    );

    // And the derived drawing is a drawing of THIS circuit: its loops still close.
    let residual = derived
        .kirchhoff_residual(&two.final_state, SPAN_S)
        .expect("a circuit");
    assert!(residual < 1e-12, "the derived drawing closes: {residual:e}");
}

/// A model that places nothing places nothing: the parasitics follow the circuit too.
///
/// Without this an arranged circuit drags its mode geometry along behind it, still sitting where a
/// drawing that no longer exists put it — which is not a small error, it is a picture of two
/// different circuits at once.
#[test]
fn the_parasitics_follow_the_circuit_they_belong_to() {
    let Some(derived) = common::variant_of("divider_maxwell.hymeko", &STRIPPED) else {
        return;
    };
    let place = |name: &str| {
        derived
            .regions
            .iter()
            .find(|r| r.name.ends_with(name))
            .unwrap_or_else(|| panic!("the model declares {name}"))
    };
    let terminal = |name: &str| {
        derived
            .terminals
            .iter()
            .find(|t| t.name.ends_with(name))
            .expect("a terminal")
            .at
    };
    let near = |a: [f64; 3], b: [f64; 3]| {
        (0..3).map(|k| (a[k] - b[k]).powi(2)).sum::<f64>().sqrt() < 1e-6
    };

    // `charge_a` is the node's capacitance: no conductor carries it, so it hangs at the node the
    // conductors it exchanges energy with have in common, which is not the reference.
    assert!(
        near(place("node_plate").centre, terminal("node_a")),
        "the node capacitance belongs at its node, not at {:?}",
        place("node_plate").centre
    );

    // `flux_1` IS carried, by the conductor between the rail and the node, so its energy is along it.
    let wire = derived
        .wires
        .iter()
        .find(|w| w.name.ends_with("wire_r1"))
        .expect("a conductor");
    let middle = wire.via.unwrap_or_else(|| {
        let (a, b) = (derived.terminals[wire.from].at, derived.terminals[wire.to].at);
        [0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]), 0.5 * (a[2] + b[2])]
    });
    assert!(
        near(place("coil_1").centre, middle),
        "the loop inductance belongs along its run: {:?} vs {middle:?}",
        place("coil_1").centre
    );

    // And it points along the run, which a shunt store's orientation is not implied by — so that one
    // keeps the default rather than being given a direction the circuit never stated.
    let along = {
        let (a, b) = (derived.terminals[wire.from].at, derived.terminals[wire.to].at);
        let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        [d[0] / n, d[1] / n, d[2] / n]
    };
    assert!(near(place("coil_1").axis, along), "the coil lies along its conductor");
    assert_eq!(place("node_plate").axis, [0.0, 1.0, 0.0], "the plate keeps the default");
}

#[test]
fn a_derived_layout_is_reproducible() {
    let edges = [(0, 1), (1, 2), (2, 3), (3, 0), (0, 2)];
    let none = vec![None; 4];
    assert_eq!(
        arrange(4, &edges, &none),
        arrange(4, &edges, &none),
        "the same graph draws the same way twice"
    );

    let (Some(a), Some(b)) = (
        common::variant_of("divider_maxwell.hymeko", &STRIPPED),
        common::variant_of("divider_maxwell.hymeko", &STRIPPED),
    ) else {
        return;
    };
    let places = |s: &System| s.terminals.iter().map(|t| t.at).collect::<Vec<_>>();
    assert_eq!(places(&a), places(&b), "and so does the same model");
}

/// A cycle has no distinguished node, and its drawing must not invent one.
#[test]
fn a_cycle_is_drawn_as_a_cycle() {
    let placed = arrange(3, &[(0, 1), (1, 2), (2, 0)], &vec![None; 3]);
    let gap = |a: [f64; 3], b: [f64; 3]| {
        (0..3).map(|k| (a[k] - b[k]).powi(2)).sum::<f64>().sqrt()
    };
    let sides = [
        gap(placed[0], placed[1]),
        gap(placed[1], placed[2]),
        gap(placed[2], placed[0]),
    ];
    let longest = sides.iter().fold(0.0f64, |m, s| m.max(*s));
    let shortest = sides.iter().fold(f64::MAX, |m, s| m.min(*s));
    assert!(
        longest - shortest < 1e-9 * longest,
        "a triangle of equals should draw as one: {sides:?}"
    );
}

/// Two anchors fix the frame; the rest of the drawing arrives inside it.
#[test]
fn pinned_terminals_carry_the_derived_ones_with_them() {
    let pins = vec![
        Some([-3.0, 0.0, 0.0]),
        None,
        Some([3.0, 0.0, 0.0]),
        None,
    ];
    let placed = arrange(4, &[(0, 1), (1, 2), (2, 3), (3, 0)], &pins);
    assert_eq!(placed[0], [-3.0, 0.0, 0.0]);
    assert_eq!(placed[2], [3.0, 0.0, 0.0]);
    for derived in [placed[1], placed[3]] {
        assert!(
            derived[0].abs() <= 3.5 && derived[1].abs() <= 3.5,
            "a derived terminal lands in the frame the pins set, not {derived:?}"
        );
    }
    // The square is symmetric about the axis through its pins, so the two free corners belong on
    // opposite sides of it. A layout that stacked them would be drawing one node twice.
    assert!(
        placed[1][1] * placed[3][1] < 0.0,
        "the free corners fall either side: {:?} and {:?}",
        placed[1],
        placed[3]
    );
}

#[test]
fn parallel_conductors_do_not_lie_on_top_of_each_other() {
    let places = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]];
    let edges = [(0, 1), (0, 1), (0, 1)];
    let derived = bends(&places, &edges, &vec![None; 3]);

    let bent: Vec<[f64; 3]> = derived.iter().filter_map(|v| *v).collect();
    assert_eq!(bent.len(), 2, "the middle lane stays straight: {derived:?}");
    assert!(
        (bent[0][1] + bent[1][1]).abs() < 1e-12 && bent[0][1] != bent[1][1],
        "the other two bow either side of it: {bent:?}"
    );

    // A conductor the model already bent keeps the bend it was given.
    let declared = vec![Some([1.0, 9.0, 0.0]), None, None];
    let kept = bends(&places, &edges, &declared);
    assert_eq!(kept[0], Some([1.0, 9.0, 0.0]));
}
