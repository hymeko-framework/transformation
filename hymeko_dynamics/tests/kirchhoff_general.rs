//! Kirchhoff's laws on systems that are not circuits.
//!
//! `tests/probes_and_layout.rs` checks the loop law on the one model that draws a circuit. This
//! checks the form that needs no drawing — `e^T J e = 0`, the interconnection moving power and not
//! making it — on every shipped model, and then the node law on the two that do draw one.
//!
//! The point of the general form is that it applies where the circuit-specific one cannot:
//! `maxwell_rlc` declares four coordinates, three ports and no layout at all, so it has no junctions
//! to balance at and no loops to close, and it still has an interconnection that has to be honest.

mod common;

use hymeko_dynamics::System;

fn load(file: &str) -> Option<System> {
    let path = common::model_path(file);
    if !path.exists() {
        return None;
    }
    let (system, _, _) = hymeko_dynamics::project_file(&path).expect("projects");
    Some(system)
}

/// States the laws have to hold at, whether or not a trajectory ever visits them.
///
/// Kirchhoff is algebraic. Checking it only where the system happens to go would let an
/// interconnection be right about the answer and wrong about the structure.
fn arbitrary(n: usize) -> Vec<Vec<f64>> {
    (1..=7)
        .map(|seed| {
            (0..n)
                .map(|k| {
                    let phase = (seed * 11 + k * 5) as f64;
                    phase.sin() * 10f64.powi(-4 - (k % 4) as i32)
                })
                .collect()
        })
        .collect()
}

const MODELS: [&str; 3] = [
    "maxwell_rlc.hymeko",
    "divider_maxwell.hymeko",
    "lattice_chain.hymeko",
];

#[test]
fn the_interconnection_moves_power_and_does_not_make_it() {
    for file in MODELS {
        let Some(system) = load(file) else { continue };
        let mut worst = 0.0f64;
        for x in arbitrary(system.dim()) {
            for &t in &[0.0, 1.7e-7, 4.0e-2] {
                worst = worst.max(system.kirchhoff(&x, t).tellegen);
            }
        }
        println!("{file:<26} Tellegen residual {worst:e}");
        assert!(
            worst < 1e-12,
            "{file}: e^T J e came to {worst:e} of the largest single-port power"
        );
    }
}

/// The general law applies where the circuit-specific ones do not exist.
#[test]
fn a_system_with_no_drawing_still_has_the_law() {
    let Some(system) = load("maxwell_rlc.hymeko") else { return };
    let laws = system.kirchhoff(&vec![1e-6; system.dim()], 0.0);
    assert!(system.wires.is_empty(), "this model draws no circuit");
    assert_eq!(laws.node, None, "no junctions, so no node law");
    assert_eq!(laws.loops, None, "no branches, so no loop law");
    assert!(laws.tellegen < 1e-12, "and Tellegen still holds: {:e}", laws.tellegen);
}

/// What the branches bring a junction equals what the store there absorbs.
///
/// The left side is the DRAWING and the right side is `J`. They are two declarations and nothing
/// makes them agree — which is what makes this a check rather than an identity.
#[test]
fn what_arrives_at_a_junction_is_what_the_store_takes() {
    for file in ["divider_maxwell.hymeko", "lattice_chain.hymeko"] {
        let Some(system) = load(file) else { continue };
        let mut worst = 0.0f64;
        for x in arbitrary(system.dim()) {
            for &t in &[0.0, 1.7e-7, 4.0e-2] {
                let laws = system.kirchhoff(&x, t);
                worst = worst.max(laws.node.expect("this model draws a circuit"));
            }
        }
        println!("{file:<26} node-law residual {worst:e}");
        assert!(worst < 1e-12, "{file}: the node law is out by {worst:e}");
    }
}

/// The negative control, for the node law this time.
///
/// One branch turned round in the model. `J` is untouched — it comes from the coupling edges — so
/// the trajectory is unchanged and the drawing now says a current arrives where it leaves.
#[test]
fn a_branch_turned_round_breaks_the_node_law() {
    let Some(flipped) = common::variant_of(
        "divider_maxwell.hymeko",
        &[(
            "@wire_r2: route.wire {\n        (+ ground_rail, - node_a, ~ flux_2);",
            "@wire_r2: route.wire {\n        (+ node_a, - ground_rail, ~ flux_2);",
        )],
    ) else {
        return;
    };
    let Some(sound) = load("divider_maxwell.hymeko") else { return };

    let x: Vec<f64> = (0..sound.dim())
        .map(|k| ((k * 7 + 3) as f64).sin() * 1e-5)
        .collect();
    let honest = sound.kirchhoff(&x, 1.0e-7);
    let broken = flipped.kirchhoff(&x, 1.0e-7);

    println!(
        "node law: shipped {:e}, one branch turned round {:e}",
        honest.node.unwrap(),
        broken.node.unwrap()
    );
    assert!(honest.node.unwrap() < 1e-12);
    assert!(
        broken.node.unwrap() > 0.1,
        "turning a branch round should break the node law, not nudge it: {:e}",
        broken.node.unwrap()
    );
    // And Tellegen is untouched, because it is a property of `J` and the drawing is not.
    assert!(
        broken.tellegen < 1e-12,
        "the interconnection is the same interconnection: {:e}",
        broken.tellegen
    );
}

/// The shape of the interconnection, and what it costs when it has a loop.
#[test]
fn the_interconnection_graph_is_a_tree_and_that_is_why_nothing_is_conserved_twice() {
    for file in MODELS {
        let Some(system) = load(file) else { continue };
        let laws = system.kirchhoff(&vec![1e-6; system.dim()], 0.0);
        println!(
            "{file:<26} |V| {} |E| {} cycles {} -> conserved {}",
            laws.graph.vertices, laws.graph.edges, laws.graph.cycles, laws.conserved
        );
        assert_eq!(
            laws.graph.cycles, 0,
            "{file}: every port is a distinct pathway, so the interconnection comes out a tree"
        );
    }
}
