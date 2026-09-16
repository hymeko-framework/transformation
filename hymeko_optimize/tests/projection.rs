//! What the projection made of the model.
//!
//! These tests assert the SHAPE of the projection — that the decision variables are the ones the
//! model typed as decisions, that the two scenarios differ by exactly one branch, that the units
//! survived — before any solver is involved. A projection fault and a solver fault look alike from
//! the far end, and this is the half that can be checked without a search.

mod common;

use hymeko_optimize::problem::{Kind, Sense, Unit};
use hymeko_optimize::read::project;

#[test]
fn dump() {
    let (ir, it) = divider!();
    let problem = project(&ir, &it).expect("the model projects");
    println!("decision variables:");
    for &q in &problem.decisions {
        let quantity = &problem.quantities[q];
        println!(
            "  {} [{}] in [{:?}, {:?}] from {:?}",
            quantity.name,
            quantity.unit.label(),
            quantity.lower,
            quantity.upper,
            quantity.start
        );
    }
    println!("quantities: {}", problem.quantities.len());
    for quantity in &problem.quantities {
        println!(
            "  {:?} {} [{}] value={:?}",
            quantity.kind,
            quantity.name,
            quantity.unit.label(),
            quantity.value
        );
    }
    println!("nets:");
    for (i, name) in problem.net_names.iter().enumerate() {
        println!(
            "  {i} {name}{}",
            if problem.ground == Some(i) {
                "  (ground)"
            } else {
                ""
            }
        );
    }
    println!("branches:");
    for (i, name) in problem.branch_names.iter().enumerate() {
        println!("  {i} {name} {:?}", problem.branches[i]);
    }
    println!("scenarios:");
    for scenario in &problem.scenarios {
        println!("  {} -> {:?}", scenario.name, scenario.branches);
    }
    println!("relations (in evaluation order):");
    for &r in &problem.order {
        println!("  {:?}", problem.relations[r]);
    }
    println!("constraints:");
    for constraint in &problem.constraints {
        println!(
            "  {} : {} {:?} {}",
            constraint.name,
            problem.quantities[constraint.left].name,
            constraint.sense,
            problem.quantities[constraint.right].name
        );
    }
    println!("objective:");
    for &(q, w) in &problem.objective {
        println!("  {} x {w}", problem.quantities[q].name);
    }
    problem
        .check()
        .expect("the projected problem is well posed");
}

/// The decision variables are the ones the model TYPED as decisions.
///
/// Not the ones named `R`-something, and not a list this crate keeps: rename them in the model and
/// this still passes; retype one as a parameter and it fails, which is the property that makes the
/// `.hymeko` the authority.
#[test]
fn decisions_are_the_typed_ones() {
    let (ir, it) = divider!();
    let problem = project(&ir, &it).expect("the model projects");

    let names: Vec<&str> = problem
        .decisions
        .iter()
        .map(|&q| problem.quantities[q].name.as_str())
        .collect();
    assert_eq!(names, vec!["resistor_divider.R1", "resistor_divider.R2"]);

    for &q in &problem.decisions {
        let quantity = &problem.quantities[q];
        assert_eq!(quantity.kind, Kind::Decision);
        assert_eq!(quantity.unit, Unit::OHM, "{}", quantity.name);
        assert_eq!(quantity.lower, Some(0.000001));
        assert_eq!(quantity.upper, Some(1000000.0));
        assert_eq!(quantity.start, Some(10.0));
    }
}

/// The scenarios differ by exactly one branch, and share everything else.
///
/// This is the claim the model makes in a comment; here it is a measurement. If somebody ever
/// answers a scenario question by copying the topology, the shared count drops and this fails.
#[test]
fn scenarios_share_one_topology() {
    let (ir, it) = divider!();
    let problem = project(&ir, &it).expect("the model projects");

    assert_eq!(problem.scenarios.len(), 2);
    let connected = problem
        .scenarios
        .iter()
        .find(|s| s.name.ends_with(".connected"))
        .expect("a connected scenario");
    let disconnected = problem
        .scenarios
        .iter()
        .find(|s| s.name.ends_with(".disconnected"))
        .expect("a disconnected scenario");

    let only_in_connected: Vec<usize> = connected
        .branches
        .iter()
        .copied()
        .filter(|b| !disconnected.branches.contains(b))
        .collect();
    assert_eq!(
        only_in_connected.len(),
        1,
        "exactly one branch may differ between the scenarios"
    );
    assert!(problem.branch_names[only_in_connected[0]].ends_with(".branch_bulb"));
    assert!(
        disconnected
            .branches
            .iter()
            .all(|b| connected.branches.contains(b)),
        "the disconnected scenario adds nothing of its own"
    );
    // Everything else is one set of branches, not two.
    assert_eq!(connected.branches.len(), disconnected.branches.len() + 1);
    assert_eq!(problem.branches.len(), connected.branches.len());
}

/// The topology is declared, and the projection kept it as a topology.
#[test]
fn network_is_a_topology_not_a_formula() {
    let (ir, it) = divider!();
    let problem = project(&ir, &it).expect("the model projects");

    assert_eq!(problem.nets.len(), 3);
    let ground = problem.ground.expect("a ground reference");
    assert!(problem.net_names[ground].ends_with(".gnd"));
    // Four branches: the source, R1, R2, and the bulb.
    assert_eq!(problem.branches.len(), 4);
}

/// The contract came across with its senses intact.
#[test]
fn constraints_keep_their_sense() {
    let (ir, it) = divider!();
    let problem = project(&ir, &it).expect("the model projects");

    assert_eq!(problem.constraints.len(), 2);
    let equality = problem
        .constraints
        .iter()
        .find(|c| c.sense == Sense::Equality)
        .expect("the operating-point equality");
    assert!(
        problem.quantities[equality.left]
            .name
            .ends_with(".V_A_connected")
    );
    assert!(
        problem.quantities[equality.right]
            .name
            .ends_with(".target_node_a")
    );

    let at_most = problem
        .constraints
        .iter()
        .find(|c| c.sense == Sense::AtMost)
        .expect("the open-circuit limit");
    assert!(
        problem.quantities[at_most.left]
            .name
            .ends_with(".V_A_disconnected")
    );
    assert!(
        problem.quantities[at_most.right]
            .name
            .ends_with(".open_circuit_limit")
    );
}

/// A term carried at weight zero stays visible.
#[test]
fn objective_carries_its_zero_weight_term() {
    let (ir, it) = divider!();
    let problem = project(&ir, &it).expect("the model projects");

    assert_eq!(problem.objective.len(), 2);
    let weights: Vec<f64> = problem.objective.iter().map(|&(_, w)| w).collect();
    assert!(weights.contains(&1.0) && weights.contains(&0.0));
}

/// An approximation carries its evidence, and the check refuses one that does not.
#[test]
fn approximation_carries_provenance() {
    let (ir, it) = divider!();
    let mut problem = project(&ir, &it).expect("the model projects");

    let rb = problem
        .index_of_name("resistor_divider.bulb.Rb")
        .expect("the linearised bulb resistance");
    assert!(problem.quantities[rb].approximation.is_some());
    assert_eq!(
        problem.quantities[rb].provenance.len(),
        2,
        "the two measurements the linearisation rests on"
    );

    // Strip the evidence and the same model must stop passing.
    problem.quantities[rb].provenance.clear();
    assert!(problem.check().is_err());
}
