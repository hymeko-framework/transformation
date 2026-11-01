//! HyMeKo source → compiled IR → kinematic model → link poses.
//!
//! Every other test in this crate builds a `KinematicModel` by hand. This one starts from a real
//! `.hymeko` file compiled by the real `ModuleStore`, so the authoring pattern documented for model
//! authors is the pattern that is actually exercised — including the part most easily got wrong,
//! the `(+ parent [[xyz],[rpy]], - child, - axis)` arc.

use std::collections::HashMap;
use std::path::PathBuf;

use hymeko::module_store::module_store::ModuleStore;
use hymeko::module_store::source_provider::StdFsProvider;
use hymeko::util::real_parser::RealParser;
use hymeko_mechanism::forward::{JointValues, forward_kinematics};
use hymeko_mechanism::kinematics::kinematic::extract_kinematic_model;
use hymeko_query::QueryEngine;

const TAU: f64 = std::f64::consts::TAU;

fn model_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
}

/// Compile the fixture and extract its kinematic model, exactly as an emitter would.
fn arm() -> hymeko_mechanism::kinematics::kinematic::KinematicModel {
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = store
        .compile(&model_path("two_link_arm.hymeko"))
        .expect("fixture must compile");
    let engine = QueryEngine::new(&program.ir, &store.it);
    extract_kinematic_model(&engine, "robot")
}

fn close(a: &[f64], b: [f64; 3]) -> bool {
    a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-9)
}

#[test]
fn the_compiled_source_yields_the_declared_links_and_joints() {
    let m = arm();
    let mut links: Vec<&str> = m.links.iter().map(|l| l.name.as_str()).collect();
    links.sort();
    assert_eq!(links, vec!["base", "tool", "upper"]);

    let mut joints: Vec<&str> = m.joints.iter().map(|j| j.name.as_str()).collect();
    joints.sort();
    assert_eq!(joints, vec!["j1", "j2"]);
}

#[test]
fn the_signed_arc_is_read_as_parent_child_and_axis() {
    // The whole authoring convention in one assertion: `+` is the parent, `-` the child, the
    // axis rides as a second `-` reference, and the origin as weights on the parent.
    let m = arm();
    let j1 = m.joints.iter().find(|j| j.name == "j1").unwrap();
    assert_eq!(j1.parent_link, "base");
    assert_eq!(j1.child_link, "upper");
    assert_eq!(j1.axis, Some([0.0, 0.0, 1.0]));
    assert_eq!(j1.origin_xyz, Some([1.0, 0.0, 0.0]));
}

#[test]
fn poses_come_out_of_a_file_the_cli_compiles() {
    let m = arm();
    let poses = forward_kinematics(&m, &JointValues::new()).expect("arm is a rooted chain");
    assert_eq!(poses.root, "base");
    assert!(close(
        &poses.get("base").unwrap().translation,
        [0.0, 0.0, 0.0]
    ));
    assert!(close(
        &poses.get("upper").unwrap().translation,
        [1.0, 0.0, 0.0]
    ));
    assert!(close(
        &poses.get("tool").unwrap().translation,
        [2.0, 0.0, 0.0]
    ));
}

#[test]
fn driving_the_declared_joint_moves_the_declared_link() {
    // j1 turns about the +z axis the source declares, so the tool swings out of the x axis.
    let m = arm();
    let mut q: JointValues = HashMap::new();
    q.insert("j1".into(), TAU / 4.0);
    let poses = forward_kinematics(&m, &q).unwrap();
    assert!(close(
        &poses.get("upper").unwrap().translation,
        [1.0, 0.0, 0.0]
    ));
    assert!(close(
        &poses.get("tool").unwrap().translation,
        [1.0, 1.0, 0.0]
    ));
}
