//! A chain below an intermediate link (a floating-base robot's limb): `ModelChain::below`.
use hymeko::module_store::module_store::ModuleStore;
use hymeko::module_store::source_provider::StdFsProvider;
use hymeko::util::real_parser::RealParser;
use hymeko_mechanism::forward::{JointValues, forward_kinematics};
use hymeko_mechanism::kinematics::ik::{IkError, IkOptions, ModelChain, solve_serial};
use hymeko_mechanism::kinematics::kinematic::{KinematicModel, extract_kinematic_model};
use hymeko_query::QueryEngine;
use std::path::PathBuf;

fn fanuc() -> KinematicModel {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../data/robotics/sim/dual_fanuc/fanuc_lrmate200id.hymeko");
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = store.compile(&path).expect("the FANUC fixture compiles");
    extract_kinematic_model(&QueryEngine::new(&program.ir, &store.it), "robot")
}

#[test]
fn a_limb_below_an_intermediate_link_names_and_solves_only_its_own_joints() {
    let model = fanuc();
    let base = model.joints.iter().find(|j| j.name == "j1").unwrap().child_link.clone();
    let names = ModelChain::path_names(&model, Some(&base), "tool_flange");
    assert_eq!(names, ["j2", "j3", "j4", "j5", "j6"]);
    assert_eq!(ModelChain::path_names(&model, None, "tool_flange"), ["j1", "j2", "j3", "j4", "j5", "j6"]);
    let chain = ModelChain::below(&model, &base, &names, "tool_flange").unwrap();
    // The joint above the base is outside the chain and stays at zero.
    let truth = [0.8, 0.7, 0.1, 1.3, -0.2];
    let values: JointValues = names.iter().zip(truth).map(|(n, v)| (n.to_string(), v)).collect();
    let pose = forward_kinematics(&model, &values).unwrap().get("tool_flange").unwrap().clone();
    let position = std::array::from_fn(|i| pose.translation[i]);
    let rotation = std::array::from_fn(|i| pose.rotation_matrix()[i]);
    let limits: Vec<[f64; 2]> = names
        .iter()
        .map(|n| {
            let l = model.joints.iter().find(|j| j.name == *n).unwrap().limits.as_ref().unwrap();
            [l.lower, l.upper]
        })
        .collect();
    let start: Vec<f64> = truth.iter().map(|q| q + 0.1).collect();
    let solution = solve_serial(&chain, &limits, &start, &position, Some(&rotation), &IkOptions::default()).unwrap();
    assert!(solution.converged, "{solution:?}");
    // Naming the joint above the base, or a base that is not a link, is refused.
    assert!(matches!(ModelChain::below(&model, &base, &["j1", "j2", "j3", "j4", "j5", "j6"], "tool_flange"), Err(IkError::Path { .. })));
    assert!(matches!(ModelChain::below(&model, "no_such_link", &names, "tool_flange"), Err(IkError::Model)));
}
