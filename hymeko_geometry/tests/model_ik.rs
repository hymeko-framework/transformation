//! Damped-least-squares IK over an origin/axis model path: the same solver the DH chain uses.
use hymeko::module_store::module_store::ModuleStore;
use hymeko::module_store::source_provider::StdFsProvider;
use hymeko::util::real_parser::RealParser;
use hymeko_geometry::forward::{JointValues, forward_kinematics};
use hymeko_geometry::kinematics::ik::{IkError, IkOptions, ModelChain, solve_serial};
use hymeko_geometry::kinematics::kinematic::{KinematicModel, extract_kinematic_model};
use hymeko_query::QueryEngine;
use std::path::PathBuf;

const JOINTS: [&str; 6] = ["j1", "j2", "j3", "j4", "j5", "j6"];

fn fanuc() -> KinematicModel {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../data/robotics/sim/dual_fanuc/fanuc_lrmate200id.hymeko");
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = store.compile(&path).expect("the FANUC fixture compiles");
    extract_kinematic_model(&QueryEngine::new(&program.ir, &store.it), "robot")
}

fn flange(model: &KinematicModel, q: &[f64]) -> ([f64; 3], [f64; 9]) {
    let values: JointValues = JOINTS.iter().zip(q).map(|(j, v)| (j.to_string(), *v)).collect();
    let pose = forward_kinematics(model, &values).unwrap().get("tool_flange").unwrap().clone();
    (std::array::from_fn(|i| pose.translation[i]), std::array::from_fn(|i| pose.rotation_matrix()[i]))
}

fn limits(model: &KinematicModel) -> Vec<[f64; 2]> {
    JOINTS
        .iter()
        .map(|name| {
            let l = model.joints.iter().find(|j| j.name == *name).unwrap().limits.as_ref().unwrap();
            [l.lower, l.upper]
        })
        .collect()
}

#[test]
fn a_posed_flange_is_found_again_from_a_nearby_start() {
    let model = fanuc();
    let chain = ModelChain::new(&model, &JOINTS, "tool_flange").unwrap();
    let bounds = limits(&model);
    for truth in [[-0.2, 0.9, 0.8, 0., 1.441592653589793, 0.], [0.2, 0.7, 0.8, 0.1, 1.6, -0.2], [0.5, 0.4, 1.1, -0.3, 1.2, 0.4]] {
        let (position, rotation) = flange(&model, &truth);
        let start: Vec<f64> = truth.iter().map(|q| q + 0.15).collect();
        let solution = solve_serial(&chain, &bounds, &start, &position, Some(&rotation), &IkOptions::default()).unwrap();
        assert!(solution.converged, "{solution:?}");
        let (reached, turned) = flange(&model, &solution.coordinates);
        let gap = (0..3).map(|i| (reached[i] - position[i]).powi(2)).sum::<f64>().sqrt();
        assert!(gap < 2e-5, "position {gap}");
        let dot = (0..9).map(|i| turned[i] * rotation[i]).sum::<f64>();
        assert!(dot > 3. - 1e-6, "orientation {dot}");
    }
}

#[test]
fn an_unreachable_point_is_reported_not_solved() {
    let model = fanuc();
    let chain = ModelChain::new(&model, &JOINTS, "tool_flange").unwrap();
    let solution = solve_serial(&chain, &limits(&model), &[0.; 6], &[2.5, 0., 0.4], None, &IkOptions::default()).unwrap();
    assert!(!solution.converged && solution.position_error_m > 1.5, "{solution:?}");
    assert!(solution.coordinates.iter().all(|q| q.is_finite()));
}

#[test]
fn a_chain_must_name_exactly_the_movable_path_in_order() {
    let model = fanuc();
    assert!(matches!(ModelChain::new(&model, &["j2", "j1", "j3", "j4", "j5", "j6"], "tool_flange"), Err(IkError::Path { index: 0 })));
    assert!(matches!(ModelChain::new(&model, &JOINTS[..5], "tool_flange"), Err(IkError::Path { index: 5 })));
    assert!(matches!(ModelChain::new(&model, &JOINTS[..5], "link_wrist"), Ok(_)));
    assert!(matches!(ModelChain::new(&model, &JOINTS, "no_such_link"), Err(IkError::Path { .. } | IkError::Model)));
}
