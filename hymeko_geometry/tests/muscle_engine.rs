//! Hydraulic braided muscles in Kawakern: the actuation layer, the sleeve, moment arms by virtual
//! work, the series balance, co-contraction stiffness and a pressurised extensor moving a leg.
use hymeko::module_store::module_store::ModuleStore;
use hymeko::module_store::source_provider::StdFsProvider;
use hymeko::util::real_parser::RealParser;
use hymeko_geometry::articulated::AbaPlan;
use hymeko_geometry::engine::Engine;
use hymeko_geometry::engine::muscle::Muscles;
use hymeko_geometry::forward::FkPlan;
use hymeko_geometry::kinematics::actuation::{BraidedSleeve, DriveLaw};
use hymeko_geometry::kinematics::kinematic::{KinematicModel, extract_kinematic_model};
use hymeko_query::QueryEngine;
use std::path::PathBuf;

fn model() -> KinematicModel {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/hydraulic_muscle.hymeko");
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = store.compile(&path).expect("the muscle fixture compiles");
    extract_kinematic_model(&QueryEngine::new(&program.ir, &store.it), "robot")
}

fn slot(model: &KinematicModel, name: &str) -> usize {
    model.joints.iter().position(|j| j.name == name).unwrap()
}

/// Muscle state after holding `q` long enough for valve and damper to settle.
fn settled(model: &KinematicModel, muscles: &mut Muscles, q: &[f64]) -> Vec<f64> {
    let aba = AbaPlan::new(model).unwrap();
    let plan = FkPlan::new(model).unwrap();
    let mut poses = plan.identity_poses_se3();
    plan.resolve_se3(model, q, &mut poses);
    let mut scratch = aba.scratch();
    aba.prepare_transforms(model, q, &mut scratch);
    for _ in 0..3 {
        muscles.step(&aba, &poses, 10.0, &mut scratch);
    }
    muscles.torque().to_vec()
}

fn path_lengths(model: &KinematicModel, muscles: &Muscles, q: &[f64]) -> Vec<f64> {
    let plan = FkPlan::new(model).unwrap();
    let mut poses = plan.identity_poses_se3();
    plan.resolve_se3(model, q, &mut poses);
    let mut probe = muscles.clone();
    probe.settle(&poses);
    probe.states().iter().map(|s| s.path_length).collect()
}

#[test]
fn a_layer_attaches_to_the_unedited_body_with_ordered_paths() {
    let model = model();
    assert!(model.actuation.errors.is_empty(), "{:?}", model.actuation.errors);
    let names: Vec<&str> = model.actuation.attachments.iter().map(|a| a.name.as_str()).collect();
    for expected in ["thigh.patella", "pelvis.rf_origin", "shin.vas_insertion"] {
        assert!(names.contains(&expected), "{names:?}");
    }
    assert_eq!(model.actuation.drives.len(), 3);
    let vas = model.actuation.drives.iter().find(|d| d.name == "vas").unwrap();
    let bfsh = model.actuation.drives.iter().find(|d| d.name == "bfsh").unwrap();
    assert_eq!((vas.via.len(), bfsh.via.len()), (1, 0));
    let DriveLaw::HydraulicMuscle { sleeve, pressure_pa, .. } = vas.law else { panic!("{:?}", vas.law) };
    assert_eq!((sleeve.braid_length_m, sleeve.turns, pressure_pa), (0.30, 1.2, 2.0e6));
}

#[test]
fn the_sleeve_follows_the_braid_shortening_and_widening_under_pressure() {
    let sleeve = BraidedSleeve { braid_length_m: 0.3, turns: 1.2 };
    let b = 0.3f64;
    let full = b * b / (2.0 * std::f64::consts::PI * 1.2 * 1.2);
    assert!((sleeve.force_per_pressure(b) - full).abs() < 1e-15);
    assert!(sleeve.force_per_pressure(sleeve.shortest_pulling_length()).abs() < 1e-12);
    assert_eq!(sleeve.force_per_pressure(0.1), 0.0);
    assert!(sleeve.diameter(0.2) > sleeve.diameter(0.28) && sleeve.diameter(b) == 0.0);
    // Under a held pressure a muscle against a fixed path settles shorter and wider.
    let model = model();
    let mut muscles = Muscles::from_model(&model).unwrap().unwrap();
    let mut q = vec![0.0; model.joints.len()];
    q[slot(&model, "knee")] = 0.6;
    settled(&model, &mut muscles, &q);
    let vas = muscles.index("vas").unwrap();
    let relaxed = muscles.states()[vas];
    muscles.command(vas, 1.5e6);
    settled(&model, &mut muscles, &q);
    let pressed = muscles.states()[vas];
    let law = &muscles.elements()[vas].law;
    assert!(pressed.sleeve_length < relaxed.sleeve_length && pressed.force > relaxed.force, "{relaxed:?} {pressed:?}");
    assert!(law.sleeve.diameter(pressed.sleeve_length) > law.sleeve.diameter(relaxed.sleeve_length));
}

#[test]
fn joint_torques_are_the_virtual_work_of_each_path_for_mono_and_biarticular_muscles() {
    let model = model();
    let mut muscles = Muscles::from_model(&model).unwrap().unwrap();
    for name in ["vas", "bfsh", "rf"] {
        let index = muscles.index(name).unwrap();
        muscles.command(index, 1.0e6);
    }
    let (hip, knee) = (slot(&model, "hip"), slot(&model, "knee"));
    let mut q = vec![0.0; model.joints.len()];
    q[hip] = 0.3;
    q[knee] = 0.7;
    let torque = settled(&model, &mut muscles, &q);
    let forces: Vec<f64> = muscles.states().iter().map(|s| s.force).collect();
    for joint in [hip, knee] {
        let eps = 1e-6;
        let (mut up, mut down) = (q.clone(), q.clone());
        up[joint] += eps;
        down[joint] -= eps;
        let (lu, ld) = (path_lengths(&model, &muscles, &up), path_lengths(&model, &muscles, &down));
        let expected: f64 = forces.iter().zip(lu.iter().zip(&ld)).map(|(f, (a, b))| -f * (a - b) / (2.0 * eps)).sum();
        assert!((torque[joint] - expected).abs() < 1e-5 * (1.0 + expected.abs()), "joint {joint}: {} vs {expected}", torque[joint]);
    }
    // The biarticular muscle alone acts on both joints; the knee muscles not on the hip.
    let lengths = |q: &[f64]| path_lengths(&model, &muscles, q);
    let rf = muscles.index("rf").unwrap();
    let vas = muscles.index("vas").unwrap();
    let mut moved = q.clone();
    moved[hip] += 1e-4;
    assert!((lengths(&moved)[rf] - lengths(&q)[rf]).abs() > 1e-7);
    assert!((lengths(&moved)[vas] - lengths(&q)[vas]).abs() < 1e-12);
}

#[test]
fn the_series_balance_holds_and_the_valve_lags_by_its_time_constant() {
    let model = model();
    let muscles = Muscles::from_model(&model).unwrap().unwrap();
    let law = &muscles.elements()[muscles.index("vas").unwrap()].law;
    for (path, pressure) in [(0.42, 0.0), (0.45, 1.0e6), (0.5, 2.0e6)] {
        let x = law.sleeve_length(path, pressure, 0.25, None);
        let (series, parallel) = (law.tendon_force(path - x), law.parallel_force(pressure, x));
        assert!((series - parallel).abs() <= 1e-6 * (1.0 + series), "{path} {pressure}: {series} vs {parallel}");
    }
    let mut muscles = muscles.clone();
    let vas = muscles.index("vas").unwrap();
    muscles.command(vas, 1.0e6);
    let aba = AbaPlan::new(&model).unwrap();
    let plan = FkPlan::new(&model).unwrap();
    let q = vec![0.0; model.joints.len()];
    let mut poses = plan.identity_poses_se3();
    plan.resolve_se3(&model, &q, &mut poses);
    let mut scratch = aba.scratch();
    aba.prepare_transforms(&model, &q, &mut scratch);
    muscles.step(&aba, &poses, 0.03, &mut scratch);
    let expected = 1.0e6 * (1.0 - (-1.0f64).exp());
    assert!((muscles.states()[vas].pressure - expected).abs() < 1e-6);
    muscles.command(vas, 9.0e6);
    assert_eq!(muscles.states()[vas].command, 2.0e6, "commands clamp to the declared pressure");
}

#[test]
fn cocontraction_stiffens_the_knee() {
    let model = model();
    let knee = slot(&model, "knee");
    let stiffness = |pressure: f64| {
        let mut muscles = Muscles::from_model(&model).unwrap().unwrap();
        for name in ["vas", "bfsh"] {
            let index = muscles.index(name).unwrap();
            muscles.command(index, pressure);
        }
        let delta = 0.02;
        let torque_at = |muscles: &mut Muscles, angle: f64| {
            let mut q = vec![0.0; model.joints.len()];
            q[knee] = angle;
            settled(&model, muscles, &q)[knee]
        };
        let up = torque_at(&mut muscles, 0.6 + delta);
        let down = torque_at(&mut muscles, 0.6 - delta);
        -(up - down) / (2.0 * delta)
    };
    let (low, high) = (stiffness(0.4e6), stiffness(1.6e6));
    assert!(high > low && high > 0.0, "{low} {high}");
}

#[test]
fn a_pressurised_extensor_straightens_the_knee_in_kawakern() {
    let model = model();
    let knee = slot(&model, "knee");
    let run = |pressure: f64| {
        let mut engine = Engine::new(model.clone()).unwrap();
        engine.ground = None;
        engine.gravity = [0.0; 3];
        engine.damping = 0.5;
        engine.contact_step = 1.0 / 480.0;
        engine.q[knee] = 1.0;
        assert_eq!(engine.enable_muscles().unwrap(), 3);
        let vas = engine.muscles().unwrap().index("vas").unwrap();
        assert!(engine.command_muscle(vas, pressure));
        engine.advance(1.0);
        assert!(engine.q.iter().chain(&engine.qd).all(|v| v.is_finite()));
        let shapes = engine.muscle_shapes();
        assert_eq!(shapes.len(), 3);
        assert!(shapes.iter().all(|s| s.diameter.is_finite() && s.sleeve.len() >= 2));
        engine.q[knee]
    };
    let (idle, pressed) = (run(0.0), run(2.0e6));
    assert!(pressed < idle - 0.3, "idle {idle} pressed {pressed}");
}
