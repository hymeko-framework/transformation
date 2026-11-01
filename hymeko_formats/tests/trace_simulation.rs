use hymeko::{
    module_store::{execution::CompiledModel, source_provider::MemProvider},
    util::real_parser::RealParser,
};
use hymeko_formats::traceability::{TraceabilityExport, simulation::SimulationBundle};
use serde_json::json;
use std::path::Path;

const SOURCE: &str = include_str!("../../data/paper/traceability_simulation.hymeko");

#[test]
fn control_period_is_optional_but_checked_and_part_of_model_identity() {
    let legacy = bundle(&SOURCE.replace("control_period_s 0.0005;", "")).unwrap();
    assert!(legacy.artifacts().unwrap().1["controls"].get("control_period_s").is_none());
    let authored = SOURCE.replace("control_period_s 0.0005;", "control_period_s 0.001;");
    let fine = bundle(&authored).unwrap();
    let retained = fine.artifacts().unwrap();
    assert_eq!(retained.1["controls"]["control_period_s"], 0.001);
    let coarse = fine.with_control("timestep_s", 0.001).unwrap();
    assert_eq!(coarse.artifacts().unwrap().1["controls"]["control_period_s"], 0.001);
    assert_ne!(fine.digest(), coarse.digest());
    assert_eq!(fine.model().program().canon.structure(), coarse.model().program().canon.structure());
    assert_ne!(fine.digest(), fine.with_control("control_period_s", 0.002).unwrap().digest());
    for bad in [0., 0.00025, 0.00075, 0.02, f64::NAN, f64::INFINITY] {
        assert!(fine.with_control("control_period_s", bad).is_err(), "{bad}");
    }
    assert!(fine.with_control("timestep_s", 0.002).is_err());
    assert_eq!(fine.artifacts().unwrap(), retained);
}

#[test]
fn checked_control_revision_preserves_retained_snapshot_and_structure() {
    let before = bundle(SOURCE).unwrap();
    let retained = before.artifacts().unwrap();
    let after = before.with_control("actuator_kp", 1200.).unwrap();
    assert_ne!(before.digest(), after.digest());
    assert_eq!(
        before.model().program().canon.structure(),
        after.model().program().canon.structure()
    );
    assert_eq!(before.artifacts().unwrap(), retained);
    assert_eq!(
        after.artifacts().unwrap().1["controls"]["actuator_kp"],
        1200.
    );
    for (field, value) in [
        ("absent", 1.),
        ("actuator_kp", f64::NAN),
        ("actuator_kp", 9000.),
    ] {
        assert!(before.with_control(field, value).is_err());
        assert_eq!(before.artifacts().unwrap(), retained);
    }
}

fn bundle(source: &str) -> Result<SimulationBundle, String> {
    let provider = MemProvider::default()
        .with_file("/paper/case.hymeko", source)
        .with_file(
            "/paper/meta_simulation_case.hymeko",
            include_str!("../../data/paper/meta_simulation_case.hymeko"),
        )
        .with_file(
            "/paper/traceability_verification.hymeko",
            include_str!("../../data/paper/traceability_verification.hymeko"),
        )
        .with_file(
            "/paper/meta_sysml_trace.hymeko",
            include_str!("../../data/paper/meta_sysml_trace.hymeko"),
        )
        .with_file(
            "/robotics/sim/dual_fanuc/fanuc_lrmate200id.hymeko",
            include_str!("../../data/robotics/sim/dual_fanuc/fanuc_lrmate200id.hymeko"),
        )
        .with_file(
            "/robotics/meta_kinematics.hymeko",
            include_str!("../../data/robotics/meta_kinematics.hymeko"),
        );
    let model = CompiledModel::compile(provider, RealParser, Path::new("/paper/case.hymeko"))
        .map_err(|e| format!("{e:?}"))?;
    SimulationBundle::new(model, "experiment")
}

#[test]
fn generated_case_reuses_robot_and_resets_mock_outcomes() {
    let case = bundle(SOURCE).unwrap();
    let (xml, plan) = case.artifacts().unwrap();
    assert!(xml.contains("eulerseq=\"XYZ\""));
    assert!(xml.contains("vacuum_cup"));
    assert_eq!(plan["joint_names"].as_array().unwrap().len(), 6);
    assert_eq!(plan["targets"].as_array().unwrap().len(), 4);
    assert_eq!(plan["rules"].as_array().unwrap().len(), 6);
    assert_eq!(plan["controls"]["actuator_kp"], 800.);
    let trace = TraceabilityExport::new(case.model(), case.trace_scope()).unwrap();
    assert!(
        trace
            .projection()
            .entities
            .iter()
            .filter(|e| e.kind == "test_case")
            .all(|e| e.outcome == Some(-1))
    );
}

#[test]
fn measured_bounds_are_inclusive_and_missing_evidence_stays_unknown() {
    let case = bundle(SOURCE).unwrap();
    let initial = TraceabilityExport::new(case.model(), case.trace_scope())
        .unwrap()
        .reference();
    let evidence = json!({"model_digest":case.digest(),"measurements":{"station_error_m":0.0005,"cycle_time_s":4.00001,"ideal_grip_retained":1,"diagnostics_complete":0,"repeatability_m":null}});
    let (model, verdicts) = case.record(&evidence).unwrap();
    let result = TraceabilityExport::new(&model, case.trace_scope())
        .unwrap()
        .reference();
    for (name, want) in [
        ("T_accuracy", 1),
        ("T_cycle", 0),
        ("T_grip", 1),
        ("T_diagnostics", 0),
        ("T_repeat", -1),
        ("T_stop", -1),
    ] {
        assert_eq!(result["entities"][name]["outcome"], want);
    }
    assert_eq!(verdicts.as_array().unwrap().len(), 6);
    assert_eq!(
        initial,
        TraceabilityExport::new(case.model(), case.trace_scope())
            .unwrap()
            .reference()
    );
    assert_eq!(
        case.model().program().canon.structure(),
        model.program().canon.structure()
    );
    assert_ne!(case.model().semantic_digest(), model.semantic_digest());
}

#[test]
fn wrong_basis_and_invalid_numeric_records_are_rejected() {
    let case = bundle(SOURCE).unwrap();
    assert!(
        case.record(&json!({"model_digest":"stale","measurements":{}}))
            .is_err()
    );
    for value in [json!(-0.1), json!("NaN"), json!(true)] {
        assert!(
            case.record(
                &json!({"model_digest":case.digest(),"measurements":{"cycle_time_s":value}})
            )
            .is_err()
        );
    }
}

#[test]
fn censored_observations_distinguish_proven_failure_from_unknown() {
    let case = bundle(SOURCE).unwrap();
    let (model, _) = case
        .record(&json!({"model_digest":case.digest(),"measurements":{
            "cycle_time_s":{"lower_bound":4.2},"stop_latency_s":{"lower_bound":0.1},
            "station_error_m":{"upper_bound":0.0004},"ideal_grip_retained":{"lower_bound":1},
            "diagnostics_complete":{"upper_bound":0}
        }}))
        .unwrap();
    let result = TraceabilityExport::new(&model, case.trace_scope())
        .unwrap()
        .reference();
    for (test, value) in [
        ("T_cycle", 0),
        ("T_stop", -1),
        ("T_accuracy", 1),
        ("T_grip", 1),
        ("T_diagnostics", 0),
    ] {
        assert_eq!(result["entities"][test]["outcome"], value);
    }
    assert!(case.record(&json!({"model_digest":case.digest(),"measurements":{"stop_latency_s":{"lower_bound":0.1,"upper_bound":0.2}}})).is_err());
}

#[test]
fn unsupported_rules_and_resource_bounds_fail_before_export() {
    for (old, new) in [
        ("station_error_m", "invented_metric"),
        ("comparison \"max\"", "comparison \"unknown\""),
        ("bound 0.0005", "bound -0.0005"),
        ("+ trace.T_accuracy", "+ trace.Arm"),
    ] {
        assert!(SOURCE.contains(old), "negative-case anchor absent: {old}");
        assert!(bundle(&SOURCE.replacen(old, new, 1)).is_err(), "{new}");
    }
    for (old, new) in [
        ("timestep_s 0.0005", "timestep_s 0.000001"),
        ("episode_s 10", "episode_s 1000"),
        ("q [-0.2,0.7,0.8,0,1.641592653589793,0]", "q [0,0]"),
    ] {
        assert!(SOURCE.contains(old), "negative-case anchor absent: {old}");
        assert!(
            bundle(&SOURCE.replacen(old, new, 1))
                .unwrap()
                .artifacts()
                .is_err(),
            "{new}"
        );
    }
}

#[test]
fn simulation_roles_require_the_declared_vocabulary_types() {
    assert!(
        bundle(&SOURCE.replacen(
            "experiment: sim.case_definition",
            "experiment: sim.scene_profile",
            1
        ))
        .is_err()
    );
    for (old, new) in [
        (
            "controls: sim.control_profile",
            "controls: sim.joint_target",
        ),
        ("home: sim.joint_target", "home: sim.scene_profile"),
    ] {
        assert!(SOURCE.contains(old));
        assert!(
            bundle(&SOURCE.replacen(old, new, 1))
                .unwrap()
                .artifacts()
                .is_err()
        );
    }
}
