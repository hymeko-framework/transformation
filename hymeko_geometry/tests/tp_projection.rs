use hymeko::module_store::{module_store::ModuleStore, source_provider::StdFsProvider};
use hymeko::util::real_parser::RealParser;
use hymeko_geometry::kinematics::{kinematic::extract_kinematic_model, tp};
use hymeko_query::QueryEngine;
use std::sync::atomic::{AtomicUsize, Ordering};
const SOURCE: &str = include_str!("data/tp_chart.hymeko");
static SERIAL: AtomicUsize = AtomicUsize::new(0);

fn check(source: &str, valid: bool) {
    let path = std::env::temp_dir().join(format!(
        "tp-chart-{}-{}.hymeko",
        std::process::id(),
        SERIAL.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::write(&path, source).unwrap();
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = store.compile(&path).unwrap();
    let query = QueryEngine::new(&program.ir, &store.it);
    let model = extract_kinematic_model(&query, "robot");
    let result = tp::project(&query, &model);
    if valid {
        let charts = result.unwrap();
        assert_eq!(charts.len(), 1);
        let chart = &charts[0];
        assert_eq!(
            chart.axes[0].joint,
            model.joints.iter().find(|j| j.name == "j1").unwrap().did
        );
        assert_eq!(
            chart.axes[1].range_rad,
            [-20_f64.to_radians(), 20_f64.to_radians()]
        );
        let hyperedges = tp::body_joint_hyperedges(&model);
        assert!(hyperedges.iter().any(|(_, joints)| joints.len() == 3));
        assert_eq!(model.joints.len(), 3);
    } else {
        assert!(result.is_err(), "{source}");
    }
    std::fs::remove_file(path).unwrap();
}

#[test]
fn core_incidence_defines_tensor_modes_and_body_hyperedges() {
    check(SOURCE, true);
}

#[test]
fn invalid_chart_parameters_fail_explicitly() {
    for (from, to) in [
        ("joint -> j2", "joint -> j1"),
        ("joint -> j2", "joint -> base"),
        ("+ y", "- y"),
        ("lower_deg -20", "lower_deg 25"),
        ("upper_deg 20", "upper_deg 90"),
        ("samples 11", "samples 11.5"),
        ("rank 3", "rank 8"),
        ("cr_grid 257", "cr_grid 258"),
        ("rest_acceleration", "unsupported"),
    ] {
        check(&SOURCE.replace(from, to), false);
    }
}
