use hymeko::{module_store::module_store::ModuleStore, util::real_parser::RealParser};
use hymeko_mechanism::kinematics::{cameras::CameraSet, kinematic::extract_kinematic_model};
use hymeko_query::QueryEngine;
use hymeko_sources::MemoryProvider;
use std::path::Path;

const SOURCE: &str = r#"
camera_fixture {}
elements { link {} rgb_camera {} @sensor_connection {} @stereo_pair {} }
robot: elements {
    head: elements.link { mass 1; }
    left: elements.rgb_camera {
        camera_id 20; optical_frame "left_optical";
        width 320; height 240; fovy_deg 60; update_rate 5;
        clip [0.02, 20]; position [0.1, 0.03, 0.1]; xy_axes [0, -1, 0, 0, 0, 1];
    }
    right: elements.rgb_camera {
        camera_id 21; optical_frame "right_optical";
        width 320; height 240; fovy_deg 60; update_rate 5;
        clip [0.02, 20]; position [0.1, -0.03, 0.1]; xy_axes [0, -1, 0, 0, 0, 1];
    }
    @mount_l: elements.sensor_connection { (+ left, - head); }
    @mount_r: elements.sensor_connection { (+ right, - head); }
    @eyes: elements.stereo_pair { (+ left, - right); }
}
"#;

fn project(source: &str) -> Result<CameraSet, String> {
    let mut store = ModuleStore::new(
        MemoryProvider::new().with("camera.hymeko", source),
        RealParser,
    );
    let program = store.compile(Path::new("camera.hymeko")).unwrap();
    let query = QueryEngine::new(&program.ir, &store.it);
    let model = extract_kinematic_model(&query, "robot");
    let cameras = CameraSet::project(&query, &model)?;
    assert_eq!(model.links.len(), 1);
    assert_eq!(model.joints.len(), 0, "sensors add no rigid-body edges");
    assert!(cameras.cameras.iter().all(|c| c.link == model.links[0].did));
    Ok(cameras)
}

#[test]
fn rectified_pair_preserves_decl_ids_and_pinhole_calibration() {
    let cameras = project(SOURCE).unwrap();
    assert_eq!(cameras.cameras.len(), 2);
    let pair = &cameras.pairs[0];
    assert_ne!(pair.left, pair.right);
    assert_eq!(pair.baseline_m, 0.06);
    let left = cameras.cameras.iter().find(|c| c.did == pair.left).unwrap();
    assert_eq!(left.camera_id, 20);
    let k = left.intrinsics();
    assert!((k[0] - 207.84609690826528).abs() < 1e-10);
    assert_eq!(
        (k[0], k[2], k[4], k[5], k[8]),
        (k[4], 159.5, k[0], 119.5, 1.0)
    );
}

#[test]
fn invalid_mounts_calibration_and_stereo_incidence_fail() {
    for (from, to) in [
        ("camera_id 21", "camera_id 20"),
        ("camera_id 21", "camera_id 1.5"),
        ("camera_id 21", "camera_id 4294967296"),
        ("right_optical", "left_optical"),
        ("right_optical", " "),
        ("camera_id 21", "distortion [0.1, 0, 0, 0, 0]; camera_id 21"),
        ("width 320", "width 2049"),
        ("fovy_deg 60", "fovy_deg 180"),
        ("update_rate 5", "update_rate 0"),
        ("clip [0.02, 20]", "clip [2, 1]"),
        ("0, -1, 0, 0, 0, 1", "0, -1, 0, 0, -1, 0"),
        ("0.1, -0.03, 0.1", "0.1, 0.03, 0.1"),
        ("0.1, -0.03, 0.1", "0.1, -0.03, 0.2"),
        ("(+ right, - head)", "(- right, + head)"),
        ("(+ left, - right)", "(+ left, + right)"),
        ("(+ left, - right)", "(+ left, - head)"),
    ] {
        assert!(project(&SOURCE.replace(from, to)).is_err(), "{to}");
    }
    assert!(project(&SOURCE.replacen("update_rate 5", "update_rate 6", 1)).is_err());
}
