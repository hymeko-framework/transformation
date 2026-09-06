//! Real Core compilation through the public projection and format boundary.
use hymeko::module_store::{module_store::ModuleStore, source_provider::StdFsProvider};
use hymeko::util::real_parser::RealParser;
use hymeko_formats::transforms::MjcfTransform;
use hymeko_geometry::{
    kinematics::kinematic::{KinematicModel, extract_kinematic_model},
    transforms::{DomainTransform, ModelView, TransformConfig},
};
use hymeko_query::QueryEngine;
use std::{
    path::PathBuf,
    sync::atomic::{AtomicUsize, Ordering},
};

const SOURCE: &str = include_str!("../../hymeko_geometry/tests/data/linear_drive.hymeko");
static SERIAL: AtomicUsize = AtomicUsize::new(0);

fn compile(source: &str) -> KinematicModel {
    let path: PathBuf = std::env::temp_dir().join(format!(
        "hymeko-drive-{}-{}.hymeko",
        std::process::id(),
        SERIAL.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::write(&path, source).unwrap();
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = store.compile(&path).expect("fixture compiles with Core");
    let model = extract_kinematic_model(&QueryEngine::new(&program.ir, &store.it), "robot");
    std::fs::remove_file(path).unwrap();
    model
}

#[test]
fn resolved_attachments_preserve_identity_and_add_no_body_parent() {
    let model = compile(SOURCE);
    assert!(
        model.actuation.errors.is_empty(),
        "{:?}",
        model.actuation.errors
    );
    assert_eq!(
        (
            model.links.len(),
            model.joints.len(),
            model.actuation.drives.len()
        ),
        (2, 1, 1)
    );
    let drive = &model.actuation.drives[0];
    assert_eq!(drive.joint, model.joints[0].did);
    let mount = model
        .actuation
        .attachments
        .iter()
        .find(|a| a.did == drive.origin)
        .unwrap();
    assert_eq!(mount.name, "upper.mount");
    assert_eq!(
        mount.link,
        model.links.iter().find(|l| l.name == "upper").unwrap().did
    );
    assert_eq!(mount.position, [0.08, 0.0, -0.2]);
    assert_eq!(
        model
            .links
            .iter()
            .find(|l| l.name == "upper")
            .unwrap()
            .inertia_com,
        Some([0.2, 0.3, 0.4, 0.01, 0.02, 0.03])
    );
    let xml = MjcfTransform
        .emit(&ModelView::Kinematic(model), &TransformConfig::default())
        .unwrap();
    assert!(xml.contains("fullinertia=\"0.2 0.3 0.4 0.01 0.02 0.03\""));
    assert!(xml.contains("tendon=\"drive\""));
    assert!(
        !xml.contains("<motor"),
        "geometric drive replaces the ideal hinge motor"
    );
}

#[test]
fn malformed_force_connections_fail_emission_instead_of_disappearing() {
    for (from, to) in [
        ("area_m2 0.00045", "area_m2 -0.00045"),
        ("- lower.rod", "+ lower.rod"),
        ("- lower.rod", "- lower"),
        ("joint -> hinge", "joint -> upper"),
        ("law \"hydraulic\"", "law \"unknown\""),
        ("length_min_m 0.1", "length_min_m 0.5"),
        ("yy 0.3", "yy 3.0"), // positive but physically impossible principal moments
        ("yz 0.03;", ""),     // partial tensors must never become legacy fallback
    ] {
        let model = ModelView::Kinematic(compile(&SOURCE.replace(from, to)));
        assert!(
            MjcfTransform.validate(&model).iter().any(|d| d.is_error()),
            "{to}"
        );
        assert!(
            MjcfTransform
                .emit(&model, &TransformConfig::default())
                .is_none(),
            "{to}"
        );
    }
}

#[test]
fn wrapping_uses_resolved_guide_and_side_and_rejects_invalid_guides() {
    let source = SOURCE.replace("mount: elements.attachment", "guide: elements.attachment { x 0.0; y 0.0; z -0.3; radius_m 0.05; }\n        side: elements.attachment { x -0.06; y 0.0; z -0.3; }\n        mount: elements.attachment")
        .replace("parameters -> cylinder;", "parameters -> cylinder; wrap -> upper.guide; side -> upper.side;");
    let model = compile(&source);
    assert!(
        model.actuation.errors.is_empty(),
        "{:?}",
        model.actuation.errors
    );
    let guide = model.actuation.drives[0].wrapping.unwrap();
    assert_eq!(
        model
            .actuation
            .attachments
            .iter()
            .find(|a| a.did == guide[0])
            .unwrap()
            .radius_m,
        Some(0.05)
    );
    let xml = MjcfTransform
        .emit(&ModelView::Kinematic(model), &TransformConfig::default())
        .unwrap();
    assert!(xml.contains("<geom geom=\"guide.upper.guide\" sidesite=\"upper.side\"/>"));
    for (from, to) in [
        ("x -0.06", "x -0.01"),
        ("radius_m 0.05", "radius_m -0.05"),
        ("wrap -> upper.guide", "wrap -> upper.mount"),
        ("side -> upper.side;", ""),
    ] {
        let bad = ModelView::Kinematic(compile(&source.replace(from, to)));
        assert!(
            MjcfTransform
                .emit(&bad, &TransformConfig::default())
                .is_none(),
            "{to}"
        );
    }
}
