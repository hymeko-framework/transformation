use std::path::{Path, PathBuf};

use hymeko::module_store::module_store::ModuleStore;
use hymeko::module_store::source_provider::MemProvider;
use hymeko::util::real_parser::RealParser;

use super::*;

fn compile_scene(source: &str) -> (hymeko::ir::ir::Ir, hymeko::resolution::interner::Interner) {
    let vocabulary_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("data")
        .join("webxr")
        .join("meta_spatial.hymeko");
    let vocabulary = std::fs::read_to_string(vocabulary_path).unwrap();
    let provider = MemProvider::default()
        .with_file("/meta_spatial.hymeko", vocabulary)
        .with_file("/scene.hymeko", source);
    let mut store = ModuleStore::new(provider, RealParser);
    let program = store.compile(Path::new("/scene.hymeko")).unwrap();
    (program.ir.clone(), store.it)
}

const SCENE: &str = r#"
fixture {
    @"meta_spatial.hymeko";
    using meta_spatial.spatial as s;
}
world: + <isa> s.kinds.spatial_frame {}
camera: + <isa> s.kinds.camera_frame {
    external_frame_id "logitech_optical";
    camera_id 7;
    image_encoding "Rgb8";
    color_space "sRGB";
}
@camera_parent: s.relations.parent_frame {
    (+ camera, - world);
    position [1.0, 2.0, 3.0];
}
marker_frame: + <isa> s.kinds.spatial_frame {}
marker: + <isa> s.kinds.fiducial_anchor {
    fiducial_kind "aruco";
    family "DICT_4X4_50";
    identifier "17";
    size_m 0.05;
}
@track: s.relations.fiducial_registration { (+ marker_frame, ~ marker); }
object: + <isa> s.kinds.spatial_entity {}
@place: s.relations.placement {
    (+ object, - marker_frame);
    position [0.0, 0.1, 0.0];
}
"#;

#[test]
fn spatial_query_exposes_frames_placements_and_fiducial_identity() {
    let (ir, interner) = compile_scene(SCENE);
    let query = SpatialQuery::new(&ir, &interner).unwrap();
    let parents = query.frame_parents().unwrap();
    let frames = query.frames();
    let cameras = query.camera_frames().unwrap();
    let bindings = query.fiducial_bindings().unwrap();
    let placements = query.placements().unwrap();

    assert_eq!(parents.len(), 1);
    assert_eq!(frames.len(), 3);
    assert_eq!(cameras.len(), 1);
    assert_eq!(cameras[0].decl, parents[0].child);
    assert_eq!(cameras[0].camera_id, 7);
    assert_eq!(cameras[0].image_encoding, "Rgb8");
    assert_eq!(cameras[0].color_space, "sRGB");
    assert_eq!(parents[0].pose.position_m, [1.0, 2.0, 3.0]);
    assert_eq!(
        query.external_frame_id(parents[0].child),
        Some("logitech_optical")
    );
    assert_eq!(bindings.len(), 1);
    assert_eq!(bindings[0].kind, FiducialKind::Aruco);
    assert_eq!(bindings[0].family, "DICT_4X4_50");
    assert_eq!(bindings[0].identifier, "17");
    assert_eq!(bindings[0].size_m, 0.05);
    assert_eq!(placements.len(), 1);
    assert_eq!(placements[0].frame, bindings[0].tracked_frame);
}
