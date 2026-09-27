use std::path::Path;

use hymeko::module_store::module_store::ModuleStore;
use hymeko::module_store::source_provider::MemProvider;
use hymeko::util::real_parser::RealParser;

use super::*;
use crate::{META_ROOM, META_SPATIAL};

fn compile(source: &str) -> (Ir, Interner) {
    let provider = MemProvider::default()
        .with_file("/meta_spatial.hymeko", META_SPATIAL)
        .with_file("/meta_room.hymeko", META_ROOM)
        .with_file("/room.hymeko", source);
    let mut store = ModuleStore::new(provider, RealParser);
    let program = store.compile(Path::new("/room.hymeko")).unwrap_or_else(|e| panic!("{e:?}"));
    (program.ir.clone(), store.it)
}

const HEADER: &str = r#"
fixture {
    @"meta_spatial.hymeko";
    @"meta_room.hymeko";
    using meta_spatial.spatial as s;
    using xr_room.kinds as rk;
}
"#;
const ROOM: &str = r#"
room: + <isa> rk.room {
    frame: + <isa> rk.room_frame {}
    root_marker: + <isa> rk.aruco_marker {
        family "DICT_4X4_50"; identifier "17"; size_m 0.08;
        sample [0,0,0,0,0,0,1,0,0,0,0,0];
    }
    @root_registration: s.relations.fiducial_registration { (+ frame, ~ root_marker); }
    landmarks {
        point19: + <isa> rk.aruco_marker {
            family "DICT_4X4_50"; identifier "19"; size_m 0.08;
            sample [0,0,0,0,0,0,1,0,0,0,0,0];
        }
        unsampled: + <isa> rk.aruco_marker { family "DICT_4X4_50"; identifier "21"; size_m 0.05; }
    }
    headset: + <isa> rk.tracked_device {
        transport "openxr.pose";
        sample [0,0,0,0,0,0,1,0,0,0];
    }
}
robot {
    marker: + <isa> rk.aruco_marker {
        family "DICT_4X4_50"; identifier "18"; size_m 0.08;
        sample [0,0,0,0,0,0,1,0,0,0,0,0];
    }
}
"#;

fn ids(markers: &[ArucoMarker]) -> Vec<u32> {
    let mut ids: Vec<_> = markers.iter().map(|m| m.id).collect();
    ids.sort_unstable();
    ids
}
fn name(interner: &Interner, ir: &Ir, decl: DeclId) -> String {
    interner.resolve(ir.decl_node(decl).unwrap().name).to_owned()
}

#[test]
fn rooms_bind_root_landmarks_and_devices_by_kind_not_by_name() {
    for renamed in [false, true] {
        let mut source = format!("{HEADER}{ROOM}");
        if renamed {
            for (from, to) in [("root_marker", "wall"), ("landmarks", "points"), ("headset", "quest"),
                ("frame: + <isa>", "origin: + <isa>"), ("(+ frame,", "(+ origin,")] {
                source = source.replace(from, to);
            }
        }
        let (ir, interner) = compile(&source);
        let query = RoomQuery::new(&ir, &interner).unwrap();
        let rooms = query.rooms().unwrap();
        assert_eq!(rooms.len(), 1);
        let room = &rooms[0];
        let root = room.root.unwrap();
        assert_eq!((root.family, root.id, root.size_m), ("DICT_4X4_50", 17, 0.08));
        assert_eq!(root.convention, "opencv_marker_x_right_y_up_z_out", "defaulted by the kind");
        assert!(root.sample.is_some());
        assert_eq!(ids(&room.landmarks), [19, 21]);
        assert!(room.landmarks.iter().any(|m| m.id == 21 && m.sample.is_none() && m.size_m == 0.05));
        assert_eq!(room.devices.len(), 1);
        assert_eq!(room.devices[0].transport, Some("openxr.pose"));
        let sample = room.devices[0].sample.unwrap();
        assert_eq!(name(&interner, &ir, sample), "sample");
        assert_eq!(ir.parent_of(sample), Some(room.devices[0].decl));
        assert_eq!(ids(&query.markers().unwrap()), [17, 18, 19, 21], "the robot's marker is outside the room");
    }
}

#[test]
fn a_kind_default_is_inherited_but_an_observation_slot_is_not() {
    let source = format!(
        "{HEADER}{}",
        r#"wall_profile: + <isa> rk.aruco_marker { family "DICT_4X4_50"; size_m 0.1; sample [0,0,0,0,0,0,1,0,0,0,0,0]; }
        room: + <isa> rk.room {
            frame: + <isa> rk.room_frame {}
            a: + <isa> wall_profile { identifier "5"; }
            b: + <isa> wall_profile { identifier "6"; size_m 0.2; sample [0,0,0,0,0,0,1,0,0,0,0,0]; }
        }"#
    );
    let (ir, interner) = compile(&source);
    let query = RoomQuery::new(&ir, &interner).unwrap();
    let markers = query.markers().unwrap();
    assert_eq!(ids(&markers), [5, 6], "a shared profile is not an instance");
    let a = markers.iter().find(|m| m.id == 5).unwrap();
    assert_eq!((a.family, a.size_m), ("DICT_4X4_50", 0.1));
    assert!(a.sample.is_none(), "the profile's slot would be shared by every instance");
    let b = markers.iter().find(|m| m.id == 6).unwrap();
    assert_eq!(b.size_m, 0.2);
    assert_eq!(ir.parent_of(b.sample.unwrap()), Some(b.decl));
    let rooms = query.rooms().unwrap();
    assert!(rooms[0].root.is_none() && rooms[0].devices.is_empty());
    assert_eq!(ids(&rooms[0].landmarks), [5, 6]);
}

#[test]
fn room_cardinality_and_marker_fields_are_checked() {
    let room = |body: &str| {
        let (ir, interner) = compile(&format!("{HEADER}room: + <isa> rk.room {{ {body} }}"));
        let query = RoomQuery::new(&ir, &interner).unwrap();
        query.rooms().map(|_| ())
    };
    let marker = |name: &str, fields: &str| format!("{name}: + <isa> rk.aruco_marker {{ {fields} }}");
    let valid = r#"family "DICT_4X4_50"; identifier "17"; size_m 0.08;"#;
    assert!(matches!(room(""), Err(SpatialQueryError::RoomFrame { found: 0, .. })));
    assert!(matches!(
        room("a: + <isa> rk.room_frame {} b: + <isa> rk.room_frame {}"),
        Err(SpatialQueryError::RoomFrame { found: 2, .. })
    ));
    let two_roots = format!(
        "f: + <isa> rk.room_frame {{}} {} {} \
         @r1: s.relations.fiducial_registration {{ (+ f, ~ m1); }} \
         @r2: s.relations.fiducial_registration {{ (+ f, ~ m2); }}",
        marker("m1", valid),
        marker("m2", &valid.replace("17", "18")),
    );
    assert!(matches!(room(&two_roots), Err(SpatialQueryError::RoomRoot { found: 2, .. })));
    let frame = "f: + <isa> rk.room_frame {}";
    for (label, fields) in [
        ("non-decimal identifier", valid.replace("\"17\"", "\"x17\"")),
        ("qr override", format!("{valid} fiducial_kind \"qr\";")),
        ("zero size", valid.replace("0.08", "0")),
        ("empty family", valid.replace("DICT_4X4_50", "")),
    ] {
        let result = room(&format!("{frame} {}", marker("m", &fields)));
        assert!(matches!(result, Err(SpatialQueryError::InvalidAttribute { .. })), "{label}: {result:?}");
    }
    let missing = room(&format!("{frame} {}", marker("m", r#"family "DICT_4X4_50"; size_m 0.08;"#)));
    assert!(matches!(missing, Err(SpatialQueryError::InvalidAttribute { .. })),
        "the kind's empty identifier default is inherited and rejected: {missing:?}");
}

#[test]
fn a_model_without_the_room_vocabulary_has_no_room_query() {
    let provider = MemProvider::default()
        .with_file("/meta_spatial.hymeko", META_SPATIAL)
        .with_file("/plain.hymeko", "plain { @\"meta_spatial.hymeko\"; } room { marker { size_m 0.08; } }");
    let mut store = ModuleStore::new(provider, RealParser);
    let program = store.compile(Path::new("/plain.hymeko")).unwrap();
    assert!(matches!(RoomQuery::new(&program.ir, &store.it), Err(SpatialQueryError::VocabularyMissing)));
}
