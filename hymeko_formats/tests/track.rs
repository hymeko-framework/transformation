use hymeko::{
    common::ids::DeclId, module_store::module_store::ModuleStore, util::real_parser::RealParser,
};
use hymeko_formats::environment::track::{Track, read_track};
use hymeko_sources::MemoryProvider;
use std::path::Path;

const CIRCLE: &str = r#"
track_fixture {}
road {
    degree 2; samples 2048; width 16;
    knots [0,0,0,1,1,2,2,3,3,4,4,4];
    controls {
        p0 { point [100,0]; weight 1; }
        p1 { point [100,100]; weight 0.7071067811865476; }
        p2 { point [0,100]; weight 1; }
        p3 { point [-100,100]; weight 0.7071067811865476; }
        p4 { point [-100,0]; weight 1; }
        p5 { point [-100,-100]; weight 0.7071067811865476; }
        p6 { point [0,-100]; weight 1; }
        p7 { point [100,-100]; weight 0.7071067811865476; }
        p8 { point [100,0]; weight 1; }
    }
    obstacles { cone { fraction 0.25; offset 3; radius 0.5; height 1; } }
}
"#;
fn project(source: &str) -> Result<Track, String> {
    let mut store = ModuleStore::new(
        MemoryProvider::new().with("track.hymeko", source),
        RealParser,
    );
    let program = store.compile(Path::new("track.hymeko")).unwrap();
    let id = program
        .ir
        .decl_nodes
        .iter()
        .position(|n| store.it.resolve(n.name) == "road")
        .unwrap();
    read_track(&program.ir, &store.it, DeclId::new(id))
}
#[test]
fn rational_circle_closes_with_exact_frames_and_canonical_controls() {
    let road = project(CIRCLE).unwrap();
    assert_eq!(road.controls.len(), 9);
    assert_eq!(road.obstacles.len(), 1);
    assert!(road.controls.iter().all(|c| c.id != road.id));
    assert!((road.length() - std::f64::consts::TAU * 100.0).abs() < 0.001);
    for i in -100..=100 {
        let s = road.sample(road.length() * i as f64 / 100.0).unwrap();
        assert!((s.position[0].hypot(s.position[1]) - 100.0).abs() < 1e-10);
        assert!((s.curvature - 0.01).abs() < 1e-12);
        assert!((s.position[0] * s.tangent[0] + s.position[1] * s.tangent[1]).abs() < 1e-10);
        let p = road
            .project(s.offset(2.0), Some((s.station, 10.0)))
            .unwrap();
        assert!((p.lateral - 2.0).abs() < 0.001);
        assert!((p.separation - 2.0).abs() < 0.001);
    }
    let obstacle = &road.obstacles[0];
    assert!(obstacle.position[0].abs() < 1e-9);
    assert!((obstacle.position[1] - 97.0).abs() < 1e-9);
    assert!(road.sample(f64::NAN).is_err());
    assert!(road.project([f64::INFINITY, 0.0], None).is_none());
    assert!(road.project([0.0, 0.0], Some((0.0, -1.0))).is_none());
}
#[test]
fn arc_length_refinement_converges_and_seam_projection_is_continuous() {
    let exact = std::f64::consts::TAU * 100.0;
    let coarse = project(&CIRCLE.replace("samples 2048", "samples 512")).unwrap();
    let fine = project(CIRCLE).unwrap();
    assert!((exact - fine.length()).abs() < (exact - coarse.length()).abs() / 15.0);
    for station in [-0.1, 0.0, 0.1] {
        let point = fine.sample(station).unwrap().offset(-3.0);
        let global = fine.project(point, None).unwrap();
        let local = fine
            .project(point, Some((fine.length() - 0.05, 5.0)))
            .unwrap();
        assert!((global.station - local.station).abs() < 1e-12);
        assert!((global.lateral + 3.0).abs() < 0.001);
    }
}
#[test]
fn malformed_or_unbounded_roads_fail_before_use() {
    for (from, to) in [
        ("degree 2", "degree 2.5"),
        ("samples 2048", "samples 8193"),
        ("width 16", "width 0"),
        ("weight 1", "weight 0"),
        ("p8 { point [100,0]", "p8 { point [101,0]"),
        ("fraction 0.25", "fraction 1"),
        ("radius 0.5", "radius -1"),
        ("offset 3", "offset 100"),
        ("height 1", "height 11"),
        ("0,0,0,1,1,2,2,3,3,4,4,4", "0,0,0,2,1,2,2,3,3,4,4,4"),
    ] {
        assert!(project(&CIRCLE.replace(from, to)).is_err(), "accepted {to}");
    }
}
