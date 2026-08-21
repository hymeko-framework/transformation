//! CORE additive kinematics extension (2026-07-23): GeometryShape::Capsule + GeometryInfo.collision, and the
//! hard-fail-on-unknown-shape behaviour. Compatibility: Box/Cylinder/Sphere parse unchanged; absent collision => None.
#[cfg(test)]
mod test_geom_extension {
    use crate::test_helpers::load_and_lower;
    use hymeko_query::engine::QueryEngine;
    use hymeko_query::kinematics::kinematic::{
        extract_kinematic_model, CollisionMask, GeometryShape, KinematicModel,
    };

    const GEOM_EXT: &str = "../data/robotics/test_geom_ext.hymeko";
    const GEOM_UNKNOWN: &str = "../data/robotics/test_geom_unknown.hymeko";
    const GALAMBOS: &str = "../data/robotics/galambos_planar.hymeko";

    fn model(path: &str, name: &str) -> KinematicModel {
        let (store, compiled) = load_and_lower(path).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        extract_kinematic_model(&engine, name)
    }

    #[test]
    fn capsule_parses_with_dimensions_and_collision() {
        let m = model(GEOM_EXT, "test_geom_ext");
        let cap = m.links.iter().find(|l| l.name == "rod_cap").expect("rod_cap link");
        let g = cap.geometry.as_ref().expect("rod_cap geometry");
        assert_eq!(g.shape, GeometryShape::Capsule);
        assert_eq!(g.dimensions, vec![0.16, 0.012]); // [length, radius]
        assert_eq!(g.collision, Some(CollisionMask { contype: 1, conaffinity: 3 }));
    }

    #[test]
    fn absent_collision_block_is_none() {
        let m = model(GEOM_EXT, "test_geom_ext");
        let hub = m.links.iter().find(|l| l.name == "hub_box").expect("hub_box link");
        let g = hub.geometry.as_ref().expect("hub_box geometry");
        assert_eq!(g.shape, GeometryShape::Box);
        assert_eq!(g.collision, None); // historical default preserved
    }

    #[test]
    fn box_geometry_parses_unchanged() {
        // The historical galambos_planar uses box links with no collision block; must be unaffected by the extension.
        let m = model(GALAMBOS, "galambos_planar");
        let mut boxes = 0;
        for l in &m.links {
            if let Some(g) = &l.geometry {
                assert_eq!(g.shape, GeometryShape::Box);
                assert_eq!(g.collision, None);
                boxes += 1;
            }
        }
        assert!(boxes >= 4, "expected the galambos box links to parse, got {boxes}");
    }

    #[test]
    #[should_panic(expected = "unknown geometry shape")]
    fn unknown_shape_hard_fails_not_silent() {
        let _ = model(GEOM_UNKNOWN, "test_geom_unknown");
    }
}
