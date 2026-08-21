#[cfg(test)]
mod test_query_meta {
    use crate::test_helpers::load_and_lower;
    use hymeko_query::Predicate;
    use hymeko_query::QueryMatch;
    use hymeko_query::engine::QueryEngine;

    const META: &str = "../data/robotics/meta_kinematics.hymeko";

    /// Helper: extract names from Vec<QueryMatch>
    fn names(matches: &[QueryMatch]) -> Vec<&str> {
        matches.iter().map(|m| m.name.as_str()).collect()
    }

    #[test]
    fn find_all_nodes() {
        let (store, compiled) = load_and_lower(META).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::node());
        assert!(!r.is_empty());
        println!("All nodes ({}): {:?}", r.len(), names(&r));
    }

    #[test]
    fn find_all_edges() {
        let (store, compiled) = load_and_lower(META).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::edge());
        assert!(!r.is_empty());
        println!("All edges ({}): {:?}", r.len(), names(&r));
    }

    #[test]
    fn find_axes_by_inheritance() {
        let (store, compiled) = load_and_lower(META).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::node().and(Predicate::inherits("axis_definition")));
        let n = names(&r);
        println!("Axes: {:?}", n);
        assert_eq!(r.len(), 4);
        assert!(n.contains(&"AXIS_X"));
        assert!(n.contains(&"AXIS_Y"));
        assert!(n.contains(&"AXIS_Z"));
        assert!(n.contains(&"AXIS_M_Z"));
    }

    #[test]
    fn transitive_inheritance_joint_types() {
        let (store, compiled) = load_and_lower(META).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::edge().and(Predicate::inherits("joint")));
        let n = names(&r);
        println!("Edges inheriting from 'joint': {:?}", n);
        assert!(n.contains(&"fixed_joint"));
        assert!(n.contains(&"rev_joint"));
        assert!(n.contains(&"conti_joint"));
        assert!(n.contains(&"prismatic_joint"));
    }

    #[test]
    fn transitive_inheritance_meta_element() {
        let (store, compiled) = load_and_lower(META).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::node().and(Predicate::inherits("meta_element")));
        let n = names(&r);
        println!("Inherits meta_element: {:?}", n);
        assert!(n.contains(&"link"));
        assert!(n.contains(&"frame"));
        assert!(n.contains(&"sensor"));
        assert!(n.contains(&"control"));
    }

    #[test]
    fn controllers_by_inheritance() {
        let (store, compiled) = load_and_lower(META).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::node().and(Predicate::inherits("meta_controller")));
        assert_eq!(r.len(), 5, "Should find 5 controller types");
    }

    #[test]
    fn sensors_by_inheritance() {
        let (store, compiled) = load_and_lower(META).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::node().and(Predicate::inherits("sensor")));
        assert_eq!(r.len(), 2, "rgb_camera and laser_scanner");
    }

    #[test]
    fn not_predicate() {
        let (store, compiled) = load_and_lower(META).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let all = engine.query(&Predicate::node());
        let non_link = engine.query(&Predicate::node().and(Predicate::named("link").not()));
        assert_eq!(non_link.len(), all.len() - 1);
    }

    #[test]
    fn or_predicate() {
        let (store, compiled) = load_and_lower(META).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine
            .query(&Predicate::node().and(Predicate::named("link").or(Predicate::named("frame"))));
        assert_eq!(r.len(), 2);
    }
}
