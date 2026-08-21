#[cfg(test)]
mod test_query_robot {
    use crate::test_helpers::load_and_lower;
    use hymeko_formats::urdf::{urdf_queries, validate_robot_schema};
    use hymeko_query::QueryMatch;
    use hymeko_query::engine::QueryEngine;
    use hymeko_query::{Predicate, ValuePredicate};

    const ROBOT: &str = "../data/robotics/robot_4wh.hymeko";

    /// Helper: extract names from Vec<QueryMatch>
    fn names(matches: &[QueryMatch]) -> Vec<&str> {
        matches.iter().map(|m| m.name.as_str()).collect()
    }

    #[test]
    fn find_robot_links() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::node().and(Predicate::inherits("link")));
        let n = names(&r);
        println!("Robot links: {:?}", n);
        assert_eq!(r.len(), 6);
        for expected in &[
            "base_link",
            "wheel_fr",
            "wheel_fl",
            "wheel_rr",
            "wheel_rl",
            "camera_link",
        ] {
            assert!(n.contains(expected), "{} missing", expected);
        }
    }

    #[test]
    fn find_robot_joints_transitive() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::edge().and(Predicate::inherits("joint")));
        let n = names(&r);
        println!("Robot joints: {:?}", n);
        assert!(n.contains(&"joint_fr"));
        assert!(n.contains(&"camera_joint"));
    }

    #[test]
    fn continuous_joints_specifically() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::edge().and(Predicate::inherits("conti_joint")));
        assert_eq!(r.len(), 4);
    }

    #[test]
    fn fixed_joints() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::edge().and(Predicate::inherits("fixed_joint")));
        assert_eq!(r.len(), 1, "Only camera_joint is fixed");
    }

    #[test]
    fn joints_with_plus_ref_to_link() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(
            &Predicate::edge()
                .and(Predicate::inherits("joint"))
                .and(Predicate::HasPlusRef(Box::new(Predicate::inherits("link")))),
        );
        println!("Joints with +link: {:?}", names(&r));
        assert!(r.len() >= 5);
    }

    #[test]
    fn joints_with_minus_ref_to_link() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::edge().and(Predicate::inherits("joint")).and(
            Predicate::HasMinusRef(Box::new(Predicate::inherits("link"))),
        ));
        println!("Joints with -link (child): {:?}", names(&r));
        assert!(r.len() >= 5);
    }

    #[test]
    fn heavy_links() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::node().and(Predicate::inherits("link")).and(
            Predicate::ChildValue("mass".into(), ValuePredicate::NumGt(10.0)),
        ));
        assert_eq!(r.len(), 1);
        assert!(names(&r).contains(&"base_link"));
    }

    #[test]
    fn name_prefix_wheels() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let r = engine.query(&Predicate::node().and(Predicate::name_prefix("wheel")));
        assert!(r.len() >= 4, "At least 4 wheel nodes, got {}", r.len());
        let n = names(&r);
        for expected in &["wheel_fr", "wheel_fl", "wheel_rr", "wheel_rl"] {
            assert!(n.contains(expected), "{} missing", expected);
        }
    }

    #[test]
    fn bindings_carry_weight_annotations() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let joints = engine.query(
            &Predicate::edge()
                .and(Predicate::inherits("conti_joint"))
                .and(Predicate::HasPlusRef(Box::new(Predicate::named(
                    "base_link",
                )))),
        );
        assert_eq!(joints.len(), 4);
        // Verify bindings carry weight annotations on plus refs
        for m in &joints {
            let plus_bindings: Vec<_> = m.arc_bindings.iter().filter(|b| b.sign == 1).collect();
            assert!(
                !plus_bindings.is_empty(),
                "Joint {} must have plus-signed bindings",
                m.name
            );
            for b in &plus_bindings {
                println!(
                    "Joint {} +ref → {} weights: {:?}",
                    m.name, b.target_name, b.weights
                );
                assert!(
                    b.weights.is_some(),
                    "Plus ref on {} must have origin weights",
                    m.name
                );
            }
        }
    }

    #[test]
    fn bindings_have_parent_child_axis() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let joints = engine.query(&Predicate::edge().and(Predicate::inherits("conti_joint")));
        for m in &joints {
            let plus_count = m.arc_bindings.iter().filter(|b| b.sign == 1).count();
            let minus_count = m.arc_bindings.iter().filter(|b| b.sign == -1).count();
            println!(
                "Joint {}: {} plus refs, {} minus refs, {} total bindings",
                m.name,
                plus_count,
                minus_count,
                m.arc_bindings.len()
            );
            assert!(
                plus_count >= 1,
                "Joint {} needs at least 1 parent (+) binding",
                m.name
            );
            assert!(
                minus_count >= 1,
                "Joint {} needs at least 1 child (-) binding",
                m.name
            );
        }
    }

    #[test]
    fn query_first_finds_base_link() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let first = engine.query_first(&Predicate::node().and(Predicate::named("base_link")));
        assert!(first.is_some());
        assert_eq!(first.unwrap().name, "base_link");
    }

    #[test]
    fn query_first_returns_none() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let first =
            engine.query_first(&Predicate::node().and(Predicate::named("nonexistent_link")));
        assert!(first.is_none());
    }

    #[test]
    fn query_iter_lazy_count() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let pred = Predicate::node().and(Predicate::inherits("link"));
        // Take only first 3 — iterator should not evaluate the rest
        let first_three: Vec<_> = engine.query_iter(&pred).take(3).collect();
        assert_eq!(first_three.len(), 3);
    }

    #[test]
    fn schema_validation_passes() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let errors = validate_robot_schema(&compiled.ir, &store.it);
        if !errors.is_empty() {
            for e in &errors {
                eprintln!("  Schema error: {}", e);
            }
        }
        assert!(errors.is_empty(), "Got {} schema errors", errors.len());
    }

    #[test]
    fn batch_query_all_urdf() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let engine = QueryEngine::new(&compiled.ir, &store.it);
        let results = engine.query_batch(&urdf_queries());
        println!("--- URDF query results ---");
        for (label, matches) in &results {
            println!(
                "  {}: {} matches → {:?}",
                label,
                matches.len(),
                names(matches)
            );
        }
        let links = results.iter().find(|(l, _)| l == "links");
        assert!(links.unwrap().1.len() >= 6);
    }
}
