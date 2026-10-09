use hymeko_formats::environment::{
    CompiledWorld,
    maze::{Profile, Topology, search::MazeNavigation},
};

#[test]
fn loop_generation_preserves_trees_and_opens_exactly_the_requested_passages() {
    for profile in [Profile::Hex, Profile::Polar] {
        let topology = Topology::new(profile);
        let (tree, source, old, _) = topology.generate(2).unwrap();
        let (same, zero, _, _) = topology.generate_loops(2, 0).unwrap();
        assert_eq!(source, zero);
        assert_eq!(tree.digest(), same.digest());
        for loops in [3, topology.max_loops()] {
            let (world, source, geometry, checks) = topology.generate_loops(2, loops).unwrap();
            assert!(checks.iter().all(|c| c["passed"] == true));
            assert_eq!(geometry.passages().len(), old.passages().len() + loops);
            assert!(
                old.passages()
                    .iter()
                    .all(|p| geometry.passages().contains(p))
            );
            assert_eq!(
                world.digest(),
                CompiledWorld::compile(&source).unwrap().digest()
            );
            assert!(MazeNavigation::new(&world).is_ok());
        }
        assert!(
            topology
                .generate_loops(0, topology.max_loops() + 1)
                .is_err()
        );
    }
}

#[test]
fn all_pairs_agree_on_cyclic_maps_and_have_clear_geometric_witnesses() {
    for profile in [Profile::Hex, Profile::Polar] {
        let (world, _, geometry, _) = Topology::new(profile).generate_loops(1, 3).unwrap();
        let navigation = MazeNavigation::new(&world).unwrap();
        for start in 0..geometry.cells().len() {
            for goal in 0..geometry.cells().len() {
                let runs = navigation.evaluate(start, goal, 0).unwrap();
                for run in &runs {
                    assert_eq!(run.cost, runs[0].cost);
                    assert!(run.samples_ns.is_empty());
                    assert_eq!(run.route_cells.first(), Some(&start));
                    assert_eq!(run.route_cells.last(), Some(&goal));
                    if start == goal {
                        assert_eq!(run.cost, 0);
                    }
                }
                assert!(runs[0].counters.is_none());
                assert_eq!(runs[2].counters.as_ref().unwrap().reopened, 0);
            }
        }
        assert!(navigation.evaluate(geometry.cells().len(), 0, 0).is_err());
        assert!(navigation.evaluate(0, 1, 1001).is_err());
        assert_eq!(
            navigation.evaluate(0, 1, 0).unwrap()[2].counters,
            navigation.evaluate(0, 1, 0).unwrap()[2].counters
        );
    }
}

#[test]
fn known_fully_open_polar_route_has_three_unit_hops() {
    let topology = Topology::new(Profile::Polar);
    let (world, _, _, _) = topology.generate_loops(0, topology.max_loops()).unwrap();
    let navigation = MazeNavigation::new(&world).unwrap();
    for run in navigation.evaluate(0, 3, 0).unwrap() {
        assert_eq!(run.cost, 3);
    }
    for run in navigation.evaluate(0, 6, 0).unwrap() {
        assert_eq!(run.cost, 1);
    }
}

#[test]
fn malformed_topology_is_rejected_before_search() {
    let (world, source, _, _) = Topology::new(Profile::Polar).generate_loops(0, 3).unwrap();
    assert!(MazeNavigation::new(&world).is_ok());
    for suffix in [
        "@illegal { (~ cell_0, ~ cell_8); }",
        "@illegal { (+ cell_0, - cell_6); }",
        "@illegal { (~ cell_0, ~ maze); }",
    ] {
        let bad = CompiledWorld::compile(&format!("{source}\n{suffix}")).unwrap();
        assert!(MazeNavigation::new(&bad).is_err());
    }
    let marker = source.find("maze_loops").unwrap();
    let tail = &source[marker..];
    let wrong = format!("{}{}", &source[..marker], tail.replacen("3", "2", 1));
    assert!(MazeNavigation::new(&CompiledWorld::compile(&wrong).unwrap()).is_err());
}
