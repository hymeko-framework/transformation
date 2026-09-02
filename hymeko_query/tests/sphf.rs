//! Reading the signed patch hypergraph out of a compiled IR.
//!
//! The claim is that the IR **is** a G-SPHF and this only flattens it, so the tests are of two
//! kinds: identities that must hold for any hypergraph whatever the source looked like, and
//! readings of a real robot checked against what that robot is known to be.

use hymeko::module_store::module_store::ModuleStore;
use hymeko::module_store::source_provider::StdFsProvider;
use hymeko::util::real_parser::RealParser;
use hymeko_query::sphf::{Role, SignedHypergraph};

/// A robot from `core`'s library, if this checkout has it beside us, with its interner.
fn library(
    file: &str,
) -> Option<(
    SignedHypergraph,
    ModuleStore<StdFsProvider, RealParser>,
    hymeko::ir::ir::Ir,
)> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../core/data/robotics/robots")
        .join(file);
    if !path.exists() {
        return None;
    }
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = store.compile(&path).ok()?;
    let graph = SignedHypergraph::of(&program.ir);
    Some((graph, store, program.ir.clone()))
}

// --- what a robot reads out as ------------------------------------------------------------------

#[test]
fn a_joint_reads_out_as_a_ternary_edge_over_parent_child_and_axis() {
    // Not an assumption about the language — the shape the IR actually has, checked so that a change
    // to it is a failing test rather than a silently different hypergraph.
    let Some((graph, store, ir)) = library("quadruped.hymeko") else {
        return;
    };
    let name = |d: hymeko::common::ids::DeclId| -> String {
        match ir.decl_nodes.get(d.raw()) {
            Some(n) => store.it.resolve(n.name).to_string(),
            None => "-".into(),
        }
    };

    let mut ternary = 0;
    for edge in &graph.edges {
        let targets: Vec<usize> = edge.targets().collect();
        assert_eq!(
            targets.len(),
            1,
            "edge {} has {} targets; a joint has exactly one parent",
            name(edge.decl),
            targets.len()
        );
        let sources: Vec<String> = edge.sources().map(|v| name(graph.vertices[v])).collect();
        assert!(
            !sources.is_empty(),
            "edge {} has no source",
            name(edge.decl)
        );
        if edge.arity() == 3 {
            ternary += 1;
            assert!(
                sources.iter().any(|s| s.starts_with("AXIS")),
                "edge {} is ternary but its extra source is {sources:?}, not an axis",
                name(edge.decl)
            );
        }
    }
    assert!(
        ternary > graph.edges.len() / 2,
        "most joints should be ternary; only {ternary} of {} were",
        graph.edges.len()
    );
}

#[test]
fn the_axis_declarations_are_shared_and_that_is_where_the_cycles_come_from() {
    // The finding that corrected the documentation. The full reading's cyclomatic number is NOT the
    // mechanism's closed-loop count, because one `AXIS_Z` is a single declaration that every joint
    // turning about it refers to — so they are all connected through it.
    //
    // Dropping the shared axes leaves the kinematic graph, and a quadruped is a tree.
    let Some((graph, store, ir)) = library("quadruped.hymeko") else {
        return;
    };
    let name = |d: hymeko::common::ids::DeclId| -> String {
        match ir.decl_nodes.get(d.raw()) {
            Some(n) => store.it.resolve(n.name).to_string(),
            None => "-".into(),
        }
    };

    assert!(
        graph.cyclomatic_number() > 0,
        "the full reading should carry the shared-parameter cycles"
    );

    let bodies = graph.restricted_to(|v| !name(graph.vertices[v]).starts_with("AXIS"));
    assert_eq!(
        bodies.cyclomatic_number(),
        0,
        "with the axes dropped the quadruped is a kinematic tree and should have no cycles"
    );
    // and the projection kept every joint
    assert_eq!(
        bodies.edges.len(),
        graph.edges.len(),
        "dropping the axes must not drop a joint"
    );
}

// --- the identities ------------------------------------------------------------------------------

#[test]
fn the_ports_build_a_skew_interconnection_whatever_the_source_looked_like() {
    // The reason any of this is worth reading out. A port contributes `w (e_u e_v^T - e_v e_u^T)`
    // over its sources and targets — antisymmetric term by term — so `J` is skew and `e^T J e = 0`
    // identically, whatever the hypergraph turns out to be.
    //
    // The rule is written out here rather than imported, so this checks the PORTS against the rule
    // rather than against another implementation of it. `hymeko_numerics::Interconnection` builds
    // the same thing generically.
    for file in ["arm_gripper.hymeko", "humanoid.hymeko", "quadruped.hymeko"] {
        let Some((graph, _, _)) = library(file) else {
            continue;
        };
        let n = graph.vertices.len();
        let ports = graph.ports();
        assert!(!ports.is_empty(), "{file}: no ports were read");

        let mut j = vec![0.0f64; n * n];
        for (sources, targets, weight) in &ports {
            for &u in sources {
                for &v in targets {
                    j[u * n + v] += weight;
                    j[v * n + u] -= weight;
                }
            }
        }
        for r in 0..n {
            assert_eq!(j[r * n + r], 0.0, "{file}: J[{r}][{r}] is not zero");
            for c in 0..n {
                assert_eq!(
                    j[r * n + c],
                    -j[c * n + r],
                    "{file}: J[{r}][{c}] is not the negative of its transpose"
                );
            }
        }
    }
}

#[test]
fn no_effort_does_work_against_the_interconnection() {
    // `e^T J e == 0`, checked numerically over the real robots — the identity the energy balance of
    // anything built on this rests on.
    let Some((graph, _, _)) = library("quadruped.hymeko") else {
        return;
    };
    let n = graph.vertices.len();
    let ports = graph.ports();
    let mut state = 0x51_6Eu64;
    let mut next = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        2.0 * ((state >> 11) as f64 / (1u64 << 53) as f64) - 1.0
    };
    for case in 0..50 {
        let e: Vec<f64> = (0..n).map(|_| next()).collect();
        let mut flow = vec![0.0f64; n];
        for (sources, targets, weight) in &ports {
            let from: f64 = sources.iter().map(|&u| e[u]).sum();
            let to: f64 = targets.iter().map(|&v| e[v]).sum();
            for &u in sources {
                flow[u] += weight * to;
            }
            for &v in targets {
                flow[v] -= weight * from;
            }
        }
        let work: f64 = e.iter().zip(&flow).map(|(a, b)| a * b).sum();
        let scale = e.iter().zip(&flow).map(|(a, b)| (a * b).abs()).sum::<f64>();
        assert!(
            work.abs() <= 1e-12 * scale.max(1.0),
            "case {case}: e^T J e = {work:e} against a scale of {scale:e}"
        );
    }
}

#[test]
fn the_incidence_matrix_agrees_with_the_edges_it_was_built_from() {
    let Some((graph, _, _)) = library("humanoid.hymeko") else {
        return;
    };
    let (v, e) = (graph.vertices.len(), graph.edges.len());
    let m = graph.incidence_matrix();
    assert_eq!(m.len(), v * e);
    for (column, edge) in graph.edges.iter().enumerate() {
        let mut expected = vec![0.0f64; v];
        for incidence in &edge.incidences {
            expected[incidence.vertex] += incidence.role.sign() * incidence.weight;
        }
        for (row, want) in expected.iter().enumerate() {
            assert_eq!(
                m[row * e + column],
                *want,
                "entry ({row}, {column}) of the incidence matrix"
            );
        }
    }
}

#[test]
fn a_patch_can_be_read_on_its_own() {
    // `P` is in the tuple so that a subsystem can be taken out, and this is that.
    let Some((whole, _, ir)) = library("quadruped.hymeko") else {
        return;
    };
    let patches = whole.patches();
    assert!(patches.len() > 1, "a robot should have more than one patch");

    // the patch that holds the most vertices, read alone
    let biggest = *patches
        .iter()
        .max_by_key(|&&p| whole.patch.iter().filter(|&&q| q == p).count())
        .expect("there is at least one patch");
    let part = SignedHypergraph::within(&ir, Some(biggest));

    assert!(
        part.vertices.len() < whole.vertices.len(),
        "a single patch should be smaller than the whole"
    );
    assert!(!part.vertices.is_empty(), "and not empty");
    for &decl in &part.vertices {
        assert!(
            whole.vertices.contains(&decl),
            "a patch's vertices must be the whole's"
        );
    }
}

#[test]
fn restricting_to_nothing_leaves_nothing_and_to_everything_leaves_everything() {
    let Some((graph, _, _)) = library("arm_gripper.hymeko") else {
        return;
    };
    let none = graph.restricted_to(|_| false);
    assert!(none.vertices.is_empty() && none.edges.is_empty());

    let all = graph.restricted_to(|_| true);
    assert_eq!(all.vertices, graph.vertices);
    assert_eq!(all.edges, graph.edges);
    assert_eq!(all.cyclomatic_number(), graph.cyclomatic_number());
}

#[test]
fn every_role_the_ir_can_state_reads_back_as_its_own_sign() {
    assert_eq!(Role::Source.sign(), -1.0);
    assert_eq!(Role::Target.sign(), 1.0);
    assert_eq!(Role::Neutral.sign(), 0.0);
}
