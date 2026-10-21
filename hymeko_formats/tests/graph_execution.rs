use hymeko::common::ids::NodeId;
use hymeko::ir::incidence::IncidenceOrder;
use hymeko_formats::graph_execution::PageRankSource;
use hymeko_graph::{PageRankWorkspace, RankOrdering};
use std::{collections::BTreeMap, path::PathBuf};

struct SourceCase;
impl SourceCase {
    fn path(file: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("data/graph_execution")
            .join(file)
    }
    fn scores(source: &PageRankSource) -> BTreeMap<String, f64> {
        let mut workspace = PageRankWorkspace::new();
        let result = source.run(&mut workspace).unwrap();
        assert!(result.progress.converged);
        result
            .scores
            .iter()
            .enumerate()
            .map(|(node, score)| {
                (
                    source.node_path(NodeId::new(node)).unwrap().join("."),
                    *score,
                )
            })
            .collect()
    }
}

#[test]
fn files_profiles_and_declaration_order_use_the_same_canonical_graph_solver() {
    let original = PageRankSource::load(&SourceCase::path("run.hymeko"), "job").unwrap();
    let permuted =
        PageRankSource::load(&SourceCase::path("run_permuted.hymeko"), "permuted_ordered").unwrap();
    assert_eq!(original.graph().program().ir.nodes.len(), 32);
    assert_eq!(original.graph().program().ir.arcs.len(), 32);
    assert_eq!(permuted.graph().program().ir.nodes.len(), 32);
    assert!(!original.profile().program().ir.nodes.is_empty());
    assert_eq!(original.settings().workers.get(), 1);
    assert_eq!(original.settings().iteration.damping, 0.85);
    assert_eq!(
        permuted.settings().preparation.ordering,
        RankOrdering::FirstEncounter
    );
    assert_eq!(
        permuted.prepare().unwrap().plan_stats().unwrap().ordering,
        IncidenceOrder::FirstEncounter
    );
    assert_eq!(
        original.graph().program().canon.hash(),
        permuted.graph().program().canon.hash()
    );
    let a = SourceCase::scores(&original);
    let b = SourceCase::scores(&permuted);
    assert_eq!(a.keys().collect::<Vec<_>>(), b.keys().collect::<Vec<_>>());
    assert!(
        a.iter()
            .map(|(name, value)| (value - b[name]).abs())
            .sum::<f64>()
            < 2e-10
    );
    assert!(original.node_path(NodeId::new(usize::MAX)).is_none());
}

#[test]
fn inherited_worker_override_drives_parallel_execution_and_errors_are_explicit() {
    let serial =
        PageRankSource::load(&SourceCase::path("run_permuted.hymeko"), "permuted_job").unwrap();
    let parallel = PageRankSource::load(
        &SourceCase::path("run_permuted.hymeko"),
        "permuted_parallel",
    )
    .unwrap();
    assert_eq!(parallel.settings().workers.get(), 4);
    assert_eq!(
        parallel.settings().preparation.ordering,
        RankOrdering::Canonical
    );
    assert!(parallel.prepare().unwrap().plan_stats().is_none());
    let a = SourceCase::scores(&serial);
    let b = SourceCase::scores(&parallel);
    assert!(
        a.iter()
            .map(|(name, value)| (value - b[name]).abs())
            .sum::<f64>()
            < 2e-10
    );
    assert!(PageRankSource::load(&SourceCase::path("run.hymeko"), "not_a_job").is_err());
    assert!(PageRankSource::load(&SourceCase::path("missing.hymeko"), "job").is_err());
}
