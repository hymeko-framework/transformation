//! Execute a compiled graph resource using its declarative run profile.
use hymeko::common::ids::NodeId;
use hymeko_formats::graph_execution::PageRankSource;
use hymeko_graph::PageRankWorkspace;
use serde_json::json;
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let arguments: Vec<_> = std::env::args().skip(1).collect();
    if !(1..=2).contains(&arguments.len()) {
        return Err("usage: graph_rank PROFILE.hymeko [JOB_PATH]".into());
    }
    let job = arguments.get(1).map_or("job", String::as_str);
    let source = PageRankSource::load(Path::new(&arguments[0]), job)?;
    let settings = source.settings();
    let mut workspace = PageRankWorkspace::new();
    let result = source.run(&mut workspace)?;
    let ranks: Vec<_> = result.scores.iter().enumerate().map(|(node, score)| {
        json!({"node_id":node,"path":source.node_path(NodeId::new(node)),"score":score})
    }).collect();
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({
            "model": source.graph().program().root.0,
            "profile": source.profile().program().root.0,
            "job": job,
            "workers": settings.workers.get(),
            "ordering_policy": format!("{:?}", settings.preparation.ordering),
            "execution": if settings.workers.get()>1 {"parallel_pull"} else {"cost_selected_serial"},
            "iterations": result.progress.iterations,
            "converged": result.progress.converged,
            "error_bound_l1": result.progress.error_bound_l1,
            "ranks": ranks,
        }))?
    );
    if !result.progress.converged {
        return Err("iteration cap exhausted".into());
    }
    Ok(())
}
