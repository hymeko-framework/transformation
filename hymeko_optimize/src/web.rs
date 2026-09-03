//! The optimisation projection, callable from a browser.
//!
//! The page hands over the `.hymeko` sources as text; this compiles them through the same module
//! store the desktop uses, projects them, solves, and returns the result as JSON. There is nothing
//! here but plumbing: no formula, no bound, no target voltage. A page that could set one of those
//! would be a second place the design lives, and the whole point of the projection is that there is
//! only one.
//!
//! The network solve needs no GPU and no canvas — it is a linear system and a search — so unlike the
//! renderer this runs in a browser exactly as it runs on a desktop.

use wasm_bindgen::prelude::*;

use hymeko::module_store::module_store::ModuleStore;
use hymeko::util::real_parser::RealParser;
use hymeko_sources::MemoryProvider;

use crate::backend::{multistart, AugmentedLagrangian};
use crate::read::project;

/// Compile, project and solve a declared design.
///
/// `paths` and `texts` are parallel: the model and every file it imports, keyed by the path it is
/// imported as. `starts` is how many points of the deterministic lattice to try; one uses the model's
/// own declared starting point.
///
/// Returns the result as JSON, or a string saying which stage refused and why.
#[wasm_bindgen]
pub fn solve(
    entry: String,
    paths: Vec<String>,
    texts: Vec<String>,
    starts: usize,
) -> Result<String, JsValue> {
    if paths.len() != texts.len() {
        return Err(JsValue::from_str("paths and texts must be the same length"));
    }
    let mut provider = MemoryProvider::new();
    for (path, text) in paths.iter().zip(&texts) {
        provider.insert(path, text);
    }

    let mut store = ModuleStore::new(provider, RealParser);
    let program = store
        .compile(std::path::Path::new(&entry))
        .map_err(|e| JsValue::from_str(&format!("compile: {e:?}")))?;
    let problem = project(&program.ir.clone(), &store.it)
        .map_err(|e| JsValue::from_str(&format!("projection: {e:?}")))?;
    problem
        .check()
        .map_err(|e| JsValue::from_str(&format!("not well posed: {e:?}")))?;

    let backend = AugmentedLagrangian::default();
    let runs = multistart(&backend, &problem, starts.max(1));
    let failures = runs.iter().filter(|r| r.is_err()).count();
    let best = runs
        .into_iter()
        .flatten()
        // Ties go to the earlier start, so the answer does not depend on the order two equally good
        // designs were found in.
        .reduce(|a, b| if b.objective < a.objective { b } else { a });

    let mut out = String::from("{\n");
    out.push_str(&format!(
        "  \"projection\": {{ \"quantities\": {}, \"decisions\": {}, \"nets\": {}, \"branches\": {}, \"scenarios\": {}, \"constraints\": {}, \"relations\": {} }},\n",
        problem.quantities.len(),
        problem.decisions.len(),
        problem.nets.len(),
        problem.branches.len(),
        problem.scenarios.len(),
        problem.constraints.len(),
        problem.relations.len()
    ));
    out.push_str(&format!("  \"starts\": {}, \"failed\": {},\n", starts.max(1), failures));
    match best {
        Some(solved) => {
            out.push_str(&format!("  \"feasible\": {},\n", solved.feasible()));
            out.push_str("  \"result\": ");
            out.push_str(solved.to_json().trim_end());
            out.push('\n');
        }
        None => out.push_str("  \"feasible\": false, \"result\": null\n"),
    }
    out.push_str("}\n");
    Ok(out)
}

/// What the projection made of a model, without solving it.
///
/// For a page that wants to show the topology it read before anybody presses anything — and for one
/// that wants to report a model that does not compile without pretending it has an answer.
#[wasm_bindgen]
pub fn describe(entry: String, paths: Vec<String>, texts: Vec<String>) -> Result<String, JsValue> {
    let mut provider = MemoryProvider::new();
    for (path, text) in paths.iter().zip(&texts) {
        provider.insert(path, text);
    }
    let mut store = ModuleStore::new(provider, RealParser);
    let program = store
        .compile(std::path::Path::new(&entry))
        .map_err(|e| JsValue::from_str(&format!("compile: {e:?}")))?;
    let problem = project(&program.ir.clone(), &store.it)
        .map_err(|e| JsValue::from_str(&format!("projection: {e:?}")))?;

    let mut out = String::from("{\n  \"nets\": [");
    for (i, name) in problem.net_names.iter().enumerate() {
        out.push_str(&format!(
            "{}\"{}{}\"",
            if i > 0 { ", " } else { "" },
            name,
            if problem.ground == Some(i) { " (ground)" } else { "" }
        ));
    }
    out.push_str("],\n  \"branches\": [");
    for (i, name) in problem.branch_names.iter().enumerate() {
        let (a, b) = problem.branches[i].nets();
        out.push_str(&format!(
            "{}{{ \"name\": \"{name}\", \"from\": \"{}\", \"to\": \"{}\" }}",
            if i > 0 { ", " } else { "" },
            problem.net_names[a],
            problem.net_names[b]
        ));
    }
    out.push_str("],\n  \"decisions\": [");
    for (i, &q) in problem.decisions.iter().enumerate() {
        let quantity = &problem.quantities[q];
        out.push_str(&format!(
            "{}{{ \"name\": \"{}\", \"unit\": \"{}\", \"lower\": {}, \"upper\": {}, \"start\": {} }}",
            if i > 0 { ", " } else { "" },
            quantity.name,
            quantity.unit.label(),
            quantity.lower.unwrap_or(f64::NEG_INFINITY),
            quantity.upper.unwrap_or(f64::INFINITY),
            quantity.start.unwrap_or(0.0)
        ));
    }
    out.push_str("],\n  \"scenarios\": [");
    for (i, scenario) in problem.scenarios.iter().enumerate() {
        out.push_str(&format!(
            "{}{{ \"name\": \"{}\", \"branches\": {} }}",
            if i > 0 { ", " } else { "" },
            scenario.name,
            scenario.branches.len()
        ));
    }
    out.push_str("]\n}\n");
    Ok(out)
}
