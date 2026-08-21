//! String-form predicate expression evaluator.
//!
//! Supports the same surface as `queries/standard.qlist`:
//!
//! ```text
//!   KIND(<name>)                — decl whose first inherited base is <name>
//!   INHERITS(<name>)            — decl transitively inheriting <name>
//!   SCOPEDIN(<name>)            — decl has an ancestor inheriting <name>
//!   HASARCREF(<sign>, <inner>)  — edge with an arc-ref of <sign> (+1/-1)
//!                                  pointing at a decl matching <inner>
//!   <a> AND <b>                  — conjunction
//!   ANY                          — always true
//! ```
//!
//! Single source of truth — both `hymeko_py::interface_python::api` and
//! `hymeko_wasm::compile` were carrying byte-identical copies before
//! consolidation.

use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{DeclKind, Ir};
use hymeko::resolution::string_table::StringTable;

/// Evaluate a predicate expression (with optional `AND`-chained
/// conjunction) against a single declaration.
pub fn match_expr(expr: &str, did: DeclId, ir: &Ir, st: &StringTable) -> bool {
    expr.split(" AND ").all(|p| match_atom(p.trim(), did, ir, st))
}

/// Evaluate a single predicate atom (one of `KIND`, `INHERITS`,
/// `SCOPEDIN`, `HASARCREF`, `ANY`) against a declaration.
pub fn match_atom(atom: &str, did: DeclId, ir: &Ir, st: &StringTable) -> bool {
    if atom == "ANY" { return true; }
    if let Some(rest) = atom.strip_prefix("KIND(") {
        let name = rest.trim_end_matches(')');
        return decl_kind_name(did, ir, st) == name;
    }
    if let Some(rest) = atom.strip_prefix("INHERITS(") {
        let name = rest.trim_end_matches(')');
        return decl_inherits(did, name, ir, st);
    }
    if let Some(rest) = atom.strip_prefix("SCOPEDIN(") {
        let name = rest.trim_end_matches(')');
        return decl_scoped_in(did, name, ir, st);
    }
    if let Some(rest) = atom.strip_prefix("HASARCREF(") {
        let rest = rest.trim_end_matches(')');
        let (sign_s, inner) = rest.split_once(',').unwrap_or((rest, ""));
        let sign: i8 = sign_s.trim().trim_start_matches('+').parse().unwrap_or(0);
        return has_arc_ref(did, sign, inner.trim(), ir, st);
    }
    false
}

/// Resolve the first-base name of a decl (used by `KIND(...)`).
pub fn decl_kind_name<'a>(did: DeclId, ir: &'a Ir, st: &'a StringTable) -> &'a str {
    let decl = &ir.decl_nodes[did.0];
    match decl.kind {
        DeclKind::Node => {
            if let Some(nid) = ir.as_node(did) {
                if let Some(b) = ir.nodes[nid.0].bases.first() {
                    return st.resolve(ir.decl_nodes[b.target().0].name);
                }
            }
            ""
        }
        DeclKind::Edge => {
            if let Some(eid) = ir.as_edge(did) {
                if let Some(b) = ir.edges[eid.0].bases.first() {
                    return st.resolve(ir.decl_nodes[b.target().0].name);
                }
            }
            ""
        }
        DeclKind::HyperArc => "",
    }
}

/// Walks the inheritance graph from `did` and tests for `target_name`.
pub fn decl_inherits(did: DeclId, target_name: &str, ir: &Ir, st: &StringTable) -> bool {
    let mut visited = std::collections::HashSet::new();
    let mut stack = vec![did];
    while let Some(d) = stack.pop() {
        if !visited.insert(d) { continue; }
        let decl = &ir.decl_nodes[d.0];
        if st.resolve(decl.name) == target_name { return true; }
        match decl.kind {
            DeclKind::Node => {
                if let Some(nid) = ir.as_node(d) {
                    for b in &ir.nodes[nid.0].bases { stack.push(b.target()); }
                }
            }
            DeclKind::Edge => {
                if let Some(eid) = ir.as_edge(d) {
                    for b in &ir.edges[eid.0].bases { stack.push(b.target()); }
                }
            }
            _ => {}
        }
    }
    false
}

/// Walks ancestor chain checking if any ancestor inherits / matches `name`.
pub fn decl_scoped_in(did: DeclId, name: &str, ir: &Ir, st: &StringTable) -> bool {
    let mut cur = ir.decl_nodes[did.0].parent;
    while cur.is_some() {
        if decl_inherits(cur, name, ir, st) { return true; }
        if st.resolve(ir.decl_nodes[cur.0].name) == name { return true; }
        cur = ir.decl_nodes[cur.0].parent;
    }
    false
}

/// Check that an edge has at least one signed arc-ref of the given
/// sign whose target matches the inner predicate.
pub fn has_arc_ref(did: DeclId, sign: i8, inner: &str, ir: &Ir, st: &StringTable) -> bool {
    let Some(eid) = ir.as_edge(did) else { return false };
    for &aid in &ir.edges[eid.0].arcs {
        for r in &ir.arcs[aid.0].refs {
            if r.sign() != sign { continue; }
            if match_expr(inner, r.target(), ir, st) { return true; }
        }
    }
    false
}
