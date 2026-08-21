//! Snapshot + DOT emitter — single source of truth for the
//! signed-incidence-hypergraph IR's debug visualisation.
//!
//! Replaces byte-identical (modulo lifetime annotations) copies that
//! previously lived in `hymeko_py::interface_python::api` and
//! `hymeko_wasm::compile`.  Both call-sites now delegate to this
//! module.
//!
//! Owned `String` shapes (matching the wasm-bindgen surface that the
//! browser demo consumes) — the small per-call allocation cost is
//! negligible compared to serde_json::to_string at the same call site.

use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{DeclKind, Ir, ValueR};
use hymeko::resolution::string_table::StringTable;

use serde::{Deserialize, Serialize};


// ─── Snapshot DTOs (JSON-shaped) ────────────────────────────────────


#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ArcDto {
    pub sign: i8,
    pub target_id: usize,
    pub target_name: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct NodeDto {
    pub id: usize,
    pub name: String,
    /// "Node" | "Edge" | "HyperArc"
    pub kind: String,
    /// Base types this decl inherits from (resolved, first-level).
    pub bases: Vec<String>,
    /// Annotation tags attached at declaration (e.g. `<temperature>`).
    pub tags: Vec<String>,
    /// The decl's rendered field value, if any (e.g. `"m"`, `[1, 0, 0]`, `1.5`, or a referenced
    /// decl's name). Lets a viewer show leaf "attribute" decls on a node's HUD instead of as
    /// separate vertices. `None` when the decl carries no value.
    #[serde(default)]
    pub value: Option<String>,
    /// Signed arc references — populated only for Edge decls.
    pub arcs: Vec<ArcDto>,
}

/// Render a field value to a compact display string for a node's HUD. Pure; refs resolve to the
/// target decl's name (the structural link is already in `relationships`).
fn render_value(v: &ValueR, ir: &Ir, st: &StringTable) -> String {
    match v {
        ValueR::Str(s) => format!("\"{}\"", st.resolve(*s)),
        ValueR::Num(n) => {
            if n.fract() == 0.0 && n.abs() < 1e15 {
                format!("{}", *n as i64)
            } else {
                format!("{n}")
            }
        }
        ValueR::List(items) => {
            let parts: Vec<String> = items.iter().map(|it| render_value(it, ir, st)).collect();
            format!("[{}]", parts.join(", "))
        }
        ValueR::Ref(d) => {
            if d.is_none() {
                "?".to_string()
            } else {
                st.resolve(ir.decl_nodes[d.0].name).to_string()
            }
        }
    }
}

/// A named, annotated relationship between two decls, by `DeclId`.
/// `kind`:
///   "scope" — containment: child → enclosing parent;
///   "isa"   — template/inheritance: decl → first-level base type;
///   "ref"   — field reference (`a -> b`): decl → the decl its value points at.
/// Signed hyperedge *membership* stays in `NodeDto::arcs` (the primary
/// hyperedge structure); these are the *secondary* structural relations.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RelDto {
    pub kind: String,
    pub from: usize,
    pub to: usize,
}

/// Collect every `Ref(DeclId)` target reachable inside a field value
/// (a bare ref, or refs nested in a list).
fn collect_ref_targets(v: &ValueR, out: &mut Vec<DeclId>) {
    match v {
        ValueR::Ref(d) => out.push(*d),
        ValueR::List(items) => {
            for it in items {
                collect_ref_targets(it, out);
            }
        }
        _ => {}
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SnapshotDto {
    pub node_count: usize,
    pub edge_count: usize,
    pub arc_count: usize,
    /// All `Node` decls (plain vertices).
    pub nodes: Vec<NodeDto>,
    /// All `Edge` decls (hyperedges).
    pub edges: Vec<NodeDto>,
    /// Named, annotated relationships (scope + isa) keyed by `DeclId`.
    pub relationships: Vec<RelDto>,
}


// ─── Snapshot construction ──────────────────────────────────────────


/// Build a `SnapshotDto` from an IR + its string table.
///
/// Used by both the Python wheel (`PyHypergraphIR::snapshot_json`) and
/// the wasm demo (`CompiledDoc::snapshot_json`) — exposed via
/// `snapshot_json()` below.
pub fn snapshot(ir: &Ir, st: &StringTable) -> SnapshotDto {
    let mk = |did: DeclId, with_arcs: bool| -> NodeDto {
        let decl = &ir.decl_nodes[did.0];
        let name = st.resolve(decl.name).to_string();
        let kind_str = match decl.kind {
            DeclKind::Node => "Node",
            DeclKind::Edge => "Edge",
            DeclKind::HyperArc => "HyperArc",
        };

        let bases: Vec<String> = match decl.kind {
            DeclKind::Node => ir
                .as_node(did)
                .map(|nid| ir.nodes[nid.0].bases.iter()
                    .map(|b| st.resolve(ir.decl_nodes[b.target().0].name).to_string())
                    .collect())
                .unwrap_or_default(),
            DeclKind::Edge => ir
                .as_edge(did)
                .map(|eid| ir.edges[eid.0].bases.iter()
                    .map(|b| st.resolve(ir.decl_nodes[b.target().0].name).to_string())
                    .collect())
                .unwrap_or_default(),
            _ => Vec::new(),
        };

        let tags: Vec<String> = decl.anno.tags.iter()
            .map(|&s| st.resolve(s).to_string())
            .collect();

        let mut arcs: Vec<ArcDto> = Vec::new();
        if with_arcs {
            if let Some(eid) = ir.as_edge(did) {
                for &aid in &ir.edges[eid.0].arcs {
                    for r in &ir.arcs[aid.0].refs {
                        let tgt = r.target();
                        if !tgt.is_none() {
                            arcs.push(ArcDto {
                                sign: r.sign(),
                                target_id: tgt.0,
                                target_name: st.resolve(ir.decl_nodes[tgt.0].name).to_string(),
                            });
                        }
                    }
                }
            }
        }
        let value = decl.anno.value.as_ref().map(|v| render_value(v, ir, st));
        NodeDto { id: did.0, name, kind: kind_str.to_string(), bases, tags, value, arcs }
    };

    let mut nodes = Vec::with_capacity(ir.nodes.len());
    let mut edges = Vec::with_capacity(ir.edges.len());
    for rec in &ir.nodes { nodes.push(mk(rec.decl, false)); }
    for rec in &ir.edges { edges.push(mk(rec.decl, true)); }

    // Named relationships keyed by DeclId. Endpoints that aren't drawn vertices
    // are filtered consumer-side.
    //   scope — every Node/Edge decl → its enclosing parent (containment);
    //   ref   — a decl whose field value points at another (`a -> b`);
    //   isa   — node/edge → each first-level base type (inheritance).
    let mut relationships: Vec<RelDto> = Vec::new();
    for (did_idx, decl) in ir.decl_nodes.iter().enumerate() {
        if !matches!(decl.kind, DeclKind::Node | DeclKind::Edge) {
            continue;
        }
        let parent = decl.parent;
        if !parent.is_none() {
            relationships.push(RelDto { kind: "scope".to_string(), from: did_idx, to: parent.0 });
        }
        if let Some(val) = &decl.anno.value {
            let mut targets = Vec::new();
            collect_ref_targets(val, &mut targets);
            for t in targets {
                if !t.is_none() {
                    relationships.push(RelDto { kind: "ref".to_string(), from: did_idx, to: t.0 });
                }
            }
        }
    }
    for rec in &ir.nodes {
        if let Some(nid) = ir.as_node(rec.decl) {
            for b in &ir.nodes[nid.0].bases {
                if !b.target().is_none() {
                    relationships.push(RelDto { kind: "isa".to_string(), from: rec.decl.0, to: b.target().0 });
                }
            }
        }
    }
    for rec in &ir.edges {
        if let Some(eid) = ir.as_edge(rec.decl) {
            for b in &ir.edges[eid.0].bases {
                if !b.target().is_none() {
                    relationships.push(RelDto { kind: "isa".to_string(), from: rec.decl.0, to: b.target().0 });
                }
            }
        }
    }

    SnapshotDto {
        node_count: ir.nodes.len(),
        edge_count: ir.edges.len(),
        arc_count:  ir.arcs.len(),
        nodes,
        edges,
        relationships,
    }
}


/// JSON-encode a snapshot.  Returns `Err` only if serde itself
/// somehow fails — the schema is fixed so this is effectively
/// infallible for well-formed IRs.
pub fn snapshot_json(ir: &Ir, st: &StringTable) -> Result<String, String> {
    serde_json::to_string(&snapshot(ir, st))
        .map_err(|e| format!("json encode: {e}"))
}


// ─── DOT (Graphviz) emitter ─────────────────────────────────────────


/// Escape `\` and `"` for DOT label syntax.
pub fn dot_escape(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}


/// Emit a DOT (Graphviz) string for the signed-incidence hypergraph.
///
/// Vertices render as ellipses, hyperedges as rounded boxes; signed
/// arcs colour-coded (blue +1, red -1, grey ~0) with arrow heads
/// matching the sign (normal, inv, odot).
pub fn emit_dot_graph(ir: &Ir, st: &StringTable, graph_name: &str) -> String {
    let mut out = String::with_capacity(4096);
    out.push_str(&format!("digraph \"{}\" {{\n", dot_escape(graph_name)));
    out.push_str("  rankdir=LR;\n");
    out.push_str("  node [fontname=\"Helvetica\"];\n\n");

    for rec in &ir.nodes {
        let name = st.resolve(ir.decl_nodes[rec.decl.0].name);
        out.push_str(&format!(
            "  \"n{}\" [label=\"{}\", shape=ellipse, style=filled, fillcolor=\"#EEF1F5\"];\n",
            rec.decl.0, dot_escape(name)
        ));
    }
    for rec in &ir.edges {
        let name = st.resolve(ir.decl_nodes[rec.decl.0].name);
        out.push_str(&format!(
            "  \"e{}\" [label=\"{}\", shape=box, style=\"rounded,filled\", fillcolor=\"#D7E4F5\"];\n",
            rec.decl.0, dot_escape(name)
        ));
    }
    out.push('\n');

    for rec in &ir.edges {
        let eid_num = rec.decl.0;
        for &aid in &rec.arcs {
            for r in &ir.arcs[aid.0].refs {
                let tgt = r.target();
                if tgt.is_none() { continue; }
                let target_is_edge = ir
                    .decl_nodes
                    .get(tgt.0)
                    .map(|d| matches!(d.kind, DeclKind::Edge))
                    .unwrap_or(false);
                let tgt_id = if target_is_edge {
                    format!("e{}", tgt.0)
                } else {
                    format!("n{}", tgt.0)
                };
                let (color, arrowhead, penwidth) = match r.sign() {
                     1 => ("#1b6ca8", "normal", 1.4),
                    -1 => ("#b02a2a", "inv",    1.4),
                     _ => ("#888888", "odot",   1.0),
                };
                out.push_str(&format!(
                    "  \"e{}\" -> \"{}\" [color=\"{}\", arrowhead=\"{}\", penwidth={:.1}];\n",
                    eid_num, tgt_id, color, arrowhead, penwidth
                ));
            }
        }
    }
    out.push_str("}\n");
    out
}
