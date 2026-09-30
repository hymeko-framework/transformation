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
use hymeko::ir::ir::{DeclKind, Ir, PortDirection, ValueR};
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
                st.resolve(ir.decl_nodes[d.raw()].name).to_string()
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
        let decl = &ir.decl_nodes[did.raw()];
        let name = st.resolve(decl.name).to_string();
        let kind_str = match decl.kind {
            DeclKind::Node => "Node",
            DeclKind::Edge => "Edge",
            DeclKind::HyperArc => "HyperArc",
        };

        let bases: Vec<String> = match decl.kind {
            DeclKind::Node => ir
                .as_node(did)
                .map(|nid| {
                    ir.nodes[nid.raw()]
                        .bases
                        .iter()
                        .map(|b| st.resolve(ir.decl_nodes[b.target().raw()].name).to_string())
                        .collect()
                })
                .unwrap_or_default(),
            DeclKind::Edge => ir
                .as_edge(did)
                .map(|eid| {
                    ir.edges[eid.raw()]
                        .bases
                        .iter()
                        .map(|b| st.resolve(ir.decl_nodes[b.target().raw()].name).to_string())
                        .collect()
                })
                .unwrap_or_default(),
            _ => Vec::new(),
        };

        let tags: Vec<String> = decl
            .anno
            .tags
            .iter()
            .map(|&s| st.resolve(s).to_string())
            .collect();

        let mut arcs: Vec<ArcDto> = Vec::new();
        if with_arcs {
            if let Some(eid) = ir.as_edge(did) {
                for &aid in &ir.edges[eid.raw()].arcs {
                    for r in &ir.arcs[aid.raw()].refs {
                        let tgt = r.target();
                        if !tgt.is_none() {
                            arcs.push(ArcDto {
                                sign: r.sign(),
                                target_id: tgt.raw(),
                                target_name: st.resolve(ir.decl_nodes[tgt.raw()].name).to_string(),
                            });
                        }
                    }
                }
            }
        }
        let value = decl.anno.value.as_ref().map(|v| render_value(v, ir, st));
        NodeDto {
            id: did.raw(),
            name,
            kind: kind_str.to_string(),
            bases,
            tags,
            value,
            arcs,
        }
    };

    let mut nodes = Vec::with_capacity(ir.nodes.len());
    let mut edges = Vec::with_capacity(ir.edges.len());
    for rec in &ir.nodes {
        nodes.push(mk(rec.decl, false));
    }
    for rec in &ir.edges {
        edges.push(mk(rec.decl, true));
    }

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
            relationships.push(RelDto {
                kind: "scope".to_string(),
                from: did_idx,
                to: parent.raw(),
            });
        }
        if let Some(val) = &decl.anno.value {
            let mut targets = Vec::new();
            collect_ref_targets(val, &mut targets);
            for t in targets {
                if !t.is_none() {
                    relationships.push(RelDto {
                        kind: "ref".to_string(),
                        from: did_idx,
                        to: t.raw(),
                    });
                }
            }
        }
    }
    for rec in &ir.nodes {
        if let Some(nid) = ir.as_node(rec.decl) {
            for b in &ir.nodes[nid.raw()].bases {
                if !b.target().is_none() {
                    relationships.push(RelDto {
                        kind: "isa".to_string(),
                        from: rec.decl.raw(),
                        to: b.target().raw(),
                    });
                }
            }
        }
    }
    for rec in &ir.edges {
        if let Some(eid) = ir.as_edge(rec.decl) {
            for b in &ir.edges[eid.raw()].bases {
                if !b.target().is_none() {
                    relationships.push(RelDto {
                        kind: "isa".to_string(),
                        from: rec.decl.raw(),
                        to: b.target().raw(),
                    });
                }
            }
        }
    }

    SnapshotDto {
        node_count: ir.nodes.len(),
        edge_count: ir.edges.len(),
        arc_count: ir.arcs.len(),
        nodes,
        edges,
        relationships,
    }
}

/// JSON-encode a snapshot.  Returns `Err` only if serde itself
/// somehow fails — the schema is fixed so this is effectively
/// infallible for well-formed IRs.
pub fn snapshot_json(ir: &Ir, st: &StringTable) -> Result<String, String> {
    serde_json::to_string(&snapshot(ir, st)).map_err(|e| format!("json encode: {e}"))
}

// ─── DOT (Graphviz) emitter ─────────────────────────────────────────

/// Escape text for a quoted DOT string, including literal control characters.
pub fn dot_escape(s: &str) -> String {
    s.replace('\\', "\\\\")
        .replace('"', "\\\"")
        .replace('\r', "\\r")
        .replace('\n', "\\n")
        .replace('\t', "    ")
}

/// Emit a DOT (Graphviz) string for the signed-incidence hypergraph.
///
/// Declarations retain their `DeclId`, arc junctions their `HyperArcId`, and
/// incidence lines their arc/slot identity in DOT and SVG `id` attributes.
/// Separate junctions preserve multiple arcs and repeated incidences. Colour
/// and labels carry sign; arrows carry only explicit `PortDirection`.
///
/// Values, containment, inheritance and field references reuse the snapshot
/// projection. This is a structural view, not a kinematic evaluation or a
/// lossless serialization. Layout coordinates have no physical meaning.
pub fn emit_dot_graph(ir: &Ir, st: &StringTable, graph_name: &str) -> String {
    use std::fmt::Write;

    let snap = snapshot(ir, st);
    let id_of = |did: DeclId| match ir.decl_nodes[did.raw()].kind {
        DeclKind::Node => format!("n{}", did.raw()),
        DeclKind::Edge => format!("e{}", did.raw()),
        DeclKind::HyperArc => format!("a{}", ir.as_arc(did).expect("validated arc").raw()),
    };
    let mut out = String::with_capacity(4096);
    let _ = writeln!(
        out,
        "// Generated from canonical HyMeKo IR; edit the .hymeko source."
    );
    let _ = writeln!(out, "digraph \"{}\" {{", dot_escape(graph_name));
    out.push_str("  graph [rankdir=LR, bgcolor=\"#FAFBFE\", pad=0.4, nodesep=0.28, ranksep=0.7, fontname=\"Helvetica\", fontsize=18, labelloc=t];\n");
    let _ = writeln!(out, "  label=\"{}\";", dot_escape(graph_name));
    out.push_str("  node [fontname=\"Helvetica\", fontsize=11, color=\"#64748B\", fontcolor=\"#172B4D\", margin=\"0.14,0.08\"];\n");
    out.push_str("  edge [fontname=\"Helvetica\", fontsize=9, arrowsize=0.7];\n");

    for node in snap.nodes.iter().chain(&snap.edges) {
        let label = match &node.value {
            Some(value) => format!("{}\n= {}", node.name, value),
            None => node.name.clone(),
        };
        let (shape, fill) = if node.kind == "Edge" {
            ("box", "#DDEAFE")
        } else if node.value.is_some() {
            ("note", "#F1F5F9")
        } else {
            ("ellipse", "#E3F3EE")
        };
        let _ = writeln!(
            out,
            "  \"{}\" [id=\"decl-{}\", label=\"{}\", tooltip=\"DeclId {}\", shape={shape}, style=\"rounded,filled\", fillcolor=\"{fill}\"];",
            id_of(DeclId::new(node.id)),
            node.id,
            dot_escape(&label),
            node.id
        );
    }

    for rec in &ir.edges {
        for (ordinal, &aid) in rec.arcs.iter().enumerate() {
            let aid_num = aid.raw();
            let owner = rec.decl.raw();
            let _ = writeln!(
                out,
                "  \"a{aid_num}\" [id=\"arc-{aid_num}\", label=\"arc {ordinal}\", tooltip=\"HyperArcId {aid_num}; owner DeclId {owner}\", shape=diamond, style=filled, fillcolor=\"#FFF1D6\"];"
            );
            let _ = writeln!(
                out,
                "  \"e{owner}\" -> \"a{aid_num}\" [id=\"owner-{aid_num}\", dir=none, color=\"#64748B\", label=\"arc\"];"
            );
            for (slot, r) in ir.arcs[aid_num].refs.iter().enumerate() {
                if r.target().is_none() {
                    continue;
                }
                let (sign, color) = match r.sign() {
                    1 => ("+", "#176B9B"),
                    -1 => ("-", "#B54747"),
                    _ => ("~", "#737D8C"),
                };
                let dir = match r.atom().direction {
                    PortDirection::In => "back",
                    PortDirection::Out => "forward",
                    PortDirection::Unspecified => "none",
                };
                let mut label = format!("{sign} [{slot}]");
                if let Some(weights) = &r.atom().weights {
                    let values: Vec<_> = weights.iter().map(|v| render_value(v, ir, st)).collect();
                    let _ = write!(label, " w=[{}]", values.join(", "));
                }
                let _ = writeln!(
                    out,
                    "  \"a{aid_num}\" -> \"{}\" [id=\"incidence-{aid_num}-{slot}\", label=\"{}\", tooltip=\"arc {aid_num}, slot {slot}, direction {:?}\", color=\"{color}\", fontcolor=\"{color}\", dir={dir}, arrowhead=normal, arrowtail=normal, penwidth=1.4];",
                    id_of(r.target()),
                    dot_escape(&label),
                    r.atom().direction
                );
            }
        }
    }

    for (index, rel) in snap.relationships.iter().enumerate() {
        let (from, to, style, color, dir) = match rel.kind.as_str() {
            "scope" => (rel.to, rel.from, "dotted", "#A1ACBB", "none"),
            "isa" => (rel.from, rel.to, "dashed", "#8161A8", "forward"),
            "ref" => (rel.from, rel.to, "solid", "#2E857A", "forward"),
            _ => continue,
        };
        // Scope defines the hierarchy for layout; references do not distort it.
        let constraint = rel.kind == "scope";
        let _ = writeln!(
            out,
            "  \"{}\" -> \"{}\" [id=\"relation-{index}\", label=\"{}\", style={style}, color=\"{color}\", fontcolor=\"{color}\", dir={dir}, constraint={constraint}];",
            id_of(DeclId::new(from)),
            id_of(DeclId::new(to)),
            rel.kind
        );
    }
    out.push_str("}\n");
    out
}
