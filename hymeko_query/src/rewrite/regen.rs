//! Regenerate a `.hymeko` source from a [`SplitProposal`] — step 4
//! of the 5-step entropy hot-swap plan.
//!
//! Given a proposal that splits scope `S` into clusters A and B, this
//! module emits a new `.hymeko` document shaped as:
//!
//! ```text
//! <name>_rewritten {}
//!
//! // Bridge vertices (targets of cross-cluster edges) promoted to outer scope.
//! <bridge-1>: <bases> { ... }
//! <bridge-2>: <bases> { ... }
//! ...
//!
//! <name>_cluster_a: <S's bases> {
//!     // cluster A non-bridge vertices + cluster A non-cross edges
//! }
//!
//! <name>_cluster_b: <S's bases> {
//!     // cluster B non-bridge vertices + cluster B non-cross edges
//! }
//!
//! // Cross-cluster edges (reference bridge vertices at outer scope).
//! @<cross-edge-1>: <bases> { ... }
//! ```
//!
//! **Round-trip scope.** For proposals with `n_cross_edges == 0` the
//! emission is syntactically valid HyMeKo and recompiles cleanly.
//! With cross edges, the emission is **informational**: cluster-scope
//! bodies reference bridge vertices at the outer scope, but HyMeKo's
//! resolver walks lexical scope downward, not upward — so inner refs
//! to outer-scope decls may not resolve without port-based wiring
//! (design note `docs/torch_backend_views.md` §"Dataflow view" covers
//! this). The produced document is still useful for review and for
//! downstream consumers (step 5 weight transfer) that read cluster
//! membership from the rendered names.
//!
//! **Imports / using aliases are lost** — the arena IR doesn't retain
//! them. The emitter produces an empty header; importing fixtures
//! need to be re-added by hand before recompile.

use std::collections::HashSet;
use std::fmt::Write;

use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{DeclKind, Ir, SignedRefR, ValueR};
use hymeko::resolution::interner::Interner;

use crate::rewrite::split::{Cluster, SplitProposal};

const INDENT: &str = "    ";

/// Emit a textual `.hymeko` rewrite from a [`SplitProposal`]. The
/// caller picks a base name (e.g. stem of the input file) — the
/// emitted scopes will be `<name>_cluster_a` and `<name>_cluster_b`.
pub fn emit_split_rewrite(
    ir: &Ir,
    it: &Interner,
    proposal: &SplitProposal,
    base_name: &str,
) -> String {
    let mut out = String::new();
    emit_header(ir, it, proposal, base_name, &mut out);

    let (bridges, non_bridge_a, non_bridge_b) = partition_vertices(ir, proposal);
    let (edges_a, edges_b, cross_edges) = partition_edges(proposal);
    let orig_name = decl_name(ir, it, proposal.target_scope);

    emit_description(base_name, &mut out);

    // Bridge vertices at outer scope — they're referenced by cross edges.
    if !bridges.is_empty() {
        let _ = writeln!(&mut out, "// Bridge vertices (targets of cross-cluster edges, promoted to outer scope)");
        for did in &bridges {
            emit_decl(ir, it, *did, 0, &mut out);
        }
        let _ = writeln!(&mut out);
    }

    // Cluster A scope.
    emit_cluster_scope(
        ir, it,
        &format!("{orig_name}_cluster_a"),
        proposal.target_scope,
        &non_bridge_a,
        &edges_a,
        "Cluster A",
        &mut out,
    );
    let _ = writeln!(&mut out);

    // Cluster B scope.
    emit_cluster_scope(
        ir, it,
        &format!("{orig_name}_cluster_b"),
        proposal.target_scope,
        &non_bridge_b,
        &edges_b,
        "Cluster B",
        &mut out,
    );
    let _ = writeln!(&mut out);

    // Cross edges at outer scope.
    if !cross_edges.is_empty() {
        let _ = writeln!(&mut out, "// Cross-cluster edges (reference bridge vertices at outer scope)");
        for did in &cross_edges {
            emit_decl(ir, it, *did, 0, &mut out);
        }
    }

    out
}

// ─── Proposal partitioning ───────────────────────────────────────────

/// Split the proposal's vertices into `(bridges, non_bridge_a, non_bridge_b)`.
/// A *bridge* is a vertex referenced by at least one `Cluster::Cross`
/// edge — these get promoted to outer scope so both clusters can refer
/// to them. Non-bridge vertices stay inside their cluster's sub-scope.
fn partition_vertices(
    ir: &Ir,
    proposal: &SplitProposal,
) -> (Vec<DeclId>, Vec<DeclId>, Vec<DeclId>) {
    let bridge_set = collect_bridge_vertices(ir, proposal);
    let bridges: Vec<DeclId> = proposal.cluster_a.iter()
        .chain(proposal.cluster_b.iter())
        .copied()
        .filter(|d| bridge_set.contains(d))
        .collect();
    let non_bridge_a: Vec<DeclId> = proposal.cluster_a.iter()
        .copied()
        .filter(|d| !bridge_set.contains(d))
        .collect();
    let non_bridge_b: Vec<DeclId> = proposal.cluster_b.iter()
        .copied()
        .filter(|d| !bridge_set.contains(d))
        .collect();
    (bridges, non_bridge_a, non_bridge_b)
}

fn collect_bridge_vertices(ir: &Ir, proposal: &SplitProposal) -> HashSet<DeclId> {
    let mut out = HashSet::new();
    for (e_did, cluster) in &proposal.edge_assignments {
        if *cluster != Cluster::Cross {
            continue;
        }
        let Some(eid) = ir.as_edge(*e_did) else { continue };
        for &aid in &ir.edges[eid.raw()].arcs {
            for r in &ir.arcs[aid.raw()].refs {
                let t = r.target();
                if t.is_some() {
                    out.insert(t);
                }
            }
        }
    }
    out
}

fn partition_edges(proposal: &SplitProposal) -> (Vec<DeclId>, Vec<DeclId>, Vec<DeclId>) {
    let mut a = Vec::new();
    let mut b = Vec::new();
    let mut cross = Vec::new();
    for (e_did, cluster) in &proposal.edge_assignments {
        match cluster {
            Cluster::A     => a.push(*e_did),
            Cluster::B     => b.push(*e_did),
            Cluster::Cross => cross.push(*e_did),
        }
    }
    (a, b, cross)
}

// ─── Emission helpers ────────────────────────────────────────────────

fn emit_header(
    ir: &Ir,
    it: &Interner,
    proposal: &SplitProposal,
    base_name: &str,
    out: &mut String,
) {
    let orig = decl_name(ir, it, proposal.target_scope);
    let _ = writeln!(out, "// Rewritten from `{orig}` — entropy hot-swap step 4");
    let _ = writeln!(out, "// Base name: {base_name}");
    let _ = writeln!(out, "//");
    let _ = writeln!(out,
        "// Cluster A: {} vertices, {} edges",
        proposal.cluster_a.len(),
        proposal.edge_assignments.iter()
            .filter(|(_, c)| *c == Cluster::A).count(),
    );
    let _ = writeln!(out,
        "// Cluster B: {} vertices, {} edges",
        proposal.cluster_b.len(),
        proposal.edge_assignments.iter()
            .filter(|(_, c)| *c == Cluster::B).count(),
    );
    let _ = writeln!(out, "// Cross edges: {}", proposal.n_cross_edges);
    let _ = writeln!(out, "// Inertia: {:.4}", proposal.inertia);
    let _ = writeln!(out, "//");
    let _ = writeln!(out,
        "// NOTE: Imports / `using` aliases are not retained by the arena IR.");
    let _ = writeln!(out,
        "// Re-add them at the top of this file before recompile.");
    let _ = writeln!(out);
}

fn emit_description(base_name: &str, out: &mut String) {
    let _ = writeln!(out, "{base_name}_rewritten {{}}");
    let _ = writeln!(out);
}

fn emit_cluster_scope(
    ir: &Ir,
    it: &Interner,
    cluster_name: &str,
    orig_scope: DeclId,
    vertices: &[DeclId],
    edges: &[DeclId],
    label: &str,
    out: &mut String,
) {
    let orig_bases = scope_bases(ir, it, orig_scope);
    let _ = writeln!(out, "// {label}");
    if vertices.is_empty() && edges.is_empty() {
        let _ = writeln!(out, "{cluster_name}{orig_bases} {{}}");
        return;
    }
    let _ = writeln!(out, "{cluster_name}{orig_bases} {{");
    for did in vertices {
        emit_decl(ir, it, *did, 1, out);
    }
    for did in edges {
        emit_decl(ir, it, *did, 1, out);
    }
    let _ = writeln!(out, "}}");
}

fn scope_bases(ir: &Ir, it: &Interner, did: DeclId) -> String {
    if did.is_none() {
        return String::new();
    }
    let Some(nid) = ir.as_node(did) else { return String::new() };
    let bases = &ir.nodes[nid.0].bases;
    if bases.is_empty() {
        return String::new();
    }
    let names: Vec<&str> = bases.iter().map(|b| decl_name(ir, it, b.target())).collect();
    format!(": {}", names.join(", "))
}

// ─── Minimal decl emitter (specialised for regen) ────────────────────

fn decl_name<'a>(ir: &'a Ir, it: &'a Interner, did: DeclId) -> &'a str {
    if did.is_none() {
        return "<root>";
    }
    it.resolve(ir.decl_nodes[did.raw()].name)
}

fn emit_decl(ir: &Ir, it: &Interner, did: DeclId, depth: usize, out: &mut String) {
    let decl = &ir.decl_nodes[did.raw()];
    let pad = INDENT.repeat(depth);
    let name = it.resolve(decl.name);
    match decl.kind {
        DeclKind::Node => emit_node_decl(ir, it, did, name, &pad, depth, out),
        DeclKind::Edge => emit_edge_decl(ir, it, did, name, &pad, depth, out),
        DeclKind::HyperArc => {} // emitted via owning edge
    }
}

fn emit_node_decl(
    ir: &Ir, it: &Interner, did: DeclId, name: &str,
    pad: &str, depth: usize, out: &mut String,
) {
    let bases = ir.as_node(did)
        .map(|nid| emit_bases(ir, it, &ir.nodes[nid.raw()].bases))
        .unwrap_or_default();
    let tags = emit_tags(ir, it, did);
    let inline_value = ir.decl_nodes[did.raw()].anno.value.as_ref();
    let has_children = ir.decl_nodes[did.raw()].first_child.is_some();

    match (inline_value, has_children) {
        (Some(ValueR::Ref(target)), false) => {
            let _ = writeln!(out, "{pad}{name}{bases}{tags} -> {};", decl_name(ir, it, *target));
        }
        (Some(v), false) => {
            let _ = writeln!(out, "{pad}{name}{bases}{tags} {};", emit_value(ir, it, v));
        }
        (_, true) => {
            let _ = writeln!(out, "{pad}{name}{bases}{tags} {{");
            for child in ir.children(did) {
                emit_decl(ir, it, child, depth + 1, out);
            }
            let _ = writeln!(out, "{pad}}}");
        }
        (None, false) => {
            let _ = writeln!(out, "{pad}{name}{bases}{tags} {{}}");
        }
    }
}

fn emit_edge_decl(
    ir: &Ir, it: &Interner, did: DeclId, name: &str,
    pad: &str, depth: usize, out: &mut String,
) {
    let bases = ir.as_edge(did)
        .map(|eid| emit_bases(ir, it, &ir.edges[eid.raw()].bases))
        .unwrap_or_default();
    let tags = emit_tags(ir, it, did);
    let _ = writeln!(out, "{pad}@{name}{bases}{tags} {{");
    if let Some(eid) = ir.as_edge(did) {
        let arc_pad = INDENT.repeat(depth + 1);
        for &aid in &ir.edges[eid.raw()].arcs {
            let refs: Vec<String> = ir.arcs[aid.raw()].refs.iter()
                .map(|r| emit_signed_ref(ir, it, r))
                .collect();
            let _ = writeln!(out, "{arc_pad}({});", refs.join(", "));
        }
    }
    for child in ir.children(did) {
        if ir.decl_nodes[child.raw()].kind == DeclKind::HyperArc {
            continue;
        }
        emit_decl(ir, it, child, depth + 1, out);
    }
    let _ = writeln!(out, "{pad}}}");
}

fn emit_signed_ref(ir: &Ir, it: &Interner, r: &SignedRefR) -> String {
    let sign = match r {
        SignedRefR::Plus(_)    => '+',
        SignedRefR::Minus(_)   => '-',
        SignedRefR::Neutral(_) => '~',
    };
    let atom = r.atom();
    let mut s = format!("{sign} {}", decl_name(ir, it, atom.target));
    if let Some(ws) = &atom.weights {
        let items: Vec<String> = ws.iter().map(|v| emit_value(ir, it, v)).collect();
        if !items.is_empty() {
            s.push_str(&format!(" [{}]", items.join(", ")));
        }
    }
    s
}

fn emit_bases(ir: &Ir, it: &Interner, bases: &[SignedRefR]) -> String {
    if bases.is_empty() { return String::new(); }
    let names: Vec<&str> = bases.iter().map(|b| decl_name(ir, it, b.target())).collect();
    format!(": {}", names.join(", "))
}

fn emit_tags(ir: &Ir, it: &Interner, did: DeclId) -> String {
    let tags = &ir.decl_nodes[did.raw()].anno.tags;
    if tags.is_empty() { return String::new(); }
    let rendered: Vec<String> = tags.iter().map(|&s| format!("<{}>", it.resolve(s))).collect();
    format!(" {}", rendered.join(" "))
}

fn emit_value(ir: &Ir, it: &Interner, v: &ValueR) -> String {
    match v {
        ValueR::Num(n) => format_num(*n),
        ValueR::Str(sid) => format!("\"{}\"", it.resolve(*sid)),
        ValueR::List(xs) => {
            let items: Vec<String> = xs.iter().map(|x| emit_value(ir, it, x)).collect();
            format!("[{}]", items.join(", "))
        }
        ValueR::Ref(did) => decl_name(ir, it, *did).to_string(),
    }
}

fn format_num(n: f64) -> String {
    if n.fract() == 0.0 && n.abs() < 1e16 {
        format!("{:.1}", n)
    } else {
        format!("{}", n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rewrite::split::propose_split;
    use hymeko::common::ids::{EdgeId, HyperArcId, NodeId, SymId};
    use hymeko::ir::ir::{AnnoR, ArcRec, DeclNode, EdgeRec, NodeRec, RefAtomR};

    fn push_decl(ir: &mut Ir, parent: DeclId, kind: DeclKind, name: SymId) -> DeclId {
        let did = DeclId::new(ir.decl_nodes.len());
        ir.decl_nodes.push(DeclNode {
            kind, name, parent,
            first_child: DeclId::NONE,
            last_child: DeclId::NONE,
            next_sibling: DeclId::NONE,
            anno: AnnoR::default(),
        });
        ir.decl_to_node.push(None);
        ir.decl_to_edge.push(None);
        ir.decl_to_arc.push(None);
        ir.decl_hash.push(None);
        if parent.is_some() {
            let first = ir.decl_nodes[parent.0].first_child;
            if first.is_none() {
                ir.decl_nodes[parent.0].first_child = did;
                ir.decl_nodes[parent.0].last_child = did;
            } else {
                let last = ir.decl_nodes[parent.0].last_child;
                ir.decl_nodes[last.0].next_sibling = did;
                ir.decl_nodes[parent.0].last_child = did;
            }
        }
        did
    }

    fn push_node_named(ir: &mut Ir, parent: DeclId, name: SymId) -> DeclId {
        let did = push_decl(ir, parent, DeclKind::Node, name);
        let nid = NodeId::new(ir.nodes.len());
        ir.nodes.push(NodeRec::new(did, Vec::new()));
        ir.decl_to_node[did.0] = Some(nid);
        did
    }

    fn push_edge_named(
        ir: &mut Ir, parent: DeclId, name: SymId,
        incidences: &[(i8, DeclId)],
    ) -> DeclId {
        let e_did = push_decl(ir, parent, DeclKind::Edge, name);
        let eid = EdgeId::new(ir.edges.len());
        ir.edges.push(EdgeRec::new(e_did, Vec::new()));
        ir.decl_to_edge[e_did.0] = Some(eid);

        let arc_did = push_decl(ir, e_did, DeclKind::HyperArc, SymId::new(0));
        let aid = HyperArcId::new(ir.arcs.len());
        let refs = incidences.iter()
            .map(|&(sign, target)| {
                let atom = RefAtomR { target, anno: AnnoR::default(), weights: None };
                match sign {
                    1  => SignedRefR::Plus(atom),
                    -1 => SignedRefR::Minus(atom),
                    _  => SignedRefR::Neutral(atom),
                }
            }).collect();
        ir.arcs.push(ArcRec { anno: AnnoR::default(), in_edge: e_did, refs });
        ir.decl_to_arc[arc_did.0] = Some(aid);
        ir.edges[eid.0].arcs.push(aid);
        e_did
    }

    /// Minimal Interner stand-in for tests: all symbols have id 0 and
    /// resolve to empty. The emitted output won't have meaningful
    /// names, but we can still check structural presence (scope braces,
    /// cluster headers, etc.) — the real integration test on
    /// simple_net.hymeko exercises the full interner.
    fn empty_ir() -> Ir {
        Ir::default()
    }

    #[test]
    fn clean_split_produces_two_nonempty_scopes_and_no_cross_section() {
        let mut ir = empty_ir();
        let root = push_node_named(&mut ir, DeclId::NONE, SymId::new(0));
        let v0 = push_node_named(&mut ir, root, SymId::new(0));
        let v1 = push_node_named(&mut ir, root, SymId::new(0));
        let v2 = push_node_named(&mut ir, root, SymId::new(0));
        let v3 = push_node_named(&mut ir, root, SymId::new(0));
        push_edge_named(&mut ir, root, SymId::new(0), &[(1, v0), (-1, v1)]);
        push_edge_named(&mut ir, root, SymId::new(0), &[(1, v2), (-1, v3)]);

        let proposal = propose_split(&ir, root).expect("clean split available");
        assert_eq!(proposal.n_cross_edges, 0);

        // Can't build a real Interner in test, so exercise just the
        // proposal partitioning helpers — they're where the regen
        // logic lives.
        let (bridges, nb_a, nb_b) = partition_vertices(&ir, &proposal);
        assert!(bridges.is_empty(), "clean split has no bridges");
        assert_eq!(nb_a.len() + nb_b.len(),
                   proposal.cluster_a.len() + proposal.cluster_b.len(),
                   "without bridges, all vertices stay in their cluster");

        let (edges_a, edges_b, cross) = partition_edges(&proposal);
        assert!(cross.is_empty());
        assert_eq!(edges_a.len() + edges_b.len(), 2);
    }

    #[test]
    fn cross_edges_land_in_cross_bucket() {
        let mut ir = empty_ir();
        let root = push_node_named(&mut ir, DeclId::NONE, SymId::new(0));
        let v0 = push_node_named(&mut ir, root, SymId::new(0));
        let v1 = push_node_named(&mut ir, root, SymId::new(0));
        let shared = push_node_named(&mut ir, root, SymId::new(0));
        push_edge_named(&mut ir, root, SymId::new(0), &[(1, v0), (-1, shared)]);
        push_edge_named(&mut ir, root, SymId::new(0), &[(1, v1), (-1, shared)]);

        let proposal = propose_split(&ir, root).expect("should split");
        assert!(proposal.n_cross_edges >= 1);
        let (_, _, cross) = partition_edges(&proposal);
        assert_eq!(cross.len(), proposal.n_cross_edges,
                   "partition_edges' cross count matches proposal.n_cross_edges");
    }
}
