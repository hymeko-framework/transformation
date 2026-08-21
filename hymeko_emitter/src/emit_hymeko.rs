//! Arena `Ir` → deterministic `.hymeko` source text.
//!
//! Step 2b scope: recursively walks the decl tree (`DeclNode::first_child` /
//! `next_sibling`) from the synthetic root `DeclId::NONE` downward, so
//! nested children, inline values, weight annotations on signed refs, tag
//! annotations, and named hyperarc tuples all survive the round trip.
//! Imports / module headers are **not** re-emitted — the arena IR does
//! not retain that AST context, so `hymeko_emitter` emits the lowered
//! surface only.
//!
//! Round-trip guarantee: `parse_description(emit_hymeko(ir, it, name))`
//! parses and yields an AST with the same (node, edge, arc) counts.
//! Byte-for-byte fixity is *not* a goal — the emitter is canonicalising
//! (normalised whitespace, sorted-by-insertion-order), not faithful to
//! the original author's formatting.

use std::fmt::Write;

use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{DeclKind, Ir, SignedRefR, ValueR};
use hymeko::resolution::interner::Interner;

const INDENT: &str = "    ";

/// Resolve a `DeclId` to its textual name via the interner.
fn decl_name<'a>(ir: &'a Ir, it: &'a Interner, did: DeclId) -> &'a str {
    it.resolve(ir.decl_nodes[did.0].name)
}

/// Render a `ValueR` as literal text. Strings are double-quoted;
/// numeric values trim trailing zeros.
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
        // Use {} to get the shortest round-trippable form.
        format!("{}", n)
    }
}

/// Render a signed reference with optional per-arc weight annotation.
///
/// Output shapes:
/// - `+ target`                   (sign + target)
/// - `+ target [[xyz],[rpy]]`     (with weight list)
fn emit_signed_ref(ir: &Ir, it: &Interner, r: &SignedRefR) -> String {
    let sign = match r {
        SignedRefR::Plus(_) => '+',
        SignedRefR::Minus(_) => '-',
        SignedRefR::Neutral(_) => '~',
    };
    let atom = r.atom();
    let name = decl_name(ir, it, atom.target);
    let mut s = format!("{sign} {name}");
    if let Some(ws) = &atom.weights {
        let items: Vec<String> = ws.iter().map(|v| emit_value(ir, it, v)).collect();
        if !items.is_empty() {
            s.push_str(&format!(" [{}]", items.join(", ")));
        }
    }
    s
}

fn emit_tags(ir: &Ir, it: &Interner, did: DeclId) -> String {
    let tags = &ir.decl_nodes[did.0].anno.tags;
    if tags.is_empty() {
        return String::new();
    }
    let rendered: Vec<String> = tags.iter().map(|&s| format!("<{}>", it.resolve(s))).collect();
    format!(" {}", rendered.join(" "))
}

fn emit_bases(ir: &Ir, it: &Interner, bases: &[SignedRefR]) -> String {
    if bases.is_empty() {
        return String::new();
    }
    let names: Vec<&str> = bases.iter().map(|b| decl_name(ir, it, b.target())).collect();
    format!(": {}", names.join(", "))
}

fn has_child_decls(ir: &Ir, did: DeclId) -> bool {
    ir.decl_nodes[did.0].first_child.is_some()
}

fn emit_decl(ir: &Ir, it: &Interner, did: DeclId, depth: usize, out: &mut String) {
    let decl = &ir.decl_nodes[did.0];
    let name = it.resolve(decl.name);
    let pad = INDENT.repeat(depth);

    match decl.kind {
        DeclKind::Node => emit_node(ir, it, did, name, &pad, depth, out),
        DeclKind::Edge => emit_edge(ir, it, did, name, &pad, depth, out),
        DeclKind::HyperArc => {
            // Anonymous arcs (`@{ ... }` without a name) live under an edge
            // and are dispatched from `emit_edge`. Standalone arcs at the
            // decl-tree top level should not occur after lowering — if
            // they do, emit them as a fallback tuple so nothing is lost.
            if let Some(aid) = ir.as_arc(did) {
                let refs: Vec<String> = ir.arcs[aid.0]
                    .refs
                    .iter()
                    .map(|r| emit_signed_ref(ir, it, r))
                    .collect();
                let _ = writeln!(out, "{pad}({});", refs.join(", "));
            }
        }
    }
}

fn emit_node(
    ir: &Ir,
    it: &Interner,
    did: DeclId,
    name: &str,
    pad: &str,
    depth: usize,
    out: &mut String,
) {
    let bases = ir
        .as_node(did)
        .map(|nid| emit_bases(ir, it, &ir.nodes[nid.0].bases))
        .unwrap_or_default();
    let tags = emit_tags(ir, it, did);
    let inline_value = ir.decl_nodes[did.0].anno.value.as_ref();
    let has_children = has_child_decls(ir, did);

    // Shape the declaration:
    //   `name{bases}{tags} -> target;`             (Ref value — attachment arc)
    //   `name{bases}{tags} value;`                 (leaf with other value)
    //   `name{bases}{tags} { <children> }`         (block form)
    //   `name{bases}{tags} {}`                     (empty block)
    match (inline_value, has_children) {
        (Some(ValueR::Ref(target)), false) => {
            // Source-syntax attachment-arc form: `visual -> link_geometry;`
            let target_name = decl_name(ir, it, *target);
            let _ = writeln!(out, "{pad}{name}{bases}{tags} -> {target_name};");
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

fn emit_edge(
    ir: &Ir,
    it: &Interner,
    did: DeclId,
    name: &str,
    pad: &str,
    depth: usize,
    out: &mut String,
) {
    let bases = ir
        .as_edge(did)
        .map(|eid| emit_bases(ir, it, &ir.edges[eid.0].bases))
        .unwrap_or_default();
    let tags = emit_tags(ir, it, did);

    let _ = writeln!(out, "{pad}@{name}{bases}{tags} {{");

    // Emit the edge's arcs as `(+ a, - b, ~ c);` tuples.
    if let Some(eid) = ir.as_edge(did) {
        let arc_pad = INDENT.repeat(depth + 1);
        for &aid in &ir.edges[eid.0].arcs {
            let refs: Vec<String> = ir.arcs[aid.0]
                .refs
                .iter()
                .map(|r| emit_signed_ref(ir, it, r))
                .collect();
            let _ = writeln!(out, "{arc_pad}({});", refs.join(", "));
        }
    }

    // Also emit any child decls nested under this edge (e.g. `@control: …`,
    // `limit -> joint0_limit;`, controller parameters).
    for child in ir.children(did) {
        if ir.decl_nodes[child.0].kind == DeclKind::HyperArc {
            // Skip — already emitted via the arcs table above.
            continue;
        }
        emit_decl(ir, it, child, depth + 1, out);
    }

    let _ = writeln!(out, "{pad}}}");
}

/// Emit a full `.hymeko` document from an arena IR.
///
/// Shape follows the parser's `Description` production:
///
/// ```text
/// <description_name> { /* optional header block — imports, usings */ }
///
/// <top-level-HyperItem>
/// <top-level-HyperItem>
/// ...
/// ```
///
/// The wrapper `{ … }` is the **header block** — it only accepts simple
/// `ident ;` statements, imports, and usings, not full nodes. Top-level
/// robots (like `mini_arm: ... { ... }`) must live *outside* the wrapper
/// as sibling `HyperItem`s, which is how the fixtures under
/// `data/robotics/` are structured. Previously the emitter nested
/// everything inside the wrapper and the output failed round-trip;
/// fixed 2026-04-18 in Plan 06 Step 2b.
pub fn emit_hymeko(ir: &Ir, interner: &Interner, description_name: &str) -> String {
    let mut out = String::new();
    let _ = writeln!(
        &mut out,
        "// Generated by hymeko_emitter::emit_hymeko (Plan 06, Step 2b)"
    );
    let _ = writeln!(
        &mut out,
        "// Imports and comments from the source are not retained in the"
    );
    let _ = writeln!(
        &mut out,
        "// arena IR, so this emitter only reproduces the lowered surface."
    );
    let _ = writeln!(&mut out);

    // Empty header block — imports and `using ... as` aliases are not
    // preserved by the arena IR, so we emit a bare header.
    let _ = writeln!(&mut out, "{description_name} {{}}");
    let _ = writeln!(&mut out);

    // Top-level decls are those with parent == DeclId::NONE. Anonymous
    // arcs also have parent NONE but are emitted from inside their owning
    // edge — skip them at the root so they aren't double-emitted.
    for (i, decl) in ir.decl_nodes.iter().enumerate() {
        if decl.parent.is_some() {
            continue;
        }
        if decl.kind == DeclKind::HyperArc {
            continue;
        }
        emit_decl(ir, interner, DeclId::new(i), 0, &mut out);
    }

    out
}
