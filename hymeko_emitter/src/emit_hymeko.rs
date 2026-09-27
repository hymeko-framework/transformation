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
use hymeko::ir::canonical_hash::DerivedPathIndex;
use hymeko::ir::ir::{DeclKind, Ir, SignedRefR, ValueR};
use hymeko::resolution::interner::Interner;

const INDENT: &str = "    ";

struct SourceEmitter<'a> {
    ir: &'a Ir,
    it: &'a Interner,
    paths: DerivedPathIndex,
}
impl SourceEmitter<'_> {
    /// The Core owns canonical paths; emit them instead of ambiguous leaf names.
    fn decl_name(&self, did: DeclId) -> String {
        match self.paths.path_of(did) {
            Some(path) => path
                .iter()
                .map(|part| self.it.resolve(part))
                .collect::<Vec<_>>()
                .join("."),
            // Anonymous arc targets have no source name. This emitter's supported
            // surface remains named declarations, as before this correction.
            None => self
                .it
                .resolve(self.ir.decl_nodes[did.raw()].name)
                .to_owned(),
        }
    }
    /// Render a `ValueR` as literal text. Strings are double-quoted;
    /// numeric values trim trailing zeros.
    fn emit_value(&self, v: &ValueR) -> String {
        let it = self.it;
        match v {
            ValueR::Num(n) => format_num(*n),
            ValueR::Str(sid) => format!("\"{}\"", it.resolve(*sid)),
            ValueR::List(xs) => {
                let items: Vec<String> = xs.iter().map(|x| self.emit_value(x)).collect();
                format!("[{}]", items.join(", "))
            }
            ValueR::Ref(did) => self.decl_name(*did),
        }
    }

    /// Render a signed reference with optional per-arc weight annotation.
    ///
    /// Output shapes:
    /// - `+ target`                   (sign + target)
    /// - `+ target [[xyz],[rpy]]`     (with weight list)
    fn emit_signed_ref(&self, r: &SignedRefR) -> String {
        let sign = match r {
            SignedRefR::Plus(_) => '+',
            SignedRefR::Minus(_) => '-',
            SignedRefR::Neutral(_) => '~',
        };
        let atom = r.atom();
        let name = self.decl_name(atom.target);
        let mut s = format!("{sign} {name}");
        if let Some(ws) = &atom.weights {
            let items: Vec<String> = ws.iter().map(|v| self.emit_value(v)).collect();
            if !items.is_empty() {
                s.push_str(&format!(" [{}]", items.join(", ")));
            }
        }
        s
    }

    fn emit_tags(&self, did: DeclId) -> String {
        let (ir, it) = (self.ir, self.it);
        let tags = &ir.decl_nodes[did.raw()].anno.tags;
        if tags.is_empty() {
            return String::new();
        }
        let rendered: Vec<String> = tags
            .iter()
            .map(|&s| format!("<{}>", it.resolve(s)))
            .collect();
        format!(" {}", rendered.join(" "))
    }

    fn emit_arc(&self, refs: &[SignedRefR], pad: &str, out: &mut String) {
        let refs: Vec<_> = refs.iter().map(|r| self.emit_signed_ref(r)).collect();
        let _ = writeln!(out, "{pad}({});", refs.join(", "));
    }

    fn emit_bases(&self, did: DeclId) -> String {
        let bases = self
            .ir
            .node_for_decl(did)
            .map(|(_, node)| node.bases.as_slice())
            .or_else(|| {
                self.ir
                    .edge_for_decl(did)
                    .map(|(_, edge)| edge.bases.as_slice())
            })
            .unwrap_or_default();
        if bases.is_empty() {
            return String::new();
        }
        let names: Vec<String> = bases.iter().map(|b| self.decl_name(b.target())).collect();
        format!(": {}", names.join(", "))
    }

    fn has_child_decls(&self, did: DeclId) -> bool {
        let ir = self.ir;
        ir.decl_nodes[did.raw()].first_child.is_some()
    }

    fn emit_decl(&self, did: DeclId, depth: usize, out: &mut String) {
        let (ir, it) = (self.ir, self.it);
        let decl = &ir.decl_nodes[did.raw()];
        let header = format!(
            "{}{}{}",
            it.resolve(decl.name),
            self.emit_bases(did),
            self.emit_tags(did)
        );
        let pad = INDENT.repeat(depth);

        match decl.kind {
            DeclKind::Node => self.emit_node(did, &header, &pad, depth, out),
            DeclKind::Edge => self.emit_edge(did, &header, &pad, depth, out),
            DeclKind::HyperArc => {
                // Anonymous arcs (`@{ ... }` without a name) live under an edge
                // and are dispatched from `emit_edge`. Standalone arcs at the
                // decl-tree top level should not occur after lowering — if
                // they do, emit them as a fallback tuple so nothing is lost.
                if let Some(aid) = ir.as_arc(did) {
                    self.emit_arc(&ir.arcs[aid.raw()].refs, &pad, out);
                }
            }
        }
    }

    fn emit_node(&self, did: DeclId, header: &str, pad: &str, depth: usize, out: &mut String) {
        let ir = self.ir;
        let inline_value = ir.decl_nodes[did.raw()].anno.value.as_ref();
        let has_children = self.has_child_decls(did);

        // Shape the declaration:
        //   `name{bases}{tags} -> target;`             (Ref value — attachment arc)
        //   `name{bases}{tags} value;`                 (leaf with other value)
        //   `name{bases}{tags} { <children> }`         (block form)
        //   `name{bases}{tags} {}`                     (empty block)
        match (inline_value, has_children) {
            (Some(ValueR::Ref(target)), false) => {
                // Source-syntax attachment-arc form: `visual -> link_geometry;`
                let target_name = self.decl_name(*target);
                let _ = writeln!(out, "{pad}{header} -> {target_name};");
            }
            (Some(v), false) => {
                let _ = writeln!(out, "{pad}{header} {};", self.emit_value(v));
            }
            (_, true) => {
                let _ = writeln!(out, "{pad}{header} {{");
                for child in ir.children(did) {
                    self.emit_decl(child, depth + 1, out);
                }
                let _ = writeln!(out, "{pad}}}");
            }
            (None, false) => {
                let _ = writeln!(out, "{pad}{header} {{}}");
            }
        }
    }

    fn emit_edge(&self, did: DeclId, header: &str, pad: &str, depth: usize, out: &mut String) {
        let ir = self.ir;

        let _ = writeln!(out, "{pad}@{header} {{");

        // Emit the edge's arcs as `(+ a, - b, ~ c);` tuples.
        if let Some(eid) = ir.as_edge(did) {
            let arc_pad = INDENT.repeat(depth + 1);
            for &aid in &ir.edges[eid.raw()].arcs {
                self.emit_arc(&ir.arcs[aid.raw()].refs, &arc_pad, out);
            }
        }

        // Also emit any child decls nested under this edge (e.g. `@control: …`,
        // `limit -> joint0_limit;`, controller parameters).
        for child in ir.children(did) {
            if ir.decl_nodes[child.raw()].kind == DeclKind::HyperArc {
                // Skip — already emitted via the arcs table above.
                continue;
            }
            self.emit_decl(child, depth + 1, out);
        }

        let _ = writeln!(out, "{pad}}}");
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
    // A canonical compiled model has a coherent hierarchy. Preserve the public
    // string API's existing precondition (valid IR plus its matching interner).
    let paths =
        DerivedPathIndex::new(ir).expect("source emission requires a coherent canonical IR");
    let emitter = SourceEmitter {
        ir,
        it: interner,
        paths,
    };
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
        emitter.emit_decl(DeclId::new(i), 0, &mut out);
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use hymeko::{
        ir::semantic_digest::semantic_model_digest,
        module_store::{execution::CompiledModel, source_provider::MemProvider},
        util::real_parser::RealParser,
    };
    use std::path::Path;

    #[test]
    fn qualified_references_survive_nested_and_ambiguous_leaf_names() {
        let compile = |source: &str| {
            CompiledModel::compile(
                MemProvider::default().with_file("model.hymeko", source),
                RealParser,
                Path::new("model.hymeko"),
            )
            .unwrap()
        };
        let original = compile(
            r#"description {} scene {
            left { target {} } right { target {} }
            attachment -> left.target;
            references [left.target, right.target];
            derived: left.target {}
            @observation { (+left.target, -right.target); }
        }"#,
        );
        let source = emit_hymeko(&original.program().ir, original.interner(), "exported");
        assert!(source.contains("scene.left.target"));
        assert!(source.contains("scene.right.target"));
        let restored = compile(&source);
        assert_eq!(
            semantic_model_digest(&original.program().ir, original.interner()),
            semantic_model_digest(&restored.program().ir, restored.interner())
        );
    }
}
