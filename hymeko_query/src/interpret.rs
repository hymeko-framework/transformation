//! Interprets a parsed .hymeko AST fragment as query predicates.
//!
//! This is the core of the "query-as-description" design: the SAME
//! grammar that describes hypergraphs describes PATTERNS over them.
//! A partial `.hymeko` fragment, parsed by the existing LALRPOP parser,
//! becomes a predicate tree when run through this interpreter.
//!
//! Conventions:
//!   - `_` as a name → wildcard (match any)
//!   - Named element  → exact name match
//!   - `: base`       → inheritance match
//!   - `<tag>`        → annotation tag match
//!   - `<gt>`, `<lt>` → comparison operator (tag-encoded)
//!   - `{ children }` → containment match
//!   - `@edge { +x -y }` → arc ref match
//!   - Value after name → value match

use hymeko::ir::ir::DeclKind;
use parser::ast::*;
use crate::{NamedQuery, Predicate, ValuePredicate};

/// Convert a parsed .hymeko description into a list of named queries.
///
/// Each top-level item in the description becomes one `NamedQuery`.
/// The description's own name becomes the query set label.
pub fn interpret_as_queries(ast: &Description<'_, &str>) -> Vec<NamedQuery> {
    interpret_items_as_queries(&ast.items)
}

/// Convert a parsed .hymeko description (transform definition) into queries.
///
/// Transform query files follow the convention:
/// ```text
/// <transform_name> {}
/// context {
///     label1: base1 {}
///     label2: base2 {}
/// }
/// ```
///
/// This helper finds a top-level `context` block and interprets its children
/// as the queries, ignoring the transform-header node. If no `context` block
/// is found, falls back to the plain [`interpret_as_queries`] behaviour.
pub fn interpret_transform_queries(ast: &Description<'_, &str>) -> Vec<NamedQuery> {
    for item in &ast.items {
        if let HyperItem::Node(n) = item {
            if n.inner.name == "context" {
                if let Some(body) = &n.inner.body {
                    return interpret_items_as_queries_opts(body, false);
                }
            }
        }
    }
    interpret_as_queries(ast)
}

fn interpret_items_as_queries(items: &[HyperItem<'_, &str>]) -> Vec<NamedQuery> {
    interpret_items_as_queries_opts(items, true)
}

fn interpret_items_as_queries_opts(
    items: &[HyperItem<'_, &str>],
    use_name_as_filter: bool,
) -> Vec<NamedQuery> {
    let mut queries = Vec::new();

    for item in items {
        match item {
            HyperItem::Node(n) => {
                let pred = interpret_node_pattern_opts(n, use_name_as_filter);
                let label = if n.inner.name == "_" {
                    format!("node_query_{}", queries.len())
                } else {
                    n.inner.name.to_string()
                };
                queries.push(NamedQuery { label, predicate: pred });
            }
            HyperItem::Edge(e) => {
                let pred = interpret_edge_pattern_opts(e, use_name_as_filter);
                let label = if e.inner.name == "_" {
                    format!("edge_query_{}", queries.len())
                } else {
                    e.inner.name.to_string()
                };
                queries.push(NamedQuery { label, predicate: pred });
            }
            HyperItem::Arc(_) => { /* top-level arcs in a query are unusual — skip */ }
        }
    }

    queries
}

/// Interpret a `NodeDecl` as a node-matching predicate.
fn interpret_node_pattern(n: &NodeDecl<'_, &str>) -> Predicate {
    interpret_node_pattern_opts(n, true)
}

/// Interpret a `NodeDecl` as a node-matching predicate.
///
/// `use_name_as_filter` — when `false`, the leading name is treated as a label
/// only (not an exact-name match constraint). Used for transform queries where
/// `links: link {}` means "any node inheriting link", labelled `links`.
fn interpret_node_pattern_opts(n: &NodeDecl<'_, &str>, use_name_as_filter: bool) -> Predicate {
    let mut parts: Vec<Predicate> = vec![Predicate::Kind(DeclKind::Node)];

    // Name: "_" = wildcard, anything else = exact match (unless suppressed)
    if use_name_as_filter && n.inner.name != "_" {
        parts.push(Predicate::named(n.inner.name));
    }

    // Bases: each base becomes an InheritsFrom constraint
    for base in &n.inner.bases {
        if let Some(base_name) = extract_leaf_name(base) {
            if base_name != "_" {
                parts.push(Predicate::inherits(base_name));
            }
        }
    }

    // Tags: partition into comparison operators vs normal tags
    let (comp_tags, normal_tags) = partition_tags(&n.anno.tags);
    for tag in &normal_tags {
        parts.push(Predicate::tagged(tag));
    }

    // Value with comparison tag
    if let Some(ref val) = n.anno.value {
        if let Some(comp) = comp_tags.first() {
            if let Some(vp) = make_comparison(comp, val) {
                parts.push(Predicate::HasValue(vp));
            }
        } else if let Some(vp) = value_to_predicate(val) {
            parts.push(Predicate::HasValue(vp));
        }
    }

    // Body children → HasChild constraints
    if let Some(ref body) = n.inner.body {
        for child in body {
            match child {
                HyperItem::Node(cn) => {
                    let cp = interpret_node_pattern(cn);
                    parts.push(Predicate::HasChild(Box::new(cp)));
                }
                HyperItem::Edge(ce) => {
                    let cp = interpret_edge_pattern(ce);
                    parts.push(Predicate::HasChild(Box::new(cp)));
                }
                _ => {}
            }
        }
    }

    if parts.len() == 1 { parts.into_iter().next().unwrap() }
    else { Predicate::And(parts) }
}

/// Interpret an `EdgeDecl` as an edge-matching predicate.
fn interpret_edge_pattern(e: &EdgeDecl<'_, &str>) -> Predicate {
    interpret_edge_pattern_opts(e, true)
}

fn interpret_edge_pattern_opts(e: &EdgeDecl<'_, &str>, use_name_as_filter: bool) -> Predicate {
    let mut parts: Vec<Predicate> = vec![Predicate::Kind(DeclKind::Edge)];

    if use_name_as_filter && e.inner.name != "_" {
        parts.push(Predicate::named(e.inner.name));
    }

    for base in &e.inner.bases {
        if let Some(base_name) = extract_leaf_name(base) {
            if base_name != "_" {
                parts.push(Predicate::inherits(base_name));
            }
        }
    }

    for tag in &e.anno.tags {
        parts.push(Predicate::tagged(tag));
    }

    // Body: arcs → arc ref predicates, nodes/edges → child predicates
    for child in &e.inner.body {
        match child {
            HyperItem::Arc(arc) => {
                for sref in &arc.inner.refs {
                    let (sign, target_pred) = interpret_signed_ref(sref);
                    let boxed = Box::new(target_pred);
                    match sign {
                        1  => parts.push(Predicate::HasPlusRef(boxed)),
                        -1 => parts.push(Predicate::HasMinusRef(boxed)),
                        0  => parts.push(Predicate::HasNeutralRef(boxed)),
                        _  => parts.push(Predicate::HasRef(boxed)),
                    }
                }
            }
            HyperItem::Node(cn) => {
                let cp = interpret_node_pattern(cn);
                parts.push(Predicate::HasChild(Box::new(cp)));
            }
            HyperItem::Edge(ce) => {
                let cp = interpret_edge_pattern(ce);
                parts.push(Predicate::HasChild(Box::new(cp)));
            }
        }
    }

    if parts.len() == 1 { parts.into_iter().next().unwrap() }
    else { Predicate::And(parts) }
}

/// Interpret a signed reference as a (sign, target_predicate) pair.
fn interpret_signed_ref(sref: &SignedRef<'_, &str>) -> (i8, Predicate) {
    let (sign, atom) = match sref {
        SignedRef::Plus(a)    => ( 1i8, a),
        SignedRef::Minus(a)   => (-1i8, a),
        SignedRef::Neutral(a) => ( 0i8, a),
    };

    let mut parts: Vec<Predicate> = Vec::new();

    // Target name: last segment of the ref path
    let leaf = atom.target.path.last().copied().unwrap_or("_");
    if leaf != "_" {
        // The ref target could be an inheritance constraint or a name
        // In query mode, a ref target like `_ : link` means "target inherits from link"
        // But the AST for `+ _ : link` puts `link` in the *bases* of the containing
        // node/edge, not on the ref. For simple refs like `+ base_link`, we match by name.
        parts.push(Predicate::named(leaf));
    }

    // Tags on the ref annotation act as additional constraints
    for tag in &atom.anno.tags {
        if *tag != "isa" {
            parts.push(Predicate::tagged(tag));
        }
    }

    let pred = if parts.is_empty() {
        Predicate::Any
    } else if parts.len() == 1 {
        parts.into_iter().next().unwrap()
    } else {
        Predicate::And(parts)
    };

    (sign, pred)
}

// ---- Helpers ----

/// Extract the leaf (last) name from a signed reference.
fn extract_leaf_name<'a>(sref: &SignedRef<'a, &'a str>) -> Option<&'a str> {
    let atom = match sref {
        SignedRef::Plus(a) | SignedRef::Minus(a) | SignedRef::Neutral(a) => a,
    };
    atom.target.path.last().copied()
}

/// Partition tags into comparison operators and normal tags.
/// Comparison operators: gt, lt, gte, lte, ne, eq
fn partition_tags<'a>(tags: &[&'a str]) -> (Vec<&'a str>, Vec<&'a str>) {
    let comps = ["gt", "lt", "gte", "lte", "ne", "eq"];
    let mut comp_tags = Vec::new();
    let mut normal_tags = Vec::new();
    for &tag in tags {
        if comps.contains(&tag) {
            comp_tags.push(tag);
        } else {
            normal_tags.push(tag);
        }
    }
    (comp_tags, normal_tags)
}

/// Build a `ValuePredicate` from a comparison operator tag and a value.
fn make_comparison(op: &str, val: &Value<'_, &str>) -> Option<ValuePredicate> {
    let n = match val {
        Value::Num(n) => *n,
        _ => return None,
    };
    Some(match op {
        "gt"  => ValuePredicate::NumGt(n),
        "lt"  => ValuePredicate::NumLt(n),
        "gte" => ValuePredicate::NumGte(n),
        "lte" => ValuePredicate::NumLte(n),
        "eq"  => ValuePredicate::NumEq(n),
        "ne"  => ValuePredicate::NumLt(n), // placeholder; ideally a separate NumNe
        _ => return None,
    })
}

/// Convert an AST value to an exact-match predicate.
fn value_to_predicate(val: &Value<'_, &str>) -> Option<ValuePredicate> {
    match val {
        Value::Num(n) => Some(ValuePredicate::NumEq(*n)),
        Value::Str(s) => Some(ValuePredicate::StrEq(s.to_string())),
        _ => Some(ValuePredicate::Any),
    }
}
