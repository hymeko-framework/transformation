//! Read-only requirements trace projection. Binary roles are validated, never truncated.
//! Identities remain declarations of the caller's Core snapshot. This profile requires
//! one flat scope; nested or ambiguous template selections are rejected explicitly.
use std::collections::{HashMap, HashSet};

use crate::{NameResolver, Predicate, QueryEngine};
use hymeko::{
    common::ids::DeclId,
    ir::ir::{DeclKind, ValueR},
};

/// Exact vocabulary identities in the borrowed document, not globally matched names.
pub struct TraceVocabulary {
    pub requirement: DeclId,
    pub component: DeclId,
    pub test_case: DeclId,
    pub satisfies: DeclId,
    pub allocated_to: DeclId,
    pub derives: DeclId,
    pub verifies: DeclId,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TraceEntity {
    pub declaration: DeclId,
    pub name: String,
    pub kind: &'static str,
    pub text: Option<String>,
    /// Authored evidence record: -1 unknown, 0 fail, 1 pass. No test is executed here.
    pub outcome: Option<i8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TraceRelation {
    pub declaration: DeclId,
    pub name: String,
    pub kind: &'static str,
    pub source: String,
    pub target: String,
}

/// A derived view, never a mutable canonical model. Named parallel edges stay distinct.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Traceability {
    pub entities: Vec<TraceEntity>,
    pub relations: Vec<TraceRelation>,
}

impl Traceability {
    /// Read the exact typed relation set. Rejects unsupported arity, weights, polarity,
    /// role types, non-flat scope, ambiguous type names and ambiguous entity membership.
    pub fn read<R: NameResolver>(
        query: &QueryEngine<'_, R>,
        scope: DeclId,
        vocabulary: &TraceVocabulary,
    ) -> Result<Self, String> {
        if !query
            .ir()
            .decl_node(scope)
            .is_some_and(|n| n.kind == DeclKind::Node)
        {
            return Err("trace scope must be a node".into());
        }
        let mut result = Self {
            entities: Vec::new(),
            relations: Vec::new(),
        };
        let mut entities = HashMap::new();
        for (base, kind, selector) in [
            (vocabulary.requirement, "requirement", "requirement"),
            (vocabulary.component, "component", "block"),
            (vocabulary.test_case, "test_case", "test_case"),
        ] {
            for matched in profile_matches(query, scope, base, selector, DeclKind::Node)? {
                if entities
                    .insert(matched.id, (kind, matched.name.clone()))
                    .is_some()
                {
                    return Err(format!("ambiguous entity kind: {}", matched.name));
                }
                let text = match own_value(query, matched.id, "text") {
                    None => None,
                    Some(ValueR::Str(value)) => Some(query.resolver().resolve(*value).to_owned()),
                    _ => return Err(format!("text must be a string: {}", matched.name)),
                };
                let outcome = if kind == "test_case" {
                    match own_value(query, matched.id, "outcome") {
                        Some(ValueR::Num(value)) if [-1., 0., 1.].contains(value) => {
                            Some(*value as i8)
                        }
                        _ => return Err(format!("outcome must be -1, 0 or 1: {}", matched.name)),
                    }
                } else {
                    None
                };
                result.entities.push(TraceEntity {
                    declaration: matched.id,
                    name: matched.name,
                    kind,
                    text,
                    outcome,
                });
            }
        }
        for (base, kind, source_kind, target_kind) in [
            (
                vocabulary.satisfies,
                "satisfies",
                "component",
                "requirement",
            ),
            (
                vocabulary.allocated_to,
                "allocated_to",
                "requirement",
                "component",
            ),
            (vocabulary.derives, "derives", "requirement", "requirement"),
            (vocabulary.verifies, "verifies", "test_case", "requirement"),
        ] {
            for matched in profile_matches(query, scope, base, kind, DeclKind::Edge)? {
                let edge = query
                    .ir()
                    .as_edge(matched.id)
                    .ok_or("expected trace edge")?;
                if query.ir().edges[edge.raw()].arcs.len() != 1 || matched.arc_bindings.len() != 2 {
                    return Err(format!(
                        "trace edge requires one binary arc: {}",
                        matched.name
                    ));
                }
                let mut endpoints = [None, None];
                for binding in &matched.arc_bindings {
                    let slot = match binding.sign {
                        1 => 0,
                        -1 => 1,
                        _ => return Err(format!("neutral trace incidence: {}", matched.name)),
                    };
                    if binding.weights.is_some()
                        || endpoints[slot].replace(binding.target).is_some()
                    {
                        return Err(format!("weighted or repeated trace role: {}", matched.name));
                    }
                }
                let endpoint = |slot: usize, expected: &str| -> Result<String, String> {
                    let (kind, name) = endpoints[slot]
                        .and_then(|id| entities.get(&id))
                        .ok_or_else(|| format!("missing typed endpoint: {}", matched.name))?;
                    if *kind != expected {
                        return Err(format!("wrong endpoint kind: {}", matched.name));
                    }
                    Ok(name.clone())
                };
                result.relations.push(TraceRelation {
                    declaration: matched.id,
                    name: matched.name.clone(),
                    kind,
                    source: endpoint(0, source_kind)?,
                    target: endpoint(1, target_kind)?,
                });
            }
        }
        result.entities.sort_by(|a, b| a.name.cmp(&b.name));
        result.relations.sort_by(|a, b| a.name.cmp(&b.name));
        let mut names = HashSet::new();
        for name in result
            .entities
            .iter()
            .map(|r| &r.name)
            .chain(result.relations.iter().map(|r| &r.name))
        {
            if !names.insert(name) {
                return Err(format!("duplicate trace name: {name}"));
            }
        }
        Ok(result)
    }
}

fn own_value<'a, R: NameResolver>(
    query: &QueryEngine<'a, R>,
    id: DeclId,
    field: &str,
) -> Option<&'a ValueR> {
    query
        .own_child(id, field)
        .and_then(|child| query.ir().decl_annotation(child)?.value.as_ref())
}

fn profile_matches<R: NameResolver>(
    query: &QueryEngine<'_, R>,
    scope: DeclId,
    base: DeclId,
    selector: &str,
    kind: DeclKind,
) -> Result<Vec<crate::QueryMatch>, String> {
    if !query.ir().decl_node(base).is_some_and(|n| n.kind == kind) {
        return Err(format!("invalid vocabulary type: {selector}"));
    }
    let exact = Predicate::Kind(kind).and(Predicate::InheritsFromId(base));
    let matches = query.query(&exact);
    let exact_ids: HashSet<_> = matches.iter().map(|m| m.id).collect();
    let template_ids: HashSet<_> = query
        .query_iter(&Predicate::Kind(kind).and(Predicate::inherits(selector)))
        .map(|m| m.id)
        .collect();
    if exact_ids != template_ids {
        return Err(format!("ambiguous template type selection: {selector}"));
    }
    if matches.iter().any(|m| query.ir().parent(m.id) != scope) {
        return Err(format!("trace profile requires one flat scope: {selector}"));
    }
    Ok(matches)
}
