//! Match context: field extraction from a QueryMatch against the IR.
//!
//! Given a matched declaration, provides `get_field("mass")` → `Some("1.5")`
//! by looking up named children under that declaration. This decouples the
//! template engine from domain-specific knowledge about what fields exist.

use crate::engine::QueryMatch;
use crate::traits::NameResolver;
use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{DeclKind, Ir, ValueR};

/// Extracted field value (string representation for template interpolation).
#[derive(Clone, Debug)]
pub enum FieldValue {
    Str(String),
    Num(f64),
    List(Vec<FieldValue>),
    /// The field exists but has no value (flag/tag).
    Present,
    /// The field does not exist.
    Missing,
}

impl FieldValue {
    pub fn is_missing(&self) -> bool {
        matches!(self, FieldValue::Missing)
    }

    pub fn is_present(&self) -> bool {
        !self.is_missing()
    }

    /// Render to string for template interpolation.
    pub fn to_template_string(&self) -> String {
        match self {
            FieldValue::Str(s) => s.clone(),
            FieldValue::Num(n) => {
                if (*n - n.round()).abs() < 1e-10 && n.abs() < 1e12 {
                    format!("{}", *n as i64)
                } else {
                    format!("{n}")
                }
            }
            FieldValue::List(vs) => vs
                .iter()
                .map(|v| v.to_template_string())
                .collect::<Vec<_>>()
                .join(" "),
            FieldValue::Present => "true".into(),
            FieldValue::Missing => String::new(),
        }
    }
}

/// Context for extracting fields from a matched declaration.
pub struct MatchContext<'a, R: NameResolver> {
    pub ir: &'a Ir,
    pub resolver: &'a R,
    pub match_result: &'a QueryMatch,
}

impl<'a, R: NameResolver> MatchContext<'a, R> {
    pub fn new(ir: &'a Ir, resolver: &'a R, match_result: &'a QueryMatch) -> Self {
        Self {
            ir,
            resolver,
            match_result,
        }
    }

    /// Get the value of a named child field.
    ///
    /// Looks for a child declaration named `field_name` under the matched
    /// declaration, and returns its value (annotation value, or the child's
    /// own name if it's a reference).
    ///
    /// Supports dotted paths: `"geometry.shape"` → find child "geometry",
    /// then find its child "shape". An empty path reads the matched declaration
    /// itself, consistent with `resolve_field_decl("")`. Intermediate value
    /// references are followed; terminal references retain their own identity.
    pub fn get_field(&self, field_path: &str) -> FieldValue {
        if field_path.is_empty() {
            return self.decl_value(self.match_result.id);
        }
        let segments: Vec<&str> = field_path.split('.').collect();
        self.get_field_at(self.match_result.id, &segments)
    }

    /// Resolve a dotted field path to the underlying `DeclId` without
    /// pulling its value — used by `{{#inherits}}` to inspect the base
    /// chain of a nested declaration.
    pub fn resolve_field_decl(&self, field_path: &str) -> Option<DeclId> {
        if field_path.is_empty() {
            return Some(self.match_result.id);
        }
        let segments: Vec<&str> = field_path.split('.').collect();
        self.resolve_decl_at(self.match_result.id, &segments)
    }

    fn resolve_decl_at(&self, parent: DeclId, segments: &[&str]) -> Option<DeclId> {
        if segments.is_empty() {
            return Some(parent);
        }
        let parent = self.reference_scope(parent)?;
        let target_name = segments[0];
        let rest = &segments[1..];
        for child in self.ir.decl_children(parent) {
            let child_name = self.resolver.resolve(self.ir.decl_nodes[child.raw()].name);
            if child_name == target_name {
                return self.resolve_decl_at(child, rest);
            }
        }
        None
    }

    fn get_field_at(&self, parent: DeclId, segments: &[&str]) -> FieldValue {
        if segments.is_empty() {
            // Return the value of this declaration itself
            return self.decl_value(parent);
        }

        let Some(parent) = self.reference_scope(parent) else {
            return FieldValue::Missing;
        };

        let target_name = segments[0];
        let rest = &segments[1..];

        // Search children for a matching name
        for child in self.ir.decl_children(parent) {
            let child_name = self.resolver.resolve(self.ir.decl_nodes[child.raw()].name);
            if child_name == target_name {
                if rest.is_empty() {
                    return self.decl_value(child);
                } else {
                    return self.get_field_at(child, rest);
                }
            }
        }

        // Try following references: if there's a child that's a reference
        // to a node named `target_name`, follow it
        for child in self.ir.decl_children(parent) {
            let decl = &self.ir.decl_nodes[child.raw()];
            if decl.kind == DeclKind::Edge {
                if let Some(eid) = self.ir.as_edge(child) {
                    let edge = &self.ir.edges[eid.raw()];
                    for base in &edge.bases {
                        let target = base.target();
                        if !target.is_none() {
                            let target_name_resolved =
                                self.resolver.resolve(self.ir.decl_nodes[target.raw()].name);
                            if target_name_resolved == target_name {
                                if rest.is_empty() {
                                    return self.decl_value(target);
                                } else {
                                    return self.get_field_at(target, rest);
                                }
                            }
                        }
                    }
                }
            }
        }

        FieldValue::Missing
    }

    // Reference aliases are not containment. Bound traversal independently of
    // path length so cyclic aliases cannot recurse indefinitely.
    fn reference_scope(&self, mut id: DeclId) -> Option<DeclId> {
        for _ in 0..32 {
            match self.ir.decl_node(id)?.anno.value {
                Some(ValueR::Ref(target)) => id = target,
                _ => return Some(id),
            }
        }
        None
    }

    /// Extract the value from a declaration node.
    fn decl_value(&self, did: DeclId) -> FieldValue {
        if did.is_none() {
            return FieldValue::Missing;
        }
        let decl = &self.ir.decl_nodes[did.raw()];

        // Check annotation value first
        if let Some(ref val) = decl.anno.value {
            return self.value_r_to_field(val);
        }

        // If it's a node, check if it has exactly one child with a value
        // (common pattern: `mass { 1.5 }` → the value is on the child)
        let mut single_child = None;
        let mut child_count = 0;
        for child in self.ir.decl_children(did) {
            child_count += 1;
            single_child = Some(child);
        }
        if child_count == 1 {
            if let Some(c) = single_child {
                let c_decl = &self.ir.decl_nodes[c.raw()];
                if let Some(ref val) = c_decl.anno.value {
                    return self.value_r_to_field(val);
                }
            }
        }

        // The declaration exists but has no scalar value — it's a structural node
        FieldValue::Present
    }

    fn value_r_to_field(&self, val: &ValueR) -> FieldValue {
        match val {
            ValueR::Num(n) => FieldValue::Num(*n),
            ValueR::Str(sid) => FieldValue::Str(self.resolver.resolve(*sid).to_string()),
            ValueR::List(vs) => {
                FieldValue::List(vs.iter().map(|v| self.value_r_to_field(v)).collect())
            }
            ValueR::Ref(did) => {
                if did.is_none() {
                    FieldValue::Missing
                } else {
                    let name = self.resolver.resolve(self.ir.decl_nodes[did.raw()].name);
                    FieldValue::Str(name.to_string())
                }
            }
        }
    }

    /// Get all children of the matched declaration as (name, FieldValue) pairs.
    pub fn children_fields(&self) -> Vec<(String, FieldValue)> {
        let mut result = Vec::new();
        for child in self.ir.decl_children(self.match_result.id) {
            let name = self
                .resolver
                .resolve(self.ir.decl_nodes[child.raw()].name)
                .to_string();
            let value = self.decl_value(child);
            result.push((name, value));
        }
        result
    }

    /// Get the n-th arc binding with the given sign (1 = +, -1 = -, 0 = neutral).
    pub fn binding_target(&self, sign: i8, index: usize) -> FieldValue {
        let filtered: Vec<_> = self
            .match_result
            .arc_bindings
            .iter()
            .filter(|b| b.sign == sign)
            .collect();
        match filtered.get(index) {
            Some(b) => FieldValue::Str(b.target_name.clone()),
            None => FieldValue::Missing,
        }
    }

    /// Get all positive binding targets as a list.
    pub fn positive_bindings(&self) -> Vec<String> {
        self.match_result
            .arc_bindings
            .iter()
            .filter(|b| b.sign > 0)
            .map(|b| b.target_name.clone())
            .collect()
    }

    /// Get all negative binding targets as a list.
    pub fn negative_bindings(&self) -> Vec<String> {
        self.match_result
            .arc_bindings
            .iter()
            .filter(|b| b.sign < 0)
            .map(|b| b.target_name.clone())
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Predicate, QueryEngine};
    use hymeko::module_store::{execution::CompiledModel, source_provider::MemProvider};
    use hymeko::util::real_parser::RealParser;

    #[test]
    fn dotted_fields_follow_aliases_but_preserve_terminal_reference_identity() {
        let mut files = MemProvider::default();
        files.insert_file("/references.hymeko", "description {} state { value 7; } alias -> state; item { observation -> alias; } cycle { next -> loop; } loop -> cycle.next;");
        let model = CompiledModel::compile(files, RealParser, std::path::Path::new("/references.hymeko")).unwrap();
        let ir = &model.program().ir;
        let query = QueryEngine::new(ir, model.interner());
        let declarations = query.query(&Predicate::Any);
        let item = declarations.iter().find(|d| d.name == "item").unwrap();
        let context = MatchContext::new(ir, model.interner(), item);
        assert!(matches!(context.get_field("observation.value"), FieldValue::Num(7.)));
        let value = context.resolve_field_decl("observation.value").unwrap();
        assert_eq!(model.interner().resolve(ir.decl_node(value).unwrap().name), "value");
        let reference = context.resolve_field_decl("observation").unwrap();
        assert_eq!(model.interner().resolve(ir.decl_node(reference).unwrap().name), "observation");
        assert!(matches!(context.get_field("observation"), FieldValue::Str(name) if name == "alias"));
        assert!(context.get_field("observation.missing").is_missing());
        let cycle = declarations.iter().find(|d| d.name == "cycle").unwrap();
        let context = MatchContext::new(ir, model.interner(), cycle);
        assert!(context.get_field("next.value").is_missing());
        assert!(context.resolve_field_decl("next.value").is_none());
    }
}
