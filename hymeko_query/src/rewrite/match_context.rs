//! Match context: field extraction from a QueryMatch against the IR.
//!
//! Given a matched declaration, provides `get_field("mass")` → `Some("1.5")`
//! by looking up named children under that declaration. This decouples the
//! template engine from domain-specific knowledge about what fields exist.

use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{DeclKind, Ir, ValueR};
use crate::engine::QueryMatch;
use crate::traits::NameResolver;

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
            FieldValue::List(vs) => {
                vs.iter()
                    .map(|v| v.to_template_string())
                    .collect::<Vec<_>>()
                    .join(" ")
            }
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
        Self { ir, resolver, match_result }
    }

    /// Get the value of a named child field.
    ///
    /// Looks for a child declaration named `field_name` under the matched
    /// declaration, and returns its value (annotation value, or the child's
    /// own name if it's a reference).
    ///
    /// Supports dotted paths: `"geometry.shape"` → find child "geometry",
    /// then find its child "shape".
    pub fn get_field(&self, field_path: &str) -> FieldValue {
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
        let target_name = segments[0];
        let rest = &segments[1..];
        for child in self.ir.decl_children(parent) {
            let child_name = self.resolver.resolve(self.ir.decl_nodes[child.0].name);
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

        let target_name = segments[0];
        let rest = &segments[1..];

        // Search children for a matching name
        for child in self.ir.decl_children(parent) {
            let child_name = self.resolver.resolve(self.ir.decl_nodes[child.0].name);
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
            let decl = &self.ir.decl_nodes[child.0];
            if decl.kind == DeclKind::Edge {
                if let Some(eid) = self.ir.as_edge(child) {
                    let edge = &self.ir.edges[eid.0];
                    for base in &edge.bases {
                        let target = base.target();
                        if !target.is_none() {
                            let target_name_resolved = self.resolver.resolve(
                                self.ir.decl_nodes[target.0].name
                            );
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

    /// Extract the value from a declaration node.
    fn decl_value(&self, did: DeclId) -> FieldValue {
        if did.is_none() {
            return FieldValue::Missing;
        }
        let decl = &self.ir.decl_nodes[did.0];

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
                let c_decl = &self.ir.decl_nodes[c.0];
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
                    let name = self.resolver.resolve(self.ir.decl_nodes[did.0].name);
                    FieldValue::Str(name.to_string())
                }
            }
        }
    }

    /// Get all children of the matched declaration as (name, FieldValue) pairs.
    pub fn children_fields(&self) -> Vec<(String, FieldValue)> {
        let mut result = Vec::new();
        for child in self.ir.decl_children(self.match_result.id) {
            let name = self.resolver.resolve(self.ir.decl_nodes[child.0].name).to_string();
            let value = self.decl_value(child);
            result.push((name, value));
        }
        result
    }

    /// Get the n-th arc binding with the given sign (1 = +, -1 = -, 0 = neutral).
    pub fn binding_target(&self, sign: i8, index: usize) -> FieldValue {
        let filtered: Vec<_> = self.match_result.arc_bindings.iter()
            .filter(|b| b.sign == sign)
            .collect();
        match filtered.get(index) {
            Some(b) => FieldValue::Str(b.target_name.clone()),
            None => FieldValue::Missing,
        }
    }

    /// Get all positive binding targets as a list.
    pub fn positive_bindings(&self) -> Vec<String> {
        self.match_result.arc_bindings.iter()
            .filter(|b| b.sign > 0)
            .map(|b| b.target_name.clone())
            .collect()
    }

    /// Get all negative binding targets as a list.
    pub fn negative_bindings(&self) -> Vec<String> {
        self.match_result.arc_bindings.iter()
            .filter(|b| b.sign < 0)
            .map(|b| b.target_name.clone())
            .collect()
    }
}
