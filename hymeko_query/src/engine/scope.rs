//! Identity-bound selection over one borrowed canonical IR; predicates retain their full context.
use super::{QueryConfig, QueryEngine};
use crate::traits::NameResolver;
use hymeko::{common::ids::DeclId, ir::ir::Ir};

/// A requested selection cannot be represented by this query view.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScopeError {
    /// The requested root does not exist in this document.
    MissingRoot(DeclId),
    /// Narrowing an already scoped view would leave its selected declaration tree.
    OutsideParent(DeclId),
}

impl std::fmt::Display for ScopeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::MissingRoot(id) => write!(f, "scope root {} is absent", id.raw()),
            Self::OutsideParent(id) => write!(
                f,
                "scope root {} is outside the current selection",
                id.raw()
            ),
        }
    }
}
impl std::error::Error for ScopeError {}

impl<'ir, R: NameResolver> QueryEngine<'ir, R> {
    /// Narrow result selection to `root` and its descendants, by canonical identity.
    ///
    /// This borrows the same IR/resolver and preserves query configuration. Inheritance and
    /// reference predicates still inspect their original context; only returned declarations
    /// are restricted. No source, graph or declaration names are copied to establish membership.
    pub fn scoped(&self, root: DeclId) -> Result<Self, ScopeError> {
        if self.ir.decl_node(root).is_none() {
            return Err(ScopeError::MissingRoot(root));
        }
        if !self.includes(root) {
            return Err(ScopeError::OutsideParent(root));
        }
        Ok(Self {
            ir: self.ir,
            resolver: self.resolver,
            config: QueryConfig {
                max_inherit_depth: self.config.max_inherit_depth,
            },
            scope: Some(root),
        })
    }

    /// Root of the result selection; `None` denotes the original unrestricted view.
    pub fn scope_root(&self) -> Option<DeclId> {
        self.scope
    }

    pub(super) fn includes(&self, id: DeclId) -> bool {
        self.scope
            .is_none_or(|root| Lineage::new(self.ir, id).any(|ancestor| ancestor == root))
    }
}

/// Shared canonical ancestry walk for result scopes and template reachability scopes.
pub(crate) struct Lineage<'ir> {
    ir: &'ir Ir,
    next: DeclId,
    remaining: usize,
}
impl<'ir> Lineage<'ir> {
    pub(crate) fn new(ir: &'ir Ir, first: DeclId) -> Self {
        Self {
            ir,
            next: first,
            remaining: ir.decl_nodes.len(),
        }
    }
}
impl Iterator for Lineage<'_> {
    type Item = DeclId;
    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 {
            return None;
        }
        let current = self.next;
        let node = self.ir.decl_node(current)?;
        self.remaining -= 1;
        self.next = node.parent;
        Some(current)
    }
}

#[cfg(test)]
#[path = "../../tests/unit/engine/scope.rs"]
mod tests;
