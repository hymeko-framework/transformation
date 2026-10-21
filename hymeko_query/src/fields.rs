//! Effective fields: a declaration's own child, else its nearest base's.
//!
//! The IR keeps base relations but does not copy a base's children into an instance
//! (`box_1: + <isa> box_type` holds only its own fields). Consumers that read a base as the
//! default and the instance as the override resolve fields through these engine methods, so the
//! rule is the same everywhere: own declaration first, then bases depth-first in authored order.
//! Repeated bases and cycles terminate; the walk visits each declaration at most once. A writer
//! that may touch only a declaration's own slot uses [`own_child`] instead.

use crate::bases::{BasePolicy, bases};
use crate::engine::QueryEngine;
use crate::traits::NameResolver;
use hymeko::common::ids::{DeclId, SymId};
use hymeko::ir::ir::{Ir, ValueR};

/// Where an effective field comes from: the child `declaration`, and the `provider` whose own
/// child it is — the declaration asked about, or the base that supplied it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FieldBinding {
    pub declaration: DeclId,
    pub provider: DeclId,
}

impl FieldBinding {
    /// Whether `decl` declares the field itself rather than inheriting it.
    #[must_use]
    pub fn is_own(&self, decl: DeclId) -> bool {
        self.provider == decl
    }
}

/// The child of `decl` itself whose name satisfies `named`; never an inherited one.
pub fn own_child_where(ir: &Ir, decl: DeclId, named: impl Fn(SymId) -> bool) -> Option<DeclId> {
    ir.decl_children(decl)
        .find(|child| ir.decl_node(*child).is_some_and(|node| named(node.name)))
}

/// The child of `decl` itself named `name`; never an inherited one. The slot rule for writers.
pub fn own_child(ir: &Ir, decl: DeclId, name: SymId) -> Option<DeclId> {
    own_child_where(ir, decl, |candidate| candidate == name)
}

impl<'a, R: NameResolver> QueryEngine<'a, R> {
    /// The field `key` of `decl` and where it comes from: `decl`'s own child, else the first base,
    /// depth-first in authored order, that has one.
    pub fn effective_binding(&self, decl: DeclId, key: &str) -> Option<FieldBinding> {
        let ir = self.ir();
        let mut pending = vec![decl];
        let mut visited = std::collections::HashSet::new();
        while let Some(provider) = pending.pop() {
            if ir.decl_node(provider).is_none() || !visited.insert(provider) {
                continue;
            }
            if let Some(declaration) = self.own_child(provider, key) {
                return Some(FieldBinding {
                    declaration,
                    provider,
                });
            }
            // Reversed so that the first authored base is popped, and searched, first.
            pending.extend(bases(ir, provider, BasePolicy::All).rev());
        }
        None
    }

    /// The child named `key` of `decl` itself, else of the first base that has one.
    pub fn effective_child(&self, decl: DeclId, key: &str) -> Option<DeclId> {
        self.effective_binding(decl, key)
            .map(|binding| binding.declaration)
    }

    /// The child named `key` of `decl` itself; never an inherited one.
    pub fn own_child(&self, decl: DeclId, key: &str) -> Option<DeclId> {
        own_child_where(self.ir(), decl, |name| self.resolver().resolve(name) == key)
    }

    /// The value annotated on the effective child `key` of `decl`, if it carries one.
    pub fn effective_value(&self, decl: DeclId, key: &str) -> Option<&'a ValueR> {
        let ir = self.ir();
        self.effective_child(decl, key)
            .and_then(|child| ir.decl_annotation(child)?.value.as_ref())
    }

    /// Every effective field of `decl`: the value-carrying children of `decl` and of its bases, as
    /// keys in walk order (own first, then bases depth-first in authored order), each bound by
    /// [`Self::effective_binding`] — so which child supplies a key is decided in one place. A child
    /// without a value is structure, not a field. Repeated bases and cycles terminate.
    pub fn effective_fields(&self, decl: DeclId) -> Vec<(&'a str, FieldBinding)> {
        let ir = self.ir();
        let mut keys = Vec::new();
        let mut seen = std::collections::HashSet::new();
        let mut visited = std::collections::HashSet::new();
        let mut pending = vec![decl];
        while let Some(provider) = pending.pop() {
            if ir.decl_node(provider).is_none() || !visited.insert(provider) {
                continue;
            }
            for child in ir.decl_children(provider) {
                if let Some(node) = ir
                    .decl_node(child)
                    .filter(|node| ir[node.anno].value.is_some())
                {
                    let key = self.resolver().resolve(node.name);
                    if seen.insert(key) {
                        keys.push(key);
                    }
                }
            }
            pending.extend(bases(ir, provider, BasePolicy::All).rev());
        }
        keys.into_iter()
            .filter_map(|key| Some((key, self.effective_binding(decl, key)?)))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::{compile, decl};
    use hymeko::module_store::execution::CompiledModel;

    fn number(model: &CompiledModel, name: &str, key: &str) -> Option<f64> {
        let engine = QueryEngine::new(&model.program().ir, model.interner());
        match engine.effective_value(decl(model, name), key) {
            Some(ValueR::Num(value)) => Some(*value),
            _ => None,
        }
    }

    #[test]
    fn effective_fields_list_own_then_inherited_keys_each_bound_as_effective_binding_binds_it() {
        let model = compile(
            "fields {} types { thing { size 1; mass 10; } heavy { mass 50; colour 3; } } \
             c: + <isa> types.heavy, + <isa> types.thing { mass 7; label \"c\"; nested { depth 2; } }",
        );
        let ir = &model.program().ir;
        let engine = QueryEngine::new(ir, model.interner());
        let c = decl(&model, "c");
        let fields = engine.effective_fields(c);
        let keys: Vec<&str> = fields.iter().map(|(key, _)| *key).collect();
        assert_eq!(
            keys,
            ["mass", "label", "colour", "size"],
            "own first, then bases in authored order"
        );
        for (key, binding) in &fields {
            assert_eq!(
                Some(*binding),
                engine.effective_binding(c, key),
                "one precedence rule"
            );
        }
        let own: Vec<bool> = fields
            .iter()
            .map(|(_, binding)| binding.is_own(c))
            .collect();
        assert_eq!(own, [true, true, false, false]);
        assert!(
            !keys.contains(&"nested"),
            "a child without a value is structure"
        );
        assert!(engine.effective_fields(decl(&model, "types")).is_empty());
    }

    #[test]
    fn own_fields_override_bases_which_are_searched_in_authored_order() {
        let model = compile(
            "fields {} types { thing { size 1; mass 10; } heavy { mass 50; colour 3; } } \
             a: + <isa> types.heavy, + <isa> types.thing {} \
             b: + <isa> types.thing, + <isa> types.heavy {} \
             c: + <isa> types.heavy { mass 7; } \
             d: + <isa> c {}",
        );
        assert_eq!(
            number(&model, "a", "mass"),
            Some(50.),
            "the first authored base wins"
        );
        assert_eq!(number(&model, "b", "mass"), Some(10.));
        assert_eq!(
            number(&model, "a", "size"),
            Some(1.),
            "later bases fill what earlier ones lack"
        );
        assert_eq!(
            number(&model, "c", "mass"),
            Some(7.),
            "an own declaration overrides every base"
        );
        assert_eq!(
            number(&model, "d", "mass"),
            Some(7.),
            "inheritance is transitive"
        );
        assert_eq!(number(&model, "d", "colour"), Some(3.));
        assert_eq!(number(&model, "d", "missing"), None);
        let ir = &model.program().ir;
        let size = QueryEngine::new(ir, model.interner())
            .effective_child(decl(&model, "a"), "size")
            .unwrap();
        assert_eq!(
            ir.parent(size),
            decl(&model, "thing"),
            "the child keeps the base's identity"
        );
    }

    #[test]
    fn repeated_bases_and_invalid_declarations_terminate() {
        let model = compile(
            "fields {} root { v 1; } left: + <isa> root {} right: + <isa> root {} \
             joined: + <isa> left, + <isa> right {}",
        );
        assert_eq!(number(&model, "joined", "v"), Some(1.));
        assert_eq!(number(&model, "joined", "w"), None);
        let engine = QueryEngine::new(&model.program().ir, model.interner());
        assert!(
            engine
                .effective_child(DeclId::new(usize::MAX >> 1), "v")
                .is_none()
        );
    }

    #[test]
    fn a_binding_names_its_provider_and_own_child_never_inherits() {
        let model = compile(
            "fields {} types { heavy { mass 50; colour 3; } } c: + <isa> types.heavy { mass 7; }",
        );
        let (c, heavy) = (decl(&model, "c"), decl(&model, "heavy"));
        let engine = QueryEngine::new(&model.program().ir, model.interner());
        let own = engine.effective_binding(c, "mass").unwrap();
        assert!(own.is_own(c) && own.provider == c);
        let inherited = engine.effective_binding(c, "colour").unwrap();
        assert_eq!(inherited.provider, heavy);
        assert!(!inherited.is_own(c));
        assert_eq!(
            engine.effective_child(c, "colour"),
            Some(inherited.declaration)
        );
        assert_eq!(engine.own_child(c, "mass"), Some(own.declaration));
        assert_eq!(
            engine.own_child(c, "colour"),
            None,
            "a writer never reaches a base's slot"
        );
        let colour = model.interner().get_id("colour").unwrap();
        assert_eq!(
            own_child(&model.program().ir, heavy, colour),
            Some(inherited.declaration)
        );
    }

    /// S4: in a diamond the shared root is reached through the first authored base before the
    /// second base is searched, so depth-first order answers with the root's value. C3 would
    /// answer with `b`'s. This pins today's rule until DS2 decides.
    #[test]
    fn a_diamond_resolves_depth_first_in_authored_order() {
        let model = compile(
            "fields {} types { r { v 1; } l: + <isa> r {} b: + <isa> r { v 2; } }              x: + <isa> types.l, + <isa> types.b {}",
        );
        assert_eq!(number(&model, "x", "v"), Some(1.));
        let engine = QueryEngine::new(&model.program().ir, model.interner());
        let binding = engine.effective_binding(decl(&model, "x"), "v").unwrap();
        assert_eq!(binding.provider, decl(&model, "r"));
    }
}
