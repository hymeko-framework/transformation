//! Effective fields: a declaration's own child, else its nearest base's.
//!
//! The IR keeps base relations but does not copy a base's children into an instance
//! (`box_1: + <isa> box_type` holds only its own fields). Consumers that read a base as the
//! default and the instance as the override resolve fields through these engine methods, so the
//! rule is the same everywhere: own declaration first, then bases depth-first in authored order.
//! Repeated bases and cycles terminate; the walk visits each declaration at most once.

use crate::engine::QueryEngine;
use crate::traits::NameResolver;
use hymeko::common::ids::DeclId;
use hymeko::ir::ir::ValueR;

impl<'a, R: NameResolver> QueryEngine<'a, R> {
    /// The child named `key` of `decl` itself, else of the first base that has one.
    pub fn effective_child(&self, decl: DeclId, key: &str) -> Option<DeclId> {
        let ir = self.ir();
        let mut pending = vec![decl];
        let mut visited = std::collections::HashSet::new();
        while let Some(decl) = pending.pop() {
            if ir.decl_node(decl).is_none() || !visited.insert(decl) {
                continue;
            }
            let own = ir.decl_children(decl).find(|child| {
                ir.decl_node(*child)
                    .is_some_and(|node| self.resolver().resolve(node.name) == key)
            });
            if own.is_some() {
                return own;
            }
            // Reversed so that the first authored base is popped, and searched, first.
            if let Some((_, node)) = ir.node_for_decl(decl) {
                pending.extend(node.bases.iter().rev().map(|base| base.target()));
            } else if let Some((_, edge)) = ir.edge_for_decl(decl) {
                pending.extend(edge.bases.iter().rev().map(|base| base.target()));
            }
        }
        None
    }

    /// The value annotated on the effective child `key` of `decl`, if it carries one.
    pub fn effective_value(&self, decl: DeclId, key: &str) -> Option<&'a ValueR> {
        let ir = self.ir();
        self.effective_child(decl, key)
            .and_then(|child| ir.decl_node(child)?.anno.value.as_ref())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hymeko::module_store::{execution::CompiledModel, source_provider::MemProvider};
    use hymeko::util::real_parser::RealParser;

    fn compile(source: &str) -> CompiledModel {
        let mut files = MemProvider::default();
        files.insert_file("/fields.hymeko", source);
        CompiledModel::compile(files, RealParser, std::path::Path::new("/fields.hymeko")).unwrap()
    }
    fn decl(model: &CompiledModel, name: &str) -> DeclId {
        let ir = &model.program().ir;
        ir.decls().find(|(_, node)| model.interner().resolve(node.name) == name).unwrap().0
    }
    fn number(model: &CompiledModel, name: &str, key: &str) -> Option<f64> {
        let engine = QueryEngine::new(&model.program().ir, model.interner());
        match engine.effective_value(decl(model, name), key) {
            Some(ValueR::Num(value)) => Some(*value),
            _ => None,
        }
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
        assert_eq!(number(&model, "a", "mass"), Some(50.), "the first authored base wins");
        assert_eq!(number(&model, "b", "mass"), Some(10.));
        assert_eq!(number(&model, "a", "size"), Some(1.), "later bases fill what earlier ones lack");
        assert_eq!(number(&model, "c", "mass"), Some(7.), "an own declaration overrides every base");
        assert_eq!(number(&model, "d", "mass"), Some(7.), "inheritance is transitive");
        assert_eq!(number(&model, "d", "colour"), Some(3.));
        assert_eq!(number(&model, "d", "missing"), None);
        let ir = &model.program().ir;
        let size = QueryEngine::new(ir, model.interner()).effective_child(decl(&model, "a"), "size").unwrap();
        assert_eq!(ir.parent(size), decl(&model, "thing"), "the child keeps the base's identity");
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
        assert!(engine.effective_child(DeclId::new(usize::MAX >> 1), "v").is_none());
    }
}
