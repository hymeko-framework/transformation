use super::*;
use crate::{Predicate, test_support};

#[test]
fn selection_is_identity_bound_and_keeps_external_predicate_context() {
    let model = test_support::compile(
        r#"
        description {} kinds { link {} }
        left { part: kinds.link {} nested { part: kinds.link {} } }
        right { part: kinds.link {} }
    "#,
    );
    let engine = QueryEngine::new(&model.program().ir, model.interner());
    let left = test_support::decl(&model, "left");
    let right = test_support::decl(&model, "right");
    let nested = test_support::decl(&model, "nested");
    let scoped = engine.scoped(left).unwrap();
    assert_eq!(scoped.scope_root(), Some(left));
    assert!(std::ptr::eq(scoped.ir(), engine.ir()));
    let links = Predicate::node().and(Predicate::inherits("link"));
    assert_eq!(engine.query(&links).len(), 3);
    assert_eq!(
        scoped.query(&links).len(),
        2,
        "the base kind outside the selection still resolves"
    );
    assert!(
        scoped.query_decl(left, &Predicate::Any).is_some(),
        "selection includes its root"
    );
    assert!(scoped.query_decl(right, &Predicate::Any).is_none());
    assert_eq!(scoped.scoped(nested).unwrap().query(&links).len(), 1);
    assert!(matches!(scoped.scoped(right), Err(ScopeError::OutsideParent(id)) if id == right));
    assert!(matches!(
        engine.scoped(DeclId::NONE),
        Err(ScopeError::MissingRoot(_))
    ));
    assert!(matches!(
        engine.scoped(DeclId::new(usize::MAX - 1)),
        Err(ScopeError::MissingRoot(_))
    ));
}
