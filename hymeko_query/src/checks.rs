//! Structural assertions over existing queries; no second graph or predicate engine.
use hymeko::common::ids::DeclId;

use crate::{NameResolver, NamedQuery, QueryEngine};

/// Required number of matches. Use `Exactly(0)` to forbid a pattern.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MatchExpectation {
    Exactly(usize),
    AtLeast(usize),
    AtMost(usize),
}

impl MatchExpectation {
    pub fn accepts(self, count: usize) -> bool {
        match self {
            Self::Exactly(n) => count == n,
            Self::AtLeast(n) => count >= n,
            Self::AtMost(n) => count <= n,
        }
    }
}

/// A named assertion and a bounded sample of its matching canonical declarations.
///
/// Witnesses are matches, not necessarily offending declarations: when an
/// `AtLeast` assertion fails, absence is the violation and there may be no IDs.
#[derive(Clone, Debug)]
pub struct QueryCheck {
    pub label: String,
    pub expected: MatchExpectation,
    pub actual: usize,
    pub passed: bool,
    pub witnesses: Vec<DeclId>,
    pub witnesses_truncated: bool,
}

impl<R: NameResolver> QueryEngine<'_, R> {
    /// Evaluate a query assertion without collecting all query matches.
    ///
    /// Counts all matches exactly; retains at most `witness_limit` declaration IDs.
    /// Predicate interpretation is identical to `query_iter`, including scope,
    /// inheritance and direct versus effective field semantics.
    pub fn check_query(
        &self,
        query: &NamedQuery,
        expected: MatchExpectation,
        witness_limit: usize,
    ) -> QueryCheck {
        let mut actual = 0;
        let mut witnesses = Vec::new();
        for matched in self.query_iter(&query.predicate) {
            actual += 1;
            if witnesses.len() < witness_limit {
                witnesses.push(matched.id);
            }
        }
        QueryCheck {
            label: query.label.clone(),
            expected,
            actual,
            passed: expected.accepts(actual),
            witnesses_truncated: actual > witnesses.len(),
            witnesses,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Predicate, ValuePredicate};
    use hymeko::module_store::{execution::CompiledModel, source_provider::MemProvider};
    use hymeko::util::real_parser::RealParser;
    use std::path::Path;

    fn model(source: &str) -> CompiledModel {
        CompiledModel::compile(
            MemProvider::default().with_file("/checks.hymeko", source),
            RealParser,
            Path::new("/checks.hymeko"),
        )
        .unwrap()
    }

    fn query(label: &str, predicate: Predicate) -> NamedQuery {
        NamedQuery {
            label: label.into(),
            predicate,
        }
    }

    #[test]
    fn expectations_count_all_matches_and_bound_witnesses() {
        let model = model("checks {} base {} a: base {} b: base {} c: base {}");
        let engine = QueryEngine::new(&model.program().ir, model.interner());
        let q = query(
            "instances",
            Predicate::node().and(Predicate::inherits("base")),
        );
        let check = engine.check_query(&q, MatchExpectation::Exactly(3), 1);
        assert!(check.passed);
        assert_eq!(check.actual, 3);
        assert_eq!(check.witnesses.len(), 1);
        assert!(check.witnesses_truncated);
        assert!(
            engine
                .query_decl(check.witnesses[0], &q.predicate)
                .is_some()
        );
        assert!(
            engine
                .check_query(&q, MatchExpectation::AtLeast(2), 0)
                .passed
        );
        assert!(
            engine
                .check_query(&q, MatchExpectation::AtMost(3), 0)
                .passed
        );
        assert!(
            !engine
                .check_query(&q, MatchExpectation::AtMost(2), 0)
                .passed
        );
        assert!(
            !engine
                .check_query(&q, MatchExpectation::Exactly(0), 0)
                .passed
        );
    }

    #[test]
    fn a_missing_required_match_fails_without_fake_witnesses() {
        let model = model("checks {} context {}");
        let engine = QueryEngine::new(&model.program().ir, model.interner());
        let q = query("required", Predicate::named("missing"));
        let check = engine.check_query(&q, MatchExpectation::AtLeast(1), 8);
        assert!(!check.passed);
        assert_eq!(check.actual, 0);
        assert!(check.witnesses.is_empty());
        assert!(!check.witnesses_truncated);
        assert!(
            engine
                .check_query(&q, MatchExpectation::Exactly(0), 0)
                .passed
        );
    }

    #[test]
    fn effective_field_checks_respect_defaults_and_local_overrides() {
        let model = model("checks {} cell { blocked 1; } a: cell {} b: cell { blocked 0; }");
        let engine = QueryEngine::new(&model.program().ir, model.interner());
        let cells = Predicate::node().and(Predicate::inherits("cell"));
        let blocked = cells.clone().and(Predicate::EffectiveChildValue(
            "blocked".into(),
            ValuePredicate::NumEq(1.0),
        ));
        let check = engine.check_query(&query("blocked", blocked), MatchExpectation::Exactly(1), 8);
        assert!(check.passed);
        assert_eq!(
            engine
                .query_decl(check.witnesses[0], &Predicate::named("a"))
                .unwrap()
                .name,
            "a"
        );
        let direct = cells.and(Predicate::ChildValue(
            "blocked".into(),
            ValuePredicate::NumEq(1.0),
        ));
        assert!(
            engine
                .check_query(&query("direct", direct), MatchExpectation::Exactly(0), 8)
                .passed
        );
    }

    #[test]
    fn neutral_and_any_sign_reference_patterns_are_distinct() {
        let model = model(
            "checks {} a {} b {} @plus { (+ a, + b); } @minus { (- a, - b); } @neutral { (~ a, ~ b); }",
        );
        let engine = QueryEngine::new(&model.program().ir, model.interner());
        for pred in [
            Predicate::HasPlusRef(Box::new(Predicate::Any)),
            Predicate::HasMinusRef(Box::new(Predicate::Any)),
            Predicate::HasNeutralRef(Box::new(Predicate::Any)),
        ] {
            assert_eq!(engine.query(&Predicate::edge().and(pred)).len(), 1);
        }
        assert_eq!(
            engine
                .query(&Predicate::edge().and(Predicate::HasRef(Box::new(Predicate::Any))))
                .len(),
            3
        );
    }
}
