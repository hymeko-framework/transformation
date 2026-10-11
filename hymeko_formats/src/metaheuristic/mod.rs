//! Static DE/GA-profile validation and canonical operator identity export.
//! Uses Core compilation/path/digest APIs and Query contracts; executes no optimizer.
//! CLI: cargo run -p hymeko_formats --bin metaheuristic_contract -- INPUT.hymeko NEW_OUTPUT_DIR

use hymeko::{
    common::ids::DeclId,
    ir::{
        canonical_hash::{Canon, DerivedPathIndex, build_path_index},
        ir::ValueR,
        semantic_digest::{SEMANTIC_DIGEST_ALGO_VERSION, SEMANTIC_DIGEST_SCHEMA_VERSION},
    },
    module_store::{execution::CompiledModel, source_provider::MemProvider},
    util::real_parser::RealParser,
};
use hymeko_query::{
    MatchExpectation, NamedQuery, Predicate, QueryCheck, QueryEngine, QueryMatch, ValuePredicate,
};
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fmt::Write as _,
    path::Path,
};

pub type Result<T> = std::result::Result<T, Box<dyn Error>>;
pub mod catalog;
mod profiles;
pub use profiles::{DE_KEY, GA_KEY, HHO_KEY};
pub const MAX_SOURCE_BYTES: u64 = 128 * 1024;
#[cfg(test)]
const SOURCE: &str = include_str!("../../data/metaheuristics/de_rand_1_bin.hymeko");
#[cfg(test)]
const GA_SOURCE: &str =
    include_str!("../../data/metaheuristics/ga_tournament_uniform_reset.hymeko");

pub fn compile(source: &str) -> Result<CompiledModel> {
    if source.len() as u64 > MAX_SOURCE_BYTES {
        return Err("source exceeds 128 KiB cap".into());
    }
    let model = CompiledModel::compile(
        MemProvider::default().with_file("/de.hymeko", source),
        RealParser,
        Path::new("/de.hymeko"),
    )
    .map_err(|error| format!("Core compilation failed: {error:?}"))?;
    if model.program().ir.decl_nodes.len() > 4096 {
        return Err("declaration cap exceeded".into());
    }
    Ok(model)
}

/// Retains the Core model; all identities are borrowed from this snapshot.
struct Contract<'a> {
    model: &'a CompiledModel,
    paths: DerivedPathIndex,
    checks: Vec<QueryCheck>,
}

impl<'a> Contract<'a> {
    fn new(model: &'a CompiledModel) -> Result<Self> {
        Ok(Self {
            model,
            paths: build_path_index(&model.program().ir)?,
            checks: vec![],
        })
    }

    fn path(&self, decl: DeclId) -> Result<Vec<String>> {
        Ok(self
            .paths
            .path_of(decl)
            .ok_or("missing canonical declaration path")?
            .0
            .iter()
            .map(|symbol| self.model.interner().resolve(*symbol).to_string())
            .collect())
    }

    fn vocabulary(&self, name: &str) -> Result<DeclId> {
        let expected = ["vocabulary", name];
        self.paths
            .iter()
            .find_map(|(path, decl)| {
                path.0
                    .iter()
                    .map(|symbol| self.model.interner().resolve(*symbol))
                    .eq(expected.iter().copied())
                    .then_some(decl)
            })
            .ok_or_else(|| format!("missing exact vocabulary path vocabulary.{name}").into())
    }

    fn check(
        &mut self,
        label: &str,
        predicate: Predicate,
        count: usize,
    ) -> Result<Vec<QueryMatch>> {
        let engine = QueryEngine::new(&self.model.program().ir, self.model.interner());
        let query = NamedQuery {
            label: label.into(),
            predicate,
        };
        let check = engine.check_query(&query, MatchExpectation::Exactly(count), 8);
        if !check.passed {
            return Err(format!("Query contract failed: {check:?}").into());
        }
        self.checks.push(check);
        Ok(engine.query(&query.predicate))
    }

    fn field(&self, decl: DeclId, key: &str) -> Result<String> {
        let engine = QueryEngine::new(&self.model.program().ir, self.model.interner());
        match engine.effective_value(decl, key) {
            Some(ValueR::Str(symbol)) => Ok(self.model.interner().resolve(*symbol).to_string()),
            _ => Err(format!("decl {} lacks string field {key}", decl.raw()).into()),
        }
    }

    fn identity(&self, decl: DeclId) -> Result<Value> {
        Ok(json!({"canonical_path": self.path(decl)?, "snapshot_local_decl_id": decl.raw()}))
    }

    fn require_children(&self, decl: DeclId, allowed: &BTreeSet<DeclId>) -> Result<()> {
        let actual: BTreeSet<_> = self.model.program().ir.decl_children(decl).collect();
        if &actual != allowed {
            return Err(format!(
                "unrepresented child declaration under {:?}",
                self.path(decl)?
            )
            .into());
        }
        Ok(())
    }

    fn require_fields(&self, decl: DeclId, keys: &[&str]) -> Result<BTreeSet<DeclId>> {
        let engine = QueryEngine::new(&self.model.program().ir, self.model.interner());
        keys.iter()
            .map(|key| {
                let field = engine
                    .effective_child(decl, key)
                    .ok_or_else(|| format!("missing field {key}"))?;
                // Local normative fields keep the source/prompt content audit explicit.
                if self.model.program().ir.parent(field) != decl {
                    return Err("normative field must be local".into());
                }
                self.require_children(field, &BTreeSet::new())?;
                Ok(field)
            })
            .collect()
    }

    /// Shared typed dependency-role admission for exact profiles and descriptive projections.
    fn dependency_role(
        &mut self,
        label: &str,
        operator: &Predicate,
        kind: DeclId,
        opposite: DeclId,
    ) -> Result<QueryMatch> {
        let role = self
            .check(
                label,
                Predicate::edge()
                    .and(Predicate::InheritsFromId(kind))
                    .and(Predicate::HasParent(Box::new(operator.clone()))),
                1,
            )?
            .remove(0);
        let engine = QueryEngine::new(&self.model.program().ir, self.model.interner());
        if engine
            .query_decl(role.id, &Predicate::InheritsFromId(opposite))
            .is_some()
        {
            return Err("a dependency cannot be both read and write role types".into());
        }
        let edge = self
            .model
            .program()
            .ir
            .edge_for_decl(role.id)
            .ok_or("role is not an edge")?
            .1;
        if edge.arcs.len() != 1
            || role
                .arc_bindings
                .iter()
                .any(|binding| binding.sign != 0 || binding.weights.is_some())
        {
            return Err("role requires exactly one neutral unweighted dependency hyperarc".into());
        }
        if self.model.program().ir.decl_children(role.id).any(|id| {
            self.model.program().ir.decl_nodes[id.raw()].kind != hymeko::ir::ir::DeclKind::HyperArc
        }) {
            return Err("dependency role has unrepresented metadata".into());
        }
        Ok(role)
    }

    fn validate(mut self) -> Result<Value> {
        let profile_type = self.vocabulary("profile_kind")?;
        let state_type = self.vocabulary("state_kind")?;
        let operator_type = self.vocabulary("operator_kind")?;
        let read_type = self.vocabulary("read_role")?;
        let write_type = self.vocabulary("write_role")?;
        let profiles = Predicate::node().and(Predicate::InheritsFromId(profile_type));
        let profile = self.check("exactly_one_profile", profiles.clone(), 1)?[0].id;
        let definition = profiles::get(&self.field(profile, "key")?)
            .ok_or("unsupported metaheuristic profile")?;
        let mut fields = BTreeMap::new();
        for &(key, expected) in definition.fields {
            self.check(
                &format!("profile.{key}"),
                profiles.clone().and(Predicate::EffectiveChildValue(
                    key.into(),
                    ValuePredicate::StrEq(expected.into()),
                )),
                1,
            )?;
            fields.insert(key, self.field(profile, key)?);
        }
        self.require_children(
            profile,
            &self.require_fields(
                profile,
                &definition
                    .fields
                    .iter()
                    .map(|(key, _)| *key)
                    .collect::<Vec<_>>(),
            )?,
        )?;
        let state_pred = Predicate::node().and(Predicate::InheritsFromId(state_type));
        let expected_states: BTreeSet<_> = definition
            .operators
            .iter()
            .flat_map(|op| op.reads.iter().chain(op.writes))
            .copied()
            .collect();
        let states = self.check("state_count", state_pred.clone(), expected_states.len())?;
        let mut state_by_key = BTreeMap::new();
        let mut state_key_by_decl = BTreeMap::new();
        for state in states {
            let key = self.field(state.id, "key")?;
            if !expected_states.contains(key.as_str())
                || state_by_key.insert(key.clone(), state.id).is_some()
            {
                return Err(format!("unexpected or duplicate state key {key}").into());
            }
            state_key_by_decl.insert(state.id, key);
            self.require_children(state.id, &self.require_fields(state.id, &["key"])?)?;
        }
        let operators = Predicate::edge().and(Predicate::InheritsFromId(operator_type));
        self.check(
            "operator_count",
            operators.clone(),
            definition.operators.len(),
        )?;
        let role_pred = Predicate::edge()
            .and(Predicate::InheritsFromId(read_type).or(Predicate::InheritsFromId(write_type)));
        let roles = self.check("role_count", role_pred, definition.operators.len() * 2)?;
        if roles.len() > 128 {
            return Err("role cap exceeded".into());
        }
        let mut operators_json = BTreeMap::new();
        let mut visited_roles = BTreeSet::new();
        for spec in definition.operators {
            let op_pred = operators.clone().and(Predicate::EffectiveChildValue(
                "key".into(),
                ValuePredicate::StrEq(spec.key.into()),
            ));
            let op = self
                .check(&format!("unique_operator:{}", spec.key), op_pred.clone(), 1)?
                .remove(0);
            if !op.arc_bindings.is_empty() {
                return Err(format!("{} has untyped direct incidences", spec.key).into());
            }
            self.check(
                &format!("rule:{}", spec.key),
                op_pred.clone().and(Predicate::EffectiveChildValue(
                    "rule".into(),
                    ValuePredicate::StrEq(spec.rule.into()),
                )),
                1,
            )?;
            let mut record = self.identity(op.id)?;
            record["rule"] = json!(self.field(op.id, "rule")?);
            let mut op_children = self.require_fields(op.id, &["key", "rule"])?;
            for (role_name, kind, expected) in [
                ("reads", read_type, spec.reads),
                ("writes", write_type, spec.writes),
            ] {
                let opposite = if kind == read_type {
                    write_type
                } else {
                    read_type
                };
                let role = self.dependency_role(
                    &format!("{}:{role_name}", spec.key),
                    &op_pred,
                    kind,
                    opposite,
                )?;
                if !visited_roles.insert(role.id) {
                    return Err("one role assigned to several operators".into());
                }
                op_children.insert(role.id);
                let expected_ids: BTreeSet<_> =
                    expected.iter().map(|key| state_by_key[*key]).collect();
                let actual_ids: BTreeSet<_> = role
                    .arc_bindings
                    .iter()
                    .map(|binding| binding.target)
                    .collect();
                if actual_ids != expected_ids || role.arc_bindings.len() != expected.len() {
                    return Err(format!(
                        "{}:{role_name} has wrong/missing/duplicate canonical dependencies",
                        spec.key
                    )
                    .into());
                }
                let mut bindings = vec![];
                // Named donor state roles carry meaning; slot order is retained as provenance.
                for (slot, binding) in role.arc_bindings.iter().enumerate() {
                    let mut state = self.identity(binding.target)?;
                    state["state_key"] = json!(state_key_by_decl[&binding.target]);
                    state["incidence_slot"] = json!(slot);
                    bindings.push(state);
                }
                record[role_name] = json!({"role": self.identity(role.id)?, "states": bindings});
            }
            self.require_children(op.id, &op_children)?;
            operators_json.insert(spec.key, record);
        }
        if visited_roles.len() != roles.len() {
            return Err("orphan role declaration".into());
        }
        let state_json: BTreeMap<_, _> = state_by_key
            .iter()
            .map(|(key, id)| Ok((key, self.identity(*id)?)))
            .collect::<Result<_>>()?;
        let digest: String = Canon::new(&self.model.program().ir, self.model.interner())
            .hash()
            .0
            .0
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect();
        let checks: Vec<_> = self.checks.iter().map(|check| json!({"label":check.label,"actual":check.actual,"passed":check.passed,
            "witness_decl_ids":check.witnesses.iter().map(|id| id.raw()).collect::<Vec<_>>(),"witnesses_truncated":check.witnesses_truncated})).collect();
        Ok(
            json!({"schema":"hymeko.metaheuristic.operator-contract/1", "profile":definition.key,
            "model_semantic_digest":digest,"digest_schema":SEMANTIC_DIGEST_SCHEMA_VERSION,"digest_algorithm":SEMANTIC_DIGEST_ALGO_VERSION,
            "identity_contract":"(model_semantic_digest, canonical_path); raw DeclId is snapshot-local diagnostic only",
            "role_contract":"typed read/write dependency hyperedges; no direction inferred from sign; memory-slot dependencies, not an executable schedule",
            "scope":"structural conformance to one frozen profile; no implementation-equivalence or efficiency proof",
            "profile_fields":fields,"states":state_json,"operators":operators_json,"checks":checks,
            "prompt_parity":{"P":"same profile fields, operator rules and named read/write dependencies in text",
                "H":"same semantic facts in compiled HyMeKo plus the common text artifact",
                "shared_artifact":"prompt_semantics.txt","guarantee":"semantic-content checklist; token balance and human review still required"}}),
        )
    }
}

pub fn validate(model: &CompiledModel) -> Result<Value> {
    if model.program().ir.decl_nodes.len() > 4096 {
        return Err("declaration cap exceeded".into());
    }
    Contract::new(model)?.validate()
}

pub fn prompt_semantics(contract: &Value) -> Result<String> {
    let mut text = format!(
        "Profile: {}\nStatic dependency contract; no execution/equivalence proof.\n",
        contract["profile"].as_str().ok_or("missing profile")?
    );
    for (key, value) in contract["profile_fields"]
        .as_object()
        .ok_or("missing profile fields")?
    {
        writeln!(text, "{key}: {}", value.as_str().ok_or("non-string field")?)?;
    }
    for (key, record) in contract["operators"]
        .as_object()
        .ok_or("missing operators")?
    {
        writeln!(
            text,
            "\n{key}: {}",
            record["rule"].as_str().ok_or("missing rule")?
        )?;
        for role in ["reads", "writes"] {
            let keys: Vec<_> = record[role]["states"]
                .as_array()
                .ok_or("missing role states")?
                .iter()
                .map(|state| state["state_key"].as_str().ok_or("missing state key"))
                .collect::<std::result::Result<_, _>>()?;
            writeln!(text, "  {role}: {}", keys.join(", "))?;
        }
    }
    Ok(text)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ga_profile_has_its_own_identity_and_roundtrips() {
        let model = compile(GA_SOURCE).unwrap();
        let ga = validate(&model).unwrap();
        assert_eq!(ga["profile"], profiles::GA_KEY);
        assert_eq!(ga["operators"].as_object().unwrap().len(), 8);
        assert_eq!(ga["states"].as_object().unwrap().len(), 22);
        assert!(ga["operators"].get("de.select").is_none());
        assert_ne!(
            ga["model_semantic_digest"],
            check(SOURCE).unwrap()["model_semantic_digest"]
        );
        let emitted =
            hymeko_emitter::emit_hymeko(&model.program().ir, model.interner(), "ga_contract");
        let second = check(&emitted).unwrap();
        assert_eq!(ga["model_semantic_digest"], second["model_semantic_digest"]);
        assert!(
            prompt_semantics(&ga)
                .unwrap()
                .starts_with(&format!("Profile: {}\n", profiles::GA_KEY))
        );
    }

    #[test]
    fn ga_semantic_and_dependency_drift_is_rejected() {
        for (from, to) in [
            (
                "ga-tournament2-uniform-reset-elitist-v1",
                "ga-unknown-profile",
            ),
            ("parents_before_children", "children_before_parents"),
            ("two_distinct_indices_each", "indices_with_replacement"),
            ("reset_probability=1/D", "reset_probability=1/N"),
            (
                "(~ states.population_old, ~ states.parent_indices",
                "(~ states.children, ~ states.parent_indices",
            ),
            (
                "@reads: vocabulary.read_role",
                "@reads: vocabulary.write_role",
            ),
        ] {
            let changed = GA_SOURCE.replacen(from, to, 1);
            assert_ne!(changed, GA_SOURCE, "fixture did not mutate {from}");
            assert!(check(&changed).is_err(), "accepted {from} -> {to}");
        }
    }

    fn check(source: &str) -> Result<Value> {
        validate(&compile(source)?)
    }

    #[test]
    fn real_source_compiles_and_exports_core_canonical_identities() {
        let result = check(SOURCE).unwrap();
        assert_eq!(result["operators"].as_object().unwrap().len(), 10);
        assert_eq!(result["states"].as_object().unwrap().len(), 27);
        assert!(
            result["checks"]
                .as_array()
                .unwrap()
                .iter()
                .all(|check| check["passed"] == true)
        );
        assert_eq!(
            result["operators"]["de.mutate"]["canonical_path"],
            json!(["operators", "mutate"])
        );
        assert_eq!(
            result["operators"]["de.mutate"]["reads"]["states"][2]["state_key"],
            "donor_negative"
        );
        assert!(
            prompt_semantics(&result)
                .unwrap()
                .contains("uniform_lt_CR_or_forced_coordinate")
        );
    }

    #[test]
    fn emitter_roundtrip_preserves_semantic_identity_and_validated_contract() {
        let original = compile(SOURCE).unwrap();
        let emitted =
            hymeko_emitter::emit_hymeko(&original.program().ir, original.interner(), "de_contract");
        let a = validate(&original).unwrap();
        let b = check(&emitted).unwrap();
        assert_eq!(a["model_semantic_digest"], b["model_semantic_digest"]);
        assert_eq!(prompt_semantics(&a).unwrap(), prompt_semantics(&b).unwrap());
    }

    #[test]
    fn missing_and_duplicate_operator_keys_fail() {
        for source in [
            SOURCE.replace("key \"de.mutate\";", "key \"de.mutate_missing\";"),
            SOURCE.replace("key \"de.mutate\";", "key \"de.select\";"),
        ] {
            assert!(check(&source).is_err());
        }
    }

    #[test]
    fn missing_duplicate_or_foreign_donor_references_fail() {
        for donor in [
            "states.donor_positive",
            "states.population_next",
            "vocabulary.state_kind",
        ] {
            let source = SOURCE.replace(
                "~ states.donor_negative, ~ states.differential_weight",
                &format!("~ {donor}, ~ states.differential_weight"),
            );
            assert!(check(&source).is_err(), "{donor}");
        }
        assert!(
            check(&SOURCE.replace(
                "~ states.donor_negative, ~ states.differential_weight",
                "~ states.differential_weight"
            ))
            .is_err()
        );
    }

    #[test]
    fn old_generation_and_explicit_role_direction_are_checked() {
        let source = SOURCE.replace(
            "(~ states.population_old, ~ states.generation",
            "(~ states.population_next, ~ states.generation",
        );
        assert!(check(&source).is_err());
        let source = SOURCE.replacen(
            "@reads: vocabulary.read_role",
            "@reads: vocabulary.write_role",
            1,
        );
        assert!(check(&source).is_err());
    }

    #[test]
    fn same_name_foreign_type_does_not_satisfy_exact_vocabulary_reference() {
        let source = SOURCE.replace(
            "profile: vocabulary.profile_kind",
            "shadow { profile_kind {} }\nprofile: shadow.profile_kind",
        );
        assert!(check(&source).is_err());
    }

    #[test]
    fn frozen_profile_drift_is_rejected() {
        for (from, to) in [
            ("trial_le_parent", "trial_lt_parent"),
            ("deferred_generation", "immediate_generation"),
            (
                "uniform_lt_CR_or_forced_coordinate",
                "uniform_le_CR_or_forced_coordinate",
            ),
            (
                "initialization_included;stop_before_B_plus_one",
                "initialization_free",
            ),
        ] {
            assert!(check(&SOURCE.replace(from, to)).is_err());
        }
    }

    #[test]
    fn sign_changes_do_not_turn_inputs_into_outputs() {
        // The profile requires neutral dependency membership and typed roles.
        let source = SOURCE.replacen(
            "~ states.initial_population",
            "+ states.initial_population",
            1,
        );
        assert!(check(&source).is_err());
    }

    #[test]
    fn source_is_bounded_and_imports_have_no_filesystem_provider() {
        assert!(compile(&" ".repeat(MAX_SOURCE_BYTES as usize + 1)).is_err());
        assert!(compile("de_contract { @\"/external.hymeko\"; }").is_err());
    }

    #[test]
    fn unrepresented_semantic_fields_are_rejected() {
        let source = SOURCE.replace(
            "key \"de.select\";",
            "key \"de.select\"; override_update \"immediate\";",
        );
        assert!(check(&source).is_err());
        let source = SOURCE.replace(
            "early_stop \"none\";",
            "early_stop \"none\"; hidden_stop \"target\";",
        );
        assert!(check(&source).is_err());
    }

    #[test]
    fn canonical_identity_does_not_alias_when_local_declaration_order_changes() {
        let start = SOURCE.find("    @initialize:").unwrap();
        let middle = SOURCE.find("    @prepare_generation:").unwrap();
        let end = SOURCE.find("    @donor_selection:").unwrap();
        let swapped = format!(
            "{}{}{}{}",
            &SOURCE[..start],
            &SOURCE[middle..end],
            &SOURCE[start..middle],
            &SOURCE[end..]
        );
        let original = check(SOURCE).unwrap();
        let reordered = check(&swapped).unwrap();
        assert_eq!(
            original["model_semantic_digest"],
            reordered["model_semantic_digest"]
        );
        let a = &original["operators"]["de.initialize"];
        let b = &reordered["operators"]["de.initialize"];
        assert_eq!(a["canonical_path"], b["canonical_path"]);
        assert_ne!(a["snapshot_local_decl_id"], b["snapshot_local_decl_id"]);
    }
}
