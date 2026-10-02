//! Static DE-profile validation and canonical operator identity export.
//! Uses Core compilation/path/digest APIs and Query contracts; executes no optimizer.
//! Run: cargo run -p hymeko_formats --example metaheuristic_contract --locked --offline -- INPUT.hymeko NEW_OUTPUT_DIR

use hymeko::{
    common::ids::DeclId,
    ir::{
        canonical_hash::{Canon, DerivedPathIndex, build_path_index},
        ir::ValueR,
        semantic_digest::{
            SEMANTIC_DIGEST_ALGO_VERSION, SEMANTIC_DIGEST_SCHEMA_VERSION,
        },
    },
    module_store::{execution::CompiledModel, source_provider::MemProvider},
    resolution::string_table::StringTable,
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
    fs,
    io::Read,
    path::Path,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const PROFILE: &str = "de-rand-1-bin-clamp-deferred-v1";
const MAX_SOURCE_BYTES: u64 = 128 * 1024;
#[cfg(test)]
const SOURCE: &str = include_str!("../data/metaheuristics/de_rand_1_bin.hymeko");

/// Fixed-profile expectations are a conformance oracle, not another executable IR.
const PROFILE_FIELDS: &[(&str, &str)] = &[
    ("key", PROFILE),
    ("optimization", "minimize"),
    ("arithmetic", "python_binary64"),
    ("dimensions", "D>=1,N>=4,B>=N"),
    ("parameters", "0<F<=2,0<=CR<=1,finite_bounds_lo<hi"),
    ("update", "deferred_generation"),
    ("boundary", "componentwise_clamp_after_crossover"),
    ("crossover", "uniform_lt_CR_or_forced_coordinate"),
    ("acceptance", "trial_le_parent"),
    ("donors", "three_distinct_excluding_target"),
    (
        "randomness",
        "named_decisions;3_ordered_donors;forced_coordinate;D_uniforms_[0,1)",
    ),
    ("budget", "initialization_included;stop_before_B_plus_one"),
    ("early_stop", "none"),
    ("partial_generation", "retain_unprocessed_parents"),
    ("target_order", "ascending_index"),
    (
        "archive",
        "min_all_evaluated;ties_keep_previous_archive_vector",
    ),
    ("result_tie", "first_population_index_at_minimum"),
    ("event_coverage", "objective_evaluations_only"),
];

struct OperatorRule {
    key: &'static str,
    rule: &'static str,
    reads: &'static [&'static str],
    writes: &'static [&'static str],
}

const OPERATORS: &[OperatorRule] = &[
    OperatorRule {
        key: "de.initialize",
        rule: "copy_provided_population;evaluate_each_initial_candidate_once",
        reads: &["initial_population", "objective", "budget_limit"],
        writes: &[
            "population_old",
            "parent_scores",
            "evaluation_count",
            "generation",
            "evaluated_candidate",
            "evaluated_score",
        ],
    },
    OperatorRule {
        key: "de.prepare_generation",
        rule: "copy_old_population_and_scores_to_next;set_target_index_zero",
        reads: &["population_old", "parent_scores"],
        writes: &["population_next", "next_scores", "target_index"],
    },
    OperatorRule {
        key: "de.donor_selection",
        rule: "draw_named_decision_and_resolve_ordered_donors_from_old_population",
        reads: &[
            "population_old",
            "generation",
            "target_index",
            "decision_source",
        ],
        writes: &[
            "donor_base",
            "donor_positive",
            "donor_negative",
            "target_old",
            "forced_coordinate",
            "crossover_uniforms",
        ],
    },
    OperatorRule {
        key: "de.mutate",
        rule: "mutant=donor_base+F*(donor_positive-donor_negative);require_finite",
        reads: &[
            "donor_base",
            "donor_positive",
            "donor_negative",
            "differential_weight",
        ],
        writes: &["mutant"],
    },
    OperatorRule {
        key: "de.crossover_repair",
        rule: "trial[j]=clamp(mutant[j])_if_j_forced_or_u[j]<CR_else_target_old[j]",
        reads: &[
            "mutant",
            "target_old",
            "forced_coordinate",
            "crossover_uniforms",
            "crossover_probability",
            "bounds",
        ],
        writes: &["trial"],
    },
    OperatorRule {
        key: "de.evaluate_trial",
        rule: "require_count<B;count_objective_call;require_finite_result",
        reads: &["trial", "objective", "budget_limit", "evaluation_count"],
        writes: &[
            "trial_score",
            "evaluation_count",
            "evaluated_candidate",
            "evaluated_score",
        ],
    },
    OperatorRule {
        key: "de.select",
        rule: "replace_next_target_and_score_iff_trial_score<=parent_score;old_population_unchanged;advance_target",
        reads: &[
            "trial",
            "trial_score",
            "target_index",
            "parent_scores",
            "population_next",
            "next_scores",
        ],
        writes: &["population_next", "next_scores", "target_index"],
    },
    OperatorRule {
        key: "de.archive",
        rule: "after_each_evaluation_keep_min;equal_value_keeps_previous_vector",
        reads: &["evaluated_candidate", "evaluated_score", "archive_best"],
        writes: &["archive_best"],
    },
    OperatorRule {
        key: "de.publish",
        rule: "after_target_loop_publish_next_snapshot;retain_unprocessed_parents;increment_generation_only_if_complete",
        reads: &[
            "population_next",
            "next_scores",
            "target_index",
            "population_old",
            "parent_scores",
            "generation",
        ],
        writes: &["population_old", "parent_scores", "generation"],
    },
    OperatorRule {
        key: "de.stop",
        rule: "continue_iff_count<B;no_target_early_stop;return_first_population_index_at_minimum",
        reads: &[
            "evaluation_count",
            "budget_limit",
            "population_old",
            "parent_scores",
        ],
        writes: &["run_status"],
    },
];

fn compile(source: &str) -> Result<CompiledModel> {
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

    fn validate(mut self) -> Result<Value> {
        let profile_type = self.vocabulary("profile_kind")?;
        let state_type = self.vocabulary("state_kind")?;
        let operator_type = self.vocabulary("operator_kind")?;
        let read_type = self.vocabulary("read_role")?;
        let write_type = self.vocabulary("write_role")?;
        let profiles = Predicate::node().and(Predicate::InheritsFromId(profile_type));
        let profile = self.check("exactly_one_profile", profiles.clone(), 1)?[0].id;
        let mut fields = BTreeMap::new();
        for &(key, expected) in PROFILE_FIELDS {
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
                &PROFILE_FIELDS
                    .iter()
                    .map(|(key, _)| *key)
                    .collect::<Vec<_>>(),
            )?,
        )?;
        let state_pred = Predicate::node().and(Predicate::InheritsFromId(state_type));
        let expected_states: BTreeSet<_> = OPERATORS
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
        self.check("operator_count", operators.clone(), OPERATORS.len())?;
        let role_pred = Predicate::edge()
            .and(Predicate::InheritsFromId(read_type).or(Predicate::InheritsFromId(write_type)));
        let roles = self.check("role_count", role_pred, OPERATORS.len() * 2)?;
        if roles.len() > 128 {
            return Err("role cap exceeded".into());
        }
        let mut operators_json = BTreeMap::new();
        let mut visited_roles = BTreeSet::new();
        for spec in OPERATORS {
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
                let role = self
                    .check(
                        &format!("{}:{role_name}", spec.key),
                        Predicate::edge()
                            .and(Predicate::InheritsFromId(kind))
                            .and(Predicate::HasParent(Box::new(op_pred.clone()))),
                        1,
                    )?
                    .remove(0);
                if !visited_roles.insert(role.id) {
                    return Err("one role assigned to several operators".into());
                }
                op_children.insert(role.id);
                // Reject mixed roles explicitly: sign never supplies input/output meaning.
                let engine = QueryEngine::new(&self.model.program().ir, self.model.interner());
                let opposite = if kind == read_type {
                    write_type
                } else {
                    read_type
                };
                if engine
                    .query_decl(role.id, &Predicate::InheritsFromId(opposite))
                    .is_some()
                {
                    return Err("a dependency cannot be both read and write role types".into());
                }
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
                    return Err(
                        "role requires exactly one neutral unweighted dependency hyperarc".into(),
                    );
                }
                if self.model.program().ir.decl_children(role.id).any(|id| {
                    self.model.program().ir.decl_nodes[id.raw()].kind
                        != hymeko::ir::ir::DeclKind::HyperArc
                }) {
                    return Err("dependency role has unrepresented metadata".into());
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
        let digest: String =
            Canon::new(&self.model.program().ir, self.model.interner()).hash()
                .0
                .0
                .iter()
                .map(|b| format!("{b:02x}"))
                .collect();
        let checks: Vec<_> = self.checks.iter().map(|check| json!({"label":check.label,"actual":check.actual,"passed":check.passed,
            "witness_decl_ids":check.witnesses.iter().map(|id| id.raw()).collect::<Vec<_>>(),"witnesses_truncated":check.witnesses_truncated})).collect();
        Ok(
            json!({"schema":"hymeko.metaheuristic.operator-contract/1", "profile":PROFILE,
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

fn validate(model: &CompiledModel) -> Result<Value> {
    Contract::new(model)?.validate()
}

fn prompt_semantics(contract: &Value) -> Result<String> {
    let mut text = format!(
        "Profile: {PROFILE}\nStatic dependency contract; no execution/equivalence proof.\n"
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

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if args.len() != 2 {
        return Err("usage: metaheuristic_contract INPUT.hymeko NEW_OUTPUT_DIR".into());
    }
    let mut source = String::new();
    fs::File::open(&args[0])?
        .take(MAX_SOURCE_BYTES + 1)
        .read_to_string(&mut source)?;
    let original = compile(&source)?;
    let original_contract = validate(&original)?;
    let canonical =
        hymeko_emitter::emit_hymeko(&original.program().ir, original.interner(), "de_contract");
    let roundtrip = compile(&canonical)?;
    let contract = validate(&roundtrip)?;
    if original_contract["model_semantic_digest"] != contract["model_semantic_digest"] {
        return Err("canonical emission changed the semantic digest".into());
    }
    let output = Path::new(&args[1]);
    fs::create_dir(output)?; // never overwrite an existing result directory
    fs::write(output.join("contract.hymeko"), canonical)?;
    fs::write(
        output.join("operator_ids.json"),
        serde_json::to_vec_pretty(&contract)?,
    )?;
    fs::write(
        output.join("prompt_semantics.txt"),
        prompt_semantics(&contract)?,
    )?;
    fs::write(
        output.join("contract.dot"),
        hymeko_formats::snapshot::emit_dot_graph(
            &roundtrip.program().ir,
            &StringTable::from_interner(roundtrip.interner()),
            "DE static dependency contract",
        ),
    )?;
    println!(
        "profile={PROFILE} operators={} states={} checks={} digest={} output={}",
        contract["operators"].as_object().unwrap().len(),
        contract["states"].as_object().unwrap().len(),
        contract["checks"].as_array().unwrap().len(),
        contract["model_semantic_digest"],
        output.display()
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

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
