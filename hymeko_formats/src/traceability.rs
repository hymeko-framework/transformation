//! Validated requirements-trace facade over Query and the registered templates.
//! The generic template registry remains unvalidated; use this facade for the bounded
//! flat/binary trace profile. SysML text extraction is not native-tool conformance.
use hymeko::{common::ids::DeclId, module_store::execution::CompiledModel};
use crate::plugin::TransformConfig;
use hymeko_query::{
    Predicate, QueryEngine,
    traceability::{TraceVocabulary, Traceability},
};
use serde_json::{Value, json};
use std::{collections::BTreeMap, path::Path};

pub mod simulation;

/// Retains the immutable Core snapshot used by both the reference and its views.
pub struct TraceabilityExport<'a> {
    model: &'a CompiledModel,
    scope: String,
    projection: Traceability,
}

impl<'a> TraceabilityExport<'a> {
    pub fn new(model: &'a CompiledModel, scope: &str) -> Result<Self, String> {
        let query = QueryEngine::new(&model.program().ir, model.interner());
        let named = |name: &str| -> Result<DeclId, String> {
            let matches = query.query(&Predicate::Named(name.into()));
            if matches.len() != 1 {
                return Err(format!("expected one declaration named {name}"));
            }
            Ok(matches[0].id)
        };
        let root = named("sysml_trace")?;
        let elements = query
            .own_child(root, "elements")
            .ok_or("missing trace elements vocabulary")?;
        let child = |parent, name| {
            query
                .own_child(parent, name)
                .ok_or_else(|| format!("missing trace vocabulary: {name}"))
        };
        let vocabulary = TraceVocabulary {
            requirement: child(elements, "requirement")?,
            component: child(elements, "block")?,
            test_case: child(elements, "test_case")?,
            satisfies: child(root, "satisfies")?,
            allocated_to: child(root, "allocated_to")?,
            derives: child(root, "derives")?,
            verifies: child(root, "verifies")?,
        };
        let projection = Traceability::read(&query, named(scope)?, &vocabulary)?;
        if projection.entities.is_empty() {
            return Err("empty trace scope".into());
        }
        for name in std::iter::once(scope)
            .chain(projection.entities.iter().map(|r| r.name.as_str()))
            .chain(projection.relations.iter().map(|r| r.name.as_str()))
        {
            if !identifier(name) || name.starts_with("hymeko_") {
                return Err(format!("unsupported trace identifier: {name}"));
            }
        }
        for entity in &projection.entities {
            if entity.kind != "requirement" && entity.text.is_some() {
                return Err(format!(
                    "text is supported on requirements only: {}",
                    entity.name
                ));
            }
            if entity.text.as_deref() == Some("") {
                return Err(format!(
                    "empty text is not a trace attribute: {}",
                    entity.name
                ));
            }
            if entity.text.as_ref().is_some_and(|text| !plain_text(text)) {
                return Err(format!(
                    "unsupported trace text delimiters: {}",
                    entity.name
                ));
            }
        }
        Ok(Self {
            model,
            scope: scope.into(),
            projection,
        })
    }

    pub fn projection(&self) -> &Traceability {
        &self.projection
    }

    /// Full selected entity attributes and named relations; never just counts.
    pub fn reference(&self) -> Value {
        let entities: BTreeMap<_, _> = self
            .projection
            .entities
            .iter()
            .map(|e| {
                (
                    e.name.clone(),
                    json!({"kind":e.kind,"text":e.text,"outcome":e.outcome}),
                )
            })
            .collect();
        let relations: BTreeMap<_, _> = self
            .projection
            .relations
            .iter()
            .map(|r| {
                (
                    r.name.clone(),
                    json!({"kind":r.kind,"source":r.source,"target":r.target}),
                )
            })
            .collect();
        json!({"entities":entities,"relations":relations})
    }

    /// Emit every required view or return an error. No cached-file or alternate-backend fallback.
    pub fn emit(&self, templates: &Path) -> Result<BTreeMap<String, String>, String> {
        let query = QueryEngine::new(&self.model.program().ir, self.model.interner());
        let registry = crate::default_registry();
        let config = TransformConfig::default().with_name(&self.scope);
        [
            ("sysml", "requirements_sysml"),
            ("dot", "requirements_dot"),
            ("csv", "requirements_csv"),
        ]
        .into_iter()
        .map(|(extension, name)| {
            let text = registry
                .render(name, &query, &config, templates)
                .ok_or_else(|| format!("missing required transform: {name}"))??;
            if text.trim().is_empty() {
                return Err(format!("empty required view: {name}"));
            }
            Ok((extension.into(), text))
        })
        .collect()
    }
}

fn identifier(text: &str) -> bool {
    let mut chars = text.chars();
    chars
        .next()
        .is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

fn plain_text(text: &str) -> bool {
    !text.chars().any(char::is_control)
        && !text.contains(['"', '\\', ','])
        && !text.contains("*/")
        && !text.contains("/*")
}
