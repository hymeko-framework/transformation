//! Geometry-processing recipes read out of HyMeKo IR. The recipe types and their execution over a
//! mesh are the Geometry repository's `hymeko_mesh::recipe`; this module only projects them.
use hymeko_mesh::recipe::{GeometryRecipe, GeometryStage};
use hymeko::{
    common::ids::DeclId,
    ir::ir::ValueR,
    module_store::{execution::CompiledModel, source_provider::MemProvider},
    util::real_parser::RealParser,
};
use hymeko_query::{Predicate, QueryEngine, traits::NameResolver};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};
use std::{
    collections::BTreeMap,
    path::{Component, Path},
};

/// A bounded virtual source workspace. Core alone resolves its imports.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecipeProject {
    pub entry: String,
    pub files: BTreeMap<String, String>,
}

impl RecipeProject {
    pub fn compile(&self, root: &str) -> Result<CompiledRecipe, String> {
        if self.files.is_empty()
            || self.files.len() > 256
            || self.files.values().map(String::len).sum::<usize>() > 4 * 1024 * 1024
            || !self.files.contains_key(&self.entry)
        {
            return Err("recipe workspace requires an entry and 1..256 files within 4 MiB".into());
        }
        let mut provider = MemProvider::default();
        for (name, source) in &self.files {
            if name.is_empty()
                || name.contains('\\')
                || name.contains(':')
                || !Path::new(name)
                    .components()
                    .all(|p| matches!(p, Component::Normal(_)))
            {
                return Err("recipe files must be relative workspace paths".into());
            }
            provider.insert_file(name, source.as_str());
        }
        let model = CompiledModel::compile(provider, RealParser, Path::new(&self.entry))
            .map_err(|e| format!("recipe compilation: {e:?}"))?;
        let recipe = project(&model, root)?;
        Ok(CompiledRecipe {
            recipe,
            semantic_digest: format!("{:?}", model.semantic_digest()),
        })
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompiledRecipe {
    pub recipe: GeometryRecipe,
    pub semantic_digest: String,
}

/// Read the recipe declared under `root` out of a compiled model.
pub fn project(model: &CompiledModel, root: &str) -> Result<GeometryRecipe, String> {
    let query = QueryEngine::new(&model.program().ir, model.interner());
    let roots = query.query(&Predicate::node().and(Predicate::named(root)));
    if roots.len() != 1 {
        return Err(format!("expected one geometry recipe named {root}"));
    }
    let mut reader = Reader {
        query,
        remaining: 4096,
    };
    let Value::Object(mut object) = reader.declaration(roots[0].id, 0)? else {
        return Err("recipe must be a group".into());
    };
    let stages = object.remove("stages").ok_or("recipe needs stages")?;
    let stages = stages.as_object().ok_or("stages must be named groups")?;
    if stages.len() > 32 {
        return Err("at most 32 geometry stages".into());
    }
    let mut ordered = Vec::new();
    for (name, stage) in stages {
        let mut stage = stage.as_object().ok_or("stage must be a group")?.clone();
        let order = stage
            .remove("order")
            .and_then(|v| v.as_u64())
            .ok_or("stage order must be a nonnegative integer")?;
        stage.insert("name".into(), Value::String(name.clone()));
        let stage: GeometryStage = serde_json::from_value(Value::Object(stage))
            .map_err(|e| format!("stage {name}: {e}"))?;
        ordered.push((order, stage));
    }
    ordered.sort_by_key(|(order, _)| *order);
    if ordered
        .iter()
        .enumerate()
        .any(|(i, (order, _))| i as u64 != *order)
    {
        return Err("stage order must be unique and contiguous from zero".into());
    }
    object.insert("name".into(), Value::String(root.into()));
    object.insert(
        "stages".into(),
        serde_json::to_value(
            ordered
                .into_iter()
                .map(|(_, stage)| stage)
                .collect::<Vec<_>>(),
        )
        .map_err(|e| e.to_string())?,
    );
    let recipe: GeometryRecipe = serde_json::from_value(Value::Object(object))
        .map_err(|e| format!("recipe schema: {e}"))?;
    recipe.validate()?;
    Ok(recipe)
}

struct Reader<'a, R: NameResolver> {
    query: QueryEngine<'a, R>,
    remaining: usize,
}
impl<R: NameResolver> Reader<'_, R> {
    fn declaration(&mut self, id: DeclId, depth: usize) -> Result<Value, String> {
        if depth > 12 || self.remaining == 0 {
            return Err("recipe configuration exceeds depth/field budget".into());
        }
        self.remaining -= 1;
        if let Some(value) = self
            .query
            .ir()
            .decl_annotation(id)
            .and_then(|a| a.value.as_ref())
        {
            return self.value(value, depth + 1);
        }
        let mut object = Map::new();
        for (key, binding) in self.query.effective_children(id) {
            object.insert(
                key.into(),
                self.declaration(binding.declaration, depth + 1)?,
            );
        }
        Ok(Value::Object(object))
    }
    fn value(&mut self, value: &ValueR, depth: usize) -> Result<Value, String> {
        if depth > 12 || self.remaining == 0 {
            return Err("recipe value exceeds depth/field budget".into());
        }
        self.remaining -= 1;
        Ok(match value {
            ValueR::Str(s) => Value::String(self.query.resolver().resolve(*s).into()),
            ValueR::Num(n)
                if n.is_finite() && n.fract() == 0. && n.abs() <= 9_007_199_254_740_991. =>
            {
                Value::from(*n as i64)
            }
            ValueR::Num(n) => serde_json::Number::from_f64(*n)
                .map(Value::Number)
                .ok_or("nonfinite recipe number")?,
            ValueR::List(items) => Value::Array(
                items
                    .iter()
                    .map(|v| self.value(v, depth + 1))
                    .collect::<Result<_, _>>()?,
            ),
            ValueR::Ref(id) => self.declaration(*id, depth + 1)?,
        })
    }
}
