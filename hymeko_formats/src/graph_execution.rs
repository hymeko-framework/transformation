//! Compiled graph resources and inherited PageRank execution profiles.
//! Core owns compilation/identities, Query owns fields, Graph owns computation.
//!
//! The `graph_rank PROFILE.hymeko [JOB_PATH]` binary is a thin client. Examples
//! live in `data/graph_execution`: `job` uses inherited defaults, `ordered_job`
//! requests first-encounter range candidates, and `parallel_job` requests four
//! workers. The permuted profile uses `permuted_job`, `permuted_ordered`, and
//! `permuted_parallel`. Ordering is a preparation policy, not an IR rewrite;
//! multi-worker pull execution uses canonical per-node reduction order.
use hymeko::{
    common::ids::{DeclId, NodeId},
    ir::{
        canonical_hash::{DerivedPathIndex, PathIndexError, build_path_index},
        ir::ValueR,
    },
    module_store::{
        execution::CompiledModel, module_store::ModuleLoadError, source_provider::StdFsProvider,
    },
    resolution::interner::Interner,
    util::real_parser::RealParser,
};
use hymeko_graph::{
    PageRank, PageRankConfig, PageRankError, PageRankModel, PageRankOptions, PageRankResult,
    PageRankWorkspace, RankFlow, RankOrdering, RankWeights,
};
use hymeko_query::QueryEngine;
use std::{
    num::NonZeroUsize,
    path::{Path, PathBuf},
};

/// Failure at the existing compiler, schema, hierarchy or graph boundary.
#[derive(Debug)]
pub enum GraphExecutionError {
    /// The named source did not compile.
    Compile {
        path: PathBuf,
        error: Box<ModuleLoadError>,
    },
    /// Canonical paths could not be constructed.
    Hierarchy(PathIndexError),
    /// No declaration exists at the requested exact profile path.
    MissingJob(String),
    /// A profile field is missing, unknown, mistyped or outside its domain.
    Field { name: String, reason: &'static str },
    /// The graph owner rejected preparation or execution.
    Rank(PageRankError),
}
impl std::fmt::Display for GraphExecutionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "graph execution: {self:?}")
    }
}
impl std::error::Error for GraphExecutionError {}
impl From<PageRankError> for GraphExecutionError {
    fn from(error: PageRankError) -> Self {
        Self::Rank(error)
    }
}
impl From<PathIndexError> for GraphExecutionError {
    fn from(error: PathIndexError) -> Self {
        Self::Hierarchy(error)
    }
}
type Result<T> = std::result::Result<T, GraphExecutionError>;

/// Validated interpretation of one authored execution profile.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GraphExecutionSettings {
    /// Graph semantics and range-layout candidate policy.
    pub preparation: PageRankOptions,
    /// Numeric iteration controls, validated by Graph.
    pub iteration: PageRankConfig,
    /// Explicit private-pool worker budget.
    pub workers: NonZeroUsize,
}

/// Retains the profile and graph as separate canonical Core compilations.
/// Profile/default/resource declarations never become ranked graph vertices.
pub struct PageRankSource {
    profile: CompiledModel,
    graph: CompiledModel,
    paths: DerivedPathIndex,
    settings: GraphExecutionSettings,
}
impl PageRankSource {
    fn compile(path: &Path) -> Result<CompiledModel> {
        CompiledModel::compile(StdFsProvider::new(), RealParser, path).map_err(|error| {
            GraphExecutionError::Compile {
                path: path.to_owned(),
                error: Box::new(error),
            }
        })
    }
    /// Load a profile and its referenced graph resource through Core.
    /// The job is an exact dot-separated canonical declaration path.
    /// # Errors
    /// Reports compilation, missing paths, unsupported fields/policies and invalid controls.
    pub fn load(profile_path: &Path, job_path: &str) -> Result<Self> {
        let profile = Self::compile(profile_path)?;
        let reader = ProfileReader::new(&profile);
        let (settings, resource) = reader.read(job_path)?;
        let parent = profile.program().root.0.parent().ok_or_else(|| {
            ProfileReader::invalid("model.path", "profile has no parent directory")
        })?;
        let graph = Self::compile(&parent.join(resource))?;
        let paths = build_path_index(&graph.program().ir)?;
        Ok(Self {
            profile,
            graph,
            paths,
            settings,
        })
    }
    /// The exact compiled profile used to interpret the request.
    pub fn profile(&self) -> &CompiledModel {
        &self.profile
    }
    /// The independently compiled graph, retaining canonical IDs and symbols.
    pub fn graph(&self) -> &CompiledModel {
        &self.graph
    }
    /// Validated execution settings, including inherited values.
    pub fn settings(&self) -> GraphExecutionSettings {
        self.settings
    }
    /// Prepare the existing Graph operator on the retained model.
    /// # Errors
    /// Propagates Graph's topology, routing and weight validation errors.
    pub fn prepare(&self) -> Result<PageRank<'_>> {
        Ok(PageRank::build(
            &self.graph.program().ir,
            self.settings.preparation,
        )?)
    }
    /// Run through the graph owner using caller-owned reusable numeric state.
    /// # Errors
    /// Includes graph validation, worker-pool admission and numerical failures.
    pub fn run<'w>(&self, workspace: &'w mut PageRankWorkspace) -> Result<PageRankResult<'w>> {
        workspace.set_workers(self.settings.workers.get())?;
        Ok(self
            .prepare()?
            .run(self.settings.iteration, workspace, None)?)
    }
    /// Framework-owned canonical path for one output NodeId.
    pub fn node_path(&self, node: NodeId) -> Option<Vec<&str>> {
        let declaration = self.graph.program().ir.nodes.get(node.raw())?.decl;
        Some(
            self.paths
                .path_of(declaration)?
                .0
                .iter()
                .map(|symbol| self.graph.interner().resolve(*symbol))
                .collect(),
        )
    }
}

struct ProfileReader<'a> {
    model: &'a CompiledModel,
    query: QueryEngine<'a, Interner>,
}
impl<'a> ProfileReader<'a> {
    const FIELDS: &'static [&'static str] = &[
        "model",
        "workers",
        "ordering",
        "damping",
        "tolerance",
        "max_iterations",
        "flow",
        "weights",
        "weight_component",
    ];
    fn new(model: &'a CompiledModel) -> Self {
        Self {
            model,
            query: QueryEngine::new(&model.program().ir, model.interner()),
        }
    }
    fn invalid(name: impl Into<String>, reason: &'static str) -> GraphExecutionError {
        GraphExecutionError::Field {
            name: name.into(),
            reason,
        }
    }
    fn value(&self, declaration: DeclId, name: &'static str) -> Result<&ValueR> {
        self.query
            .effective_value(declaration, name)
            .ok_or_else(|| Self::invalid(name, "required field is missing"))
    }
    fn number(&self, declaration: DeclId, name: &'static str) -> Result<f64> {
        match self.value(declaration, name)? {
            ValueR::Num(value) if value.is_finite() => Ok(*value),
            _ => Err(Self::invalid(name, "expected a finite number")),
        }
    }
    fn integer(&self, declaration: DeclId, name: &'static str) -> Result<usize> {
        let value = self.number(declaration, name)?;
        if value < 0.0 || value.fract() != 0.0 || value >= usize::MAX as f64 {
            return Err(Self::invalid(
                name,
                "expected a representable nonnegative integer",
            ));
        }
        Ok(value as usize)
    }
    fn text(&self, declaration: DeclId, name: &'static str) -> Result<&str> {
        match self.value(declaration, name)? {
            ValueR::Str(symbol) => Ok(self.model.interner().resolve(*symbol)),
            _ => Err(Self::invalid(name, "expected a string")),
        }
    }
    fn fields(&self, declaration: DeclId, allowed: &[&str]) -> Result<()> {
        for (name, _) in self.query.effective_fields(declaration) {
            if !allowed.contains(&name) {
                return Err(Self::invalid(name, "unknown effective field"));
            }
        }
        for child in self.model.program().ir.decl_children(declaration) {
            let node = self
                .model
                .program()
                .ir
                .decl_node(child)
                .ok_or_else(|| Self::invalid("declaration", "invalid child binding"))?;
            let name = self.model.interner().resolve(node.name);
            if !allowed.contains(&name) {
                return Err(Self::invalid(name, "unknown child declaration"));
            }
        }
        Ok(())
    }
    fn read(&self, job_path: &str) -> Result<(GraphExecutionSettings, PathBuf)> {
        let paths = build_path_index(&self.model.program().ir)?;
        let symbols: Option<Vec<_>> = job_path
            .split('.')
            .map(|name| self.model.interner().get_id(name))
            .collect();
        let declaration = symbols
            .and_then(|path| paths.decl_of(&path))
            .ok_or_else(|| GraphExecutionError::MissingJob(job_path.into()))?;
        self.fields(declaration, Self::FIELDS)?;
        let resource = match self.value(declaration, "model")? {
            ValueR::Ref(target) => *target,
            _ => {
                return Err(Self::invalid(
                    "model",
                    "expected a graph-resource reference",
                ));
            }
        };
        self.fields(resource, &["path"])?;
        let path = self.text(resource, "path")?;
        if path.is_empty() {
            return Err(Self::invalid("path", "empty graph-resource path"));
        }
        let iteration = PageRankConfig {
            damping: self.number(declaration, "damping")?,
            tolerance: self.number(declaration, "tolerance")?,
            max_iterations: self.integer(declaration, "max_iterations")?,
        };
        iteration.validate()?;
        let workers = NonZeroUsize::new(self.integer(declaration, "workers")?)
            .ok_or_else(|| Self::invalid("workers", "worker count must be positive"))?;
        let flow = match self.text(declaration, "flow")? {
            "undirected" => RankFlow::Undirected,
            "minus_to_plus" => RankFlow::MinusToPlus,
            "in_to_out" => RankFlow::InToOut,
            _ => return Err(Self::invalid("flow", "unknown routing policy")),
        };
        let weights = match self.text(declaration, "weights")? {
            "unit" => RankWeights::Unit,
            "component" => RankWeights::Component(self.integer(declaration, "weight_component")?),
            _ => return Err(Self::invalid("weights", "unknown weight policy")),
        };
        let ordering = match self.text(declaration, "ordering")? {
            "auto" => RankOrdering::Auto,
            "canonical" => RankOrdering::Canonical,
            "first_encounter" => RankOrdering::FirstEncounter,
            _ => return Err(Self::invalid("ordering", "unknown coordinate policy")),
        };
        Ok((
            GraphExecutionSettings {
                preparation: PageRankModel { flow, weights }.with_ordering(ordering),
                iteration,
                workers,
            },
            PathBuf::from(path),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hymeko::module_store::source_provider::MemProvider;

    struct ProfileCase;
    impl ProfileCase {
        fn compile(fields: &str) -> CompiledModel {
            let schema = include_str!("../data/graph_execution/meta_pagerank.hymeko");
            let source = format!(
                "profile_test {{ @\"meta.hymeko\" -> meta; }} \
                 resource: + <isa> meta.graph_source {{ path \"unused.hymeko\"; }} \
                 job: + <isa> meta.pagerank_profile {{ model -> resource; {fields} }}"
            );
            CompiledModel::compile(
                MemProvider::default()
                    .with_file("/meta.hymeko", schema)
                    .with_file("/profile.hymeko", source),
                RealParser,
                Path::new("/profile.hymeko"),
            )
            .unwrap()
        }
    }

    #[test]
    fn inherited_fields_are_validated_by_the_real_consumer() {
        let valid = ProfileCase::compile("workers 2; ordering \"canonical\";");
        let (settings, resource) = ProfileReader::new(&valid).read("job").unwrap();
        assert_eq!(settings.workers.get(), 2);
        assert_eq!(resource, PathBuf::from("unused.hymeko"));
        for fields in [
            "workers 0;",
            "workers 2.5;",
            "workers \"many\";",
            "ordering \"unknown\";",
            "flow \"unknown\";",
            "weights \"unknown\";",
            "weights \"component\"; weight_component -1;",
            "damping 1.0;",
            "tolerance 0;",
            "max_iterations 0;",
            "tolernce 0.01;",
            "unexpected {}",
        ] {
            let model = ProfileCase::compile(fields);
            assert!(ProfileReader::new(&model).read("job").is_err(), "{fields}");
        }
        let routed =
            ProfileCase::compile("flow \"in_to_out\"; weights \"component\"; weight_component 1;");
        let (settings, _) = ProfileReader::new(&routed).read("job").unwrap();
        assert_eq!(settings.preparation.model.flow, RankFlow::InToOut);
        assert_eq!(
            settings.preparation.model.weights,
            RankWeights::Component(1)
        );
    }
}
