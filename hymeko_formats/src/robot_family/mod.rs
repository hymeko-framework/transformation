//! Parameterised robot families: a declared design space and one generation route,
//! `.hymeko` family declaration -> design point -> HyMeKo robot source -> Core compilation.
//!
//! A family is declared in HyMeKo (`data/robotics/families`): its continuous parameters with
//! units, bounds and the default that reproduces its reference robot, and the constants of its
//! mass and structure models. [`ParameterSpace`] reads them through Query from the compiled
//! declaration. A [`RobotFamily`] owns its robot's invariants: how a design point becomes links
//! and joints ([`RobotSource`]) and which static requirements a design must meet. Candidates are
//! compiled by Core and checked by [`check`]; nothing here writes IR tables. Geometry/Formats
//! project the compiled model as for any authored robot.
//!
//! Topology (e.g. the leg count) is a family's structural choice, made when the family value is
//! constructed; parameters only move values, so every design of one family value has the same
//! Core structure hash ([`check::Candidate::structure`]).
use hymeko::{
    common::ids::DeclId,
    generators::{GraphGenerator, SyntheticGenerator},
    ir::ir::ValueR,
    module_store::{execution::CompiledModel, source_provider::StdFsProvider},
    util::real_parser::RealParser,
};
use hymeko_query::{Predicate, QueryEngine};
use serde::Serialize;
use std::{collections::BTreeMap, fmt, path::Path};

pub mod cell;
pub mod check;
pub mod legged;
pub mod manipulator;
pub mod source;

pub use source::{JointKind, Limits, LinkSpec, RobotSource, Shape};

/// Why a family, design or candidate was refused. Every refusal names its cause.
#[derive(Debug, Clone, PartialEq)]
pub enum FamilyError {
    Declaration(String),
    Rejected(String),
    Compile(String),
    Projection(String),
}

impl fmt::Display for FamilyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Declaration(m) => write!(f, "family declaration: {m}"),
            Self::Rejected(m) => write!(f, "design rejected: {m}"),
            Self::Compile(m) => write!(f, "Core compilation: {m}"),
            Self::Projection(m) => write!(f, "projection: {m}"),
        }
    }
}

impl std::error::Error for FamilyError {}

/// Write `files` (path relative to `workspace` -> text) into the new directory `workspace` and
/// compile `root` there with Core. The directory stays as the candidate's source snapshot.
pub fn compile_snapshot(workspace: &Path, files: &BTreeMap<String, String>, root: &str) -> Result<CompiledModel, FamilyError> {
    if workspace.exists() {
        return Err(FamilyError::Declaration(format!("snapshot workspace {} must be new", workspace.display())));
    }
    for (path, text) in files {
        let target = workspace.join(path);
        std::fs::create_dir_all(target.parent().expect("relative file path"))
            .and_then(|_| std::fs::write(&target, text))
            .map_err(|e| FamilyError::Declaration(format!("{}: {e}", target.display())))?;
    }
    CompiledModel::compile(StdFsProvider::new(), RealParser, &workspace.join(root)).map_err(|e| FamilyError::Compile(format!("{e:?}")))
}

/// One declared continuous parameter.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ParameterSpec {
    pub name: String,
    pub unit: String,
    pub lower: f64,
    pub upper: f64,
    pub default: f64,
    /// Admissible values are integer multiples of this step (a manufacturing grid).
    pub resolution: f64,
}

impl ParameterSpec {
    /// The grid value nearest `v`, computed by integer steps so that decimal grids stay exact.
    pub fn snap(&self, v: f64) -> f64 {
        let steps = (1. / self.resolution).round();
        if (steps * self.resolution - 1.).abs() < 1e-12 { (v * steps).round() / steps } else { (v / self.resolution).round() * self.resolution }
    }
    pub fn on_grid(&self, v: f64) -> bool {
        (self.snap(v) - v).abs() <= 1e-9 * self.resolution.max(v.abs() * 1e-6)
    }
}

/// The declared design space of one family: its parameters, in declaration order, and the
/// numeric constants of its declared material/structure blocks (`block.field`).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ParameterSpace {
    pub family: String,
    pub generator: String,
    pub parameters: Vec<ParameterSpec>,
    pub constants: BTreeMap<String, f64>,
    pub lists: BTreeMap<String, Vec<f64>>,
}

/// A design admitted by its space: every declared parameter, finite and within bounds, and
/// nothing else. Only [`ParameterSpace::admit`] constructs one.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct DesignPoint {
    values: BTreeMap<String, f64>,
}

impl DesignPoint {
    pub fn get(&self, name: &str) -> f64 {
        self.values[name]
    }
    pub fn values(&self) -> &BTreeMap<String, f64> {
        &self.values
    }
}

impl ParameterSpace {
    /// Compile a family declaration file and read the named family from it.
    pub fn load(path: &Path, family: &str) -> Result<Self, FamilyError> {
        let model = CompiledModel::compile(StdFsProvider::new(), RealParser, path)
            .map_err(|e| FamilyError::Compile(format!("{e:?}")))?;
        Self::read(&model, family)
    }

    /// Read a family from a compiled declaration through Query.
    pub fn read(model: &CompiledModel, family: &str) -> Result<Self, FamilyError> {
        let query = QueryEngine::new(&model.program().ir, model.interner());
        let declaration = |m: String| FamilyError::Declaration(m);
        let name = |id: DeclId| query.resolver().resolve(query.ir().decl_nodes[id.raw()].name).to_owned();
        let vocabulary = |kind: &str| -> Result<DeclId, FamilyError> {
            let rows = query.query(&Predicate::Named("robot_family".into()));
            let root = rows.first().filter(|_| rows.len() == 1).ok_or_else(|| declaration("robot_family vocabulary absent".into()))?;
            query.own_child(root.id, kind).ok_or_else(|| declaration(format!("vocabulary kind {kind} absent")))
        };
        let is = |id: DeclId, kind: DeclId| query.query_decl(id, &Predicate::node().and(Predicate::InheritsFromId(kind))).is_some();
        let rows = query.query(&Predicate::Named(family.into()));
        let root = match rows.as_slice() {
            [row] => row.id,
            _ => return Err(declaration(format!("family {family} absent or ambiguous"))),
        };
        if !is(root, vocabulary("family")?) {
            return Err(declaration(format!("{family} is not a robot_family.family")));
        }
        let number = |id: DeclId, field: &str| match query.effective_value(id, field) {
            Some(ValueR::Num(v)) if v.is_finite() => Ok(*v),
            _ => Err(declaration(format!("{} needs a finite number {field}", name(id)))),
        };
        let text = |id: DeclId, field: &str| match query.effective_value(id, field) {
            Some(ValueR::Str(s)) => Ok(query.resolver().resolve(*s).to_owned()),
            _ => Err(declaration(format!("{} needs a string {field}", name(id)))),
        };
        let (parameter, material, structure) = (vocabulary("parameter")?, vocabulary("material")?, vocabulary("structure")?);
        let mut parameters = Vec::new();
        let mut constants = BTreeMap::new();
        let mut lists = BTreeMap::new();
        for child in query.ir().decl_children(root) {
            let label = name(child);
            if is(child, parameter) {
                let spec = ParameterSpec {
                    name: label.clone(),
                    unit: text(child, "unit")?,
                    lower: number(child, "lower")?,
                    upper: number(child, "upper")?,
                    default: number(child, "default")?,
                    resolution: number(child, "resolution")?,
                };
                if !(spec.lower < spec.upper && (spec.lower..=spec.upper).contains(&spec.default)) {
                    return Err(declaration(format!("parameter {label} needs lower < upper and a default between them")));
                }
                if !(spec.resolution > 0.) || [spec.lower, spec.upper, spec.default].iter().any(|v| !spec.on_grid(*v)) {
                    return Err(declaration(format!("parameter {label} needs a positive resolution its bounds and default lie on")));
                }
                parameters.push(spec);
            } else if is(child, material) || is(child, structure) {
                for field in query.ir().decl_children(child) {
                    constants.insert(format!("{label}.{}", name(field)), number(child, &name(field))?);
                }
            } else if let Some(ValueR::List(items)) = query.effective_value(root, &label) {
                let numbers: Option<Vec<f64>> = items.iter().map(|v| if let ValueR::Num(x) = v { Some(*x) } else { None }).collect();
                if let Some(numbers) = numbers {
                    lists.insert(label, numbers);
                }
            }
        }
        if parameters.is_empty() {
            return Err(declaration(format!("family {family} declares no parameter")));
        }
        Ok(Self { family: family.to_owned(), generator: text(root, "generator")?, parameters, constants, lists })
    }

    pub fn constant(&self, key: &str) -> Result<f64, FamilyError> {
        self.constants.get(key).copied().ok_or_else(|| FamilyError::Declaration(format!("constant {key} absent")))
    }

    pub fn defaults(&self) -> DesignPoint {
        DesignPoint { values: self.parameters.iter().map(|p| (p.name.clone(), p.default)).collect() }
    }

    /// Admit a proposal: every declared parameter present, finite and within its bounds, no
    /// undeclared name. Nothing is clipped; an out-of-range proposal is refused.
    pub fn admit(&self, values: &BTreeMap<String, f64>) -> Result<DesignPoint, FamilyError> {
        if let Some(unknown) = values.keys().find(|k| !self.parameters.iter().any(|p| &p.name == *k)) {
            return Err(FamilyError::Rejected(format!("undeclared parameter {unknown}")));
        }
        for p in &self.parameters {
            let v = *values.get(&p.name).ok_or_else(|| FamilyError::Rejected(format!("missing parameter {}", p.name)))?;
            if !v.is_finite() || v < p.lower || v > p.upper {
                return Err(FamilyError::Rejected(format!("{} = {v} {} outside [{}, {}]", p.name, p.unit, p.lower, p.upper)));
            }
            if !p.on_grid(v) {
                return Err(FamilyError::Rejected(format!("{} = {v} is not on its {} {} grid", p.name, p.resolution, p.unit)));
            }
        }
        Ok(DesignPoint { values: values.clone() })
    }

    /// The grid design nearest unit-box coordinates `u` (declaration order); `u` must lie in `[0, 1]`.
    pub fn from_unit(&self, u: &[f64]) -> Result<DesignPoint, FamilyError> {
        if u.len() != self.parameters.len() || u.iter().any(|x| !(0. ..=1.).contains(x)) {
            return Err(FamilyError::Rejected("unit coordinates must lie in [0, 1], one per parameter".into()));
        }
        let values = self.parameters.iter().zip(u).map(|(p, x)| (p.name.clone(), p.snap(p.lower + x * (p.upper - p.lower)).clamp(p.lower, p.upper))).collect();
        self.admit(&values)
    }

    pub fn to_unit(&self, design: &DesignPoint) -> Vec<f64> {
        self.parameters.iter().map(|p| (design.get(&p.name) - p.lower) / (p.upper - p.lower)).collect()
    }
}

/// Static plausibility of a design under the family's declared structure model.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Requirement {
    pub name: String,
    pub value: f64,
    pub bound: f64,
    pub unit: String,
    pub passed: bool,
}

/// What a family promises about every robot it emits; [`check`] verifies it on the compiled model.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Expectation {
    pub root: String,
    pub links: Vec<String>,
    pub movable: Vec<String>,
    /// Expected unit axis of each movable joint, in its joint frame.
    pub axes: BTreeMap<String, [f64; 3]>,
    /// Link whose world position must be strictly above (`+z`) this other link at the zero pose.
    pub above: Vec<(String, String)>,
    /// Link whose world position must be on the `+y` side of its mirror partner at the zero pose.
    pub left_of: Vec<(String, String)>,
    /// Link whose world position must be ahead (`+x`) of another at the zero pose.
    pub ahead_of: Vec<(String, String)>,
}

/// One robot family: its declared space and the invariants of the robots it emits.
pub trait RobotFamily {
    fn space(&self) -> &ParameterSpace;
    /// File name (and model root label) of the emitted robot module.
    fn module(&self) -> String;
    /// The robot module source for an admitted design. Deterministic in the design.
    fn emit(&self, design: &DesignPoint) -> Result<String, FamilyError>;
    fn expectation(&self) -> Expectation;
    /// Static requirements of the family's structure model; a failed one rejects the design.
    fn requirements(&self, design: &DesignPoint) -> Result<Vec<Requirement>, FamilyError>;
    /// The link masses the family's mass model assigns, with the primitive each is modelled as.
    fn link_specs(&self, design: &DesignPoint) -> Result<Vec<LinkSpec>, FamilyError>;
}

/// A generated robot: the design, its source and the family it came from.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct GeneratedRobot {
    pub family: String,
    pub module: String,
    pub design: DesignPoint,
    pub source: String,
}

/// Generate one fixed design ([`GraphGenerator`]).
pub struct FamilyGenerator<'f, F: RobotFamily> {
    pub family: &'f F,
    pub design: DesignPoint,
}

impl<F: RobotFamily> GraphGenerator for FamilyGenerator<'_, F> {
    type Output = GeneratedRobot;
    type Error = FamilyError;
    fn generate(&self) -> Result<GeneratedRobot, FamilyError> {
        let design = self.family.space().admit(self.design.values())?;
        Ok(GeneratedRobot {
            family: self.family.space().family.clone(),
            module: self.family.module(),
            source: self.family.emit(&design)?,
            design,
        })
    }
}

/// Reproducible uniform sampling of a family's declared box ([`SyntheticGenerator`]): the same
/// seed gives the same design. SplitMix64; no implicit entropy.
pub struct FamilySampler<'f, F: RobotFamily> {
    pub family: &'f F,
    seed: u64,
}

impl<'f, F: RobotFamily> FamilySampler<'f, F> {
    pub fn new(family: &'f F, seed: u64) -> Self {
        Self { family, seed }
    }
    pub fn design(&self) -> Result<DesignPoint, FamilyError> {
        let mut state = self.seed;
        let u: Vec<f64> = (0..self.family.space().parameters.len())
            .map(|_| {
                state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
                let mut z = state;
                z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
                z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
                ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
            })
            .collect();
        self.family.space().from_unit(&u)
    }
}

impl<F: RobotFamily> GraphGenerator for FamilySampler<'_, F> {
    type Output = GeneratedRobot;
    type Error = FamilyError;
    fn generate(&self) -> Result<GeneratedRobot, FamilyError> {
        FamilyGenerator { family: self.family, design: self.design()? }.generate()
    }
}

impl<F: RobotFamily> SyntheticGenerator for FamilySampler<'_, F> {
    type Seed = u64;
    fn seed(&self) -> u64 {
        self.seed
    }
    fn reseed(&mut self, seed: u64) {
        self.seed = seed;
    }
}
