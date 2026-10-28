//! Validated authoring recipes projected from Core IR, never a second canonical model.
use crate::mesh::{
    TriangleMesh,
    fair::{FairOptions, SurfaceFairing},
    normals::NormalSmoothing,
    regularize::{QuadSurface, RegularizeOptions, SurfaceRegularizer},
};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;

#[cfg(feature = "authoring")]
mod source;
#[cfg(feature = "authoring")]
pub use source::{CompiledRecipe, RecipeProject};

/// A stage has one input (the preceding stage), explicit order and owner-defined parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GeometryStage {
    pub name: String,
    pub operation: String,
    #[serde(default)]
    pub parameters: BTreeMap<String, Value>,
}

/// Mesh operators execute here; character stages are admitted by the existing asset adapter.
/// No authored operation registers a shell command or executable.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GeometryRecipe {
    pub schema_version: u32,
    pub length_unit: String,
    #[serde(default = "default_up_axis")]
    pub up_axis: String,
    pub workflow: String,
    pub name: String,
    #[serde(default)]
    pub settings: BTreeMap<String, Value>,
    pub stages: Vec<GeometryStage>,
}
fn default_up_axis() -> String {
    "Y".into()
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StageReport {
    pub name: String,
    pub operation: String,
    pub vertices: usize,
    pub triangles: usize,
    pub maximum_displacement: f64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PipelineResult {
    pub mesh: TriangleMesh,
    pub quad_surface: Option<QuadSurface>,
    pub stages: Vec<StageReport>,
}

#[derive(Deserialize, Serialize)]
#[serde(default, deny_unknown_fields)]
struct Normals {
    iterations: usize,
    crease_degrees: f32,
}
impl Default for Normals {
    fn default() -> Self {
        Self {
            iterations: 8,
            crease_degrees: 65.,
        }
    }
}
#[derive(Deserialize, Serialize)]
#[serde(default, deny_unknown_fields)]
struct Lod {
    levels: usize,
    max_relative_error: f32,
}
impl Default for Lod {
    fn default() -> Self {
        Self {
            levels: 3,
            max_relative_error: 0.005,
        }
    }
}

impl GeometryRecipe {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != 1
            || self.length_unit != "m"
            || !["X", "Y", "Z"].contains(&self.up_axis.as_str())
            || (self.workflow == "character" && self.up_axis != "Y")
            || !["surface", "character"].contains(&self.workflow.as_str())
            || self.name.is_empty()
            || self.name.len() > 128
            || !(1..=32).contains(&self.stages.len())
        {
            return Err("geometry recipe needs schema 1, metre units, a supported workflow and 1..32 stages".into());
        }
        let mut names = std::collections::BTreeSet::new();
        let mut regularized = false;
        for stage in &self.stages {
            if stage.name.is_empty() || !names.insert(&stage.name) || stage.parameters.len() > 128 {
                return Err("geometry stage names must be unique and parameters bounded".into());
            }
            if self.workflow == "surface" {
                match stage.operation.as_str() {
                    "fair" if !regularized => stage.options::<FairOptions>()?.validate()?,
                    "normals" => {
                        let v: Normals = stage.options()?;
                        if v.iterations > 32 || !(5.0..=85.).contains(&v.crease_degrees) {
                            return Err("invalid normal options".into());
                        }
                    }
                    "regularize" if !regularized => {
                        let _: RegularizeOptions = stage.options()?;
                        regularized = true;
                    }
                    "lod" => {
                        let v: Lod = stage.options()?;
                        if !(1..=4).contains(&v.levels)
                            || !(0.0..=0.1).contains(&v.max_relative_error)
                        {
                            return Err("invalid LOD options".into());
                        }
                    }
                    _ => {
                        return Err(format!(
                            "unsupported surface stage/order: {}",
                            stage.operation
                        ));
                    }
                }
            } else if !["head", "neck_shoulders", "validate", "pack"]
                .contains(&stage.operation.as_str())
            {
                return Err(format!("unsupported character stage {}", stage.operation));
            }
        }
        Ok(())
    }

    /// Runs against a private mesh copy. A refused stage never mutates the input.
    pub fn run(
        &self,
        source: &TriangleMesh,
        mut progress: impl FnMut(&StageReport) -> Result<(), String>,
    ) -> Result<PipelineResult, String> {
        self.validate()?;
        source.validate()?;
        if self.workflow != "surface" {
            return Err("character workflow requires the Kawamirai asset adapter".into());
        }
        let mut out = PipelineResult {
            mesh: source.clone(),
            quad_surface: None,
            stages: vec![],
        };
        for stage in &self.stages {
            let displacement = match stage.operation.as_str() {
                "fair" => {
                    SurfaceFairing::apply(&mut out.mesh, &stage.options()?)?.maximum_displacement
                }
                "normals" => {
                    let v: Normals = stage.options()?;
                    NormalSmoothing::apply(&mut out.mesh, v.iterations, v.crease_degrees)?;
                    0.
                }
                "regularize" => {
                    let surface = SurfaceRegularizer::new(&out.mesh)?.prepare(
                        &out.mesh
                            .lods
                            .last()
                            .ok_or("missing control surface")?
                            .indices,
                        &stage.options()?,
                    )?;
                    let delta = surface.maximum_relaxation;
                    out.mesh = surface.mesh.clone();
                    out.quad_surface = Some(surface);
                    delta
                }
                "lod" => {
                    let options: Lod = stage.options()?;
                    #[cfg(feature = "mesh-optimization")]
                    out.mesh
                        .optimize(options.levels, options.max_relative_error)?;
                    #[cfg(not(feature = "mesh-optimization"))]
                    {
                        let _ = options;
                        return Err("LOD execution needs mesh-optimization".into());
                    }
                    #[allow(unreachable_code)]
                    0.
                }
                _ => return Err("unadmitted operation".into()),
            };
            let report = StageReport {
                name: stage.name.clone(),
                operation: stage.operation.clone(),
                vertices: out.mesh.positions.len(),
                triangles: out.mesh.lods[0].indices.len() / 3,
                maximum_displacement: displacement,
            };
            progress(&report)?;
            out.stages.push(report);
        }
        if let Some(quad) = &mut out.quad_surface {
            quad.mesh = out.mesh.clone();
        }
        Ok(out)
    }
}

impl GeometryStage {
    fn options<T: serde::de::DeserializeOwned + Serialize + Default>(&self) -> Result<T, String> {
        let defaults = serde_json::to_value(T::default()).map_err(|e| e.to_string())?;
        if self
            .parameters
            .keys()
            .any(|key| defaults.get(key).is_none())
        {
            return Err(format!(
                "{} contains an unknown operation parameter",
                self.name
            ));
        }
        serde_json::from_value(serde_json::to_value(&self.parameters).map_err(|e| e.to_string())?)
            .map_err(|e| format!("{}: {e}", self.name))
    }
}
