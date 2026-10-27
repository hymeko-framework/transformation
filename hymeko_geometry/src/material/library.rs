//! Portable asset data, not a second scene model. Scene identity stays in Core IR.
use super::{PbrMaterial, procedural::SurfaceRecipe};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, sync::OnceLock};

/// A measured or explicitly assumed scalar. The consuming domain validates dimensions,
/// admissible values and applicability; visual roughness never implies contact friction.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PhysicalProperty {
    pub value: f64,
    pub unit: String,
    pub source: String,
    /// E.g. temperature, moisture, direction, mating surface, test method.
    #[serde(default)]
    pub conditions: BTreeMap<String, String>,
}

/// An asset recipe with physical tile scale and explicitly optional physical metadata.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SurfaceMaterial {
    pub label: String,
    pub source: String,
    pub tile_m: f32,
    pub appearance: PbrMaterial,
    #[serde(default)]
    pub procedural: Option<SurfaceRecipe>,
    /// Namespaced keys, e.g. `mechanics.density` or `thermal.conductivity`.
    /// Absence means unknown, never zero or a guessed default.
    #[serde(default)]
    pub physical: BTreeMap<String, PhysicalProperty>,
}

impl SurfaceMaterial {
    pub fn validate(&self) -> Result<(), String> {
        if self.label.trim().is_empty() || self.source.trim().is_empty() {
            return Err("material label and source must be nonempty".into());
        }
        if !self.tile_m.is_finite() || !(0.001..=100.).contains(&self.tile_m) {
            return Err("material tile must be 0.001..100 metres".into());
        }
        self.appearance.validate()?;
        if let Some(recipe) = &self.procedural {
            recipe.validate()?;
            if self.appearance.textures.iter().any(Option::is_some) {
                return Err("procedural material must not also supply baked textures".into());
            }
        }
        for (key, property) in &self.physical {
            if !key.contains('.') || key.split('.').any(|s| !identifier(s)) {
                return Err(format!("physical property needs a namespaced key: {key}"));
            }
            if !property.value.is_finite()
                || property.unit.trim().is_empty()
                || property.source.trim().is_empty()
                || property
                    .conditions
                    .iter()
                    .any(|(k, v)| k.trim().is_empty() || v.trim().is_empty())
            {
                return Err(format!("invalid physical property {key}"));
            }
        }
        Ok(())
    }

    /// Bake once at asset load, then share the returned material between instances.
    /// Call `SurfaceRecipe::bake` directly when the metre height field is also needed.
    pub fn bake(&self, size: u32) -> Result<PbrMaterial, String> {
        self.validate()?;
        let mut appearance = self.appearance.clone();
        if let Some(recipe) = &self.procedural {
            let baked =
                recipe.bake(size, self.tile_m, appearance.roughness, appearance.metallic)?;
            appearance.textures = baked.textures();
            // The MR map contains absolute factors, not factors to multiply twice.
            appearance.roughness = 1.;
            appearance.metallic = 1.;
        }
        appearance.validate()?;
        Ok(appearance)
    }
}

/// A versioned collection of domain assets, using Geometry's existing PBR model.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MaterialLibrary {
    pub schema_version: u32,
    pub materials: BTreeMap<String, SurfaceMaterial>,
}

impl MaterialLibrary {
    /// Bounded JSON asset decoding. This is not a HyMeKo source parser.
    pub fn from_json(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() > 4 * 1024 * 1024 {
            return Err("material library exceeds 4 MiB".into());
        }
        let library: Self = serde_json::from_slice(bytes).map_err(|e| e.to_string())?;
        library.validate()?;
        Ok(library)
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != 1 || !(1..=256).contains(&self.materials.len()) {
            return Err("material library requires schema 1 and 1..256 entries".into());
        }
        for (id, material) in &self.materials {
            if !identifier(id) {
                return Err(format!("invalid material identifier {id}"));
            }
            material.validate().map_err(|e| format!("{id}: {e}"))?;
        }
        Ok(())
    }

    /// The shared collection ships with the crate, without a runtime sibling path.
    pub fn builtin() -> &'static Self {
        static LIBRARY: OnceLock<MaterialLibrary> = OnceLock::new();
        LIBRARY.get_or_init(|| {
            Self::from_json(include_bytes!("../../data/materials.json"))
                .expect("built-in material library is validated by provider tests")
        })
    }
}

fn identifier(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 96
        && value
            .bytes()
            .all(|c| c.is_ascii_alphanumeric() || c == b'-' || c == b'_')
}
