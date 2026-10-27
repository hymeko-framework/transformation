//! Neutral, bounded surface appearance. No renderer or image decoder dependencies.
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// Shared authored surface assets and optional nonvisual property metadata.
pub mod library;
/// Deterministic, periodic CPU baking into the existing texture contract.
pub mod procedural;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RgbaImage {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>,
}

impl RgbaImage {
    /// Maximum decoded LDR axis; at most 64 MiB of tightly packed RGBA8.
    pub const MAX_DIMENSION: u32 = 4096;

    /// Accepts at most 4096x4096 tightly packed RGBA8 texels; leaves input unchanged.
    pub fn validate(&self) -> Result<(), String> {
        if !(1..=Self::MAX_DIMENSION).contains(&self.width)
            || !(1..=Self::MAX_DIMENSION).contains(&self.height)
            || self.pixels.len() != self.width as usize * self.height as usize * 4
        {
            return Err("texture must contain 1..4096 by 1..4096 RGBA8 texels".into());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RadianceImage {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<[f32; 4]>,
}

impl RadianceImage {
    /// Linear nonnegative radiance, at most 2048x1024, representable by RGBA16F.
    pub fn validate(&self) -> Result<(), String> {
        if !(1..=2048).contains(&self.width)
            || !(1..=1024).contains(&self.height)
            || self.pixels.len() != self.width as usize * self.height as usize
        {
            return Err("HDR must contain 1..2048 by 1..1024 texels".into());
        }
        if self
            .pixels
            .iter()
            .flatten()
            .any(|v| !v.is_finite() || !(0.0..=65504.0).contains(v))
        {
            return Err("HDR radiance must be finite, nonnegative and fit f16".into());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum TextureWrap {
    Repeat,
    Clamp,
    Mirror,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TextureMap {
    pub image: Arc<RgbaImage>,
    pub wrap: [TextureWrap; 2],
    pub mag_linear: bool,
    pub min_linear: bool,
    pub mip_linear: bool,
    pub mipmapped: bool,
}

/// Optional skin optics. Radii are RGB Gaussian standard deviations in world metres,
/// an artistic visible-surface diffusion profile, not measured tissue coefficients.
/// With this profile present, AO channels are R=occlusion, G=specular multiplier,
/// A=explicit skin mask. White G preserves the legacy uniform specular value.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SkinProfile {
    pub radius: [f32; 3],
    pub strength: f32,
    /// Scale on Beckmann RMS slope: scale * (MR.g * material roughness)^2.
    pub roughness: f32,
    pub ior: f32,
    pub specular: f32,
    /// Multiplier on the existing tangent normal map, only inside the skin mask.
    pub normal_scale: f32,
    /// RGB attenuation lengths in metres for light-space skin transmission.
    /// Authored approximation, separate from the visible-surface diffusion radius.
    pub transmission_distance: [f32; 3],
    pub transmission_strength: f32,
}
impl Default for SkinProfile {
    fn default() -> Self {
        Self {
            radius: [0.0045, 0.002, 0.001],
            strength: 0.9,
            roughness: 1.0,
            ior: 1.4,
            specular: 0.6,
            normal_scale: 1.,
            transmission_distance: [0.003, 0.0012, 0.00065],
            transmission_strength: 0.55,
        }
    }
}
impl SkinProfile {
    pub fn validate(&self) -> Result<(), String> {
        if self
            .radius
            .iter()
            .any(|r| !r.is_finite() || !(0.00001..=0.02).contains(r))
            || !self.strength.is_finite()
            || !(0.0..=1.0).contains(&self.strength)
            || !self.roughness.is_finite()
            || !(0.1..=1.0).contains(&self.roughness)
            || !self.ior.is_finite()
            || !(1.0..=2.0).contains(&self.ior)
            || !self.specular.is_finite()
            || !(0.0..=1.0).contains(&self.specular)
            || !self.normal_scale.is_finite()
            || !(0.0..=4.0).contains(&self.normal_scale)
            || self.transmission_distance.iter().any(|v| !v.is_finite() || !(0.00005..=0.02).contains(v))
            || !self.transmission_strength.is_finite()
            || !(0.0..=1.0).contains(&self.transmission_strength)
        {
            return Err("invalid bounded skin optical profile".into());
        }
        Ok(())
    }
}

/// Opaque metallic/roughness workflow. Slots: base colour, metal/rough, normal, AO, emission.
/// Signed height is packed into the normal texture's alpha: byte128 is zero,
/// 255 is +height_m and 1 is -height_m. Existing normal RGB remains unchanged.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReliefProfile {
    pub height_m: f32,
    pub tile_m: f32,
    pub steps: u32,
}
impl ReliefProfile {
    pub fn validate(&self) -> Result<(), String> {
        if !self.height_m.is_finite() || !(0.0..=0.02).contains(&self.height_m)
            || !self.tile_m.is_finite() || !(0.001..=100.).contains(&self.tile_m)
            || self.height_m > self.tile_m * 0.1 || !(8..=64).contains(&self.steps)
        { return Err("relief requires bounded metre height/tile scale and 8..64 steps".into()); }
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct PbrMaterial {
    pub metallic: f32,
    pub roughness: f32,
    pub emissive: [f32; 3],
    pub normal_scale: f32,
    pub occlusion_strength: f32,
    pub double_sided: bool,
    /// glTF `KHR_materials_clearcoat`: a clear dielectric layer over the base, 0..1 (0: none),
    /// and its own roughness, 0..1. Omitted from serialized output while zero.
    #[serde(skip_serializing_if = "is_zero")]
    pub clearcoat: f32,
    #[serde(skip_serializing_if = "is_zero")]
    pub clearcoat_roughness: f32,
    /// glTF `KHR_materials_sheen`: the linear colour of a fibre lobe at grazing angles, 0..1
    /// (black: none), and its roughness, 0..1. Omitted from serialized output while zero.
    #[serde(skip_serializing_if = "is_black")]
    pub sheen_color: [f32; 3],
    #[serde(skip_serializing_if = "is_zero")]
    pub sheen_roughness: f32,
    pub textures: [Option<TextureMap>; 5],
    /// Opt-in skin shading; opaque compatibility is unchanged when absent.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub skin: Option<SkinProfile>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub relief: Option<ReliefProfile>,
}

fn is_zero(value: &f32) -> bool {
    *value == 0.0
}

fn is_black(colour: &[f32; 3]) -> bool {
    colour.iter().all(|c| *c == 0.0)
}

impl Default for PbrMaterial {
    fn default() -> Self {
        Self {
            metallic: 1.0,
            roughness: 1.0,
            emissive: [0.; 3],
            normal_scale: 1.0,
            occlusion_strength: 1.0,
            double_sided: false,
            clearcoat: 0.0,
            clearcoat_roughness: 0.0,
            sheen_color: [0.0; 3],
            sheen_roughness: 0.0,
            textures: Default::default(),
            skin: None,
            relief: None,
        }
    }
}

impl PbrMaterial {
    /// Rejects invalid factors/images without modifying the material or shared images.
    pub fn validate(&self) -> Result<(), String> {
        if let Some(skin) = &self.skin {
            skin.validate()?;
        }
        if let Some(relief) = &self.relief {
            relief.validate()?;
            if self.textures[2].is_none() { return Err("relief requires a normal/height map".into()); }
        }
        let factors = [
            self.metallic,
            self.roughness,
            self.occlusion_strength,
            self.clearcoat,
            self.clearcoat_roughness,
            self.sheen_roughness,
        ];
        for value in factors.into_iter().chain(self.sheen_color) {
            if !value.is_finite() || !(0.0..=1.0).contains(&value) {
                return Err("PBR factors must be 0..1".into());
            }
        }
        if !self.normal_scale.is_finite()
            || !(0.0..=4.0).contains(&self.normal_scale)
            || self
                .emissive
                .iter()
                .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
        {
            return Err("normal scale must be 0..4 and emissive factors 0..1".into());
        }
        for map in self.textures.iter().flatten() {
            map.image.validate()?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn relief_profile_rejects_invalid_scale_and_missing_height_data() {
        let profile=ReliefProfile {height_m:0.005,tile_m:0.5,steps:24};
        assert!(profile.validate().is_ok());
        for bad in [ReliefProfile {height_m:f32::NAN,..profile},ReliefProfile {tile_m:0.,..profile},ReliefProfile {steps:65,..profile},ReliefProfile {tile_m:0.01,..profile}] {
            assert!(bad.validate().is_err());
        }
        assert!(PbrMaterial {relief:Some(profile),..Default::default()}.validate().is_err());
    }
    #[test]
    fn skin_profile_is_opt_in_bounded_and_round_trips() {
        let old: PbrMaterial = serde_json::from_str("{}").unwrap();
        assert!(old.skin.is_none());
        assert!(!serde_json::to_string(&old).unwrap().contains("\"skin\""));
        let profile = SkinProfile::default();
        assert_eq!(
            serde_json::from_str::<SkinProfile>("{}")
                .unwrap()
                .normal_scale,
            1.
        );
        assert!(profile.validate().is_ok());
        assert_eq!(
            serde_json::from_str::<SkinProfile>(&serde_json::to_string(&profile).unwrap()).unwrap(),
            profile
        );
        for bad in [
            SkinProfile {
                radius: [f32::NAN, 0.001, 0.001],
                ..profile
            },
            SkinProfile {
                radius: [0.1; 3],
                ..profile
            },
            SkinProfile {
                strength: 1.1,
                ..profile
            },
            SkinProfile {
                ior: 0.9,
                ..profile
            },
            SkinProfile {
                roughness: 0.,
                ..profile
            },
            SkinProfile {
                specular: f32::INFINITY,
                ..profile
            },
            SkinProfile {
                normal_scale: f32::NAN,
                ..profile
            },
            SkinProfile {
                normal_scale: 4.01,
                ..profile
            },
            SkinProfile { transmission_distance: [0.; 3], ..profile },
            SkinProfile { transmission_distance: [f32::NAN; 3], ..profile },
            SkinProfile { transmission_strength: 1.01, ..profile },
        ] {
            assert!(
                PbrMaterial {
                    skin: Some(bad),
                    ..Default::default()
                }
                .validate()
                .is_err()
            );
        }
    }
    #[test]
    fn bounds_and_nonfinite_values_are_rejected_without_clamping() {
        let mut hdr = RadianceImage {
            width: 1,
            height: 1,
            pixels: vec![[4., 2., 0., 1.]],
        };
        assert!(hdr.validate().is_ok());
        for value in [f32::NAN, f32::INFINITY, -1., 65505.] {
            hdr.pixels[0][0] = value;
            assert!(hdr.validate().is_err());
        }
        let mut rgba = RgbaImage {
            width: 1,
            height: 1,
            pixels: vec![255; 4],
        };
        assert!(rgba.validate().is_ok());
        rgba.width = RgbaImage::MAX_DIMENSION + 1;
        assert!(rgba.validate().is_err());
        rgba.width = 1;
        rgba.pixels.pop();
        assert!(rgba.validate().is_err());
        let mut material = PbrMaterial::default();
        assert!(material.validate().is_ok());
        material.roughness = f32::NAN;
        assert!(material.validate().is_err());
        material.roughness = 0.;
        material.normal_scale = 4.1;
        assert!(material.validate().is_err());
    }

    #[test]
    fn coat_and_sheen_validate_to_unit_range_and_serialize_only_when_present() {
        let plain = serde_json::to_value(PbrMaterial::default()).unwrap();
        for key in [
            "clearcoat",
            "clearcoat_roughness",
            "sheen_color",
            "sheen_roughness",
        ] {
            assert!(plain.get(key).is_none(), "{key} is written while absent");
        }
        let coated = PbrMaterial {
            clearcoat: 1.0,
            clearcoat_roughness: 0.1,
            sheen_color: [0.5, 0.4, 0.3],
            sheen_roughness: 0.6,
            ..Default::default()
        };
        assert!(coated.validate().is_ok());
        let text = serde_json::to_string(&coated).unwrap();
        let back: PbrMaterial = serde_json::from_str(&text).unwrap();
        assert_eq!(
            (
                back.clearcoat,
                back.clearcoat_roughness,
                back.sheen_color,
                back.sheen_roughness
            ),
            (1.0, 0.1, [0.5, 0.4, 0.3], 0.6)
        );
        for broken in [
            PbrMaterial {
                clearcoat: 1.5,
                ..coated.clone()
            },
            PbrMaterial {
                clearcoat_roughness: f32::NAN,
                ..coated.clone()
            },
            PbrMaterial {
                sheen_color: [0.5, -0.1, 0.3],
                ..coated.clone()
            },
            PbrMaterial {
                sheen_roughness: 2.0,
                ..coated
            },
        ] {
            assert!(broken.validate().is_err());
        }
    }
}
