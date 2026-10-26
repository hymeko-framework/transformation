//! Neutral, bounded surface appearance. No renderer or image decoder dependencies.
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RgbaImage {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>,
}

impl RgbaImage {
    /// Maximum decoded axis for bounded LDR material images (64 MiB RGBA8).
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

/// Opaque metallic/roughness workflow. Slots: base colour, metal/rough, normal, AO, emission.
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
        }
    }
}

impl PbrMaterial {
    /// Rejects invalid factors/images without modifying the material or shared images.
    pub fn validate(&self) -> Result<(), String> {
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
        rgba.width = 4097;
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
    fn rgba_accepts_four_k_and_keeps_axis_and_payload_bounds() {
        for (width, height) in [(4096, 1), (1, 4096), (4096, 4096)] {
            let mut image = RgbaImage {
                width,
                height,
                pixels: vec![127; width as usize * height as usize * 4],
            };
            assert!(image.validate().is_ok());
            image.pixels.pop();
            assert!(image.validate().is_err());
        }
        for (width, height) in [(4097, 1), (1, 4097), (0, 1), (u32::MAX, u32::MAX)] {
            assert!(
                RgbaImage {
                    width,
                    height,
                    pixels: vec![]
                }
                .validate()
                .is_err()
            );
        }
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
