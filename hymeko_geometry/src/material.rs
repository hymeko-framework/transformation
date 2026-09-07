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
    /// Accepts at most 2048x2048 tightly packed RGBA8 texels; leaves input unchanged.
    pub fn validate(&self) -> Result<(), String> {
        if !(1..=2048).contains(&self.width)
            || !(1..=2048).contains(&self.height)
            || self.pixels.len() != self.width as usize * self.height as usize * 4
        {
            return Err("texture must contain 1..2048 by 1..2048 RGBA8 texels".into());
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
    pub textures: [Option<TextureMap>; 5],
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
            textures: Default::default(),
        }
    }
}

impl PbrMaterial {
    /// Rejects invalid factors/images without modifying the material or shared images.
    pub fn validate(&self) -> Result<(), String> {
        for value in [self.metallic, self.roughness, self.occlusion_strength] {
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
        rgba.width = 2049;
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
}
