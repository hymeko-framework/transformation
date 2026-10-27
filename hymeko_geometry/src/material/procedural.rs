//! Periodic surface recipes. Heights are metres; RGB is sRGB; MR and normals are linear.
//! No GPU, image codec, random-number package or simulation dependency.
use super::{RgbaImage, TextureMap, TextureWrap};
use serde::{Deserialize, Serialize};
use std::{f64::consts::TAU, sync::Arc};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SurfacePattern {
    Wood,
    Plaster,
    Weave,
    Brushed,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SurfaceRecipe {
    pub pattern: SurfacePattern,
    pub seed: u32,
    pub color_srgb: [f32; 3],
    /// Relief amplitude in metres, independent of texture resolution.
    pub height_m: f32,
    /// Integer count preserves tile periodicity.
    pub cycles: u32,
    pub roughness_variation: f32,
}

/// Continuous periodic sample before quantization. Roughness is a signed offset.
#[derive(Debug, Clone, Copy)]
pub struct SurfaceSample {
    pub color_srgb: [f32; 3],
    pub height_m: f32,
    pub roughness_offset: f32,
}

pub struct BakedSurface {
    pub color: Arc<RgbaImage>,
    pub metal_rough: Arc<RgbaImage>,
    pub normal: Arc<RgbaImage>,
    /// Row-major metre heights, matching the texel centres of all three maps.
    pub height_m: Vec<f32>,
}

impl BakedSurface {
    pub fn textures(&self) -> [Option<TextureMap>; 5] {
        let mut textures = [None, None, None, None, None];
        for (slot, image) in [&self.color, &self.metal_rough, &self.normal]
            .into_iter()
            .enumerate()
        {
            textures[slot] = Some(TextureMap {
                image: image.clone(),
                wrap: [TextureWrap::Repeat; 2],
                mag_linear: true,
                min_linear: true,
                mip_linear: true,
                mipmapped: true,
            });
        }
        textures
    }
}

impl SurfaceRecipe {
    pub fn validate(&self) -> Result<(), String> {
        if self
            .color_srgb
            .iter()
            .any(|v| !v.is_finite() || !(0.0..=1.).contains(v))
            || !self.height_m.is_finite()
            || !(0.0..=0.01).contains(&self.height_m)
            || !(1..=128).contains(&self.cycles)
            || !self.roughness_variation.is_finite()
            || !(0.0..=0.5).contains(&self.roughness_variation)
        {
            return Err("invalid procedural colour, relief (0..0.01 m), cycles (1..128) or roughness variation (0..0.5)".into());
        }
        Ok(())
    }

    /// Evaluate anywhere in a repeating UV domain, including negative coordinates.
    pub fn sample(&self, uv: [f64; 2]) -> Result<SurfaceSample, String> {
        self.validate()?;
        if uv.iter().any(|v| !v.is_finite()) {
            return Err("surface UV must be finite".into());
        }
        Ok(self.evaluate(uv, 1.))
    }

    /// Bounded once-per-asset bake; no per-frame work. Wrapped central differences
    /// use physical texel spacing. Frequencies near Nyquist fade before sampling.
    pub fn bake(
        &self,
        size: u32,
        tile_m: f32,
        roughness: f32,
        metallic: f32,
    ) -> Result<BakedSurface, String> {
        self.validate()?;
        if !(32..=1024).contains(&size)
            || !size.is_power_of_two()
            || !tile_m.is_finite()
            || !(0.001..=100.).contains(&tile_m)
            || [roughness, metallic]
                .iter()
                .any(|v| !v.is_finite() || !(0.0..=1.).contains(v))
        {
            return Err("bake needs power-of-two size 32..1024, tile 0.001..100 m and unit-range MR factors".into());
        }
        let count = (size * size) as usize;
        let mut color = Vec::with_capacity(count * 4);
        let mut mr = Vec::with_capacity(count * 4);
        let mut heights = Vec::with_capacity(count);
        let detail = (2. - 4. * self.cycles as f64 / size as f64).clamp(0., 1.);
        for y in 0..size {
            for x in 0..size {
                let sample = self.evaluate(
                    [
                        (x as f64 + 0.5) / size as f64,
                        (y as f64 + 0.5) / size as f64,
                    ],
                    detail,
                );
                color.extend(sample.color_srgb.map(unorm));
                color.push(255);
                mr.extend([
                    255,
                    unorm(roughness + sample.roughness_offset),
                    unorm(metallic),
                    255,
                ]);
                heights.push(sample.height_m);
            }
        }
        let mut normal = Vec::with_capacity(count * 4);
        let n = size as usize;
        for y in 0..n {
            for x in 0..n {
                let dx = (heights[y * n + (x + 1) % n] - heights[y * n + (x + n - 1) % n])
                    * size as f32
                    / (2. * tile_m);
                let dy = (heights[((y + 1) % n) * n + x] - heights[((y + n - 1) % n) * n + x])
                    * size as f32
                    / (2. * tile_m);
                let length = (dx * dx + dy * dy + 1.).sqrt();
                normal.extend(
                    [-dx / length, -dy / length, 1. / length].map(|v| unorm(v * 0.5 + 0.5)),
                );
                normal.push(255);
            }
        }
        let image = |pixels| {
            Arc::new(RgbaImage {
                width: size,
                height: size,
                pixels,
            })
        };
        Ok(BakedSurface {
            color: image(color),
            metal_rough: image(mr),
            normal: image(normal),
            height_m: heights,
        })
    }

    fn evaluate(&self, uv: [f64; 2], detail: f64) -> SurfaceSample {
        let [u, v] = uv.map(|v| v.rem_euclid(1.));
        let coarse = noise(u, v, self.seed);
        let wave = TAU * self.cycles as f64;
        let (shade, relief, roughness) = match self.pattern {
            SurfacePattern::Wood => {
                let grain = (wave * u + 2.5 * (TAU * v * 2.).sin() + coarse).sin() * detail;
                let board = (u * 4.).floor();
                let across = (u * 4.).fract();
                let end = (v * 2. + board.rem_euclid(2.) * 0.5).fract();
                // Smooth grooves have continuous height, including the periodic boundary.
                let seam = groove(across, 0.014).max(groove(end, 0.009));
                (
                    (0.92 + 0.09 * coarse + 0.045 * grain) * (1. - 0.4 * seam),
                    0.12 * grain - seam,
                    coarse,
                )
            }
            SurfacePattern::Plaster => {
                let scale = (self.cycles / 16).max(1) as f64;
                let fine = noise(u * scale, v * scale, self.seed) * detail;
                (1. + 0.025 * fine, fine, fine)
            }
            SurfacePattern::Weave => {
                let weave = (wave * u).sin() * (wave * v).sin() * detail;
                (0.96 + 0.06 * weave + 0.04 * coarse, weave, 0.5 * coarse)
            }
            SurfacePattern::Brushed => {
                let brush = (wave * u + 0.6 * (TAU * v).sin()).sin() * detail;
                (
                    1. + 0.025 * brush + 0.015 * coarse,
                    brush,
                    0.7 * brush + 0.3 * coarse,
                )
            }
        };
        SurfaceSample {
            color_srgb: self.color_srgb.map(|c| (c * shade as f32).clamp(0., 1.)),
            height_m: self.height_m * relief as f32,
            roughness_offset: self.roughness_variation * roughness as f32,
        }
    }
}

fn unorm(value: f32) -> u8 {
    (value.clamp(0., 1.) * 255.).round() as u8
}
fn groove(t: f64, width: f64) -> f64 {
    let q = (t.min(1. - t) / width).clamp(0., 1.);
    1. - q * q * (3. - 2. * q)
}

// Periodic, smooth value noise. Explicit integer mixing gives a stable seed contract.
fn noise(u: f64, v: f64, seed: u32) -> f64 {
    let hash = |x: u32, y: u32| {
        let mut h = seed ^ x.wrapping_mul(0x9e3779b9) ^ y.wrapping_mul(0x85ebca6b);
        h = (h ^ (h >> 16)).wrapping_mul(0x7feb352d);
        h = (h ^ (h >> 15)).wrapping_mul(0x846ca68b);
        ((h ^ (h >> 16)) as f64 / u32::MAX as f64) * 2. - 1.
    };
    let smooth = |t: f64| t * t * t * (t * (t * 6. - 15.) + 10.);
    let mut sum = 0.;
    let mut total = 0.;
    for octave in 0..5 {
        let band = 2u32 << octave;
        let x = u.rem_euclid(1.) * band as f64;
        let y = v.rem_euclid(1.) * band as f64;
        let ix = x.floor() as u32;
        let iy = y.floor() as u32;
        let tx = smooth(x.fract());
        let ty = smooth(y.fract());
        let a = hash(ix % band, iy % band) * (1. - tx) + hash((ix + 1) % band, iy % band) * tx;
        let b = hash(ix % band, (iy + 1) % band) * (1. - tx)
            + hash((ix + 1) % band, (iy + 1) % band) * tx;
        let weight = 0.55f64.powi(octave);
        sum += (a * (1. - ty) + b * ty) * weight;
        total += weight;
    }
    sum / total
}
