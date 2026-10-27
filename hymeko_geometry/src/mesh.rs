//! Validated indexed surfaces. Visual LODs never replace physical colliders.
use serde::{Deserialize, Serialize};

/// Native, constrained quad-surface preparation; independent of a renderer.
pub mod regularize;
/// Seam-aware smoothing of the existing indexed normal field.
pub mod normals;
/// Bounded fairing of shared-position vertices with protected landmarks.
pub mod fair;
/// Attribute-preserving triangulation between existing surface boundaries.
pub mod bridge;
/// Smooth Catmull--Clark limit samples on a quad control cage.
pub mod subdivision;
/// Bounded, transverse union of a closed surface with a convex addition.
pub mod solid;
mod topology;
mod pairing;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MeshLod {
    pub indices: Vec<u32>,
    /// Meshoptimizer's measured error in local position units, not a Hausdorff certificate.
    pub error: f32,
}

impl MeshLod {
    fn validate(&self, n: usize, previous: (usize, f32)) -> Result<(), String> {
        if self.indices.len() < 3
            || !self.indices.len().is_multiple_of(3)
            || self.indices.len() > previous.0
            || self.indices.iter().any(|&i| i as usize >= n)
            || !self.error.is_finite()
            || self.error < previous.1
        {
            return Err("invalid mesh indices or nonmonotone LODs".into());
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TriangleMesh {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub lods: Vec<MeshLod>,
    #[serde(default)]
    pub uv: Vec<[f32; 2]>,
    #[serde(default)]
    pub tangents: Vec<[f32; 4]>,
}

impl TriangleMesh {
    pub fn validate(&self) -> Result<(), String> {
        let n = self.positions.len();
        self.validate_attributes(n)?;
        if !(3..=1_000_000).contains(&n) || self.normals.len() != n {
            return Err("mesh requires 3..1000000 vertices and one normal per vertex".into());
        }
        if self
            .positions
            .iter()
            .flatten()
            .chain(self.normals.iter().flatten())
            .any(|v| !v.is_finite())
        {
            return Err("mesh contains nonfinite coordinates".into());
        }
        if self
            .normals
            .iter()
            .any(|v| v.iter().map(|x| x * x).sum::<f32>() < 1e-12)
        {
            return Err("mesh contains a zero normal".into());
        }
        if self.lods.is_empty() || self.lods.len() > 4 || self.lods[0].error != 0.0 {
            return Err("mesh requires 1..4 LODs with an exact LOD0".into());
        }
        let mut previous = (6_000_000, 0.0);
        for lod in &self.lods {
            lod.validate(n, previous)?;
            previous = (lod.indices.len(), lod.error);
        }
        Ok(())
    }

    fn validate_attributes(&self, n: usize) -> Result<(), String> {
        if !self.uv.is_empty() && self.uv.len() != n
            || !self.tangents.is_empty() && self.tangents.len() != n
        {
            return Err("UV/tangent count must match vertices".into());
        }
        if self
            .uv
            .iter()
            .flatten()
            .chain(self.tangents.iter().flatten())
            .any(|v| !v.is_finite())
        {
            return Err("nonfinite UV or tangent".into());
        }
        if self
            .tangents
            .iter()
            .any(|t| t[3].abs() != 1.0 || t[..3].iter().map(|v| v * v).sum::<f32>() < 1e-12)
        {
            return Err("invalid tangent direction or handedness".into());
        }
        Ok(())
    }

    /// Bounds are derived from the surface, never trusted from an external file.
    /// Requires a nonempty mesh accepted by `validate`.
    pub fn bounds(&self) -> ([f32; 3], [f32; 3]) {
        let mut lo = [f32::INFINITY; 3];
        let mut hi = [f32::NEG_INFINITY; 3];
        for p in &self.positions {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        (lo, hi)
    }

    /// Generate deterministic index-only LODs, retaining vertices and material boundaries.
    #[cfg(feature = "mesh-optimization")]
    pub fn optimize(&mut self, levels: usize, max_relative_error: f32) -> Result<(), String> {
        self.validate()?;
        if !(1..=4).contains(&levels)
            || !max_relative_error.is_finite()
            || !(0.0..=0.1).contains(&max_relative_error)
        {
            return Err("LOD levels must be 1..4 and relative error 0..0.1".into());
        }
        self.lods.truncate(1);
        self.lods[0].indices =
            meshopt::optimize_vertex_cache(&self.lods[0].indices, self.positions.len());
        // DecodePosition avoids casting arbitrary user buffers or duplicating numerical kernels.
        let vertices: Vec<meshopt::Vertex> = self
            .positions
            .iter()
            .zip(&self.normals)
            .map(|(&p, &n)| meshopt::Vertex { p, n, t: [0.0; 2] })
            .collect();
        let scale = meshopt::simplify_scale_decoder(&vertices);
        for level in 1..levels {
            let mut error = 0.0;
            let indices = meshopt::simplify_decoder(
                &self.lods[0].indices,
                &vertices,
                (self.lods[0].indices.len() / (1 << level) / 3).max(1) * 3,
                max_relative_error,
                meshopt::SimplifyOptions::LockBorder,
                Some(&mut error),
            );
            // validate() established LOD0; truncation and appending keep the vector nonempty.
            let previous = self.lods.last().unwrap();
            if indices.len() >= previous.indices.len() {
                break;
            }
            let error = (error * scale).max(previous.error);
            self.lods.push(MeshLod {
                indices: meshopt::optimize_vertex_cache(&indices, self.positions.len()),
                error,
            });
        }
        self.validate()
    }
}

#[cfg(all(test, feature = "mesh-optimization"))]
mod tests {
    use super::*;

    #[test]
    fn planar_lods_reduce_triangles_without_moving_the_surface() {
        let n = 12;
        let positions = (0..=n)
            .flat_map(|y| (0..=n).map(move |x| [x as f32, y as f32, 0.0]))
            .collect::<Vec<_>>();
        let mut indices = Vec::new();
        for y in 0..n {
            for x in 0..n {
                let a = y * (n + 1) + x;
                indices.extend([a, a + 1, a + n + 1, a + 1, a + n + 2, a + n + 1]);
            }
        }
        let mut mesh = TriangleMesh {
            uv: Vec::new(),
            tangents: Vec::new(),
            normals: vec![[0.0, 0.0, 1.0]; positions.len()],
            positions,
            lods: vec![MeshLod {
                indices,
                error: 0.0,
            }],
        };
        let original = mesh.positions.clone();
        mesh.optimize(4, 0.01).unwrap();
        assert_eq!(mesh.positions, original);
        assert!(mesh.lods.len() > 1);
        assert!(mesh.lods.last().unwrap().indices.len() < mesh.lods[0].indices.len() / 2);
        assert_eq!(mesh.bounds(), ([0.0, 0.0, 0.0], [12.0, 12.0, 0.0]));
        let before = mesh
            .lods
            .iter()
            .map(|l| l.indices.clone())
            .collect::<Vec<_>>();
        mesh.optimize(4, 0.01).unwrap();
        assert_eq!(
            before,
            mesh.lods
                .iter()
                .map(|l| l.indices.clone())
                .collect::<Vec<_>>()
        );
        let mut invalid = mesh.clone();
        invalid.lods[0].indices[0] = u32::MAX;
        assert!(invalid.optimize(4, 0.01).is_err());
        let mut invalid = mesh.clone();
        invalid.positions[0][0] = f32::NAN;
        assert!(invalid.validate().is_err());
        assert!(mesh.optimize(5, 0.01).is_err());
        assert!(mesh.optimize(2, f32::NAN).is_err());
    }
}
