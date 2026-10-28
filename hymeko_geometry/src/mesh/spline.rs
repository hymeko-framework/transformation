//! Regular tensor-product cubic B-spline charts, including Catmull--Clark regular regions.
//! A chart needs a complete control grid; extraordinary topology is deliberately not inferred.
use super::{MeshLod, TriangleMesh};
use serde::{Deserialize, Serialize};

/// Ascending power coefficients, shared by the CPU evaluator and GPU backends.
pub const CUBIC_BASIS: [[f32; 4]; 4] = [
    [1. / 6., -0.5, 0.5, -1. / 6.],
    [4. / 6., 0., -1., 0.5],
    [1. / 6., 0.5, 0.5, -0.5],
    [0., 0., 0., 1. / 6.],
];

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SplineSurface {
    pub columns: u32,
    pub rows: u32,
    pub controls: Vec<[f32; 3]>,
    pub uv_scale: [f32; 2],
}
#[derive(Debug, Clone, Copy)]
pub struct PatchSample {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub tangent: [f32; 3],
    pub uv: [f32; 2],
}
impl SplineSurface {
    pub fn validate(&self) -> Result<(), String> {
        if !(4..=35).contains(&self.columns)
            || !(4..=35).contains(&self.rows)
            || self.controls.len() != (self.columns * self.rows) as usize
            || self
                .controls
                .iter()
                .flatten()
                .any(|v| !v.is_finite() || v.abs() > 10000.)
            || self
                .uv_scale
                .iter()
                .any(|v| !v.is_finite() || !(0.001..=100.).contains(v))
        {
            return Err(
                "spline chart needs a finite 4..35 by 4..35 control grid and bounded UV scale"
                    .into(),
            );
        }
        Ok(())
    }
    /// Full grid dimensions at one shared density; adjacent patch edges use the same records.
    pub fn grid_size(&self, steps: u32) -> Result<[u32; 2], String> {
        self.validate()?;
        if ![2, 4, 8, 16].contains(&steps) {
            return Err("spline density must be 2,4,8 or16".into());
        }
        Ok([(self.columns - 3) * steps + 1, (self.rows - 3) * steps + 1])
    }
    pub fn indices(&self, steps: u32) -> Result<Vec<u32>, String> {
        let [width, height] = self.grid_size(steps)?;
        Ok(Self::grid_indices(width, height))
    }
    fn grid_indices(width: u32, height: u32) -> Vec<u32> {
        let mut out = Vec::with_capacity(((width - 1) * (height - 1) * 6) as usize);
        for y in 0..height - 1 {
            for x in 0..width - 1 {
                let a = y * width + x;
                out.extend([a, a + 1, a + width, a + 1, a + width + 1, a + width]);
            }
        }
        out
    }
    pub fn sample(&self, uv: [f32; 2]) -> Result<PatchSample, String> {
        self.validate()?;
        if uv.iter().any(|v| !v.is_finite() || !(0.0..=1.).contains(v)) {
            return Err("spline parameter outside chart".into());
        }
        self.sample_validated(uv)
    }
    fn sample_validated(&self, uv: [f32; 2]) -> Result<PatchSample, String> {
        let cells = [self.columns - 3, self.rows - 3];
        let p = [uv[0] * cells[0] as f32, uv[1] * cells[1] as f32];
        let cell = [
            (p[0] as u32).min(cells[0] - 1),
            (p[1] as u32).min(cells[1] - 1),
        ];
        let local = [p[0] - cell[0] as f32, p[1] - cell[1] as f32];
        let basis = |t: f32| {
            CUBIC_BASIS.map(|c| {
                [
                    c[0] + t * (c[1] + t * (c[2] + t * c[3])),
                    c[1] + t * (2. * c[2] + 3. * t * c[3]),
                ]
            })
        };
        let u = basis(local[0]);
        let v = basis(local[1]);
        let mut position = [0.; 3];
        let mut du = [0.; 3];
        let mut dv = [0.; 3];
        for y in 0..4 {
            for x in 0..4 {
                let point = self.controls[((cell[1] + y) * self.columns + cell[0] + x) as usize];
                for axis in 0..3 {
                    position[axis] += point[axis] * u[x as usize][0] * v[y as usize][0];
                    du[axis] += point[axis] * u[x as usize][1] * v[y as usize][0];
                    dv[axis] += point[axis] * u[x as usize][0] * v[y as usize][1];
                }
            }
        }
        let cross = [
            du[1] * dv[2] - du[2] * dv[1],
            du[2] * dv[0] - du[0] * dv[2],
            du[0] * dv[1] - du[1] * dv[0],
        ];
        let unit = |v: [f32; 3]| {
            let length = v.iter().map(|x| x * x).sum::<f32>().sqrt();
            if length < 1e-10 {
                Err("degenerate spline derivative".to_string())
            } else {
                Ok(v.map(|x| x / length))
            }
        };
        Ok(PatchSample {
            position,
            normal: unit(cross)?,
            tangent: unit(du)?,
            uv: [uv[0] * self.uv_scale[0], uv[1] * self.uv_scale[1]],
        })
    }
    /// Triangle interchange/fallback view; all samples and UVs come from the same chart.
    pub fn mesh(&self, steps: u32) -> Result<TriangleMesh, String> {
        let [width, height] = self.grid_size(steps)?;
        let mut mesh = TriangleMesh {
            positions: Vec::new(),
            normals: Vec::new(),
            uv: Vec::new(),
            tangents: Vec::new(),
            lods: vec![MeshLod {
                indices: self.indices(steps)?,
                error: 0.,
            }],
        };
        for y in 0..height {
            for x in 0..width {
                let p = self.sample_validated([
                    x as f32 / (width - 1) as f32,
                    y as f32 / (height - 1) as f32,
                ])?;
                mesh.positions.push(p.position);
                mesh.normals.push(p.normal);
                mesh.uv.push(p.uv);
                mesh.tangents
                    .push([p.tangent[0], p.tangent[1], p.tangent[2], 1.]);
            }
        }
        mesh.validate()?;
        Ok(mesh)
    }
    /// A conservative-bounds triangle view with sampled LOD errors relative to density16.
    /// The LOD error is measured at the finest grid samples, not a Hausdorff certificate.
    pub fn render_mesh(&self) -> Result<TriangleMesh, String> {
        let mut mesh = self.mesh(16)?;
        let [width, height] = self.grid_size(16)?;
        let mut previous = 0_f32;
        for steps in [8, 4, 2] {
            let stride = 16 / steps;
            let [w, h] = self.grid_size(steps)?;
            let indices = self
                .indices(steps)?
                .into_iter()
                .map(|i| (i / w) * stride * width + (i % w) * stride)
                .collect();
            let mut error = previous;
            for y in 0..height {
                for x in 0..width {
                    let gx = (x / stride).min(w - 2);
                    let gy = (y / stride).min(h - 2);
                    let u = (x - gx * stride) as f32 / stride as f32;
                    let v = (y - gy * stride) as f32 / stride as f32;
                    let a = (gy * stride * width + gx * stride) as usize;
                    let b = a + stride as usize;
                    let c = a + (stride * width) as usize;
                    let d = c + stride as usize;
                    let estimated = std::array::from_fn::<_, 3, _>(|k| {
                        if u + v <= 1. {
                            mesh.positions[a][k] * (1. - u - v)
                                + mesh.positions[b][k] * u
                                + mesh.positions[c][k] * v
                        } else {
                            mesh.positions[b][k] * (1. - v)
                                + mesh.positions[c][k] * (1. - u)
                                + mesh.positions[d][k] * (u + v - 1.)
                        }
                    });
                    let exact = mesh.positions[(y * width + x) as usize];
                    error = error.max(
                        (0..3)
                            .map(|k| (exact[k] - estimated[k]).powi(2))
                            .sum::<f32>()
                            .sqrt(),
                    );
                }
            }
            mesh.lods.push(MeshLod { indices, error });
            previous = error;
        }
        let (lo, hi) = self.bounds()?;
        // Unreferenced hull corners keep ordinary mesh culling/shadow fitting conservative,
        // including the maximum admitted relief displacement. They are never rasterized.
        for i in 0..8 {
            mesh.positions.push(std::array::from_fn(|k| {
                if i & (1 << k) == 0 {
                    lo[k] - 0.02
                } else {
                    hi[k] + 0.02
                }
            }));
            mesh.normals.push([0., 0., 1.]);
            mesh.uv.push([0.; 2]);
            mesh.tangents.push([1., 0., 0., 1.]);
        }
        mesh.validate()?;
        Ok(mesh)
    }
    /// Convex-hull bounds; displacement padding is a separate renderer/material contract.
    pub fn bounds(&self) -> Result<([f32; 3], [f32; 3]), String> {
        self.validate()?;
        let mut lo = [f32::INFINITY; 3];
        let mut hi = [f32::NEG_INFINITY; 3];
        for p in &self.controls {
            for i in 0..3 {
                lo[i] = lo[i].min(p[i]);
                hi[i] = hi[i].max(p[i]);
            }
        }
        Ok((lo, hi))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn chart() -> SplineSurface {
        SplineSurface {
            columns: 5,
            rows: 5,
            controls: (0..5)
                .flat_map(|y| (0..5).map(move |x| [x as f32, y as f32, 0.]))
                .collect(),
            uv_scale: [1.; 2],
        }
    }
    #[test]
    fn regular_limit_plane_and_shared_edge_samples_are_consistent() {
        let surface = chart();
        for uv in [[0., 0.], [0.5, 0.2], [0.5, 0.5], [1., 1.]] {
            let p = surface.sample(uv).unwrap();
            for (actual, expected) in
                p.position
                    .into_iter()
                    .zip([1. + uv[0] * 2., 1. + uv[1] * 2., 0.])
            {
                assert!((actual - expected).abs() < 1e-5);
            }
            assert!((p.normal[2] - 1.).abs() < 1e-6);
        }
        let mesh = surface.mesh(4).unwrap();
        assert_eq!(mesh.positions.len(), 81);
        assert_eq!(mesh.lods[0].indices.len(), 384);
        assert!(mesh.positions.iter().all(|p| p[2] == 0.));
    }
    #[test]
    fn curved_surface_is_smooth_and_refuses_invalid_inputs() {
        let mut surface = chart();
        surface.controls[12][2] = 1.;
        let a = surface.sample([0.5 - 0.0001, 0.5]).unwrap();
        let b = surface.sample([0.5 + 0.0001, 0.5]).unwrap();
        assert!(a.position[2] > 0.4 && (a.position[2] - b.position[2]).abs() < 1e-5);
        assert!(
            a.normal
                .iter()
                .zip(b.normal)
                .all(|(x, y)| (x - y).abs() < 0.002)
        );
        assert!(surface.sample([f32::NAN, 0.]).is_err());
        assert!(surface.grid_size(17).is_err());
        surface.controls.pop();
        assert!(surface.validate().is_err());
    }
    #[test]
    fn triangle_view_has_conservative_bounds_and_monotone_lods() {
        let mut surface=chart();surface.controls[12][2]=0.4;
        let mesh=surface.render_mesh().unwrap();
        assert_eq!(mesh.lods.len(),4);assert!(mesh.lods[3].error>mesh.lods[1].error);
        for (lod,steps) in mesh.lods.iter().zip([16,8,4,2]) {
            assert_eq!(lod.indices.len(),surface.indices(steps).unwrap().len());
        }
        let (lo,hi)=mesh.bounds();
        assert!(surface.controls.iter().all(|p|(0..3).all(|i|p[i]>=lo[i] && p[i]<=hi[i])));
    }
}
