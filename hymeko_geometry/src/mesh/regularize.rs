//! Quality pairing, conforming quad refinement and surface-constrained relaxation.
//! This is our local method, not a QuadriFlow or mixed-integer cross-field solver.
use super::{MeshLod, TriangleMesh, normals::NormalSmoothing, pairing::Pairing, topology::*};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Bounds in the input mesh's length units.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct RegularizeOptions {
    pub relaxation_steps: usize,
    pub relaxation: f64,
    pub max_displacement: f64,
    pub crease_degrees: f32,
    pub normal_steps: usize,
}
impl Default for RegularizeOptions {
    fn default() -> Self {
        Self {
            relaxation_steps: 4,
            relaxation: 0.2,
            max_displacement: 0.0015,
            crease_degrees: 60.,
            normal_steps: 8,
        }
    }
}

/// A genuine all-quad authoring surface and its seam-split GPU representation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuadSurface {
    pub positions: Vec<[f32; 3]>,
    pub quads: Vec<[u32; 4]>,
    pub corner_uv: Vec<[[f32; 2]; 4]>,
    pub mesh: TriangleMesh,
    pub pinned_vertices: usize,
    pub paired_control_faces: usize,
    pub maximum_relaxation: f64,
    pub maximum_surface_distance: f64,
    /// Original vertex to geometric control vertex; absent after control reduction.
    /// UV duplicates map to the same control vertex. Existing readers may ignore it.
    #[serde(default)]
    pub source_vertices: Vec<Option<u32>>,
}

/// The original detailed surface remains the projection/attribute owner.
pub struct SurfaceRegularizer<'a> {
    source: &'a TriangleMesh,
}

type TriangulatedSurface = (TriangleMesh, Vec<[[f32; 2]; 4]>);

struct Work {
    positions: Vec<P>,
    quads: Vec<[usize; 4]>,
    pinned: Vec<bool>,
    hints: Vec<Vec<usize>>,
    face_hints: Vec<Vec<usize>>,
}

impl<'a> SurfaceRegularizer<'a> {
    pub fn new(source: &'a TriangleMesh) -> Result<Self, String> {
        source.validate()?;
        Ok(Self { source })
    }

    /// Control indices refer to original vertices. Existing LOD preparation can
    /// reduce the control budget; every new point projects back onto the full
    /// original surface through local connected-face walks, not a global snap.
    pub fn prepare(
        &self,
        control_indices: &[u32],
        options: &RegularizeOptions,
    ) -> Result<QuadSurface, String> {
        if options.relaxation_steps > 32
            || !options.relaxation.is_finite()
            || !(0. ..=0.5).contains(&options.relaxation)
            || !options.max_displacement.is_finite()
            || !(0. ..=1.).contains(&options.max_displacement)
            || !options.crease_degrees.is_finite()
            || !(5. ..=85.).contains(&options.crease_degrees)
            || options.normal_steps > 32
        {
            return Err("invalid surface regularization bounds".into());
        }
        let mut control = self.source.clone();
        control.lods = vec![MeshLod {
            indices: control_indices.to_vec(),
            error: 0.,
        }];
        let cosine = (options.crease_degrees as f64).to_radians().cos();
        let original = Topology::new(self.source, cosine)?;
        let topology = Topology::new(&control, cosine)?;
        if topology.faces.len() > 250_000 {
            return Err("control surface exceeds250000 triangles".into());
        }
        if topology
            .raw
            .iter()
            .flatten()
            .any(|&i| original.raw_incident[i as usize].is_empty())
        {
            return Err("control vertices must belong to the original surface".into());
        }
        let polygons = Pairing::new(&topology).polygons(&topology);
        let paired_control_faces = polygons.iter().filter(|p| p.raw.len() == 4).count() * 2;
        let mut work = Work {
            positions: Vec::new(),
            quads: Vec::new(),
            pinned: Vec::new(),
            hints: Vec::new(),
            face_hints: Vec::new(),
        };
        let mut corners = BTreeMap::new();
        let mut mids = BTreeMap::new();
        for polygon in polygons {
            let hints = original.hints(&polygon.raw);
            let mut vertices = Vec::new();
            let mut midpoints = Vec::new();
            for k in 0..polygon.raw.len() {
                let a = polygon.raw[k] as usize;
                let b = polygon.raw[(k + 1) % polygon.raw.len()] as usize;
                let ga = topology.alias[a];
                let gb = topology.alias[b];
                let vertex = *corners.entry(ga).or_insert_with(|| {
                    work.insert(
                        topology.positions[ga],
                        topology.pinned[ga],
                        original.hints(&[a as u32]),
                    )
                });
                vertices.push(vertex);
                let midpoint = *mids.entry(edge(ga, gb)).or_insert_with(|| {
                    let seeds = original.hints(&[a as u32, b as u32]);
                    let p = mul(add(topology.positions[ga], topology.positions[gb]), 0.5);
                    work.insert(p, topology.edges[&edge(ga, gb)].protected, seeds)
                });
                midpoints.push(midpoint);
            }
            let centre = polygon.raw.iter().fold([0.; 3], |sum, &i| {
                add(sum, point(control.positions[i as usize]))
            });
            let centre = mul(centre, 1. / polygon.raw.len() as f64);
            let center = work.insert(centre, false, hints.clone());
            for k in 0..vertices.len() {
                work.quads.push([
                    vertices[k],
                    midpoints[k],
                    center,
                    midpoints[(k + vertices.len() - 1) % vertices.len()],
                ]);
                work.face_hints.push(hints.clone());
            }
        }
        if work.quads.len() > 400_000 {
            return Err("quad surface exceeds400000 faces".into());
        }
        work.validate()?;
        work.project_safely(&original, options.max_displacement)?;
        let maximum_relaxation = work.relax(&original, options)?;
        let maximum_surface_distance = work
            .positions
            .iter()
            .zip(&work.hints)
            .map(|(&p, hints)| length(sub(original.project(p, hints).position, p)))
            .fold(0., f64::max);
        let (mut mesh, corner_uv) = work.triangles(&original)?;
        NormalSmoothing::apply(&mut mesh, options.normal_steps, options.crease_degrees)?;
        Ok(QuadSurface {
            positions: work.positions.iter().map(|p| p.map(|x| x as f32)).collect(),
            quads: work.quads.iter().map(|f| f.map(|i| i as u32)).collect(),
            corner_uv,
            mesh,
            pinned_vertices: work.pinned.iter().filter(|&&x| x).count(),
            paired_control_faces,
            maximum_relaxation,
            maximum_surface_distance,
            source_vertices: original
                .alias
                .iter()
                .map(|a| corners.get(a).map(|&i| i as u32))
                .collect(),
        })
    }
}

impl Work {
    fn project_safely(&mut self, source: &Topology<'_>, limit: f64) -> Result<(), String> {
        let base = self.positions.clone();
        let targets = base
            .iter()
            .zip(&self.hints)
            .map(|(&p, h)| source.project(p, h).position)
            .collect::<Vec<_>>();
        let normals = self
            .quads
            .iter()
            .map(|&q| Self::normal(&base, q))
            .collect::<Vec<_>>();
        let mut alpha = vec![1.; base.len()];
        for _ in 0..16 {
            for i in 0..base.len() {
                self.positions[i] = add(base[i], mul(sub(targets[i], base[i]), alpha[i]));
            }
            let mut rejected = BTreeSet::new();
            for (&q, &n) in self.quads.iter().zip(&normals) {
                if !Self::valid(&self.positions, q, n) {
                    rejected.extend(q);
                }
            }
            if rejected.is_empty() {
                break;
            }
            for i in rejected {
                alpha[i] = if alpha[i] < 0.001 { 0. } else { alpha[i] * 0.5 };
            }
        }
        self.validate()?;
        let distance = self
            .positions
            .iter()
            .zip(&self.hints)
            .map(|(&p, h)| length(sub(source.project(p, h).position, p)))
            .fold(0., f64::max);
        if distance > limit + 1e-9 {
            return Err(format!(
                "projected control surface residual {distance} exceeds {limit}; use a denser control surface"
            ));
        }
        Ok(())
    }
    fn insert(&mut self, p: P, pinned: bool, hints: Vec<usize>) -> usize {
        let i = self.positions.len();
        self.positions.push(p);
        self.pinned.push(pinned);
        self.hints.push(hints);
        i
    }

    fn normal(p: &[P], q: [usize; 4]) -> P {
        add(
            cross(sub(p[q[1]], p[q[0]]), sub(p[q[2]], p[q[0]])),
            cross(sub(p[q[2]], p[q[0]]), sub(p[q[3]], p[q[0]])),
        )
    }

    fn valid(p: &[P], q: [usize; 4], reference: P) -> bool {
        let n = Self::normal(p, q);
        if length(n) < 1e-14 || dot(n, reference) <= 0. {
            return false;
        }
        (0..4).all(|i| {
            dot(
                cross(
                    sub(p[q[(i + 1) % 4]], p[q[i]]),
                    sub(p[q[(i + 3) % 4]], p[q[i]]),
                ),
                n,
            ) > 1e-24
        })
    }

    fn validate(&self) -> Result<(), String> {
        if self.positions.iter().flatten().any(|x| !x.is_finite()) {
            return Err("nonfinite quad surface".into());
        }
        for (i, &q) in self.quads.iter().enumerate() {
            if !Self::valid(&self.positions, q, Self::normal(&self.positions, q)) {
                return Err(format!(
                    "folded or degenerate projected quad {i}: {:?}; use a denser control surface",
                    q.map(|v| self.positions[v])
                ));
            }
        }
        Ok(())
    }

    fn relax(&mut self, source: &Topology<'_>, options: &RegularizeOptions) -> Result<f64, String> {
        let initial = self.positions.clone();
        let mut neighbors = vec![BTreeSet::new(); initial.len()];
        for q in &self.quads {
            for k in 0..4 {
                neighbors[q[k]].insert(q[(k + 1) % 4]);
                neighbors[q[(k + 1) % 4]].insert(q[k]);
            }
        }
        for _ in 0..options.relaxation_steps {
            let before = self.positions.clone();
            let mut normals = vec![[0.; 3]; before.len()];
            let face_normals = self
                .quads
                .iter()
                .map(|&q| Self::normal(&before, q))
                .collect::<Vec<_>>();
            for (q, &n) in self.quads.iter().zip(&face_normals) {
                for &i in q {
                    normals[i] = add(normals[i], n);
                }
            }
            let mut candidate = before.clone();
            for i in 0..before.len() {
                if self.pinned[i] || neighbors[i].is_empty() {
                    continue;
                }
                let mean = mul(
                    neighbors[i]
                        .iter()
                        .fold([0.; 3], |sum, &j| add(sum, before[j])),
                    1. / neighbors[i].len() as f64,
                );
                let n = unit(normals[i]);
                let delta = sub(mean, before[i]);
                let tangent = sub(delta, mul(n, dot(delta, n)));
                let proposed = add(before[i], mul(tangent, options.relaxation));
                let offset = sub(proposed, initial[i]);
                let bounded = add(
                    initial[i],
                    mul(
                        offset,
                        (options.max_displacement / length(offset).max(1e-30)).min(1.),
                    ),
                );
                let projected = source.project(bounded, &self.hints[i]);
                if length(sub(projected.position, initial[i])) <= options.max_displacement + 1e-10 {
                    candidate[i] = projected.position;
                }
            }
            // Revert only unsafe local moves. A sliver must not prevent the
            // rest of a valid connected surface from regularizing.
            for _ in 0..8 {
                let mut rejected = BTreeSet::new();
                for (&q, &normal) in self.quads.iter().zip(&face_normals) {
                    if !Self::valid(&candidate, q, normal) {
                        rejected.extend(q);
                    }
                }
                if rejected.is_empty() {
                    break;
                }
                for i in rejected {
                    candidate[i] = before[i];
                }
            }
            if self
                .quads
                .iter()
                .zip(&face_normals)
                .all(|(&q, &n)| Self::valid(&candidate, q, n))
            {
                self.positions = candidate;
            }
        }
        self.validate()?;
        Ok(self
            .positions
            .iter()
            .zip(initial)
            .map(|(&p, q)| length(sub(p, q)))
            .fold(0., f64::max))
    }

    fn triangles(&self, source: &Topology<'_>) -> Result<TriangulatedSurface, String> {
        let mut mesh = TriangleMesh {
            positions: Vec::new(),
            normals: Vec::new(),
            uv: Vec::new(),
            tangents: Vec::new(),
            lods: vec![MeshLod {
                indices: Vec::new(),
                error: 0.,
            }],
        };
        let mut lookup = BTreeMap::new();
        let mut corner_uv = Vec::new();
        for (q, hints) in self.quads.iter().zip(&self.face_hints) {
            let mut indices = [0; 4];
            let mut coordinates = [[0.; 2]; 4];
            for k in 0..4 {
                let sample = source.project(self.positions[q[k]], hints);
                let uv = source.uv(sample);
                coordinates[k] = uv;
                let n = unit((0..3).fold([0.; 3], |sum, j| {
                    add(
                        sum,
                        mul(
                            point(source.mesh.normals[source.raw[sample.triangle][j] as usize]),
                            sample.bary[j],
                        ),
                    )
                }));
                let key = (
                    q[k],
                    uv.map(|x| (x as f64 * 1e6).round() as i64),
                    n.map(|x| (x * 1e3).round() as i64),
                );
                indices[k] = *lookup.entry(key).or_insert_with(|| {
                    let index = mesh.positions.len() as u32;
                    mesh.positions.push(self.positions[q[k]].map(|x| x as f32));
                    mesh.normals.push(n.map(|x| x as f32));
                    if !source.mesh.uv.is_empty() {
                        mesh.uv.push(uv);
                    }
                    index
                });
            }
            mesh.lods[0].indices.extend([
                indices[0], indices[1], indices[2], indices[0], indices[2], indices[3],
            ]);
            corner_uv.push(coordinates);
        }
        mesh.validate()?;
        Ok((mesh, corner_uv))
    }
}
