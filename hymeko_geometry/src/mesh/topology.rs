//! Indexed-surface adjacency and local projection; no spatial partition or IR graph.
use super::TriangleMesh;
use std::collections::{BTreeMap, BTreeSet, VecDeque};

pub(super) type P = [f64; 3];
pub(super) fn add(a: P, b: P) -> P {
    std::array::from_fn(|i| a[i] + b[i])
}
pub(super) fn sub(a: P, b: P) -> P {
    std::array::from_fn(|i| a[i] - b[i])
}
pub(super) fn mul(a: P, s: f64) -> P {
    a.map(|x| x * s)
}
pub(super) fn dot(a: P, b: P) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}
pub(super) fn cross(a: P, b: P) -> P {
    std::array::from_fn(|i| a[(i + 1) % 3] * b[(i + 2) % 3] - a[(i + 2) % 3] * b[(i + 1) % 3])
}
pub(super) fn length(a: P) -> f64 {
    dot(a, a).sqrt()
}
pub(super) fn unit(a: P) -> P {
    mul(a, 1. / length(a).max(1e-30))
}
pub(super) fn point(a: [f32; 3]) -> P {
    a.map(f64::from)
}
pub(super) fn edge(a: usize, b: usize) -> (usize, usize) {
    (a.min(b), a.max(b))
}

#[derive(Clone)]
pub(super) struct Edge {
    pub faces: Vec<usize>,
    pub protected: bool,
}

pub(super) struct Topology<'a> {
    pub mesh: &'a TriangleMesh,
    pub positions: Vec<P>,
    pub alias: Vec<usize>,
    pub faces: Vec<[usize; 3]>,
    pub raw: Vec<[u32; 3]>,
    pub normals: Vec<P>,
    pub edges: BTreeMap<(usize, usize), Edge>,
    pub raw_incident: Vec<Vec<usize>>,
    pub adjacent: Vec<Vec<usize>>,
    pub pinned: Vec<bool>,
}

#[derive(Clone, Copy)]
pub(super) struct Sample {
    pub position: P,
    pub triangle: usize,
    pub bary: P,
}

impl<'a> Topology<'a> {
    pub fn new(mesh: &'a TriangleMesh, crease_cos: f64) -> Result<Self, String> {
        Self::build(mesh, crease_cos, false)
    }

    /// Filtering retains transport topology while omitting zero-area work faces.
    /// Reconstruction keeps using the strict constructor.
    pub fn for_processing(mesh: &'a TriangleMesh, crease_cos: f64) -> Result<Self, String> {
        Self::build(mesh, crease_cos, true)
    }

    fn build(mesh: &'a TriangleMesh, crease_cos: f64, processing: bool) -> Result<Self, String> {
        mesh.validate()?;
        let mut lookup = BTreeMap::new();
        let mut positions = Vec::new();
        let alias = mesh
            .positions
            .iter()
            .map(|&p| {
                let key = p.map(|x| if x == 0. { 0 } else { x.to_bits() });
                *lookup.entry(key).or_insert_with(|| {
                    let i = positions.len();
                    positions.push(point(p));
                    i
                })
            })
            .collect::<Vec<_>>();
        let input = mesh.lods[0]
            .indices
            .chunks_exact(3)
            .map(|v| [v[0], v[1], v[2]])
            .collect::<Vec<_>>();
        let mut raw = Vec::with_capacity(input.len());
        let mut faces = Vec::with_capacity(input.len());
        let mut normals = Vec::with_capacity(input.len());
        let mut edges: BTreeMap<_, Edge> = BTreeMap::new();
        let mut raw_incident = vec![Vec::new(); mesh.positions.len()];
        for (i, triangle) in input.iter().enumerate() {
            let f = triangle.map(|v| alias[v as usize]);
            let n = cross(
                sub(positions[f[1]], positions[f[0]]),
                sub(positions[f[2]], positions[f[0]]),
            );
            if f[0] == f[1] || f[1] == f[2] || f[2] == f[0] || length(n) <= 1e-14 {
                if processing {
                    continue;
                }
                return Err(format!("degenerate source triangle {i}"));
            }
            let index = faces.len();
            raw.push(*triangle);
            faces.push(f);
            normals.push(unit(n));
            for k in 0..3 {
                raw_incident[triangle[k] as usize].push(index);
                let entry = edges.entry(edge(f[k], f[(k + 1) % 3])).or_insert(Edge {
                    faces: Vec::new(),
                    protected: false,
                });
                entry.faces.push(index);
                if entry.faces.len() > 2 && !processing {
                    return Err("nonmanifold surface edge".into());
                }
            }
        }
        if faces.is_empty() {
            return Err("surface has no nondegenerate work faces".into());
        }
        let mut pinned = vec![false; positions.len()];
        let mut adjacent = vec![Vec::new(); faces.len()];
        for (&(a, b), entry) in &mut edges {
            entry.protected = entry.faces.len() != 2;
            if let [i, j] = entry.faces[..] {
                let direction = |f: [usize; 3]| (0..3).any(|k| f[k] == a && f[(k + 1) % 3] == b);
                if direction(faces[i]) == direction(faces[j]) {
                    return Err("inconsistent source winding".into());
                }
                let seam = !mesh.uv.is_empty()
                    && [a, b].iter().any(|&v| {
                        let first = (0..3).find(|&k| faces[i][k] == v).unwrap();
                        let second = (0..3).find(|&k| faces[j][k] == v).unwrap();
                        mesh.uv[raw[i][first] as usize]
                            .iter()
                            .zip(mesh.uv[raw[j][second] as usize])
                            .any(|(x, y)| (x - y).abs() > 1e-6)
                    });
                entry.protected = seam || dot(normals[i], normals[j]) < crease_cos;
                // Local projection may walk across smooth continuous charts,
                // but never jump between UV islands or across a sharp fold.
                if !entry.protected {
                    adjacent[i].push(j);
                    adjacent[j].push(i);
                }
            }
            if entry.protected {
                pinned[a] = true;
                pinned[b] = true;
            }
        }
        Ok(Self {
            mesh,
            positions,
            alias,
            faces,
            raw,
            normals,
            edges,
            raw_incident,
            adjacent,
            pinned,
        })
    }

    pub fn hints(&self, vertices: &[u32]) -> Vec<usize> {
        vertices
            .iter()
            .flat_map(|&v| self.raw_incident[v as usize].iter().copied())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect()
    }

    pub fn project(&self, p: P, hints: &[usize]) -> Sample {
        let mut queue: VecDeque<_> = hints.iter().copied().collect();
        let mut seen = BTreeSet::new();
        let mut best = Sample {
            position: p,
            triangle: hints[0],
            bary: [1., 0., 0.],
        };
        let mut distance = f64::INFINITY;
        while let Some(i) = queue.pop_front() {
            if !seen.insert(i) {
                continue;
            }
            if seen.len() > 512 {
                break;
            }
            let triangle = self.faces[i].map(|v| self.positions[v]);
            let bary = Self::closest(p, triangle);
            let q = (0..3).fold([0.; 3], |sum, k| add(sum, mul(triangle[k], bary[k])));
            let d = dot(sub(q, p), sub(q, p));
            if d <= distance + 1e-18 {
                if d < distance || i < best.triangle {
                    best = Sample {
                        position: q,
                        triangle: i,
                        bary,
                    };
                    distance = d;
                }
                queue.extend(self.adjacent[i].iter().copied());
            }
        }
        best
    }

    pub fn uv(&self, sample: Sample) -> [f32; 2] {
        if self.mesh.uv.is_empty() {
            return [0.; 2];
        }
        std::array::from_fn(|axis| {
            (0..3)
                .map(|k| {
                    self.mesh.uv[self.raw[sample.triangle][k] as usize][axis] as f64
                        * sample.bary[k]
                })
                .sum::<f64>() as f32
        })
    }

    // Closest point on one triangle, expressed in its original corner basis.
    fn closest(p: P, triangle: [P; 3]) -> P {
        let [a, b, c] = triangle;
        let ab = sub(b, a);
        let ac = sub(c, a);
        let ap = sub(p, a);
        let n = cross(ab, ac);
        let denominator = dot(n, n);
        let u = dot(cross(ap, ac), n) / denominator;
        let v = dot(cross(ab, ap), n) / denominator;
        let interior = [1. - u - v, u, v];
        if interior.iter().all(|&w| w >= 0.) {
            return interior;
        }
        let mut best = [1., 0., 0.];
        let mut distance = f64::INFINITY;
        for i in 0..3 {
            let j = (i + 1) % 3;
            let direction = sub(triangle[j], triangle[i]);
            let t = (dot(sub(p, triangle[i]), direction) / dot(direction, direction)).clamp(0., 1.);
            let q = add(triangle[i], mul(direction, t));
            let delta = sub(q, p);
            let d = dot(delta, delta);
            if d < distance {
                distance = d;
                best = [0.; 3];
                best[i] = 1. - t;
                best[j] = t;
            }
        }
        best
    }
}
