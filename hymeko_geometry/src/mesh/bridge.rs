//! Bounded triangulation of a gap between two existing, projectable boundary loops.
use super::{TriangleMesh, topology::*};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Existing vertex records in cyclic order. UV records may remain split at seams.
/// No positions, attributes or source faces are modified by this operation.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BoundaryBridge {
    pub first: Vec<u32>,
    pub second: Vec<u32>,
    pub maximum_edge: f64,
}
impl BoundaryBridge {
    /// Return new triangle indices joining two disjoint boundary loops.
    ///
    /// # Errors
    /// Rejects non-boundaries, repeated vertices, invalid limits, more than4M
    /// search cells/ear checks, and gaps without a nondegenerate, consistently projected strip.
    /// Source attributes and indices remain unchanged on success and failure.
    pub fn indices(&self, mesh: &TriangleMesh) -> Result<Vec<u32>, String> {
        if !self.maximum_edge.is_finite()
            || self.maximum_edge <= 0.
            || self.first.len() < 3
            || self.second.len() < 3
            || self.first.len().saturating_add(self.second.len()) > 2048
            || self
                .first
                .len()
                .saturating_add(1)
                .saturating_mul(self.second.len().saturating_add(1))
                > 4_194_304
        {
            return Err("invalid bridge dimensions or edge bound".into());
        }
        let topology = Topology::for_processing(mesh, -1.)?;
        let existing: BTreeSet<_> = topology
            .faces
            .iter()
            .map(|face| {
                let mut key = *face;
                key.sort_unstable();
                key
            })
            .collect();
        let occupied_source = |triangle: [u32; 3]| {
            let mut key = triangle.map(|v| topology.alias[v as usize]);
            key.sort_unstable();
            existing.contains(&key)
        };
        let mut first = self.first.clone();
        let mut second = self.second.clone();
        Self::orient(&topology, &mut first, false)?;
        Self::orient(&topology, &mut second, true)?;
        let vertices: BTreeSet<_> = first
            .iter()
            .chain(&second)
            .map(|&i| topology.alias[i as usize])
            .collect();
        if vertices.len() != first.len() + second.len() {
            return Err("bridge loops overlap or repeat a geometric vertex".into());
        }
        let p = |id: u32| point(mesh.positions[id as usize]);
        let axis = unit(
            first
                .iter()
                .zip(first.iter().cycle().skip(1))
                .fold([0.; 3], |sum, (&a, &b)| add(sum, cross(p(a), p(b)))),
        );
        if length(axis) < 0.5 {
            return Err("bridge requires a noncollapsed projected ring".into());
        }
        let mut pairs = first
            .iter()
            .enumerate()
            .flat_map(|(i, &a)| {
                second
                    .iter()
                    .enumerate()
                    .map(move |(j, &b)| (length(sub(p(a), p(b))), i, j))
            })
            .collect::<Vec<_>>();
        pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
        let (_, a, b) = pairs
            .into_iter()
            .take(4_194_304 / (first.len() + second.len()))
            .find(|&(distance, i, j)| {
                distance <= self.maximum_edge
                    && !Self::crosses(p(first[i]), p(second[j]), &first, &second, mesh, axis)
            })
            .ok_or("no visible bridge connection")?;
        first.rotate_left(a);
        second.rotate_left(b);
        // Cut the annulus along one visible connection. Repeated endpoint IDs
        // are the two sides of that cut, not duplicate geometric vertices.
        let mut polygon = first.clone();
        polygon.push(first[0]);
        polygon.push(second[0]);
        polygon.extend(second[1..].iter().rev());
        polygon.push(second[0]);
        self.triangulate(polygon, p, axis, occupied_source)
    }

    /// The same bounded ear clipping used by boundary bridges and face cuts.
    pub(super) fn polygon_indices(
        positions: &[P],
        polygon: &[u32],
        axis: P,
    ) -> Result<Vec<u32>, String> {
        let owner = Self {
            first: vec![],
            second: vec![],
            maximum_edge: f64::MAX,
        };
        owner.triangulate(polygon.to_vec(), |i| positions[i as usize], axis, |_| false)
    }

    fn triangulate(
        &self,
        mut polygon: Vec<u32>,
        p: impl Fn(u32) -> P + Copy,
        axis: P,
        occupied_source: impl Fn([u32; 3]) -> bool,
    ) -> Result<Vec<u32>, String> {
        if polygon.len() < 3 || polygon.len() > 2048 {
            return Err("invalid polygon dimensions".into());
        }
        let mut indices = Vec::with_capacity((polygon.len() - 2) * 3);
        let mut work = 0usize;
        while polygon.len() > 3 {
            let n = polygon.len();
            let mut ear = None;
            let mut best = f64::INFINITY;
            for i in 0..n {
                let triangle = [polygon[(i + n - 1) % n], polygon[i], polygon[(i + 1) % n]];
                let points = triangle.map(p);
                let cost = self.cost(points, axis);
                // A folded source rim may already contain this projected ear.
                // Adding its opposite would create a detached zero-volume pair.
                if cost >= best || occupied_source(triangle) {
                    continue;
                }
                // A legal ear must leave a legal final triangle. Collinear
                // samples on a retained boundary cannot be discarded.
                if n == 4 {
                    let remaining = std::array::from_fn(|k| polygon[(i + 1 + k) % n]);
                    if !self.cost(remaining.map(p), axis).is_finite() || occupied_source(remaining)
                    {
                        continue;
                    }
                }
                work += n;
                if work > 4_194_304 {
                    return Err("bridge triangulation work budget exceeded".into());
                }
                let occupied = polygon
                    .iter()
                    .any(|&v| !triangle.contains(&v) && Self::inside(p(v), points, axis));
                if !occupied {
                    ear = Some((i, triangle));
                    best = cost;
                }
            }
            let (index, triangle) = ear.ok_or("no bounded oriented bridge triangulation")?;
            indices.extend(triangle);
            polygon.remove(index);
        }
        let last = [polygon[0], polygon[1], polygon[2]];
        if !self.cost(last.map(p), axis).is_finite() || occupied_source(last) {
            return Err("degenerate final bridge triangle".into());
        }
        indices.extend(last);
        Ok(indices)
    }
    fn inside(p: P, triangle: [P; 3], axis: P) -> bool {
        (0..3).all(|i| {
            dot(
                cross(sub(triangle[(i + 1) % 3], triangle[i]), sub(p, triangle[i])),
                axis,
            ) >= -1e-14
        })
    }
    fn crosses(a: P, b: P, first: &[u32], second: &[u32], mesh: &TriangleMesh, axis: P) -> bool {
        [first, second].into_iter().any(|ring| {
            ring.iter()
                .zip(ring.iter().cycle().skip(1))
                .any(|(&u, &v)| {
                    let c = point(mesh.positions[u as usize]);
                    let d = point(mesh.positions[v as usize]);
                    let side = |x: P, y: P, z: P| dot(cross(sub(y, x), sub(z, x)), axis);
                    side(a, b, c) * side(a, b, d) < -1e-24 && side(c, d, a) * side(c, d, b) < -1e-24
                })
        })
    }
    fn orient(topology: &Topology<'_>, ring: &mut [u32], forward: bool) -> Result<(), String> {
        let mut direction = None;
        for (&a, &b) in ring.iter().zip(ring.iter().cycle().skip(1)) {
            let (Some(&a), Some(&b)) = (
                topology.alias.get(a as usize),
                topology.alias.get(b as usize),
            ) else {
                return Err("bridge vertex outside source".into());
            };
            let faces = &topology
                .edges
                .get(&edge(a, b))
                .ok_or("bridge is not an existing edge")?
                .faces;
            if faces.len() != 1 {
                return Err("bridge edge must bound exactly one source triangle".into());
            }
            let face = topology.faces[faces[0]];
            let current = (0..3).any(|k| face[k] == a && face[(k + 1) % 3] == b);
            if direction.is_some_and(|v| v != current) {
                return Err("bridge ring is not consistently ordered".into());
            }
            direction = Some(current);
        }
        if direction != Some(forward) {
            ring.reverse();
        }
        Ok(())
    }
    fn cost(&self, p: [P; 3], axis: P) -> f64 {
        let edges = [sub(p[1], p[0]), sub(p[2], p[1]), sub(p[0], p[2])];
        let normal = cross(edges[0], mul(edges[2], -1.));
        let area = length(normal);
        if area <= 1e-14
            || dot(normal, axis) <= area * 1e-6
            || edges.iter().any(|&e| length(e) > self.maximum_edge)
        {
            return f64::INFINITY;
        }
        edges.iter().map(|&e| dot(e, e)).sum::<f64>() / area
    }
}

#[cfg(test)]
mod tests {
    use super::super::MeshLod;
    use super::*;
    fn fixture() -> (TriangleMesh, BoundaryBridge) {
        let mut positions = Vec::new();
        let mut indices = Vec::new();
        for (count, radii, heights) in [(12, [3., 2.], [0., 0.]), (7, [1., 1.], [0.3, 0.8])] {
            let start = positions.len() as u32;
            for (radius, y) in radii.into_iter().zip(heights) {
                for i in 0..count {
                    let a = i as f32 * std::f32::consts::TAU / count as f32;
                    positions.push([radius * a.cos(), y, -radius * a.sin()]);
                }
            }
            for i in 0..count {
                let a = start + i;
                let b = start + (i + 1) % count;
                indices.extend([a, b, a + count, b, b + count, a + count]);
            }
        }
        let mesh = TriangleMesh {
            normals: vec![[0., 1., 0.]; positions.len()],
            positions,
            lods: vec![MeshLod { indices, error: 0. }],
            uv: vec![],
            tangents: vec![],
        };
        (
            mesh,
            BoundaryBridge {
                first: (12..24).collect(),
                second: (24..31).collect(),
                maximum_edge: 2.,
            },
        )
    }
    #[test]
    fn collinear_boundary_samples_survive_polygon_triangulation() {
        let p = [
            [0., 0., 0.],
            [1., 0., 0.],
            [2., 0., 0.],
            [3., 0., 0.],
            [3., 2., 0.],
            [0., 2., 0.],
        ];
        let faces = BoundaryBridge::polygon_indices(&p, &[0, 1, 2, 3, 4, 5], [0., 0., 1.]).unwrap();
        assert_eq!(faces.len(), 12);
        let area: f64 = faces
            .chunks_exact(3)
            .map(|f| {
                let n = cross(
                    sub(p[f[1] as usize], p[f[0] as usize]),
                    sub(p[f[2] as usize], p[f[0] as usize]),
                );
                assert!(n[2] > 0.);
                n[2] * 0.5
            })
            .sum();
        assert!((area - 6.).abs() < 1e-12);
    }

    #[test]
    fn unequal_loops_share_manifold_edges_and_retain_all_source_records() {
        let (mesh, mut bridge) = fixture();
        let original = serde_json::to_vec(&mesh).unwrap();
        for reversed in [false, true] {
            if reversed {
                bridge.first.reverse();
                bridge.second.reverse();
            }
            let strip = bridge.indices(&mesh).unwrap();
            assert_eq!(strip.len(), (12 + 7) * 3);
            let mut joined = mesh.clone();
            joined.lods[0].indices.extend(strip);
            let topology = Topology::new(&joined, -1.).unwrap();
            for ring in [&bridge.first, &bridge.second] {
                for (&a, &b) in ring.iter().zip(ring.iter().cycle().skip(1)) {
                    assert_eq!(topology.edges[&edge(a as usize, b as usize)].faces.len(), 2);
                }
            }
            assert_eq!(serde_json::to_vec(&mesh).unwrap(), original);
        }
    }
    #[test]
    fn concave_boundary_retains_its_existing_edges() {
        let (mut mesh, bridge) = fixture();
        mesh.positions[26][0] *= 0.25;
        mesh.positions[26][2] *= 0.25;
        let strip = bridge.indices(&mesh).unwrap();
        assert_eq!(strip.len(), (bridge.first.len() + bridge.second.len()) * 3);
        mesh.lods[0].indices.extend(strip);
        let topology = Topology::new(&mesh, -1.).unwrap();
        for (&a, &b) in bridge
            .second
            .iter()
            .zip(bridge.second.iter().cycle().skip(1))
        {
            assert_eq!(topology.edges[&edge(a as usize, b as usize)].faces.len(), 2);
        }
    }
    #[test]
    fn invalid_rings_and_unreachable_gaps_are_rejected() {
        let (mesh, bridge) = fixture();
        let mut invalid = bridge.clone();
        invalid.maximum_edge = 0.1;
        assert!(invalid.indices(&mesh).is_err());
        invalid = bridge.clone();
        invalid.first[1] = invalid.first[0];
        assert!(invalid.indices(&mesh).is_err());
        invalid = bridge.clone();
        invalid.second[0] = u32::MAX;
        assert!(invalid.indices(&mesh).is_err());
        invalid = bridge;
        invalid.maximum_edge = f64::NAN;
        assert!(invalid.indices(&mesh).is_err());
    }
}
