//! Offline surface union with a convex addition and explicit attribute provenance.
//!
//! Inputs must be embedded, closed, positively oriented surfaces. Cross-operand
//! contacts must be transverse: coincident faces and vertex/edge contacts return
//! an error. This bounded operation is not a general-purpose robust CSG kernel.
//! Combinatorial manifold checks do not certify arbitrary input self-intersections.
use super::{MeshLod, TriangleMesh, bridge::BoundaryBridge, topology::*};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct UnionOptions {
    /// Relative to the combined bounding-box diagonal; ambiguous contacts fail.
    pub tolerance: f64,
    pub maximum_faces: usize,
    pub maximum_work: usize,
}
impl Default for UnionOptions {
    fn default() -> Self {
        Self {
            tolerance: 1e-10,
            maximum_faces: 20_000,
            maximum_work: 50_000_000,
        }
    }
}

/// Index zero denotes the base, index one the convex addition.
#[derive(Clone, Debug, Serialize)]
pub struct VertexOrigin {
    pub operand: usize,
    pub triangle: usize,
    /// An unchanged source record, when present; otherwise use barycentric weights.
    pub vertex: Option<u32>,
    pub barycentric: [f64; 3],
}
#[derive(Clone, Debug, Serialize)]
pub struct FaceOrigin {
    pub operand: usize,
    pub triangle: usize,
}
#[derive(Clone, Debug, Serialize)]
pub struct UnionSurface {
    pub mesh: TriangleMesh,
    pub vertices: Vec<VertexOrigin>,
    pub faces: Vec<FaceOrigin>,
    /// Attribute records on the shared geometric intersection curves.
    pub intersection_vertices: Vec<u32>,
    pub intersection_segments: usize,
}

/// One canonical implementation; no renderer or character-specific data enters it.
pub struct SurfaceUnion;

#[derive(Clone)]
struct Facet {
    nodes: [u32; 3],
    axis: P,
    lo: P,
    hi: P,
}
impl Facet {
    fn new(nodes: [u32; 3], points: &[P]) -> Self {
        let p = nodes.map(|i| points[i as usize]);
        Self {
            nodes,
            axis: unit(cross(sub(p[1], p[0]), sub(p[2], p[0]))),
            lo: std::array::from_fn(|k| p.iter().map(|v| v[k]).fold(f64::INFINITY, f64::min)),
            hi: std::array::from_fn(|k| p.iter().map(|v| v[k]).fold(f64::NEG_INFINITY, f64::max)),
        }
    }
    fn distance(&self, p: P, points: &[P]) -> f64 {
        dot(sub(p, points[self.nodes[0] as usize]), self.axis)
    }
    fn barycentric(&self, p: P, points: &[P]) -> P {
        let [a, b, c] = self.nodes.map(|i| points[i as usize]);
        let area = dot(cross(sub(b, a), sub(c, a)), self.axis);
        [
            dot(cross(sub(b, p), sub(c, p)), self.axis) / area,
            dot(cross(sub(c, p), sub(a, p)), self.axis) / area,
            dot(cross(sub(a, p), sub(b, p)), self.axis) / area,
        ]
    }
    fn overlaps(&self, other: &Self, tolerance: f64) -> bool {
        (0..3)
            .all(|k| self.lo[k] <= other.hi[k] + tolerance && other.lo[k] <= self.hi[k] + tolerance)
    }
}

#[derive(Default)]
struct FaceCut {
    boundary: [BTreeSet<u32>; 3],
    segments: BTreeSet<(u32, u32)>,
}

struct Arrangement {
    points: Vec<P>,
    facets: [Vec<Facet>; 2],
    cuts: [Vec<FaceCut>; 2],
    // An edge crossing a face has one identity in both adjacent source triangles.
    crossing: BTreeMap<(usize, u32, u32, usize), u32>,
    segments: BTreeSet<(u32, u32)>,
    original_nodes: usize,
    tolerance: f64,
    work: usize,
    maximum_work: usize,
    maximum_nodes: usize,
}

impl SurfaceUnion {
    /// Preserve source attributes and return LOD0 plus barycentric provenance.
    ///
    /// Both operands must be connected, embedded closed boundaries. The addition
    /// is checked for convexity. Disjoint results may have two components. More
    /// than one isolated contour in a source triangle is currently unsupported.
    /// All results pass closed-edge, winding, vertex-fan and volume checks at the
    /// actual float32 output precision. Errors never mutate either input.
    pub fn with_convex(
        base: &TriangleMesh,
        addition: &TriangleMesh,
        options: &UnionOptions,
    ) -> Result<UnionSurface, String> {
        if !options.tolerance.is_finite()
            || !(1e-12..=1e-6).contains(&options.tolerance)
            || !(4..=200_000).contains(&options.maximum_faces)
            || !(1..=100_000_000).contains(&options.maximum_work)
        {
            return Err("invalid bounded surface union options".into());
        }
        if base.uv.is_empty() != addition.uv.is_empty()
            || base.tangents.is_empty() != addition.tangents.is_empty()
        {
            return Err("union operands must have matching UV/tangent streams".into());
        }
        let meshes = [base, addition];
        if meshes.iter().any(|m| {
            m.positions.len() > options.maximum_faces.saturating_mul(3)
                || m.lods
                    .first()
                    .is_some_and(|lod| lod.indices.len() / 3 > options.maximum_faces)
        }) {
            return Err("surface union input budget exceeded".into());
        }
        let topologies = [Self::closed(base, true)?, Self::closed(addition, true)?];
        if topologies
            .iter()
            .any(|t| t.faces.len() > options.maximum_faces)
            || topologies[0]
                .faces
                .len()
                .saturating_mul(topologies[1].faces.len())
                > options.maximum_work
        {
            return Err("surface union input budget exceeded".into());
        }
        let lo: P = std::array::from_fn(|k| {
            topologies
                .iter()
                .flat_map(|t| &t.positions)
                .map(|p| p[k])
                .fold(f64::INFINITY, f64::min)
        });
        let hi: P = std::array::from_fn(|k| {
            topologies
                .iter()
                .flat_map(|t| &t.positions)
                .map(|p| p[k])
                .fold(f64::NEG_INFINITY, f64::max)
        });
        let scale = length(sub(hi, lo));
        if !scale.is_finite() || scale <= 1e-9 {
            return Err("collapsed union bounds".into());
        }
        let offsets = [0, topologies[0].positions.len() as u32];
        let points: Vec<P> = topologies
            .iter()
            .flat_map(|t| &t.positions)
            .map(|&p| mul(sub(p, lo), 1. / scale))
            .collect();
        let facets: [Vec<Facet>; 2] = std::array::from_fn(|s| {
            topologies[s]
                .faces
                .iter()
                .map(|f| Facet::new(f.map(|v| v as u32 + offsets[s]), &points))
                .collect()
        });
        let convex_work = facets[1]
            .len()
            .saturating_mul(topologies[1].positions.len());
        if convex_work > options.maximum_work {
            return Err("convex validation work budget exceeded".into());
        }
        let referenced: BTreeSet<_> = facets[1].iter().flat_map(|f| f.nodes).collect();
        if facets[1].iter().any(|f| {
            referenced
                .iter()
                .any(|&i| f.distance(points[i as usize], &points) > options.tolerance)
        }) {
            return Err("union addition is not convex".into());
        }
        let mut arrangement = Arrangement {
            original_nodes: points.len(),
            points,
            cuts: std::array::from_fn(|s| {
                (0..facets[s].len()).map(|_| FaceCut::default()).collect()
            }),
            facets,
            crossing: BTreeMap::new(),
            segments: BTreeSet::new(),
            tolerance: options.tolerance,
            work: convex_work,
            maximum_work: options.maximum_work,
            maximum_nodes: topologies.iter().map(|t| t.positions.len()).sum::<usize>()
                + options.maximum_faces.saturating_mul(2),
        };
        arrangement.intersect()?;
        let mut output = UnionSurface {
            mesh: TriangleMesh {
                positions: vec![],
                normals: vec![],
                uv: vec![],
                tangents: vec![],
                lods: vec![MeshLod {
                    indices: vec![],
                    error: 0.,
                }],
            },
            vertices: vec![],
            faces: vec![],
            intersection_vertices: vec![],
            intersection_segments: arrangement.segments.len(),
        };
        let mut records = BTreeMap::new();
        for s in 0..2 {
            for f in 0..arrangement.facets[s].len() {
                let cut = &arrangement.cuts[s][f];
                if !cut.segments.is_empty() {
                    let n = 3
                        + cut.boundary.iter().map(BTreeSet::len).sum::<usize>()
                        + cut.segments.len() * 2;
                    arrangement.spend(n.saturating_pow(3).min(8_388_608))?;
                }
                let triangles = arrangement.regions(s, f)?;
                for tri in triangles.chunks_exact(3) {
                    let centre = mul(
                        tri.iter()
                            .fold([0.; 3], |sum, &i| add(sum, arrangement.points[i as usize])),
                        1. / 3.,
                    );
                    if arrangement.inside(1 - s, centre)? {
                        continue;
                    }
                    if output.faces.len() >= options.maximum_faces {
                        return Err("union output face budget exceeded".into());
                    }
                    let source = topologies[s].raw[f];
                    for &node in tri {
                        let facet = &arrangement.facets[s][f];
                        let corner = facet.nodes.iter().position(|&v| v == node);
                        let copied = corner.map(|k| source[k]);
                        // New records stay split across source faces/UV seams,
                        // but their positions share a canonical intersection ID.
                        let key = (s, copied.map_or(f, |v| v as usize), node, copied.is_some());
                        let index = if let Some(&index) = records.get(&key) {
                            index
                        } else {
                            let index = output.mesh.positions.len() as u32;
                            let bary = corner.map_or_else(
                                || {
                                    facet.barycentric(
                                        arrangement.points[node as usize],
                                        &arrangement.points,
                                    )
                                },
                                |k| std::array::from_fn(|i| if i == k { 1. } else { 0. }),
                            );
                            if bary
                                .iter()
                                .any(|&v| !v.is_finite() || !(-1e-7..=1.0000001).contains(&v))
                            {
                                return Err("union provenance left its source triangle".into());
                            }
                            let position = copied.map_or_else(
                                || {
                                    add(lo, mul(arrangement.points[node as usize], scale))
                                        .map(|v| v as f32)
                                },
                                |v| meshes[s].positions[v as usize],
                            );
                            let blend = |values: &[[f32; 3]]| -> P {
                                (0..3).fold([0.; 3], |sum, k| {
                                    add(sum, mul(point(values[source[k] as usize]), bary[k]))
                                })
                            };
                            let normal = copied.map_or_else(
                                || unit(blend(&meshes[s].normals)).map(|v| v as f32),
                                |v| meshes[s].normals[v as usize],
                            );
                            output.mesh.positions.push(position);
                            output.mesh.normals.push(normal);
                            if !meshes[s].uv.is_empty() {
                                output.mesh.uv.push(copied.map_or_else(
                                    || {
                                        std::array::from_fn(|axis| {
                                            (0..3)
                                                .map(|k| {
                                                    bary[k]
                                                        * meshes[s].uv[source[k] as usize][axis]
                                                            as f64
                                                })
                                                .sum::<f64>()
                                                as f32
                                        })
                                    },
                                    |v| meshes[s].uv[v as usize],
                                ));
                            }
                            if !meshes[s].tangents.is_empty() {
                                let t = if let Some(v) = copied {
                                    meshes[s].tangents[v as usize]
                                } else {
                                    let sign = meshes[s].tangents[source[0] as usize][3];
                                    if source
                                        .iter()
                                        .any(|&v| meshes[s].tangents[v as usize][3] != sign)
                                    {
                                        return Err(
                                            "union cut crosses mixed tangent handedness".into()
                                        );
                                    }
                                    let t: P = std::array::from_fn(|axis| {
                                        (0..3)
                                            .map(|k| {
                                                bary[k]
                                                    * meshes[s].tangents[source[k] as usize][axis]
                                                        as f64
                                            })
                                            .sum()
                                    });
                                    let n = unit(point(normal));
                                    let t = unit(sub(t, mul(n, dot(t, n))));
                                    [t[0] as f32, t[1] as f32, t[2] as f32, sign]
                                };
                                output.mesh.tangents.push(t);
                            }
                            output.vertices.push(VertexOrigin {
                                operand: s,
                                triangle: f,
                                vertex: copied,
                                barycentric: bary,
                            });
                            if node as usize >= arrangement.original_nodes {
                                output.intersection_vertices.push(index);
                            }
                            records.insert(key, index);
                            index
                        };
                        output.mesh.lods[0].indices.push(index);
                    }
                    output.faces.push(FaceOrigin {
                        operand: s,
                        triangle: f,
                    });
                }
            }
        }
        Self::closed(&output.mesh, false)
            .map_err(|e| format!("union output is not a closed float32 surface: {e}"))?;
        Ok(output)
    }

    fn closed(mesh: &TriangleMesh, connected: bool) -> Result<Topology<'_>, String> {
        let t = Topology::new(mesh, -1.)?;
        if t.edges.values().any(|e| e.faces.len() != 2) {
            return Err("solid requires two faces per geometric edge".into());
        }
        let mut adjacent = vec![Vec::new(); t.faces.len()];
        for e in t.edges.values() {
            let a = e.faces[0];
            let b = e.faces[1];
            adjacent[a].push(b);
            adjacent[b].push(a);
        }
        let mut seen = BTreeSet::new();
        let mut components = 0;
        for first in 0..t.faces.len() {
            if seen.contains(&first) {
                continue;
            }
            components += 1;
            let mut queue = VecDeque::from([first]);
            seen.insert(first);
            let origin = t.positions[t.faces[first][0]];
            let mut volume = 0.;
            while let Some(i) = queue.pop_front() {
                let p = t.faces[i].map(|v| sub(t.positions[v], origin));
                volume += dot(p[0], cross(p[1], p[2])) / 6.;
                for &j in &adjacent[i] {
                    if seen.insert(j) {
                        queue.push_back(j);
                    }
                }
            }
            if volume <= 0. {
                return Err("solid component is not positively oriented".into());
            }
        }
        if connected && components != 1 {
            return Err("union input must have one connected boundary".into());
        }
        let mut incident = vec![BTreeSet::new(); t.positions.len()];
        for (i, f) in t.faces.iter().enumerate() {
            for &v in f {
                incident[v].insert(i);
            }
        }
        for faces in &incident {
            let Some(&first) = faces.first() else {
                continue;
            };
            let mut reached = BTreeSet::from([first]);
            let mut queue = VecDeque::from([first]);
            while let Some(i) = queue.pop_front() {
                for &j in &adjacent[i] {
                    if faces.contains(&j) && reached.insert(j) {
                        queue.push_back(j);
                    }
                }
            }
            if reached.len() != faces.len() {
                return Err("solid contains a disconnected vertex fan".into());
            }
        }
        Ok(t)
    }
}

impl Arrangement {
    fn spend(&mut self, n: usize) -> Result<(), String> {
        self.work = self.work.saturating_add(n);
        if self.work > self.maximum_work {
            Err("surface union work budget exceeded".into())
        } else {
            Ok(())
        }
    }
    fn intersect(&mut self) -> Result<(), String> {
        for a in 0..self.facets[0].len() {
            for b in 0..self.facets[1].len() {
                self.spend(1)?;
                if !self.facets[0][a].overlaps(&self.facets[1][b], self.tolerance) {
                    continue;
                }
                let faces = [self.facets[0][a].clone(), self.facets[1][b].clone()];
                if (0..2).any(|s| {
                    let d = faces[s]
                        .nodes
                        .map(|v| faces[1 - s].distance(self.points[v as usize], &self.points));
                    d.iter().all(|&x| x > self.tolerance) || d.iter().all(|&x| x < -self.tolerance)
                }) {
                    continue;
                }
                if length(cross(faces[0].axis, faces[1].axis)) <= self.tolerance {
                    if self.coplanar_overlap(&faces) {
                        return Err("coplanar or tangential union contact is unsupported".into());
                    }
                    continue;
                }
                let mut cut = BTreeSet::new();
                for s in 0..2 {
                    let own = &faces[s];
                    let other = &faces[1 - s];
                    let fid = [a, b];
                    for &v in &own.nodes {
                        let p = self.points[v as usize];
                        if other.distance(p, &self.points).abs() <= self.tolerance
                            && other
                                .barycentric(p, &self.points)
                                .iter()
                                .all(|&w| w >= -self.tolerance)
                        {
                            return Err("vertex/face union contact is numerically ambiguous".into());
                        }
                    }
                    for k in 0..3 {
                        let x = own.nodes[k].min(own.nodes[(k + 1) % 3]);
                        let y = own.nodes[k].max(own.nodes[(k + 1) % 3]);
                        let p = self.points[x as usize];
                        let q = self.points[y as usize];
                        let d = other.distance(p, &self.points);
                        let e = other.distance(q, &self.points);
                        if d * e >= 0. {
                            continue;
                        }
                        let t = d / (d - e);
                        let position = add(p, mul(sub(q, p), t));
                        let weights = other.barycentric(position, &self.points);
                        if weights.iter().any(|&w| w < -self.tolerance) {
                            continue;
                        }
                        if weights.iter().any(|&w| w <= self.tolerance)
                            || t <= self.tolerance
                            || t >= 1. - self.tolerance
                        {
                            return Err("edge/edge union contact is numerically ambiguous".into());
                        }
                        let key = (s, x, y, fid[1 - s]);
                        let id = if let Some(&i) = self.crossing.get(&key) {
                            i
                        } else {
                            if self.points.len() >= self.maximum_nodes {
                                return Err(
                                    "surface union intersection vertex budget exceeded".into()
                                );
                            }
                            let i = self.points.len() as u32;
                            self.points.push(position);
                            self.crossing.insert(key, i);
                            i
                        };
                        self.cuts[s][fid[s]].boundary[k].insert(id);
                        cut.insert(id);
                    }
                }
                if cut.is_empty() {
                    continue;
                }
                if cut.len() != 2 {
                    return Err("non-transverse triangle intersection".into());
                }
                let ids: Vec<_> = cut.into_iter().collect();
                let segment = (ids[0], ids[1]);
                self.cuts[0][a].segments.insert(segment);
                self.cuts[1][b].segments.insert(segment);
                self.segments.insert(segment);
            }
        }
        let mut degree = BTreeMap::new();
        for &(a, b) in &self.segments {
            *degree.entry(a).or_insert(0) += 1;
            *degree.entry(b).or_insert(0) += 1;
        }
        if degree.values().any(|&v| v != 2) {
            return Err("intersection curves are not closed disjoint loops".into());
        }
        Ok(())
    }
    fn coplanar_overlap(&self, faces: &[Facet; 2]) -> bool {
        if faces[0].nodes.iter().any(|&v| {
            faces[1]
                .distance(self.points[v as usize], &self.points)
                .abs()
                > self.tolerance
        }) {
            return false;
        }
        for s in 0..2 {
            if faces[s].nodes.iter().any(|&v| {
                faces[1 - s]
                    .barycentric(self.points[v as usize], &self.points)
                    .iter()
                    .all(|&w| w >= -self.tolerance)
            }) {
                return true;
            }
        }
        let axis = faces[0].axis;
        let side = |a: P, b: P, c: P| dot(cross(sub(b, a), sub(c, a)), axis);
        (0..3).any(|i| {
            (0..3).any(|j| {
                let a = self.points[faces[0].nodes[i] as usize];
                let b = self.points[faces[0].nodes[(i + 1) % 3] as usize];
                let c = self.points[faces[1].nodes[j] as usize];
                let d = self.points[faces[1].nodes[(j + 1) % 3] as usize];
                side(a, b, c) * side(a, b, d) < 0. && side(c, d, a) * side(c, d, b) < 0.
            })
        })
    }
    fn inside(&mut self, operand: usize, p: P) -> Result<bool, String> {
        self.spend(self.facets[operand].len())?;
        if operand == 1 {
            return Ok(self.facets[1]
                .iter()
                .all(|f| f.distance(p, &self.points) < 0.));
        }
        let mut angle = 0.;
        for f in &self.facets[0] {
            let [a, b, c] = f.nodes.map(|i| sub(self.points[i as usize], p));
            let [la, lb, lc] = [length(a), length(b), length(c)];
            angle += 2.
                * dot(a, cross(b, c))
                    .atan2(la * lb * lc + dot(a, b) * lc + dot(b, c) * la + dot(c, a) * lb);
        }
        if angle.abs() < 1e-5 {
            Ok(false)
        } else if (angle.abs() - 4. * std::f64::consts::PI).abs() < 1e-5 {
            Ok(true)
        } else {
            Err("ambiguous solid-angle containment".into())
        }
    }

    fn regions(&self, operand: usize, index: usize) -> Result<Vec<u32>, String> {
        let facet = &self.facets[operand][index];
        let cut = &self.cuts[operand][index];
        if cut.segments.is_empty() {
            return Ok(facet.nodes.to_vec());
        }
        let mut edges = cut.segments.clone();
        for k in 0..3 {
            let a = facet.nodes[k];
            let b = facet.nodes[(k + 1) % 3];
            let direction = sub(self.points[b as usize], self.points[a as usize]);
            let mut nodes: Vec<_> = cut.boundary[k].iter().copied().collect();
            nodes.extend([a, b]);
            nodes.sort_by(|&x, &y| {
                dot(self.points[x as usize], direction)
                    .total_cmp(&dot(self.points[y as usize], direction))
            });
            nodes.dedup();
            for pair in nodes.windows(2) {
                edges.insert((pair[0].min(pair[1]), pair[0].max(pair[1])));
            }
        }
        let mut adjacent: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
        for &(a, b) in &edges {
            adjacent.entry(a).or_default().push(b);
            adjacent.entry(b).or_default().push(a);
        }
        let u = unit(sub(
            self.points[facet.nodes[1] as usize],
            self.points[facet.nodes[0] as usize],
        ));
        let v = cross(facet.axis, u);
        for (&node, neighbors) in &mut adjacent {
            let angle = |i: u32| {
                let d = sub(self.points[i as usize], self.points[node as usize]);
                dot(d, v).atan2(dot(d, u))
            };
            neighbors.sort_by(|&a, &b| angle(a).total_cmp(&angle(b)));
        }
        let mut visited = BTreeSet::new();
        let mut cycles = Vec::new();
        for (&a, neighbors) in &adjacent {
            for &b in neighbors {
                if visited.contains(&(a, b)) {
                    continue;
                }
                let start = (a, b);
                let mut current = start;
                let mut polygon = Vec::new();
                loop {
                    if !visited.insert(current) {
                        return Err("face cut has a repeated directed edge".into());
                    }
                    polygon.push(current.0);
                    let list = &adjacent[&current.1];
                    let back = list.iter().position(|&i| i == current.0).unwrap();
                    current = (current.1, list[(back + list.len() - 1) % list.len()]);
                    if current == start {
                        break;
                    }
                    if polygon.len() > edges.len() * 2 {
                        return Err("face cut traversal exceeded its edge budget".into());
                    }
                }
                let area = polygon
                    .iter()
                    .zip(polygon.iter().cycle().skip(1))
                    .map(|(&a, &b)| {
                        dot(
                            cross(self.points[a as usize], self.points[b as usize]),
                            facet.axis,
                        )
                    })
                    .sum::<f64>();
                if area > 1e-16 {
                    cycles.push((area, polygon));
                }
            }
        }
        let mut reached = BTreeSet::new();
        let mut components = 0;
        for &node in adjacent.keys() {
            if !reached.insert(node) {
                continue;
            }
            components += 1;
            let mut pending = vec![node];
            while let Some(i) = pending.pop() {
                for &j in &adjacent[&i] {
                    if reached.insert(j) {
                        pending.push(j);
                    }
                }
            }
        }
        if components == 1 {
            let mut result = Vec::new();
            for (_, p) in cycles {
                result.extend(BoundaryBridge::polygon_indices(
                    &self.points,
                    &p,
                    facet.axis,
                )?);
            }
            return Ok(result);
        }
        if components != 2 || cycles.len() != 2 {
            return Err("multiple isolated face contours are unsupported".into());
        }
        cycles.sort_by(|a, b| b.0.total_cmp(&a.0));
        let outer = &cycles[0].1;
        let inner = &cycles[1].1;
        let mapping: Vec<_> = outer.iter().chain(inner).copied().collect();
        let n = mapping.len();
        let mut positions: Vec<_> = mapping
            .iter()
            .map(|&i| self.points[i as usize].map(|v| v as f32))
            .collect();
        for ring in [outer, inner] {
            positions.push(
                mul(
                    ring.iter()
                        .fold([0.; 3], |sum, &i| add(sum, self.points[i as usize])),
                    1. / ring.len() as f64,
                )
                .map(|v| v as f32),
            );
        }
        let mut indices = Vec::new();
        for (start, count, centre, reverse) in [
            (0, outer.len(), n, true),
            (outer.len(), inner.len(), n + 1, false),
        ] {
            for i in 0..count {
                let mut f = [
                    (start + i) as u32,
                    (start + (i + 1) % count) as u32,
                    centre as u32,
                ];
                if reverse {
                    f.reverse();
                }
                indices.extend(f);
            }
        }
        let query = TriangleMesh {
            normals: vec![facet.axis.map(|v| v as f32); positions.len()],
            positions,
            uv: vec![],
            tangents: vec![],
            lods: vec![MeshLod { indices, error: 0. }],
        };
        let bridge = BoundaryBridge {
            first: (0..outer.len() as u32).collect(),
            second: (outer.len() as u32..n as u32).collect(),
            maximum_edge: 4.,
        };
        let mut result: Vec<_> = bridge
            .indices(&query)?
            .iter()
            .map(|&i| mapping[i as usize])
            .collect();
        result.extend(BoundaryBridge::polygon_indices(
            &self.points,
            inner,
            facet.axis,
        )?);
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn octahedron(centre: P, radii: P) -> TriangleMesh {
        let positions: Vec<[f32; 3]> = (0..3)
            .flat_map(|axis| {
                [-1., 1.].map(move |sign| {
                    std::array::from_fn(|k| {
                        (centre[k] + if k == axis { sign * radii[k] } else { 0. }) as f32
                    })
                })
            })
            .collect();
        let mut indices = Vec::new();
        for x in [0, 1] {
            for y in [2, 3] {
                for z in [4, 5] {
                    let mut f = [x, y, z];
                    let p = f.map(|i| point(positions[i as usize]));
                    if dot(cross(sub(p[1], p[0]), sub(p[2], p[0])), sub(p[0], centre)) < 0. {
                        f.swap(1, 2);
                    }
                    indices.extend(f);
                }
            }
        }
        TriangleMesh {
            normals: positions
                .iter()
                .map(|&p| unit(sub(point(p), centre)).map(|v| v as f32))
                .collect(),
            positions,
            lods: vec![MeshLod { indices, error: 0. }],
            uv: vec![],
            tangents: vec![],
        }
    }
    fn volume(mesh: &TriangleMesh) -> f64 {
        mesh.lods[0]
            .indices
            .chunks_exact(3)
            .map(|f| {
                dot(
                    point(mesh.positions[f[0] as usize]),
                    cross(
                        point(mesh.positions[f[1] as usize]),
                        point(mesh.positions[f[2] as usize]),
                    ),
                ) / 6.
            })
            .sum()
    }
    #[test]
    fn transverse_union_is_closed_and_retains_source_provenance() {
        let mut a = octahedron([0.; 3], [1.1, 0.9, 1.]);
        let mut b = octahedron([0.65, 0.13, 0.17], [0.7, 0.63, 0.56]);
        for mesh in [&mut a, &mut b] {
            mesh.uv = mesh.positions.iter().map(|p| [p[0], p[1]]).collect();
        }
        let before = [
            serde_json::to_vec(&a).unwrap(),
            serde_json::to_vec(&b).unwrap(),
        ];
        let result = SurfaceUnion::with_convex(&a, &b, &UnionOptions::default()).unwrap();
        assert!(result.intersection_segments > 0);
        assert_eq!(result.vertices.len(), result.mesh.positions.len());
        assert!(volume(&result.mesh) > volume(&a).max(volume(&b)));
        assert!(volume(&result.mesh) < volume(&a) + volume(&b));
        for (i, source) in result.vertices.iter().enumerate() {
            let mesh = [&a, &b][source.operand];
            let f = &mesh.lods[0].indices[source.triangle * 3..source.triangle * 3 + 3];
            for axis in 0..3 {
                let p = (0..3)
                    .map(|k| source.barycentric[k] * mesh.positions[f[k] as usize][axis] as f64)
                    .sum::<f64>();
                assert!((p - result.mesh.positions[i][axis] as f64).abs() < 2e-6);
            }
            if let Some(v) = source.vertex {
                assert_eq!(result.mesh.positions[i], mesh.positions[v as usize]);
                assert_eq!(result.mesh.normals[i], mesh.normals[v as usize]);
            }
            assert!((result.mesh.uv[i][0] - result.mesh.positions[i][0]).abs() < 2e-6);
        }
        assert_eq!(
            before,
            [
                serde_json::to_vec(&a).unwrap(),
                serde_json::to_vec(&b).unwrap()
            ]
        );
    }
    #[test]
    fn disjoint_and_contained_solids_have_correct_volume() {
        let a = octahedron([0.; 3], [2.; 3]);
        for (centre, expected) in [
            ([4., 0.12, 0.07], volume(&a) + 1. / 6.),
            ([0.1, 0.03, 0.02], volume(&a)),
        ] {
            let b = octahedron(centre, [0.5; 3]);
            let result = SurfaceUnion::with_convex(&a, &b, &UnionOptions::default()).unwrap();
            assert!((volume(&result.mesh) - expected).abs() < 1e-5);
            assert_eq!(result.intersection_segments, 0);
        }
    }
    #[test]
    fn isolated_face_contour_is_triangulated_as_a_hole() {
        let a = octahedron([0.; 3], [2.; 3]);
        let b = octahedron([0.75, 0.75, 0.75], [0.4, 0.32, 0.29]);
        let result = SurfaceUnion::with_convex(&a, &b, &UnionOptions::default()).unwrap();
        assert!(result.intersection_segments >= 3);
        assert!(volume(&result.mesh) > volume(&a));
    }
    #[test]
    fn scale_changes_preserve_normalized_union_volume() {
        let mut expected = None;
        for scale in [0.01, 1., 1000.] {
            let a = octahedron([0.; 3], [1.1 * scale, 0.9 * scale, scale]);
            let b = octahedron(
                [0.65 * scale, 0.13 * scale, 0.17 * scale],
                [0.7 * scale, 0.63 * scale, 0.56 * scale],
            );
            let result = SurfaceUnion::with_convex(&a, &b, &UnionOptions::default()).unwrap();
            let v = volume(&result.mesh) / scale.powi(3);
            if let Some(reference) = expected {
                assert!(f64::abs(v - reference) < 2e-6);
            } else {
                expected = Some(v);
            }
        }
    }
    #[test]
    fn inward_and_nonconvex_additions_are_rejected() {
        let a = octahedron([0.; 3], [2.; 3]);
        let mut b = octahedron([0.; 3], [1.; 3]);
        b.positions[0] = [0.2, 0., 0.];
        assert!(
            SurfaceUnion::with_convex(&a, &b, &UnionOptions::default())
                .unwrap_err()
                .contains("not convex")
        );
        b = octahedron([0.; 3], [1.; 3]);
        for f in b.lods[0].indices.chunks_exact_mut(3) {
            f.swap(0, 1);
        }
        assert!(
            SurfaceUnion::with_convex(&a, &b, &UnionOptions::default())
                .unwrap_err()
                .contains("positively oriented")
        );
    }

    #[test]
    fn tangent_records_and_operand_stream_contracts_are_preserved() {
        let mut a = octahedron([0.; 3], [1.1, 0.9, 1.]);
        let mut b = octahedron([0.65, 0.13, 0.17], [0.7, 0.63, 0.56]);
        for mesh in [&mut a, &mut b] {
            mesh.tangents = vec![[1., 0., 0., 1.]; mesh.positions.len()];
        }
        let result = SurfaceUnion::with_convex(&a, &b, &UnionOptions::default()).unwrap();
        for (i, origin) in result.vertices.iter().enumerate() {
            let t = result.mesh.tangents[i];
            assert_eq!(t[3], 1.);
            if let Some(v) = origin.vertex {
                assert_eq!(t, [&a, &b][origin.operand].tangents[v as usize]);
            } else {
                assert!(
                    dot(
                        point(result.mesh.normals[i]),
                        [t[0] as f64, t[1] as f64, t[2] as f64]
                    )
                    .abs()
                        < 1e-5
                );
            }
        }
        b.tangents.clear();
        assert!(
            SurfaceUnion::with_convex(&a, &b, &UnionOptions::default())
                .unwrap_err()
                .contains("matching UV/tangent")
        );
        let small = octahedron([0.1, 0.02, 0.03], [0.2; 3]);
        let large = octahedron([0.; 3], [2.; 3]);
        let contained =
            SurfaceUnion::with_convex(&small, &large, &UnionOptions::default()).unwrap();
        assert!((volume(&contained.mesh) - volume(&large)).abs() < 1e-6);
        assert!(contained.vertices.iter().all(|v| v.operand == 1));
    }

    #[test]
    fn contacts_open_boundaries_and_budgets_fail_without_mutation() {
        let a = octahedron([0.; 3], [1.; 3]);
        let mut b = a.clone();
        assert!(SurfaceUnion::with_convex(&a, &b, &UnionOptions::default()).is_err());
        b.lods[0].indices.truncate(21);
        assert!(SurfaceUnion::with_convex(&a, &b, &UnionOptions::default()).is_err());
        b = octahedron([0.65, 0.13, 0.17], [0.7, 0.63, 0.56]);
        assert!(
            SurfaceUnion::with_convex(
                &a,
                &b,
                &UnionOptions {
                    maximum_work: 1,
                    ..UnionOptions::default()
                }
            )
            .is_err()
        );
    }
}
