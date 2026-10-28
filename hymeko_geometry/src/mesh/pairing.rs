//! Deterministic quality matching on the triangle dual adjacency.
use super::topology::*;

pub(super) struct Polygon {
    pub raw: Vec<u32>,
}

#[derive(Clone)]
struct Candidate {
    a: usize,
    b: usize,
    raw: [u32; 4],
    score: f64,
}

pub(super) struct Pairing {
    candidates: Vec<Candidate>,
    adjacent: Vec<Vec<usize>>,
}

impl Pairing {
    pub fn new(topology: &Topology<'_>) -> Self {
        let mut candidates = Vec::new();
        for (&(u, v), edge) in &topology.edges {
            if edge.protected {
                continue;
            }
            let [a, b] = edge.faces[..] else {
                continue;
            };
            let triangle = topology.raw[a];
            let k = (0..3)
                .find(|&k| {
                    let x = topology.alias[triangle[k] as usize];
                    let y = topology.alias[triangle[(k + 1) % 3] as usize];
                    (x == u && y == v) || (x == v && y == u)
                })
                .unwrap();
            let opposite = *topology.raw[b]
                .iter()
                .find(|&&i| {
                    let x = topology.alias[i as usize];
                    x != u && x != v
                })
                .unwrap();
            let raw = [
                triangle[k],
                opposite,
                triangle[(k + 1) % 3],
                triangle[(k + 2) % 3],
            ];
            let p = raw.map(|i| point(topology.mesh.positions[i as usize]));
            let n = unit(add(topology.normals[a], topology.normals[b]));
            let mut score = 1_f64;
            let lengths: Vec<_> = (0..4).map(|i| length(sub(p[(i + 1) % 4], p[i]))).collect();
            for i in 0..4 {
                let x = sub(p[(i + 1) % 4], p[i]);
                let y = sub(p[(i + 3) % 4], p[i]);
                score = score.min(dot(cross(x, y), n) / (length(x) * length(y)).max(1e-30));
            }
            score *= lengths.iter().copied().fold(f64::INFINITY, f64::min)
                / lengths.iter().copied().fold(0., f64::max);
            if score > 0.08 {
                candidates.push(Candidate { a, b, raw, score });
            }
        }
        candidates.sort_by(|a, b| {
            b.score
                .total_cmp(&a.score)
                .then(a.a.cmp(&b.a))
                .then(a.b.cmp(&b.b))
        });
        let mut adjacent = vec![Vec::new(); topology.faces.len()];
        for (i, c) in candidates.iter().enumerate() {
            adjacent[c.a].push(i);
            adjacent[c.b].push(i);
        }
        Self {
            candidates,
            adjacent,
        }
    }

    fn other(&self, candidate: usize, face: usize) -> usize {
        let c = &self.candidates[candidate];
        if c.a == face { c.b } else { c.a }
    }

    pub fn polygons(&self, topology: &Topology<'_>) -> Vec<Polygon> {
        let mut matched = vec![None; topology.faces.len()];
        for (i, c) in self.candidates.iter().enumerate() {
            if matched[c.a].is_none() && matched[c.b].is_none() {
                matched[c.a] = Some(i);
                matched[c.b] = Some(i);
            }
        }
        // A bounded augmenting path replaces one pair with two. Remaining
        // triangles are handled conformingly by refinement, never discarded.
        for start in 0..matched.len() {
            if matched[start].is_some() {
                continue;
            }
            'augment: for &first in &self.adjacent[start] {
                let a = self.other(first, start);
                let Some(middle) = matched[a] else {
                    continue;
                };
                let b = self.other(middle, a);
                for &last in &self.adjacent[b] {
                    let end = self.other(last, b);
                    if end != start && matched[end].is_none() {
                        matched[start] = Some(first);
                        matched[a] = Some(first);
                        matched[b] = Some(last);
                        matched[end] = Some(last);
                        break 'augment;
                    }
                }
            }
        }
        let mut output = Vec::new();
        for (i, entry) in matched.iter().enumerate() {
            if let Some(k) = *entry {
                let c = &self.candidates[k];
                if i == c.a.min(c.b) {
                    output.push(Polygon {
                        raw: c.raw.to_vec(),
                    });
                }
            } else {
                output.push(Polygon {
                    raw: topology.raw[i].to_vec(),
                });
            }
        }
        output
    }
}
