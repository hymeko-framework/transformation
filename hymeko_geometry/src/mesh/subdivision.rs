//! Catmull--Clark limit samples of an oriented manifold quad control cage.
//!
//! These are exact limit positions at control-vertex parameter locations, not
//! arbitrary (u,v) patch evaluation or adaptive tessellation. Regular regions
//! agree with bicubic uniform B-splines; extraordinary vertices use their actual
//! valence. Boundary curves use the cubic rule, with single-face corners fixed.
use super::topology::{P, add, edge, mul, point};
use std::collections::{BTreeMap, BTreeSet};

/// A borrowed geometric cage; UV chart duplication belongs to the consumer.
pub struct SubdivisionSurface<'a> {
    positions: &'a [[f32; 3]],
    quads: &'a [[u32; 4]],
}

impl<'a> SubdivisionSurface<'a> {
    /// Validate bounded finite data before allocating adjacency.
    pub fn new(positions: &'a [[f32; 3]], quads: &'a [[u32; 4]]) -> Result<Self, String> {
        if !(4..=1_000_000).contains(&positions.len())
            || !(1..=400_000).contains(&quads.len())
            || positions.iter().flatten().any(|x| !x.is_finite())
            || quads.iter().any(|q| {
                q.iter().any(|&v| v as usize >= positions.len())
                    || q.iter().collect::<BTreeSet<_>>().len() != 4
            })
        {
            return Err("invalid bounded quad control cage".into());
        }
        Ok(Self { positions, quads })
    }

    /// Sample the smooth limit at every cage vertex, retaining vertex order.
    /// Unreferenced vertices and disconnected/bow-tie vertex fans are rejected.
    pub fn limit_positions(&self) -> Result<Vec<[f32; 3]>, String> {
        self.sample_limits(false)
    }

    /// Hold every patch boundary CV fixed. Several open fans may meet at a fixed
    /// boundary corner; each must be an oriented chain, never a closed fan mixed
    /// with a boundary. Interior fans retain the ordinary manifold limit rule.
    /// Useful for fitting clipped brushes whose surrounding surface stays exact.
    pub fn limit_positions_with_fixed_boundary(&self) -> Result<Vec<[f32; 3]>, String> {
        self.sample_limits(true)
    }

    fn sample_limits(&self, fixed_boundary: bool) -> Result<Vec<[f32; 3]>, String> {
        let n = self.positions.len();
        let mut links = vec![BTreeMap::<usize, usize>::new(); n];
        let mut neighbors = vec![BTreeSet::new(); n];
        let mut opposites = vec![[0.; 3]; n];
        let mut counts = vec![0usize; n];
        let mut edges: BTreeMap<(usize, usize), (usize, usize)> = BTreeMap::new();
        for q in self.quads {
            let q = q.map(|v| v as usize);
            for k in 0..4 {
                let (a, b, previous) = (q[k], q[(k + 1) % 4], q[(k + 3) % 4]);
                if links[a].insert(previous, b).is_some() {
                    return Err("duplicate or inconsistently wound quad fan".into());
                }
                let entry = edges.entry(edge(a, b)).or_insert((0, a));
                if entry.0 >= 2 || entry.0 == 1 && entry.1 == a {
                    return Err("nonmanifold or inconsistently wound quad edge".into());
                }
                entry.0 += 1;
                neighbors[a].extend([b, previous]);
                opposites[a] = add(opposites[a], point(self.positions[q[(k + 2) % 4]]));
                counts[a] += 1;
            }
        }
        let mut boundary = vec![Vec::new(); n];
        for ((a, b), (count, _)) in edges {
            if count == 1 {
                boundary[a].push(b);
                boundary[b].push(a);
            }
        }
        (0..n)
            .map(|i| {
                let count = counts[i];
                let boundary_count = boundary[i].len();
                if fixed_boundary && boundary_count > 0 {
                    let mut visited = BTreeSet::new();
                    for &start in &boundary[i] {
                        if !links[i].contains_key(&start) {
                            continue;
                        }
                        let mut current = start;
                        while let Some(&next) = links[i].get(&current) {
                            if !visited.insert(current) {
                                return Err("invalid fixed-boundary quad fan".into());
                            }
                            current = next;
                        }
                        if !boundary[i].contains(&current) {
                            return Err("unterminated fixed-boundary quad fan".into());
                        }
                    }
                    if visited.len() != count {
                        return Err("closed fan meets a fixed patch boundary".into());
                    }
                    return Ok(self.positions[i]);
                }
                if count == 0
                    || !matches!(boundary_count, 0 | 2)
                    || neighbors[i].len() != count + usize::from(boundary_count == 2)
                {
                    return Err("disconnected or invalid quad vertex fan".into());
                }
                let start = if boundary_count == 0 {
                    *links[i].keys().next().ok_or("empty quad fan")?
                } else {
                    *boundary[i]
                        .iter()
                        .find(|v| links[i].contains_key(v))
                        .ok_or("invalid boundary fan")?
                };
                let mut current = start;
                let mut visited = BTreeSet::new();
                while let Some(&next) = links[i].get(&current) {
                    if !visited.insert(current) {
                        break;
                    }
                    current = next;
                }
                if visited.len() != count || (boundary_count == 0 && current != start) {
                    return Err("disconnected quad vertex fan".into());
                }
                let p = point(self.positions[i]);
                let limit: P = if boundary_count == 2 {
                    if count == 1 {
                        p
                    } else {
                        mul(
                            add(
                                mul(p, 4.),
                                add(
                                    point(self.positions[boundary[i][0]]),
                                    point(self.positions[boundary[i][1]]),
                                ),
                            ),
                            1. / 6.,
                        )
                    }
                } else {
                    let adjacent = neighbors[i]
                        .iter()
                        .fold([0.; 3], |sum, &j| add(sum, point(self.positions[j])));
                    let valence = count as f64;
                    mul(
                        add(
                            add(mul(p, valence * valence), mul(adjacent, 4.)),
                            opposites[i],
                        ),
                        1. / (valence * (valence + 5.)),
                    )
                };
                Ok(limit.map(|x| x as f32))
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clipped_patches_can_share_only_explicitly_fixed_boundary_corners() {
        let points = [
            [0., 0., 0.],
            [1., 0., 0.],
            [1., 1., 0.],
            [0., 1., 0.],
            [-1., 0., 0.],
            [-1., -1., 0.],
            [0., -1., 0.],
        ];
        let quads = [[0, 1, 2, 3], [0, 4, 5, 6]];
        let surface = SubdivisionSurface::new(&points, &quads).unwrap();
        assert!(surface.limit_positions().is_err());
        assert_eq!(
            surface.limit_positions_with_fixed_boundary().unwrap(),
            points
        );
        let malformed = [[0, 1, 2, 3], [0, 1, 5, 6]];
        assert!(
            SubdivisionSurface::new(&points, &malformed)
                .unwrap()
                .limit_positions_with_fixed_boundary()
                .is_err()
        );
    }

    #[test]
    fn regular_bicubic_limit_and_boundary_corner_rules() {
        let points: Vec<_> = (0..3)
            .flat_map(|y| (0..3).map(move |x| [x as f32, y as f32, (x * x + y * y) as f32]))
            .collect();
        let quads = [[0, 1, 4, 3], [1, 2, 5, 4], [3, 4, 7, 6], [4, 5, 8, 7]];
        let result = SubdivisionSurface::new(&points, &quads)
            .unwrap()
            .limit_positions()
            .unwrap();
        let fixed = SubdivisionSurface::new(&points, &quads)
            .unwrap()
            .limit_positions_with_fixed_boundary()
            .unwrap();
        assert_eq!(fixed[4], result[4]);
        for i in [0, 1, 2, 3, 5, 6, 7, 8] {
            assert_eq!(fixed[i], points[i]);
        }
        assert!((result[4][2] - 8. / 3.).abs() < 1e-6);
        assert_eq!(result[0], points[0]);
        assert_eq!(&result[1][..2], &points[1][..2]);
        assert!((result[1][2] - 4. / 3.).abs() < 1e-6);
        assert_eq!(
            result,
            SubdivisionSurface::new(&points, &quads)
                .unwrap()
                .limit_positions()
                .unwrap()
        );
    }

    #[test]
    fn extraordinary_cube_corners_have_exact_affine_covariant_limits() {
        let points = [
            [-1., -1., -1.],
            [1., -1., -1.],
            [1., 1., -1.],
            [-1., 1., -1.],
            [-1., -1., 1.],
            [1., -1., 1.],
            [1., 1., 1.],
            [-1., 1., 1.],
        ];
        let quads = [
            [0, 3, 2, 1],
            [4, 5, 6, 7],
            [0, 1, 5, 4],
            [1, 2, 6, 5],
            [2, 3, 7, 6],
            [3, 0, 4, 7],
        ];
        let limits = SubdivisionSurface::new(&points, &quads)
            .unwrap()
            .limit_positions()
            .unwrap();
        for (p, l) in points.iter().zip(&limits) {
            assert_eq!(p.map(|x| x * 0.5), *l);
        }
        let translated: Vec<_> = points.iter().map(|p| p.map(|x| 2. * x + 3.)).collect();
        let result = SubdivisionSurface::new(&translated, &quads)
            .unwrap()
            .limit_positions()
            .unwrap();
        for (a, b) in limits.iter().zip(result) {
            assert_eq!(a.map(|x| 2. * x + 3.), b);
        }
    }

    #[test]
    fn invalid_indices_winding_and_disconnected_fans_are_rejected() {
        let points = [[0., 0., 0.]; 7];
        assert!(SubdivisionSurface::new(&points, &[[0, 1, 2, 8]]).is_err());
        assert!(SubdivisionSurface::new(&points, &[[0, 1, 2, 2]]).is_err());
        for quads in [
            vec![[0, 1, 2, 3], [0, 1, 5, 4]],
            vec![[0, 1, 2, 3], [0, 4, 5, 6]],
        ] {
            assert!(
                SubdivisionSurface::new(&points, &quads)
                    .unwrap()
                    .limit_positions()
                    .is_err()
            );
        }
    }
}
