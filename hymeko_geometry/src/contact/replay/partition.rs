use super::{Limit, Method, Problem};
use hymeko_sbsh::{CountSplit, OrthConfig, build_orthtree, point_bbox};

pub(super) fn order(
    problem: &Problem,
    anchors: &[Option<[f64; 3]>],
    method: Method,
) -> (Vec<usize>, usize) {
    if method != Method::AdmmSbsh || anchors.is_empty() {
        return ((0..anchors.len()).collect(), 0);
    }
    let groups = groups(problem);
    let spatial: Vec<_> = groups
        .iter()
        .enumerate()
        .filter_map(|(i, g)| anchors[g[0]].map(|p| (i, p)))
        .collect();
    if spatial.is_empty() {
        return ((0..anchors.len()).collect(), 0);
    }
    let points: Vec<_> = spatial.iter().flat_map(|(_, p)| *p).collect();
    let (lo, hi) = point_bbox(&points, 3, 1e-6);
    let ids: Vec<_> = (0..spatial.len() as u32).collect();
    let tree = build_orthtree(
        3,
        &lo,
        &hi,
        &points,
        &ids,
        &CountSplit { max_per_cell: 2 },
        &OrthConfig {
            max_depth: 4,
            drop_empty: true,
        },
    );
    let mut out = Vec::with_capacity(anchors.len());
    let leaves = tree.leaves();
    for leaf in &leaves {
        for &member in &tree.members[*leaf] {
            out.extend_from_slice(&groups[spatial[member as usize].0]);
        }
    }
    for group in &groups {
        if anchors[group[0]].is_none() {
            out.extend_from_slice(group);
        }
    }
    (out, leaves.len())
}

fn groups(problem: &Problem) -> Vec<Vec<usize>> {
    problem
        .limits
        .iter()
        .enumerate()
        .filter(|(_, limit)| !matches!(limit, Limit::Friction { .. }))
        .map(|(k, _)| {
            let mut rows = vec![k];
            rows.extend(problem.limits.iter().enumerate().filter_map(|(i, limit)| {
                matches!(limit, Limit::Friction { normal, .. } if *normal == k).then_some(i)
            }));
            rows
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spatial_order_is_a_permutation_and_keeps_friction_blocks_whole() {
        let mut p = super::super::tests::impact(0.8, [0.0; 3]);
        p.limits.push(Limit::Pushing);
        let points = [Some([0.0, 0.0, 0.0]); 3]
            .into_iter()
            .chain([None])
            .collect::<Vec<_>>();
        let (order, patches) = order(&p, &points, Method::AdmmSbsh);
        assert_eq!(order, vec![0, 1, 2, 3]);
        assert_eq!(patches, 1);
        let (global, patches) = super::order(&p, &[None; 4], Method::AdmmSbsh);
        assert_eq!(global, order);
        assert_eq!(patches, 0);
    }

    #[test]
    fn multiple_leaves_reorder_whole_cones_without_losing_the_global_rows() {
        let base = super::super::tests::impact(0.8, [-1.0, 0.0, 0.0]);
        let mut p = base.clone();
        p.limits.clear();
        let mut anchors = Vec::new();
        for x in [3.0, -2.0, 1.0, -4.0, 5.0, -6.0] {
            let start = p.limits.len();
            p.limits.extend([
                Limit::Pushing,
                Limit::Friction {
                    normal: start,
                    partner: start + 2,
                    mu: 0.8,
                },
                Limit::Friction {
                    normal: start,
                    partner: start + 1,
                    mu: 0.8,
                },
            ]);
            anchors.extend([Some([x, 0.0, 0.0]); 3]);
        }
        p.limits.push(Limit::Free);
        anchors.push(None);
        let (indices, patches) = order(&p, &anchors, Method::AdmmSbsh);
        assert!(patches > 1);
        assert_ne!(indices, (0..19).collect::<Vec<_>>());
        let mut sorted = indices.clone();
        sorted.sort_unstable();
        assert_eq!(sorted, (0..19).collect::<Vec<_>>());
        for group in indices[..18].chunks_exact(3) {
            assert_eq!(group, &[group[0], group[0] + 1, group[0] + 2]);
            assert_eq!(group[0] % 3, 0);
        }
        assert_eq!(indices[18], 18);
    }
}
