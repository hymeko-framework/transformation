use hymeko_geometry::mesh::{
    MeshLod, TriangleMesh,
    normals::NormalSmoothing,
    regularize::{RegularizeOptions, SurfaceRegularizer},
};
use std::collections::BTreeMap;

fn grid(n: usize) -> TriangleMesh {
    let mut positions = Vec::new();
    let mut uv = Vec::new();
    let mut indices = Vec::new();
    for y in 0..=n {
        for x in 0..=n {
            let shift = if x > 0 && x < n && y > 0 && y < n {
                if (x + y) % 2 == 0 { 0.08 } else { -0.08 }
            } else {
                0.
            };
            positions.push([x as f32 + shift, y as f32, 0.]);
            uv.push([x as f32 / n as f32, y as f32 / n as f32]);
        }
    }
    for y in 0..n {
        for x in 0..n {
            let a = (y * (n + 1) + x) as u32;
            let stride = (n + 1) as u32;
            indices.extend([a, a + 1, a + stride + 1, a, a + stride + 1, a + stride]);
        }
    }
    TriangleMesh {
        normals: vec![[0., 0., 1.]; positions.len()],
        positions,
        uv,
        tangents: Vec::new(),
        lods: vec![MeshLod { indices, error: 0. }],
    }
}

#[test]
fn quad_source_correspondence_reaches_original_and_uv_alias_vertices() {
    let mut mesh = grid(2);
    mesh.positions.push(mesh.positions[4]);
    mesh.normals.push(mesh.normals[4]);
    mesh.uv.push([0.9, 0.9]);
    let options = RegularizeOptions {
        relaxation_steps: 0,
        ..Default::default()
    };
    let cage = SurfaceRegularizer::new(&mesh)
        .unwrap()
        .prepare(&mesh.lods[0].indices, &options)
        .unwrap();
    assert_eq!(cage.source_vertices.len(), mesh.positions.len());
    assert_eq!(cage.source_vertices[4], cage.source_vertices[9]);
    for (p, i) in mesh.positions.iter().zip(&cage.source_vertices) {
        assert_eq!(*p, cage.positions[i.unwrap() as usize]);
    }
    let samples =
        hymeko_geometry::mesh::subdivision::SubdivisionSurface::new(&cage.positions, &cage.quads)
            .unwrap()
            .limit_positions()
            .unwrap();
    assert!(samples.iter().all(|p| p[2] == 0.));
}

#[test]
fn authored_normals_follow_a_bend_without_becoming_raw_geometric_normals() {
    let mut original = grid(3);
    let scale = 1.04_f32.sqrt();
    let authored = [0.2 / scale, 0., 1. / scale];
    original.normals.fill(authored);
    let mut bent = original.clone();
    NormalSmoothing::follow_geometry(&original, &mut bent, 8, 85.).unwrap();
    assert_eq!(bent.normals, original.normals);
    let (s, c) = 0.6_f32.sin_cos();
    let rotate = |p: [f32; 3]| [c * p[0] + s * p[2], p[1], -s * p[0] + c * p[2]];
    for p in &mut bent.positions {
        *p = rotate(*p);
    }
    let points = bent.positions.clone();
    NormalSmoothing::follow_geometry(&original, &mut bent, 8, 85.).unwrap();
    let expected = rotate(authored);
    for n in &bent.normals {
        for k in 0..3 {
            assert!((n[k] - expected[k]).abs() < 2e-6);
        }
    }
    assert_eq!(bent.positions, points);
    assert_eq!(bent.uv, original.uv);
    let accepted = bent.normals.clone();
    bent.lods[0].indices.swap(0, 1);
    assert!(NormalSmoothing::follow_geometry(&original, &mut bent, 8, 85.).is_err());
    assert_eq!(bent.normals, accepted);
    bent = original.clone();
    for p in &mut bent.positions {
        p[0] = -p[0];
        p[2] = -p[2];
    }
    assert!(NormalSmoothing::follow_geometry(&original, &mut bent, 8, 85.).is_err());
    assert_eq!(bent.normals, original.normals);
}

#[test]
fn local_fitting_keeps_distant_progress_and_shared_aliases_without_flips() {
    use hymeko_geometry::mesh::fair::{BoundaryMode, FairOptions, FairStep, SurfaceFairing};
    let mut mesh = grid(4);
    // A tiny isolated triangle limits a uniform step, but must not limit the grid.
    let base = mesh.positions.len() as u32;
    mesh.positions
        .extend([[8., 0., 0.], [8.001, 0., 0.], [8., 0.001, 0.]]);
    mesh.normals.extend([[0., 0., 1.]; 3]);
    mesh.uv.extend([[0., 0.]; 3]);
    mesh.lods[0].indices.extend([base, base + 1, base + 2]);
    let alias = mesh.positions.len();
    mesh.positions.push(mesh.positions[12]);
    mesh.normals.push(mesh.normals[12]);
    mesh.uv.push([0., 0.]);
    let before = mesh.clone();
    let mut targets = mesh.positions.clone();
    targets[12][2] = 0.01;
    targets[alias] = targets[12];
    targets[base as usize][0] += 0.01;
    let options = FairOptions {
        max_displacement: 0.02,
        boundary_mode: BoundaryMode::Free,
        step: FairStep::Local,
        ..Default::default()
    };
    let report = SurfaceFairing::fit_targets(&mut mesh, &targets, &options, None).unwrap();
    assert!(report.step_scale < 0.25);
    assert_eq!(mesh.positions[12], targets[12]);
    assert_eq!(mesh.positions[alias], targets[12]);
    assert!(mesh.positions[base as usize][0] < mesh.positions[base as usize + 1][0]);
    assert_eq!(mesh.uv, before.uv);
    assert_eq!(mesh.lods[0].indices, before.lods[0].indices);
    let mut global = before;
    SurfaceFairing::fit_targets(
        &mut global,
        &targets,
        &FairOptions {
            step: FairStep::Global,
            ..options
        },
        None,
    )
    .unwrap();
    assert!(global.positions[12][2] < mesh.positions[12][2] * 0.25);
}

#[test]
fn local_backtracking_rechecks_connected_neighbours_and_is_transactional() {
    use hymeko_geometry::mesh::fair::{FairOptions, FairStep, SurfaceFairing};
    let mut mesh = grid(5);
    for p in &mut mesh.positions {
        *p = p.map(|v| v * 0.001);
    }
    let before = mesh.clone();
    let mut targets = mesh.positions.clone();
    for (i, p) in targets.iter_mut().enumerate() {
        p[0] += if i % 2 == 0 { 0.006 } else { -0.006 };
        p[1] += if i % 3 == 0 { 0.004 } else { -0.004 };
    }
    let options = FairOptions {
        max_displacement: 0.01,
        step: FairStep::Local,
        ..Default::default()
    };
    SurfaceFairing::fit_targets(&mut mesh, &targets, &options, None).unwrap();
    for face in mesh.lods[0].indices.chunks_exact(3) {
        let a = mesh.positions[face[0] as usize];
        let b = mesh.positions[face[1] as usize];
        let c = mesh.positions[face[2] as usize];
        assert!((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) > 0.);
    }
    for (i, p) in before.positions.iter().enumerate() {
        if i < 6 || i >= 30 || i % 6 == 0 || i % 6 == 5 {
            assert_eq!(*p, mesh.positions[i]);
        }
    }
    let accepted = mesh.positions.clone();
    targets[7][0] = f32::NAN;
    assert!(SurfaceFairing::fit_targets(&mut mesh, &targets, &options, None).is_err());
    assert_eq!(mesh.positions, accepted);
}

#[test]
fn supplied_limit_targets_share_fairing_bounds_and_transactional_refusal() {
    use hymeko_geometry::mesh::fair::{FairOptions, SurfaceFairing};
    let mut mesh = grid(2);
    let before = mesh.clone();
    let mut targets = mesh.positions.clone();
    targets[4][2] = 1.;
    let options = FairOptions {
        max_displacement: 0.01,
        ..Default::default()
    };
    let report = SurfaceFairing::fit_targets(&mut mesh, &targets, &options, None).unwrap();
    assert!(report.maximum_displacement > 0.009 && report.maximum_displacement <= 0.01);
    assert_eq!(mesh.uv, before.uv);
    assert_eq!(mesh.lods[0].indices, before.lods[0].indices);
    for i in [0, 1, 2, 3, 5, 6, 7, 8] {
        assert_eq!(mesh.positions[i], before.positions[i]);
    }
    let accepted = mesh.positions.clone();
    targets[0][0] = f32::NAN;
    assert!(SurfaceFairing::fit_targets(&mut mesh, &targets, &options, None).is_err());
    assert_eq!(mesh.positions, accepted);
    mesh.positions.push(mesh.positions[4]);
    mesh.normals.push(mesh.normals[4]);
    mesh.uv.push([0.2, 0.3]);
    targets = mesh.positions.clone();
    targets[9][2] += 0.1;
    let accepted = mesh.positions.clone();
    assert!(SurfaceFairing::fit_targets(&mut mesh, &targets, &options, None).is_err());
    assert_eq!(mesh.positions, accepted);
    // Reuse the aliased fixture to isolate normal-only fitting from tangential drift.
    targets[4] = [
        accepted[4][0] + 0.006,
        accepted[4][1] - 0.005,
        accepted[4][2] + 0.003,
    ];
    targets[9] = targets[4];
    SurfaceFairing::fit_targets(
        &mut mesh,
        &targets,
        &FairOptions {
            normal_displacement: true,
            ..options
        },
        None,
    )
    .unwrap();
    assert_eq!(&mesh.positions[4][..2], &accepted[4][..2]);
    assert!((mesh.positions[4][2] - accepted[4][2] - 0.003).abs() < 1e-7);
    assert_eq!(mesh.positions[4], mesh.positions[9]);
    assert_eq!(mesh.lods[0].indices, before.lods[0].indices);
    for i in [0, 1, 2, 3, 5, 6, 7, 8] {
        assert_eq!(mesh.positions[i], accepted[i]);
    }
}

#[test]
fn conforming_quads_retain_boundary_and_project_onto_source() {
    let mesh = grid(5);
    let options = RegularizeOptions {
        max_displacement: 0.1,
        ..Default::default()
    };
    let result = SurfaceRegularizer::new(&mesh)
        .unwrap()
        .prepare(&mesh.lods[0].indices, &options)
        .unwrap();
    assert!(!result.quads.is_empty());
    assert!(result.paired_control_faces > mesh.lods[0].indices.len() / 6);
    assert!(result.maximum_relaxation > 0. && result.maximum_relaxation <= 0.1 + 1e-9);
    assert!(result.positions.iter().all(|p| p[2].abs() < 1e-7));
    let mut edges = BTreeMap::new();
    for q in &result.quads {
        assert_eq!(q.iter().collect::<std::collections::BTreeSet<_>>().len(), 4);
        for k in 0..4 {
            let a = q[k];
            let b = q[(k + 1) % 4];
            *edges.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    assert!(edges.values().all(|&n| n <= 2));
    for (&(a, b), &count) in &edges {
        if count == 1 {
            let (p, q) = (result.positions[a as usize], result.positions[b as usize]);
            assert!(
                (p[0] == 0. && q[0] == 0.)
                    || (p[0] == 5. && q[0] == 5.)
                    || (p[1] == 0. && q[1] == 0.)
                    || (p[1] == 5. && q[1] == 5.)
            );
        }
    }
    assert_eq!(result.mesh.lods[0].indices.len(), result.quads.len() * 6);
    for (p, uv) in result.mesh.positions.iter().zip(&result.mesh.uv) {
        // UV projection follows the original piecewise-linear triangles.
        assert!((0. ..=1.).contains(&uv[0]) && (0. ..=1.).contains(&uv[1]));
        assert_eq!(p[2], 0.);
    }
    result.mesh.validate().unwrap();
    let again = SurfaceRegularizer::new(&mesh)
        .unwrap()
        .prepare(&mesh.lods[0].indices, &options)
        .unwrap();
    assert_eq!(result.quads, again.quads);
    assert_eq!(result.mesh.positions, again.mesh.positions);
    assert_eq!(result.mesh.uv, again.mesh.uv);
    assert_eq!(result.mesh.normals, again.mesh.normals);
}

#[test]
fn uv_seams_are_pinned_and_remain_distinct_corner_coordinates() {
    let mut mesh = grid(1);
    mesh.positions.extend_from_within(..);
    mesh.normals.extend_from_within(..);
    mesh.uv
        .extend([[0.1, 0.2], [0.3, 0.2], [0.1, 0.4], [0.3, 0.4]]);
    mesh.lods[0].indices = [0, 1, 3, 4, 7, 6].to_vec();
    let result = SurfaceRegularizer::new(&mesh)
        .unwrap()
        .prepare(&mesh.lods[0].indices, &Default::default())
        .unwrap();
    assert_eq!(
        result.paired_control_faces, 0,
        "a UV discontinuity must not be hidden inside a quad"
    );
    assert_eq!(result.quads.len(), 6);
    let seam: Vec<_> = result
        .mesh
        .positions
        .iter()
        .zip(&result.mesh.uv)
        .filter(|(p, _)| (p[0] - p[1]).abs() < 1e-6)
        .collect();
    assert!(seam.iter().any(|(_, uv)| uv[0] > 0.9));
    assert!(seam.iter().any(|(_, uv)| uv[0] < 0.4));
    assert!(result.mesh.normals.iter().all(|n| n[2] > 0.9999));
}

#[test]
fn normal_smoothing_preserves_positions_uvs_and_sharp_cones() {
    let mut mesh = grid(2);
    let positions = mesh.positions.clone();
    let uv = mesh.uv.clone();
    let indices = mesh.lods[0].indices.clone();
    for (i, n) in mesh.normals.iter_mut().enumerate() {
        *n = if i % 2 == 0 {
            [0.1, 0., 0.995]
        } else {
            [-0.1, 0., 0.995]
        };
    }
    NormalSmoothing::apply(&mut mesh, 8, 60.).unwrap();
    assert_eq!(mesh.positions, positions);
    assert_eq!(mesh.uv, uv);
    assert_eq!(mesh.lods[0].indices, indices);
    assert!(
        mesh.normals
            .iter()
            .all(|n| n[0].abs() < 1e-6 && n[2] > 0.9999)
    );
    let mut corner = TriangleMesh {
        positions: vec![
            [0., 0., 0.],
            [1., 0., 0.],
            [0., 1., 0.],
            [0., 0., 0.],
            [0., 1., 0.],
            [0., 0., 1.],
        ],
        normals: vec![
            [0., 0., 1.],
            [0., 0., 1.],
            [0., 0., 1.],
            [1., 0., 0.],
            [1., 0., 0.],
            [1., 0., 0.],
        ],
        uv: Vec::new(),
        tangents: Vec::new(),
        lods: vec![MeshLod {
            indices: vec![0, 1, 2, 3, 4, 5],
            error: 0.,
        }],
    };
    let normals = corner.normals.clone();
    NormalSmoothing::apply(&mut corner, 12, 45.).unwrap();
    assert_eq!(corner.normals, normals);
}

#[test]
fn invalid_and_nonmanifold_surfaces_fail_before_output() {
    let mut mesh = grid(1);
    mesh.lods[0].indices.extend([0, 1, 3]);
    assert!(
        SurfaceRegularizer::new(&mesh)
            .unwrap()
            .prepare(&mesh.lods[0].indices, &Default::default())
            .is_err()
    );
    let mesh = grid(1);
    for options in [
        RegularizeOptions {
            relaxation: f64::NAN,
            ..Default::default()
        },
        RegularizeOptions {
            normal_steps: 33,
            ..Default::default()
        },
    ] {
        assert!(
            SurfaceRegularizer::new(&mesh)
                .unwrap()
                .prepare(&mesh.lods[0].indices, &options)
                .is_err()
        );
    }
    assert!(
        SurfaceRegularizer::new(&mesh)
            .unwrap()
            .prepare(&[0, 1, u32::MAX], &Default::default())
            .is_err()
    );
}
