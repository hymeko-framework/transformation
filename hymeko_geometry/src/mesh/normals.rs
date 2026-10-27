//! Normal diffusion over geometry seams, preserving separate crease cones.
use super::{TriangleMesh, topology::*};
use std::collections::{BTreeMap, BTreeSet};

/// Smooth a normal field without moving positions, UVs, or triangle indices.
pub struct NormalSmoothing;

impl NormalSmoothing {
    /// Retain an authored shading field while following a small geometric bend.
    /// Uses the shortest rotation between filtered geometric normals, not a full
    /// deformation Jacobian (in-plane twist cannot be recovered from normals).
    /// Vertex/index correspondence is required; output is transactional.
    pub fn follow_geometry(
        reference: &TriangleMesh,
        deformed: &mut TriangleMesh,
        iterations: usize,
        crease_degrees: f32,
    ) -> Result<(), String> {
        reference.validate()?;
        deformed.validate()?;
        if reference.positions.len() != deformed.positions.len()
            || reference.lods[0].indices != deformed.lods[0].indices
        {
            return Err("normal following requires the same vertex and triangle domain".into());
        }
        if iterations == 0 {
            return Err("normal following needs at least one filter step".into());
        }
        for face in reference.lods[0].indices.chunks_exact(3) {
            let normal = |mesh: &TriangleMesh| {
                let p = [face[0], face[1], face[2]].map(|i| point(mesh.positions[i as usize]));
                cross(sub(p[1], p[0]), sub(p[2], p[0]))
            };
            let original = normal(reference);
            let changed = normal(deformed);
            if length(original) > 1e-14
                && (length(changed) <= 1e-14 || dot(original, changed) <= 0.)
            {
                return Err("normal following requires non-inverted small bends".into());
            }
        }
        let mut before = reference.clone();
        let mut after = deformed.clone();
        // Identical cone references keep UV/crease decisions comparable.
        after.normals.clone_from(&reference.normals);
        Self::apply(&mut before, iterations, crease_degrees)?;
        Self::apply(&mut after, iterations, crease_degrees)?;
        let mut output = Vec::with_capacity(reference.positions.len());
        for ((&a, &b), &authored) in before
            .normals
            .iter()
            .zip(&after.normals)
            .zip(&reference.normals)
        {
            if a == b {
                output.push(authored);
                continue;
            }
            let from = unit(point(a));
            let to = unit(point(b));
            let cosine = dot(from, to).clamp(-1., 1.);
            if cosine < -0.9999 {
                return Err("normal following cannot resolve an antipodal bend".into());
            }
            let axis = cross(from, to);
            let normal = point(authored);
            let first = cross(axis, normal);
            output.push(
                unit(add(
                    add(normal, first),
                    mul(cross(axis, first), 1. / (1. + cosine)),
                ))
                .map(|v| v as f32),
            );
        }
        if deformed.normals != output {
            deformed.tangents.clear();
        }
        deformed.normals = output;
        Ok(())
    }

    /// Shared-position UV duplicates share a normal cone; hard crease cones
    /// remain independent. Iterations are bounded and all output normals unit.
    pub fn apply(
        mesh: &mut TriangleMesh,
        iterations: usize,
        crease_degrees: f32,
    ) -> Result<(), String> {
        if iterations > 32 || !crease_degrees.is_finite() || !(5. ..=85.).contains(&crease_degrees)
        {
            return Err("normal smoothing requires 0..32 steps and a5..85 degree crease".into());
        }
        mesh.validate()?;
        if iterations == 0 {
            return Ok(());
        }
        let cosine = (crease_degrees as f64).to_radians().cos();
        let topology = Topology::for_processing(mesh, cosine)?;
        let mut cones: Vec<Vec<usize>> = vec![Vec::new(); topology.positions.len()];
        let mut reference: Vec<P> = Vec::new();
        let mut membership = vec![0; mesh.positions.len()];
        for (i, &a) in topology.alias.iter().enumerate() {
            let n = unit(point(mesh.normals[i]));
            let group = cones[a]
                .iter()
                .copied()
                .find(|&g| dot(reference[g], n) >= cosine)
                .unwrap_or_else(|| {
                    let g = reference.len();
                    reference.push(n);
                    cones[a].push(g);
                    g
                });
            membership[i] = group;
        }
        let mut normals = vec![[0.; 3]; reference.len()];
        let mut neighbors: Vec<BTreeMap<usize, f64>> = vec![BTreeMap::new(); reference.len()];
        for (i, face) in topology.faces.iter().enumerate() {
            let raw = topology.raw[i];
            let p = face.map(|v| topology.positions[v]);
            let area = length(cross(sub(p[1], p[0]), sub(p[2], p[0])));
            // One face contributes once per shared-position normal cone,
            // independently of how many UV records refer to that point.
            for k in 0..3 {
                for &g in &cones[face[k]] {
                    if dot(reference[g], topology.normals[i]) >= cosine {
                        normals[g] = add(normals[g], mul(topology.normals[i], area));
                    }
                }
                let a = membership[raw[k] as usize];
                let b = membership[raw[(k + 1) % 3] as usize];
                if a != b && dot(reference[a], reference[b]) >= cosine {
                    let weight = 1. / length(sub(p[k], p[(k + 1) % 3])).max(1e-12);
                    neighbors[a].insert(b, weight);
                    neighbors[b].insert(a, weight);
                }
            }
        }
        for (n, &fallback) in normals.iter_mut().zip(&reference) {
            *n = if length(*n) > 1e-20 {
                unit(*n)
            } else {
                fallback
            };
        }
        let active: BTreeSet<_> = neighbors
            .iter()
            .enumerate()
            .filter_map(|(i, n)| (!n.is_empty()).then_some(i))
            .collect();
        for _ in 0..iterations {
            let previous = normals.clone();
            for &i in &active {
                let mut mean = [0.; 3];
                let mut total = 0.;
                for (&j, &weight) in &neighbors[i] {
                    let conductance =
                        ((dot(previous[i], previous[j]) - cosine) / (1. - cosine)).clamp(0., 1.);
                    let w = weight * conductance * conductance;
                    mean = add(mean, mul(previous[j], w));
                    total += w;
                }
                if total > 0. {
                    normals[i] = unit(add(mul(previous[i], 0.65), mul(mean, 0.35 / total)));
                }
            }
        }
        let output = membership
            .iter()
            .map(|&g| normals[g].map(|v| v as f32))
            .collect();
        drop(topology);
        if mesh.normals != output {
            mesh.tangents.clear();
        }
        mesh.normals = output;
        mesh.validate()
    }
}
