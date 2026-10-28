//! Topology-preserving surface fairing. Input/output stay in the same vertex/UV domain.
use super::{TriangleMesh, topology::*};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BoundaryMode {
    #[default]
    Pin,
    Free,
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FairNormals {
    #[default]
    Changed,
    Rebuild,
}

/// Scope of orientation backtracking; local steps do not throttle distant patches.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FairStep {
    #[default]
    Global,
    Local,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct FairOptions {
    pub iterations: usize,
    pub relaxation: f64,
    pub recovery: f64,
    pub max_displacement: f64,
    pub crease_degrees: f32,
    pub boundary_mode: BoundaryMode,
    pub normals: FairNormals,
    pub step: FairStep,
    /// Remove tangential drift using the authored input normal at each alias.
    /// Useful when vertex-attached attributes must keep their surface locations.
    pub normal_displacement: bool,
    /// Optional smooth box support: [lower xyz, upper xyz]. Outside stays exact.
    pub bounds: Option<[[f64; 3]; 2]>,
    /// Metre falloff from each box face; zero selects a hard region.
    pub feather: f64,
}
impl Default for FairOptions {
    fn default() -> Self {
        Self {
            iterations: 16,
            relaxation: 0.5,
            recovery: 0.53,
            max_displacement: 0.003,
            crease_degrees: 65.,
            boundary_mode: BoundaryMode::Pin,
            normals: FairNormals::Changed,
            step: FairStep::Global,
            normal_displacement: false,
            bounds: None,
            feather: 0.005,
        }
    }
}
impl FairOptions {
    pub fn validate(&self) -> Result<(), String> {
        if self.iterations > 96
            || !self.relaxation.is_finite()
            || !(0.0..=0.5).contains(&self.relaxation)
            || !self.recovery.is_finite()
            || !(0.0..=0.6).contains(&self.recovery)
            || !self.max_displacement.is_finite()
            || !(0.0..=0.05).contains(&self.max_displacement)
            || !self.crease_degrees.is_finite()
            || !(5.0..=85.).contains(&self.crease_degrees)
            || !self.feather.is_finite()
            || !(0.0..=1.).contains(&self.feather)
        {
            return Err(
                "fairing exceeds its iteration, relaxation, displacement or crease limits".into(),
            );
        }
        if let Some([lo, hi]) = self.bounds
            && (0..3).any(|i| !lo[i].is_finite() || !hi[i].is_finite() || lo[i] >= hi[i])
        {
            return Err("fairing needs finite ordered bounds".into());
        }
        Ok(())
    }
    fn weight(&self, p: P) -> f64 {
        let Some([lo, hi]) = self.bounds else {
            return 1.;
        };
        (0..3)
            .map(|i| {
                let distance = (p[i] - lo[i]).min(hi[i] - p[i]);
                if distance <= 0. {
                    return 0.;
                }
                if self.feather == 0. {
                    return 1.;
                }
                let t = (distance / self.feather).clamp(0., 1.);
                t * t * (3. - 2. * t)
            })
            .product()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FairReport {
    pub moved_vertices: usize,
    pub pinned_vertices: usize,
    pub maximum_displacement: f64,
    /// Minimum backtracking fraction admitted by the triangle-orientation check.
    pub step_scale: f64,
    pub skipped_degenerate_triangles: usize,
}

pub struct SurfaceFairing;
impl SurfaceFairing {
    /// Transactional: validates topology and orientation before replacing any stream.
    /// Shared-position aliases have one displacement; pinned source rows stay exact.
    pub fn apply(mesh: &mut TriangleMesh, options: &FairOptions) -> Result<FairReport, String> {
        Self::apply_weighted(mesh, options, None)
    }

    /// Optional per-source-vertex influence. Position aliases share their maximum
    /// influence, so attribute seams cannot crack under a region brush.
    pub fn apply_weighted(
        mesh: &mut TriangleMesh,
        options: &FairOptions,
        influence: Option<&[f64]>,
    ) -> Result<FairReport, String> {
        Self::process(mesh, options, influence, None)
    }

    /// Fit a supplied target field with the same displacement/orientation guards.
    /// Targets must agree at exact position aliases. Iteration settings are ignored;
    /// influence and box support blend the target before the displacement bound.
    pub fn fit_targets(
        mesh: &mut TriangleMesh,
        targets: &[[f32; 3]],
        options: &FairOptions,
        influence: Option<&[f64]>,
    ) -> Result<FairReport, String> {
        if targets.len() != mesh.positions.len() || targets.iter().flatten().any(|v| !v.is_finite())
        {
            return Err("target positions must be finite and match the source".into());
        }
        Self::process(mesh, options, influence, Some(targets))
    }

    fn process(
        mesh: &mut TriangleMesh,
        options: &FairOptions,
        influence: Option<&[f64]>,
        targets: Option<&[[f32; 3]]>,
    ) -> Result<FairReport, String> {
        options.validate()?;
        // Rebuilding normals admits unused attribute aliases with no input
        // normal. Temporary directions are discarded; the result is computed
        // from connected geometric faces and validated before committing.
        let mut admitted = mesh.clone();
        if matches!(options.normals, FairNormals::Rebuild) {
            for n in &mut admitted.normals {
                if n.iter().all(|x| x.is_finite()) && n.iter().all(|x| *x == 0.) {
                    *n = [0., 0., 1.];
                }
            }
        }
        admitted.validate()?;
        if influence.is_some_and(|w| {
            w.len() != mesh.positions.len()
                || w.iter().any(|v| !v.is_finite() || !(0.0..=1.).contains(v))
        }) {
            return Err("fairing influence must match vertices and lie in 0..1".into());
        }
        if mesh.positions.len() > 250_000 || mesh.lods[0].indices.len() / 3 > 500_000 {
            return Err("fairing is bounded to 250000 vertices and 500000 triangles".into());
        }
        let topology = Topology::for_processing(
            &admitted,
            (options.crease_degrees as f64).to_radians().cos(),
        )?;
        let original = &topology.positions;
        let mut positions = original.clone();
        let mut authored = vec![if influence.is_some() { 0f64 } else { 1. }; positions.len()];
        if let Some(w) = influence {
            for (i, &alias) in topology.alias.iter().enumerate() {
                authored[alias] = authored[alias].max(w[i]);
            }
        }
        let mut neighbors = vec![BTreeSet::new(); positions.len()];
        for &(a, b) in topology.edges.keys() {
            neighbors[a].insert(b);
            neighbors[b].insert(a);
        }
        let weights: Vec<_> = original
            .iter()
            .enumerate()
            .map(|(i, &p)| {
                if matches!(options.boundary_mode, BoundaryMode::Pin) && topology.pinned[i] {
                    0.
                } else {
                    options.weight(p) * authored[i]
                }
            })
            .collect();
        if let Some(targets) = targets {
            let mut shared = vec![None; positions.len()];
            for (i, &alias) in topology.alias.iter().enumerate() {
                if shared[alias].is_some_and(|p| p != targets[i]) {
                    return Err("target positions disagree across a shared-position seam".into());
                }
                shared[alias] = Some(targets[i]);
                let delta = mul(sub(point(targets[i]), original[alias]), weights[alias]);
                let bound = (options.max_displacement
                    - 4. * f32::EPSILON as f64 * length(original[alias]).max(1.))
                .max(0.);
                positions[alias] = add(
                    original[alias],
                    mul(delta, (bound / length(delta).max(1e-30)).min(1.)),
                );
            }
        }
        for _ in 0..if targets.is_some() {
            0
        } else {
            options.iterations
        } {
            for step in [options.relaxation, -options.recovery] {
                let previous = positions.clone();
                for (i, adjacent) in neighbors.iter().enumerate() {
                    if weights[i] == 0. || adjacent.is_empty() {
                        continue;
                    }
                    let mean = mul(
                        adjacent.iter().fold([0.; 3], |s, &j| add(s, previous[j])),
                        1. / adjacent.len() as f64,
                    );
                    let target = add(previous[i], mul(sub(mean, previous[i]), step * weights[i]));
                    let delta = sub(target, original[i]);
                    positions[i] = add(
                        original[i],
                        mul(
                            delta,
                            ((options.max_displacement
                                - 4. * f32::EPSILON as f64 * length(original[i]).max(1.))
                            .max(0.)
                                / length(delta).max(1e-30))
                            .min(1.),
                        ),
                    );
                }
            }
        }
        let mut delta: Vec<_> = positions
            .iter()
            .zip(original)
            .map(|(&p, &q)| sub(p, q))
            .collect();
        if options.normal_displacement {
            let mut normals = vec![[0.; 3]; positions.len()];
            for (i, &alias) in topology.alias.iter().enumerate() {
                normals[alias] = add(normals[alias], unit(point(admitted.normals[i])));
            }
            for (d, n) in delta.iter_mut().zip(normals) {
                let n = unit(n);
                *d = mul(n, dot(*d, n));
            }
        }
        let mut scales = vec![1.; positions.len()];
        loop {
            for i in 0..positions.len() {
                positions[i] = add(original[i], mul(delta[i], scales[i])).map(|v| v as f32 as f64);
            }
            let mut rejected = BTreeSet::new();
            for (i, f) in topology.faces.iter().enumerate() {
                let p = f.map(|v| positions[v]);
                let n = cross(sub(p[1], p[0]), sub(p[2], p[0]));
                if length(n) <= 1e-14 || dot(n, topology.normals[i]) <= 0. {
                    rejected.extend(f.iter().copied());
                }
            }
            if rejected.is_empty() {
                break;
            }
            if matches!(options.step, FairStep::Global) {
                if scales[0] <= 1. / 65536. {
                    return Err(
                        "fairing cannot preserve triangle orientation within the step budget"
                            .into(),
                    );
                }
                for scale in &mut scales {
                    *scale *= 0.5;
                }
            } else {
                // Every rejected face shrinks all of its displacements together.
                // Recheck the whole mesh: reducing one step can affect a neighbour.
                // Below the budget, restore the exact input; finite propagation
                // terminates at the already validated original surface.
                for i in rejected {
                    scales[i] = if scales[i] > 1. / 65536. {
                        scales[i] * 0.5
                    } else {
                        0.
                    };
                }
            }
        }
        let mut accumulated = vec![[0.; 3]; positions.len()];
        for (i, f) in topology.faces.iter().enumerate() {
            let p = f.map(|v| positions[v]);
            let normal = cross(sub(p[1], p[0]), sub(p[2], p[0]));
            if length(normal) <= 1e-14 || dot(normal, topology.normals[i]) <= 0. {
                return Err(format!(
                    "fairing folds triangle {i}; reduce relaxation or displacement"
                ));
            }
            for &v in f {
                accumulated[v] = add(accumulated[v], normal);
            }
        }
        let mut output = mesh.clone();
        let mut report = FairReport {
            moved_vertices: 0,
            pinned_vertices: 0,
            maximum_displacement: 0.,
            step_scale: scales.iter().copied().fold(1., f64::min),
            skipped_degenerate_triangles: mesh.lods[0].indices.len() / 3 - topology.faces.len(),
        };
        for (i, &alias) in topology.alias.iter().enumerate() {
            let delta = length(sub(positions[alias], original[alias]));
            report.maximum_displacement = report.maximum_displacement.max(delta);
            let normal = if length(accumulated[alias]) > 1e-30 {
                unit(accumulated[alias]).map(|v| v as f32)
            } else {
                admitted.normals[i]
            };
            if matches!(options.normals, FairNormals::Rebuild) {
                output.normals[i] = normal;
            }
            if delta == 0. {
                report.pinned_vertices += 1;
                continue;
            }
            report.moved_vertices += 1;
            output.positions[i] = positions[alias].map(|v| v as f32);
            output.normals[i] = normal;
        }
        // Position edits invalidate precomputed simplification error bounds.
        if report.moved_vertices > 0 {
            output.lods.truncate(1);
        }
        if report.moved_vertices > 0 || output.normals != mesh.normals {
            output.tangents.clear();
        }
        output.validate()?;
        drop(topology);
        *mesh = output;
        Ok(report)
    }
}
