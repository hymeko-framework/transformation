//! Scalar continuum diffusion projected from G-SPHF octree declarations.
//! Graph owns subdivision/identity, Tensor owns the numerical operator.
use hymeko::{
    common::ids::DeclId,
    ir::ir::{Ir, ValueR},
    resolution::interner::Interner,
};
use hymeko_graph::octree::{Octree, field};
use hymeko_numerics::diffusion::Diffusion;

/// A fixed-topology, closed-boundary diffusion experiment in SI units.
pub struct DiffusionSystem {
    /// Core-derived octree; internal cells are organizers only.
    tree: Octree,
    /// Concentrations parallel to `tree.leaves`.
    state: Vec<f64>,
    /// Physical time in seconds.
    time: f64,
    /// Scalar diffusivity in m^2/s.
    diffusivity: f64,
    operator: Diffusion,
}
impl DiffusionSystem {
    /// Immutable topology; changing a domain requires a fresh projection.
    pub fn tree(&self) -> &Octree {
        &self.tree
    }
    /// Read-only physical state, parallel to the active leaves.
    pub fn state(&self) -> &[f64] {
        &self.state
    }
    /// Time of the current state.
    pub fn time(&self) -> f64 {
        self.time
    }
    /// Authored scalar diffusion coefficient.
    pub fn diffusivity(&self) -> f64 {
        self.diffusivity
    }

    /// Read a declared domain. Initial concentration is authored on every leaf.
    pub fn project(ir: &Ir, it: &Interner, scope: DeclId) -> Result<Self, String> {
        let tree = Octree::from_ir(ir, it, scope)?;
        let numeric = |decl, key| match field(ir, it, decl, key) {
            Some(ValueR::Num(v)) if v.is_finite() => Ok(*v),
            _ => Err(format!("{key} must be a finite number")),
        };
        let diffusivity = numeric(scope, "diffusivity")?;
        if diffusivity <= 0.0 {
            return Err("diffusivity must be positive".into());
        }
        let mut state = Vec::new();
        let mut capacity = Vec::new();
        for &i in &tree.leaves {
            let cell = &tree.cells[i];
            let c = numeric(cell.decl, "concentration")?;
            if c < 0.0 {
                return Err("concentration must be nonnegative".into());
            }
            state.push(c);
            capacity.push(cell.volume());
        }
        // Closed boundary: only shared faces exchange material. Each coarse/fine
        // subface appears once; conductance uses normal centre distance.
        let edges = tree
            .faces()
            .into_iter()
            .map(|(a, b, area, d)| (a, b, diffusivity * area / d))
            .collect();
        let operator = Diffusion::new(capacity, edges).map_err(str::to_owned)?;
        Ok(Self {
            tree,
            state,
            time: 0.0,
            diffusivity,
            operator,
        })
    }
    /// Advance atomically; failed or invalid steps leave state and time unchanged.
    pub fn step(&mut self, dt: f64) -> Result<(), String> {
        if !dt.is_finite() || dt <= 0.0 || !(self.time + dt).is_finite() {
            return Err("invalid physical time step".into());
        }
        self.operator
            .step(&mut self.state, dt, 1e-11)
            .map_err(str::to_owned)?;
        self.time += dt;
        Ok(())
    }
    /// Total transported quantity.
    pub fn quantity(&self) -> f64 {
        self.operator.quantity(&self.state)
    }
    /// Dissipating quadratic functional; not labelled thermodynamic energy.
    pub fn energy(&self) -> f64 {
        self.operator.energy(&self.state)
    }
}
