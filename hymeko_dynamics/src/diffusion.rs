//! Scalar continuum diffusion projected from G-SPHF octree declarations.
//! Graph owns subdivision/identity, Tensor owns the numerical operator.
use hymeko::{
    common::ids::DeclId,
    ir::ir::{Ir, ValueR},
    resolution::interner::Interner,
};
use hymeko_graph::octree::{Octree, field};
use hymeko_numerics::diffusion::Diffusion;

/// A closed-boundary diffusion experiment over an authored hierarchy in SI units.
#[derive(Clone)]
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
    revision: u64,
    policy: Option<Adaptivity>,
    adaptive: bool,
    next_adapt: f64,
}

#[derive(Clone)]
struct Adaptivity {
    interval: f64,
    refine_jump: f64,
    coarsen_jump: f64,
    initial_depth: usize,
    min_depth: usize,
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

    /// Revision of the active topology. Physical stepping alone does not change it.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    /// Change active cells by canonical declaration identity, preserving time and quantity.
    /// A stale revision or failed projection leaves the entire system unchanged.
    pub fn adapt(
        &mut self,
        expected_revision: u64,
        refine: &[DeclId],
        coarsen: &[DeclId],
    ) -> Result<bool, String> {
        if expected_revision != self.revision {
            return Err("stale octree topology revision".into());
        }
        let indexes = |ids: &[DeclId]| {
            ids.iter()
                .map(|id| {
                    self.tree
                        .cells
                        .iter()
                        .position(|c| c.decl == *id)
                        .ok_or_else(|| "unknown cell declaration".to_owned())
                })
                .collect::<Result<Vec<_>, String>>()
        };
        let remesh = self.tree.adapt(&indexes(refine)?, &indexes(coarsen)?)?;
        if remesh.tree.leaves == self.tree.leaves {
            return Ok(false);
        }
        let revision = self
            .revision
            .checked_add(1)
            .ok_or("topology revision exhausted")?;
        let operator = Self::operator(&remesh.tree, self.diffusivity)?;
        let state = self
            .operator
            .remap(&self.state, &operator, &remesh.overlaps)
            .map_err(str::to_owned)?;
        self.tree = remesh.tree;
        self.state = state;
        self.operator = operator;
        self.revision = revision;
        Ok(true)
    }

    /// Enable the model's declared policy and conservatively project its initial depth.
    /// Preparation is atomic and only allowed at physical time zero.
    pub fn enable_adaptivity(&mut self) -> Result<(), String> {
        if self.time != 0.0 || self.adaptive {
            return Err("adaptivity must be configured once before stepping".into());
        }
        let policy = self
            .policy
            .as_ref()
            .ok_or("model has no adaptivity declaration")?
            .clone();
        let mut candidate = self.clone();
        for _ in 0..16 {
            let coarsen: Vec<_> = candidate
                .tree
                .cells
                .iter()
                .filter(|c| {
                    c.depth >= policy.initial_depth
                        && c.children.len() == 8
                        && c.children.iter().all(|i| candidate.tree.leaves.contains(i))
                })
                .map(|c| c.decl)
                .collect();
            if coarsen.is_empty() || !candidate.adapt(candidate.revision, &[], &coarsen)? {
                break;
            }
        }
        candidate.adaptive = true;
        candidate.next_adapt = policy.interval;
        *self = candidate;
        Ok(())
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
        for &i in &tree.leaves {
            let cell = &tree.cells[i];
            let c = numeric(cell.decl, "concentration")?;
            if c < 0.0 {
                return Err("concentration must be nonnegative".into());
            }
            state.push(c);
        }
        let policy = ir
            .decl_children(scope)
            .find(|&id| it.resolve(ir.decl_nodes[id.raw()].name) == "adaptivity")
            .map(|id| {
                let interval = numeric(id, "interval")?;
                let refine_jump = numeric(id, "refine_jump")?;
                let coarsen_jump = numeric(id, "coarsen_jump")?;
                let initial_depth = numeric(id, "initial_depth")?;
                let min_depth = numeric(id, "min_depth")?;
                if !(0.01..=3600.0).contains(&interval)
                    || coarsen_jump < 0.0
                    || refine_jump <= coarsen_jump
                    || min_depth < 0.0
                    || initial_depth < min_depth
                    || initial_depth > 16.0
                    || initial_depth.fract() != 0.0
                    || min_depth.fract() != 0.0
                {
                    return Err("invalid adaptivity interval, hysteresis or depth".to_owned());
                }
                Ok(Adaptivity {
                    interval,
                    refine_jump,
                    coarsen_jump,
                    initial_depth: initial_depth as usize,
                    min_depth: min_depth as usize,
                })
            })
            .transpose()?;
        let operator = Self::operator(&tree, diffusivity)?;
        Ok(Self {
            tree,
            state,
            time: 0.0,
            diffusivity,
            operator,
            revision: 0,
            policy,
            adaptive: false,
            next_adapt: 0.0,
        })
    }

    fn operator(tree: &Octree, diffusivity: f64) -> Result<Diffusion, String> {
        // Closed boundary: only shared faces exchange material. Each coarse/fine
        // subface appears once; conductance uses normal centre distance.
        let edges = tree
            .faces()
            .into_iter()
            .map(|(a, b, area, d)| (a, b, diffusivity * area / d))
            .collect();
        let capacity = tree
            .leaves
            .iter()
            .map(|&i| tree.cells[i].volume())
            .collect();
        Diffusion::new(capacity, edges).map_err(str::to_owned)
    }
    /// Advance atomically; failed or invalid steps leave state and time unchanged.
    pub fn step(&mut self, dt: f64) -> Result<(), String> {
        if !dt.is_finite() || dt <= 0.0 || !(self.time + dt).is_finite() {
            return Err("invalid physical time step".into());
        }
        if self.adaptive && self.time + dt >= self.next_adapt {
            let mut candidate = self.clone();
            candidate.advance_fixed(dt)?;
            candidate.apply_policy()?;
            let interval = candidate
                .policy
                .as_ref()
                .ok_or("missing active policy")?
                .interval;
            candidate.next_adapt = candidate.time + interval;
            *self = candidate;
            return Ok(());
        }
        self.advance_fixed(dt)
    }

    fn advance_fixed(&mut self, dt: f64) -> Result<(), String> {
        self.operator
            .step(&mut self.state, dt, 1e-11)
            .map_err(str::to_owned)?;
        self.time += dt;
        Ok(())
    }

    fn apply_policy(&mut self) -> Result<(), String> {
        let policy = self.policy.as_ref().ok_or("missing active policy")?;
        let mut jump = vec![0.0_f64; self.state.len()];
        for (a, b, _, _) in self.tree.faces() {
            let error = (self.state[a] - self.state[b]).abs();
            jump[a] = jump[a].max(error);
            jump[b] = jump[b].max(error);
        }
        let refine: Vec<_> = self
            .tree
            .leaves
            .iter()
            .enumerate()
            .filter(|(a, i)| {
                jump[*a] > policy.refine_jump && !self.tree.cells[**i].children.is_empty()
            })
            .map(|(_, i)| self.tree.cells[*i].decl)
            .collect();
        let mut coarsen = Vec::new();
        for cell in &self.tree.cells {
            if cell.depth < policy.min_depth || cell.children.len() != 8 {
                continue;
            }
            let indexes: Option<Vec<_>> = cell
                .children
                .iter()
                .map(|i| self.tree.leaves.iter().position(|j| j == i))
                .collect();
            if let Some(indexes) = indexes {
                if indexes.iter().all(|&i| jump[i] < policy.coarsen_jump) {
                    coarsen.push(cell.decl);
                }
            }
        }
        if !refine.is_empty() || !coarsen.is_empty() {
            self.adapt(self.revision, &refine, &coarsen)?;
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use hymeko::{module_store::module_store::ModuleStore, util::real_parser::RealParser};
    use hymeko_sources::MemoryProvider;

    #[test]
    fn policy_is_read_by_core_and_rejects_invalid_hysteresis_or_depth() {
        let source = "fixture {}\ndomain { diffusivity 0.1; adaptivity { interval 0.25; refine_jump 0.1; coarsen_jump 0.01; initial_depth 0; min_depth 0; } root { lo [0,0,0]; hi [1,1,1]; concentration 1; } }";
        let compile = |text: &str| {
            let mut store =
                ModuleStore::new(MemoryProvider::new().with("model.hymeko", text), RealParser);
            let p = store.compile(std::path::Path::new("model.hymeko")).unwrap();
            let scope =
                p.ir.decls()
                    .find(|(_, n)| store.it.resolve(n.name) == "domain")
                    .unwrap()
                    .0;
            DiffusionSystem::project(&p.ir, &store.it, scope)
        };
        let mut valid = compile(source).unwrap();
        valid.enable_adaptivity().unwrap();
        assert!(valid.enable_adaptivity().is_err());
        valid.step(0.5).unwrap();
        assert_eq!(valid.state(), &[1.0]);
        assert_eq!(valid.revision(), 0);
        assert_eq!(valid.diffusivity(), 0.1);
        for bad in [
            source.replace("interval 0.25", "interval 0"),
            source.replace("coarsen_jump 0.01", "coarsen_jump 0.2"),
            source.replace("initial_depth 0", "initial_depth 0.5"),
            source.replace("min_depth 0", "min_depth 1"),
        ] {
            assert!(compile(&bad).is_err());
        }
        let mut no_policy = compile("fixture {}\ndomain { diffusivity 0.1; root { lo [0,0,0]; hi [1,1,1]; concentration 1; } }").unwrap();
        assert!(no_policy.enable_adaptivity().is_err());
    }
}
