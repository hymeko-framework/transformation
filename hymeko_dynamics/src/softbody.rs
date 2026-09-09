//! Small-displacement elastic networks projected from canonical signed Core IR.
//! Spring extensions and particle momenta use Tensor's existing pH integrator.
use crate::read::{number, vector};
use hymeko::{
    common::ids::DeclId,
    ir::ir::{Ir, ValueR},
    resolution::interner::Interner,
};
use hymeko_graph::{
    octree::field,
    sphf::{Role, SignedHypergraph},
};
use hymeko_numerics::{Interconnection, Port, PortHamiltonian};
use std::collections::{HashMap, HashSet};

/// An authored particle; fixed particles have reaction forces but no momentum state.
#[derive(Clone, Debug)]
pub struct Particle {
    pub decl: DeclId,
    pub name: String,
    pub rest: [f64; 3],
    pub mass: f64,
    pub fixed: bool,
}
/// One linear axial spring, oriented from source to target in the reference shape.
#[derive(Clone, Debug)]
pub struct Spring {
    pub decl: DeclId,
    pub ends: [usize; 2],
    pub direction: [f64; 3],
    pub stiffness: f64,
}

/// Deformable small-displacement truss; geometry is derived from the same pH state.
pub struct SoftBody {
    particles: Vec<Particle>,
    springs: Vec<Spring>,
    triangles: Vec<[usize; 3]>,
    positions: Vec<[f64; 3]>,
    momentum: Vec<Option<usize>>,
    state: Vec<f64>,
    system: PortHamiltonian,
    max_displacement: f64,
    time: f64,
    balance_error: f64,
}

impl SoftBody {
    /// Read particles with `position`/`mass`, signed `spring` edges and `surface` faces.
    /// Reference directions are fixed: this is not a finite-strain material law.
    pub fn project(ir: &Ir, it: &Interner, scope: DeclId) -> Result<Self, String> {
        let graph = SignedHypergraph::within(ir, Some(scope));
        let max_displacement = number(ir, it, scope, "max_displacement", None)?;
        if !(1e-6..=1.0).contains(&max_displacement) {
            return Err("max_displacement must be in 1e-6..1 metres".into());
        }
        let mut particles = Vec::new();
        let mut positions = Vec::new();
        let mut velocities = Vec::new();
        for &decl in &graph.vertices {
            if field(ir, it, decl, "position").is_none() {
                continue;
            }
            let rest = vector(ir, it, decl, "position", None)?;
            let displacement = vector(ir, it, decl, "displacement", Some([0.0; 3]))?;
            let velocity = vector(ir, it, decl, "velocity", Some([0.0; 3]))?;
            let mass = number(ir, it, decl, "mass", None)?;
            let fixed = number(ir, it, decl, "fixed", Some(0.0))?;
            if !(1e-6..=1e3).contains(&mass)
                || (fixed != 0.0 && fixed != 1.0)
                || rest.iter().any(|x| x.abs() > 1000.0)
                || norm(displacement) > max_displacement
                || velocity.iter().any(|v| !v.is_finite() || v.abs() > 100.0)
                || (fixed == 1.0 && (displacement != [0.0; 3] || velocity != [0.0; 3]))
            {
                return Err(
                    "invalid particle mass, anchor, position, displacement or velocity".into(),
                );
            }
            positions.push(std::array::from_fn(|k| rest[k] + displacement[k]));
            velocities.push(velocity);
            particles.push(Particle {
                decl,
                name: it.resolve(ir.decl_nodes[decl.raw()].name).to_owned(),
                rest,
                mass,
                fixed: fixed == 1.0,
            });
            if particles.len() > 128 {
                return Err("at most 128 softbody particles".into());
            }
        }
        if particles.is_empty() || particles.iter().all(|p| p.fixed) {
            return Err("softbody needs at least one free particle".into());
        }
        let indexes: HashMap<_, _> = particles
            .iter()
            .enumerate()
            .map(|(i, p)| (p.decl, i))
            .collect();
        let mut springs = Vec::new();
        let mut triangles = Vec::new();
        let mut pairs = HashSet::new();
        for edge in &graph.edges {
            let Some(ValueR::Str(kind)) = field(ir, it, edge.decl, "kind") else {
                continue;
            };
            let kind = it.resolve(*kind);
            if kind != "spring" && kind != "surface" {
                continue;
            }
            let nodes: Vec<_> = edge
                .incidences
                .iter()
                .map(|i| {
                    if i.weight != 1.0 {
                        return Err("softbody incidences must have unit weight".to_owned());
                    }
                    indexes
                        .get(&graph.vertices[i.vertex])
                        .copied()
                        .ok_or_else(|| "edge endpoint is not a particle".to_owned())
                })
                .collect::<Result<_, _>>()?;
            if nodes.iter().collect::<HashSet<_>>().len() != nodes.len() {
                return Err("repeated softbody edge endpoint".into());
            }
            if kind == "surface" {
                if nodes.len() != 3 || edge.incidences.iter().any(|i| i.role != Role::Target) {
                    return Err("surface needs three ordered positive endpoints".into());
                }
                let [a, b, c] = [nodes[0], nodes[1], nodes[2]];
                let u: [f64; 3] =
                    std::array::from_fn(|k| particles[b].rest[k] - particles[a].rest[k]);
                let v: [f64; 3] =
                    std::array::from_fn(|k| particles[c].rest[k] - particles[a].rest[k]);
                if norm([
                    u[1] * v[2] - u[2] * v[1],
                    u[2] * v[0] - u[0] * v[2],
                    u[0] * v[1] - u[1] * v[0],
                ]) < 1e-12
                {
                    return Err("degenerate surface triangle".into());
                }
                triangles.push([a, b, c]);
                if triangles.len() > 512 {
                    return Err("at most 512 surface triangles".into());
                }
                continue;
            }
            if nodes.len() != 2
                || edge
                    .incidences
                    .iter()
                    .filter(|i| i.role == Role::Source)
                    .count()
                    != 1
                || edge
                    .incidences
                    .iter()
                    .filter(|i| i.role == Role::Target)
                    .count()
                    != 1
            {
                return Err("spring needs one negative and one positive endpoint".into());
            }
            let from = edge
                .incidences
                .iter()
                .position(|i| i.role == Role::Source)
                .ok_or("missing spring source")?;
            let [a, b] = [nodes[from], nodes[1 - from]];
            if !pairs.insert((a.min(b), a.max(b))) {
                return Err("duplicate particle spring".into());
            }
            let delta = std::array::from_fn(|k| particles[b].rest[k] - particles[a].rest[k]);
            let length = norm(delta);
            let stiffness = number(ir, it, edge.decl, "stiffness", None)?;
            if length < 1e-6 || !(1e-6..=1e6).contains(&stiffness) {
                return Err("invalid spring length or stiffness".into());
            }
            springs.push(Spring {
                decl: edge.decl,
                ends: [a, b],
                direction: delta.map(|x| x / length),
                stiffness,
            });
            if springs.len() > 512 {
                return Err("at most 512 springs".into());
            }
        }
        if springs.is_empty() {
            return Err("softbody needs elastic connections".into());
        }
        let mut state = Vec::new();
        let mut energy = Vec::new();
        for spring in &springs {
            let [a, b] = spring.ends;
            state.push(
                (0..3)
                    .map(|k| {
                        spring.direction[k]
                            * ((positions[b][k] - particles[b].rest[k])
                                - (positions[a][k] - particles[a].rest[k]))
                    })
                    .sum(),
            );
            energy.push(spring.stiffness);
        }
        let mut momentum = Vec::new();
        for (p, velocity) in particles.iter().zip(velocities) {
            momentum.push(if p.fixed {
                None
            } else {
                let start = state.len();
                state.extend(velocity.map(|v| p.mass * v));
                energy.extend([1.0 / p.mass; 3]);
                Some(start)
            });
        }
        let mut ports = Vec::new();
        for (s, spring) in springs.iter().enumerate() {
            for (i, sign) in [(spring.ends[0], -1.0), (spring.ends[1], 1.0)] {
                if let Some(p) = momentum[i] {
                    for k in 0..3 {
                        if spring.direction[k] != 0.0 {
                            ports.push(Port::edge(s, p + k, sign * spring.direction[k]));
                        }
                    }
                }
            }
        }
        let system = PortHamiltonian::new(
            Interconnection::new(state.len(), ports),
            vec![0.0; state.len()],
            energy,
        );
        Ok(Self {
            particles,
            springs,
            triangles,
            positions,
            momentum,
            state,
            system,
            max_displacement,
            time: 0.0,
            balance_error: 0.0,
        })
    }
    pub(crate) fn extensions(&self) -> &[f64] { &self.state[..self.springs.len()] }

    pub fn particles(&self) -> &[Particle] {
        &self.particles
    }
    pub fn springs(&self) -> &[Spring] {
        &self.springs
    }
    pub fn triangles(&self) -> &[[usize; 3]] {
        &self.triangles
    }
    pub fn positions(&self) -> &[[f64; 3]] {
        &self.positions
    }
    pub fn time(&self) -> f64 {
        self.time
    }
    pub fn energy(&self) -> f64 {
        self.system.hamiltonian(&self.state)
    }
    pub fn balance_error(&self) -> f64 {
        self.balance_error
    }
    pub fn velocity(&self, index: usize) -> Option<[f64; 3]> {
        self.particles.get(index).map(|p| {
            self.momentum[index]
                .map(|start| std::array::from_fn(|k| self.state[start + k] / p.mass))
                .unwrap_or([0.0; 3])
        })
    }

    /// Atomic midpoint step with world gravity and externally supplied nodal forces.
    /// Forces are sampled at the start of the step and held fixed during it.
    pub fn step(
        &mut self,
        dt: f64,
        gravity: [f64; 3],
        damping: f64,
        forces: &[[f64; 3]],
    ) -> Result<(), String> {
        if !dt.is_finite()
            || dt <= 0.0
            || dt > 1.0 / 120.0
            || !(self.time + dt).is_finite()
            || !damping.is_finite()
            || !(0.0..=100.0).contains(&damping)
            || gravity.iter().any(|x| !x.is_finite() || x.abs() > 100.0)
            || forces.len() != self.particles.len()
            || forces
                .iter()
                .flatten()
                .any(|x| !x.is_finite() || x.abs() > 1e5)
        {
            return Err("invalid softbody timestep, gravity, damping or nodal forces".into());
        }
        let mut system = self.system.clone();
        let mut input = vec![0.0; self.state.len()];
        for (i, p) in self.particles.iter().enumerate() {
            if let Some(start) = self.momentum[i] {
                for k in 0..3 {
                    input[start + k] = p.mass * gravity[k] + forces[i][k];
                    system.dissipation[start + k] = damping;
                }
            }
        }
        let mut next = self.state.clone();
        let report = system.step(&mut next, &input, dt, 1e-11);
        if !report.converged || next.iter().any(|x| !x.is_finite()) {
            return Err("softbody solve failed".into());
        }
        let mut positions = self.positions.clone();
        for (i, p) in self.particles.iter().enumerate() {
            if let Some(start) = self.momentum[i] {
                for k in 0..3 {
                    positions[i][k] +=
                        dt * 0.5 * (self.state[start + k] + next[start + k]) / p.mass;
                }
                if positions[i].iter().any(|x| !x.is_finite())
                    || norm(std::array::from_fn(|k| positions[i][k] - p.rest[k]))
                        > self.max_displacement
                {
                    return Err("softbody exceeded its declared small-displacement envelope; reduce load or increase stiffness".into());
                }
            }
        }
        let error = system.balance_residual(
            &self.state,
            &next,
            &input,
            dt,
            self.energy().max(system.hamiltonian(&next)).max(1e-12),
        );
        self.balance_error = self.balance_error.max(error);
        self.system = system;
        self.state = next;
        self.positions = positions;
        self.time += dt;
        Ok(())
    }
}

fn norm(v: [f64; 3]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use hymeko::{module_store::module_store::ModuleStore, util::real_parser::RealParser};
    use hymeko_sources::MemoryProvider;
    const MODEL: &str = "fixture {}\nbody { max_displacement 0.5; a {position [0,0,0]; mass 1; fixed 1;} b {position [1,0,0]; mass 2; displacement [0.1,0,0];} @spring {kind \"spring\";stiffness 8;(-body.a,+body.b);} }";
    fn read(text: &str) -> Result<SoftBody, String> {
        let mut store =
            ModuleStore::new(MemoryProvider::new().with("body.hymeko", text), RealParser);
        let p = store.compile(std::path::Path::new("body.hymeko")).unwrap();
        let scope =
            p.ir.decls()
                .find(|(_, n)| store.it.resolve(n.name) == "body")
                .unwrap()
                .0;
        SoftBody::project(&p.ir, &store.it, scope)
    }
    #[test]
    fn oscillator_matches_displacement_and_velocity_oracles_and_keeps_energy() {
        for (initial, x, v) in [
            (
                "displacement [0.1,0,0]",
                0.1 * 2.0_f64.cos(),
                -0.2 * 2.0_f64.sin(),
            ),
            (
                "velocity [0.2,0,0]",
                0.1 * 2.0_f64.sin(),
                0.2 * 2.0_f64.cos(),
            ),
        ] {
            let mut body = read(&MODEL.replace("displacement [0.1,0,0]", initial)).unwrap();
            assert!((body.energy() - 0.04).abs() < 1e-12);
            for _ in 0..240 {
                body.step(1.0 / 240.0, [0.0; 3], 0.0, &[[0.0; 3]; 2])
                    .unwrap();
            }
            assert!((body.positions()[1][0] - (1.0 + x)).abs() < 2e-6);
            assert!((body.velocity(1).unwrap()[0] - v).abs() < 3e-6);
            assert_eq!(body.positions()[0], [0.0; 3]);
            assert!((body.energy() - 0.04).abs() < 1e-10);
            assert!(body.balance_error() < 1e-9);
            assert!(body.velocity(99).is_none());
        }
        assert!(read(&MODEL.replace("fixed 1", "fixed 1; velocity [1,0,0]")).is_err());
        assert!(read(&MODEL.replace("mass 2", "mass 2; velocity [101,0,0]")).is_err());
    }
    #[test]
    fn equilibrium_and_gravity_use_mass_and_stiffness() {
        let mut body =
            read(&MODEL.replace("displacement [0.1,0,0]", "displacement [0.25,0,0]")).unwrap();
        // F = m*g = 2 N and extension F/k = 0.25 m.
        for _ in 0..120 {
            body.step(1.0 / 240.0, [1.0, 0.0, 0.0], 0.1, &[[0.0; 3]; 2])
                .unwrap();
        }
        assert!((body.positions()[1][0] - 1.25).abs() < 1e-12);
    }
    #[test]
    fn invalid_models_and_failed_steps_preserve_the_last_valid_state() {
        for text in [
            MODEL.replace("mass 2", "mass 0"),
            MODEL.replace("-body.a", "+body.a"),
            MODEL.replace("stiffness 8", "stiffness -1"),
            MODEL.replace("fixed 1", "fixed 2"),
        ] {
            assert!(read(&text).is_err());
        }
        let mut body = read(MODEL).unwrap();
        let before = body.positions().to_vec();
        assert!(body.step(f64::NAN, [0.0; 3], 0.0, &[[0.0; 3]; 2]).is_err());
        assert!(
            body.step(1.0 / 120.0, [0.0; 3], 0.0, &[[1e5; 3]; 2])
                .is_err()
        );
        assert_eq!(body.positions(), before);
        assert_eq!(body.time(), 0.0);
    }
}
