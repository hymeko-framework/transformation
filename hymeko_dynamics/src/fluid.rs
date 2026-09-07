//! Three-dimensional free-surface PIC liquid over an authored G-SPHF octree.
//! Pressure projection uses Tensor GMRES. PIC transfer is dissipative, not an exact pH step.
mod grid;
use crate::read::{number, vector};
use grid::Grid;
use hymeko::{
    common::ids::DeclId,
    ir::ir::{Ir, ValueR},
    resolution::interner::Interner,
};
use hymeko_graph::{
    octree::{Octree, field},
    sphf::{Role, SignedHypergraph},
};

/// Live boundary controls; flow is requested cubic metres per second.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Controls {
    pub inlet_flow: f64,
    pub valve: f64,
    pub drag: f64,
}
impl Controls {
    /// Validate without mutating a running liquid state.
    pub fn validate(&self) -> Result<(), String> {
        if !self.inlet_flow.is_finite()
            || !(0.0..=0.1).contains(&self.inlet_flow)
            || !self.valve.is_finite()
            || !(0.0..=1.0).contains(&self.valve)
            || !self.drag.is_finite()
            || !(0.0..=100.0).contains(&self.drag)
        {
            return Err(
                "invalid fluid flow (0..0.1 m3/s), valve (0..1) or drag (0..100 /s)".into(),
            );
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hymeko::{module_store::module_store::ModuleStore, util::real_parser::RealParser};
    use hymeko_sources::MemoryProvider;

    fn fixture() -> String {
        let mut source = String::from(
            "fixture {}\ntank {density 1000;initial_height 0.5;supply{} drain{} domain {lo [0,0,0];hi [1,1,1];",
        );
        for i in 0..8 {
            let lo: Vec<_> = (0..3)
                .map(|a| format!("{}", ((i >> a) & 1) as f64 * 0.5))
                .collect();
            let hi: Vec<_> = (0..3)
                .map(|a| format!("{}", ((i >> a) & 1) as f64 * 0.5 + 0.5))
                .collect();
            source += &format!("c{i} {{lo [{}];hi [{}];}}", lo.join(","), hi.join(","));
        }
        source += "@split {kind \"subdivision\";(-tank.domain,+tank.domain.c0,+tank.domain.c1,+tank.domain.c2,+tank.domain.c3,+tank.domain.c4,+tank.domain.c5,+tank.domain.c6,+tank.domain.c7);}}";
        source += "@inlet {kind \"inlet\";(-tank.supply,+tank.domain);centre [0.5,0.5,1];width 0.5;} @outlet {kind \"outlet\";(-tank.domain,+tank.drain);lower [1,0,0];upper [1,0.5,0.5];}}";
        source
    }
    fn compile(source: &str) -> Result<Fluid, String> {
        let mut store = ModuleStore::new(
            MemoryProvider::new().with("tank.hymeko", source),
            RealParser,
        );
        let product = store
            .compile(std::path::Path::new("tank.hymeko"))
            .map_err(|e| format!("{e:?}"))?;
        let scope = product
            .ir
            .decls()
            .find(|(_, n)| store.it.resolve(n.name) == "tank")
            .unwrap()
            .0;
        Fluid::project(&product.ir, &store.it, scope)
    }
    #[test]
    fn upward_free_surface_escape_is_counted_and_cfl_failure_is_atomic() {
        let mut fluid =
            compile(&fixture().replace("initial_height 0.5", "initial_height 0")).unwrap();
        // One valid material marker just below the open top, with upward momentum.
        fluid.markers.push(Marker {
            position: [0.5, 0.5, 0.999],
            velocity: [0.0, 0.0, 2.0],
        });
        fluid.observation.volume = fluid.marker_volume;
        fluid.observation.initial_volume = fluid.marker_volume;
        let controls = Controls {
            inlet_flow: 0.0,
            valve: 0.0,
            drag: 0.0,
        };
        fluid.step(1.0 / 240.0, 0.0, controls).unwrap();
        let s = fluid.observation();
        assert_eq!(s.volume, 0.0);
        assert_eq!(s.overflow, s.initial_volume);
        assert_eq!(s.outlet, 0.0);
        fluid.markers.push(Marker {
            position: [0.5, 0.5, 0.75],
            velocity: [0.0, 0.0, 1000.0],
        });
        let before = format!("{:?}{:?}", fluid.markers(), fluid.observation());
        let error = fluid.step(1.0 / 240.0, 0.0, controls).unwrap_err();
        assert!(error.contains("CFL"), "{error}");
        assert_eq!(
            format!("{:?}{:?}", fluid.markers(), fluid.observation()),
            before
        );
    }
    #[test]
    fn empty_and_full_tanks_bound_inventory_and_report_rejected_inlet() {
        let closed = Controls {
            inlet_flow: 0.0,
            valve: 0.0,
            drag: 0.0,
        };
        let mut empty =
            compile(&fixture().replace("initial_height 0.5", "initial_height 0")).unwrap();
        empty.step(1.0 / 240.0, 9.81, closed).unwrap();
        assert_eq!(empty.observation().volume, 0.0);
        assert_eq!(empty.observation().energy, 0.0);
        let mut full =
            compile(&fixture().replace("initial_height 0.5", "initial_height 1")).unwrap();
        for _ in 0..120 {
            full.step(
                1.0 / 240.0,
                9.81,
                Controls {
                    inlet_flow: 0.1,
                    ..closed
                },
            )
            .unwrap();
        }
        let s = full.observation();
        assert_eq!(full.markers().len(), 64);
        assert_eq!(s.volume, 1.0);
        assert_eq!(s.inlet, 0.0);
        assert!(s.blocked > 0.0);
        assert!((s.requested_inlet - s.blocked - s.pending).abs() < 1e-12);
    }
    #[test]
    fn pressure_holds_hydrostatic_liquid_and_mass_at_closed_walls() {
        let mut fluid = compile(&fixture()).unwrap();
        let controls = Controls {
            inlet_flow: 0.0,
            valve: 0.0,
            drag: 0.0,
        };
        let initial = fluid.observation().volume;
        for _ in 0..120 {
            fluid.step(1.0 / 240.0, 9.81, controls).unwrap();
        }
        let state = fluid.observation();
        assert_eq!(state.volume, initial);
        assert_eq!(state.outlet + state.overflow + state.inlet, 0.0);
        assert!(state.divergence < 1e-5, "{state:?}");
        assert!(
            state.max_pressure > 1000.0 && state.max_pressure < 10000.0,
            "{state:?}"
        );
        assert!(
            fluid
                .markers()
                .iter()
                .all(|p| p.velocity.iter().all(|u| u.abs() < 1e-6))
        );
        assert_eq!(fluid.tree().leaves.len(), 8);
        assert_eq!(fluid.spacing(), 0.5);
        assert_ne!(fluid.boundary().inlet, fluid.boundary().outlet);
    }
    #[test]
    fn inlet_outlet_and_overflow_have_exact_inventory_accounting() {
        let mut fluid = compile(&fixture()).unwrap();
        let open = Controls {
            inlet_flow: 0.08,
            valve: 1.0,
            drag: 0.1,
        };
        for _ in 0..480 {
            fluid.step(1.0 / 240.0, 9.81, open).unwrap();
        }
        let s = fluid.observation();
        assert!(s.inlet > 0.0 && s.outlet > 0.0, "{s:?}");
        assert!((s.volume - s.initial_volume - s.inlet + s.outlet + s.overflow).abs() < 1e-12);
        assert!((s.requested_inlet - s.inlet - s.pending - s.blocked).abs() < 1e-12);
        let before = s.outlet;
        for _ in 0..60 {
            fluid
                .step(1.0 / 240.0, 9.81, Controls { valve: 0.0, ..open })
                .unwrap();
        }
        assert_eq!(fluid.observation().outlet, before);
    }
    #[test]
    fn invalid_models_and_live_steps_fail_without_partial_state() {
        for (from, to) in [
            ("density 1000", "density -1"),
            ("initial_height 0.5", "initial_height 2"),
            ("+tank.domain);centre", "-tank.domain);centre"),
            ("width 0.5", "width 0"),
        ] {
            assert!(compile(&fixture().replace(from, to)).is_err());
        }
        let mut fluid = compile(&fixture()).unwrap();
        let before = format!("{:?}{:?}", fluid.markers(), fluid.observation());
        let good = Controls {
            inlet_flow: 0.0,
            valve: 0.0,
            drag: 0.0,
        };
        for (dt, g, c) in [
            (0.1, 9.81, good),
            (0.001, f64::NAN, good),
            (
                0.001,
                9.81,
                Controls {
                    valve: f64::NAN,
                    ..good
                },
            ),
        ] {
            assert!(fluid.step(dt, g, c).is_err());
            assert_eq!(
                format!("{:?}{:?}", fluid.markers(), fluid.observation()),
                before
            );
        }
    }
    #[test]
    fn pic_transfer_preserves_all_three_components_of_a_constant_field() {
        let mut fluid = compile(&fixture()).unwrap();
        for p in &mut fluid.markers {
            p.velocity = [0.2, -0.3, 0.4];
        }
        let (velocity, _) = fluid.grid.transfer(&fluid.markers);
        let sampled = fluid.grid.sample(&velocity, [0.5, 0.5, 0.25]);
        for (a, b) in sampled.iter().zip([0.2, -0.3, 0.4]) {
            assert!((a - b).abs() < 1e-12);
        }
    }

    #[test]
    fn viscosity_matches_discrete_shear_decay_for_signed_velocity() {
        let fluid = compile(&fixture()).unwrap();
        let grid = &fluid.grid;
        let count = 3 * (grid.n + 1).pow(3);
        let dt = 1.0 / 240.0;
        let nu = 0.1;
        let lambda =
            8.0 / grid.dx.powi(2) * (std::f64::consts::PI / (2.0 * grid.n as f64)).sin().powi(2);
        for amplitude in [-2.0, -0.1, 0.0, 0.3, 2.0] {
            let mut velocity = vec![0.0; count];
            for f in &grid.faces {
                if f.axis == 0 {
                    velocity[f.slot] = amplitude
                        * (std::f64::consts::PI * f.centre[0]).sin()
                        * (std::f64::consts::PI * f.centre[1]).cos();
                }
            }
            let initial = velocity.clone();
            let liquid = vec![true; grid.n.pow(3)];
            assert_eq!(
                grid.diffuse(&mut velocity, &liquid, 0.0, dt, 0.0).unwrap(),
                0.0
            );
            assert_eq!(velocity, initial);
            let loss = grid.diffuse(&mut velocity, &liquid, nu, dt, 0.0).unwrap();
            assert!(loss >= -1e-12);
            for (v, before) in velocity.iter().zip(&initial) {
                assert!((v - before / (1.0 + dt * nu * lambda)).abs() < 1e-11);
            }
            assert!(velocity.iter().sum::<f64>().abs() < 1e-11);
        }
    }

    #[test]
    fn authored_viscosity_preserves_inventory_and_reports_dissipation() {
        let source = fixture().replace(
            "density 1000",
            "density 1000;kinematic_viscosity 0.01;initial_velocity [0.2,-0.1,0]",
        );
        let mut fluid = compile(&source).unwrap();
        let controls = Controls {
            inlet_flow: 0.0,
            valve: 0.0,
            drag: 0.0,
        };
        for _ in 0..120 {
            fluid.step(1.0 / 240.0, 9.81, controls).unwrap();
        }
        let s = fluid.observation();
        assert_eq!(s.kinematic_viscosity, 0.01);
        assert!(s.viscous_loss > 0.0 && s.viscous_loss.is_finite());
        assert_eq!(s.volume, s.initial_volume);
        assert!(s.divergence < 1e-5);
        for invalid in ["-0.1", "1.01"] {
            assert!(
                compile(&source.replace(
                    "kinematic_viscosity 0.01",
                    &format!("kinematic_viscosity {invalid}")
                ))
                .is_err()
            );
        }
        assert_eq!(
            compile(&fixture())
                .unwrap()
                .observation()
                .kinematic_viscosity,
            0.0
        );
    }

    #[test]
    fn vanishing_viscosity_preserves_free_surface_transfer() {
        let source = fixture().replace(
            "density 1000",
            "density 1000;initial_velocity [0.2,-0.1,0.1]",
        );
        let mut inviscid = compile(&source).unwrap();
        let mut viscous = compile(&source.replace(
            "density 1000",
            "density 1000;kinematic_viscosity 0.000000000001",
        ))
        .unwrap();
        let controls = Controls {
            inlet_flow: 0.0,
            valve: 0.5,
            drag: 0.0,
        };
        for _ in 0..10 {
            inviscid.step(1.0 / 240.0, 9.81, controls).unwrap();
            viscous.step(1.0 / 240.0, 9.81, controls).unwrap();
            for (a, b) in inviscid.markers().iter().zip(viscous.markers()) {
                for axis in 0..3 {
                    assert!((a.position[axis] - b.position[axis]).abs() < 1e-9);
                    assert!((a.velocity[axis] - b.velocity[axis]).abs() < 1e-9);
                }
            }
        }
    }
}
/// Equal-volume material marker; volume accounting uses these actual particles.
#[derive(Clone, Debug)]
pub struct Marker {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
}
/// Boundary geometry is projected from signed inlet/outlet declarations.
#[derive(Clone, Debug)]
pub struct Boundary {
    pub inlet: DeclId,
    pub outlet: DeclId,
    pub inlet_centre: [f64; 3],
    pub inlet_width: f64,
    pub outlet_lo: [f64; 3],
    pub outlet_hi: [f64; 3],
}
impl Boundary {
    fn project(ir: &Ir, it: &Interner, scope: DeclId, root: DeclId) -> Result<Self, String> {
        let graph = SignedHypergraph::within(ir, Some(scope));

        let mut inlet = None;
        let mut outlet = None;
        for edge in &graph.edges {
            let Some(ValueR::Str(kind)) = field(ir, it, edge.decl, "kind") else {
                continue;
            };
            let role = match it.resolve(*kind) {
                "inlet" => Role::Target,
                "outlet" => Role::Source,
                _ => continue,
            };
            if edge.incidences.len() != 2
                || !edge
                    .incidences
                    .iter()
                    .any(|inc| graph.vertices[inc.vertex] == root && inc.role == role)
                || edge
                    .incidences
                    .iter()
                    .filter(|inc| inc.role == Role::Source)
                    .count()
                    != 1
                || edge
                    .incidences
                    .iter()
                    .filter(|inc| inc.role == Role::Target)
                    .count()
                    != 1
            {
                return Err(
                    "fluid ports must have opposite signed incidences at the domain boundary"
                        .into(),
                );
            }
            let target = if role == Role::Target {
                &mut inlet
            } else {
                &mut outlet
            };
            if target.replace(edge.decl).is_some() {
                return Err("exactly one inlet and outlet are supported".into());
            }
        }
        let inlet = inlet.ok_or("missing signed inlet")?;
        let outlet = outlet.ok_or("missing signed outlet")?;
        Ok(Self {
            inlet,
            outlet,
            inlet_centre: vector(ir, it, inlet, "centre", None)?,
            inlet_width: number(ir, it, inlet, "width", None)?,
            outlet_lo: vector(ir, it, outlet, "lower", None)?,
            outlet_hi: vector(ir, it, outlet, "upper", None)?,
        })
    }
    fn validate(&self, grid: &Grid) -> Result<(), String> {
        let inlet_valid = (grid.dx..=(grid.hi[0] - grid.lo[0]) * 0.5).contains(&self.inlet_width)
            && (self.inlet_centre[2] - grid.hi[2]).abs() <= 1e-8
            && (0..2).all(|a| {
                self.inlet_centre[a] - self.inlet_width * 0.5 >= grid.lo[a]
                    && self.inlet_centre[a] + self.inlet_width * 0.5 <= grid.hi[a]
            });
        let outlet_valid = (self.outlet_lo[0] - grid.hi[0]).abs() <= 1e-8
            && (self.outlet_hi[0] - grid.hi[0]).abs() <= 1e-8
            && (1..3).all(|a| {
                self.outlet_lo[a] >= grid.lo[a]
                    && self.outlet_hi[a] <= grid.hi[a]
                    && self.outlet_hi[a] - self.outlet_lo[a] >= grid.dx
            });
        if !inlet_valid || !outlet_valid {
            return Err("invalid top-inlet/+X-outlet geometry".into());
        }
        Ok(())
    }
}
/// Read-only physical observations. Volumes are m3; energy is J, pressure Pa.
#[derive(Clone, Copy, Debug)]
pub struct Observation {
    pub time: f64,
    pub volume: f64,
    pub initial_volume: f64,
    pub requested_inlet: f64,
    pub inlet: f64,
    pub outlet: f64,
    pub overflow: f64,
    pub pending: f64,
    pub blocked: f64,
    pub energy: f64,
    pub divergence: f64,
    pub max_pressure: f64,
    /// Authored kinematic viscosity, m2/s. Zero retains the inviscid prototype.
    pub kinematic_viscosity: f64,
    /// Cumulative MAC dual-grid kinetic-energy loss in the implicit viscosity
    /// substep, J. Includes backward-Euler damping; excludes PIC/pressure losses.
    pub viscous_loss: f64,
}
/// Bounded particle-in-cell liquid. Walls are fixed, free-slip and axis aligned.
pub struct Fluid {
    tree: Octree,
    grid: Grid,
    boundary: Boundary,
    markers: Vec<Marker>,
    density: f64,
    marker_volume: f64,
    sequence: usize,
    observation: Observation,
}
impl Fluid {
    /// Compile a uniform octree and exactly one signed inlet/outlet into a liquid domain.
    pub fn project(ir: &Ir, it: &Interner, scope: DeclId) -> Result<Self, String> {
        let tree = Octree::from_ir(ir, it, scope)?;
        let boundary = Boundary::project(ir, it, scope, tree.cells[tree.root].decl)?;
        let density = number(ir, it, scope, "density", None)?;
        let viscosity = number(ir, it, scope, "kinematic_viscosity", Some(0.0))?;
        let height = number(ir, it, scope, "initial_height", None)?;
        let initial_velocity = vector(ir, it, scope, "initial_velocity", Some([0.0; 3]))?;
        let grid = Grid::new(&tree, boundary.outlet_lo, boundary.outlet_hi)?;
        if !(1.0..=20000.0).contains(&density)
            || !(0.0..=1.0).contains(&viscosity)
            || !(0.0..=(grid.hi[2] - grid.lo[2])).contains(&height)
            || initial_velocity.iter().any(|v| v.abs() > 2.0)
        {
            return Err(
                "invalid liquid density, viscosity (0..1 m2/s), fill height or initial velocity"
                    .into(),
            );
        }
        boundary.validate(&grid)?;
        let marker_volume = (grid.dx * 0.5).powi(3);
        let m = 2 * grid.n;
        let markers: Vec<_> = (0..m.pow(3))
            .filter_map(|flat| {
                let c = [flat / (m * m), flat / m % m, flat % m];
                let position =
                    std::array::from_fn(|a| grid.lo[a] + (c[a] as f64 + 0.5) * grid.dx * 0.5);
                (position[2] < grid.lo[2] + height).then_some(Marker {
                    position,
                    velocity: initial_velocity,
                })
            })
            .collect();
        let volume = markers.len() as f64 * marker_volume;
        Ok(Self {
            tree,
            grid,
            boundary,
            markers,
            density,
            marker_volume,
            sequence: 0,
            observation: Observation {
                time: 0.0,
                volume,
                initial_volume: volume,
                requested_inlet: 0.0,
                inlet: 0.0,
                outlet: 0.0,
                overflow: 0.0,
                pending: 0.0,
                blocked: 0.0,
                energy: 0.0,
                divergence: 0.0,
                max_pressure: 0.0,
                kinematic_viscosity: viscosity,
                viscous_loss: 0.0,
            },
        })
    }
    /// Canonical authored cell hierarchy; dynamic occupancy is separate physical state.
    pub fn tree(&self) -> &Octree {
        &self.tree
    }
    /// Physical particle positions/velocities; observation never advances time.
    pub fn markers(&self) -> &[Marker] {
        &self.markers
    }
    /// Signed physical boundary declaration identities and geometry.
    pub fn boundary(&self) -> &Boundary {
        &self.boundary
    }
    /// Current measured inventory, boundary integrals and solver diagnostics.
    pub fn observation(&self) -> Observation {
        self.observation
    }
    /// Cubic MAC cell side in metres.
    pub fn spacing(&self) -> f64 {
        self.grid.dx
    }

    fn inject(
        &self,
        next: &mut Vec<Marker>,
        state: &mut Observation,
        controls: Controls,
        dt: f64,
    ) -> usize {
        state.requested_inlet += controls.inlet_flow * dt;
        let supplied = state.pending + controls.inlet_flow * dt;
        let requested = (supplied / self.marker_volume).floor() as usize;
        let count = requested.min((8 * self.grid.n.pow(3)).saturating_sub(next.len()));
        state.pending = supplied - requested as f64 * self.marker_volume;
        state.blocked += (requested - count) as f64 * self.marker_volume;
        state.inlet += count as f64 * self.marker_volume;
        for n in 0..count {
            let sequence = self.sequence + n;
            let mut position = self.boundary.inlet_centre;
            for (a, p) in position.iter_mut().enumerate().take(2) {
                let offset = if a == 0 {
                    sequence % 4
                } else {
                    sequence / 4 % 4
                };
                *p += (offset as f64 / 4.0 - 0.375) * self.boundary.inlet_width;
            }
            position[2] -= self.grid.dx * 0.1;
            next.push(Marker {
                position,
                velocity: [
                    0.0,
                    0.0,
                    -controls.inlet_flow / self.boundary.inlet_width.powi(2),
                ],
            });
        }
        count
    }
    /// Atomic bounded physical step. Closed walls retain particles; open boundaries count escapes.
    pub fn step(&mut self, dt: f64, g: f64, controls: Controls) -> Result<(), String> {
        controls.validate()?;
        if !dt.is_finite()
            || dt <= 0.0
            || dt > 1.0 / 120.0
            || !g.is_finite()
            || !(0.0..=100.0).contains(&g)
        {
            return Err("fluid requires 0 < dt <= 1/120 s and 0 <= g <= 100 m/s2".into());
        }
        let mut next = self.markers.clone();
        let mut state = self.observation;
        let injected = self.inject(&mut next, &mut state, controls, dt);
        let (velocity, pressure, residual, viscous_loss) = self.grid.project(
            &next,
            controls,
            g,
            dt,
            self.density,
            state.kinematic_viscosity,
        )?;
        state.viscous_loss += viscous_loss;
        for p in &mut next {
            p.velocity = self
                .grid
                .sample(&velocity, p.position)
                .map(|u| u * (-controls.drag * dt).exp());
            if p.velocity
                .iter()
                .any(|u| !u.is_finite() || u.abs() * dt > 0.45 * self.grid.dx)
            {
                return Err(
                    "fluid step exceeds the half-cell CFL travel budget; reduce forcing".into(),
                );
            }
            for a in 0..3 {
                p.position[a] += p.velocity[a] * dt;
            }
        }
        next.retain_mut(|p| {
            if p.position[2] > self.grid.hi[2] {
                state.overflow += self.marker_volume;
                return false;
            }
            if controls.valve > 0.0
                && p.position[0] > self.grid.hi[0]
                && self.grid.is_outlet(p.position)
            {
                state.outlet += self.marker_volume;
                return false;
            }
            for a in 0..3 {
                let low = self.grid.lo[a] + self.grid.dx * 1e-4;
                let high = self.grid.hi[a] - self.grid.dx * 1e-4;
                if p.position[a] < low {
                    p.position[a] = low;
                    p.velocity[a] = p.velocity[a].max(0.0);
                }
                if p.position[a] > high {
                    p.position[a] = high;
                    p.velocity[a] = p.velocity[a].min(0.0);
                }
            }
            true
        });
        state.time += dt;
        state.volume = next.len() as f64 * self.marker_volume;
        state.divergence = residual;
        state.max_pressure = pressure.iter().copied().fold(0.0_f64, f64::max);
        state.energy = next
            .iter()
            .map(|p| {
                self.density
                    * self.marker_volume
                    * (0.5 * p.velocity.iter().map(|u| u * u).sum::<f64>()
                        + g * (p.position[2] - self.grid.lo[2]))
            })
            .sum();
        self.markers = next;
        self.observation = state;
        self.sequence += injected;
        Ok(())
    }
}
