//! Bounded, read-only power views of existing physical projections.
//! This is an observation result, not an IR, solver, or independent state model.
use hymeko::common::ids::DeclId;
use hymeko_geometry::{
    articulated::AbaPlan, engine::Engine, kinematics::kinematic::KinematicModel,
};

/// A conjugate physical port or a public interface declared by the owner.
#[derive(Clone, Debug, serde::Serialize)]
pub struct Port {
    #[serde(rename = "id")]
    pub decl: DeclId,
    pub name: String,
}
/// Oriented reference connection. Incidence signs are independent of weight sign
/// and instantaneous power flow. `decl=None` denotes a derived projection bond.
#[derive(Clone, Debug, serde::Serialize)]
pub struct Connection {
    #[serde(rename = "source")]
    pub negative: String,
    #[serde(rename = "target")]
    pub positive: String,
    #[serde(rename = "id")]
    pub decl: Option<DeclId>,
    pub weight: Option<f64>,
}
/// Display-only physical law supplied by its domain owner, not an expression IR.
/// `value` observes the current state; None means unavailable, not zero.
#[derive(Clone, Debug, serde::Serialize)]
pub struct Equation {
    pub label: &'static str,
    pub expression: &'static str,
    pub value: Option<f64>,
    pub unit: &'static str,
}
impl Equation {
    fn new(
        label: &'static str,
        expression: &'static str,
        value: Option<f64>,
        unit: &'static str,
    ) -> Self {
        Self {
            label,
            expression,
            value,
            unit,
        }
    }
}
/// A visible storage, dissipation, boundary or encapsulated subsystem.
#[derive(Clone, Debug, serde::Serialize)]
pub struct Node {
    pub key: String,
    #[serde(rename = "id")]
    pub decl: DeclId,
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scope: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub value: Option<f64>,
    pub unit: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub energy: Option<f64>,
    pub ports: Vec<Port>,
}
impl Node {
    fn scalar(key: String, decl: DeclId, name: String, value: Option<f64>, unit: &str) -> Self {
        Self {
            key,
            decl,
            name,
            scope: None,
            value,
            unit: unit.into(),
            energy: None,
            ports: vec![],
        }
    }
}
/// One requested scope. Unknown measurements remain None, never fabricated zero power.
#[derive(Clone, Debug, serde::Serialize)]
pub struct PowerGraph {
    pub name: String,
    pub scope: Option<usize>,
    pub parent: Option<usize>,
    pub selection: Option<DeclId>,
    pub nodes: Vec<Node>,
    pub edges: Vec<Connection>,
    pub energy: Option<f64>,
    #[serde(rename = "energyUnit")]
    pub energy_unit: String,
    #[serde(rename = "power")]
    pub supplied_power: Option<f64>,
    #[serde(rename = "dissipation")]
    pub dissipated_power: Option<f64>,
    pub time: f64,
    pub note: String,
    pub truncated: bool,
    pub equations: Vec<Equation>,
    #[serde(rename = "equationNote")]
    pub equation_note: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mapping: Option<MechanicalProjection>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub audit: Option<hymeko_geometry::engine::EnergyAudit>,
}
impl PowerGraph {
    /// Bounded observation for existing presentation hosts. Derived objects
    /// have null declaration IDs; missing physical measurements remain null.
    pub fn to_json(&self) -> Result<serde_json::Value, String> {
        if let Some(mapping) = &self.mapping {
            mapping.validate()?;
        }
        if let Some(audit) = &self.audit {
            let wire = serde_json::to_value(audit).map_err(|e| e.to_string())?;
            if wire
                .as_object()
                .is_none_or(|o| o.values().any(serde_json::Value::is_null))
            {
                return Err("nonfinite energy audit".into());
            }
        }
        let layout = self
            .arrangement()
            .ok_or("invalid or over-budget power graph")?;
        let values = [
            self.energy,
            self.supplied_power,
            self.dissipated_power,
            Some(self.time),
        ]
        .into_iter()
        .chain(self.nodes.iter().flat_map(|n| [n.energy, n.value]))
        .chain(self.edges.iter().map(|e| e.weight))
        .chain(self.equations.iter().map(|e| e.value));
        if values.flatten().any(|v| !v.is_finite()) {
            return Err("nonfinite power observation".into());
        }
        let mut value = serde_json::to_value(self).map_err(|e| e.to_string())?;
        for (i, node) in self.nodes.iter().enumerate() {
            let row = &mut value["nodes"][i];
            row["position"] = serde_json::json!(layout[i]);
            if node.decl.is_none() {
                row["id"] = serde_json::Value::Null;
            }
            for (j, port) in node.ports.iter().enumerate() {
                if port.decl.is_none() {
                    row["ports"][j]["id"] = serde_json::Value::Null;
                }
            }
        }
        for (i, edge) in self.edges.iter().enumerate() {
            value["edges"][i]["sourceSign"] = (-1).into();
            value["edges"][i]["targetSign"] = 1.into();
            if edge.decl.is_some_and(|id| id.is_none()) {
                value["edges"][i]["id"] = serde_json::Value::Null;
            }
        }
        Ok(value)
    }

    fn new(name: String, scope: Option<usize>, time: f64, energy: Option<f64>) -> Self {
        Self {
            name,
            scope,
            parent: None,
            selection: None,
            nodes: vec![],
            edges: vec![],
            energy,
            energy_unit: "J".into(),
            supplied_power: None,
            dissipated_power: None,
            time,
            note: String::new(),
            truncated: false,
            equations: vec![],
            equation_note: "Ehhez a projekcióhoz még nincs egyenletmegfigyelés. A gráf topológiája önmagában nem határoz meg Lagrange-függvényt.".into(),
            mapping: None,
            audit: None,
        }
    }
    fn push(&mut self, node: Node) {
        if self.nodes.len() < 500 {
            self.nodes.push(node);
        } else {
            self.truncated = true;
        }
    }
    fn connect(&mut self, a: String, b: String) {
        self.connect_identified(a, b, None, None);
    }
    fn connect_identified(
        &mut self,
        a: String,
        b: String,
        decl: Option<DeclId>,
        weight: Option<f64>,
    ) {
        if a != b && self.nodes.iter().any(|n| n.key == a) && self.nodes.iter().any(|n| n.key == b)
        {
            if self.edges.len() < 2000 {
                self.edges.push(Connection {
                    negative: a,
                    positive: b,
                    decl,
                    weight,
                });
            } else {
                self.truncated = true;
            }
        }
    }
    /// Deterministic display positions, ordered like `nodes`, without changing
    /// physical state. Returns None for malformed or over-budget public input.
    pub fn arrangement(&self) -> Option<Vec<[f64; 2]>> {
        if self.nodes.len() > 500 || self.edges.len() > 2000 {
            return None;
        }
        let index: std::collections::HashMap<_, _> = self
            .nodes
            .iter()
            .enumerate()
            .map(|(i, n)| (n.key.as_str(), i))
            .collect();
        if index.len() != self.nodes.len() {
            return None;
        }
        let edges: Option<Vec<_>> = self
            .edges
            .iter()
            .map(|e| {
                Some((
                    *index.get(e.positive.as_str())?,
                    *index.get(e.negative.as_str())?,
                ))
            })
            .collect();
        crate::layout::layered(self.nodes.len(), &edges?)
    }
    /// Existing declared pH system, preserving its exact exported-port encapsulation.
    /// Returns None for stale scopes or malformed state; never advances the producer.
    pub fn system(
        system: &crate::System,
        state: &[f64],
        time: f64,
        scope: Option<usize>,
    ) -> Option<Self> {
        if !time.is_finite()
            || state.iter().any(|v| !v.is_finite())
            || state.len() != system.dim()
            || scope.is_some_and(|i| i >= system.hierarchy.systems.len())
        {
            return None;
        }
        let balance = system.subsystem_balance(scope, state, time);
        let mut out = Self::new(system.name.clone(), scope, time, Some(balance.energy));
        out.supplied_power = Some(balance.supplied_power + balance.boundary_power);
        out.dissipated_power = Some(balance.dissipated_power);
        out.equations.push(Equation::new(
            "Hamilton-függvény",
            "Hₛ(x) = Σ[k ∈ S] xₖ² / (2cₖ)",
            Some(balance.energy),
            "J",
        ));
        out.equation_note = "S a kiválasztott rendszer összes tárolója, a beágyazott alrendszerekkel együtt; cₖ a modellben deklarált kapacitás. A tárolókoordinátákból nincs deklarált kanonikus (q,p) pár: Euler–Lagrange-alakot ebből nem képezünk automatikusan.".into();
        if scope.is_none() {
            out.equations.push(Equation::new(
                "Port-Hamiltoni dinamika",
                "ẋ = (J − R)∇H + Gu;  y = Gᵀ∇H",
                None,
                "",
            ));
            out.equations.push(Equation::new(
                "Teljesítménymérleg",
                "dH/dt = uᵀy − (∇H)ᵀR∇H",
                None,
                "W",
            ));
        }
        if let Some(i) = scope {
            let g = &system.hierarchy.systems[i];
            out.name = g.name.clone();
            out.parent = g.parent;
            out.selection = Some(g.decl);
        }
        let children: Vec<_> = system
            .hierarchy
            .systems
            .iter()
            .enumerate()
            .filter(|(_, g)| g.parent == scope)
            .collect();
        for (i, g) in &children {
            let mut node = Node::scalar(format!("s{i}"), g.decl, g.name.clone(), None, "J");
            node.scope = Some(*i);
            node.energy = Some(system.subsystem_balance(Some(*i), state, time).energy);
            node.ports = g
                .ports
                .iter()
                .map(|p| Port {
                    decl: p.decl,
                    name: p.name.clone(),
                })
                .collect();
            out.push(node);
        }
        for (i, c) in system.coordinates.iter().enumerate() {
            if system.hierarchy.coordinate_owner[i] == scope {
                out.push(Node::scalar(
                    format!("c{i}"),
                    c.decl,
                    c.name.clone(),
                    Some(state[i]),
                    &c.unit,
                ));
            }
        }
        let endpoint = |i| {
            if system.hierarchy.coordinate_owner[i] == scope {
                Some(format!("c{i}"))
            } else {
                children
                    .iter()
                    .find(|(g, s)| {
                        system.hierarchy.contains(*g, i)
                            && s.ports.iter().any(|p| p.coordinate == i)
                    })
                    .map(|(g, _)| format!("s{g}"))
            }
        };
        for (i, p) in system.blueprint.ports.iter().enumerate() {
            if let (Some(a), Some(b)) = (endpoint(p.source), endpoint(p.target)) {
                out.connect_identified(
                    a,
                    b,
                    Some(p.decl),
                    system
                        .port_hamiltonian
                        .interconnection
                        .ports()
                        .get(i)
                        .map(|p| p.weight),
                );
            }
        }
        Some(out)
    }
}

mod canonical;
mod continuum;
mod electromagnetic;
mod mechanical;
pub use canonical::MechanicalProjection;

#[cfg(test)]
mod tests {
    use super::*;
    use hymeko_geometry::kinematics::{
        joints::{JointInfo, JointType},
        kinematic::LinkInfo,
    };
    fn body(joint_type: JointType) -> Engine {
        let model = KinematicModel {
            name: "reference".into(),
            actuation: Default::default(),
            frames: vec!["world".into()],
            links: vec![LinkInfo {
                did: DeclId::new(1),
                name: "mass".into(),
                mass: Some(2.0),
                origin: Some(vec![0.0, 0.0, -1.0]),
                inertia_com: None,
                geometry: None,
                color: None,
            }],
            joints: vec![JointInfo {
                did: DeclId::new(2),
                name: "joint".into(),
                joint_type,
                parent: DeclId::NONE,
                child: DeclId::new(1),
                parent_link: "world".into(),
                child_link: "mass".into(),
                axis: Some([0.0, 1.0, 0.0]),
                origin_xyz: None,
                origin_rpy_deg: None,
                limits: None,
            }],
        };
        let mut engine = Engine::new(model).unwrap();
        engine.ground = None;
        engine.q[0] = 0.4;
        engine.qd[0] = 3.0;
        engine.torque[0] = 2.0;
        engine.damping = 0.5;
        engine
    }
    fn rotor_body() -> Engine {
        let mut engine = body(JointType::Revolute);
        engine.set_joint_dynamics(&[3.], &[2.]).unwrap();
        engine
    }
    #[test]
    fn power_wire_observation_preserves_unknowns_and_rejects_nonfinite_data() {
        let mut engine = body(JointType::Revolute);
        let graph = PowerGraph::mechanism(&engine, 1., None).unwrap();
        let wire = graph.to_json().unwrap();
        assert_eq!(wire["energy"], serde_json::json!(graph.energy));
        assert_eq!(wire["dissipation"], 4.5);
        assert_eq!(wire["nodes"][0]["id"], 1);
        assert!(wire["nodes"][0]["position"].is_array());
        let mut unknown = graph.clone();
        unknown.nodes[0].decl = DeclId::NONE;
        unknown.supplied_power = None;
        assert!(unknown.to_json().unwrap()["nodes"][0]["id"].is_null());
        assert!(unknown.to_json().unwrap()["power"].is_null());
        unknown.energy = Some(f64::NAN);
        assert!(unknown.to_json().is_err());
        engine.damping = -1.;
        assert!(PowerGraph::mechanism(&engine, 1., None).is_none());
        engine.set_joint_dynamics(&[0.], &[0.8]).unwrap();
        assert_eq!(engine.viscous_power(), Some(7.200000000000001));
        engine.qd[0] = f64::NAN;
        assert!(engine.viscous_power().is_none());
    }

    #[test]
    fn rotor_storage_and_per_joint_loss_match_the_active_engine_properties() {
        let engine = rotor_body();
        let before = (engine.q.clone(), engine.qd.clone(), engine.regime_counts());
        let graph = PowerGraph::mechanism(&engine, 1.0, None).unwrap();
        let expected = 22.5 - 2.0 * 9.80665 * 0.4_f64.cos();
        assert!((graph.energy.unwrap() - expected).abs() < 1e-10);
        assert_eq!(graph.dissipated_power, Some(18.0));
        let part = PowerGraph::mechanism(&engine, 1.0, Some(0)).unwrap();
        assert_eq!(graph.energy, part.energy);
        assert_eq!(
            part.nodes.iter().find(|n| n.key == "p0").unwrap().value,
            Some(15.0)
        );
        assert_eq!(
            before,
            (engine.q.clone(), engine.qd.clone(), engine.regime_counts())
        );
    }

    #[test]
    fn canonical_pendulum_matches_analytic_hamiltonian_and_rejects_corruption() {
        let mut e = rotor_body();
        let before = (e.q.clone(), e.qd.clone(), e.regime_counts());
        let m = MechanicalProjection::observe(&e).unwrap();
        assert!((m.energy - (22.5 - 2. * 9.80665 * 0.4_f64.cos())).abs() < 1e-10);
        assert_eq!(m.p, vec![15.]);
        assert!((m.gradient[0] - 2. * 9.80665 * 0.4_f64.sin()).abs() < 1e-8);
        assert!((m.gradient[1] - 3.).abs() < 1e-12);
        assert!((m.smooth_rate[1] - (-m.gradient[0] + 2. - 6.)).abs() < 1e-10);
        assert!(m.balance_residual.abs() < 1e-12);
        let wire = PowerGraph::canonical_mechanics(&e, 1.)
            .unwrap()
            .to_json()
            .unwrap();
        assert_eq!(wire["mapping"]["declarations"][0], 2);
        assert_eq!(wire["nodes"].as_array().unwrap().len(), 2);
        assert_eq!(before, (e.q.clone(), e.qd.clone(), e.regime_counts()));
        for fault in 0..4 {
            let mut bad = m.clone();
            match fault {
                0 => bad.interconnection[1] = -1.,
                1 => bad.damping[0] = -1.,
                2 => bad.input_power += 1.,
                _ => bad.gradient[0] += 1.,
            }
            assert!(bad.validate().is_err());
        }
        assert!(PowerGraph::canonical_mechanics(&e, f64::NAN).is_err());
        e.enable_energy_audit().unwrap();
        let mut invalid_audit = PowerGraph::canonical_mechanics(&e, 1.).unwrap();
        invalid_audit.audit.as_mut().unwrap().drive_work = f64::NAN;
        assert!(invalid_audit.to_json().is_err());
    }

    #[test]
    fn canonical_coupled_mechanism_uses_fixed_momentum_gradient() {
        let source = body(JointType::Revolute);
        let mut model = source.model().clone();
        let mut second = model.links[0].clone();
        second.did = DeclId::new(3);
        second.name = "second".into();
        second.mass = Some(1.);
        model.links.push(second);
        let mut joint = model.joints[0].clone();
        joint.did = DeclId::new(4);
        joint.name = "elbow".into();
        joint.parent = DeclId::new(1);
        joint.child = DeclId::new(3);
        joint.parent_link = "mass".into();
        joint.child_link = "second".into();
        joint.origin_xyz = Some([0., 0., -1.]);
        model.joints.push(joint);
        let mut e = Engine::new(model).unwrap();
        e.ground = None;
        e.q = vec![0.4, 0.6];
        e.qd = vec![0.7, -1.1];
        e.torque = vec![2., -1.];
        e.set_joint_dynamics(&[0.3, 0.2], &[0.2, 0.4]).unwrap();
        let m = MechanicalProjection::observe(&e).unwrap();
        let c = 0.6_f64.cos();
        let s = 0.6_f64.sin();
        let g = 9.80665;
        let p0 = (4.3 + 2. * c) * 0.7 + (1. + c) * (-1.1);
        let p1 = (1. + c) * 0.7 + 1.2 * (-1.1);
        assert!((m.p[0] - p0).abs() < 1e-11);
        assert!((m.p[1] - p1).abs() < 1e-11);
        assert!(
            (m.energy
                - (0.5 * (p0 * 0.7 + p1 * (-1.1)) - g * (3. * 0.4_f64.cos() + 1.0_f64.cos())))
            .abs()
                < 1e-10
        );
        assert!((m.gradient[0] - g * (3. * 0.4_f64.sin() + 1.0_f64.sin())).abs() < 1e-7);
        let expected = s * (0.7 * 0.7 + 0.7 * (-1.1)) + g * 1.0_f64.sin();
        assert!((m.gradient[1] - expected).abs() < 1e-7);
        // A derivative at fixed velocity has the opposite kinetic term.
        assert!((m.gradient[1] - (g * 1.0_f64.sin() - s * (0.7 * 0.7 + 0.7 * (-1.1)))).abs() > 0.1);
        assert!(m.velocity_residual < 1e-12);
        assert!(m.balance_residual.abs() < 1e-12);
    }

    #[test]
    fn canonical_projection_rejects_singular_and_invalid_inputs() {
        let e = body(JointType::Revolute);
        let mut model = e.model().clone();
        model.links[0].origin = Some(vec![0.; 3]);
        let singular = Engine::new(model).unwrap();
        assert!(MechanicalProjection::observe(&singular).is_err());
        assert!(e.dynamics().solve_mass(e.model(), &[], &[]).is_none());
        let mut invalid = body(JointType::Prismatic);
        invalid.damping = -1.;
        assert!(MechanicalProjection::observe(&invalid).is_err());
        invalid.damping = 0.;
        invalid.qd[0] = f64::NAN;
        assert!(MechanicalProjection::observe(&invalid).is_err());
        assert!(MechanicalProjection::observe(&body(JointType::Fixed)).is_err());
        assert!(MechanicalProjection::observe(&body(JointType::Prismatic)).is_ok());
    }

    #[test]
    fn nonlinear_pendulum_energy_momentum_and_power_are_observation_only() {
        let engine = body(JointType::Revolute);
        let before = (engine.q.clone(), engine.qd.clone(), engine.regime_counts());
        let root = PowerGraph::mechanism(&engine, 1.0, None).unwrap();
        assert!((root.energy.unwrap() - (9.0 - 2.0 * 9.80665 * 0.4_f64.cos())).abs() < 1e-10);
        assert_eq!(root.supplied_power, Some(6.0));
        assert_eq!(root.dissipated_power, Some(4.5));
        let lagrangian = root
            .equations
            .iter()
            .find(|e| e.label == "Lagrange-függvény")
            .unwrap();
        assert!((lagrangian.value.unwrap() - (9.0 + 2.0 * 9.80665 * 0.4_f64.cos())).abs() < 1e-10);
        assert!(
            root.equations
                .iter()
                .any(|e| e.label == "Euler–Lagrange-egyenlet")
        );
        assert!(root.equation_note.contains("invertálható"));
        let part = PowerGraph::mechanism(&engine, 1.0, Some(0)).unwrap();
        assert_eq!(part.selection, Some(DeclId::new(1)));
        assert!(
            (part
                .nodes
                .iter()
                .find(|n| n.key == "p0")
                .unwrap()
                .value
                .unwrap()
                - 6.0)
                .abs()
                < 1e-10
        );
        assert_eq!(root.energy, part.energy);
        assert!(part.supplied_power.is_none());
        assert_eq!(part.equations.len(), 4);
        assert!(part.equation_note.contains("nem önálló"));
        assert_eq!(
            before,
            (engine.q.clone(), engine.qd.clone(), engine.regime_counts())
        );
        assert!(PowerGraph::mechanism(&engine, 1.0, Some(99)).is_none());
        let structural = PowerGraph::kinematics(engine.model(), &engine.q, None).unwrap();
        assert!(structural.energy.is_none() && structural.nodes.iter().all(|n| n.energy.is_none()));
        assert!(structural.equations.iter().all(|e| e.value.is_none()));
        assert!(PowerGraph::kinematics(engine.model(), &[], None).is_none());
    }
    #[test]
    fn bounded_graph_caps_nodes_and_connections_without_dangling_edges() {
        let mut graph = PowerGraph::new("bounded".into(), None, 0.0, None);
        for i in 0..501 {
            graph.push(Node::scalar(
                i.to_string(),
                DeclId::new(i),
                i.to_string(),
                None,
                "",
            ));
        }
        for _ in 0..2001 {
            graph.connect("0".into(), "1".into());
        }
        graph.connect("0".into(), "500".into());
        assert_eq!(graph.nodes.len(), 500);
        assert_eq!(graph.edges.len(), 2000);
        assert!(graph.truncated);
        let layout = graph.arrangement().unwrap();
        assert_eq!(layout.len(), 500);
        assert!(
            layout[1][0] < layout[0][0],
            "positive body precedes negative body"
        );
        assert_eq!(layout, graph.arrangement().unwrap());
        graph.edges[0].negative = "absent".into();
        assert!(graph.arrangement().is_none());
    }
    #[test]
    fn layered_layout_handles_cycles_parallel_edges_and_budget_without_overlap() {
        let edges = [(0, 1), (1, 2), (2, 0), (0, 1)];
        let positions = crate::layout::layered(5, &edges).unwrap();
        assert_eq!(positions, crate::layout::layered(5, &edges).unwrap());
        for i in 0..5 {
            for j in 0..i {
                assert_ne!(positions[i], positions[j]);
            }
        }
        assert!(crate::layout::layered(501, &[]).is_none());
        assert!(crate::layout::layered(2, &[(0, 2)]).is_none());
        assert!(crate::layout::layered(2, &vec![(0, 1); 2001]).is_none());
        assert_eq!(crate::layout::layered(0, &[]), Some(vec![]));
        let large: Vec<_> = (0..2000).map(|i| (i % 500, (i + 1) % 500)).collect();
        let start = std::time::Instant::now();
        for _ in 0..20 {
            assert_eq!(crate::layout::layered(500, &large).unwrap().len(), 500);
        }
        assert!(
            start.elapsed().as_secs_f64() < 1.0,
            "bounded diagram layout budget"
        );
    }
}
