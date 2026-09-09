//! Bounded, read-only power views of existing physical projections.
//! This is an observation result, not an IR, solver, or independent state model.
use hymeko::common::ids::DeclId;
use hymeko_geometry::{
    articulated::AbaPlan, engine::Engine, kinematics::kinematic::KinematicModel,
};

/// A conjugate physical port or a public interface declared by the owner.
#[derive(Clone, Debug)]
pub struct Port {
    pub decl: DeclId,
    pub name: String,
}
/// A visible storage, dissipation, boundary or encapsulated subsystem.
#[derive(Clone, Debug)]
pub struct Node {
    pub key: String,
    pub decl: DeclId,
    pub name: String,
    pub scope: Option<usize>,
    pub value: Option<f64>,
    pub unit: String,
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
#[derive(Clone, Debug)]
pub struct PowerGraph {
    pub name: String,
    pub scope: Option<usize>,
    pub parent: Option<usize>,
    pub selection: Option<DeclId>,
    pub nodes: Vec<Node>,
    pub edges: Vec<(String, String)>,
    pub energy: Option<f64>,
    pub energy_unit: String,
    pub supplied_power: Option<f64>,
    pub dissipated_power: Option<f64>,
    pub time: f64,
    pub note: String,
    pub truncated: bool,
}
impl PowerGraph {
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
        if a != b && self.nodes.iter().any(|n| n.key == a) && self.nodes.iter().any(|n| n.key == b)
        {
            if self.edges.len() < 2000 {
                self.edges.push((a, b));
            } else {
                self.truncated = true;
            }
        }
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
        for p in &system.blueprint.ports {
            if let (Some(a), Some(b)) = (endpoint(p.source), endpoint(p.target)) {
                out.connect(a, b);
            }
        }
        Some(out)
    }
}

mod mechanical;
mod continuum;

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
    #[test]
    fn nonlinear_pendulum_energy_momentum_and_power_are_observation_only() {
        let engine = body(JointType::Revolute);
        let before = (engine.q.clone(), engine.qd.clone(), engine.regime_counts());
        let root = PowerGraph::mechanism(&engine, 1.0, None).unwrap();
        assert!((root.energy.unwrap() - (9.0 - 2.0 * 9.80665 * 0.4_f64.cos())).abs() < 1e-10);
        assert_eq!(root.supplied_power, Some(6.0));
        assert_eq!(root.dissipated_power, Some(4.5));
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
        assert_eq!(
            before,
            (engine.q.clone(), engine.qd.clone(), engine.regime_counts())
        );
        assert!(PowerGraph::mechanism(&engine, 1.0, Some(99)).is_none());
        let structural = PowerGraph::kinematics(engine.model(), &engine.q, None).unwrap();
        assert!(structural.energy.is_none() && structural.nodes.iter().all(|n| n.energy.is_none()));
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
    }
}
