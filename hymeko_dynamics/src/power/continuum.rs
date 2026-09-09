use super::*;

impl PowerGraph {
    /// Actual spring extensions and momenta from the existing elastic pH state.
    pub fn elastic(body: &crate::softbody::SoftBody, scope: Option<usize>) -> Option<Self> {
        if scope.is_some_and(|i| i >= body.particles().len()) {
            return None;
        }
        let mut out = Self::new(
            "Elastic system".into(),
            scope,
            body.time(),
            Some(body.energy()),
        );
        if let Some(i) = scope {
            let p = &body.particles()[i];
            let v = body.velocity(i)?;
            out.name = p.name.clone();
            out.selection = Some(p.decl);
            out.energy = Some(0.5 * p.mass * v.iter().map(|v| v * v).sum::<f64>());
            for k in 0..3 {
                out.push(Node::scalar(
                    format!("p{k}"),
                    p.decl,
                    format!("momentum {k}"),
                    Some(p.mass * v[k]),
                    "kg m/s",
                ));
            }
            for (j, s) in body
                .springs()
                .iter()
                .enumerate()
                .filter(|(_, s)| s.ends.contains(&i))
            {
                out.push(Node::scalar(
                    format!("spring{j}"),
                    s.decl,
                    "Spring boundary effort".into(),
                    Some(s.stiffness * body.extensions()[j]),
                    "N",
                ));
                for k in 0..3 {
                    if s.direction[k] != 0.0 {
                        out.connect(format!("spring{j}"), format!("p{k}"));
                    }
                }
            }
        } else {
            for (i, p) in body.particles().iter().enumerate() {
                let mut n = Node::scalar(format!("particle{i}"), p.decl, p.name.clone(), None, "J");
                n.scope = Some(i);
                n.energy = body
                    .velocity(i)
                    .map(|v| 0.5 * p.mass * v.iter().map(|v| v * v).sum::<f64>());
                n.ports = body
                    .springs()
                    .iter()
                    .filter(|s| s.ends.contains(&i))
                    .map(|s| Port {
                        decl: s.decl,
                        name: "force / velocity".into(),
                    })
                    .collect();
                out.push(n);
            }
            for (j, s) in body.springs().iter().enumerate() {
                let mut n = Node::scalar(
                    format!("spring{j}"),
                    s.decl,
                    "Elastic storage".into(),
                    Some(body.extensions()[j]),
                    "m",
                );
                n.energy = Some(0.5 * s.stiffness * body.extensions()[j].powi(2));
                out.push(n);
                for &i in &s.ends {
                    out.connect(format!("spring{j}"), format!("particle{i}"));
                }
            }
        }
        out.note="Linear axial elasticity: extensions and momenta are the integrated pH state. External work and impact losses are not sampled here.".into();
        Some(out)
    }
    /// G-SPHF cell hierarchy with the diffusion model's Lyapunov storage.
    /// Its functional is not labelled joules or thermodynamic energy.
    pub fn diffusion(
        system: &crate::diffusion::DiffusionSystem,
        scope: Option<usize>,
    ) -> Option<Self> {
        let tree = system.tree();
        let root = scope.unwrap_or(tree.root);
        if root >= tree.cells.len() {
            return None;
        }
        let mut parent = vec![None; tree.cells.len()];
        for (i, c) in tree.cells.iter().enumerate() {
            for &child in &c.children {
                parent[child] = Some(i);
            }
        }
        let within = |mut i: usize, ancestor: usize| {
            loop {
                if i == ancestor {
                    return true;
                }
                match parent[i] {
                    Some(p) => i = p,
                    None => return false,
                }
            }
        };
        if !tree.leaves.iter().any(|&i| within(i, root)) {
            return None;
        }
        let energy = tree
            .leaves
            .iter()
            .enumerate()
            .filter(|(_, i)| within(**i, root))
            .map(|(k, &i)| 0.5 * tree.cells[i].volume() * system.state()[k].powi(2))
            .sum();
        let mut out = Self::new(
            "G-SPHF diffusion".into(),
            scope,
            system.time(),
            Some(energy),
        );
        out.energy_unit = "concentration^2 m^3".into();
        out.selection = Some(tree.cells[root].decl);
        out.parent = scope.and_then(|i| parent[i]).filter(|&i| i != tree.root);
        let active = tree.leaves.iter().position(|&i| i == root);
        let visible = if active.is_some() {
            vec![root]
        } else {
            tree.cells[root].children.clone()
        };
        for &i in &visible {
            let leaf = tree.leaves.iter().position(|&l| l == i);
            let mut n = Node::scalar(
                format!("cell{i}"),
                tree.cells[i].decl,
                format!("cell {}", tree.cells[i].decl.raw()),
                leaf.map(|k| system.state()[k]),
                "concentration",
            );
            if leaf.is_none() {
                n.scope = Some(i);
            }
            out.push(n);
        }
        let endpoint = |leaf: usize| {
            visible
                .iter()
                .find(|&&i| within(tree.leaves[leaf], i))
                .copied()
        };
        for (a, b, _, _) in tree.faces() {
            if let (Some(a), Some(b)) = (endpoint(a), endpoint(b)) {
                out.connect(format!("cell{a}"), format!("cell{b}"));
            }
        }
        out.note="Closed dissipative transport; connections come from owner-projected shared faces. Storage is a Lyapunov functional, not thermodynamic energy.".into();
        Some(out)
    }
    /// Observed liquid storage and authored inlet/outlet boundaries. Volume integrals
    /// are shown as such; pressure-flow power is unknown, not inferred from max pressure.
    pub fn liquid(fluid: &crate::fluid::Fluid, scope: Option<usize>) -> Option<Self> {
        if scope.is_some_and(|i| i != 0) {
            return None;
        }
        let state = fluid.observation();
        let boundary = fluid.boundary();
        let root = fluid.tree().cells[fluid.tree().root].decl;
        let mut out = Self::new(
            "Liquid domain".into(),
            scope,
            state.time,
            Some(state.energy),
        );
        out.selection = Some(root);
        let mut storage = Node::scalar(
            "liquid".into(),
            root,
            "Liquid storage".into(),
            Some(state.volume),
            "m^3",
        );
        if scope.is_none() {
            storage.scope = Some(0);
        }
        storage.ports = vec![
            Port {
                decl: boundary.inlet,
                name: "pressure / volume flow inlet".into(),
            },
            Port {
                decl: boundary.outlet,
                name: "pressure / volume flow outlet".into(),
            },
        ];
        out.push(storage);
        for (key, id, name, value) in [
            (
                "inlet",
                boundary.inlet,
                "Cumulative inlet volume",
                state.inlet,
            ),
            (
                "outlet",
                boundary.outlet,
                "Cumulative outlet volume",
                state.outlet,
            ),
        ] {
            out.push(Node::scalar(
                key.into(),
                id,
                name.into(),
                Some(value),
                "m^3",
            ));
            out.connect(key.into(), "liquid".into());
        }
        out.push(Node::scalar(
            "viscosity".into(),
            root,
            "Cumulative viscosity loss".into(),
            Some(state.viscous_loss),
            "J",
        ));
        out.connect("liquid".into(), "viscosity".into());
        out.note="Measured liquid energy and integrated boundary volumes. Instantaneous boundary power and pressure/PIC dissipation are not yet measured.".into();
        Some(out)
    }
}
