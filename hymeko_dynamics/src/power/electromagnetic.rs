//! Continuous Maxwell power structure derived from a resolved field declaration.
//! The constitutive law and boundary integrals stay symbolic until supplied by
//! the physical model. An analytic display alone is not an energy measurement.
use super::{Equation, Node, Port, PowerGraph};
use hymeko::{
    common::ids::DeclId,
    ir::ir::{Ir, ValueR},
    resolution::interner::Interner,
};

impl PowerGraph {
    /// Project the Maxwell structure selected by an existing HyMeKo field producer.
    ///
    /// # Preconditions
    /// `ir` and `interner` belong to the same resolved program.
    /// # Postconditions
    /// All identities refer to the source declaration. No state is advanced.
    /// Energy and power remain unknown without a material law, spatial domain and
    /// field observations; the symbolic constitutive pairs are still available.
    /// Returns None for unrelated producers, invalid roots/scopes or nonfinite time.
    pub fn electromagnetic(
        ir: &Ir,
        interner: &Interner,
        root: DeclId,
        scope: Option<usize>,
        time: f64,
    ) -> Option<Self> {
        let declaration = ir.decl_node(root)?;
        if !time.is_finite() || scope.is_some_and(|s| s > 2) {
            return None;
        }
        let reader = crate::read::Reader { ir, it: interner };
        let ValueR::Str(producer) = reader.field(root, "producer")? else {
            return None;
        };
        let law = match interner.resolve(*producer) {
            "electro.gauss_e" => "div D = rho (electric Gauss constraint)",
            "electro.gauss_b" => "div B = 0 (magnetic Gauss constraint)",
            "electro.faraday" => "dB/dt = -curl E (Faraday)",
            "electro.ampere_maxwell" => "dD/dt = curl H - J (Ampere-Maxwell)",
            "electro.wave" => "dD/dt = curl H; dB/dt = -curl E (wave)",
            _ => return None,
        };
        let mut graph = Self::new(interner.resolve(declaration.name).into(), scope, time, None);
        graph.selection = Some(root);
        graph.equation_note = "Szimbolikus Maxwell-összefüggések. A mért energia kiszámításához anyagtörvény, tartomány és mezőállapot kell. Lagrange-sűrűséghez további mezőváltozók, potenciálok, forrás- és peremfeltételek szükségesek; a pH-gráf önmagában nem elég.".into();
        if scope.is_none() {
            graph.equations = vec![
                Equation::new("Hamilton-funkcionál", "H[D,B] = ∫Ω w(D,B) dV", None, "J"),
                Equation::new("Konjugált mezők", "E = ∂w/∂D;  Hₘ = ∂w/∂B", None, ""),
                Equation::new(
                    "Maxwell-dinamika",
                    "∂D/∂t = ∇×Hₘ − J;  ∂B/∂t = −∇×E",
                    None,
                    "",
                ),
                Equation::new(
                    "Poynting-mérleg",
                    "dH/dt = −∫Ω E·J dV − ∮∂Ω (E×Hₘ)·n dA",
                    None,
                    "W",
                ),
            ];
        }
        graph.note = format!(
            "HyMeKo-derived continuous Maxwell structure: {law}. H = integral w(D,B) dV; E = d w/dD, H_field = d w/dB. For linear reciprocal materials w = (E.D + H_field.B)/2. Curl coupling is power preserving up to the Poynting boundary port. Material law, integration domain and live field samples are not supplied by this analytic description: numerical energy/power are unknown. Gauss laws constrain admissible fields; they are not time integrators."
        );
        let stores = [
            ("electric", "Electric field storage", "D / E"),
            ("magnetic", "Magnetic field storage", "B / H_field"),
            (
                "environment",
                "Sources, losses and boundary",
                "J / E; tangential E / H_field",
            ),
        ];
        if let Some(index) = scope {
            graph.name = format!("{}: {}", graph.name, stores[index].1);
            let terms: &[(&str, &str, &str)] = match index {
                0 => &[
                    ("D", "Electric displacement D", "C/m^2"),
                    ("E", "Conjugate electric field E", "V/m"),
                    ("constraint", "div D = rho", "C/m^3"),
                ],
                1 => &[
                    ("B", "Magnetic induction B", "T"),
                    ("H", "Conjugate magnetic field H", "A/m"),
                    ("constraint", "div B = 0", "T/m"),
                ],
                _ => &[
                    ("source", "Impressed current: -integral E.J_ext dV", "W"),
                    ("loss", "Ohmic loss: integral E.sigma.E dV", "W"),
                    (
                        "boundary",
                        "Outward Poynting flux: integral (E x H).n dA",
                        "W",
                    ),
                ],
            };
            for (key, name, unit) in terms {
                graph.push(Node::scalar(
                    (*key).into(),
                    root,
                    (*name).into(),
                    None,
                    unit,
                ));
            }
            if index < 2 {
                graph.connect(terms[0].0.into(), terms[1].0.into());
            }
        } else {
            for (index, (key, name, pair)) in stores.iter().enumerate() {
                let mut node = Node::scalar(
                    (*key).into(),
                    root,
                    (*name).into(),
                    None,
                    if index < 2 { "J" } else { "W" },
                );
                node.scope = Some(index);
                node.ports.push(Port {
                    decl: root,
                    name: (*pair).into(),
                });
                graph.push(node);
            }
            graph.connect("electric".into(), "magnetic".into());
            graph.connect("environment".into(), "electric".into());
            graph.connect("magnetic".into(), "environment".into());
        }
        Some(graph)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn resolved_field_semantics_ignore_scene_names_and_preserve_unknowns() {
        use hymeko::{module_store::module_store::ModuleStore, util::real_parser::RealParser};
        for producer in ["gauss_e", "gauss_b", "faraday", "ampere_maxwell", "wave"] {
            let mut provider = crate::MemoryProvider::new();
            provider.insert("field.hymeko", &format!("field {{}} renamed {{ producer \"electro.{producer}\"; }} unrelated {{ producer \"unknown\"; }}"));
            let mut store = ModuleStore::new(provider, RealParser);
            let program = store.compile(std::path::Path::new("field.hymeko")).unwrap();
            let ir = &program.ir;
            let root = ir
                .decls()
                .find(|(_, n)| store.it.resolve(n.name) == "renamed")
                .unwrap()
                .0;
            let graph = PowerGraph::electromagnetic(ir, &store.it, root, None, 0.3).unwrap();
            assert_eq!(graph.nodes.len(), 3);
            assert_eq!(graph.edges.len(), 3);
            assert_eq!(graph.equations.len(), 4);
            assert!(graph.equations.iter().all(|e| e.value.is_none()));
            assert!(
                graph
                    .equations
                    .iter()
                    .any(|e| e.expression.contains("w(D,B)"))
            );
            assert!(graph.equation_note.contains("Lagrange-sűrűséghez"));
            assert!(
                graph.energy.is_none()
                    && graph.supplied_power.is_none()
                    && graph.dissipated_power.is_none()
            );
            for node in graph.nodes {
                assert_eq!(node.decl, root);
                let sub =
                    PowerGraph::electromagnetic(ir, &store.it, root, node.scope, 0.3).unwrap();
                assert_eq!(sub.selection, Some(root));
                assert_eq!(sub.parent, None);
                assert!(sub.equations.is_empty());
                assert!(
                    sub.nodes
                        .iter()
                        .all(|n| n.value.is_none() && n.decl == root)
                );
            }
            assert!(PowerGraph::electromagnetic(ir, &store.it, root, Some(3), 0.0).is_none());
            assert!(PowerGraph::electromagnetic(ir, &store.it, root, None, f64::NAN).is_none());
            assert!(PowerGraph::electromagnetic(ir, &store.it, DeclId::NONE, None, 0.0).is_none());
            let unrelated = ir
                .decls()
                .find(|(_, n)| store.it.resolve(n.name) == "unrelated")
                .unwrap()
                .0;
            assert!(PowerGraph::electromagnetic(ir, &store.it, unrelated, None, 0.0).is_none());
        }
    }
}
