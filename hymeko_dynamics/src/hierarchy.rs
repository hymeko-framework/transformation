//! Encapsulated dynamical subsystems over the owner's existing coordinate projection.
use crate::{
    read::{ReadError, Reader},
    system::Coordinate,
};
use hymeko::{common::ids::DeclId, ir::ir::DeclKind};
use std::collections::{HashMap, HashSet};

/// An explicitly exported scalar power interface, bound through Core references.
#[derive(Clone, Debug, PartialEq)]
pub struct PublicPort {
    pub decl: DeclId,
    pub name: String,
    pub coordinate: usize,
}

/// A sealed subsystem. Its stores are private; public ports form its external interface.
#[derive(Clone, Debug, PartialEq)]
pub struct Subsystem {
    pub decl: DeclId,
    pub name: String,
    pub parent: Option<usize>,
    pub ports: Vec<PublicPort>,
    /// Directly owned coordinates, excluding children's coordinates.
    pub coordinates: Vec<usize>,
}

/// Source hierarchy retained beside the assembled numerical system.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Hierarchy {
    pub systems: Vec<Subsystem>,
    pub coordinate_owner: Vec<Option<usize>>,
    scopes: Vec<Option<usize>>,
    exports: HashMap<DeclId, (usize, DeclId)>,
}

impl Hierarchy {
    pub(crate) fn project(r: &Reader<'_>, coordinates: &[Coordinate]) -> Result<Self, ReadError> {
        let mut out = Self::default();
        for (decl, node) in r.ir.decls() {
            if node.kind == DeclKind::Node && r.inherits(decl, "subsystem") {
                if out.systems.len() >= 256 {
                    return Err(Self::error(r, decl, "at most 256 subsystems"));
                }
                out.systems.push(Subsystem {
                    decl,
                    name: r.qualified(decl),
                    parent: None,
                    ports: Vec::new(),
                    coordinates: Vec::new(),
                });
            }
        }
        let index: HashMap<_, _> = out
            .systems
            .iter()
            .enumerate()
            .map(|(i, g)| (g.decl, i))
            .collect();
        out.scopes = (0..r.ir.decl_nodes.len())
            .map(|raw| {
                let mut d = DeclId::new(raw);
                for _ in 0..=r.ir.decl_nodes.len() {
                    if let Some(g) = index.get(&d) {
                        return Some(*g);
                    }
                    if d.is_none() {
                        return None;
                    }
                    d = r.ir.decl_nodes[d.raw()].parent;
                }
                None
            })
            .collect();
        for group in &mut out.systems {
            let p = r.ir.decl_nodes[group.decl.raw()].parent;
            group.parent = p.is_some().then(|| out.scopes[p.raw()]).flatten();
        }
        for i in 0..out.systems.len() {
            let mut here = Some(i);
            let mut depth = 0;
            while let Some(g) = here {
                depth += 1;
                if depth > 32 {
                    return Err(Self::error(
                        r,
                        out.systems[i].decl,
                        "subsystem depth exceeds 32",
                    ));
                }
                here = out.systems[g].parent;
            }
        }
        for (i, c) in coordinates.iter().enumerate() {
            let owner = out.scopes[c.decl.raw()];
            out.coordinate_owner.push(owner);
            if let Some(g) = owner {
                out.systems[g].coordinates.push(i);
            }
        }
        for (decl, node) in r.ir.decls() {
            if node.kind == DeclKind::Node && r.inherits(decl, "public_port") {
                let owner = out.scopes[decl.raw()].ok_or_else(|| {
                    Self::error(r, decl, "public port requires an owning subsystem")
                })?;
                let target = r.reference(decl, "bind").ok_or_else(|| {
                    Self::error(
                        r,
                        decl,
                        "public port requires a Core-resolved bind reference",
                    )
                })?;
                out.exports.insert(decl, (owner, target));
            }
        }
        let coordinate_index = coordinates
            .iter()
            .enumerate()
            .map(|(i, c)| (c.decl, i))
            .collect();
        let mut exports: Vec<_> = out.exports.iter().map(|(d, (g, _))| (*d, *g)).collect();
        exports.sort_by_key(|(d, _)| d.raw());
        for (decl, owner) in exports {
            let coordinate = out.resolve(r, decl, decl, &coordinate_index)?;
            out.systems[owner].ports.push(PublicPort {
                decl,
                name: r.qualified(decl),
                coordinate,
            });
        }
        Ok(out)
    }

    fn error(r: &Reader<'_>, at: DeclId, why: &str) -> ReadError {
        ReadError::Boundary {
            at: r.qualified(at),
            why: why.into(),
        }
    }

    pub(crate) fn resolve(
        &self,
        r: &Reader<'_>,
        from: DeclId,
        target: DeclId,
        index: &HashMap<DeclId, usize>,
    ) -> Result<usize, ReadError> {
        let mut scope = self.scopes[from.raw()];
        let mut target = target;
        let mut seen = HashSet::new();
        loop {
            if !seen.insert(target) {
                return Err(Self::error(r, from, "cyclic public port binding"));
            }
            if let Some(&coordinate) = index.get(&target) {
                if self.coordinate_owner[coordinate] != scope {
                    return Err(Self::error(
                        r,
                        from,
                        "private coordinate crosses a subsystem boundary; connect an exported port",
                    ));
                }
                return Ok(coordinate);
            }
            let Some(&(owner, bind)) = self.exports.get(&target) else {
                return Err(ReadError::Unresolved {
                    at: r.qualified(from),
                    what: r.qualified(target),
                });
            };
            if scope != Some(owner) && scope != self.systems[owner].parent {
                return Err(Self::error(
                    r,
                    from,
                    "public port bypasses an enclosing subsystem; re-export it at that boundary",
                ));
            }
            scope = Some(owner);
            target = bind;
        }
    }

    /// Is a coordinate inside this subsystem, including nested descendants?
    pub fn contains(&self, group: usize, coordinate: usize) -> bool {
        let Some(mut here) = self.coordinate_owner.get(coordinate).copied() else {
            return false;
        };
        while let Some(g) = here {
            if g == group {
                return true;
            }
            here = self.systems[g].parent;
        }
        false
    }
}

/// Instantaneous energy balance for one subtree (or the complete system).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Balance {
    pub energy: f64,
    /// Net power entering through conservative interconnections.
    pub boundary_power: f64,
    pub supplied_power: f64,
    pub dissipated_power: f64,
}

impl crate::System {
    /// Count each descendant store once; internal conservative exchanges cancel.
    pub fn subsystem_balance(&self, group: Option<usize>, x: &[f64], time: f64) -> Balance {
        assert_eq!(x.len(), self.dim());
        assert!(group.is_none_or(|g| g < self.hierarchy.systems.len()));
        let mut effort = vec![0.0; self.dim()];
        self.effort(x, &mut effort);
        let mut flow = vec![0.0; self.dim()];
        self.port_hamiltonian
            .interconnection
            .apply(&effort, &mut flow);
        let mut input = vec![0.0; self.dim()];
        self.input_at(time, &mut input);
        let mut out = Balance::default();
        for i in 0..self.dim() {
            if group.is_none_or(|g| self.hierarchy.contains(g, i)) {
                out.energy += 0.5 * x[i] * effort[i];
                out.boundary_power += effort[i] * flow[i];
                out.supplied_power += effort[i] * input[i];
                out.dissipated_power +=
                    self.port_hamiltonian.dissipation[i] * effort[i] * effort[i];
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use crate::{SourceError, project_sources, read::ReadError};
    const SOURCE: &str = r#"
fixture {}
vocabulary { parameter {} capacitive {} inductive {} @port {} subsystem {} public_port {} }
model {
    capacity: vocabulary.parameter { value 1.0; }
    weight: vocabulary.parameter { value 0.3; }
    enclosure: vocabulary.subsystem {
        left: vocabulary.subsystem {
            q: vocabulary.capacitive { capacity model.capacity; }
            out: vocabulary.public_port { bind q; }
        }
        right: vocabulary.subsystem {
            p: vocabulary.inductive { capacity model.capacity; }
            out: vocabulary.public_port { bind p; }
        }
        @exchange: vocabulary.port { (+ right.out, - left.out, ~ model.weight); }
        outside: vocabulary.public_port { bind left.out; }
    }
    reservoir: vocabulary.capacitive { capacity model.capacity; }
    @external: vocabulary.port { (+ reservoir, - enclosure.outside, ~ weight); }
}
"#;
    fn project(source: &str) -> Result<crate::System, SourceError> {
        project_sources("test.hymeko", &[("test.hymeko", source)]).map(|v| v.0)
    }
    #[test]
    fn sealed_subsystems_preserve_energy_and_nested_exports() {
        let s = project(SOURCE).unwrap();
        assert_eq!(s.hierarchy.systems.len(), 3);
        let outer = &s.hierarchy.systems[0];
        assert_eq!(outer.ports.len(), 1);
        assert_eq!(s.hierarchy.systems[1].parent, Some(0));
        let mut x = vec![1.0, 0.5, 0.0];
        let whole = s.subsystem_balance(None, &x, 0.0);
        let left = s.subsystem_balance(Some(1), &x, 0.0);
        let right = s.subsystem_balance(Some(2), &x, 0.0);
        let parent = s.subsystem_balance(Some(0), &x, 0.0);
        assert!((parent.energy - left.energy - right.energy).abs() < 1e-14);
        assert!((left.boundary_power + right.boundary_power - parent.boundary_power).abs() < 1e-14);
        assert!(left.boundary_power.abs() > 0.0);
        assert!(whole.boundary_power.abs() < 1e-14);
        for _ in 0..1000 {
            s.port_hamiltonian.step(&mut x, &[0.0; 3], 0.01, 1e-14);
        }
        assert!((s.energy(&x) - whole.energy).abs() < 1e-10);
        assert_eq!(
            s.at(s.blueprint.capacity_from[0], 2.0).hierarchy,
            s.hierarchy
        );
    }
    #[test]
    fn private_access_skipped_boundary_and_export_cycles_are_rejected() {
        for source in [
            SOURCE.replace("- left.out,", "- left.q,"),
            SOURCE.replace("- enclosure.outside,", "- enclosure.left.out,"),
            SOURCE.replace("bind left.out;", "bind outside;"),
        ] {
            assert!(matches!(
                project(&source),
                Err(SourceError::Projection(ReadError::Boundary { .. }))
            ));
        }
    }
}
