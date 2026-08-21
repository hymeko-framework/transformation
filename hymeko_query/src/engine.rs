use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{DeclKind, Ir, ValueR};
use crate::predicate::*;
use crate::traits::NameResolver;

/// A single match result with captured arc bindings.
#[derive(Debug, Clone)]
pub struct QueryMatch {
    /// The matched declaration
    pub id: DeclId,
    /// Resolved human-readable name
    pub name: String,
    /// Declaration kind
    pub kind: DeclKind,
    /// Depth in the declaration tree (0 = root)
    pub depth: usize,
    /// Captured arc-reference bindings from edge patterns
    pub arc_bindings: Vec<ArcBinding>,
}

/// A captured arc reference from a matched edge.
/// Domain transforms read these directly — zero re-traversal.
#[derive(Debug, Clone)]
pub struct ArcBinding {
    /// The sign of this reference (+1, -1, 0)
    pub sign: i8,
    /// Target declaration ID
    pub target: DeclId,
    /// Resolved target name
    pub target_name: String,
    /// Optional weight values on this reference
    pub weights: Option<Vec<ValueR>>,
}

pub struct QueryConfig {
    pub max_inherit_depth: usize,
}

impl Default for QueryConfig {
    fn default() -> Self { Self { max_inherit_depth: 8 } }
}

pub struct QueryEngine<'a, R: NameResolver> {
    ir: &'a Ir,
    resolver: &'a R,
    config: QueryConfig,
}

impl<'a, R: NameResolver> QueryEngine<'a, R> {
    pub fn new(ir: &'a Ir, resolver: &'a R) -> Self {
        Self { ir, resolver, config: QueryConfig::default() }
    }

    pub fn ir(&self) -> &Ir { self.ir }
    pub fn resolver(&self) -> &R { self.resolver }

    /// Iterator-based core: lazy evaluation, early exit possible.
    pub fn query_iter<'b>(
        &'b self,
        pred: &'b Predicate,
    ) -> impl Iterator<Item = QueryMatch> + 'b {
        let ir = self.ir;
        (0..ir.decl_nodes.len())
            .map(DeclId::new)
            .filter_map(move |did| {
                if self.matches(did, pred, 0) {
                    Some(self.build_match(did))
                } else {
                    None
                }
            })
    }

    /// Collect all matches (convenience wrapper around query_iter).
    pub fn query(&self, pred: &Predicate) -> Vec<QueryMatch> {
        self.query_iter(pred).collect()
    }

    /// First match only (early exit).
    pub fn query_first(&self, pred: &Predicate) -> Option<QueryMatch> {
        self.query_iter(pred).next()
    }

    /// Batch: run multiple named queries, return results keyed by label.
    pub fn query_batch(&self, queries: &[NamedQuery]) -> Vec<(String, Vec<QueryMatch>)> {
        queries.iter().map(|nq| {
            (nq.label.clone(), self.query(&nq.predicate))
        }).collect()
    }

    /// Check if a DeclId matches a predicate.
    fn matches(&self, did: DeclId, pred: &Predicate, depth: usize) -> bool {
        if did.is_none() { return false; }
        let decl = &self.ir.decl_nodes[did.0];

        match pred {
            Predicate::Any => true,
            Predicate::Kind(k) => decl.kind == *k,
            Predicate::Named(n) => self.resolver.resolve(decl.name) == n.as_str(),
            Predicate::NamePrefix(p) => self.resolver.resolve(decl.name).starts_with(p.as_str()),

            Predicate::InheritsFrom(base) => {
                self.check_inheritance(did, base, self.config.max_inherit_depth)
            }

            Predicate::HasTag(tag) => {
                decl.anno.tags.iter().any(|&t| self.resolver.resolve(t) == tag.as_str())
            }

            Predicate::HasChild(inner) => {
                self.ir.decl_children(did).any(|c| self.matches(c, inner, depth + 1))
            }

            Predicate::HasParent(inner) => {
                let parent = self.ir.parent(did);
                !parent.is_none() && self.matches(parent, inner, depth)
            }

            Predicate::HasValue(vp) => self.match_value(did, vp),

            Predicate::ChildValue(name, vp) => {
                self.ir.decl_children(did).any(|c| {
                    let cn = &self.ir.decl_nodes[c.0];
                    self.resolver.resolve(cn.name) == name.as_str()
                        && self.match_value(c, vp)
                })
            }

            Predicate::HasPlusRef(inner) => self.match_signed_ref(did, 1, inner),
            Predicate::HasMinusRef(inner) => self.match_signed_ref(did, -1, inner),
            Predicate::HasNeutralRef(inner) => self.match_signed_ref(did, 0, inner),
            Predicate::HasRef(inner) => {
                self.match_signed_ref(did, 1, inner)
                    || self.match_signed_ref(did, -1, inner)
                    || self.match_signed_ref(did, 0, inner)
            }

            Predicate::And(preds) => preds.iter().all(|p| self.matches(did, p, depth)),
            Predicate::Or(preds) => preds.iter().any(|p| self.matches(did, p, depth)),
            Predicate::Not(inner) => !self.matches(did, inner, depth),
        }
    }

    /// Walk the base chain up to max_depth.
    fn check_inheritance(&self, did: DeclId, target_base: &str, max_depth: usize) -> bool {
        if max_depth == 0 { return false; }

        let decl = &self.ir.decl_nodes[did.0];
        let bases = match decl.kind {
            DeclKind::Node => self.ir.as_node(did).map(|nid| &self.ir.nodes[nid.0].bases),
            DeclKind::Edge => self.ir.as_edge(did).map(|eid| &self.ir.edges[eid.0].bases),
            DeclKind::HyperArc => None,
        };

        if let Some(bases) = bases {
            for b in bases {
                let target_did = b.target();
                if !target_did.is_none() {
                    let bn = self.resolver.resolve(self.ir.decl_nodes[target_did.0].name);
                    if bn == target_base {
                        return true;
                    }
                    if self.check_inheritance(target_did, target_base, max_depth - 1) {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// Check signed arc references for an edge declaration.
    fn match_signed_ref(&self, did: DeclId, sign: i8, inner: &Predicate) -> bool {
        let Some(eid) = self.ir.as_edge(did) else { return false; };
        let edge = &self.ir.edges[eid.0];

        for &aid in &edge.arcs {
            let arc = &self.ir.arcs[aid.0];
            for r in &arc.refs {
                if (sign == 0 || r.sign() == sign) && self.matches(r.target(), inner, 0) {
                    return true;
                }
            }
        }
        false
    }

    fn match_value(&self, did: DeclId, vp: &ValuePredicate) -> bool {
        let decl = &self.ir.decl_nodes[did.0];
        match &decl.anno.value {
            Some(ValueR::Num(n)) => vp.matches_num(*n),
            Some(ValueR::Str(s)) => vp.matches_str(self.resolver.resolve(*s)),
            _ => matches!(vp, ValuePredicate::Any),
        }
    }

    /// Build a QueryMatch with arc bindings for domain transforms.
    fn build_match(&self, did: DeclId) -> QueryMatch {
        let decl = &self.ir.decl_nodes[did.0];
        let mut bindings = Vec::new();

        // If it's an edge, capture arc bindings
        if let Some(eid) = self.ir.as_edge(did) {
            let edge = &self.ir.edges[eid.0];
            for &aid in &edge.arcs {
                let arc = &self.ir.arcs[aid.0];
                for r in &arc.refs {
                    let atom = r.atom();
                    let target_name = if !atom.target.is_none() {
                        self.resolver.resolve(self.ir.decl_nodes[atom.target.0].name).to_string()
                    } else {
                        String::new()
                    };

                    let weights = atom.weights.clone();

                    bindings.push(ArcBinding {
                        sign: r.sign(),
                        target: atom.target,
                        target_name,
                        weights,
                    });
                }
            }
        }

        // Compute depth
        let mut depth = 0;
        let mut cur = self.ir.parent(did);
        while !cur.is_none() {
            depth += 1;
            cur = self.ir.parent(cur);
        }

        QueryMatch {
            id: did,
            name: self.resolver.resolve(decl.name).to_string(),
            kind: decl.kind,
            depth,
            arc_bindings: bindings,
        }
    }
}