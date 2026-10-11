//! Scope filter for `{{#each}}`: keep only the matches that lie in the part of the model
//! reachable from a root, so one template renders one view per subsystem.
//!
//! The scope is the least declaration set that contains every declaration named by the root
//! and is closed under the configured sources:
//! - an edge query label: for each of its matches with a `+` binding inside the scope, add the
//!   `-` binding targets (`~` bindings are never followed); `inverse:<label>` follows `-` -> `+`;
//! - `specializations`: add every declaration with a direct base inside the scope.
//!
//! A declaration is *inside* when it or one of its ancestors is in the set, so a port goes with
//! its block. A root is a declaration name or `bind:<sign>:<index|all>`, the binding targets of
//! the match the enclosing `{{#each}}` is on, so a nested iteration takes its root from the outer
//! match. Roots and sources are comma-separated lists, and a `config:<key>` item is replaced
//! by the comma-separated config value, so one template serves every view. Unknown labels and
//! unresolved roots give an empty scope rather than an error, matching how the template treats
//! unknown labels elsewhere.

use super::template::{EachFilter, RenderContext, ScopeClause, ScopeMode, nth_binding, parse_sign};
use crate::bases::{BasePolicy, bases};
use crate::engine::QueryMatch;
use crate::traits::NameResolver;
use hymeko::common::ids::DeclId;
use hymeko::ir::ir::Ir;
use std::collections::HashSet;

/// Source keyword that follows specialization (base → specific) instead of an edge label.
pub(crate) const SPECIALIZATIONS: &str = "specializations";
/// Source prefix that follows an edge label backwards (`-` → `+`).
const INVERSE: &str = "inverse:";

/// The reachable declaration set of one [`ScopeClause`].
struct ScopeSet<'a> {
    ir: &'a Ir,
    members: HashSet<DeclId>,
}

impl<'a> ScopeSet<'a> {
    fn build<R: NameResolver>(ctx: &RenderContext<'a, R>, clause: &ScopeClause) -> Self {
        let mut scope = ScopeSet {
            ir: ctx.ir,
            members: resolve_roots(ctx, &clause.root),
        };
        let sources: Vec<String> = clause
            .sources
            .iter()
            .flat_map(|source| expand(ctx, source))
            .collect();
        loop {
            let before = scope.members.len();
            for source in &sources {
                if source == SPECIALIZATIONS {
                    scope.add_specializations();
                } else if let Some(label) = source.strip_prefix(INVERSE) {
                    if let Some(edges) = ctx.results.get(label) {
                        scope.add_edge_targets(edges, -1, 1);
                    }
                } else if let Some(edges) = ctx.results.get(source) {
                    scope.add_edge_targets(edges, 1, -1);
                }
            }
            if scope.members.len() == before {
                return scope;
            }
        }
    }

    /// `decl` or one of its ancestors is a member.
    fn inside(&self, decl: DeclId) -> bool {
        let mut current = decl;
        loop {
            if self.members.contains(&current) {
                return true;
            }
            let parent = self.ir.parent(current);
            if parent.is_none() {
                return false;
            }
            current = parent;
        }
    }

    /// For each edge with a `from`-signed binding inside, add its `to`-signed binding targets.
    fn add_edge_targets(&mut self, edges: &[QueryMatch], from: i8, to: i8) {
        let mut found = Vec::new();
        for edge in edges {
            if edge
                .arc_bindings
                .iter()
                .any(|b| b.sign == from && self.inside(b.target))
            {
                found.extend(
                    edge.arc_bindings
                        .iter()
                        .filter(|b| b.sign == to)
                        .map(|b| b.target),
                );
            }
        }
        self.members.extend(found);
    }

    fn add_specializations(&mut self) {
        let found: Vec<DeclId> = (0..self.ir.decl_nodes.len())
            .map(DeclId::new)
            .filter(|&decl| !self.members.contains(&decl))
            .filter(|&decl| {
                bases(self.ir, decl, BasePolicy::All).any(|b| self.members.contains(&b))
            })
            .collect();
        self.members.extend(found);
    }

    /// Whether a match lies in the scope: a node match when inside; an edge match when all
    /// (`within`) or some (`touching`) of its `±` binding targets are inside. `outside` is the
    /// complement of `within`.
    fn admits(&self, m: &QueryMatch, mode: ScopeMode) -> bool {
        let signed: Vec<DeclId> = m
            .arc_bindings
            .iter()
            .filter(|b| b.sign != 0)
            .map(|b| b.target)
            .collect();
        let within = if signed.is_empty() {
            self.inside(m.id)
        } else {
            signed.iter().all(|&decl| self.inside(decl))
        };
        match mode {
            ScopeMode::Within => within,
            ScopeMode::Touching if !signed.is_empty() => {
                signed.iter().any(|&decl| self.inside(decl))
            }
            ScopeMode::Touching => within,
            ScopeMode::Outside => !within,
        }
    }
}

/// The items of a comma-separated list, each `config:<key>` item replaced by the items of that
/// config value (a missing key contributes nothing).
fn expand<R: NameResolver>(ctx: &RenderContext<'_, R>, list: &str) -> Vec<String> {
    list.split(',')
        .flat_map(|item| match item.strip_prefix("config:") {
            Some(key) => ctx
                .config
                .get(key)
                .map(|value| value.split(',').map(|v| v.trim().to_string()).collect())
                .unwrap_or_default(),
            None => vec![item.to_string()],
        })
        .filter(|item| !item.is_empty())
        .collect()
}

/// Every root of `root`: declarations named by an item, and the binding targets of the current
/// match for `bind:<sign>:<index|all>` items (none outside an `{{#each}}`).
fn resolve_roots<R: NameResolver>(ctx: &RenderContext<'_, R>, root: &str) -> HashSet<DeclId> {
    let (bound, names): (Vec<String>, Vec<String>) = expand(ctx, root)
        .into_iter()
        .partition(|item| item.starts_with("bind:"));
    let names: HashSet<String> = names.into_iter().collect();
    let mut roots: HashSet<DeclId> = (0..ctx.ir.decl_nodes.len())
        .map(DeclId::new)
        .filter(|&decl| names.contains(ctx.resolver.resolve(ctx.ir.decl_nodes[decl.raw()].name)))
        .collect();
    if let Some(m) = ctx.current_match {
        for item in &bound {
            roots.extend(binding_targets(m, &item["bind:".len()..]));
        }
    }
    roots
}

/// Targets of `<sign>:<index|all>` on `m`; a malformed item selects nothing.
fn binding_targets(m: &QueryMatch, spec: &str) -> Vec<DeclId> {
    let Some((sign, which)) = spec.split_once(':') else {
        return Vec::new();
    };
    let Some(sign) = parse_sign(sign) else {
        return Vec::new();
    };
    match which {
        "all" => m
            .arc_bindings
            .iter()
            .filter(|b| b.sign == sign)
            .map(|b| b.target)
            .collect(),
        index => index
            .parse()
            .ok()
            .and_then(|i| nth_binding(m, sign, i))
            .map(|b| vec![b.target])
            .unwrap_or_default(),
    }
}

/// An [`EachFilter`] evaluated against one render context.
pub(crate) struct CompiledFilter<'a> {
    scope: Option<(ScopeSet<'a>, ScopeMode)>,
    except: HashSet<DeclId>,
}

impl<'a> CompiledFilter<'a> {
    pub(crate) fn new<R: NameResolver>(ctx: &RenderContext<'a, R>, filter: &EachFilter) -> Self {
        let scope = filter
            .scope
            .as_ref()
            .map(|clause| (ScopeSet::build(ctx, clause), clause.mode));
        let except = filter
            .except
            .as_ref()
            .and_then(|label| ctx.results.get(label))
            .map(|matches| matches.iter().map(|m| m.id).collect())
            .unwrap_or_default();
        CompiledFilter { scope, except }
    }

    pub(crate) fn admits(&self, m: &QueryMatch) -> bool {
        !self.except.contains(&m.id)
            && self
                .scope
                .as_ref()
                .is_none_or(|(scope, mode)| scope.admits(m, *mode))
    }
}

#[cfg(test)]
mod tests {
    use super::super::template::{TransformSpec, execute_transform, parse_template};
    use hymeko::module_store::{execution::CompiledModel, source_provider::MemProvider};
    use hymeko::util::real_parser::RealParser;
    use std::collections::HashMap;

    // Root ◆ Sub ◆ Leaf by composition; Other composes Root (upward, never followed from Root);
    // Spec specializes Base; Leaf carries a tag; links run port to port, one with a `~` binding.
    const MODEL: &str = "description {} \
        voc { block {} port {} product {} @comp {} @link {} @real {} } \
        sys { \
            Root: voc.block {} \
            Sub: voc.block { p: voc.port {} } \
            Leaf <mark>: voc.block { q: voc.port {} } \
            Other: voc.block { r: voc.port {} } \
            Base: voc.block {} \
            Spec: + <isa> Base {} \
            Prod: voc.product {} \
            @r1: voc.real { (+ Prod, - Leaf); } \
            @c1: voc.comp { (+ Root, - Sub); } \
            @c2: voc.comp { (+ Sub, - Leaf); } \
            @c3: voc.comp { (+ Other, - Root); } \
            @w_in: voc.link { (+ Sub.p, - Leaf.q, ~ Other); } \
            @w_cross: voc.link { (+ Leaf.q, - Other.r); } \
        }";
    const QUERIES: &str = "q {} context { blocks: block {} ports: port {} \
        marked <mark>: block {} products: product {} @comps: comp {} @links: link {} @reals: real {} }";

    fn render_cfg(template: &str, config: &[(&str, &str)]) -> Result<String, String> {
        let mut files = MemProvider::default();
        files.insert_file("/model.hymeko", MODEL);
        let model =
            CompiledModel::compile(files, RealParser, std::path::Path::new("/model.hymeko"))
                .unwrap();
        let spec = TransformSpec {
            name: "scope".into(),
            query_source: QUERIES.into(),
            template_source: template.into(),
        };
        let config: HashMap<String, String> = config
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect();
        execute_transform(&model.program().ir, model.interner(), &spec, &config)
    }

    fn names(template: &str) -> Vec<String> {
        let mut out: Vec<String> = render_cfg(template, &[])
            .unwrap()
            .split(';')
            .filter(|s| !s.is_empty())
            .map(str::to_string)
            .collect();
        out.sort();
        out
    }

    #[test]
    fn within_follows_composition_from_the_root_only() {
        assert_eq!(
            names("{{#each blocks within comps from Root}}{{name}};{{/each}}"),
            ["Leaf", "Root", "Sub"],
            "Other composes Root, so it is upstream and stays out"
        );
    }

    #[test]
    fn counter_test_without_clause_every_block_is_emitted() {
        // The filter, not the query, removes Other, Base and Spec above.
        assert_eq!(
            names("{{#each blocks}}{{name}};{{/each}}"),
            ["Base", "Leaf", "Other", "Root", "Spec", "Sub"]
        );
    }

    #[test]
    fn ports_are_inside_through_their_block() {
        assert_eq!(
            names("{{#each ports within comps from Root}}{{owner}}.{{name}};{{/each}}"),
            ["Leaf.q", "Sub.p"]
        );
    }

    #[test]
    fn neutral_bindings_are_never_followed() {
        // From Sub along links only: + Sub.p reaches Leaf.q, + Leaf.q reaches Other.r; the
        // `~ Other` of w_in is not followed, and a reached port does not pull in its block.
        assert_eq!(
            names("{{#each blocks within links from Sub}}{{name}};{{/each}}"),
            ["Sub"]
        );
        assert_eq!(
            names("{{#each ports within links from Sub}}{{owner}}.{{name}};{{/each}}"),
            ["Leaf.q", "Other.r", "Sub.p"]
        );
    }

    #[test]
    fn touching_keeps_edges_crossing_the_boundary_within_does_not() {
        assert_eq!(
            names("{{#each links within comps from Root}}{{name}};{{/each}}"),
            ["w_in"]
        );
        assert_eq!(
            names("{{#each links touching comps from Root}}{{name}};{{/each}}"),
            ["w_cross", "w_in"]
        );
    }

    #[test]
    fn outside_is_the_complement_of_within() {
        assert_eq!(
            names("{{#each blocks outside comps from Root}}{{name}};{{/each}}"),
            ["Base", "Other", "Spec"]
        );
        assert_eq!(
            names("{{#each links outside comps from Root}}{{name}};{{/each}}"),
            ["w_cross"]
        );
    }

    #[test]
    fn except_drops_the_matches_of_another_label() {
        assert_eq!(
            names("{{#each blocks within comps from Root except marked}}{{name}};{{/each}}"),
            ["Root", "Sub"]
        );
        assert_eq!(
            names("{{#each blocks except marked}}{{name}};{{/each}}").len(),
            5,
            "except also works without a scope"
        );
    }

    #[test]
    fn specializations_follow_bases_downward() {
        assert_eq!(
            names("{{#each blocks within specializations from Base}}{{name}};{{/each}}"),
            ["Base", "Spec"]
        );
    }

    #[test]
    fn several_sources_and_config_root() {
        let out = render_cfg(
            "{{#each blocks within comps,links from config:root}}{{name}};{{/each}}",
            &[("root", "Sub")],
        )
        .unwrap();
        let mut got: Vec<&str> = out.split(';').filter(|s| !s.is_empty()).collect();
        got.sort();
        assert_eq!(got, ["Leaf", "Sub"]);
    }

    #[test]
    fn config_lists_supply_several_roots_and_sources() {
        let out = render_cfg(
            "{{#each blocks within config:edges from config:roots}}{{name}};{{/each}}",
            &[("edges", "comps, specializations"), ("roots", "Leaf,Base")],
        )
        .unwrap();
        let mut got: Vec<&str> = out.split(';').filter(|s| !s.is_empty()).collect();
        got.sort();
        assert_eq!(got, ["Base", "Leaf", "Spec"]);
    }

    #[test]
    fn counter_test_unknown_source_follows_nothing_unknown_root_is_empty() {
        assert_eq!(
            names("{{#each blocks within nope from Root}}{{name}};{{/each}}"),
            ["Root"],
            "an unknown source label adds nothing beyond the root"
        );
        assert!(names("{{#each blocks within comps from Nobody}}{{name}};{{/each}}").is_empty());
        assert!(
            render_cfg(
                "{{#each blocks within comps from config:root}}{{name}};{{/each}}",
                &[]
            )
            .unwrap()
            .is_empty(),
            "a missing config key resolves no root"
        );
    }

    #[test]
    fn counter_test_malformed_clauses_are_parse_errors() {
        for tag in [
            "{{#each blocks within comps Root}}x{{/each}}",
            "{{#each blocks sideways comps from Root}}x{{/each}}",
            "{{#each blocks within comps, from Root}}x{{/each}}",
            "{{#each blocks except}}x{{/each}}",
            "{{#each blocks within comps from Root trailing}}x{{/each}}",
        ] {
            assert!(parse_template(tag).is_err(), "accepted: {tag}");
        }
        assert!(
            parse_template("{{#each blocks within comps from Root except marked}}x{{/each}}")
                .is_ok()
        );
    }

    #[test]
    fn bind_root_takes_the_enclosing_match() {
        let out = render_cfg(
            "{{#each comps}}{{name}}:{{#each blocks within comps from bind:-:0}}{{name}},{{/each}};{{/each}}",
            &[],
        )
        .unwrap();
        assert_eq!(out, "c1:Sub,Leaf,;c2:Leaf,;c3:Root,Sub,Leaf,;");
    }

    #[test]
    fn bind_all_roots_and_counter_test_outside_any_match() {
        let out = render_cfg(
            "{{#each links}}{{name}}:{{#each blocks within comps from bind:+:all}}{{name}},{{/each}};{{/each}}",
            &[],
        )
        .unwrap();
        assert_eq!(
            out, "w_in:;w_cross:;",
            "ports are roots, their blocks are not inside"
        );
        assert!(
            names("{{#each blocks within comps from bind:-:0}}{{name}};{{/each}}").is_empty(),
            "a bind root at top level has no match to read"
        );
        assert!(
            render_cfg(
                "{{#each comps}}{{#each blocks within comps from bind:x:0,bind:-:9}}{{name}}{{/each}}{{/each}}",
                &[]
            )
            .unwrap()
            .is_empty(),
            "a bad sign or an index past the bindings selects nothing"
        );
    }

    #[test]
    fn inverse_sources_reach_realizers_and_wholes() {
        assert_eq!(
            names("{{#each products within comps,inverse:reals from Sub}}{{name}};{{/each}}"),
            ["Prod"]
        );
        assert_eq!(
            names("{{#each blocks within inverse:comps from Leaf}}{{name}};{{/each}}"),
            ["Leaf", "Other", "Root", "Sub"],
            "upward: Sub and Root contain Leaf, Other contains Root"
        );
    }

    #[test]
    fn counter_test_without_inverse_a_realizer_is_not_reached() {
        assert!(
            names("{{#each products within comps,reals from Sub}}{{name}};{{/each}}").is_empty()
        );
        assert!(
            names("{{#each products within comps,inverse:nope from Sub}}{{name}};{{/each}}")
                .is_empty()
        );
    }
}
