//! Mini-template engine for hypergraph query results.
//!
//! Syntax:
//!   {{name}}                  Match name
//!   {{kind}}                  "node" / "edge" / "arc"
//!   {{field:mass}}            Value of child named "mass"
//!   {{field:geometry.shape}}  Dotted path traversal
//!   {{bind:+:0}}              First positive arc binding target name
//!   {{bind:-:0}}              First negative arc binding target name
//!   {{bind:+:all}}            All positive bindings, space-separated
//!   {{config:robot_name}}     Value from the config map
//!
//!   {{#each <query_label>}}   Iterate over matches of a named query
//!   {{/each}}                 End iteration
//!
//!   {{#if <field>}}           Conditional: emit block if field exists
//!   {{/if}}                   End conditional
//!
//!   {{#comment}} ... {{/comment}}  Stripped from output

use hymeko::ir::ir::Ir;
use crate::engine::QueryMatch;
use crate::traits::NameResolver;
use super::match_context::{FieldValue, MatchContext};

use std::collections::HashMap;

/// Walk a [`FieldValue::List`] of degrees and emit space-separated
/// radian values. Non-list values pass through via
/// `to_template_string` so single scalars like `{{rad:angle}}` also
/// work.
fn deg_list_to_rad_string(v: &FieldValue) -> String {
    const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;
    fn as_num(v: &FieldValue) -> Option<f64> {
        match v {
            FieldValue::Num(n) => Some(*n),
            _ => None,
        }
    }
    match v {
        FieldValue::List(items) => items
            .iter()
            .filter_map(|it| as_num(it).map(|n| format!("{:.6}", n * DEG_TO_RAD)))
            .collect::<Vec<_>>()
            .join(" "),
        FieldValue::Num(n) => format!("{:.6}", *n * DEG_TO_RAD),
        _ => String::new(),
    }
}

/// Parsed template block.
#[derive(Clone, Debug)]
pub enum Block {
    /// Literal text.
    Literal(String),
    /// {{expr}} — interpolate a value.
    Interpolate(String),
    /// {{#each label}} ... {{/each}} — iterate over query results.
    Each { label: String, body: Vec<Block> },
    /// {{#if field}} ... {{/if}} — conditional.
    If { field: String, body: Vec<Block> },
    /// `{{#inherits field "base_name"}} ... {{/inherits}}` — render the
    /// body only when the declaration at `field` (or the current match
    /// if `field` is `.`) transitively inherits from a decl named
    /// `base_name`. Used for geometry dispatch (`box` / `cylinder` /
    /// `sphere`) without needing a full switch/case construct.
    Inherits {
        field: String,
        base: String,
        body: Vec<Block>,
    },
}

/// Parse a template string into blocks.
pub fn parse_template(src: &str) -> Result<Vec<Block>, String> {
    let mut blocks = Vec::new();
    parse_blocks(src, &mut blocks, &mut 0)?;
    Ok(blocks)
}

fn parse_blocks(src: &str, out: &mut Vec<Block>, pos: &mut usize) -> Result<(), String> {
    while *pos < src.len() {
        let Some(idx) = src[*pos..].find("{{") else {
            out.push(Block::Literal(src[*pos..].to_string()));
            *pos = src.len();
            break;
        };
        let abs = *pos + idx;
        if abs > *pos {
            out.push(Block::Literal(src[*pos..abs].to_string()));
        }
        *pos = abs + 2;

        let close = src[*pos..].find("}}")
            .ok_or_else(|| format!("Unclosed {{{{ at position {abs}"))?;
        let tag = src[*pos..*pos + close].trim().to_string();
        *pos += close + 2;

        if dispatch_tag(&tag, src, out, pos)? {
            return Ok(());
        }
    }
    Ok(())
}

/// Dispatch a single `{{tag}}` occurrence. Returns `true` when the tag
/// closes the caller's enclosing block (`/each`, `/if`, `/inherits`),
/// signalling `parse_blocks` to return from the current invocation.
fn dispatch_tag(
    tag: &str,
    src: &str,
    out: &mut Vec<Block>,
    pos: &mut usize,
) -> Result<bool, String> {
    if let Some(label) = tag.strip_prefix("#each ") {
        let body = parse_nested_body(src, pos)?;
        out.push(Block::Each { label: label.trim().to_string(), body });
        return Ok(false);
    }
    if let Some(field) = tag.strip_prefix("#if ") {
        let body = parse_nested_body(src, pos)?;
        out.push(Block::If { field: field.trim().to_string(), body });
        return Ok(false);
    }
    if let Some(rest) = tag.strip_prefix("#inherits ") {
        // Syntax: {{#inherits <field> "<base_name>"}}
        //   where <field> is `.` or a field path like `geometry.shape`,
        //   and <base_name> is a quoted string.
        let (field, base) = parse_inherits_tag(rest)
            .ok_or_else(|| format!("Malformed #inherits tag: `{tag}`"))?;
        let body = parse_nested_body(src, pos)?;
        out.push(Block::Inherits { field, base, body });
        return Ok(false);
    }
    if tag.starts_with("#comment") {
        if let Some(end) = src[*pos..].find("{{/comment}}") {
            *pos += end + "{{/comment}}".len();
        }
        return Ok(false);
    }
    if matches!(tag, "/each" | "/if" | "/inherits") {
        return Ok(true);
    }
    out.push(Block::Interpolate(tag.to_string()));
    Ok(false)
}

/// Parse a body nested inside an opening tag, up to the matching
/// closer (`/each`, `/if`, or `/inherits`). The closer is consumed by
/// the recursive `parse_blocks` call and reported via its `Ok(())`
/// return (driven by `dispatch_tag` returning `true`).
fn parse_nested_body(src: &str, pos: &mut usize) -> Result<Vec<Block>, String> {
    let mut body = Vec::new();
    parse_blocks(src, &mut body, pos)?;
    Ok(body)
}

/// Rendering context.
pub struct RenderContext<'a, R: NameResolver> {
    pub ir: &'a Ir,
    pub resolver: &'a R,
    /// Query results keyed by label.
    pub results: &'a HashMap<String, Vec<QueryMatch>>,
    /// Additional config values (robot_name, etc.).
    pub config: &'a HashMap<String, String>,
    /// Currently active match (inside an {{#each}} block).
    pub current_match: Option<&'a QueryMatch>,
}

/// Render a parsed template to a string.
pub fn render<R: NameResolver>(
    blocks: &[Block],
    ctx: &RenderContext<R>,
) -> String {
    let mut out = String::new();
    render_blocks(blocks, ctx, &mut out);
    out
}

fn render_blocks<R: NameResolver>(
    blocks: &[Block],
    ctx: &RenderContext<R>,
    out: &mut String,
) {
    for block in blocks {
        match block {
            Block::Literal(s) => out.push_str(s),

            Block::Interpolate(expr) => {
                let val = resolve_expr(expr, ctx);
                out.push_str(&val);
            }

            Block::Each { label, body } => {
                if let Some(matches) = ctx.results.get(label) {
                    for m in matches {
                        let inner_ctx = RenderContext {
                            ir: ctx.ir,
                            resolver: ctx.resolver,
                            results: ctx.results,
                            config: ctx.config,
                            current_match: Some(m),
                        };
                        render_blocks(body, &inner_ctx, out);
                    }
                }
            }

            Block::If { field, body } => {
                let val = resolve_expr(field, ctx);
                if !val.is_empty() {
                    render_blocks(body, ctx, out);
                }
            }

            Block::Inherits { field, base, body } => {
                if inherits_matches(field, base, ctx) {
                    render_blocks(body, ctx, out);
                }
            }
        }
    }
}

/// Split an `#inherits` tag into (field_path, base_name).
///
/// Forms supported:
/// - `. "base"`             — current match inherits from base
/// - `field.path "base"`    — declaration at field path inherits
fn parse_inherits_tag(rest: &str) -> Option<(String, String)> {
    let rest = rest.trim();
    let quote_start = rest.find('"')?;
    let field = rest[..quote_start].trim().to_string();
    let after = &rest[quote_start + 1..];
    let quote_end = after.find('"')?;
    let base = after[..quote_end].to_string();
    if field.is_empty() || base.is_empty() {
        return None;
    }
    Some((field, base))
}

/// Resolve a field path to a DeclId, then ask the MatchContext whether
/// that declaration transitively inherits from `base_name`.
fn inherits_matches<R: NameResolver>(
    field: &str,
    base_name: &str,
    ctx: &RenderContext<R>,
) -> bool {
    let Some(m) = ctx.current_match else { return false; };
    let mc = MatchContext::new(ctx.ir, ctx.resolver, m);
    let target_did = if field == "." {
        m.id
    } else {
        match mc.resolve_field_decl(field) {
            Some(d) => d,
            None => return false,
        }
    };
    decl_inherits_from(ctx.ir, ctx.resolver, target_did, base_name, 16)
}

/// Walk the decl's bases transitively up to `max_depth` levels and
/// return `true` if any of them is named `base_name`. 16 levels is
/// more than deep enough for any real hypergraph hierarchy.
fn decl_inherits_from<R: NameResolver>(
    ir: &hymeko::ir::ir::Ir,
    resolver: &R,
    did: hymeko::common::ids::DeclId,
    base_name: &str,
    max_depth: usize,
) -> bool {
    if max_depth == 0 || did.is_none() {
        return false;
    }
    let decl = &ir.decl_nodes[did.0];

    // Direct-name match on the decl itself.
    if resolver.resolve(decl.name) == base_name {
        return true;
    }

    // Pull bases list from NodeRec or EdgeRec.
    let bases: Vec<hymeko::common::ids::DeclId> = if let Some(nid) = ir.as_node(did) {
        ir.nodes[nid.0]
            .bases
            .iter()
            .map(|b| b.target())
            .collect()
    } else if let Some(eid) = ir.as_edge(did) {
        ir.edges[eid.0]
            .bases
            .iter()
            .map(|b| b.target())
            .collect()
    } else {
        Vec::new()
    };

    for base_did in bases {
        if decl_inherits_from(ir, resolver, base_did, base_name, max_depth - 1) {
            return true;
        }
    }
    false
}

/// Resolve an expression against the current context.
fn resolve_expr<R: NameResolver>(expr: &str, ctx: &RenderContext<R>) -> String {
    // config:key
    if let Some(key) = expr.strip_prefix("config:") {
        return ctx.config.get(key).cloned().unwrap_or_default();
    }

    // Must have a current match for the rest
    let Some(m) = ctx.current_match else {
        return String::new();
    };

    match expr {
        "name" => m.name.clone(),
        "kind" => match m.kind {
            hymeko::ir::ir::DeclKind::Node => "node".into(),
            hymeko::ir::ir::DeclKind::Edge => "edge".into(),
            hymeko::ir::ir::DeclKind::HyperArc => "arc".into(),
        },
        "depth" => m.depth.to_string(),
        "id" => m.id.0.to_string(),

        _ if expr.starts_with("field:") => {
            let field_path = &expr["field:".len()..];
            let mc = MatchContext::new(ctx.ir, ctx.resolver, m);
            mc.get_field(field_path).to_template_string()
        }

        // {{rad:field_path}} — treat the field as a list of degree
        // values and emit space-separated radians. Useful for joint
        // origin RPY which HyMeKo stores in degrees and URDF / SDF
        // expect in radians.
        _ if expr.starts_with("rad:") => {
            let field_path = &expr["rad:".len()..];
            let mc = MatchContext::new(ctx.ir, ctx.resolver, m);
            deg_list_to_rad_string(&mc.get_field(field_path))
        }

        // {{nth:field_path:N}} — pick the N-th element of a list field,
        // rendered as a scalar. Out-of-range returns empty.
        _ if expr.starts_with("nth:") => {
            let rest = &expr["nth:".len()..];
            let (field_path, idx_str) = match rest.rfind(':') {
                Some(p) => (&rest[..p], &rest[p + 1..]),
                None => return String::new(),
            };
            let Ok(idx) = idx_str.parse::<usize>() else {
                return String::new();
            };
            let mc = MatchContext::new(ctx.ir, ctx.resolver, m);
            match mc.get_field(field_path) {
                FieldValue::List(items) => items
                    .get(idx)
                    .map(|v| v.to_template_string())
                    .unwrap_or_default(),
                _ => String::new(),
            }
        }

        _ if expr.starts_with("bind:") => {
            // bind:+:0, bind:-:0, bind:+:all, bind:-:all
            let rest = &expr["bind:".len()..];
            let parts: Vec<&str> = rest.splitn(2, ':').collect();
            if parts.len() < 2 {
                return String::new();
            }
            let sign: i8 = match parts[0] {
                "+" => 1,
                "-" => -1,
                "~" | "0" => 0,
                _ => return String::new(),
            };

            let mc = MatchContext::new(ctx.ir, ctx.resolver, m);

            if parts[1] == "all" {
                let names: Vec<String> = m.arc_bindings.iter()
                    .filter(|b| b.sign == sign)
                    .map(|b| b.target_name.clone())
                    .collect();
                names.join(" ")
            } else if parts[1] == "all_csv" {
                // Comma-separated: needed for emitting multi-input
                // function calls e.g. `self.mixer(a, b, c, d)`
                // when a multi-source dataflow hyperedge has many `+`
                // operands.  Used by the torch_dataflow Tier-3
                // arity_mixer fan-in.
                let names: Vec<String> = m.arc_bindings.iter()
                    .filter(|b| b.sign == sign)
                    .map(|b| b.target_name.clone())
                    .collect();
                names.join(", ")
            } else if let Ok(idx) = parts[1].parse::<usize>() {
                mc.binding_target(sign, idx).to_template_string()
            } else {
                String::new()
            }
        }

        // Fallback: try as a field path directly (shorthand for field:X)
        _ => {
            let mc = MatchContext::new(ctx.ir, ctx.resolver, m);
            let val = mc.get_field(expr);
            if val.is_missing() {
                String::new()
            } else {
                val.to_template_string()
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════
// Convenience: load + parse + execute a full transform
// ═══════════════════════════════════════════════════════════════

/// A transform specification loaded from files.
pub struct TransformSpec {
    /// Name of this transform (e.g., "urdf").
    pub name: String,
    /// Source of query definitions (parsed from .hymeko).
    pub query_source: String,
    /// Template source.
    pub template_source: String,
}

/// Execute a full query-driven transform: parse queries → run → render template.
pub fn execute_transform<R: NameResolver>(
    ir: &Ir,
    resolver: &R,
    spec: &TransformSpec,
    config: &HashMap<String, String>,
) -> Result<String, String> {
    // 1. Parse query definitions
    let ast = parser::parse_description(&spec.query_source)
        .map_err(|e| format!("Query parse error in '{}': {e:?}", spec.name))?;

    let queries = crate::interpret::interpret_transform_queries(&ast);
    if queries.is_empty() {
        return Err(format!("No queries found in transform '{}'", spec.name));
    }

    // 2. Run queries against the IR
    let engine = crate::QueryEngine::new(ir, resolver);
    let batch = engine.query_batch(&queries);

    let results: HashMap<String, Vec<QueryMatch>> = batch.into_iter().collect();

    // 3. Parse and render template
    let blocks = parse_template(&spec.template_source)
        .map_err(|e| format!("Template parse error in '{}': {e}", spec.name))?;

    let ctx = RenderContext {
        ir,
        resolver,
        results: &results,
        config,
        current_match: None,
    };

    Ok(render(&blocks, &ctx))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_template() {
        let tmpl = "Hello {{name}}, you are a {{kind}}.";
        let blocks = parse_template(tmpl).unwrap();
        assert_eq!(blocks.len(), 5); // Literal, Interp, Literal, Interp, Literal
    }

    #[test]
    fn test_parse_each_block() {
        let tmpl = "{{#each links}}<link name=\"{{name}}\"/>{{/each}}";
        let blocks = parse_template(tmpl).unwrap();
        assert_eq!(blocks.len(), 1);
        match &blocks[0] {
            Block::Each { label, body } => {
                assert_eq!(label, "links");
                assert_eq!(body.len(), 3); // literal, interp, literal
            }
            _ => panic!("expected Each block"),
        }
    }

    #[test]
    fn test_parse_nested() {
        let tmpl = "{{#each joints}}{{#if field:mass}}has mass{{/if}}{{/each}}";
        let blocks = parse_template(tmpl).unwrap();
        assert_eq!(blocks.len(), 1); // One Each
    }
}
