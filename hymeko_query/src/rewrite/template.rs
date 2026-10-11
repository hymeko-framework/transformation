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
//!   {{owner}}                 Name of the match's parent declaration
//!   {{bindowner:-:0}}         Parent name of the first negative binding target
//!   {{bases:<query_label>}}   Direct bases that are matches of <query_label>, comma-separated
//!   {{config:robot_name}}     Value from the config map
//!
//!   {{#each <query_label>}}   Iterate over matches of a named query
//!   {{#each <label> within <edges> from <root>}}
//!                             ... only matches reachable from <root> along the `+` -> `-`
//!                             bindings of the <edges> results (comma-separated labels, or
//!                             `specializations`); also `touching`, `outside`; an optional
//!                             trailing `except <label>` drops that label's matches;
//!                             <root> is a name or `bind:<sign>:<index|all>` (targets of
//!                             the enclosing match); a source `inverse:<label>` follows
//!                             `-` -> `+`; roots and sources are comma lists whose
//!                             `config:<key>` items expand (see `scope.rs`)
//!   {{/each}}                 End iteration
//!
//!   {{#if <field>}}           Conditional: emit block if field exists
//!   {{/if}}                   End conditional
//!
//!   {{#comment}} ... {{/comment}}  Stripped from output

use super::match_context::{FieldValue, MatchContext};
use crate::bases::{BasePolicy, bases};
use crate::engine::{ArcBinding, QueryMatch};
use crate::traits::NameResolver;
use hymeko::common::ids::DeclId;
use hymeko::ir::ir::Ir;

use std::collections::{HashMap, HashSet};
use std::fmt;
use std::fs;
use std::path::{Path, PathBuf};

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
    /// {{#each label [scope] [except X]}} ... {{/each}} — iterate over query results,
    /// optionally restricted by an [`EachFilter`].
    Each {
        label: String,
        filter: EachFilter,
        body: Vec<Block>,
    },
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

/// Restriction on the matches an `{{#each}}` iterates; the default keeps every match.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct EachFilter {
    /// Reachability scope (`within` / `touching` / `outside` ... `from` ...).
    pub scope: Option<ScopeClause>,
    /// Query label whose matches are dropped (`except <label>`).
    pub except: Option<String>,
}

/// `<mode> <sources> from <root>` in an `{{#each}}` tag.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ScopeClause {
    pub mode: ScopeMode,
    /// Edge query labels whose `+` -> `-` bindings are followed, `inverse:<label>` for
    /// `-` -> `+`, or `specializations`;
    /// a `config:<key>` item expands to the comma-separated config value.
    pub sources: Vec<String>,
    /// Root declaration names or `bind:<sign>:<index|all>` binding targets of the enclosing
    /// match (comma list; `config:<key>` items expand from the config).
    pub root: String,
}

/// Which matches a [`ScopeClause`] keeps.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScopeMode {
    /// Node inside the scope; edge with every `±` binding inside.
    Within,
    /// Node inside the scope; edge with some `±` binding inside.
    Touching,
    /// Complement of `Within`.
    Outside,
}

/// Parse the text after `#each`: `<label> [<mode> <sources> from <root>] [except <label>]`.
fn parse_each_tag(rest: &str) -> Result<(String, EachFilter), String> {
    let malformed = || format!("Malformed #each tag: `#each {rest}`");
    let mut words = rest.split_whitespace();
    let label = words.next().ok_or_else(malformed)?.to_string();
    let mut filter = EachFilter::default();
    let mut word = words.next();
    let mode = match word {
        Some("within") => Some(ScopeMode::Within),
        Some("touching") => Some(ScopeMode::Touching),
        Some("outside") => Some(ScopeMode::Outside),
        _ => None,
    };
    if let Some(mode) = mode {
        let (Some(sources), Some("from"), Some(root)) = (words.next(), words.next(), words.next())
        else {
            return Err(malformed());
        };
        let sources: Vec<String> = sources.split(',').map(str::to_string).collect();
        if sources.iter().any(String::is_empty) {
            return Err(malformed());
        }
        filter.scope = Some(ScopeClause {
            mode,
            sources,
            root: root.to_string(),
        });
        word = words.next();
    }
    if word == Some("except") {
        filter.except = Some(words.next().ok_or_else(malformed)?.to_string());
        word = words.next();
    }
    match word {
        None => Ok((label, filter)),
        Some(_) => Err(malformed()),
    }
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

        let close = src[*pos..]
            .find("}}")
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
    if let Some(rest) = tag.strip_prefix("#each ") {
        let (label, filter) = parse_each_tag(rest)?;
        let body = parse_nested_body(src, pos)?;
        out.push(Block::Each {
            label,
            filter,
            body,
        });
        return Ok(false);
    }
    if let Some(field) = tag.strip_prefix("#if ") {
        let body = parse_nested_body(src, pos)?;
        out.push(Block::If {
            field: field.trim().to_string(),
            body,
        });
        return Ok(false);
    }
    if let Some(rest) = tag.strip_prefix("#inherits ") {
        // Syntax: {{#inherits <field> "<base_name>"}}
        //   where <field> is `.` or a field path like `geometry.shape`,
        //   and <base_name> is a quoted string.
        let (field, base) =
            parse_inherits_tag(rest).ok_or_else(|| format!("Malformed #inherits tag: `{tag}`"))?;
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
pub fn render<R: NameResolver>(blocks: &[Block], ctx: &RenderContext<R>) -> String {
    let mut out = String::new();
    render_blocks(blocks, ctx, &mut out);
    out
}

fn render_blocks<R: NameResolver>(blocks: &[Block], ctx: &RenderContext<R>, out: &mut String) {
    for block in blocks {
        match block {
            Block::Literal(s) => out.push_str(s),

            Block::Interpolate(expr) => {
                let val = resolve_expr(expr, ctx);
                out.push_str(&val);
            }

            Block::Each {
                label,
                filter,
                body,
            } => {
                if let Some(matches) = ctx.results.get(label) {
                    let filter = super::scope::CompiledFilter::new(ctx, filter);
                    for m in matches.iter().filter(|m| filter.admits(m)) {
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
fn inherits_matches<R: NameResolver>(field: &str, base_name: &str, ctx: &RenderContext<R>) -> bool {
    let Some(m) = ctx.current_match else {
        return false;
    };
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
    let decl = &ir.decl_nodes[did.raw()];

    // Direct-name match on the decl itself.
    if resolver.resolve(decl.name) == base_name {
        return true;
    }

    // Pull bases list from NodeRec or EdgeRec.
    let bases: Vec<hymeko::common::ids::DeclId> = if let Some(nid) = ir.as_node(did) {
        ir.nodes[nid.raw()]
            .bases
            .iter()
            .map(|b| b.target())
            .collect()
    } else if let Some(eid) = ir.as_edge(did) {
        ir.edges[eid.raw()]
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
        "id" => m.id.raw().to_string(),
        "owner" => parent_name(ctx, m.id),

        // {{bindowner:-:0}} — the declaration that owns a binding target, e.g. the block of a
        // port, so a template can name `Block_port` without flattening the authored nesting.
        _ if expr.starts_with("bindowner:") => {
            let rest = &expr["bindowner:".len()..];
            let Some((sign, index)) = rest.split_once(':') else {
                return String::new();
            };
            match (parse_sign(sign), index.parse::<usize>()) {
                (Some(sign), Ok(index)) => nth_binding(m, sign, index)
                    .map(|b| parent_name(ctx, b.target))
                    .unwrap_or_default(),
                _ => String::new(),
            }
        }

        // {{bases:<label>}} — direct bases that are themselves matches of `<label>`, so a
        // specialization between model blocks is emitted and vocabulary metaclasses are not.
        _ if expr.starts_with("bases:") => {
            let label = &expr["bases:".len()..];
            matched_bases(ctx, m.id, label).join(", ")
        }

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
            let Some(sign) = parse_sign(parts[0]) else {
                return String::new();
            };

            let mc = MatchContext::new(ctx.ir, ctx.resolver, m);

            if parts[1] == "all" {
                let names: Vec<String> = m
                    .arc_bindings
                    .iter()
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
                let names: Vec<String> = m
                    .arc_bindings
                    .iter()
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

/// The binding sign token of `bind:` and `bindowner:`: `+`, `-`, or `~` / `0` for neutral.
pub(super) fn parse_sign(token: &str) -> Option<i8> {
    match token {
        "+" => Some(1),
        "-" => Some(-1),
        "~" | "0" => Some(0),
        _ => None,
    }
}

/// The `index`-th arc binding of `m` with the given sign, in authored order.
pub(super) fn nth_binding(m: &QueryMatch, sign: i8, index: usize) -> Option<&ArcBinding> {
    m.arc_bindings.iter().filter(|b| b.sign == sign).nth(index)
}

fn decl_name<R: NameResolver>(ctx: &RenderContext<R>, decl: DeclId) -> String {
    ctx.resolver
        .resolve(ctx.ir.decl_nodes[decl.raw()].name)
        .to_string()
}

/// Name of the parent of `decl`; empty for a root-level declaration.
fn parent_name<R: NameResolver>(ctx: &RenderContext<R>, decl: DeclId) -> String {
    let parent = ctx.ir.parent(decl);
    if parent.is_none() {
        String::new()
    } else {
        decl_name(ctx, parent)
    }
}

/// Direct bases of `decl` (every base relation, authored order) that are matches of the
/// query `label`. An unknown label yields no bases rather than every base.
fn matched_bases<R: NameResolver>(
    ctx: &RenderContext<R>,
    decl: DeclId,
    label: &str,
) -> Vec<String> {
    let Some(members) = ctx.results.get(label) else {
        return Vec::new();
    };
    let members: HashSet<DeclId> = members.iter().map(|m| m.id).collect();
    bases(ctx.ir, decl, BasePolicy::All)
        .filter(|base| members.contains(base))
        .map(|base| decl_name(ctx, base))
        .collect()
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

/// Why a [`TransformSpec`] could not be loaded from a transform registry directory.
///
/// Typed so that callers -- the Python bindings, the CLI, other crates -- can distinguish a missing
/// registry entry from a malformed transform directory without matching on formatted strings.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TransformSpecError {
    /// `<transforms_dir>/<name>` is not an existing directory.
    TransformNotFound {
        transforms_dir: String,
        name: String,
    },
    /// `queries.hymeko` is absent or unreadable.
    QueriesUnreadable { path: String, reason: String },
    /// The transform directory contains no `template.*` file.
    TemplateMissing { dir: String },
    /// A `template.*` file exists but could not be read.
    TemplateUnreadable { path: String, reason: String },
}

impl TransformSpecError {
    /// Stable machine-readable discriminant, for structured diagnostics.
    pub fn kind(&self) -> &'static str {
        match self {
            Self::TransformNotFound { .. } => "transform_not_found",
            Self::QueriesUnreadable { .. } => "queries_unreadable",
            Self::TemplateMissing { .. } => "template_missing",
            Self::TemplateUnreadable { .. } => "template_unreadable",
        }
    }
}

impl fmt::Display for TransformSpecError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TransformNotFound {
                transforms_dir,
                name,
            } => write!(f, "transform '{name}' not found under '{transforms_dir}'"),
            Self::QueriesUnreadable { path, reason }
            | Self::TemplateUnreadable { path, reason } => {
                write!(f, "failed to read {path}: {reason}")
            }
            Self::TemplateMissing { dir } => write!(f, "no template.* file found in {dir}"),
        }
    }
}

impl std::error::Error for TransformSpecError {}

impl TransformSpec {
    /// Load a spec from `<transforms_dir>/<name>/`: the `queries.hymeko` definitions plus the single
    /// `template.*` file beside them.
    ///
    /// This is the library owner of transform-spec loading, so that a binding and a CLI cannot drift
    /// apart on the registry layout.
    ///
    /// # Errors
    /// Returns a typed [`TransformSpecError`] and never terminates the process: an invalid transform
    /// directory is a recoverable caller error, which is what makes this usable from a library.
    pub fn from_dir(
        transforms_dir: impl AsRef<Path>,
        name: &str,
    ) -> Result<Self, TransformSpecError> {
        let root = transforms_dir.as_ref();
        let dir = root.join(name);
        if !dir.is_dir() {
            return Err(TransformSpecError::TransformNotFound {
                transforms_dir: root.display().to_string(),
                name: name.to_string(),
            });
        }
        let query_path = dir.join("queries.hymeko");
        let query_source =
            fs::read_to_string(&query_path).map_err(|e| TransformSpecError::QueriesUnreadable {
                path: query_path.display().to_string(),
                reason: e.to_string(),
            })?;
        let template_path =
            Self::find_template_path(&dir).ok_or_else(|| TransformSpecError::TemplateMissing {
                dir: dir.display().to_string(),
            })?;
        let template_source = fs::read_to_string(&template_path).map_err(|e| {
            TransformSpecError::TemplateUnreadable {
                path: template_path.display().to_string(),
                reason: e.to_string(),
            }
        })?;
        Ok(Self {
            name: name.to_string(),
            query_source,
            template_source,
        })
    }

    /// Deterministically pick the `template.*` file: sorted, so the choice cannot vary by filesystem.
    fn find_template_path(dir: &Path) -> Option<PathBuf> {
        let mut found: Vec<PathBuf> = fs::read_dir(dir)
            .ok()?
            .flatten()
            .map(|e| e.path())
            .filter(|p| {
                p.file_name()
                    .map(|n| n.to_string_lossy().starts_with("template."))
                    .unwrap_or(false)
            })
            .collect();
        found.sort();
        found.into_iter().next()
    }
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
            Block::Each { label, body, .. } => {
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

    mod structural_expressions {
        use super::super::*;
        use hymeko::module_store::{execution::CompiledModel, source_provider::MemProvider};
        use hymeko::util::real_parser::RealParser;

        const MODEL: &str = "description {} \
            voc { block {} port {} @link {} } \
            sys { \
                A: voc.block {} \
                B: + <isa> A { eth: voc.port {} } \
                C: voc.block { p: voc.port {} } \
                @wire: voc.link { (+ B.eth, - C.p, ~ A); } \
            }";
        const QUERIES: &str = "q {} context { blocks: block {} ports: port {} @links: link {} }";

        fn render_with(template: &str) -> String {
            let mut files = MemProvider::default();
            files.insert_file("/model.hymeko", MODEL);
            let model =
                CompiledModel::compile(files, RealParser, std::path::Path::new("/model.hymeko"))
                    .unwrap();
            let spec = TransformSpec {
                name: "structural".into(),
                query_source: QUERIES.into(),
                template_source: template.into(),
            };
            execute_transform(
                &model.program().ir,
                model.interner(),
                &spec,
                &HashMap::new(),
            )
            .unwrap()
        }

        #[test]
        fn bases_are_filtered_to_label_members() {
            let out = render_with("{{#each blocks}}{{name}}<{{bases:blocks}}>;{{/each}}");
            assert!(
                out.contains("B<A>;"),
                "specialization between blocks is emitted: {out}"
            );
            assert!(
                out.contains("A<>;"),
                "the vocabulary metaclass `block` is not a block match: {out}"
            );
            assert!(out.contains("C<>;"), "{out}");
        }

        #[test]
        fn bases_counter_test_label_decides_membership() {
            // Same match, other label: A is a block, not a port, so nothing is emitted. An
            // unknown label must not degrade to "all bases".
            let out = render_with(
                "{{#each blocks}}{{name}}<{{bases:ports}}|{{bases:missing}}>;{{/each}}",
            );
            assert!(out.contains("B<|>;"), "{out}");
        }

        #[test]
        fn if_tests_bases_expression() {
            let out = render_with(
                "{{#each blocks}}{{name}}{{#if bases:blocks}} :> {{bases:blocks}}{{/if}};{{/each}}",
            );
            assert!(out.contains("B :> A;"), "{out}");
            assert!(out.contains("A;") && !out.contains("A :>"), "{out}");
        }

        #[test]
        fn owner_names_the_parent_declaration() {
            let out = render_with("{{#each ports}}{{owner}}_{{name}};{{/each}}");
            assert!(out.contains("B_eth;") && out.contains("C_p;"), "{out}");
        }

        #[test]
        fn bindowner_qualifies_binding_targets() {
            let out = render_with(
                "{{#each links}}{{bindowner:+:0}}_{{bind:+:0}}->{{bindowner:-:0}}_{{bind:-:0}}~{{bind:~:all}}[{{bindowner:-:3}}|{{bindowner:x:0}}]{{/each}}",
            );
            assert_eq!(
                out, "B_eth->C_p~A[|]",
                "out-of-range index and bad sign are empty"
            );
        }

        #[test]
        fn owner_counter_test_root_level_decl_is_empty() {
            let mut files = MemProvider::default();
            files.insert_file("/model.hymeko", MODEL);
            let model =
                CompiledModel::compile(files, RealParser, std::path::Path::new("/model.hymeko"))
                    .unwrap();
            let ir = &model.program().ir;
            let all = crate::QueryEngine::new(ir, model.interner()).query(&crate::Predicate::Any);
            let named = |name: &str| all.iter().find(|d| d.name == name).unwrap().id;
            let (results, config) = (HashMap::new(), HashMap::new());
            let ctx = RenderContext {
                ir,
                resolver: model.interner(),
                results: &results,
                config: &config,
                current_match: None,
            };
            assert_eq!(parent_name(&ctx, named("A")), "sys");
            assert_eq!(
                parent_name(&ctx, named("sys")),
                "",
                "a root-level declaration has no owner"
            );
        }
    }
}
