//! The relational reading of a compiled HyMeKo model: the canonical IR as the tables a SQL database holds, and the
//! PostgreSQL that holds them ([`postgres`]).
//!
//! - `declaration` — every declaration: its id, canonical path, name, parent, kind (node, edge, arc), source file,
//!   whether it is a kind (it has an `<isa>` base) and whether it is a field (a value-carrying child of another);
//! - `field` — the fields a declaration states itself; `effective_field` — its effective fields, own else inherited,
//!   with the declaration that provides each (`hymeko_query`'s precedence, read, never re-derived);
//! - `base` — a declaration's bases in authored order: an instance's type, a kind's `<isa>` bases;
//! - `arc` — every edge's signed incidences;
//! - a table per kind (`kind_<name>`): a row per instance of it at any depth (through its bases), a column per field
//!   its instances carry, holding their effective values, typed text, number, reference (to `declaration`) or JSON;
//!   `kind_table` and `kind_column` list them.
//!
//! Identity across revisions is the canonical path; ids are the IR's, valid within one projection only. Nothing here
//! writes a model.

use hymeko::common::ids::DeclId;
use hymeko::ir::canonical_hash::build_path_index;
use hymeko::ir::ir::{DeclKind, Ir, SignedRefR, ValueR};
use hymeko_query::bases::bases;
use hymeko_query::{BasePolicy, NameResolver, QueryEngine};
use std::collections::{BTreeMap, HashMap, HashSet};

pub mod postgres;

/// A field's value as a column holds it.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Text(String),
    Number(f64),
    /// A declaration, by its id in this projection.
    Reference(u64),
    List(Vec<Value>),
}

/// The type of a column: what every value in it is; a column of mixed values (or lists) holds JSON.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ColumnType {
    Text,
    Number,
    Reference,
    Json,
}

impl Value {
    /// The column type that holds this value alone.
    #[must_use]
    pub fn column_type(&self) -> ColumnType {
        match self {
            Self::Text(_) => ColumnType::Text,
            Self::Number(_) => ColumnType::Number,
            Self::Reference(_) => ColumnType::Reference,
            Self::List(_) => ColumnType::Json,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct DeclarationRow {
    pub id: u64,
    /// The canonical dotted path; none for a declaration the path index does not name (an anonymous arc).
    pub path: Option<String>,
    pub name: String,
    pub parent: Option<u64>,
    /// `node`, `edge` or `arc`.
    pub kind: &'static str,
    pub file: Option<String>,
    pub is_kind: bool,
    pub is_field: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FieldRow {
    pub owner: u64,
    pub key: String,
    /// The field's own declaration.
    pub declaration: u64,
    pub value: Value,
}

#[derive(Clone, Debug, PartialEq)]
pub struct EffectiveFieldRow {
    pub owner: u64,
    pub key: String,
    /// The field declaration that supplies the value.
    pub declaration: u64,
    /// The declaration whose own field it is: the owner, or the base that supplies it.
    pub provider: u64,
    pub own: bool,
    pub value: Value,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BaseRow {
    pub declaration: u64,
    pub ordinal: u32,
    pub base: u64,
    /// Whether the base is written `<isa>` (a kind's specialisation) rather than an instance's type.
    pub isa: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ArcRow {
    pub edge: u64,
    pub arc: u32,
    pub position: u32,
    pub sign: i8,
    pub target: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Column {
    /// The field's key in the model.
    pub key: String,
    /// The column's name in SQL: the key made an identifier, unique in its table.
    pub name: String,
    pub column_type: ColumnType,
}

#[derive(Clone, Debug, PartialEq)]
pub struct KindRow {
    pub declaration: u64,
    pub path: Option<String>,
    /// One value per column, in column order; none where the instance has no such field.
    pub values: Vec<Option<Value>>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct KindTable {
    pub kind: u64,
    pub path: Option<String>,
    /// The table's name in SQL: `kind_` and the kind's name made an identifier, unique in the projection.
    pub name: String,
    pub columns: Vec<Column>,
    pub rows: Vec<KindRow>,
}

/// A model as tables. Rows are in declaration order, kinds in the order of their declarations, columns in the order
/// their fields first appear: the same model always gives the same projection.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Projection {
    pub declarations: Vec<DeclarationRow>,
    pub fields: Vec<FieldRow>,
    pub effective_fields: Vec<EffectiveFieldRow>,
    pub bases: Vec<BaseRow>,
    pub arcs: Vec<ArcRow>,
    pub kinds: Vec<KindTable>,
}

/// A model that cannot be projected.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProjectionError {
    /// The canonical path index cannot be built.
    Paths(String),
}

impl std::fmt::Display for ProjectionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Paths(reason) => write!(f, "the canonical path index cannot be built: {reason}"),
        }
    }
}

impl std::error::Error for ProjectionError {}

fn id(decl: DeclId) -> u64 {
    decl.raw() as u64
}

/// The base references of a declaration, in authored order: a node's or an edge's.
fn base_refs(ir: &Ir, decl: DeclId) -> &[SignedRefR] {
    if let Some((_, node)) = ir.node_for_decl(decl) {
        &node.bases
    } else if let Some((_, edge)) = ir.edge_for_decl(decl) {
        &edge.bases
    } else {
        &[]
    }
}

/// A value as a column holds it; an unresolved reference is no value.
fn convert<R: NameResolver>(value: &ValueR, resolver: &R) -> Option<Value> {
    Some(match value {
        ValueR::Str(text) => Value::Text(resolver.resolve(*text).to_owned()),
        ValueR::Num(number) => Value::Number(*number),
        ValueR::Ref(target) if target.is_none() => return None,
        ValueR::Ref(target) => Value::Reference(id(*target)),
        ValueR::List(items) => Value::List(items.iter().filter_map(|item| convert(item, resolver)).collect()),
    })
}

/// A name made a SQL identifier: lowercase ASCII letters, digits and `_`, accents dropped, never starting with a
/// digit, short enough to take a prefix and a suffix within PostgreSQL's 63 bytes.
#[must_use]
pub fn sql_name(name: &str) -> String {
    let plain = |c: char| match c {
        'á' | 'à' | 'â' | 'ä' | 'Á' | 'À' | 'Â' | 'Ä' => 'a',
        'é' | 'è' | 'ê' | 'ë' | 'É' | 'È' | 'Ê' | 'Ë' => 'e',
        'í' | 'ì' | 'î' | 'ï' | 'Í' | 'Ì' | 'Î' | 'Ï' => 'i',
        'ó' | 'ò' | 'ô' | 'ö' | 'ő' | 'Ó' | 'Ò' | 'Ô' | 'Ö' | 'Ő' => 'o',
        'ú' | 'ù' | 'û' | 'ü' | 'ű' | 'Ú' | 'Ù' | 'Û' | 'Ü' | 'Ű' => 'u',
        c if c.is_ascii_alphanumeric() => c.to_ascii_lowercase(),
        _ => '_',
    };
    let mut out: String = name.chars().map(plain).collect();
    out = out.trim_matches('_').to_owned();
    if out.is_empty() {
        out = "x".to_owned();
    }
    if out.starts_with(|c: char| c.is_ascii_digit()) {
        out.insert_str(0, "n_");
    }
    out.truncate(48);
    out
}

/// `base`, or `base_2`, `base_3`, … : the first not yet taken, which it then takes.
fn unique(base: String, taken: &mut HashSet<String>) -> String {
    if taken.insert(base.clone()) {
        return base;
    }
    (2..)
        .map(|n| format!("{base}_{n}"))
        .find(|candidate| taken.insert(candidate.clone()))
        .expect("an unbounded range has a free name")
}

/// The relational reading of a compiled model (see the crate documentation).
///
/// # Errors
/// [`ProjectionError::Paths`] when the canonical path index cannot be built.
pub fn project<R: NameResolver>(ir: &Ir, resolver: &R) -> Result<Projection, ProjectionError> {
    let paths = build_path_index(ir).map_err(|error| ProjectionError::Paths(error.to_string()))?;
    let engine = QueryEngine::new(ir, resolver);
    let isa = engine.isa_tag();
    let path_of = |decl: DeclId| {
        paths.path_of(decl).map(|key| key.0.iter().map(|sym| resolver.resolve(*sym)).collect::<Vec<_>>().join("."))
    };
    let mut projection = Projection::default();
    let mut effective: HashMap<u64, Vec<(String, Value)>> = HashMap::new();
    let mut instances = Vec::new();

    for index in 0..ir.decl_nodes.len() {
        let decl = DeclId::new(index);
        let Some(node) = ir.decl_node(decl) else { continue };
        let parent = (!node.parent.is_none()).then(|| id(node.parent));
        let is_field = node.anno.value.is_some() && parent.is_some();
        let is_kind = isa.is_some_and(|tag| bases(ir, decl, BasePolicy::Tagged(tag)).next().is_some());
        let name = resolver.resolve(node.name).to_owned();
        let file = ir
            .decl_file
            .get(index)
            .filter(|file| **file != u32::MAX)
            .and_then(|file| ir.files.get(*file as usize))
            .cloned();
        let kind = match node.kind {
            DeclKind::Node => "node",
            DeclKind::Edge => "edge",
            DeclKind::HyperArc => "arc",
        };
        if let Some(value) = node.anno.value.as_ref().filter(|_| is_field).and_then(|value| convert(value, resolver)) {
            projection.fields.push(FieldRow { owner: id(node.parent), key: name.clone(), declaration: id(decl), value });
        }
        for (ordinal, base) in base_refs(ir, decl).iter().enumerate() {
            if base.target().is_none() {
                continue;
            }
            projection.bases.push(BaseRow {
                declaration: id(decl),
                ordinal: ordinal as u32,
                base: id(base.target()),
                isa: isa.is_some_and(|tag| base.atom().anno.tags.contains(&tag)),
            });
        }
        if !is_field && node.kind != DeclKind::HyperArc {
            let mut fields = Vec::new();
            for (key, binding) in engine.effective_fields(decl) {
                let Some(value) = ir.decl_node(binding.declaration).and_then(|field| field.anno.value.as_ref()).and_then(|value| convert(value, resolver)) else { continue };
                projection.effective_fields.push(EffectiveFieldRow {
                    owner: id(decl),
                    key: key.to_owned(),
                    declaration: id(binding.declaration),
                    provider: id(binding.provider),
                    own: binding.is_own(decl),
                    value: value.clone(),
                });
                fields.push((key.to_owned(), value));
            }
            effective.insert(id(decl), fields);
            // An instance: not a kind, and of a type (it has a base).
            if !is_kind && bases(ir, decl, BasePolicy::All).next().is_some() {
                instances.push(decl);
            }
        }
        projection.declarations.push(DeclarationRow { id: id(decl), path: path_of(decl), name, parent, kind, file, is_kind, is_field });
    }

    for edge in &ir.edges {
        for (arc_ordinal, arc) in edge.arcs.iter().enumerate() {
            let Some(record) = ir.arcs.get(arc.raw()) else { continue };
            for (position, reference) in record.refs.iter().enumerate() {
                if reference.target().is_none() {
                    continue;
                }
                projection.arcs.push(ArcRow {
                    edge: id(edge.decl),
                    arc: arc_ordinal as u32,
                    position: position as u32,
                    sign: reference.sign(),
                    target: id(reference.target()),
                });
            }
        }
    }

    // Every instance under every kind its bases reach, at any depth; a repeated base or a cycle is walked once.
    let mut members: BTreeMap<u64, Vec<u64>> = BTreeMap::new();
    for &instance in &instances {
        let mut pending: Vec<DeclId> = bases(ir, instance, BasePolicy::All).collect();
        let mut seen = HashSet::new();
        while let Some(kind) = pending.pop() {
            if kind.is_none() || ir.decl_node(kind).is_none() || !seen.insert(kind) {
                continue;
            }
            members.entry(id(kind)).or_default().push(id(instance));
            pending.extend(bases(ir, kind, BasePolicy::All));
        }
    }
    let mut tables = HashSet::from(["declaration", "field", "effective_field", "base", "arc", "kind_table", "kind_column"].map(String::from));
    for (kind, mut rows) in members {
        rows.sort_unstable();
        let decl = DeclId::new(kind as usize);
        let name = unique(format!("kind_{}", sql_name(ir.decl_node(decl).map_or("", |node| resolver.resolve(node.name)))), &mut tables);
        let mut keys: Vec<String> = Vec::new();
        for row in &rows {
            for (key, _) in effective.get(row).into_iter().flatten() {
                if !keys.contains(key) {
                    keys.push(key.clone());
                }
            }
        }
        let mut taken = HashSet::from(["id".to_owned(), "path".to_owned()]);
        let rows: Vec<KindRow> = rows
            .into_iter()
            .map(|row| {
                let held = effective.get(&row);
                KindRow {
                    declaration: row,
                    path: path_of(DeclId::new(row as usize)),
                    values: keys.iter().map(|key| held.and_then(|fields| fields.iter().find(|(k, _)| k == key)).map(|(_, value)| value.clone())).collect(),
                }
            })
            .collect();
        let columns = keys
            .iter()
            .enumerate()
            .map(|(index, key)| Column {
                key: key.clone(),
                name: unique(sql_name(key), &mut taken),
                column_type: rows
                    .iter()
                    .filter_map(|row| row.values[index].as_ref().map(Value::column_type))
                    .reduce(|a, b| if a == b { a } else { ColumnType::Json })
                    .unwrap_or(ColumnType::Text),
            })
            .collect();
        projection.kinds.push(KindTable { kind, path: path_of(decl), name, columns, rows });
    }
    Ok(projection)
}
