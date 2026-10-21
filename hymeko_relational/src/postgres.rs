//! PostgreSQL for a [`Projection`]: the statements that create a schema's tables and fill them.
//!
//! Every reference is a foreign key to `declaration` that is checked at commit (`DEFERRABLE INITIALLY DEFERRED`), so
//! the statements hold in one transaction whatever the order of ids. [`statements`] leaves the transaction and the
//! schema's lifecycle (dropping an earlier projection) to the caller; [`script`] wraps them in one transaction for
//! `psql`. Text is quoted as a standard string literal (a NUL, which PostgreSQL text cannot hold, becomes U+FFFD);
//! a non-finite number is written as PostgreSQL spells it; a list is JSON, a reference in it `{"declaration": id}`.

use crate::{ColumnType, Projection, Value};
use std::fmt::Write as _;

/// Rows per `INSERT`.
const BATCH: usize = 500;

/// `name` as a quoted identifier.
#[must_use]
pub fn identifier(name: &str) -> String {
    format!("\"{}\"", name.replace('"', "\"\""))
}

/// `text` as a standard string literal.
#[must_use]
pub fn literal(text: &str) -> String {
    format!("'{}'", text.replace('\0', "\u{FFFD}").replace('\'', "''"))
}

/// A number as a `double precision` value: the shortest form that reads back the same, or PostgreSQL's spelling of
/// a non-finite one.
#[must_use]
pub fn number(value: f64) -> String {
    if value.is_nan() {
        "'NaN'::double precision".to_owned()
    } else if value.is_infinite() {
        format!("'{}Infinity'::double precision", if value < 0.0 { "-" } else { "" })
    } else {
        format!("{value:?}")
    }
}

fn json_string(text: &str, out: &mut String) {
    out.push('"');
    for c in text.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\0' => out.push('\u{FFFD}'),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

fn json_value(value: &Value, out: &mut String) {
    match value {
        Value::Text(text) => json_string(text, out),
        Value::Number(n) if n.is_finite() => {
            let _ = write!(out, "{n:?}");
        }
        Value::Number(n) => json_string(if n.is_nan() { "NaN" } else if *n < 0.0 { "-Infinity" } else { "Infinity" }, out),
        Value::Reference(declaration) => {
            let _ = write!(out, "{{\"declaration\":{declaration}}}");
        }
        Value::List(items) => {
            out.push('[');
            for (index, item) in items.iter().enumerate() {
                if index > 0 {
                    out.push(',');
                }
                json_value(item, out);
            }
            out.push(']');
        }
    }
}

/// A value as JSON text (a `jsonb` literal's content).
#[must_use]
pub fn json(value: &Value) -> String {
    let mut out = String::new();
    json_value(value, &mut out);
    out
}

/// A value in a column of `column_type`; none is NULL.
fn cell(value: Option<&Value>, column_type: ColumnType) -> String {
    match (value, column_type) {
        (None, _) => "NULL".to_owned(),
        (Some(Value::Text(text)), ColumnType::Text) => literal(text),
        (Some(Value::Number(n)), ColumnType::Number) => number(*n),
        (Some(Value::Reference(declaration)), ColumnType::Reference) => declaration.to_string(),
        (Some(value), _) => format!("{}::jsonb", literal(&json(value))),
    }
}

/// The four value columns of `field` and `effective_field`, the one the value's own type names filled.
fn value_cells(value: &Value) -> [String; 4] {
    let mut cells = ["NULL".to_owned(), "NULL".to_owned(), "NULL".to_owned(), "NULL".to_owned()];
    let slot = match value.column_type() {
        ColumnType::Text => 0,
        ColumnType::Number => 1,
        ColumnType::Reference => 2,
        ColumnType::Json => 3,
    };
    cells[slot] = cell(Some(value), value.column_type());
    cells
}

fn optional_text(text: Option<&str>) -> String {
    text.map_or_else(|| "NULL".to_owned(), literal)
}

fn optional_id(id: Option<u64>) -> String {
    id.map_or_else(|| "NULL".to_owned(), |id| id.to_string())
}

fn insert(statements: &mut Vec<String>, table: &str, columns: &[&str], rows: Vec<String>) {
    for chunk in rows.chunks(BATCH) {
        statements.push(format!("INSERT INTO {table} ({}) VALUES\n  {}", columns.join(", "), chunk.join(",\n  ")));
    }
}

/// The statements that create the projection's tables in `schema` (created if absent) and fill them, in order.
#[must_use]
pub fn statements(projection: &Projection, schema: &str) -> Vec<String> {
    let schema_name = identifier(schema);
    let table = |name: &str| format!("{schema_name}.{}", identifier(name));
    let declaration = table("declaration");
    let reference = format!("REFERENCES {declaration} (id) DEFERRABLE INITIALLY DEFERRED");
    let value_columns = format!("text_value text, number_value double precision, reference bigint {reference}, json_value jsonb");
    let mut out = vec![
        format!("CREATE SCHEMA IF NOT EXISTS {schema_name}"),
        format!("CREATE TABLE {declaration} (id bigint PRIMARY KEY, path text, name text NOT NULL, parent bigint {reference}, kind text NOT NULL, file text, is_kind boolean NOT NULL, is_field boolean NOT NULL)"),
        format!("CREATE INDEX ON {declaration} (path)"),
        format!("CREATE TABLE {} (declaration bigint PRIMARY KEY {reference}, owner bigint NOT NULL {reference}, key text NOT NULL, {value_columns})", table("field")),
        format!("CREATE INDEX ON {} (owner, key)", table("field")),
        format!("CREATE TABLE {} (owner bigint NOT NULL {reference}, key text NOT NULL, declaration bigint NOT NULL {reference}, provider bigint NOT NULL {reference}, own boolean NOT NULL, {value_columns}, PRIMARY KEY (owner, key))", table("effective_field")),
        format!("CREATE TABLE {} (declaration bigint NOT NULL {reference}, ordinal integer NOT NULL, base bigint NOT NULL {reference}, isa boolean NOT NULL, PRIMARY KEY (declaration, ordinal))", table("base")),
        format!("CREATE TABLE {} (edge bigint NOT NULL {reference}, arc integer NOT NULL, position integer NOT NULL, sign smallint NOT NULL, target bigint NOT NULL {reference}, PRIMARY KEY (edge, arc, position))", table("arc")),
        format!("CREATE TABLE {} (name text PRIMARY KEY, kind bigint NOT NULL {reference}, path text, instances integer NOT NULL)", table("kind_table")),
        format!("CREATE TABLE {} (table_name text NOT NULL REFERENCES {} (name) DEFERRABLE INITIALLY DEFERRED, ordinal integer NOT NULL, name text NOT NULL, key text NOT NULL, column_type text NOT NULL, PRIMARY KEY (table_name, ordinal))", table("kind_column"), table("kind_table")),
    ];
    for kind in &projection.kinds {
        let mut columns = format!("id bigint PRIMARY KEY {reference}, path text");
        for column in &kind.columns {
            let sql_type = match column.column_type {
                ColumnType::Text => "text".to_owned(),
                ColumnType::Number => "double precision".to_owned(),
                ColumnType::Reference => format!("bigint {reference}"),
                ColumnType::Json => "jsonb".to_owned(),
            };
            let _ = write!(columns, ", {} {sql_type}", identifier(&column.name));
        }
        out.push(format!("CREATE TABLE {} ({columns})", table(&kind.name)));
    }

    insert(&mut out, &declaration, &["id", "path", "name", "parent", "kind", "file", "is_kind", "is_field"], projection.declarations.iter().map(|row| {
        format!("({}, {}, {}, {}, {}, {}, {}, {})", row.id, optional_text(row.path.as_deref()), literal(&row.name), optional_id(row.parent), literal(row.kind), optional_text(row.file.as_deref()), row.is_kind, row.is_field)
    }).collect());
    insert(&mut out, &table("field"), &["declaration", "owner", "key", "text_value", "number_value", "reference", "json_value"], projection.fields.iter().map(|row| {
        format!("({}, {}, {}, {})", row.declaration, row.owner, literal(&row.key), value_cells(&row.value).join(", "))
    }).collect());
    insert(&mut out, &table("effective_field"), &["owner", "key", "declaration", "provider", "own", "text_value", "number_value", "reference", "json_value"], projection.effective_fields.iter().map(|row| {
        format!("({}, {}, {}, {}, {}, {})", row.owner, literal(&row.key), row.declaration, row.provider, row.own, value_cells(&row.value).join(", "))
    }).collect());
    insert(&mut out, &table("base"), &["declaration", "ordinal", "base", "isa"], projection.bases.iter().map(|row| format!("({}, {}, {}, {})", row.declaration, row.ordinal, row.base, row.isa)).collect());
    insert(&mut out, &table("arc"), &["edge", "arc", "position", "sign", "target"], projection.arcs.iter().map(|row| format!("({}, {}, {}, {}, {})", row.edge, row.arc, row.position, row.sign, row.target)).collect());
    insert(&mut out, &table("kind_table"), &["name", "kind", "path", "instances"], projection.kinds.iter().map(|kind| format!("({}, {}, {}, {})", literal(&kind.name), kind.kind, optional_text(kind.path.as_deref()), kind.rows.len())).collect());
    insert(&mut out, &table("kind_column"), &["table_name", "ordinal", "name", "key", "column_type"], projection.kinds.iter().flat_map(|kind| kind.columns.iter().enumerate().map(move |(ordinal, column)| {
        let column_type = match column.column_type {
            ColumnType::Text => "text",
            ColumnType::Number => "number",
            ColumnType::Reference => "reference",
            ColumnType::Json => "json",
        };
        format!("({}, {ordinal}, {}, {}, {})", literal(&kind.name), literal(&column.name), literal(&column.key), literal(column_type))
    })).collect());
    for kind in &projection.kinds {
        let mut columns = vec!["id".to_owned(), "path".to_owned()];
        columns.extend(kind.columns.iter().map(|column| identifier(&column.name)));
        let names: Vec<&str> = columns.iter().map(String::as_str).collect();
        insert(&mut out, &table(&kind.name), &names, kind.rows.iter().map(|row| {
            let mut cells = vec![row.declaration.to_string(), optional_text(row.path.as_deref())];
            cells.extend(row.values.iter().zip(&kind.columns).map(|(value, column)| cell(value.as_ref(), column.column_type)));
            format!("({})", cells.join(", "))
        }).collect());
    }
    out
}

/// [`statements`] as one `psql` script, in one transaction.
#[must_use]
pub fn script(projection: &Projection, schema: &str) -> String {
    let mut out = String::from("-- HyMeKo relational projection (hymeko_relational): generated, do not edit.\nBEGIN;\n");
    for statement in statements(projection, schema) {
        out.push_str(&statement);
        out.push_str(";\n");
    }
    out.push_str("COMMIT;\n");
    out
}
