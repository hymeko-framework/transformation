//! The relational reading of a small inventory-like model: declarations, own and effective fields, bases, arcs and a
//! table per kind; then its PostgreSQL.

use hymeko::module_store::{execution::CompiledModel, source_provider::MemProvider};
use hymeko::util::real_parser::RealParser;
use hymeko_relational::postgres::{identifier, json, literal, number, script, statements};
use hymeko_relational::{ColumnType, Projection, Value, project, sql_name};

const MODEL: &str = r#"
fields {}
types {
    item { label "?"; }
    product: + <isa> item {}
    cable: + <isa> product { label "Cable"; unit -> units.metre; quantity 1; }
    @tie {}
}
units { metre { symbol "m"; } }
places { shelf {} }
stock {
    reel: types.cable { quantity 305; at -> places.shelf; Mass_KG 2; id 9; }
    screw: types.item { label "M3 'csavar'"; quantity 5; sizes [3, 4.5]; }
}
links {
    @t1: types.tie { (+ stock.reel, - stock.screw); }
}
"#;

fn projected() -> Projection {
    let mut files = MemProvider::default();
    files.insert_file("/model.hymeko", MODEL);
    let model = CompiledModel::compile(files, RealParser, std::path::Path::new("/model.hymeko")).expect("the model compiles");
    project(&model.program().ir, model.interner()).expect("the model projects")
}

fn by_path<'a>(projection: &'a Projection, path: &str) -> &'a hymeko_relational::DeclarationRow {
    projection.declarations.iter().find(|row| row.path.as_deref() == Some(path)).unwrap_or_else(|| panic!("no declaration {path}"))
}

#[test]
fn declarations_carry_their_path_parent_kind_and_whether_they_are_kinds_or_fields() {
    let p = projected();
    let reel = by_path(&p, "stock.reel");
    assert_eq!((reel.name.as_str(), reel.kind, reel.is_kind, reel.is_field), ("reel", "node", false, false));
    assert_eq!(reel.parent, Some(by_path(&p, "stock").id));
    assert!(by_path(&p, "types.cable").is_kind && by_path(&p, "types.product").is_kind);
    assert!(!by_path(&p, "types.item").is_kind, "a root kind has no <isa> base");
    assert!(by_path(&p, "stock.reel.quantity").is_field);
    assert_eq!(by_path(&p, "links.t1").kind, "edge");
    let ids: Vec<u64> = p.declarations.iter().map(|row| row.id).collect();
    assert!(ids.windows(2).all(|pair| pair[0] < pair[1]), "in declaration order");
}

#[test]
fn own_fields_and_effective_fields_with_their_providers() {
    let p = projected();
    let reel = by_path(&p, "stock.reel").id;
    let own: Vec<(&str, &Value)> = p.fields.iter().filter(|row| row.owner == reel).map(|row| (row.key.as_str(), &row.value)).collect();
    assert_eq!(own.iter().map(|(key, _)| *key).collect::<Vec<_>>(), ["quantity", "at", "Mass_KG", "id"]);
    let effective: Vec<(&str, bool, u64)> = p.effective_fields.iter().filter(|row| row.owner == reel).map(|row| (row.key.as_str(), row.own, row.provider)).collect();
    let cable = by_path(&p, "types.cable").id;
    assert_eq!(effective, [("quantity", true, reel), ("at", true, reel), ("Mass_KG", true, reel), ("id", true, reel), ("label", false, cable), ("unit", false, cable)], "own first, then what the type gives");
    let label = p.effective_fields.iter().find(|row| row.owner == reel && row.key == "label").unwrap();
    assert_eq!(label.value, Value::Text("Cable".into()), "the nearest base's value");
    let unit = p.effective_fields.iter().find(|row| row.owner == reel && row.key == "unit").unwrap();
    assert_eq!(unit.value, Value::Reference(by_path(&p, "units.metre").id));
    let sizes = p.fields.iter().find(|row| row.key == "sizes").unwrap();
    assert_eq!(sizes.value, Value::List(vec![Value::Number(3.0), Value::Number(4.5)]));
}

#[test]
fn bases_tell_an_instances_type_from_a_kinds_isa_and_arcs_keep_their_signs() {
    let p = projected();
    let reel = by_path(&p, "stock.reel").id;
    let cable = by_path(&p, "types.cable").id;
    assert_eq!(p.bases.iter().filter(|row| row.declaration == reel).map(|row| (row.base, row.isa)).collect::<Vec<_>>(), [(cable, false)]);
    assert_eq!(p.bases.iter().filter(|row| row.declaration == cable).map(|row| (row.base, row.isa)).collect::<Vec<_>>(), [(by_path(&p, "types.product").id, true)]);
    let tie = by_path(&p, "links.t1").id;
    let arcs: Vec<(u32, i8, u64)> = p.arcs.iter().filter(|row| row.edge == tie).map(|row| (row.position, row.sign, row.target)).collect();
    assert_eq!(arcs, [(0, 1, reel), (1, -1, by_path(&p, "stock.screw").id)]);
}

#[test]
fn a_table_per_kind_holds_its_instances_at_any_depth_with_typed_columns() {
    let p = projected();
    let names: Vec<&str> = p.kinds.iter().map(|kind| kind.name.as_str()).collect();
    assert_eq!(names, ["kind_item", "kind_product", "kind_cable", "kind_tie"]);
    let item = &p.kinds[0];
    assert_eq!(item.rows.iter().map(|row| row.path.as_deref().unwrap()).collect::<Vec<_>>(), ["stock.reel", "stock.screw"], "the reel is an item through cable and product");
    let columns: Vec<(&str, &str, ColumnType)> = item.columns.iter().map(|c| (c.key.as_str(), c.name.as_str(), c.column_type)).collect();
    assert_eq!(columns, [
        ("quantity", "quantity", ColumnType::Number),
        ("at", "at", ColumnType::Reference),
        ("Mass_KG", "mass_kg", ColumnType::Number),
        ("id", "id_2", ColumnType::Number),
        ("label", "label", ColumnType::Text),
        ("unit", "unit", ColumnType::Reference),
        ("sizes", "sizes", ColumnType::Json),
    ]);
    let screw = &item.rows[1];
    assert_eq!(screw.values[0], Some(Value::Number(5.0)));
    assert_eq!(screw.values[1], None, "the screw has no place");
    assert_eq!(p.kinds[2].rows.len(), 1, "only the reel is a cable");
    assert_eq!(p.kinds[3].rows.iter().map(|row| row.path.as_deref().unwrap()).collect::<Vec<_>>(), ["links.t1"], "an edge is an instance of its edge kind");
}

#[test]
fn names_become_identifiers_and_never_collide() {
    assert_eq!(sql_name("Árú-név"), "aru_nev");
    assert_eq!(sql_name("3d"), "n_3d");
    assert_eq!(sql_name("--"), "x");
    assert_eq!(sql_name(&"a".repeat(80)).len(), 48);
}

#[test]
fn postgres_quotes_text_and_names_and_spells_every_number() {
    assert_eq!(identifier("we\"ird"), "\"we\"\"ird\"");
    assert_eq!(literal("M3 'csavar'\0"), "'M3 ''csavar''\u{FFFD}'");
    assert_eq!([number(1.5), number(305.0), number(1e300), number(f64::NAN), number(f64::NEG_INFINITY)], ["1.5", "305.0", "1e300", "'NaN'::double precision", "'-Infinity'::double precision"]);
    assert_eq!(json(&Value::List(vec![Value::Text("a\"b".into()), Value::Number(f64::INFINITY), Value::Reference(7), Value::List(vec![])])), r#"["a\"b","Infinity",{"declaration":7},[]]"#);
}

#[test]
fn the_script_creates_every_table_and_fills_it_the_same_way_every_time() {
    let p = projected();
    let sql = script(&p, "leltar");
    assert_eq!(sql, script(&projected(), "leltar"), "deterministic");
    assert!(sql.starts_with("-- HyMeKo relational projection") && sql.contains("\nBEGIN;\n") && sql.ends_with("COMMIT;\n"));
    for table in ["declaration", "field", "effective_field", "base", "arc", "kind_table", "kind_column", "kind_item", "kind_product", "kind_cable", "kind_tie"] {
        assert!(sql.contains(&format!("CREATE TABLE \"leltar\".\"{table}\" (")), "{table}");
    }
    assert!(sql.contains("\"at\" bigint REFERENCES \"leltar\".\"declaration\" (id) DEFERRABLE INITIALLY DEFERRED"));
    assert!(sql.contains("'M3 ''csavar'''"));
    assert!(sql.contains("'[3.0,4.5]'::jsonb"));
    let inserts = statements(&p, "leltar").iter().filter(|statement| statement.starts_with("INSERT INTO \"leltar\".\"kind_item\"")).count();
    assert_eq!(inserts, 1);
}
