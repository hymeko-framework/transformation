//! Run a textual query (`hymeko_query::predicate_expr`) over HyMeKo models and print the matches.
//!
//!   cargo run -p hymeko_query --example query -- [--fields a,b] [--json] [--imported] QUERY PATH...
//!
//! A PATH is a `.hymeko` file or a directory searched recursively. Each file is compiled on its
//! own by Core, so its imports resolve as usual. Only declarations written in that file are
//! reported unless `--imported` is given; a vocabulary imported by many files is listed once,
//! where it is written. A file that does not compile is reported on stderr and skipped.
//!
//! Example: the projects (not the project kinds) in Mission Control's declarations, with labels:
//!   query --fields label,state "INHERITS(project) AND NOT BASETAG(isa)" ../mission_control/declarations
use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{DeclKind, Ir, ValueR};
use hymeko::module_store::{execution::CompiledModel, source_provider::StdFsProvider};
use hymeko::resolution::string_table::StringTable;
use hymeko::util::real_parser::RealParser;
use hymeko_query::{QueryEngine, predicate_expr};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

struct Hit {
    file: String,
    path: String,
    kind: &'static str,
    base: String,
    tags: Vec<String>,
    fields: Vec<(String, Option<String>)>,
}

fn files(path: &Path, out: &mut Vec<PathBuf>) {
    if path.is_dir() {
        let mut entries: Vec<_> = std::fs::read_dir(path).into_iter().flatten().flatten().map(|e| e.path()).collect();
        entries.sort();
        for entry in entries {
            files(&entry, out);
        }
    } else if path.extension().is_some_and(|e| e == "hymeko") {
        out.push(path.to_path_buf());
    }
}

fn dotted(ir: &Ir, st: &StringTable, mut did: DeclId) -> String {
    let mut parts = Vec::new();
    while !did.is_none() {
        parts.push(st.resolve(ir.decl_nodes[did.raw()].name).to_owned());
        did = ir.parent(did);
    }
    parts.reverse();
    parts.join(".")
}

fn render(value: &ValueR, ir: &Ir, st: &StringTable) -> String {
    match value {
        ValueR::Num(n) => format!("{n}"),
        ValueR::Str(s) => st.resolve(*s).to_owned(),
        ValueR::Ref(d) if ir.decl_node(*d).is_some() => dotted(ir, st, *d),
        ValueR::List(items) => format!("[{}]", items.iter().map(|v| render(v, ir, st)).collect::<Vec<_>>().join(", ")),
        other => format!("{other:?}"),
    }
}

fn json(text: &str) -> String {
    let mut out = String::from("\"");
    for c in text.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1).peekable();
    let (mut fields, mut as_json, mut imported) = (Vec::new(), false, false);
    while let Some(flag) = args.peek().filter(|a| a.starts_with("--")).cloned() {
        args.next();
        match flag.as_str() {
            "--json" => as_json = true,
            "--imported" => imported = true,
            "--fields" => fields = args.next().unwrap_or_default().split(',').filter(|f| !f.is_empty()).map(String::from).collect(),
            other => {
                eprintln!("unknown option {other}");
                return ExitCode::from(2);
            }
        }
    }
    let (Some(query), paths) = (args.next(), args.collect::<Vec<_>>()) else {
        eprintln!("usage: query [--fields a,b] [--json] [--imported] QUERY PATH...");
        return ExitCode::from(2);
    };
    let predicate = match predicate_expr::parse(&query) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("{query}\n{}^ {e}", " ".repeat(e.position));
            return ExitCode::from(2);
        }
    };
    let mut sources = Vec::new();
    for path in &paths {
        files(Path::new(path), &mut sources);
    }
    let (mut hits, mut failed) = (Vec::new(), 0);
    for source in &sources {
        let model = match CompiledModel::compile(StdFsProvider::new(), RealParser, source) {
            Ok(m) => m,
            Err(e) => {
                eprintln!("skipped {}: {e:?}", source.display());
                failed += 1;
                continue;
            }
        };
        let ir = &model.program().ir;
        let st = StringTable::from_interner(model.interner());
        let own = std::fs::canonicalize(source).ok();
        let engine = QueryEngine::new(ir, &st);
        for m in engine.query_iter(&predicate) {
            let file = ir.decl_file.get(m.id.raw()).filter(|f| **f != u32::MAX).and_then(|f| ir.files.get(*f as usize));
            let written_here = file.is_some_and(|f| std::fs::canonicalize(f).ok() == own);
            if !imported && !written_here {
                continue;
            }
            let base = predicate_expr::decl_kind_name(m.id, ir, &st).to_owned();
            let tags = ir[ir.decl_nodes[m.id.raw()].anno].tags.iter().map(|t| st.resolve(*t).to_owned()).collect();
            let fields = fields.iter().map(|f| (f.clone(), engine.effective_value(m.id, f).map(|v| render(v, ir, &st)))).collect();
            let kind = match m.kind {
                DeclKind::Node => "node",
                DeclKind::Edge => "edge",
                DeclKind::HyperArc => "arc",
            };
            let file = file.map_or_else(|| source.display().to_string(), |f| f.trim_start_matches(r"\?\").trim_start_matches("//?/").to_owned());
            hits.push(Hit { file, path: dotted(ir, &st, m.id), kind, base, tags, fields });
        }
    }
    if as_json {
        let rows: Vec<String> = hits
            .iter()
            .map(|h| {
                let fields: Vec<String> = h.fields.iter().map(|(k, v)| format!("{}:{}", json(k), v.as_deref().map_or("null".into(), json))).collect();
                let tags: Vec<String> = h.tags.iter().map(|t| json(t)).collect();
                format!(
                    "{{\"file\":{},\"path\":{},\"kind\":{},\"base\":{},\"tags\":[{}],\"fields\":{{{}}}}}",
                    json(&h.file),
                    json(&h.path),
                    json(h.kind),
                    json(&h.base),
                    tags.join(","),
                    fields.join(",")
                )
            })
            .collect();
        println!("[{}]", rows.join(",\n "));
    } else {
        for h in &hits {
            let fields: Vec<String> = h.fields.iter().map(|(k, v)| format!("{k}={}", v.as_deref().unwrap_or("-"))).collect();
            println!("{}\t{}\t{}\t{}\t{}", h.path, h.kind, h.base, fields.join("  "), h.file);
        }
        eprintln!("{} matches in {} files ({} skipped)", hits.len(), sources.len() - failed, failed);
    }
    if failed > 0 && hits.is_empty() { ExitCode::from(1) } else { ExitCode::SUCCESS }
}
