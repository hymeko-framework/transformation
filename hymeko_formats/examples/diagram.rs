//! Development driver for the existing canonical IR → DOT exporter.
use std::{error::Error, path::PathBuf};

use hymeko::module_store::{module_store::ModuleStore, source_provider::StdFsProvider};
use hymeko::resolution::string_table::StringTable;
use hymeko::util::real_parser::RealParser;
use hymeko_formats::snapshot::emit_dot_graph;

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if args.len() != 2 {
        return Err(
            "usage: cargo run -p hymeko_formats --example diagram -- INPUT.hymeko OUTPUT.dot"
                .into(),
        );
    }
    let source = PathBuf::from(&args[0]);
    let destination = PathBuf::from(&args[1]);
    if destination.exists() && source.canonicalize()? == destination.canonicalize()? {
        return Err("input and output must be distinct files".into());
    }
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = store
        .compile(&source)
        .map_err(|e| format!("compile failed: {e:?}"))?;
    let strings = StringTable::from_interner(&store.it);
    let title = source.file_stem().unwrap_or_default().to_string_lossy();
    let output = emit_dot_graph(&program.ir, &strings, &title);
    std::fs::write(destination, output)?;
    Ok(())
}
