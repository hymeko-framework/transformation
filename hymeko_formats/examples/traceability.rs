//! Thin artifact client: source compilation and export belong to Core/Query/Formats.
use hymeko::{
    module_store::{execution::CompiledModel, source_provider::StdFsProvider},
    util::real_parser::RealParser,
};
use hymeko_formats::traceability::TraceabilityExport;
use std::{
    error::Error,
    path::{Path, PathBuf},
};

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.len() != 3 {
        return Err("usage: traceability INPUT.hymeko SCOPE NEW_OUTPUT_DIR".into());
    }
    let model = CompiledModel::compile(StdFsProvider::new(), RealParser, Path::new(&args[0]))
        .map_err(|e| format!("Core compile: {e:?}"))?;
    let export = TraceabilityExport::new(&model, &args[1])?;
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../transforms");
    let views = export.emit(&root)?;
    let output = Path::new(&args[2]);
    std::fs::create_dir(output)?;
    for (extension, text) in views {
        std::fs::write(output.join(format!("trace.{extension}")), text)?;
    }
    std::fs::write(
        output.join("reference.json"),
        serde_json::to_vec_pretty(&export.reference())?,
    )?;
    println!(
        "{} entities, {} named relations",
        export.projection().entities.len(),
        export.projection().relations.len()
    );
    Ok(())
}
