//! Prepare a simulation bundle or record observations through the existing Core value-update API.
use hymeko::{
    ir_stream::DecodeLimits,
    module_store::{execution::CompiledModel, source_provider::StdFsProvider},
    resolution::string_table::StringTable,
    util::real_parser::RealParser,
    writers::cbor_writer,
};
use hymeko_formats::traceability::{TraceabilityExport, simulation::SimulationBundle};
use serde_json::{Value, json};
use std::{
    error::Error,
    path::{Path, PathBuf},
};

fn views(model: &CompiledModel, scope: &str, folder: &Path) -> Result<(), Box<dyn Error>> {
    std::fs::create_dir(folder)?;
    let export = TraceabilityExport::new(model, scope)?;
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../transforms");
    for (ext, text) in export.emit(&root)? {
        std::fs::write(folder.join(format!("trace.{ext}")), text)?;
    }
    std::fs::write(
        folder.join("reference.json"),
        serde_json::to_vec_pretty(&export.reference())?,
    )?;
    let strings = StringTable::from_interner(model.interner());
    std::fs::write(
        folder.join("hypergraph.json"),
        hymeko_formats::snapshot::snapshot_json(&model.program().ir, &strings)?,
    )?;
    std::fs::write(
        folder.join("hypergraph.dot"),
        hymeko_formats::snapshot::emit_dot_graph(&model.program().ir, &strings, "canonical model"),
    )?;
    Ok(())
}

fn prepare(bundle: &SimulationBundle, output: &Path) -> Result<(), Box<dyn Error>> {
    let (xml, manifest) = bundle.artifacts()?;
    let program = bundle.model().program();
    let mut workspace = program.root.0.parent().ok_or("source has no parent")?;
    while program
        .imports
        .iter()
        .any(|(_, key)| !key.0.starts_with(workspace))
    {
        workspace = workspace
            .parent()
            .ok_or("imports have no common workspace")?;
    }
    let cbor = cbor_writer::encode_compiled(workspace, program, bundle.model().interner())?;
    std::fs::create_dir(output)?;
    std::fs::write(output.join("robot.xml"), xml)?;
    std::fs::write(
        output.join("bundle.json"),
        serde_json::to_vec_pretty(&manifest)?,
    )?;
    std::fs::write(output.join("model.cbor"), cbor)?;
    views(
        bundle.model(),
        bundle.trace_scope(),
        &output.join("initial"),
    )?;
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let mode = args.first().map(String::as_str).unwrap_or("");
    if !((mode == "prepare" && args.len() == 4)
        || (mode == "record" && args.len() == 5)
        || (mode == "revise" && args.len() == 6))
    {
        return Err("usage: trace_simulation prepare SOURCE ROOT NEW_OUTPUT | record SOURCE ROOT EVIDENCE.json NEW_OUTPUT | revise SOURCE ROOT CONTROL VALUE NEW_OUTPUT".into());
    }
    let model = if args[1].ends_with(".cbor") {
        CompiledModel::from_decoded_cbor(cbor_writer::decode(
            &std::fs::read(&args[1])?,
            &DecodeLimits::default(),
        )?)
    } else {
        CompiledModel::compile(StdFsProvider::new(), RealParser, Path::new(&args[1]))
            .map_err(|e| format!("Core compile: {e:?}"))?
    };
    let bundle = SimulationBundle::new(model, &args[2])?;
    let output = Path::new(args.last().unwrap());
    if mode == "prepare" {
        prepare(&bundle, output)?;
        println!("prepared {}", output.display());
    } else if mode == "revise" {
        let retained = bundle.artifacts()?;
        let updated = bundle.with_control(&args[3], args[4].parse()?)?;
        if retained != bundle.artifacts()? {
            return Err("retained snapshot changed".into());
        }
        std::fs::create_dir(output)?;
        prepare(&bundle, &output.join("before"))?;
        prepare(&updated, &output.join("after"))?;
        std::fs::write(
            output.join("revision.json"),
            serde_json::to_vec_pretty(
                &json!({"control":args[3],"value":args[4],"before":bundle.digest(),"after":updated.digest(),"structure_preserved":bundle.model().program().canon.structure()==updated.model().program().canon.structure(),"retained_outputs_equal":true}),
            )?,
        )?;
        println!("revised {}", output.display());
    } else {
        let evidence: Value = serde_json::from_slice(&std::fs::read(&args[3])?)?;
        let (updated, verdicts) = bundle.record(&evidence)?;
        std::fs::create_dir(output)?;
        views(bundle.model(), bundle.trace_scope(), &output.join("before"))?;
        views(&updated, bundle.trace_scope(), &output.join("after"))?;
        std::fs::write(
            output.join("verdicts.json"),
            serde_json::to_vec_pretty(&verdicts)?,
        )?;
        std::fs::write(
            output.join("receipt.json"),
            serde_json::to_vec_pretty(
                &json!({"before_digest":bundle.digest(),"after_digest":format!("{:?}",updated.semantic_digest()),"structure_preserved":bundle.model().program().canon.structure()==updated.program().canon.structure(),"evidence_file":args[3]}),
            )?,
        )?;
        println!("{}", serde_json::to_string_pretty(&verdicts)?);
    }
    Ok(())
}
