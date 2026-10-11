//! File-oriented application of the Formats contract API.
use hymeko::resolution::string_table::StringTable;
use hymeko_formats::metaheuristic::{
    DE_KEY, HHO_KEY, MAX_SOURCE_BYTES, Result, compile, prompt_semantics, validate,
};
use std::{fs, io::Read, path::Path};

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let structural = args.first().is_some_and(|arg| arg == "--structural");
    let args = &args[usize::from(structural)..];
    if args.len() != 2 {
        return Err(
            "usage: metaheuristic_contract [--structural] INPUT.hymeko NEW_OUTPUT_DIR".into(),
        );
    }
    let mut source = String::new();
    fs::File::open(&args[0])?
        .take(MAX_SOURCE_BYTES + 1)
        .read_to_string(&mut source)?;
    let project = |model| {
        if structural {
            hymeko_formats::metaheuristic::catalog::project(model)
        } else {
            validate(model)
        }
    };
    let original = compile(&source)?;
    let original_contract = project(&original)?;
    let name = if structural {
        "structural_contract"
    } else if original_contract["profile"] == DE_KEY {
        "de_contract"
    } else if original_contract["profile"] == HHO_KEY {
        "hho_contract"
    } else {
        "ga_contract"
    };
    let canonical = hymeko_emitter::emit_hymeko(&original.program().ir, original.interner(), name);
    let roundtrip = compile(&canonical)?;
    let contract = project(&roundtrip)?;
    if original_contract["model_semantic_digest"] != contract["model_semantic_digest"] {
        return Err("canonical emission changed the semantic digest".into());
    }
    let output = Path::new(&args[1]);
    fs::create_dir(output)?; // never overwrite an existing result directory
    fs::write(output.join("contract.hymeko"), canonical)?;
    fs::write(
        output.join(if structural {
            "structure.json"
        } else {
            "operator_ids.json"
        }),
        serde_json::to_vec_pretty(&contract)?,
    )?;
    if !structural {
        fs::write(
            output.join("prompt_semantics.txt"),
            prompt_semantics(&contract)?,
        )?;
    }
    fs::write(
        output.join("contract.dot"),
        hymeko_formats::snapshot::emit_dot_graph(
            &roundtrip.program().ir,
            &StringTable::from_interner(roundtrip.interner()),
            &format!(
                "{} static dependency contract",
                if structural {
                    &contract["label"]
                } else {
                    &contract["profile"]
                }
            ),
        ),
    )?;
    if structural {
        println!(
            "label={} operators={} states={} digest={}",
            contract["label"],
            contract["operators"].as_array().unwrap().len(),
            contract["states"].as_array().unwrap().len(),
            contract["model_semantic_digest"]
        );
        return Ok(());
    }
    println!(
        "profile={} operators={} states={} checks={} digest={} output={}",
        contract["profile"].as_str().ok_or("missing profile")?,
        contract["operators"].as_object().unwrap().len(),
        contract["states"].as_object().unwrap().len(),
        contract["checks"].as_array().unwrap().len(),
        contract["model_semantic_digest"],
        output.display()
    );
    Ok(())
}
