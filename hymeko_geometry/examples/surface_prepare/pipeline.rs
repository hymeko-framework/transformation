use hymeko::{
    module_store::{execution::CompiledModel, source_provider::StdFsProvider},
    util::real_parser::RealParser,
};
use hymeko_geometry::{
    mesh::TriangleMesh,
    pipeline::{CompiledRecipe, GeometryRecipe, RecipeProject},
};
use std::{
    fs::{File, OpenOptions},
    io::{BufReader, BufWriter, Write},
    path::Path,
};

fn read(path: &str, limit: u64) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    use std::io::Read;
    let mut bytes = Vec::new();
    File::open(path)?.take(limit + 1).read_to_end(&mut bytes)?;
    if bytes.len() as u64 > limit {
        return Err("authoring input exceeds its byte budget".into());
    }
    Ok(bytes)
}
fn write(path: &str, value: &impl serde::Serialize) -> Result<(), Box<dyn std::error::Error>> {
    let file = OpenOptions::new().write(true).create_new(true).open(path)?;
    let mut writer = BufWriter::new(file);
    serde_json::to_writer(&mut writer, value)?;
    writer.flush()?;
    Ok(())
}
fn compile(path: &str, root: &str) -> Result<CompiledRecipe, Box<dyn std::error::Error>> {
    if Path::new(path).extension().is_some_and(|v| v == "json") {
        let project: RecipeProject = serde_json::from_slice(&read(path, 8 * 1024 * 1024)?)?;
        return Ok(project.compile(root)?);
    }
    if std::fs::metadata(path)?.len() > 4 * 1024 * 1024 {
        return Err("source exceeds 4 MiB".into());
    }
    let model = CompiledModel::compile(StdFsProvider::new(), RealParser, Path::new(path))
        .map_err(|e| format!("recipe compilation: {e:?}"))?;
    Ok(CompiledRecipe {
        recipe: GeometryRecipe::project(&model, root)?,
        semantic_digest: format!("{:?}", model.semantic_digest()),
    })
}

pub fn run(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    if !((args[0] == "recipe" && args.len() == 4) || (args[0] == "pipeline" && args.len() == 5)) {
        return Err("surface_prepare recipe SOURCE.hymeko|PROJECT.json ROOT OUTPUT.json | pipeline SOURCE ROOT MESH.json OUTPUT.json".into());
    }
    let compiled = compile(&args[1], &args[2])?;
    if args[0] == "recipe" {
        return write(&args[3], &compiled);
    }
    let input: serde_json::Value =
        serde_json::from_reader(BufReader::new(read(&args[3], 64 * 1024 * 1024)?.as_slice()))?;
    let mesh: TriangleMesh = serde_json::from_value(input.get("mesh").unwrap_or(&input).clone())?;
    let cancellation = std::env::var_os("HYMEKO_GEOMETRY_CANCEL_FILE");
    let result = compiled.recipe.run(&mesh, |stage| {
        println!(
            "{}",
            serde_json::json!({"status":"stage_complete","stage":stage})
        );
        if cancellation.as_ref().is_some_and(|p| Path::new(p).exists()) {
            return Err("authoring cancelled between stages".into());
        }
        Ok(())
    })?;
    write(
        &args[4],
        &serde_json::json!({"schema_version":1,"source_digest":compiled.semantic_digest,"recipe":compiled.recipe,"result":result}),
    )
}
