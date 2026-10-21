//! A HyMeKo model as a PostgreSQL script: `cargo run -p hymeko_relational --example project -- MODEL.hymeko [SCHEMA]`
//! writes to standard output the statements that create SCHEMA's tables (default `hymeko`) and fill them, in one
//! transaction. Pipe it to `psql`; drop an earlier projection's schema first.

use hymeko::module_store::{module_store::ModuleStore, source_provider::StdFsProvider};
use hymeko::resolution::string_table::StringTable;
use hymeko::util::real_parser::RealParser;
use std::{error::Error, path::PathBuf};

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if args.is_empty() || args.len() > 2 {
        return Err("usage: cargo run -p hymeko_relational --example project -- MODEL.hymeko [SCHEMA]".into());
    }
    let source = PathBuf::from(&args[0]);
    let schema = args.get(1).map_or_else(|| "hymeko".to_owned(), |schema| schema.to_string_lossy().into_owned());
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = store.compile(&source).map_err(|error| format!("compile failed: {error:?}"))?;
    let strings = StringTable::from_interner(&store.it);
    let projection = hymeko_relational::project(&program.ir, &strings)?;
    print!("{}", hymeko_relational::postgres::script(&projection, &schema));
    Ok(())
}
