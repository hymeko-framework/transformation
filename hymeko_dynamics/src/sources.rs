//! Compiling a declared system from sources, on disk or in memory.
//!
//! The provider itself lives in `hymeko_sources`, because the optimisation projection needs the same
//! one and two copies of a `SourceProvider` is exactly the duplication this codebase spends its time
//! avoiding. What is here is the step from sources to a [`System`].

use std::path::Path;

use hymeko::ir::ir::Ir;
use hymeko::module_store::module_store::ModuleStore;
use hymeko::resolution::interner::Interner;
use hymeko::util::real_parser::RealParser;
pub use hymeko_sources::MemoryProvider;

use crate::read::{ReadError, project};
use crate::system::System;

/// Why a set of sources could not become a system.
#[derive(Debug, Clone, PartialEq)]
pub enum SourceError {
    /// The model did not compile. Carries what the compiler said.
    Compile(String),
    /// It compiled and did not project.
    Projection(ReadError),
}

/// Compile a model from a path and project it.
///
/// The file-system twin of [`project_sources`]. It compiles on every target because
/// `StdFsProvider` does; on one with no file system it will simply never find anything, which is
/// the honest failure rather than a missing symbol.
pub fn project_file(path: &Path) -> Result<(System, Ir, Interner), SourceError> {
    use hymeko::module_store::source_provider::StdFsProvider;
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = store
        .compile(path)
        .map_err(|error| SourceError::Compile(format!("{}: {error:?}", path.display())))?;
    let ir = program.ir.clone();
    let system = project(&ir, &store.it).map_err(SourceError::Projection)?;
    Ok((system, ir, store.it))
}

/// Compile a model from sources held in memory and project it.
///
/// `entry` is the path of the model itself; `files` is every source it and its imports need, keyed by
/// the path they are imported as.
pub fn project_sources(
    entry: &str,
    files: &[(&str, &str)],
) -> Result<(System, Ir, Interner), SourceError> {
    let mut provider = MemoryProvider::new();
    for (path, text) in files {
        provider.insert(path, text);
    }
    let mut store = ModuleStore::new(provider, RealParser);
    let program = store
        .compile(Path::new(entry))
        .map_err(|error| SourceError::Compile(format!("{error:?}")))?;
    let ir = program.ir.clone();
    let system = project(&ir, &store.it).map_err(SourceError::Projection)?;
    Ok((system, ir, store.it))
}
