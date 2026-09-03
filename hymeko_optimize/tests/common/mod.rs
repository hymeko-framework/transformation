//! Compiling a model from the `core` data library.
//!
//! The models live in `hymeko/core`, which owns them; this crate consumes them through the public
//! `ModuleStore` API and copies nothing. A checkout without the sibling repository beside it gets
//! `None` and skips, rather than a failure that says more about the layout than about the code.

use std::path::PathBuf;

use hymeko::ir::ir::Ir;
use hymeko::module_store::module_store::ModuleStore;
use hymeko::module_store::source_provider::StdFsProvider;
use hymeko::resolution::interner::Interner;
use hymeko::util::real_parser::RealParser;

/// Where `core`'s optimisation models sit relative to this crate.
pub fn model_path(file: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../core/data/optimization")
        .join(file)
}

/// Compile a model, or `None` when the sibling repository is not in this checkout.
pub fn compile(file: &str) -> Option<(Ir, Interner)> {
    let path = model_path(file);
    if !path.exists() {
        return None;
    }
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = store.compile(&path).expect("the model compiles");
    Some((program.ir.clone(), store.it))
}

/// The divider model, or a skip.
#[macro_export]
macro_rules! divider {
    () => {
        match $crate::common::compile("resistor_divider.hymeko") {
            Some(pair) => pair,
            None => {
                eprintln!("skipped: core/data/optimization is not in this checkout");
                return;
            }
        }
    };
}
