//! Unit-test fixtures shared by the query modules: a one-file model and a declaration by name.

use hymeko::common::ids::DeclId;
use hymeko::module_store::{execution::CompiledModel, source_provider::MemProvider};
use hymeko::util::real_parser::RealParser;

pub(crate) fn compile(source: &str) -> CompiledModel {
    let mut files = MemProvider::default();
    files.insert_file("/model.hymeko", source);
    CompiledModel::compile(files, RealParser, std::path::Path::new("/model.hymeko")).unwrap()
}

pub(crate) fn decl(model: &CompiledModel, name: &str) -> DeclId {
    let ir = &model.program().ir;
    ir.decls()
        .find(|(_, node)| model.interner().resolve(node.name) == name)
        .unwrap()
        .0
}
