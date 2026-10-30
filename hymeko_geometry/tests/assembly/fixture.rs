use hymeko::{
    common::ids::DeclId,
    ir::canonical_hash::{Canon, build_path_index},
    module_store::{execution::CompiledModel, source_provider::MemProvider},
    resolution::interner::Interner,
    util::real_parser::RealParser,
};
use hymeko_geometry::kinematics::assembly::{AssemblyError, KinematicAssembly};
use hymeko_query::QueryEngine;
use std::path::Path;

pub const SOURCE: &str = include_str!("../fixtures/assembly/room.hymeko");

pub struct Fixture {
    pub model: CompiledModel,
}
impl Fixture {
    pub fn new(source: &str) -> Self {
        let files = MemProvider::default()
            .with_file("room.hymeko", source)
            .with_file(
                "meta_kinematics.hymeko",
                include_str!("../../../data/robotics/meta_kinematics.hymeko"),
            );
        Self {
            model: CompiledModel::compile(files, RealParser, Path::new("room.hymeko")).unwrap(),
        }
    }
    pub fn id(&self, path: &str) -> DeclId {
        let parts = path
            .split('.')
            .map(|part| self.model.interner().get_id(part).unwrap())
            .collect::<Vec<_>>();
        build_path_index(&self.model.program().ir)
            .unwrap()
            .decl_of(&parts)
            .unwrap()
    }
    pub fn read(&self, path: &str) -> Result<KinematicAssembly, AssemblyError> {
        KinematicAssembly::read(&self.query(), self.id(path))
    }
    pub fn query(&self) -> QueryEngine<'_, Interner> {
        QueryEngine::new(&self.model.program().ir, self.model.interner())
    }
    pub fn hash(&self) -> String {
        format!(
            "{:?}",
            Canon::new(&self.model.program().ir, self.model.interner()).hash()
        )
    }
}
