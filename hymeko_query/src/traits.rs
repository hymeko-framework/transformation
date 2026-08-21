use hymeko::common::ids::SymId;
use hymeko::resolution::interner::Interner;
use hymeko::resolution::string_table::StringTable;

pub trait NameResolver {
    fn resolve(&self, id: SymId) -> &str;
}

impl NameResolver for Interner {
    #[inline]
    fn resolve(&self, id: SymId) -> &str {
        Interner::resolve(self, id)
    }
}

impl NameResolver for StringTable {
    #[inline]
    fn resolve(&self, id: SymId) -> &str {
        StringTable::resolve(self, id)
    }
}