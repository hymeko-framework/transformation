fn main() {
    let p = std::path::PathBuf::from(std::env::args().nth(1).expect("path"));
    let mut s = hymeko::module_store::module_store::ModuleStore::new(
        hymeko::module_store::source_provider::StdFsProvider::new(), hymeko::util::real_parser::RealParser);
    match s.compile(&p) {
        Ok(pr) => {
            let ir = &pr.ir;
            println!("OK  decls={} nodes={} edges={} arcs={}",
                ir.decl_nodes.len(), ir.nodes.len(), ir.edges.len(), ir.arcs.len());
        }
        Err(e) => { println!("FAIL {e:?}"); std::process::exit(1); }
    }
}
