use env_logger::Env;
use hymeko::common::ids::{DeclId, SymId};
use hymeko::ir::ir::{DeclKind, Ir, NodeRec, SignedRefR, ValueR};
use hymeko::module_store::module_store::{
    CompiledProgram, HymekoParser, ModuleLoadError, ModuleStore,
};
use hymeko::module_store::source_provider::StdFsProvider;
use hymeko::resolution::interner::Interner;
use log::{LevelFilter, info};
use parser::ast::AstStr;
use std::fmt::Write;
use std::io::{self, Write as IoWrite};
use std::path::Path;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, SystemTime};

/// Load, parse, resolve, and lower a HyMeKo module using the production ModuleStore pipeline.
/// Returns the lowered IR, the resolved index, and a reference to the shared interner.
///
/// This is meant for tests only.
pub struct LalrpopParser;

impl HymekoParser for LalrpopParser {
    fn parse<'a>(&self, src: &'a str) -> Result<AstStr<'a>, String> {
        parser::parse_description(src).map_err(|e| format!("{e:?}"))
    }
}

/// Test-only helper: build ModuleStore, compile root, and return both.
/// Returning the store lets tests access `store.it` without cloning.
pub fn load_and_lower(
    root_path: impl AsRef<Path>,
) -> Result<
    (
        ModuleStore<StdFsProvider, LalrpopParser>,
        Arc<CompiledProgram>,
    ),
    ModuleLoadError,
> {
    let fs = StdFsProvider::new();
    let parser = LalrpopParser;
    let mut store = ModuleStore::new(fs, parser);

    let compiled = store.compile(root_path.as_ref())?;
    Ok((store, compiled))
}

pub fn print_dense_matrix(m: &[Vec<f32>], title: &str) {
    init_test_logger();
    let lock = MATRIX_PRINT_LOCK
        .get_or_init(|| Mutex::new(()))
        .lock()
        .expect("matrix print lock poisoned");
    let cols = if m.is_empty() { 0 } else { m[0].len() };
    let mut buf = String::new();
    let _ = writeln!(buf, "{title} ({}x{}):", m.len(), cols);
    for row in m {
        for &x in row {
            if (x - x.round()).abs() < 1e-6 {
                let _ = write!(buf, "{:>3} ", x.round() as i32);
            } else {
                let _ = write!(buf, "{:>6.2} ", x);
            }
        }
        buf.push('\n');
    }
    info!("--- BEGIN TENSOR: {title} ---");
    let prev_level = log::max_level();
    log::set_max_level(LevelFilter::Warn);
    print!("{}", buf);
    let _ = io::stdout().flush();
    log::set_max_level(prev_level);
    info!("--- END TENSOR: {title} ---");
    drop(lock);
}

pub fn find_decl(ir: &Ir, it: &Interner, name: &str, kind: DeclKind) -> DeclId {
    for (i, node) in ir.decl_nodes.iter().enumerate() {
        if node.kind == kind && it.resolve(node.name) == name {
            return DeclId::new(i);
        }
    }
    panic!("decl not found: {}", name);
}

pub fn get_node<'a>(ir: &'a Ir, did: DeclId) -> &'a NodeRec {
    let nid = ir.as_node(did).expect("decl should be a node");
    &ir.nodes[nid.0]
}

pub fn has_tag(it: &Interner, tags: &[SymId], name: &str) -> bool {
    let tid = it.get_id(name).expect("tag should be interned");
    tags.contains(&tid)
}

pub fn weight0(r: &SignedRefR) -> f64 {
    let atom = match r {
        SignedRefR::Plus(a) | SignedRefR::Minus(a) | SignedRefR::Neutral(a) => a,
    };

    // 1) Ha weights: Option<Vec<ValueR>>
    #[allow(unreachable_patterns)]
    match atom.weights.as_ref() {
        Some(ws) => {
            // ValueR-s változat
            #[allow(unreachable_patterns)]
            match &ws[0] {
                ValueR::Num(n) => *n,
                other => panic!("expected numeric weight, got {:?}", other),
            }
        }
        None => panic!("expected weight(s), got None"),
    }
}

static TEST_LOGGER: OnceLock<()> = OnceLock::new();
static MATRIX_PRINT_LOCK: OnceLock<Mutex<()>> = OnceLock::new();

fn init_test_logger() {
    TEST_LOGGER.get_or_init(|| {
        let _ = env_logger::Builder::from_env(Env::default().default_filter_or("info"))
            .is_test(true)
            .try_init();
    });
}

pub fn log_test_header(title: &str, details: &str) {
    init_test_logger();
    info!("============================================================");
    info!("TEST START: {title}");
    if !details.is_empty() {
        info!("DETAILS: {details}");
    }
    info!("START TIME: {:?}", SystemTime::now());
    info!("============================================================");
}

pub fn log_test_footer(title: &str, duration: Option<Duration>, summary: &str) {
    init_test_logger();
    info!("------------------------------------------------------------");
    if let Some(d) = duration {
        info!("DURATION: {:.3} seconds", d.as_secs_f64());
    }
    if !summary.is_empty() {
        info!("SUMMARY: {summary}");
    }
    info!("END TEST: {title}");
    info!("============================================================");
}
