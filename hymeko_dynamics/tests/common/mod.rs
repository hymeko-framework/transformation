//! Compiling declared systems, including one-number variants of them.

use std::path::PathBuf;

use hymeko::ir::ir::Ir;
use hymeko::module_store::module_store::ModuleStore;
use hymeko::module_store::source_provider::StdFsProvider;
use hymeko::resolution::interner::Interner;
use hymeko::util::real_parser::RealParser;
use hymeko_dynamics::System;

pub fn model_path(file: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../core/data/dynamics")
        .join(file)
}

fn compile_at(path: &PathBuf) -> (Ir, Interner) {
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = store.compile(path).expect("the model compiles");
    (program.ir.clone(), store.it)
}

/// The shipped Maxwell-RLC system, or `None` when `core` is not in this checkout.
pub fn maxwell() -> Option<System> {
    let path = model_path("maxwell_rlc.hymeko");
    if !path.exists() {
        return None;
    }
    let (ir, it) = compile_at(&path);
    Some(hymeko_dynamics::project(&ir, &it).expect("the model projects"))
}

/// The same system with textual substitutions applied to its declaration.
///
/// Every sweep in these tests goes through the MODEL rather than through a mutated projection: a
/// test that reached into the assembled system to change a coupling would be checking the analysis
/// against itself, and would keep passing if the reader stopped reading.
pub fn variant(edits: &[(&str, &str)]) -> Option<System> {
    variant_of("maxwell_rlc.hymeko", edits)
}

/// The same, for any model in `core/data/dynamics`.
#[allow(dead_code)]
pub fn variant_of(file: &str, edits: &[(&str, &str)]) -> Option<System> {
    Some(
        try_variant_of(file, edits)?
            .unwrap_or_else(|e| panic!("the variant of {file} projects: {e:?}")),
    )
}

/// The same again, handing back whatever the projection said — a refusal included.
///
/// A projection that rejects a model is doing its job, and a test that wants to see it reject one
/// needs the error rather than a panic.
#[allow(dead_code)]
pub fn try_variant_of(
    file: &str,
    edits: &[(&str, &str)],
) -> Option<Result<System, hymeko_dynamics::SourceError>> {
    let source = model_path(file);
    if !source.exists() {
        return None;
    }
    // Fixture mutations use LF literals; Git may check out the same source with CRLF.
    let mut text = std::fs::read_to_string(&source)
        .expect("readable")
        .replace("\r\n", "\n");
    for (from, to) in edits {
        assert!(text.contains(from), "the model no longer contains {from:?}");
        text = text.replace(from, to);
    }

    // A per-call serial rather than a content hash. Two tests asking for the SAME variant is the
    // normal case here — several of them want the lossless system — and a content hash would then
    // hand both the same directory to write into at the same time. Which it did.
    static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let hash = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    // The whole neighbourhood is reproduced INSIDE one hashed directory, `dynamics/` beside
    // `optimization/`, because `meta_dynamics` reaches the optimisation vocabulary by a
    // parent-relative path. Putting the sibling in the shared temp root instead would have every
    // variant in a parallel test run writing the same file, which is a race and was one.
    let root = std::env::temp_dir().join(format!(
        "hymeko-dynamics-variant-{}-{hash:04}",
        std::process::id()
    ));
    let directory = root.join("dynamics");
    let optimization = root.join("optimization");
    std::fs::create_dir_all(&directory).expect("a temporary directory");
    std::fs::create_dir_all(&optimization).expect("a sibling directory");
    std::fs::copy(
        model_path("meta_dynamics.hymeko"),
        directory.join("meta_dynamics.hymeko"),
    )
    .expect("the dynamics meta copies");
    std::fs::copy(
        model_path("../optimization/meta_optimization.hymeko"),
        optimization.join("meta_optimization.hymeko"),
    )
    .expect("the optimisation meta copies");

    let path = directory.join(file);
    std::fs::write(&path, text).expect("the variant writes");
    Some(hymeko_dynamics::project_file(&path).map(|(system, _, _)| system))
}

/// The declared parameters, as the analytic oracles need them.
pub struct Parameters {
    pub c: f64,
    pub l: f64,
    pub c_field: f64,
    pub l_field: f64,
    pub k: f64,
    pub g: f64,
}

impl Parameters {
    /// The simulator's defaults, which are also the model's.
    pub const DECLARED: Self = Self {
        c: 47.0e-9,
        l: 8.0e-3,
        c_field: 33.0e-9,
        l_field: 5.0e-3,
        k: 0.36,
        g: 0.72,
    };

    /// The two lossless normal-mode angular frequencies, in closed form.
    ///
    /// Eliminating the flux coordinates from the lossless chain gives a pair of coupled oscillators
    ///
    /// ```text
    /// u'' = -wc^2 u + (k/(L Cf)) v
    /// v'' = (k/(L C)) u - (kappa + wf^2) v,     kappa = k^2/(L Cf)
    /// ```
    ///
    /// whose characteristic equation is the biquadratic
    ///
    /// ```text
    /// s^4 + (wc^2 + wf^2 + kappa) s^2 + wc^2 wf^2 = 0
    /// ```
    ///
    /// The coupling enters the SUM of the squared frequencies and not their PRODUCT. That is exactly
    /// the signature of an avoided crossing, and it is an invariant worth testing on its own.
    pub fn normal_modes(&self) -> (f64, f64) {
        let wc2 = 1.0 / (self.l * self.c);
        let wf2 = self.g * self.g / (self.l_field * self.c_field);
        let kappa = self.k * self.k / (self.l * self.c_field);
        let sum = wc2 + wf2 + kappa;
        let product = wc2 * wf2;
        let discriminant = (sum * sum - 4.0 * product).max(0.0).sqrt();
        (
            (0.5 * (sum - discriminant)).sqrt(),
            (0.5 * (sum + discriminant)).sqrt(),
        )
    }

    /// The uncoupled circuit and field frequencies.
    pub fn uncoupled(&self) -> (f64, f64) {
        (
            1.0 / (self.l * self.c).sqrt(),
            self.g / (self.l_field * self.c_field).sqrt(),
        )
    }
}
