//! Compiling a model that is not on a disk.
//!
//! `hymeko_core`'s [`ModuleStore`] reads through a [`SourceProvider`], and the one it ships reads a
//! file system. A browser has no file system, so the model would have to be pre-baked into a trace
//! somewhere else and shipped as numbers — which would make the page a viewer of somebody's export
//! rather than a consumer of the model.
//!
//! [`MemoryProvider`] serves a map of path to text instead. The same compiler, the same IR, the same
//! projection; only where the bytes come from is different. That is what the trait is for, and it is
//! the reason a `.hymeko` file can be the input on a target with no `std::fs`.
//!
//! This would sit better in `hymeko_core` beside `StdFsProvider`, which owns the module store. It is
//! a crate of its own because it is needed by more than one consumer — the optimisation projection
//! and the dynamical one both compile models in a browser — and one copy in a shared crate is the
//! difference between a shared implementation and a duplicated one. Adding it to `core` is a change
//! to another owner, and that is a conversation rather than a commit.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use hymeko::module_store::source_provider::SourceProvider;

/// Sources held in memory, addressed by path.
///
/// Imports inside a model are relative, so the paths given here must be the ones the model asks for:
/// a file importing `../optimization/meta_optimization.hymeko` needs that key, spelled the way
/// [`Self::normalise`] spells it.
#[derive(Debug, Clone, Default)]
pub struct MemoryProvider {
    files: HashMap<String, Arc<str>>,
}

impl MemoryProvider {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add one source. Returns self so a whole model can be assembled in one expression.
    pub fn with(mut self, path: &str, text: &str) -> Self {
        self.files.insert(Self::normalise(Path::new(path)), text.into());
        self
    }

    pub fn insert(&mut self, path: &str, text: &str) {
        self.files.insert(Self::normalise(Path::new(path)), text.into());
    }

    /// Which paths this provider can serve, for a diagnostic that names them.
    pub fn paths(&self) -> Vec<&str> {
        let mut out: Vec<&str> = self.files.keys().map(String::as_str).collect();
        out.sort_unstable();
        out
    }

    /// A path in one spelling: forward slashes, and `a/../b` collapsed.
    ///
    /// The module store canonicalises before it reads, and a relative import resolves against the
    /// importing file's directory — so `dynamics/meta_dynamics.hymeko` importing
    /// `../optimization/meta_optimization.hymeko` arrives here as `dynamics/../optimization/...`,
    /// which is the same file as `optimization/...` and must hash to the same key.
    fn normalise(path: &Path) -> String {
        let mut parts: Vec<String> = Vec::new();
        for component in path.to_string_lossy().replace('\\', "/").split('/') {
            match component {
                "" | "." => {}
                ".." => {
                    parts.pop();
                }
                other => parts.push(other.to_string()),
            }
        }
        parts.join("/")
    }
}

impl SourceProvider for MemoryProvider {
    fn canonicalize(&self, p: &Path) -> Result<PathBuf, String> {
        Ok(PathBuf::from(Self::normalise(p)))
    }

    fn read(&mut self, p: &Path) -> Result<Arc<str>, String> {
        let key = Self::normalise(p);
        self.files.get(&key).cloned().ok_or_else(|| {
            format!(
                "{key}: not among the sources provided ({})",
                self.paths().join(", ")
            )
        })
    }

    /// One version, always: the sources cannot change under a run that was handed all of them.
    fn version(&self, _path: &Path) -> Result<u64, String> {
        Ok(1)
    }
}

