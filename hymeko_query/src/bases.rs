//! Base relations: the declarations a declaration inherits from, and whether it is a kind.
//!
//! One walk for every reader (plan `docs/plans/2026-09-30-semantic-reads`). A declaration's bases
//! are the references in its node's or edge's `bases`, in authored order. A [`BasePolicy`] says
//! which of them count: all, or those whose annotation carries a tag such as `<isa>`. The walk
//! needs only the IR, so readers without a name table (validators) use it as well. It visits a
//! shared base once (a diamond) and reports a cycle on its own path instead of cutting the walk
//! off at a depth.

use crate::engine::QueryEngine;
use crate::traits::NameResolver;
use hymeko::common::ids::{DeclId, SymId};
use hymeko::ir::ir::{Ir, SignedRefR};
use std::collections::HashSet;

/// Which base relations count.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BasePolicy {
    /// Every base reference.
    All,
    /// Only base references whose annotation carries this tag: the interned `isa` for `<isa>`.
    /// A model whose interner has no such symbol has no tagged base.
    Tagged(SymId),
}

/// A base walk that cannot be answered.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BaseError {
    /// `decl` was reached again while it was still being walked.
    Cycle { decl: DeclId },
    /// `decl` names `base`, which is outside the declaration table.
    Dangling { decl: DeclId, base: DeclId },
}

fn base_refs(ir: &Ir, decl: DeclId) -> &[SignedRefR] {
    if let Some((_, node)) = ir.node_for_decl(decl) {
        &node.bases
    } else if let Some((_, edge)) = ir.edge_for_decl(decl) {
        &edge.bases
    } else {
        &[]
    }
}

/// The direct bases of `decl` that count under `policy`, in authored order. Unresolved bases
/// (the none declaration) are skipped.
pub fn bases(
    ir: &Ir,
    decl: DeclId,
    policy: BasePolicy,
) -> impl DoubleEndedIterator<Item = DeclId> + '_ {
    base_refs(ir, decl)
        .iter()
        .filter(move |base| match policy {
            BasePolicy::All => true,
            BasePolicy::Tagged(tag) => base.atom().anno.tags.contains(&tag),
        })
        .map(SignedRefR::target)
        .filter(|target| !target.is_none())
}

/// Whether `decl` is `kind`, or reaches it through bases that count under `policy`. Reflexive,
/// with no depth limit. Bases are searched depth-first in authored order, and the first cycle the
/// search meets is an error, even if another branch would reach `kind`.
///
/// # Errors
/// [`BaseError::Cycle`] for a cycle on the walk, [`BaseError::Dangling`] for a base outside the
/// declaration table.
pub fn is_a(ir: &Ir, decl: DeclId, kind: DeclId, policy: BasePolicy) -> Result<bool, BaseError> {
    if decl == kind {
        return Ok(true);
    }
    if ir.decl_node(decl).is_none() {
        return Ok(false);
    }
    // Validators ask this for every declaration and kind, so the common shallow case must not
    // allocate. The bounded walk below keeps its path on the call stack; a deep chain or a wide
    // diamond ladder falls back to the walk that remembers finished declarations.
    let mut visits = 0;
    match shallow(
        ir,
        &Frame { decl, parent: None },
        kind,
        policy,
        0,
        &mut visits,
    ) {
        Shallow::Answer(answer) => answer,
        Shallow::TooLarge => walk(ir, decl, kind, policy),
    }
}

/// One declaration on the shallow walk's path.
struct Frame<'f> {
    decl: DeclId,
    parent: Option<&'f Frame<'f>>,
}

enum Shallow {
    Answer(Result<bool, BaseError>),
    TooLarge,
}

const SHALLOW_DEPTH: u32 = 64;
const SHALLOW_VISITS: u32 = 512;

/// [`is_a`] without allocation, the same answers in the same order, bounded in depth and visits.
fn shallow(
    ir: &Ir,
    frame: &Frame<'_>,
    kind: DeclId,
    policy: BasePolicy,
    depth: u32,
    visits: &mut u32,
) -> Shallow {
    *visits += 1;
    if depth >= SHALLOW_DEPTH || *visits > SHALLOW_VISITS {
        return Shallow::TooLarge;
    }
    for base in bases(ir, frame.decl, policy) {
        if ir.decl_node(base).is_none() {
            return Shallow::Answer(Err(BaseError::Dangling {
                decl: frame.decl,
                base,
            }));
        }
        if base == kind {
            return Shallow::Answer(Ok(true));
        }
        let mut on_path = Some(frame);
        while let Some(step) = on_path {
            if step.decl == base {
                return Shallow::Answer(Err(BaseError::Cycle { decl: base }));
            }
            on_path = step.parent;
        }
        let next = Frame {
            decl: base,
            parent: Some(frame),
        };
        match shallow(ir, &next, kind, policy, depth + 1, visits) {
            Shallow::Answer(Ok(false)) => {}
            other => return other,
        }
    }
    Shallow::Answer(Ok(false))
}

/// [`is_a`] for any depth: an explicit path, and a set of finished declarations so a shared base
/// is walked once.
fn walk(ir: &Ir, decl: DeclId, kind: DeclId, policy: BasePolicy) -> Result<bool, BaseError> {
    let mut path = vec![(decl, 0usize)];
    let mut on_path = HashSet::from([decl]);
    let mut done = HashSet::new();
    while let Some(&(current, next)) = path.last() {
        let Some(base) = bases(ir, current, policy).nth(next) else {
            path.pop();
            on_path.remove(&current);
            done.insert(current);
            continue;
        };
        if let Some(top) = path.last_mut() {
            top.1 += 1;
        }
        if ir.decl_node(base).is_none() {
            return Err(BaseError::Dangling {
                decl: current,
                base,
            });
        }
        if base == kind {
            return Ok(true);
        }
        if on_path.contains(&base) {
            return Err(BaseError::Cycle { decl: base });
        }
        if !done.contains(&base) {
            on_path.insert(base);
            path.push((base, 0));
        }
    }
    Ok(false)
}

impl<'a, R: NameResolver> QueryEngine<'a, R> {
    /// [`bases`] over this engine's IR.
    pub fn bases(&self, decl: DeclId, policy: BasePolicy) -> impl Iterator<Item = DeclId> + 'a {
        bases(self.ir(), decl, policy)
    }

    /// [`is_a`] over this engine's IR.
    ///
    /// # Errors
    /// As [`is_a`].
    pub fn is_a(&self, decl: DeclId, kind: DeclId, policy: BasePolicy) -> Result<bool, BaseError> {
        is_a(self.ir(), decl, kind, policy)
    }
}

#[cfg(test)]
#[path = "bases/tests.rs"]
mod tests;
