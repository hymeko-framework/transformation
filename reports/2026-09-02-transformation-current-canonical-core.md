# TRANSFORMATION-M2B — migration to canonical core `5fe2c88`

**STRATEGY-ID:** HYMEKO-PLATFORM-2026-09-01-v1 · **ROADMAP-TASK:** TRANSFORMATION-M2B ·
**APPROVED-TRANSFORMATION-EDIT:** migrate-to-canonical-core-5fe2c88 ·
**APPROVED-DEPENDENCY-EDIT:** transformation-current-core-pin. 2026-09-02. Branch
`feat/transformation-current-canonical-core` off `main` `1db1d98`; **not merged**.

Re-pins the transformation workspace (`hymeko_query`, `hymeko_formats`, `hymeko_emitter`)
from `hymeko_core@15223bcd` to the canonical `hymeko_core@5fe2c88`, unblocking the
downstream standalone Python binding. No `hymeko_core` edit; no second IR/DTO/bridge; no
transform semantics change.

## Why it was blocked, and why it is feasible
core@5fe2c88 replaced the three public `decl_to_{node,edge,arc}: Vec<Option<Id>>` maps with
a **private** 8-byte dispatch slot (`decl_to_record`, "option D") and made `Id.0`
`pub(crate)`. The workspace wrote those maps directly and used `id.0`. core@5fe2c88 exposes
the exact replacement seam — public `Ir::push_decl(DeclNode)->DeclId` and public
`bind_{node,edge,arc}_decl(d,id)->Result<(),DispatchError>` — and `Id::raw()`. **Audit gate:
every IR-construction site builds a *fresh* `Ir` (no in-place topology mutation anywhere;
the only production `&mut Ir` is `bridge::to_compiler_ir`, an editor→arena builder), so the
public seam suffices with no missing core mutation API. No HARD STOP.**

## Change set (mechanical)
- `Cargo.toml`/`Cargo.lock`: `hymeko_core` rev `15223bcd → 5fe2c88`. Lockfile diff = **only**
  that source line; no package added/removed, no other version moved, parser undrifted.
  `cargo tree`: **exactly one** `hymeko_core`.
- Construction (1 production + test helpers): the local `push_decl` helpers →
  `ir.push_decl(node)` (folds the three `decl_to_*.push(None)` into the private-slot push;
  keeps `decl_hash.push(None)` + parent-chaining); `ir.decl_to_X[did] = Some(id)` →
  `ir.bind_X_decl(did, id).expect("fresh decl: NONE slot, in-range record")` (documented-
  invariant `expect` per §6.4 — a fresh decl's slot is NONE and the record index fits the
  62-bit domain). Files: `hymeko_emitter/src/bridge.rs`, `hymeko_query/src/entropy.rs`,
  `rewrite/{regen,split}.rs`, `hymeko_emitter/tests/{test_bridge,migration_oracles}.rs`.
- IDs: every external `id.0` → `id.raw()` (`bridge.rs`, `emit_hymeko.rs`, `emit_sysml.rs`,
  `emit_lean4.rs`, `emit_rust_stubs.rs`, `migration_oracles.rs`); private-field test reads
  `ir.decl_to_node[did]` → `ir.as_node(did)` / `as_edge`.
- Accessors: `decl_name(ir, it, *target)` fixed (`inline_value` is `Option<&ValueR>`).

## Own mistake / retracted (on record)
A blanket `\.0\b → .raw()` sed over-reached onto **float string-literals** in `emit_sysml.rs`
(`"+1.0"`→`"+1.raw()"`, `weight = 1.0`→`1.raw()`), which the `emitter_output_fingerprints_
are_frozen` golden caught (emit_sysml FNV diff). Diagnosed by dumping and `diff`-ing the
SysML output at both revs (the diff was exactly the corrupted float tokens, **not** a
core-lowering change), then reverted with a precise `\b[0-9]+\.raw()\b → $1.0` (digit-
preceded only; `Id.raw()` is word-preceded). The frozen fingerprints then matched — proving
the migration is byte-identical, not the sed.

## V&V (all green)
- `fmt`: the migration's touched files are `rustfmt`-clean. The workspace has **pre-existing**
  fmt-dirty files (e.g. import ordering in `hymeko_query/src/engine.rs`) that the migration
  did **not** touch — left as-is to keep the diff minimal and auditable rather than churn ~25
  unrelated files; a workspace-wide `cargo fmt` is a separate cleanup. `clippy --workspace
  --all-features --all-targets -D warnings` clean; `check`/`build` ok.
- Tests: **294 passed / 0 failed** across 12 suites (289 frozen baseline @15223bcd + 5 new
  fault mutants); doctests 0-fail. The suite includes the emitter output fingerprints and
  arc-cardinality goldens → **query/transform/emit output byte-identical**.
- Fault mutants (`hymeko_emitter/tests/fault_mutants.rs`, all named + passing): sign loss
  flips output; duplicate incidence not deduped (2 refs kept); wrong-bind caught by the
  `*_for_decl` bijection; emitter reads the passed `Ir`; emit does not mutate the `Ir`
  (partial-commit impossible by `&Ir` type). Structural mutants documented, not runtime-
  injectable: cross-version `Ir` (prevented by single-`hymeko_core` `cargo tree`), old
  resolution-`Index` hash (no production call exists — grep-verified). In-memory fixtures →
  nothing on disk to restore.
- Benchmark A/B (`bench_workflow`, mini_arm, release): compile ~2.5 ms warm; emit sub-ms
  (urdf 1181 B, sdf 1695 B, mjcf 771 B, dot 302 B, mermaid 391 B) — **emit byte-sizes stable
  vs baseline** (output parity). The migration touched only construction sites, not these
  read-paths, so A/B is migration-neutral; absolute cost reflects core@5fe2c88. Peak RSS ≪
  16 GiB (KB-scale fixtures).

## Coverage
`cargo llvm-cov` not installed on host → **UNMEASURED** (no estimate, per contract).
Correctness rests on the frozen-baseline parity (294) + the fault-mutant net.

## Provenance / next
rustc 1.97.1, x86_64-pc-windows-msvc. Target pin = remote-verified core `main` `5fe2c88`.
Branch pushed + remote-SHA-verified (in the return report); **not merged**. Unblocks
PY1-INTEGRATION (standalone `hymeko/hymeko_python` re-pins to current core + this migrated
transformation). User dirty checkout of transformation (`feat/geometry-…`) untouched — all
work in the clone `/d/hnnwork/transformation`.
