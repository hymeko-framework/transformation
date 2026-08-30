# Transformation — unified-IR M2b pre-migration audit & oracles (Phase B)

**Written:** 2026-08-30 03:50 CEST · **Authorisation:** `APPROVED-TRANSFORMATION-TEST-EDIT: unified-ir-m2b-audit-oracles`
**Base HEAD:** `f67b723` (Phase A pin) · **core pin:** `15223bc` · **Scope:** tests + docs + one std-only bench example. **Zero production source migrated.**

## 1. Summary

Established the oracles, inventory, and decision record that make the later M2b
source migration (raw `ir.<arena>[id.0]` → accessor API) mechanical and
oracle-checked. No production `src` in any of the three crates changed; the 277
pre-existing tests stay green and are joined by 12 new PASS oracles + 2 `#[ignore]`
defect counterexamples + 2 generators. The single-canonical-IR law and the
serialization architecture (HyMeKo resident / CBOR binary / `ir_stream` transport
/ JSON view) are recorded and, where testable now, proven.

## 2. Files touched (all additive except the one-line test-harness registration)

| File | ± | role |
|---|---|---|
| `hymeko_emitter/tests/migration_oracles.rs` | +464 | writer-list proof, accessor equivalence, checked-composite, HyMeKo round-trip, frozen fingerprints, 2 defect counterexamples |
| `hymeko_query/tests/test_ir_migration_oracle.rs` | +141 | reader proof, wire-DTO freeze, determinism, `target_id==DeclId.0` |
| `hymeko_query/tests/mod.rs` | +1 | register the new module (`autotests=false`) |
| `hymeko_query/tests/data/wire/minimal_edge_snapshot.json` | +1 | frozen wire fixture (603 B) |
| `hymeko_emitter/examples/ir_seam_bench.rs` | +133 | std-only writer/reader benchmark probe |
| `docs/unified-ir-construction-seam.md` | +193 | seam decision record + test-layer traceability |
| `docs/cbor-serialization-audit.md` | +152 | read-only audit of core's CBOR + `ir_stream` path |
| `docs/transformation-ir-consumer-inventory.md` | +31 | 149-site raw-access inventory (regenerable) |
| `docs/plans/2026-08-30-transformation-m2b-audit-oracles/*` | +bundle | plan.tex/tikz/mmd (pdf waived — no LaTeX) |
| `reports/coverage/transformation-m2b-oracles.lcov` | +artifact | coverage export |
| `reports/2026-08-30-transformation-unified-ir-pre-migration.md` | +this | report |

`CORE.YAML`: none (transformation has none; the core it consumes is untouched). No
dependency added. No production `src` change. No schema-version field on the wire.

## 3. Findings

### 3.1 Consumer inventory (layer 8)
149 raw arena-index sites — **emitter 24 / query 111 / formats 14** — dominated by
`.decl_nodes[…]` name lookups (83). 33 accessor sites already present across 10
files: the workspace is **mid-migration**. Per-arena breakdown and accessor
replacements in `docs/transformation-ir-consumer-inventory.md`.

### 3.2 Writer-list proof — refuted and sharpened
"bridge.rs is the only file that writes IR" is **false**; the sharper truth:
- **`bridge::to_compiler_ir` is the only *production* writer** in the workspace.
- Three files write IR **only in `#[cfg(test)]`** (`entropy` @228, `regen` @382,
  `split` @426); in production they are readers — split *proposes*
  (`&Ir → SplitProposal`), regen emits *HyMeKo source text* (`&Ir → String`),
  entropy computes a *scalar*. Proven at runtime by
  `production_query_paths_do_not_mutate_the_ir` and
  `bridge_writes_and_the_four_emitters_only_read`; the only `&mut Ir` in non-test
  code is `bridge::push_decl`.

### 3.3 Duplication → construction seam
`push_decl`/`push_node`/`push_edge` (push `DeclNode` + parallel
`decl_to_node`/`decl_to_edge`/`decl_to_arc`/`decl_hash` slots + back-link) is
**copy-pasted across four sites** (bridge production + 3 test modules), each
re-encoding core-internal invariants. Decision: one core-owned construction API
(`IrBuilder`/transaction) that all four call. `docs/unified-ir-construction-seam.md`.

### 3.4 HyMeKo is the resident serialization (layer 1) — tested
`hymeko_source_is_the_resident_serialization`: compile → `Ir₁` → `emit_hymeko` →
re-parse in memory → `Ir₂`, structurally equal (node/edge/arc counts + per-edge
signed-incidence signature). **Passes.** Byte-exact round-trip is *not* claimed
(imports/comments are not retained by the arena); the lowered structure is.

### 3.5 CBOR / `ir_stream` audit (layer 2, deferred) — read-only
Core `CborPayload` is a **serde-derive struct shell**: no encode/decode fn, **no
CBOR dependency** in core, referenced nowhere. `ir_stream` (envelope/section/
assembler/version/limits, ~5k LOC) is real and safe-by-construction (bounded
decode, no transmute, `SnapshotVersion` authority). Field classification and the
`ir+index+interned_strings+hash+imports` duplication (2 of 6 fields are
recomputable cache; 1 duplicates the string table) plus decoder-safety and the
future HyMeKo-vs-CBOR benchmark suite are in `docs/cbor-serialization-audit.md`.
**No codec implemented, no dependency added.**

### 3.6 Wire DTO frozen (JSON is a view, not canonical)
`SnapshotDto` is a lossy, serialize-only viewer projection (value→display string,
arcs inlined, first-level relations only, no `Dto→Ir`). `NodeDto::id` /
`ArcDto::target_id` are the raw `DeclId.0`. Frozen by `wire_snapshot_json_is_frozen`
+ `arc_dto_target_id_is_the_raw_declid` + a determinism test, so ID migration
cannot silently break the external viewer surface.

### 3.7 Editor-IR defects (layers 3, 5)
- `IRDelta::Batch` doc claims "succeeds-or-fails as one unit" but `apply()` is
  **non-atomic** — `batch_delta_should_be_atomic` (`#[ignore]`) reproduces it
  (post-fail vertex count 1, want 0). Un-ignore when the transaction API lands.
- Editor layout metadata (position/level/weight) has no arena home and is dropped
  on round-trip — `editor_layout_metadata_survives_roundtrip` (`#[ignore]`)
  reproduces it. Both verified to **fail** when run (a counterexample must).

## 4. Test results (per layer)

| suite | passed | ignored | note |
|---|---|---|---|
| `hymeko_emitter` migration_oracles | 8 | 3 | 2 defect counterexamples + 1 generator ignored |
| `hymeko_query` migration oracle (in `integration`) | 4 | 1 | wire-fixture generator ignored |
| **workspace total** | **289** | **8** | baseline 277 + 12 new; 0 failed |

`cargo clippy --workspace --all-targets -D warnings`: **clean** (incl. new tests).
Feature matrix (query default / `--no-default-features` / `interpret`): unchanged.
New test files are rustfmt-clean (formatted per-file; the pre-existing 239-hunk
workspace fmt drift is untouched and out of scope).

## 5. Coverage (layer 12)

`cargo-llvm-cov 0.9.0`, workspace: **80.74 % line** (region 80.99 %, fn 78.60 %) —
includes vendored `slotmap`. Oracle-targeted files: **`bridge.rs` 100.00 % line**
(sole production writer fully exercised), `emit_sysml` 99 %, `emit_hymeko` 90 %,
`editor_ir` 81 %, `snapshot.rs` 44 % (only the JSON path on a tiny fixture; DOT and
branches unhit — reported honestly, not inflated). Artifact:
`reports/coverage/transformation-m2b-oracles.lcov` (paths normalized repo-relative).

## 6. Fault injection (layer 12)

| fault | caught by | verified |
|---|---|---|
| FNV prime mutated (`…01b3`→`…01b4`) | `fnv_known_answer_vector_is_stable` | FAILED (left≠right), restored byte-identical |
| corrupted `NodeRec.decl` back-link | `node_for_decl_rejects_a_corrupted_backlink…` | PASSES (checked-composite → `None`) |
| non-atomic Batch (defect) | `batch_delta_should_be_atomic` | FAILS when un-ignored |
| dropped editor metadata (limitation) | `editor_layout_metadata_survives_roundtrip` | FAILS when un-ignored |

## 7. Performance (std-only probe, layer 11 partial)

`ir_seam_bench` (release, medians), writer = `to_compiler_ir`, readers = 4 emitters:

| scale | decls | inc | write | emit_hymeko | emit_sysml |
|---|---|---|---|---|---|
| 32×4 | 192 | 128 | 37.9 µs | 19.8 µs | 26.0 µs |
| 128×5 | 896 | 640 | 232 µs | 57.9 µs | 88.1 µs |
| 512×6 | 4096 | 3072 | 1069 µs | 259 µs | 413 µs |
| 2048×8 | 20480 | 16384 | 6213 µs | 1316 µs | 2681 µs |

Roughly linear in declarations; the write path (arena construction) dominates.
Each emitter output carries a byte length + FNV fingerprint in the probe.
Allocation/RSS/backend-crossover axes deferred to core/M2d.

## 8. Own errors & retracted claims

- **Retracted mid-phase:** an earlier draft of the seam doc floated "a JSON or
  binary canonical wire format" as hypothetical. That was wrong on two counts —
  (a) losslessness is a schema property, not a syntax property (JSON isn't
  inherently canonical, HyMeKo isn't inherently lossy; HyMeKo text is in fact the
  faithful serialization), and (b) the binary path already exists as core's CBOR
  + `ir_stream`. Corrected: the doc now names HyMeKo as resident, CBOR as the
  (unwired) lossless binary codec, `ir_stream` as transport, JSON as view-only,
  and forbids inventing a parallel envelope.
- The two `#[ignore]` tests are **counterexamples**, not PASS tests: verified to
  fail when run, so they mark real gaps and are never counted toward the pass total.

## 9. Deferred (not this phase) — mapped to the 12 mandatory layers

CBOR codec + HyMeKo↔CBOR round-trip (2), stable/stale IDs (3), canonical dynamic
mutation + rename/incidence (4), Batch atomicity *fix* (5), incremental view vs
rebuild (6), replay/resync (7), the actual accessor source migration (8),
large source-generated loads + hypergraph families (9,10), alloc/RSS/crossover
benchmarks (11). Owners: M2c/M2d and core (Phase E). Traceability matrix in
`docs/unified-ir-construction-seam.md` §4.1.

## HARD STOP (Phase B)

No production source migration, no `IrBuilder`, no transaction API, no CBOR codec,
no dependency, no stable/generational ID, no schema-version field. Those are the
subsequent phases. Next: Phase C — Hotaru core pin to `15223bc`.
