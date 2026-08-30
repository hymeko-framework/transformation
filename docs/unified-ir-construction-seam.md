# Unified-IR construction seam — transformation consumer decision doc

**Written:** 2026-08-30 · **core pin:** `15223bc` · **base HEAD:** `f67b723`
**Authorisation:** `APPROVED-TRANSFORMATION-TEST-EDIT: unified-ir-m2b-audit-oracles`
**Status:** decision record for the M2b/M2d unified-IR work. No production source
is changed by this phase; this doc fixes *where* the later seam goes and *what
must not be duplicated*.

## 0. The law this doc serves

`hymeko_core::ir::Ir` (the arena IR) is the **single canonical model and source
of truth**. Consumer crates may read it via accessors, hold derived rebuildable
views, and (in future) propose modifications via a core transaction API. They may
**not** invent a second IR, a second ID system, a second mutation vocabulary, or a
second serialization envelope that core will own.

## 1. Resident serialization: HyMeKo text, not JSON

The framework is HyMeKo; the parser reads HyMeKo; `emit_hymeko` writes HyMeKo.
**That pair is the resident, faithful serialization of the canonical `Ir`.**

- `parser::parse_description` : HyMeKo text → `Ir`
- `hymeko_emitter::emit_hymeko` : `Ir` → HyMeKo text

This round-trip is a **tested fact**, not an assertion:
`migration_oracles::hymeko_source_is_the_resident_serialization` compiles a
self-contained fixture to `Ir₁`, emits HyMeKo, re-parses entirely in memory to
`Ir₂`, and asserts structural equivalence (node/edge/arc counts and the per-edge
signed-incidence signature). It passes. (Byte-exact round-trip is *not* claimed —
`emit_hymeko` drops imports/comments that the arena does not retain; the preserved
invariant is the lowered structure.)

**Losslessness is a property of the schema, not the syntax.** JSON is not
inherently canonical and HyMeKo text is not inherently lossy; the opposite holds
here. The layering is:

| role | artifact | owner | direction | fidelity |
|---|---|---|---|---|
| source of truth | `hymeko_core::ir::Ir` (arena) | core | — | canonical |
| **resident serialization** (human) | **HyMeKo source text** | core `ir_writer` / `parser` / emitter `emit_hymeko` | `parse` / write | structure-preserving (tested) |
| **binary codec** (compact lossless) | **CBOR — `hymeko_core::writers::cbor_writer::CborPayload`** | core | *intended* `Ir` ⇄ bytes | lossless by design — **codec not yet wired** (§1.2) |
| **transport framing** | **`hymeko_core::ir_stream`** (envelope/section/assembler/version) | core | snapshot + delta over the wire | bounded, validated, versioned (§1.2) |
| view | JSON `SnapshotDto` | consumer `hymeko_formats` | `Ir` → JSON only | **lossy** (§1.1) |
| view | DOT / URDF / SDF / MJCF | consumer `hymeko_formats` | `Ir` → text only | lossy, purpose-specific |

Consequence for the unified-IR work: **do not design a canonical JSON envelope,
and do not invent a new binary format.** HyMeKo text is the human-readable
canonical serialization; the compact lossless binary path already exists as
core's **CBOR** payload plus the **`ir_stream`** transport framing. Both are
core-owned. The correct action is to *audit and finish wiring the existing CBOR
path* (§1.2, and `docs/cbor-serialization-audit.md`), not to add a parallel
envelope. No encoding is a second *IR* — each is one more encoding of the one
`Ir`, and none may hold independently mutable state, its own ID system, its own
delta vocabulary, its own schema authority, or its own version counter
(`ir_stream::SnapshotVersion` is that single authority).

### 1.1 Why the JSON snapshot is a view, not a serialization

`hymeko_formats::snapshot::SnapshotDto` is a deliberately reduced projection for
external viewers (the Python wheel and the wasm browser demo). It cannot rebuild
the `Ir`:

- `value` is rendered to a *display string* (`render_value`), discarding the
  structured `ValueR` (nested lists, typed numerics, refs);
- `HyperArc` decls are not first-class — arcs are inlined into their edge, and
  per-ref `weights` (`RefAtomR::weights`) are dropped;
- only first-level `scope`/`isa`/`ref` relationships are kept; full
  parent/child/sibling links, `meta`, `doc_hash`, `decl_hash` are not;
- it is **serialize-only** — the DTOs derive `Deserialize`, but there is **no
  `Dto → Ir` reconstruction** anywhere in the tree.

The `snapshot` surface exposes the raw arena index as the wire ID
(`NodeDto::id`, `ArcDto::target_id` are `DeclId.0`). That is frozen — not blessed
as canonical — by `test_ir_migration_oracle::{wire_snapshot_json_is_frozen,
arc_dto_target_id_is_the_raw_declid}`, so a stable/generational-ID migration
cannot silently break the external viewer surface. The DTO must stay a view; it
must not accrete `Deserialize → Ir` and become a second source of truth.

### 1.2 The CBOR path exists but is not wired; `ir_stream` is real

Audited read-only against the `15223bc` core checkout (full audit in
`docs/cbor-serialization-audit.md`):

- **`CborPayload` is a serde-derive struct shell.** No `encode`/`decode`
  function exists, **no CBOR dependency** is declared in core (`[dependencies]`
  is `parser` + `serde` only), and the struct is referenced nowhere in
  production. The compact binary codec is *declared, not implemented*.
- Its six fields mix layers: `ir` (**canonical**), `interned_strings`
  (**canonical companion** — duplicates the string table), `index` and
  `canon_hash` (**derived/cache**, recomputable from `ir`), `root_path` and
  `imports` (**transport**). Embedding the two derived fields is the duplication
  to watch: a snapshot whose `index`/`canon_hash` disagrees with its `ir` is
  malformed, so a decoder must **recompute-and-verify** them, not trust them.
- **`ir_stream` is substantially implemented and safe-by-construction**:
  `ValidatedEnvelope::decode`, `DecodeLimits`/`AssemblerLimits` (bounded before
  allocation), typed `SectionKind`/`ValidatedSection` reached only through
  `try_from_wire` (no `transmute`, no struct cast), `SnapshotVersion` (the single
  version authority), and a `TransactionAssembler` with replay protection. This
  is the transport framing; it already enforces bounded-validate-before-use.

Decision: the binary path is CBOR + `ir_stream`, both core-owned. This phase
audits it and records the future requirements (bounded decode, semantic
round-trip equal to parsing the equivalent HyMeKo source, and the HyMeKo-vs-CBOR
benchmark suite) in `docs/cbor-serialization-audit.md`. No codec is implemented
and no dependency is added here.

## 2. The writer-list finding (refutes "bridge is the only writer")

Grep + the `#[cfg(test)]` line boundaries + runtime oracles establish:

- **`hymeko_emitter::bridge::to_compiler_ir` is the ONLY production writer** of an
  `Ir` in the whole transformation workspace. It is the editor→arena bridge.
- Three more files construct/mutate an `Ir`, but **only inside their
  `#[cfg(test)]` modules**: `hymeko_query::entropy` (boundary line 228),
  `rewrite::regen` (382), `rewrite::split` (426). In *production* these are
  **readers**: `propose_split` returns a `SplitProposal` from `&Ir`;
  `emit_split_rewrite` emits HyMeKo *source text* from `&Ir`; `compute_entropy*`
  returns a scalar from `&Ir`. Proven at runtime by
  `test_ir_migration_oracle::production_query_paths_do_not_mutate_the_ir` and, for
  the emitters, `migration_oracles::bridge_writes_and_the_four_emitters_only_read`.

So the precise statement is: **one production writer (bridge); every other
production path takes `&Ir` and is read-only by construction** (the compiler
enforces it). The only `&mut Ir` in non-test code is `bridge::push_decl`.

## 3. The duplication the seam removes

`push_decl` / `push_node` / `push_edge` — the canonical-IR construction primitive
(push a `DeclNode`, push the parallel `decl_to_node`/`decl_to_edge`/`decl_to_arc`/
`decl_hash` slots, patch the back-link, maintain the arc back-reference) — is
**copy-pasted across four locations**: `bridge.rs` (production) and the three
`#[cfg(test)]` modules. Every copy re-encodes core-internal invariant knowledge
(the parallel-arena push discipline and the `rec.decl == id` back-link that
`node_for_decl`/`edge_for_decl` later verify).

This is the argument for the seam: **a single core-owned construction API**
(`IrBuilder` / the future transaction API) that all four call, so no consumer
re-implements arena bookkeeping. `bridge::to_compiler_ir` would then call
`ir.add_node(name)` / `ir.add_edge(name, refs)` and never touch `decl_to_node` or
`decl_hash`. The test writers collapse onto the same API.

## 4. Seam placement (decision)

1. **Construction/mutation lives in core**, behind one invariant-preserving API.
   Consumers never push arena slots directly. `bridge.rs` migrates to it; the
   test writers migrate to it; no consumer-side builder is introduced.
2. **Reads migrate to accessors** (`decls`, `kind_of`, `parent_of`, `decl_node`,
   `as_node`/`node`, `as_edge`/`edge`, `arc`, `node_for_decl`/`edge_for_decl`).
   149 raw `ir.<arena>[id.0]` sites are inventoried in
   `docs/transformation-ir-consumer-inventory.md`. The checked-composite
   accessors additionally *catch* a corrupted back-link that the raw projection
   would return as a silently-wrong record
   (`migration_oracles::node_for_decl_rejects_a_corrupted_backlink_where_raw_would_mislead`).
3. **Editor IR deltas become transactional.** `IRDelta::Batch` today is
   non-atomic (its own doc claims "succeeds-or-fails as one unit" but `apply()`
   partial-applies; counterexample
   `migration_oracles::batch_delta_should_be_atomic`, `#[ignore]` until fixed).
   The core transaction API is where Batch atomicity is delivered — not a
   consumer-local snapshot hack.
4. **Editor layout metadata** (position, level, weight, patch_id) has no arena
   home at `15223bc` and is dropped on round-trip
   (`migration_oracles::editor_layout_metadata_survives_roundtrip`, `#[ignore]`).
   When the canonical IR grows attribute storage, these thread through the same
   construction seam — not a side channel.

## 4.1 Mandatory test-layer traceability (binding architectural invariant)

The binding invariant defines twelve mandatory test layers. Phase B (audit +
oracles, no production migration) covers or defers each as follows; deferred
layers land in M2c/M2d or core, never silently dropped.

| # | layer | Phase B | evidence / owner |
|---|---|---|---|
| 1 | parser → IR → emitter semantic round-trip | **covered** | `hymeko_source_is_the_resident_serialization`, emitter fingerprint goldens |
| 2 | HyMeKo ↔ IR ↔ CBOR losslessness | HyMeKo↔IR **covered**; CBOR **deferred** | round-trip test; CBOR has no codec (`docs/cbor-serialization-audit.md`) |
| 3 | stable / stale / dangling IDs | dangling **covered**; stable/stale **deferred** | `node_for_decl_rejects_a_corrupted_backlink…`; stable IDs are M2c |
| 4 | dynamic add/remove/rename/incidence | editor-IR **partial**; canonical **deferred** | existing `test_editor_ir`; core transaction API is M2d |
| 5 | batch atomicity & rollback | gap **documented** | `batch_delta_should_be_atomic` (`#[ignore]` counterexample, reproduces) |
| 6 | incremental view vs full rebuild | **deferred** | M2c/M2d view layer |
| 7 | replay/dup/reorder/version-conflict/resync | **deferred to core** | `ir_stream::assembler` exists; core (Phase E) |
| 8 | consumers use only the public core API | **partial** | 149 raw sites inventoried; accessor equivalence proven; migration is M2b |
| 9 | small goldens + large generated loads | small **covered**; large **partial** | wire fixture + mini_arm; `ir_seam_bench` to 20 480 decls |
| 10 | hypergraph families × sizes | **deferred** | graph workload corpus; transformation reuses later |
| 11 | CPU/mem/alloc/scaling/crossover benchmark | CPU+scaling **partial**; rest **deferred** | `ir_seam_bench`; alloc/RSS/crossover are core+M2d |
| 12 | traceability + touched-line cov + mutation | **covered** | this matrix; `bridge.rs` 100 % line; FNV-mutant + corrupted-backlink + 2 defect counterexamples |

## 5. What this phase deliberately does NOT do

No production source migration; no `IrBuilder`; no transaction API; no stable/
generational ID; no schema-version field on the wire DTO; no editor-IR feature
growth. Those are M2c/M2d. This phase only pins the pin (Phase A), inventories the
consumers, proves the writer list, freezes the surfaces, and records this seam
decision so the later edit is mechanical and oracle-checked.
