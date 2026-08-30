# CBOR / `ir_stream` serialization audit (read-only, pre-implementation)

**Written:** 2026-08-30 · **audited:** `hymeko_core` @ `15223bc` (checkout, read-only)
**Authorisation:** `APPROVED-TRANSFORMATION-TEST-EDIT: unified-ir-m2b-audit-oracles`
**Status:** audit only. No codec implemented, no dependency added, no core source
changed. This records what exists so the binary-serialization work is finished on
the *existing* path, never reinvented — and never as a second IR.

## 1. Architecture being served (binding)

- HyMeKo source text — resident human-readable serialization.
- `hymeko_core::ir::Ir` — sole canonical in-memory model.
- **CBOR** — compact lossless binary snapshot / cache / wire codec of that same
  model (`hymeko_core::writers::cbor_writer::CborPayload`).
- **`ir_stream`** — transport framing around snapshots and deltas.
- JSON `SnapshotDto` — lossy UI/view DTO only.

No layer may invent a second ID system, delta vocabulary, schema authority,
version counter, or independently mutable state. `ir_stream::SnapshotVersion` is
the single version authority; the `Ir` is the single ID space.

## 2. What actually exists (evidence)

### 2.1 `writers/cbor_writer.rs` — payload shell, no codec

```rust
#[derive(Serialize, Deserialize)]
pub struct CborPayload {
    pub root_path: PathBuf,
    pub ir: Ir,
    pub index: Index,
    pub interned_strings: Vec<String>,
    pub canon_hash: HashId,
    pub imports: Vec<(SymId, ModuleKey)>,
}
```

- **Production encode/decode functions: NONE.** No `to_cbor`/`from_cbor`/
  `encode`/`decode`; the struct only derives `serde::{Serialize, Deserialize}`,
  which is format-agnostic — nothing binds it to CBOR.
- **Direct CBOR dependency: NONE.** `hymeko_core/Cargo.toml` `[dependencies]` is
  `parser` + `serde` (derive) only — no `ciborium`, `minicbor`, or `serde_cbor`.
- **Usage: NONE.** `CborPayload` is referenced nowhere in production (only a
  comment in `resolution/string_table.rs`). It is a declared shape awaiting a
  codec.

Conclusion: the "CBOR path" is a **payload contract, not a working codec**.

### 2.2 `ir_stream/` — transport framing, substantially implemented and safe

| module | surface | role |
|---|---|---|
| `envelope.rs` | `EnvelopeDraft`, `ValidatedEnvelope::decode(bytes) -> Result<_, WireError>`, `MessageType` | wire envelope, decode-guarded |
| `codec.rs` | `validate_message`, `encode_message(draft, payload) -> Vec<u8>` | message validate/encode |
| `section.rs` | `SectionKind`/`ElementKind`/`ValidatedSection`, `try_from_wire`, `to_u32_vec`/`to_f32_vec` | typed sections, no transmute |
| `limits.rs` | `DecodeLimits` | **bound sizes before allocation** |
| `assembler.rs` (~2500 LOC) | `TransactionAssembler`, `AssemblerLimits`, `CommittedTransaction`, `TransactionKind`, replay window | snapshot+delta assembly, replay protection, memory bounds |
| `version.rs` | `SnapshotVersion` (explicit LE wire image, never a struct cast) | **the single version authority** |
| `error.rs` | `WireError` | typed wire errors |

The module docstring states the invariants already hold: *"the input buffer is
never cast to a Rust struct and never `transmute`d"*, and *"a downstream crate
cannot construct a `ValidatedEnvelope` without going through validation."* So the
"bounded and validated before allocation" requirement is a **property of the
existing design**, not something to add — the CBOR payload codec must plug into
it, not bypass it.

### 2.3 `writers/ir_writer.rs` — the core-side text writer

`IrWriter::new(ir, interner).write_all::<W: Write>()` — `Ir` + `Interner` → text.
This is core's own resident-serialization writer (the counterpart of the
consumer-side `emit_hymeko`). It confirms the resident format is text, in core.

## 3. Field classification (`CborPayload`)

| field | class | notes |
|---|---|---|
| `ir: Ir` | **canonical** | the whole model, embedded by value |
| `interned_strings: Vec<String>` | **canonical companion** | the string table the `Ir`'s `SymId`s resolve against; required for meaning, but a full copy of the interner |
| `index: Index` | **derived / cache** | resolution index, rebuildable from `ir` (+ `imports`); the struct comment even notes it must gain `Serialize` derives |
| `canon_hash: HashId` | **derived / cache** | canonical hash of `ir`, recomputable |
| `root_path: PathBuf` | **transport** | which module root this snapshot is of; also a host-path leak risk (see §5) |
| `imports: Vec<(SymId, ModuleKey)>` | **transport** | module-graph edges for multi-module resolution |

## 4. Duplication caused by `Ir + Index + interned_strings + hash + imports`

Two of six fields (`index`, `canon_hash`) are **recomputable from the canonical
`ir`**; a third (`interned_strings`) **duplicates the interner** that the runtime
already holds. So a CBOR snapshot as currently shaped carries the canonical model
*plus* two derived caches *plus* a full string-table copy.

- **Bytes.** `index` and `interned_strings` are the large redundant terms —
  `index` scales with declaration count, `interned_strings` with total identifier
  text. Embedding them trades snapshot size for decode-time speed.
- **Correctness risk (the real cost).** A snapshot whose `index` or `canon_hash`
  disagrees with its `ir` is **malformed**. Trusting an embedded derived field is
  a silent-wrong-answer hazard of the same class the checked-composite accessors
  guard against on the read side. Therefore:
  - the decoder must **recompute `canon_hash` from the decoded `ir` and reject on
    mismatch** (integrity + tamper check), and
  - **rebuild `index` from `ir` (+ `imports`) and verify**, or treat the embedded
    `index` as an untrusted hint only.
- **Decision to record (not implement):** the canonical snapshot is
  `(ir, interned_strings, imports, root_path)`; `index` and `canon_hash` are
  cache/integrity fields that the decoder regenerates and checks. Whether to ship
  the caches on the wire at all is a size-vs-decode-time question the §6 benchmark
  answers — it is not decided here.

## 5. Decoder safety requirements (for the future codec)

1. **Bounded before allocation.** Every length (arena vectors, `Vec<String>`,
   section counts) is checked against `DecodeLimits`/`AssemblerLimits` *before*
   any allocation — reuse `ir_stream`, do not allocate from attacker-controlled
   lengths.
2. **Validated, no transmute.** Bytes are parsed through the `ir_stream`
   validated path; the buffer is never cast to a Rust struct.
3. **Semantic equivalence to HyMeKo.** `cbor_decode(cbor_encode(ir))` must yield
   the **same canonical semantic state** as `parse(emit_hymeko(ir))` — the same
   equivalence already proven for the text path by
   `migration_oracles::hymeko_source_is_the_resident_serialization`. The CBOR
   round-trip test must assert it against the *same* fixtures.
4. **Integrity.** Recompute `canon_hash` from the decoded `ir` and reject on
   mismatch; rebuild/verify `index`.
5. **No second IR.** Decoding produces an `Ir` (+ its string table); it does not
   produce a parallel model, a parallel ID space, or independently mutable state.
6. **No host-path leak.** `root_path: PathBuf` must not serialize an absolute host
   path into a shareable snapshot/QR (privacy + reproducibility); store a
   workspace-relative path or a module key.

## 6. Future benchmark requirements — HyMeKo text vs CBOR (deferred)

To be implemented on the core side (Phase E / core work), **not here**. Same
semantic workload encoded both ways, at increasing scales (reuse the existing IR
workload/fixture corpus — do not copy a generator):

| axis | measure |
|---|---|
| encoded size | bytes, HyMeKo text vs CBOR, and CBOR with/without the derived caches |
| encode wall | median/IQR, `emit_hymeko`/`ir_writer` vs `cbor_encode` |
| decode wall | median/IQR, `parse` vs `cbor_decode` (incl. hash re-verify + index rebuild) |
| allocations | count + bytes during decode (canonical profiler) |
| peak RSS | decode of the largest scale, under the 16 GB cap |
| semantic round-trip | `cbor_decode(cbor_encode(x))` ≡ `parse(emit_hymeko(x))` at every scale |

Scales follow the canonical IR corpus (up to ~100k declarations). Report medians,
IQR and worst case; topologies never averaged into one number.

## 7. What this audit does NOT do

No codec, no `ciborium`/`minicbor` dependency, no core source change, no benchmark
run. Those are future core work. This document exists so that work extends the
existing CBOR + `ir_stream` path and honours the single-canonical-IR law.
