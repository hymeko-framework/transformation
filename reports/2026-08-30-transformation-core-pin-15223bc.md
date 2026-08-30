# Transformation → hymeko_core pin to `15223bc` (accessor API) — Phase A

**Written:** 2026-08-30 03:17 CEDT · **Authorisation:** `APPROVED-DEPENDENCY-UPDATE: transformation-core-pin-to-15223bc`
**Base HEAD:** `21b5826720b2fa56eff40886c5d0ec51b2da5ddc` · **Scope:** dependency lockfile only — zero production source, zero `Cargo.toml`.

## 1. Summary

Advanced the `hymeko_core` git revision the `transformation` workspace resolves,
from `71c1f997783821155d9aa67e6e2f4aeb07bd80f3` to the accessor-API production
commit `15223bcd762806b69909ea73f7526586c2d2ded6`. The `parser` revision is held
at `e28a205147c48273980e63aa3338ca286974a6ed`. The change is **one line of
`Cargo.lock`**. `Cargo.toml`, every `src`, every `tests` file, and every fixture
are byte-identical to `21b5826`.

## 2. Files touched

| File | change |
|---|---|
| `Cargo.lock` | +1 −1 (the `hymeko_core` `source` revision) |
| `docs/plans/2026-08-30-transformation-core-pin-audit/{plan.tex,plan.tikz,plan.mmd}` | plan bundle (pdf waived — no LaTeX toolchain) |
| `reports/2026-08-30-transformation-core-pin-15223bc.md` | this report |

No `CORE.YAML` item touched (transformation has no `CORE.YAML`; the core repo it
consumes is untouched). No dependency added or removed. No `Cargo.toml` change.

## 3. Gate matrix — baseline (`71c1f997`) vs after (`15223bc`)

| Gate | baseline | after | verdict |
|---|---|---|---|
| `cargo check --workspace --all-targets` | ok | ok | = |
| workspace test (default) | **277 passed / 0 failed / 1 ignored** | **277 / 0 / 1** | byte-identical vector |
| — hymeko_emitter lib / test_bridge / test_editor_ir / test_emitters | 0 / 6 / 10 / 14 | 0 / 6 / 10 / 14 | = |
| — hymeko_formats lib / hymeko_query lib / integration | 12 / 19 / 216(+1 ign) | 12 / 19 / 216(+1 ign) | = |
| — doctests emitter / formats / query | 0 / 0 / 0(+3 ign) | 0 / 0 / 0(+3 ign) | = |
| `cargo clippy --workspace --all-targets -D warnings` | clean | clean | = |
| query `--no-default-features` (interpret OFF) | 19 + 216(+1) + 3 ign | same | = |
| query `--features interpret` | 19 + 216(+1) + 3 ign | same | = |

The per-binary test vector is the **functional golden oracle**: `test_emitters`
asserts the four M2T emitters (`emit_hymeko`, `emit_sysml`, `emit_rust_stubs`,
`emit_lean4`) against a real `mini_arm.hymeko` fixture; `test_bridge` round-trips
both bridge directions; `test_editor_ir` covers the editor IR / delta vocabulary;
the 216 integration tests cover query results+order, rewrite, snapshot JSON and
the formats emitters. All assertions pass identically at both pins, so every
emitter/query/formats/bridge output is byte-unchanged across the pin.

## 4. Independent corroboration — generated artifacts

The integration tests write emitter output into `generated/gazebo_launch/moveo/`
(4 files: two `moveo.urdf`, two `gz_sim.launch.py`). After both the baseline and
the post-pin runs, `git diff --ignore-all-space` on those files is **empty** and
`--numstat` shows **zero** line changes — the only delta is LF/CRLF from the test
writer. Content is byte-identical, independently confirming the pin changed no
emitter output. These test side-effects were reverted to HEAD, not committed.

## 5. Lockfile invariants

- `[[package]]` count: **141 → 141** (unchanged).
- git sources: exactly one `hymeko_core` (`15223bc`) + one `parser` (`e28a2051`).
- No second `hymeko_core` version; `cargo tree -d` duplicates (`either`,
  `hashbrown`, `lalrpop-util`, `log`) are the pre-existing transitive set,
  unaffected by a core-rev bump.
- Lockfile diff: **1 insertion, 1 deletion**, on the `hymeko_core` `source` line.

The targeted `cargo update -p hymeko_core --precise` also drifted `parser`
(`e28a2051` → `284c490d`); it was pinned back to `e28a2051` exactly, so the final
tracked diff is core-only.

## 6. Pre-existing baseline conditions (recorded, not introduced, not fixed)

- **`cargo fmt --check` is red at baseline** — 239 diff hunks across the workspace
  (style-edition import ordering, e.g. `{new_key_type, SlotMap}` → `{SlotMap,
  new_key_type}`, and method-chain wrapping). This is the committed state at
  `21b5826`, out of scope for a dependency-only pin (which touches zero `.rs`
  files) and therefore invariant. Not reformatted — that would be an unauthorised
  source edit under this authorisation.
- One integration test and three `hymeko_query` doctests are `#[ignore]` at
  baseline; they remain ignored (not counted as PASS).

## 7. Provenance

- Host: Windows 11, this workstation; `cargo`/`rustc` workspace toolchain.
- git dep fetch: `CARGO_NET_GIT_FETCH_WITH_CLI=true`; remote resolves via
  `amaterasu.local` for git; no SSH/git config, `known_hosts`, or deploy key
  modified.
- No seed/dataset involved (no experiment; a dependency pin).

## 8. Retained hypotheses / open items

- The consumers compile clean against `15223bc` **without source change**, i.e.
  transformation does not yet touch the changed accessor surface; the *migration*
  of its raw `.0`/`.raw()`/arena reads to the accessor API is Phase B (audit +
  oracles only, no source migration). This pin merely makes the accessor API
  available to that later work.

## HARD STOP (Phase A)

Dependency pin only. No consumer source migration, no accessor rewrite, no M2b/M2c
work — those are the subsequent phases.
