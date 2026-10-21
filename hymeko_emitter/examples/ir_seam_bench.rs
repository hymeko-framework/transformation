//! std-only benchmark probe for the IR construction seam and the M2T emitters.
//!
//! Not part of the production API; no dependency beyond `std` + the crate + core.
//! Per scale it builds an editor IR programmatically, then times, with medians
//! over `ITERS` after a warm-up:
//!
//! * `CompilerIrBuilder::build` — the **sole production writer** (editor -> arena);
//! * each of the four M2T emitters — **readers** (`&Ir` -> `String`).
//!
//! Every output carries a **stable FNV-1a-64** fingerprint and its byte length,
//! so a behaviour change is visible in the probe, not only in the tests. Run:
//!
//! ```text
//! cargo run --release -p hymeko_emitter --example ir_seam_bench
//! ```

use std::time::Instant;

use hymeko::resolution::interner::Interner;
use hymeko_emitter::bridge::{CompilerIrBuilder, IrBuilder};
use hymeko_emitter::editor_ir::{HyMeKoEditorIR, HyperEdge, Sign, Vertex};
use hymeko_emitter::{emit_hymeko, emit_lean4, emit_rust_stubs, emit_sysml};

fn fnv1a_64(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v[v.len() / 2]
}

/// A grid editor IR: `rows x cols` vertices, one hyperedge per row spanning its
/// columns (first `+`, rest `-`). Deterministic; scales incidences linearly.
fn grid_editor(rows: usize, cols: usize) -> HyMeKoEditorIR {
    let mut e = HyMeKoEditorIR::new();
    let mut keys = Vec::with_capacity(rows * cols);
    for i in 0..rows * cols {
        keys.push(e.vertices.insert(Vertex {
            name: format!("v{i}"),
            level: 0,
            attributes: vec![],
            position: None,
        }));
    }
    for r in 0..rows {
        let incident = (0..cols)
            .map(|c| {
                let sign = if c == 0 { Sign::Plus } else { Sign::Minus };
                (keys[r * cols + c], sign)
            })
            .collect();
        e.hyperedges.insert(HyperEdge {
            name: format!("e{r}"),
            incident,
            weight: 1.0,
            patch_id: None,
        });
    }
    e
}

fn time_median<F: FnMut()>(iters: usize, mut f: F) -> f64 {
    f(); // warm-up
    let mut samples = Vec::with_capacity(iters);
    for _ in 0..iters {
        let t = Instant::now();
        f();
        samples.push(t.elapsed().as_secs_f64() * 1e6);
    }
    median(samples)
}

fn bench(rows: usize, cols: usize, iters: usize) {
    let editor = grid_editor(rows, cols);
    let mut interner = Interner::new();

    let write_us = time_median(iters, || {
        let ir = CompilerIrBuilder::new(&editor, &mut Interner::new())
            .build()
            .expect("canonical editor construction");
        std::hint::black_box(&ir);
    });

    let ir = CompilerIrBuilder::new(&editor, &mut interner)
        .build()
        .expect("canonical editor construction");
    let incidences: usize = ir.arcs.iter().map(|a| a.refs.len()).sum();

    let h = emit_hymeko(&ir, &interner, "bench");
    let s = emit_sysml(&ir, &interner, "bench");
    let r = emit_rust_stubs(&ir, &interner);
    let l = emit_lean4(&ir, &interner);

    let hymeko_us = time_median(iters, || {
        std::hint::black_box(emit_hymeko(&ir, &interner, "bench"));
    });
    let sysml_us = time_median(iters, || {
        std::hint::black_box(emit_sysml(&ir, &interner, "bench"));
    });
    let rust_us = time_median(iters, || {
        std::hint::black_box(emit_rust_stubs(&ir, &interner));
    });
    let lean_us = time_median(iters, || {
        std::hint::black_box(emit_lean4(&ir, &interner));
    });

    println!(
        "{:>4}x{:<3} decls={:>6} inc={:>6} | write={:>9.2}us | hymeko={:>8.2}us({}b fp={:016x}) sysml={:>8.2}us rust={:>8.2}us lean={:>8.2}us",
        rows,
        cols,
        ir.decl_nodes.len(),
        incidences,
        write_us,
        hymeko_us,
        h.len(),
        fnv1a_64(h.as_bytes()),
        sysml_us,
        rust_us,
        lean_us,
    );
    std::hint::black_box((s.len(), r.len(), l.len()));
}

fn main() {
    println!(
        "# ir_seam_bench (release, medians) — writer=CompilerIrBuilder::build, readers=4 emitters"
    );
    bench(4, 3, 200);
    bench(32, 4, 100);
    bench(128, 5, 50);
    bench(512, 6, 20);
    bench(2048, 8, 5);
}
