//! End-to-end test for the `torch_dataflow` template-driven backend.
//!
//! Exercises the full path:
//!   `data/nn/simple_net.hymeko` → ModuleStore compile → IR
//!     → `transforms/torch_dataflow/{queries.hymeko, template.py}`
//!     → emitted PyTorch source string.
//!
//! The Python forward-pass round-trip (which depends on the
//! `ehk_torch_stub` package + a working `torch` install) lives in the
//! `python_forward_pass_round_trip` test below and is `#[ignore]` by
//! default — run it manually with
//!   `cargo test -p hymeko_query torch_dataflow -- --ignored`.

#[cfg(test)]
mod test_torch_dataflow {
    use std::path::PathBuf;
    use std::process::{Command, Stdio};

    use hymeko_query::transforms::TransformConfig;

    use crate::test_helpers::load_and_lower;

    const SIMPLE_NET: &str = "../data/nn/simple_net.hymeko";

    fn transforms_root() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("hymeko_query has a parent (workspace root)")
            .join("transforms")
    }

    fn render(name: &str) -> String {
        let (store, compiled) = load_and_lower(SIMPLE_NET).expect("compile simple_net");
        let reg = hymeko_formats::default_registry();
        let cfg = TransformConfig::default().with_name(name);
        reg.render_from_templates(
            "torch_dataflow",
            &compiled.ir,
            &store.it,
            &cfg,
            &transforms_root(),
        )
        .expect("torch_dataflow registered")
        .expect("template render succeeded")
    }

    #[test]
    fn emits_module_class_with_configured_name() {
        let out = render("SimpleNet");
        assert!(
            out.contains("class SimpleNet(nn.Module):"),
            "expected `class SimpleNet(nn.Module):` in output:\n{out}"
        );
    }

    #[test]
    fn emits_both_layer_constructors_with_correct_dims() {
        let out = render("SimpleNet");
        // layer_0: hypergraph_conv 3 → 5, bspline kernel.
        assert!(
            out.contains("self.layer_0 = HypergraphConv("),
            "missing layer_0 ctor in:\n{out}"
        );
        assert!(out.contains("d_in=3"), "layer_0 d_in=3 missing");
        assert!(out.contains("d_out=5"), "layer_0 d_out=5 missing");

        // layer_1: hypergraph_conv 5 → 2, rbf kernel.
        assert!(
            out.contains("self.layer_1 = HypergraphConv("),
            "missing layer_1 ctor in:\n{out}"
        );
        assert!(out.contains("d_in=5"), "layer_1 d_in=5 missing");
        assert!(out.contains("d_out=2"), "layer_1 d_out=2 missing");
    }

    #[test]
    fn emits_inheritance_branched_ggk_specs() {
        let out = render("SimpleNet");
        assert!(
            out.contains("basis=\"bspline\""),
            "bspline branch missing — `{{#inherits kernel \"bspline\"}}` did not fire"
        );
        assert!(out.contains("degree=3"), "bspline degree missing");
        assert!(out.contains("n_knots=8"), "bspline n_knots missing");

        assert!(
            out.contains("basis=\"rbf\""),
            "rbf branch missing — `{{#inherits kernel \"rbf\"}}` did not fire"
        );
        assert!(out.contains("n_centres=16"), "rbf n_centres missing");
    }

    #[test]
    fn emits_dataflow_forward_in_declaration_order() {
        let out = render("SimpleNet");
        // Each `@dataflow { (+ in_t, ~ layer, - out_t); }` becomes
        // `out_t = self.layer(in_t)`. Ordering must match the
        // hyperedge declaration order in the source: flow_0 then flow_1.
        let h_call = out
            .find("h = self.layer_0(x)")
            .expect("layer_0 call missing");
        let y_call = out
            .find("y = self.layer_1(h)")
            .expect("layer_1 call missing");
        assert!(
            h_call < y_call,
            "dataflow ordering inverted: layer_1 emitted before layer_0"
        );
    }

    #[test]
    fn emits_return_for_terminal_output_tensor() {
        let out = render("SimpleNet");
        assert!(
            out.contains("return y"),
            "expected `return y` (the t_output decl) in forward()"
        );
    }

    #[test]
    fn no_meta_type_pollution_in_constructor() {
        // Regression: meta-type defs from `meta_nn.hymeko` (e.g. the
        // `hypergraph_conv` decl that all user layers inherit from)
        // must not be emitted as `self.hypergraph_conv = HypergraphConv(...)`
        // — they have no `d_in` field and are filtered by the template's
        // `{{#if field:d_in}}` guard.
        let out = render("SimpleNet");
        assert!(
            !out.contains("self.hypergraph_conv = HypergraphConv("),
            "meta-type leaked through into constructor:\n{out}"
        );
    }

    /// Round-trip through Python: render → write to a tmp file →
    /// `python -c "import; instantiate; forward(randn(1,3))"`. Asserts
    /// the output shape is `(1, 2)`.
    ///
    /// Skipped by default because it requires the `ehk_torch_stub`
    /// package + `torch` to be importable from the system Python. Run
    /// with `cargo test torch_dataflow -- --ignored`.
    #[test]
    #[ignore]
    fn python_forward_pass_round_trip() {
        let out = render("SimpleNet");

        let tmp_dir = std::env::temp_dir().join("hymeko_torch_round_trip");
        std::fs::create_dir_all(&tmp_dir).expect("create tmp dir");
        let py_path = tmp_dir.join("simple_net.py");
        std::fs::write(&py_path, &out).expect("write emitted module");

        let driver = format!(
            r#"
import sys
sys.path.insert(0, {tmp_dir:?})
import torch
import simple_net
m = simple_net.SimpleNet()
y = m(torch.randn(1, 3))
assert tuple(y.shape) == (1, 2), f"unexpected shape {{tuple(y.shape)}}"
print("OK", tuple(y.shape))
"#,
            tmp_dir = tmp_dir.to_string_lossy(),
        );

        let status = Command::new("python")
            .arg("-c")
            .arg(&driver)
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit())
            .status()
            .expect("spawn python");

        assert!(
            status.success(),
            "python forward-pass failed — is `ehk_torch_stub` installed? \
             (`pip install -e python/ehk_torch_stub`)"
        );
    }
}
