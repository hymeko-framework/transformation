//! Integration test harness for `hymeko_query` (multi-module fixtures, nested
//! `mod test_*` mirrors of one-crate-per-test layout). Pedantic clippy rules
//! for application code are suppressed here so `-D warnings` stays focused on
//! the library crate.
#![allow(
    dead_code,
    unused_imports,
    unused_mut,
    clippy::collapsible_if,
    clippy::doc_overindented_list_items,
    clippy::empty_line_after_doc_comments,
    clippy::module_inception,
    clippy::needless_lifetimes
)]

mod bench_workflow;
mod codegen;
mod test_anthropomorphic_generation;
mod test_const_resolve;
mod test_entropy;
mod test_gazebo_sim_launch;
mod test_gazebo_world;
mod test_generation_engine;
mod test_geom_extension;
mod test_helpers;
mod test_imported_real;
mod test_ir_migration_oracle;
mod test_mermaid;
mod test_prop_witnesses;
mod test_split;
mod test_sysml_emit;
mod test_template_driven;
mod test_torch_dataflow;
mod test_transform_ecosystem;
