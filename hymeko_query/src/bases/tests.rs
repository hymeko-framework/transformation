use super::*;
use crate::test_support::{compile, decl};
use hymeko::ir::ir::RefAtomR;
use hymeko::module_store::execution::CompiledModel;

const TYPES: &str = "bases {} types { thing {} part: + <isa> thing {} tool: + <isa> thing {} \
    probe: + <isa> part, + <isa> tool {} marked {} } \
    item: + <isa> types.probe, + types.marked {} other {}";

/// The model's IR with one more base on `thing`, pointing at `target`: `part` makes a cycle
/// (thing -> part -> thing), a number past the table a dangling base.
fn thing_with_base(model: &CompiledModel, target: DeclId) -> Ir {
    let mut ir = model.program().ir.clone();
    let mut base = ir.node_for_decl(decl(model, "part")).unwrap().1.bases[0].clone();
    let (SignedRefR::Plus(RefAtomR { target: t, .. })
    | SignedRefR::Minus(RefAtomR { target: t, .. })
    | SignedRefR::Neutral(RefAtomR { target: t, .. })) = &mut base;
    *t = target;
    let node = ir.node_for_decl(decl(model, "thing")).unwrap().0.raw();
    ir.nodes[node].bases.push(base);
    ir
}

#[test]
fn direct_bases_follow_authored_order_and_the_tag_policy() {
    let model = compile(TYPES);
    let (ir, isa) = (&model.program().ir, model.interner().get_id("isa").unwrap());
    let item = decl(&model, "item");
    let all: Vec<_> = bases(ir, item, BasePolicy::All).collect();
    assert_eq!(all, [decl(&model, "probe"), decl(&model, "marked")]);
    let tagged: Vec<_> = bases(ir, item, BasePolicy::Tagged(isa)).collect();
    assert_eq!(
        tagged,
        [decl(&model, "probe")],
        "the untagged base does not count as isa"
    );
    assert_eq!(bases(ir, decl(&model, "other"), BasePolicy::All).count(), 0);
}

#[test]
fn is_a_is_reflexive_transitive_and_visits_a_diamond_once() {
    let model = compile(TYPES);
    let (ir, isa) = (&model.program().ir, model.interner().get_id("isa").unwrap());
    let [item, thing, probe, marked, other] =
        ["item", "thing", "probe", "marked", "other"].map(|name| decl(&model, name));
    assert_eq!(is_a(ir, item, item, BasePolicy::All), Ok(true), "reflexive");
    assert_eq!(
        is_a(ir, item, thing, BasePolicy::All),
        Ok(true),
        "through the diamond"
    );
    assert_eq!(is_a(ir, probe, thing, BasePolicy::Tagged(isa)), Ok(true));
    assert_eq!(is_a(ir, item, marked, BasePolicy::All), Ok(true));
    assert_eq!(
        is_a(ir, item, marked, BasePolicy::Tagged(isa)),
        Ok(false),
        "untagged base"
    );
    assert_eq!(is_a(ir, item, other, BasePolicy::All), Ok(false));
    assert_eq!(
        is_a(ir, thing, item, BasePolicy::All),
        Ok(false),
        "not downwards"
    );
    let engine = QueryEngine::new(ir, model.interner());
    assert_eq!(engine.is_a(item, thing, BasePolicy::All), Ok(true));
    assert_eq!(engine.bases(item, BasePolicy::Tagged(isa)).count(), 1);
}

#[test]
fn a_cycle_and_a_dangling_base_are_errors_not_answers() {
    let model = compile(TYPES);
    let [thing, part, other] = ["thing", "part", "other"].map(|name| decl(&model, name));
    let cyclic = thing_with_base(&model, part);
    assert_eq!(
        is_a(&cyclic, part, other, BasePolicy::All),
        Err(BaseError::Cycle { decl: part })
    );
    assert_eq!(
        is_a(&cyclic, part, thing, BasePolicy::All),
        Ok(true),
        "found before the cycle"
    );

    let missing = DeclId::new(model.program().ir.decl_nodes.len() + 7);
    let dangling = thing_with_base(&model, missing);
    assert_eq!(
        is_a(&dangling, thing, other, BasePolicy::All),
        Err(BaseError::Dangling {
            decl: thing,
            base: missing
        })
    );
}

#[test]
fn the_shallow_walk_answers_as_the_full_walk_does() {
    let model = compile(TYPES);
    let cyclic = thing_with_base(&model, decl(&model, "part"));
    let isa = model.interner().get_id("isa").unwrap();
    for ir in [&model.program().ir, &cyclic] {
        for (a, _) in ir.decls() {
            for (b, _) in ir.decls() {
                for policy in [BasePolicy::All, BasePolicy::Tagged(isa)] {
                    let full = if a == b {
                        Ok(true)
                    } else {
                        walk(ir, a, b, policy)
                    };
                    assert_eq!(is_a(ir, a, b, policy), full, "{a:?} {b:?}");
                }
            }
        }
    }
}

#[test]
fn a_deep_chain_and_a_diamond_ladder_fall_back_to_the_full_walk() {
    let mut chain = String::from("bases {} c0 {} ");
    for i in 1..100 {
        chain.push_str(&format!("c{i}: + <isa> c{} {{}} ", i - 1));
    }
    let model = compile(&chain);
    let (ir, top, root) = (&model.program().ir, decl(&model, "c99"), decl(&model, "c0"));
    assert_eq!(
        is_a(ir, top, root, BasePolicy::All),
        Ok(true),
        "deeper than the shallow walk"
    );
    assert_eq!(is_a(ir, root, top, BasePolicy::All), Ok(false));

    // Twelve diamonds in a row: 4096 paths to the root, more than the shallow walk visits.
    let mut ladder = String::from("bases {} m0 {} other {} ");
    for i in 1..=12 {
        let below = i - 1;
        ladder.push_str(&format!(
            "a{i}: + <isa> m{below} {{}} b{i}: + <isa> m{below} {{}} m{i}: + <isa> a{i}, + <isa> b{i} {{}} "
        ));
    }
    let model = compile(&ladder);
    let ir = &model.program().ir;
    let [top, root, other] = ["m12", "m0", "other"].map(|name| decl(&model, name));
    assert_eq!(is_a(ir, top, root, BasePolicy::All), Ok(true));
    assert_eq!(is_a(ir, top, other, BasePolicy::All), Ok(false));
    assert_eq!(walk(ir, top, other, BasePolicy::All), Ok(false));
}
