//! End-to-end tests for the Tier B `const` + arithmetic resolver.
//!
//! Each test compiles a `.hymeko` fixture that uses the new
//! `const NAME = <expr>;` syntax (with arithmetic, builtins, and
//! forward references) and asserts that the emitted artefacts match
//! a literal-only equivalent fixture byte-for-byte. By Proposition 2,
//! the IR is identical iff the const resolution produced the same
//! numeric values, so byte-equal emission is the right test.

#[cfg(test)]
mod test_const_resolve {
    use crate::test_helpers::load_and_lower;
    use hymeko_formats::sdf::generate_sdf;
    use hymeko_formats::urdf::generate_urdf;

    const ROOT_NAME: &str = "mini_with_consts";
    const CONSTS_FIXTURE: &str = "../data/minimal_examples/constants/mini_with_consts.hymeko";
    const LITERALS_FIXTURE: &str = "../data/minimal_examples/constants/mini_literals.hymeko";

    #[test]
    fn const_fixture_emits_same_urdf_as_literal_fixture() {
        let (consts_store, consts_compiled) =
            load_and_lower(CONSTS_FIXTURE).expect("compile const fixture");
        let (literals_store, literals_compiled) =
            load_and_lower(LITERALS_FIXTURE).expect("compile literal fixture");

        let consts_urdf = generate_urdf(&consts_compiled.ir, &consts_store.it, ROOT_NAME);
        let literals_urdf = generate_urdf(&literals_compiled.ir, &literals_store.it, ROOT_NAME);

        assert_eq!(
            consts_urdf, literals_urdf,
            "URDF emitted from const fixture should byte-equal the literal fixture",
        );
    }

    #[test]
    fn const_fixture_emits_same_sdf_as_literal_fixture() {
        let (consts_store, consts_compiled) =
            load_and_lower(CONSTS_FIXTURE).expect("compile const fixture");
        let (literals_store, literals_compiled) =
            load_and_lower(LITERALS_FIXTURE).expect("compile literal fixture");

        let consts_sdf = generate_sdf(&consts_compiled.ir, &consts_store.it, ROOT_NAME);
        let literals_sdf = generate_sdf(&literals_compiled.ir, &literals_store.it, ROOT_NAME);

        assert_eq!(
            consts_sdf, literals_sdf,
            "SDF emitted from const fixture should byte-equal the literal fixture",
        );
    }

    #[test]
    fn const_fixture_compiles_with_forward_reference() {
        // `const LENGTH = LINK_HEIGHT * 2.0;` references LINK_HEIGHT
        // which is declared two lines later. Forward references must
        // be supported by the two-pass resolver.
        let result = load_and_lower(CONSTS_FIXTURE);
        assert!(
            result.is_ok(),
            "fixture with forward const reference failed to compile: {:?}",
            result.err()
        );
    }
}
