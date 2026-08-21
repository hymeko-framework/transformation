//! XML / HTML escaping utility shared across format emitters.
//!
//! Single source of truth for `&`, `<`, `>`, `"`, `'` escaping. All
//! six XML-flavoured emitters in this crate (URDF, SDF, MJCF, DOT,
//! Mermaid, Gazebo) plus the WASM `compile` mirror call through this
//! one function instead of carrying their own copies.

/// Escape XML reserved characters: `& < > " '`.
///
/// Conforms to the XML 1.0 entity reference set. The apostrophe
/// (`&apos;`) is included because URDF attributes can contain
/// quotes that need escaping for XML well-formedness.
pub fn xml_escape(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn escapes_ampersand() {
        assert_eq!(xml_escape("a & b"), "a &amp; b");
    }

    #[test]
    fn escapes_all_five() {
        assert_eq!(
            xml_escape("&<>\"'"),
            "&amp;&lt;&gt;&quot;&apos;"
        );
    }

    #[test]
    fn ampersand_first_no_double_escape() {
        // The ampersand replacement must run first, otherwise later
        // replacements would corrupt the &lt; / &gt; etc. entities.
        assert_eq!(xml_escape("<&"), "&lt;&amp;");
    }
}
