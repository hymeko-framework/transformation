use hymeko::ir::ir::DeclKind;



/// Value comparison for numeric/string predicates.
#[derive(Debug, Clone)]
pub enum ValuePredicate {
    /// Exact numeric match (within epsilon).
    NumEq(f64),
    NumGt(f64),
    NumLt(f64),
    NumGte(f64),
    NumLte(f64),
    /// Exact string match.
    StrEq(String),
    /// Match any value (used for existence checks).
    Any,
}

impl ValuePredicate {
    pub fn matches_num(&self, v: f64) -> bool {
        match self {
            Self::NumEq(t)  => (v - t).abs() < 1e-9,
            Self::NumGt(t)  => v > *t,
            Self::NumLt(t)  => v < *t,
            Self::NumGte(t) => v >= *t,
            Self::NumLte(t) => v <= *t,
            Self::Any        => true,
            Self::StrEq(_)   => false,
        }
    }

    pub fn matches_str(&self, s: &str) -> bool {
        match self {
            Self::StrEq(t) => s == t,
            Self::Any       => true,
            _ => false,
        }
    }
}

/// Composable predicate for matching IR elements.
///
/// The predicate tree maps 1:1 to the HyMeKo grammar constructs:
///
/// | HyMeKo Syntax       | Predicate                      |
/// |----------------------|--------------------------------|
/// | `name { }`           | `Kind(Node) ∧ Named(name)`    |
/// | `_`                  | `Any` (wildcard)               |
/// | `n : base`           | `InheritsFrom(base)`           |
/// | `<tag>`              | `HasTag(tag)`                  |
/// | `mass 25.0;`         | `ChildValue("mass", NumEq(25))`|
/// | `@edge { }`          | `Kind(Edge)`                   |
/// | `+ref` in edge       | `HasPlusRef(ref-pred)`         |
/// | `-ref` in edge       | `HasMinusRef(ref-pred)`        |
/// | `{ child }` nesting  | `HasChild(child-pred)`         |
#[derive(Debug, Clone)]
pub enum Predicate {
    /// Always matches.
    Any,
    /// All sub-predicates must match.
    And(Vec<Predicate>),
    /// At least one sub-predicate must match.
    Or(Vec<Predicate>),
    /// Inner predicate must not match.
    Not(Box<Predicate>),

    /// Match by declaration kind (Node, Edge, HyperArc).
    Kind(DeclKind),
    /// Match by exact resolved name.
    Named(String),
    /// Match by resolved name prefix.
    NamePrefix(String),
    /// Match element that transitively inherits from `base_name`.
    InheritsFrom(String),
    /// Match element whose annotation contains `tag_name`.
    HasTag(String),
    /// Match element that has at least one child matching `inner`.
    HasChild(Box<Predicate>),
    /// Match element whose parent matches `inner`.
    HasParent(Box<Predicate>),
    /// Match element with a value satisfying the predicate.
    HasValue(ValuePredicate),
    /// Match element with a child named `name` whose value satisfies `pred`.
    ChildValue(String, ValuePredicate),

    // --- Edge-specific predicates (arc ref matching) ---

    /// Edge has a Plus-signed ref whose target matches `inner`.
    HasPlusRef(Box<Predicate>),
    /// Edge has a Minus-signed ref whose target matches `inner`.
    HasMinusRef(Box<Predicate>),
    /// Edge has a Neutral-signed ref whose target matches `inner`.
    HasNeutralRef(Box<Predicate>),
    /// Edge has any-sign ref whose target matches `inner`.
    HasRef(Box<Predicate>),
}

/// Builder methods for readable predicate composition.
///
/// # Example
/// ```ignore
/// let heavy_links = Predicate::node()
///     .and(Predicate::inherits("link"))
///     .and(Predicate::ChildValue("mass".into(), ValuePredicate::NumGt(10.0)));
/// ```
impl Predicate {
    pub fn node() -> Self { Self::Kind(DeclKind::Node) }
    pub fn edge() -> Self { Self::Kind(DeclKind::Edge) }

    pub fn named(n: &str) -> Self { Self::Named(n.to_string()) }
    pub fn name_prefix(p: &str) -> Self { Self::NamePrefix(p.to_string()) }
    pub fn inherits(base: &str) -> Self { Self::InheritsFrom(base.to_string()) }
    pub fn has_tag(tag: &str) -> Self { Self::HasTag(tag.to_string()) }
    pub fn tagged(tag: &str) -> Self { Self::HasTag(tag.to_string()) }

    /// Combine with AND, flattening nested And nodes.
    pub fn and(self, other: Self) -> Self {
        match self {
            Self::And(mut v) => { v.push(other); Self::And(v) }
            _ => Self::And(vec![self, other]),
        }
    }

    /// Combine with OR, flattening nested Or nodes.
    pub fn or(self, other: Self) -> Self {
        match self {
            Self::Or(mut v) => { v.push(other); Self::Or(v) }
            _ => Self::Or(vec![self, other]),
        }
    }

    /// Negate this predicate.
    pub fn not(self) -> Self { Self::Not(Box::new(self)) }
}

/// A labeled query: associates a human-readable name with a predicate.
/// Used by `QueryEngine::query_all` for batch evaluation.
#[derive(Debug, Clone)]
pub struct NamedQuery {
    pub label: String,
    pub predicate: Predicate,
}