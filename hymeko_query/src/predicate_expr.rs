//! The textual query language: one string that compiles to a [`Predicate`].
//!
//! ```text
//!   query   := or
//!   or      := and ("OR" and)*
//!   and     := unary ("AND" unary)*
//!   unary   := "NOT" unary | "(" query ")" | atom
//!
//!   ANY | NODE | EDGE
//!   KIND(name)               first declared base is `name`
//!   INHERITS(name)           is `name` or inherits it at any depth (the declaration included)
//!   SCOPEDIN(name)           some ancestor is or inherits `name`
//!   WITHIN(query)            some ancestor matches `query`
//!   CHILD(query) PARENT(query)
//!   HASARCREF(sign, query)   edge with a `sign` (+1, -1, 0) arc reference to a match
//!   REF(query)               edge with an arc reference of any sign to a match
//!   NAME(text)  NAMEPREFIX(text)  NAME~text
//!   TAG(name)                the declaration's annotation carries tag `name`
//!   BASETAG(name)            a base reference carries tag `name`; BASETAG(isa) marks a kind
//!   FIELD(key)               an effective (own or inherited) field `key` exists
//!   FIELD(key op value)      op: = != < <= > >= ~   (~ is substring)
//!   TEXT(text)               name or any effective field value contains `text`
//! ```
//!
//! `~` matches are case- and accent-insensitive ([`fold`]). Text is a bare token
//! (letters, digits, `_ . - : /` and any non-ASCII letter) or a double-quoted string.
//! Keywords are upper case. Bad input is an error that names its position, never an
//! empty result.
//!
//! The atoms `KIND`, `INHERITS`, `SCOPEDIN`, `HASARCREF`, `ANY` and `AND` keep the
//! meaning of the former string evaluator, which the SDK, Mission Control and the
//! editors already use; [`match_expr`] keeps its signature.

use crate::engine::QueryEngine;
use crate::predicate::{Predicate, ValuePredicate};
use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{DeclKind, Ir};
use hymeko::resolution::string_table::StringTable;
use std::fmt;

/// Where and why a query string was refused.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseError {
    pub position: usize,
    pub message: String,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "query error at character {}: {}", self.position, self.message)
    }
}

impl std::error::Error for ParseError {}

/// Compile a query string.
pub fn parse(query: &str) -> Result<Predicate, ParseError> {
    let mut parser = Parser { tokens: tokenize(query)?, at: 0, end: query.len() };
    let predicate = parser.or()?;
    match parser.peek() {
        None => Ok(predicate),
        Some((position, token)) => Err(ParseError { position: *position, message: format!("unexpected {token}") }),
    }
}

/// Evaluate a query string against one declaration. A query that does not parse
/// matches nothing, as before; use [`parse`] to see why.
pub fn match_expr(expr: &str, did: DeclId, ir: &Ir, st: &StringTable) -> bool {
    parse(expr).is_ok_and(|p| QueryEngine::new(ir, st).query_decl(did, &p).is_some())
}

/// Evaluate a single atom; kept for callers of the former evaluator.
pub fn match_atom(atom: &str, did: DeclId, ir: &Ir, st: &StringTable) -> bool {
    match_expr(atom, did, ir, st)
}

/// The name of the declaration's first declared base, or `""`.
pub fn decl_kind_name<'a>(did: DeclId, ir: &'a Ir, st: &'a StringTable) -> &'a str {
    let first = match ir.decl_nodes[did.raw()].kind {
        DeclKind::Node => ir.as_node(did).and_then(|n| ir[ir.nodes[n.raw()].bases].first()),
        DeclKind::Edge => ir.as_edge(did).and_then(|e| ir[ir.edges[e.raw()].bases].first()),
        DeclKind::HyperArc => None,
    };
    first.map_or("", |b| st.resolve(ir.decl_nodes[b.target().raw()].name))
}

pub fn decl_inherits(did: DeclId, target_name: &str, ir: &Ir, st: &StringTable) -> bool {
    QueryEngine::new(ir, st).query_decl(did, &Predicate::IsA(target_name.into())).is_some()
}

pub fn decl_scoped_in(did: DeclId, name: &str, ir: &Ir, st: &StringTable) -> bool {
    QueryEngine::new(ir, st).query_decl(did, &Predicate::Within(Box::new(Predicate::IsA(name.into())))).is_some()
}

pub fn has_arc_ref(did: DeclId, sign: i8, inner: &str, ir: &Ir, st: &StringTable) -> bool {
    let Ok(inner) = parse(inner) else { return false };
    let predicate = signed_ref(sign, inner);
    QueryEngine::new(ir, st).query_decl(did, &predicate).is_some()
}

fn signed_ref(sign: i8, inner: Predicate) -> Predicate {
    match sign {
        1 => Predicate::HasPlusRef(Box::new(inner)),
        -1 => Predicate::HasMinusRef(Box::new(inner)),
        _ => Predicate::HasNeutralRef(Box::new(inner)),
    }
}

#[derive(Debug, Clone, PartialEq)]
enum Token {
    Word(String),
    Text(String),
    Number(f64),
    Open,
    Close,
    Comma,
    Op(&'static str),
}

impl fmt::Display for Token {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Word(w) => write!(f, "`{w}`"),
            Self::Text(t) => write!(f, "\"{t}\""),
            Self::Number(n) => write!(f, "{n}"),
            Self::Open => f.write_str("`(`"),
            Self::Close => f.write_str("`)`"),
            Self::Comma => f.write_str("`,`"),
            Self::Op(o) => write!(f, "`{o}`"),
        }
    }
}

fn word_char(c: char) -> bool {
    c.is_alphanumeric() || matches!(c, '_' | '.' | '-' | ':' | '/' | '+')
}

fn tokenize(query: &str) -> Result<Vec<(usize, Token)>, ParseError> {
    let mut out = Vec::new();
    let mut chars = query.char_indices().peekable();
    while let Some(&(at, c)) = chars.peek() {
        if c.is_whitespace() {
            chars.next();
            continue;
        }
        let single = match c {
            '(' => Some(Token::Open),
            ')' => Some(Token::Close),
            ',' => Some(Token::Comma),
            '~' => Some(Token::Op("~")),
            _ => None,
        };
        if let Some(token) = single {
            chars.next();
            out.push((at, token));
            continue;
        }
        if matches!(c, '=' | '!' | '<' | '>') {
            chars.next();
            let double = chars.peek().is_some_and(|&(_, n)| n == '=');
            if double {
                chars.next();
            }
            let op = match (c, double) {
                ('=', _) => "=",
                ('!', true) => "!=",
                ('<', false) => "<",
                ('<', true) => "<=",
                ('>', false) => ">",
                ('>', true) => ">=",
                _ => return Err(ParseError { position: at, message: "`!` must be followed by `=`".into() }),
            };
            out.push((at, Token::Op(op)));
            continue;
        }
        if c == '"' {
            chars.next();
            let mut text = String::new();
            loop {
                match chars.next() {
                    Some((_, '"')) => break,
                    Some((_, '\\')) => match chars.next() {
                        Some((_, e)) => text.push(e),
                        None => return Err(ParseError { position: at, message: "unterminated string".into() }),
                    },
                    Some((_, ch)) => text.push(ch),
                    None => return Err(ParseError { position: at, message: "unterminated string".into() }),
                }
            }
            out.push((at, Token::Text(text)));
            continue;
        }
        if word_char(c) {
            let mut word = String::new();
            while let Some(&(_, ch)) = chars.peek() {
                if !word_char(ch) {
                    break;
                }
                word.push(ch);
                chars.next();
            }
            let token = match word.parse::<f64>() {
                Ok(n) if n.is_finite() && !word.chars().any(char::is_alphabetic) => Token::Number(n),
                _ => Token::Word(word),
            };
            out.push((at, token));
            continue;
        }
        return Err(ParseError { position: at, message: format!("unexpected character `{c}`") });
    }
    Ok(out)
}

struct Parser {
    tokens: Vec<(usize, Token)>,
    at: usize,
    end: usize,
}

impl Parser {
    fn peek(&self) -> Option<&(usize, Token)> {
        self.tokens.get(self.at)
    }

    fn position(&self) -> usize {
        self.peek().map_or(self.end, |(p, _)| *p)
    }

    fn error<T>(&self, message: impl Into<String>) -> Result<T, ParseError> {
        Err(ParseError { position: self.position(), message: message.into() })
    }

    fn next(&mut self) -> Option<Token> {
        let token = self.tokens.get(self.at).map(|(_, t)| t.clone());
        self.at += 1;
        token
    }

    fn keyword(&mut self, word: &str) -> bool {
        if matches!(self.peek(), Some((_, Token::Word(w))) if w == word) {
            self.at += 1;
            true
        } else {
            false
        }
    }

    fn expect(&mut self, token: Token) -> Result<(), ParseError> {
        match self.peek() {
            Some((_, t)) if *t == token => {
                self.at += 1;
                Ok(())
            }
            Some((_, t)) => self.error(format!("expected {token}, found {t}")),
            None => self.error(format!("expected {token} at the end")),
        }
    }

    fn or(&mut self) -> Result<Predicate, ParseError> {
        let mut terms = vec![self.and()?];
        while self.keyword("OR") {
            terms.push(self.and()?);
        }
        Ok(if terms.len() == 1 { terms.pop().unwrap() } else { Predicate::Or(terms) })
    }

    fn and(&mut self) -> Result<Predicate, ParseError> {
        let mut terms = vec![self.unary()?];
        while self.keyword("AND") {
            terms.push(self.unary()?);
        }
        Ok(if terms.len() == 1 { terms.pop().unwrap() } else { Predicate::And(terms) })
    }

    fn unary(&mut self) -> Result<Predicate, ParseError> {
        if self.keyword("NOT") {
            return Ok(Predicate::Not(Box::new(self.unary()?)));
        }
        if matches!(self.peek(), Some((_, Token::Open))) {
            self.at += 1;
            let inner = self.or()?;
            self.expect(Token::Close)?;
            return Ok(inner);
        }
        self.atom()
    }

    /// A name or text argument: a bare word, a quoted string or a number written as text.
    fn text(&mut self) -> Result<String, ParseError> {
        match self.next() {
            Some(Token::Word(w)) | Some(Token::Text(w)) => Ok(w),
            Some(Token::Number(_)) => Ok(self.tokens[self.at - 1].1.to_string()),
            Some(t) => {
                self.at -= 1;
                self.error(format!("expected a name or text, found {t}"))
            }
            None => self.error("expected a name or text at the end"),
        }
    }

    fn call<T>(&mut self, body: impl FnOnce(&mut Self) -> Result<T, ParseError>) -> Result<T, ParseError> {
        self.expect(Token::Open)?;
        let value = body(self)?;
        self.expect(Token::Close)?;
        Ok(value)
    }

    fn atom(&mut self) -> Result<Predicate, ParseError> {
        let start = self.at;
        let word = match self.next() {
            Some(Token::Word(w)) => w,
            Some(t) => {
                self.at = start;
                return self.error(format!("expected a query atom, found {t}"));
            }
            None => return self.error("expected a query atom at the end"),
        };
        Ok(match word.as_str() {
            "ANY" => Predicate::Any,
            "NODE" => Predicate::Kind(DeclKind::Node),
            "EDGE" => Predicate::Kind(DeclKind::Edge),
            "KIND" => Predicate::FirstBase(self.call(Self::text)?),
            "INHERITS" => Predicate::IsA(self.call(Self::text)?),
            "SCOPEDIN" => Predicate::Within(Box::new(Predicate::IsA(self.call(Self::text)?))),
            "WITHIN" => Predicate::Within(Box::new(self.call(Self::or)?)),
            "CHILD" => Predicate::HasChild(Box::new(self.call(Self::or)?)),
            "PARENT" => Predicate::HasParent(Box::new(self.call(Self::or)?)),
            "REF" => Predicate::HasRef(Box::new(self.call(Self::or)?)),
            "HASARCREF" => self.call(|p| {
                let sign = match p.next() {
                    Some(Token::Number(n)) if [1., -1., 0.].contains(&n) => n as i8,
                    Some(Token::Word(w)) if w == "+1" || w == "+" => 1,
                    Some(Token::Word(w)) if w == "-" => -1,
                    _ => {
                        p.at -= 1;
                        return p.error("HASARCREF needs a sign: +1, -1 or 0");
                    }
                };
                p.expect(Token::Comma)?;
                Ok(signed_ref(sign, p.or()?))
            })?,
            "NAME" => match self.peek() {
                Some((_, Token::Op("~"))) => {
                    self.at += 1;
                    Predicate::NameContains(self.text()?)
                }
                _ => Predicate::Named(self.call(Self::text)?),
            },
            "NAMEPREFIX" => Predicate::NamePrefix(self.call(Self::text)?),
            "TAG" => Predicate::HasTag(self.call(Self::text)?),
            "BASETAG" => Predicate::BaseTagged(self.call(Self::text)?),
            "TEXT" => {
                let text = self.call(Self::text)?;
                Predicate::Or(vec![Predicate::NameContains(text.clone()), Predicate::AnyFieldValue(ValuePredicate::StrContains(text))])
            }
            "FIELD" => self.call(Self::field)?,
            other => {
                self.at = start;
                return self.error(format!("unknown query atom `{other}` (atoms and AND/OR/NOT are upper case)"));
            }
        })
    }

    fn field(&mut self) -> Result<Predicate, ParseError> {
        let key = self.text()?;
        let op = match self.peek() {
            Some((_, Token::Op(op))) => *op,
            _ => return Ok(Predicate::EffectiveChildValue(key, ValuePredicate::Any)),
        };
        self.at += 1;
        let at = self.position();
        let value = match self.next() {
            Some(t @ (Token::Number(_) | Token::Word(_) | Token::Text(_))) => t,
            _ => {
                self.at -= 1;
                return self.error(format!("FIELD({key} {op} …) needs a value"));
            }
        };
        let field = |vp| Predicate::EffectiveChildValue(key.clone(), vp);
        Ok(match (op, value) {
            ("~", Token::Number(n)) => field(ValuePredicate::StrContains(n.to_string())),
            ("~", Token::Word(t) | Token::Text(t)) => field(ValuePredicate::StrContains(t)),
            ("=", Token::Number(n)) => field(ValuePredicate::NumEq(n)),
            ("!=", Token::Number(n)) => Predicate::Not(Box::new(field(ValuePredicate::NumEq(n)))),
            ("<", Token::Number(n)) => field(ValuePredicate::NumLt(n)),
            ("<=", Token::Number(n)) => field(ValuePredicate::NumLte(n)),
            (">", Token::Number(n)) => field(ValuePredicate::NumGt(n)),
            (">=", Token::Number(n)) => field(ValuePredicate::NumGte(n)),
            ("=", Token::Word(t) | Token::Text(t)) => field(ValuePredicate::StrEq(t)),
            ("!=", Token::Word(t) | Token::Text(t)) => Predicate::Not(Box::new(field(ValuePredicate::StrEq(t)))),
            (op, _) => return Err(ParseError { position: at, message: format!("`{op}` compares numbers") }),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::predicate::fold;

    fn shape(query: &str) -> String {
        format!("{:?}", parse(query).unwrap())
    }

    #[test]
    fn the_former_atoms_keep_their_meaning() {
        assert_eq!(shape("KIND(link)"), r#"FirstBase("link")"#);
        assert_eq!(shape("INHERITS(link)"), r#"IsA("link")"#);
        assert_eq!(shape("SCOPEDIN(robot)"), r#"Within(IsA("robot"))"#);
        assert_eq!(shape("HASARCREF(+1, INHERITS(link))"), r#"HasPlusRef(IsA("link"))"#);
        assert_eq!(shape("HASARCREF(-1,KIND(axis))"), r#"HasMinusRef(FirstBase("axis"))"#);
        assert_eq!(shape("ANY"), "Any");
        assert_eq!(shape("INHERITS(a) AND SCOPEDIN(b)"), r#"And([IsA("a"), Within(IsA("b"))])"#);
    }

    #[test]
    fn nesting_precedence_and_new_atoms() {
        assert_eq!(shape("HASARCREF(1, INHERITS(a) AND KIND(b))"), r#"HasPlusRef(And([IsA("a"), FirstBase("b")]))"#);
        assert_eq!(shape("NOT TAG(x) AND NAME(a) OR NAME~b"), r#"Or([And([Not(HasTag("x")), Named("a")]), NameContains("b")])"#);
        assert_eq!(shape("NOT (NAME(a) OR NAME(b))"), r#"Not(Or([Named("a"), Named("b")]))"#);
        assert_eq!(shape(r#"FIELD(title ~ "Oktatás")"#), r#"EffectiveChildValue("title", StrContains("Oktatás"))"#);
        assert_eq!(shape("FIELD(mass >= 2.5)"), r#"EffectiveChildValue("mass", NumGte(2.5))"#);
        assert_eq!(shape("FIELD(mass != 3)"), r#"Not(EffectiveChildValue("mass", NumEq(3.0)))"#);
        assert_eq!(shape("FIELD(hidden)"), r#"EffectiveChildValue("hidden", Any)"#);
        assert_eq!(shape("WITHIN(NAME(robot)) AND EDGE"), r#"And([Within(Named("robot")), Kind(Edge)])"#);
        assert!(shape("TEXT(oktat)").starts_with(r#"Or([NameContains("oktat"), AnyFieldValue(StrContains("oktat"))])"#));
        assert_eq!(shape("NAME(hk.elements.project)"), r#"Named("hk.elements.project")"#);
        assert_eq!(shape("INHERITS(project) AND NOT BASETAG(isa)"), r#"And([IsA("project"), Not(BaseTagged("isa"))])"#);
    }

    #[test]
    fn bad_queries_are_errors_with_a_position() {
        for (query, at) in [("KIND(link", 9), ("INHERITS(a) AND", 15), ("FOO(a)", 0), ("NAME(a) b", 8), ("FIELD(mass > big)", 13), ("\"open", 0), ("inherits(a)", 0), ("HASARCREF(x, ANY)", 10)] {
            let error = parse(query).unwrap_err();
            assert_eq!(error.position, at, "{query}: {error}");
        }
    }

    /// The former split-on-" AND " evaluator, verbatim, as the compatibility reference.
    mod legacy {
        use hymeko::common::ids::DeclId;
        use hymeko::ir::ir::{DeclKind, Ir};
        use hymeko::resolution::string_table::StringTable;

        pub fn match_expr(expr: &str, did: DeclId, ir: &Ir, st: &StringTable) -> bool {
            expr.split(" AND ").all(|p| match_atom(p.trim(), did, ir, st))
        }
        fn match_atom(atom: &str, did: DeclId, ir: &Ir, st: &StringTable) -> bool {
            if atom == "ANY" {
                return true;
            }
            if let Some(rest) = atom.strip_prefix("KIND(") {
                return kind_name(did, ir, st) == rest.trim_end_matches(')');
            }
            if let Some(rest) = atom.strip_prefix("INHERITS(") {
                return inherits(did, rest.trim_end_matches(')'), ir, st);
            }
            if let Some(rest) = atom.strip_prefix("SCOPEDIN(") {
                let name = rest.trim_end_matches(')');
                let mut cur = ir.decl_nodes[did.raw()].parent;
                while cur.is_some() {
                    if inherits(cur, name, ir, st) || st.resolve(ir.decl_nodes[cur.raw()].name) == name {
                        return true;
                    }
                    cur = ir.decl_nodes[cur.raw()].parent;
                }
                return false;
            }
            if let Some(rest) = atom.strip_prefix("HASARCREF(") {
                let rest = rest.trim_end_matches(')');
                let (sign_s, inner) = rest.split_once(',').unwrap_or((rest, ""));
                let sign: i8 = sign_s.trim().trim_start_matches('+').parse().unwrap_or(0);
                let Some(eid) = ir.as_edge(did) else { return false };
                return ir.edges[eid.raw()].arcs.iter().any(|&aid| ir.arcs[aid.raw()].refs.iter().any(|r| r.sign() == sign && match_expr(inner.trim(), r.target(), ir, st)));
            }
            false
        }
        fn kind_name<'a>(did: DeclId, ir: &'a Ir, st: &'a StringTable) -> &'a str {
            let first = match ir.decl_nodes[did.raw()].kind {
                DeclKind::Node => ir.as_node(did).and_then(|n| ir[ir.nodes[n.raw()].bases].first()),
                DeclKind::Edge => ir.as_edge(did).and_then(|e| ir[ir.edges[e.raw()].bases].first()),
                DeclKind::HyperArc => None,
            };
            first.map_or("", |b| st.resolve(ir.decl_nodes[b.target().raw()].name))
        }
        fn inherits(did: DeclId, target: &str, ir: &Ir, st: &StringTable) -> bool {
            let mut seen = std::collections::HashSet::new();
            let mut stack = vec![did];
            while let Some(d) = stack.pop() {
                if !seen.insert(d) {
                    continue;
                }
                let decl = &ir.decl_nodes[d.raw()];
                if st.resolve(decl.name) == target {
                    return true;
                }
                match decl.kind {
                    DeclKind::Node => stack.extend(ir.as_node(d).into_iter().flat_map(|n| ir[ir.nodes[n.raw()].bases].iter().map(|b| b.target()))),
                    DeclKind::Edge => stack.extend(ir.as_edge(d).into_iter().flat_map(|e| ir[ir.edges[e.raw()].bases].iter().map(|b| b.target()))),
                    DeclKind::HyperArc => {}
                }
            }
            false
        }
    }

    const COMPATIBLE: [&str; 16] = [
        "ANY",
        "KIND(link)",
        "KIND(rev_joint)",
        "INHERITS(link)",
        "INHERITS(joint)",
        "INHERITS(elements)",
        "SCOPEDIN(robot)",
        "SCOPEDIN(quadruped)",
        "SCOPEDIN(kinematics)",
        "INHERITS(link) AND SCOPEDIN(robot)",
        "HASARCREF(+1, INHERITS(link))",
        "HASARCREF(-1, INHERITS(link))",
        "HASARCREF(-1, INHERITS(axis_definition))",
        "HASARCREF(1, ANY)",
        "INHERITS(rev_joint) AND HASARCREF(-1, KIND(link))",
        "KIND(experiment) AND SCOPEDIN(simulation_types)",
    ];

    fn agree(model: &hymeko::module_store::execution::CompiledModel) -> usize {
        let st = StringTable::from_interner(model.interner());
        let ir = &model.program().ir;
        let mut hits = 0;
        for query in COMPATIBLE {
            for (did, _) in ir.decls() {
                let (new, old) = (match_expr(query, did, ir, &st), legacy::match_expr(query, did, ir, &st));
                assert_eq!(new, old, "{query} on {}", st.resolve(ir.decl_nodes[did.raw()].name));
                hits += usize::from(new);
            }
        }
        hits
    }

    #[test]
    fn the_new_evaluator_agrees_with_the_former_one_on_every_declaration() {
        let synthetic = crate::test_support::compile(
            "t_description {}
base {}
link: base { mass 1; }
robot {
  a: link {}
  b: link { mass 2; }
  axis_definition {}
  x: axis_definition {}
  @j: link { (+ a, - b, - x); }
}
",
        );
        assert!(agree(&synthetic) > 0);
        let data = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../data");
        let mut fixtures = 0;
        for file in ["robotics/robots/quadruped.hymeko", "robotics/sim/dual_fanuc/fanuc_lrmate200id.hymeko", "paper/traceability_simulation.hymeko"] {
            let path = data.join(file);
            if !path.exists() {
                continue;
            }
            let model = hymeko::module_store::execution::CompiledModel::compile(hymeko::module_store::source_provider::StdFsProvider::new(), hymeko::util::real_parser::RealParser, &path).unwrap();
            assert!(agree(&model) > 0, "{file}");
            fixtures += 1;
        }
        assert!(fixtures >= 2, "the checked-in robot fixtures are present");
    }

    #[test]
    fn new_atoms_select_what_they_say() {
        let model = crate::test_support::compile(
            "t_description {}
element {}
project: + <isa> element { noun \"projekt\"; }
course: + <isa> project {}
work {
  ablak: project { label \"Ablakemelő-vezérlő\"; mass 3; }
  ipar: course { label \"Ipar 4.0\"; mass 1; }
  note: element { label \"ablakemelo megjegyzés\"; }
}
",
        );
        let st = StringTable::from_interner(model.interner());
        let ir = &model.program().ir;
        let names = |query: &str| {
            let predicate = parse(query).unwrap();
            let mut out: Vec<String> = QueryEngine::new(ir, &st).query(&predicate).into_iter().map(|m| m.name).collect();
            out.sort();
            out
        };
        assert_eq!(names("INHERITS(project) AND NOT BASETAG(isa)"), ["ablak", "ipar"]);
        assert_eq!(names("INHERITS(project) AND BASETAG(isa)"), ["course", "project"]);
        assert_eq!(names("FIELD(mass > 2)"), ["ablak"]);
        assert_eq!(names("FIELD(label ~ IPAR)"), ["ipar"]);
        assert_eq!(names("TEXT(ablakemelo) AND WITHIN(NAME(work))"), ["ablak", "note"]);
        assert_eq!(names("FIELD(noun = projekt) AND NOT NAME(project)"), ["ablak", "course", "ipar"], "nouns are inherited fields");
        assert_eq!(names("NAME~ABL OR NAMEPREFIX(ip)"), ["ablak", "ipar"]);
    }

    #[test]
    fn folding_ignores_case_and_accents() {
        assert_eq!(fold("Oktatás ŐSZ Straße"), "oktatas osz strasse");
        assert!(ValuePredicate::StrContains("OKTATAS".into()).matches_str("Oktatási projekt"));
    }
}
