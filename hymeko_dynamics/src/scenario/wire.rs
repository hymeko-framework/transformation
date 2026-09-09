use super::ScenarioSession;
use super::model::*;
use ciborium::value::{CanonicalValue, Value};

pub(crate) fn encoding_error(error: impl std::fmt::Display) -> Refusal {
    Refusal::new(RefusalCode::InvalidRequest, format!("encoding: {error}"))
}
impl Snapshot {
    pub fn to_json(&self) -> Result<String, Refusal> {
        serde_json::to_string(self).map_err(encoding_error)
    }
    /// Canonical CBOR: preferred integer widths, definite lengths, recursively
    /// canonical map order. Schema v1 permits only text map keys, integers, bools,
    /// null and arrays, so RFC 8949 core and length-first key order coincide.
    pub fn to_cbor(&self) -> Result<Vec<u8>, Refusal> {
        let mut value = Value::serialized(self).map_err(encoding_error)?;
        Self::canonicalize(&mut value);
        let mut bytes = Vec::new();
        ciborium::ser::into_writer(&value, &mut bytes).map_err(encoding_error)?;
        Ok(bytes)
    }
    fn canonicalize(value: &mut Value) {
        match value {
            Value::Map(entries) => {
                for (_, child) in entries.iter_mut() {
                    Self::canonicalize(child);
                }
                entries.sort_by_cached_key(|(key, _)| CanonicalValue::from(key.clone()));
            }
            Value::Array(entries) => {
                for child in entries {
                    Self::canonicalize(child);
                }
            }
            _ => (),
        }
    }
}
impl ScenarioSession {
    /// Stable JSON bytes for validated model/options/initial state and ordered events.
    pub fn export_log(&self) -> Result<String, Refusal> {
        serde_json::to_string(&self.log).map_err(encoding_error)
    }
}
