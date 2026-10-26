//! Thin source-to-consumer runner; it never plans or commands a robot.
//! `cargo run -p hymeko_dynamics --example inspection -- MODEL.hymeko ROOT [--cbor OUTPUT]`
//! stdin: one JSON operation per line (`observe`, `tick`, `resume`, `history`).
use hymeko::{
    module_store::{execution::CompiledModel, source_provider::StdFsProvider},
    util::real_parser::RealParser,
};
use hymeko_dynamics::scenario::{
    Refusal, Snapshot,
    inspection::{InspectionSession, InspectionSnapshot, PoseObservation},
};
use serde::{Deserialize, Serialize};
use std::{
    io::{BufRead, Read, Write},
    path::{Path, PathBuf},
    process::ExitCode,
};

const MAX_LINE: usize = 65_536;

#[derive(Deserialize)]
#[serde(tag = "operation", rename_all = "snake_case", deny_unknown_fields)]
enum Operation {
    Observe {
        sample: PoseObservation,
        now_ns: u64,
    },
    Tick {
        now_ns: u64,
    },
    Resume {
        now_ns: u64,
    },
    History {
        since: Option<u32>,
    },
}
#[derive(Serialize)]
struct Reply {
    state: InspectionSnapshot,
    error: Option<Refusal>,
    #[serde(skip_serializing_if = "Option::is_none")]
    transitions: Option<Snapshot>,
}

struct Application {
    session: InspectionSession,
    model: CompiledModel,
    workspace: PathBuf,
}
impl Application {
    fn open(path: &Path, root: &str) -> Result<Self, String> {
        let model = CompiledModel::compile(StdFsProvider::new(), RealParser, path)
            .map_err(|e| format!("{e:?}"))?;
        let matches: Vec<_> = model
            .program()
            .ir
            .decl_nodes
            .iter()
            .enumerate()
            .filter(|(_, node)| {
                node.parent.is_none() && model.interner().resolve(node.name) == root
            })
            .map(|(id, _)| id)
            .collect();
        let [root] = matches.as_slice() else {
            return Err("one named top-level task root required".into());
        };
        Ok(Self {
            session: InspectionSession::open(&model.program().ir, model.interner(), *root)
                .map_err(|e| e.to_string())?,
            model,
            workspace: path
                .canonicalize()
                .map_err(|e| e.to_string())?
                .parent()
                .ok_or("model has no parent")?
                .to_owned(),
        })
    }
    fn encode(&self) -> Result<Vec<u8>, String> {
        let model = self
            .model
            .with_value_updates(&self.session.value_updates())
            .map_err(|e| e.to_string())?;
        hymeko::writers::cbor_writer::encode_compiled(
            &self.workspace,
            model.program(),
            model.interner(),
        )
        .map_err(|e| e.to_string())
    }
    fn dispatch(&mut self, operation: Operation) -> Reply {
        let mut transitions = None;
        let result = match operation {
            Operation::Observe { sample, now_ns } => self.session.observe(sample, now_ns),
            Operation::Tick { now_ns } => self.session.tick(now_ns),
            Operation::Resume { now_ns } => self.session.resume(now_ns),
            Operation::History { since } => self.session.transitions(since).map(|snapshot| {
                transitions = Some(snapshot);
                self.session.snapshot()
            }),
        };
        Reply {
            state: self.session.snapshot(),
            error: result.err(),
            transitions,
        }
    }
    fn write(output: &mut impl Write, reply: &Reply) -> Result<(), String> {
        serde_json::to_writer(&mut *output, reply).map_err(|e| e.to_string())?;
        output
            .write_all(b"\n")
            .and_then(|_| output.flush())
            .map_err(|e| e.to_string())
    }
    fn run(&mut self, mut input: impl BufRead, mut output: impl Write) -> Result<(), String> {
        Self::write(
            &mut output,
            &Reply {
                state: self.session.snapshot(),
                error: None,
                transitions: None,
            },
        )?;
        loop {
            let mut line = Vec::new();
            let count = (&mut input)
                .take((MAX_LINE + 1) as u64)
                .read_until(b'\n', &mut line)
                .map_err(|e| e.to_string())?;
            if count == 0 {
                return Ok(());
            }
            if count > MAX_LINE {
                return Err("inspection input exceeds 64 KiB".into());
            }
            let operation = serde_json::from_slice(&line)
                .map_err(|e| format!("invalid inspection operation: {e}"))?;
            Self::write(&mut output, &self.dispatch(operation))?;
        }
    }
}

fn main() -> ExitCode {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let selection = match args.as_slice() {
        [path, root] => Ok((path, root, None)),
        [path, root, flag, output] if flag == "--cbor" => Ok((path, root, Some(output))),
        _ => Err(
            "usage: inspection MODEL.hymeko ROOT [--cbor OUTPUT]; reads JSON operations from stdin"
                .to_owned(),
        ),
    };
    let result = selection.and_then(|(path, root, output)| {
        let mut app = Application::open(Path::new(path), root)?;
        app.run(std::io::stdin().lock(), std::io::stdout().lock())?;
        if let Some(output) = output {
            std::fs::write(output, app.encode()?).map_err(|e| e.to_string())?;
        }
        Ok(())
    });
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    struct Fixture;
    impl Fixture {
        fn path() -> std::path::PathBuf {
            Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/inspection/task.hymeko")
        }
        fn app() -> Application {
            Application::open(&Self::path(), "inspection").unwrap()
        }
        fn sample(app: &Application) -> PoseObservation {
            PoseObservation {
                source: "gazebo-ur5".into(),
                epoch: "test".into(),
                frame: "world".into(),
                sequence: 1,
                acquired_ns: 1,
                pose: app.session.target().unwrap().pose,
            }
        }
    }
    #[test]
    fn runner_compiles_source_and_exposes_refusals_without_mutating_progress() {
        let mut app = Fixture::app();
        let sample = Fixture::sample(&app);
        let operations = [
            serde_json::json!({"operation":"observe","sample":sample,"now_ns":1}),
            serde_json::json!({"operation":"tick","now_ns":600_000_001u64}),
            serde_json::json!({"operation":"resume","now_ns":600_000_001u64}),
            serde_json::json!({"operation":"history","since":0}),
        ];
        let input = operations
            .iter()
            .map(|op| format!("{op}\n"))
            .collect::<String>();
        let mut output = Vec::new();
        app.run(std::io::Cursor::new(input), &mut output).unwrap();
        let rows: Vec<serde_json::Value> = output
            .split(|b| *b == b'\n')
            .filter(|b| !b.is_empty())
            .map(|b| serde_json::from_slice(b).unwrap())
            .collect();
        assert_eq!(rows.len(), 5);
        assert_eq!(rows[2]["state"]["phase"], "blocked");
        assert!(!rows[3]["error"].is_null());
        assert_eq!(
            rows[4]["transitions"]["events"].as_array().unwrap().len(),
            1
        );
        assert_eq!(rows[4]["state"]["completedVisits"], 0);
    }
    #[test]
    fn final_cbor_contains_the_same_canonical_output_values() {
        use hymeko::{ir::ir::ValueR, ir_stream::DecodeLimits, writers::cbor_writer};
        let mut app = Fixture::app();
        let sample = Fixture::sample(&app);
        assert!(
            app.dispatch(Operation::Observe { sample, now_ns: 1 })
                .error
                .is_none()
        );
        let model = CompiledModel::from_decoded_cbor(
            cbor_writer::decode(&app.encode().unwrap(), &DecodeLimits::default()).unwrap(),
        );
        let fresh = model
            .program()
            .ir
            .decl_nodes
            .iter()
            .find(|n| model.interner().resolve(n.name) == "fresh")
            .unwrap();
        assert!(matches!(fresh.anno.value, Some(ValueR::Num(1.))));
        assert_eq!(
            model.program().ir.decl_nodes.len(),
            app.model.program().ir.decl_nodes.len()
        );
    }
    #[test]
    fn bounded_transport_and_wrong_roots_fail_explicitly() {
        assert!(Application::open(&Fixture::path(), "missing").is_err());
        assert!(
            Fixture::app()
                .run(std::io::Cursor::new(vec![b' '; MAX_LINE + 1]), Vec::new())
                .is_err()
        );
        assert!(
            Fixture::app()
                .run(
                    std::io::Cursor::new(b"{\"operation\":\"unknown\"}\n"),
                    Vec::new()
                )
                .is_err()
        );
    }
}
