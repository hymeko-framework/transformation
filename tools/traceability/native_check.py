"""Run the official SysML pilot and compare its resolved AST with the full reference."""

from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess

from check import TraceInvariant


class NativeSysml:
    def __init__(self, java: Path, jar: Path, library: Path, output: Path):
        self.java, self.jar, self.library, self.output = java, jar, library, output
        output.mkdir(parents=True, exist_ok=False)
        source = Path(__file__).with_name("NativeSysml.java")
        subprocess.run(
            [
                str(java.with_name("javac.exe" if os.name == "nt" else "javac")),
                "-encoding",
                "UTF-8",
                "-cp",
                str(jar),
                "-d",
                str(output),
                str(source),
            ],
            check=True,
        )

    @staticmethod
    def leaf(name):
        if not isinstance(name, str):
            raise ValueError("unresolved native reference")
        return name.rsplit("::", 1)[-1]

    @classmethod
    def target(cls, name):
        name = cls.leaf(name)
        for prefix in ["hymeko_req_", "hymeko_part_"]:
            if name.startswith(prefix):
                return name[len(prefix) :]
        raise ValueError("unrecognised explicit export binding")

    @classmethod
    def reference(cls, report):
        if not report["accepted"]:
            raise ValueError("native parser/validator rejected the model")
        projection = TraceInvariant()
        attributes = {
            e["name"].rsplit("::", 1)[0]: e.get("recorded_value")
            for e in report["elements"]
            if cls.leaf(e["name"]) == "recorded_outcome"
        }
        kinds = {
            "RequirementDefinition": "requirement",
            "PartDefinition": "component",
            "VerificationCaseDefinition": "test_case",
        }
        for e in report["elements"]:
            name, kind = cls.leaf(e["name"]), e["kind"]
            if kind in kinds:
                text = (
                    e.get("documentation", [None])[0]
                    if kind == "RequirementDefinition"
                    else None
                )
                projection.entity(
                    name,
                    kinds[kind],
                    text.strip() if text else None,
                    attributes.get(e["name"])
                    if kind == "VerificationCaseDefinition"
                    else None,
                )
            elif kind == "SatisfyRequirementUsage":
                projection.relation(
                    name, "satisfies", cls.target(e["by"]), cls.target(e["requirement"])
                )
            elif kind in ["AllocationUsage", "ConnectionUsage"]:
                if len(e["targets"]) != 1:
                    raise ValueError("unsupported native connector arity")
                relation = "allocated_to" if kind == "AllocationUsage" else "derives"
                if relation == "derives":
                    if not name.startswith("hymeko_derive_"):
                        raise ValueError("unknown connection projection")
                    name = name.removeprefix("hymeko_derive_")
                projection.relation(
                    name, relation, cls.target(e["source"]), cls.target(e["targets"][0])
                )
            elif kind == "VerificationCaseUsage":
                if len(e["verifies"]) != 1:
                    raise ValueError("unsupported native verification target")
                projection.relation(
                    name,
                    "verifies",
                    cls.leaf(e["definition"]),
                    cls.target(e["verifies"][0]),
                )
        return projection.record()

    def run(self, source: Path, name: str, expected=None):
        output = self.output / f"{name}.json"
        command = [
            str(self.java),
            "-Xmx2g",
            "-cp",
            str(self.jar) + os.pathsep + str(self.output),
            "NativeSysml",
            str(self.library),
            str(source),
            str(output),
        ]
        result = subprocess.run(command, capture_output=True, timeout=90)
        (self.output / f"{name}.log").write_bytes(result.stdout + result.stderr)
        if result.returncode not in [0, 1] or not output.exists():
            raise RuntimeError(f"native validation did not finish: {result.returncode}")
        report = json.loads(output.read_text("utf-8"))
        equal = None
        if expected is not None and report["accepted"]:
            try:
                equal = self.reference(report) == expected
            except ValueError:
                equal = False
        return {
            "case": name,
            "native_accepted": report["accepted"],
            "reference_equal": equal,
            "issues": report["issues"],
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ["java", "jar", "library", "evidence", "output"]:
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    audit = NativeSysml(
        args.java.resolve(),
        args.jar.resolve(),
        args.library.resolve(),
        args.output.resolve(),
    )
    checks = []
    cases = {
        "unknown": args.evidence / "revision-checked/before/initial",
        "before": args.evidence / "recorded-before/after",
        "after": args.evidence / "recorded-after/after",
    }
    for name, folder in cases.items():
        expected = json.loads((folder / "reference.json").read_text("utf-8"))
        result = audit.run(folder / "trace.sysml", name, expected)
        assert result["native_accepted"] and result["reference_equal"], result
        checks.append(result)
    source = (cases["after"] / "trace.sysml").read_text("utf-8")
    expected = json.loads((cases["after"] / "reference.json").read_text("utf-8"))
    mutations = {
        "syntax": source[:-3],
        "dangling": source.replace("by hymeko_part_Arm;", "by absent_part;", 1),
        "wrong_target": source.replace(
            "by hymeko_part_Arm;", "by hymeko_part_Diagnostics;", 1
        ),
        "wrong_outcome": source.replace(
            "verification def T_cycle {\n"
            "        attribute recorded_outcome : ScalarValues::Integer = 1;",
            "verification def T_cycle {\n"
            "        attribute recorded_outcome : ScalarValues::Integer = 0;",
            1,
        ),
    }
    for name, text in mutations.items():
        path = audit.output / f"{name}.sysml"
        path.write_text(text, encoding="utf-8")
        result = audit.run(path, name, expected)
        assert not result["native_accepted"] or result["reference_equal"] is False, (
            result
        )
        checks.append(result)
    stale = audit.run(cases["before"] / "trace.sysml", "stale", expected)
    assert stale["native_accepted"] and stale["reference_equal"] is False
    checks.append(stale)
    summary = {
        "tool": "SysML pilot 2026-08 / 0.62.0",
        "jar_sha256": hashlib.sha256(args.jar.read_bytes()).hexdigest(),
        "positive_models": 3,
        "faults_detected": 5,
        "checks": checks,
        "scope": (
            "generated flat traceability profile; native resolved AST; "
            "boundary documentation whitespace trimmed; no verification execution"
        ),
    }
    (audit.output / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
