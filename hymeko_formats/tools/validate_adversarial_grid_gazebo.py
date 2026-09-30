"""Finite, isolated Gazebo validation of an exported campaign world on Amaterasu."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET


def run(command, destination, env, timeout=20):
    result = subprocess.run(command, capture_output=True, text=True, env=env, timeout=timeout)
    destination.write_text(result.stdout, encoding="utf-8")
    destination.with_suffix(destination.suffix + ".stderr").write_text(result.stderr, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(f"Command failed ({result.returncode}): {command!r}; see {destination}")
    return result.stdout


def topic(name, destination, env):
    text = run(["gz", "topic", "--echo", "--json-output", "--num", "1", "--topic", name], destination, env)
    return json.loads(text)


def simulation_seconds(stats):
    stamp = stats.get("simTime", stats.get("sim_time", {}))
    return float(stamp.get("sec", 0)) + float(stamp.get("nsec", 0)) * 1e-9


def validate(output):
    if socket.gethostname().lower() != "amaterasu":
        raise RuntimeError("Gazebo validation is restricted to Amaterasu")
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    sample = next((row["selected"] for row in summary["seeds"] if row.get("selected")), None)
    if sample is None:
        return {"status": "skipped", "reason": "no feasible generated world"}
    world_file = output / sample["world_path"]
    world = ET.parse(world_file).getroot().find("world")
    if world is None:
        raise ValueError("SDF contains no world")
    world_name = world.attrib["name"]
    expected = {}
    for model in world.findall("model"):
        if model.attrib["name"] == "ground_plane":
            continue
        pose = [float(value) for value in model.findtext("pose", "").split()]
        if len(pose) != 6 or model.findtext("static") != "true":
            raise ValueError("Expected a static obstacle with an explicit pose")
        for tag in ("collision", "visual"):
            size = model.findtext(f"link/{tag}/geometry/box/size", "")
            if [float(value) for value in size.split()] != [1.0, 1.0, 1.0]:
                raise ValueError("Grid obstacle box is inconsistent with unit cell geometry")
        expected[model.attrib["name"]] = pose[:3]
    authored_walls = sorted((p["x"], p["y"], 0.5) for p in sample["layout"]["walls"])
    if sorted(tuple(pose) for pose in expected.values()) != authored_walls:
        raise ValueError("SDF obstacle poses disagree with canonical projected grid walls")
    if len(expected) != sample["wall_count"]:
        raise ValueError("SDF obstacle count differs from the scored grid")

    logs = output / "gazebo"
    logs.mkdir(exist_ok=True)
    partition = f"hymeko-adversarial-{os.getpid()}"
    env = dict(os.environ, GZ_PARTITION=partition, GZ_IP="127.0.0.1")
    run(["gz", "sdf", "--check", str(world_file)], logs / "sdf-check.txt", env)
    command = ["gz", "sim", "-s", "-r", "--iterations", "10000", "-v", "3", str(world_file)]
    started = time.monotonic()
    with (logs / "server.log").open("w", encoding="utf-8") as server_log:
        process = subprocess.Popen(command, stdout=server_log, stderr=subprocess.STDOUT,
                                   env=env, start_new_session=True)
        (logs / "process.json").write_text(json.dumps({"pid": process.pid, "partition": partition,
                                                     "command": command}, indent=2), encoding="utf-8")
        try:
            before = topic(f"/world/{world_name}/stats", logs / "stats-before.json", env)
            poses = topic(f"/world/{world_name}/pose/info", logs / "poses.json", env)
            after = topic(f"/world/{world_name}/stats", logs / "stats-after.json", env)
            if simulation_seconds(after) <= simulation_seconds(before):
                raise ValueError("Simulation clock did not advance between observations")
            by_name = {pose.get("name"): pose for pose in poses.get("pose", [])}
            for name, expected_position in expected.items():
                if name not in by_name:
                    raise ValueError(f"Gazebo did not publish obstacle {name}")
                actual = by_name[name].get("position", {})
                if not all(math.isclose(float(actual.get(axis, 0)), value, abs_tol=1e-8)
                           for axis, value in zip(("x", "y", "z"), expected_position)):
                    raise ValueError(f"Gazebo obstacle pose differs from canonical source: {name}")
            # --iterations is the natural stop; enforce a wall-clock ceiling as well.
            code = process.wait(timeout=max(1, 60 - (time.monotonic() - started)))
            if code:
                raise RuntimeError(f"Gazebo exited with {code}")
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
    log_text = (logs / "server.log").read_text(encoding="utf-8")
    if "[err]" in log_text.lower() or "[error]" in log_text.lower():
        raise ValueError("Gazebo reported an error; inspect server.log")
    return {"status": "passed", "hostname": socket.gethostname(), "world_path": sample["world_path"],
            "partition": partition, "observed_obstacles": len(expected), "iteration_limit": 10000,
            "observed_sim_seconds_before": simulation_seconds(before),
            "observed_sim_seconds_after": simulation_seconds(after),
            "exit_code": code, "wall_seconds": time.monotonic() - started,
            "scope": "Static obstacles loaded at canonical poses; clock advanced. No robot/controller tested."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    result_file = args.output / "gazebo-validation.json"
    try:
        result = validate(args.output.resolve())
    except Exception as exc:
        result_file.write_text(json.dumps({"status": "failed", "error": str(exc)}, indent=2) + "\n", encoding="utf-8")
        raise
    result_file.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
