"""Capture three real Gazebo camera views of a canonical SDF.

Requires the host's existing Gazebo and Pillow installations. The capture-only
SDF adds sensors, preserving every original world element and model unchanged.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import platform
import subprocess
import time
import xml.etree.ElementTree as ET

from validate_adversarial_grid_gazebo import run


def capture(source, output):
    from PIL import Image

    started = time.monotonic()
    deadline = started + 80  # Reserve ten seconds for owned-process cleanup.
    tree = ET.parse(source)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("SDF contains no world")
    original = [ET.tostring(child) for child in world]
    models = world.findall("model")
    positions = [list(map(float, model.findtext("pose", "").split()))
                 for model in models if model.attrib["name"] != "ground_plane"]
    if not positions or any(len(pose) != 6 for pose in positions):
        raise ValueError("Expected positioned grid obstacle models")
    cx = (min(p[0] for p in positions) + max(p[0] for p in positions)) / 2
    cy = (min(p[1] for p in positions) + max(p[1] for p in positions)) / 2
    span = max(max(p[i] for p in positions) - min(p[i] for p in positions) + 1
               for i in (0, 1))
    if world.find(".//sensor") is not None:
        raise ValueError("Capture source must not already contain sensors")
    plugin = ET.SubElement(world, "plugin", filename="gz-sim-sensors-system",
                           name="gz::sim::systems::Sensors")
    ET.SubElement(plugin, "render_engine").text = "ogre2"
    views = {}
    for name, offset in {"overview": (1.22, -1.41, 1.50),
                         "top": (0, 0, 1.69),
                         "oblique": (-1.34, -1.47, 0.81)}.items():
        eye = [cx + span * offset[0], cy + span * offset[1], span * offset[2]]
        dx, dy = cx - eye[0], cy - eye[1]
        pose = eye + [0, math.atan2(eye[2], math.hypot(dx, dy)), math.atan2(dy, dx)]
        topic = f"/hymeko/capture/{name}"
        model = ET.SubElement(world, "model", name=f"capture_{name}")
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = " ".join(map(str, pose))
        link = ET.SubElement(model, "link", name="camera_link")
        sensor = ET.SubElement(link, "sensor", name="camera", type="camera")
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = "2"
        ET.SubElement(sensor, "topic").text = topic
        camera = ET.SubElement(sensor, "camera")
        ET.SubElement(camera, "horizontal_fov").text = str(math.pi / 3)
        image = ET.SubElement(camera, "image")
        ET.SubElement(image, "width").text = "1280"
        ET.SubElement(image, "height").text = "960"
        ET.SubElement(image, "format").text = "R8G8B8"
        clip = ET.SubElement(camera, "clip")
        ET.SubElement(clip, "near").text = "0.1"
        ET.SubElement(clip, "far").text = str(max(100, 5 * span))
        views[name] = {"pose": pose, "topic": topic, "horizontal_fov": math.pi / 3}
    if [ET.tostring(child) for child in list(world)[:len(original)]] != original:
        raise ValueError("Capture setup changed an original world element")
    output.mkdir(parents=True, exist_ok=True)
    capture_world = output / "capture.world.sdf"
    if capture_world.resolve() == source.resolve():
        raise ValueError("Capture output must not replace the source SDF")
    tree.write(capture_world, encoding="utf-8", xml_declaration=True)
    partition = f"hymeko-grid-capture-{os.getpid()}"
    env = dict(os.environ, GZ_PARTITION=partition, GZ_IP="127.0.0.1")

    def bounded(command, destination):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("Capture exhausted its 80-second command budget")
        return run(command, output / destination, env, timeout=min(25, remaining))

    bounded(["gz", "sdf", "--check", str(capture_world)], "sdf-check.txt")
    command = ["gz", "sim", "-s", "-r", "--headless-rendering", "--iterations", "60000",
               "-v", "4", str(capture_world)]
    evidence = {"platform": platform.platform(), "source": str(source),
                "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                "capture_sdf_sha256": hashlib.sha256(capture_world.read_bytes()).hexdigest(),
                "original_world_elements_unchanged": True,
                "observed_source_obstacles": len(positions), "partition": partition,
                "command": command, "views": views,
                "lighting": "Original SDF sun/materials; no scene, lighting or material overrides",
                "scope": "Gazebo camera sensor pixels; static world, no robot/controller or UI capture"}
    with (output / "server.log").open("w", encoding="utf-8") as server_log:
        process = subprocess.Popen(command, stdout=server_log, stderr=subprocess.STDOUT,
                                   env=env, start_new_session=True)
        evidence["pid"] = process.pid
        (output / "process.json").write_text(json.dumps(evidence, indent=2), encoding="utf-8")
        try:
            for name, view in views.items():
                raw = bounded(["gz", "topic", "--echo", "--json-output", "--num", "1",
                               "--topic", view["topic"]], f"{name}.image.json")
                message = json.loads(raw)
                width, height, step = (int(message[key]) for key in ("width", "height", "step"))
                if (width, height, step, message.get("pixelFormatType")) != (1280, 960, 3840, "RGB_INT8"):
                    raise ValueError(f"Unexpected Gazebo image format: {name}")
                pixels = base64.b64decode(message["data"], validate=True)
                if len(pixels) != step * height:
                    raise ValueError(f"Incomplete Gazebo image: {name}")
                picture = Image.frombytes("RGB", (width, height), pixels)
                extrema = picture.getextrema()
                if all(low == high for low, high in extrema):
                    raise ValueError(f"Uniform frame from camera {name}")
                destination = output / f"{name}.png"
                picture.save(destination)
                view.update({"png": destination.name, "sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
                             "width": width, "height": height, "header": message.get("header"),
                             "channel_extrema": extrema})
            bounded(["gz", "topic", "--echo", "--json-output", "--num", "1", "--topic",
                     f"/world/{world.attrib['name']}/stats"], "stats.json")
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            evidence["exit_code"] = process.returncode
            evidence["wall_seconds"] = time.monotonic() - started
            (output / "capture.json").write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
    return evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    print(json.dumps(capture(args.source.resolve(), args.output.resolve()), indent=2))


if __name__ == "__main__":
    main()
