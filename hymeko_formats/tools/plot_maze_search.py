"""Plot existing Rust search receipts; contains no graph or geometry solver.

Requires the experiment host's matplotlib and Pillow. Called by --benchmark-mazes.
"""
from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path


def draw_map(axis, geometry):
    from matplotlib.patches import Polygon

    for wall in geometry["walls"]:
        x, y, _ = wall["centre_m"]
        width, height, _ = wall["full_size_m"]
        angle = wall["yaw_rad"]
        c, s = math.cos(angle), math.sin(angle)
        corners = [(x + c * u - s * v, y + s * u + c * v)
                   for u, v in [(-width / 2, -height / 2), (width / 2, -height / 2),
                                (width / 2, height / 2), (-width / 2, height / 2)]]
        axis.add_patch(Polygon(corners, color="#34495e"))
    cells = [cell["centre_m"] for cell in geometry["cells"]]
    for a, b in geometry["passages"]:
        axis.plot([cells[a][0], cells[b][0]], [cells[a][1], cells[b][1]],
                  color="#b9c8d0", linewidth=0.7, zorder=0)
    axis.scatter(*zip(*cells), color="#5685a3", s=9)
    axis.autoscale_view()
    axis.set_aspect("equal")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")


def render(root: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation

    data = json.loads((root / "results.json").read_text())
    tasks = data["tasks"]
    summary = []
    figure, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    groups = sorted({(t["profile"], t["loops"]) for t in tasks})
    algorithms = ["bfs", "dijkstra", "astar"]
    for index, algorithm in enumerate(algorithms):
        times, expansions = [], []
        for profile, loops in groups:
            selected = [r for t in tasks if (t["profile"], t["loops"]) == (profile, loops)
                        for r in t["runs"] if r["algorithm"] == algorithm]
            medians = [statistics.median(r["samples_ns"]) / 1000 for r in selected]
            counts = [r["counters"]["expanded_nodes"] for r in selected if r["counters"]]
            times.append(statistics.median(medians))
            expansions.append(statistics.mean(counts) if counts else 0)
            summary.append({"profile": profile, "loops": loops, "algorithm": algorithm,
                            "tasks": len(selected), "median_warm_us": times[-1],
                            "mean_expanded_nodes": expansions[-1] if counts else None,
                            "median_hops": statistics.median(r["cost"] for r in selected)})
        offsets = [i + (index - 1) * 0.25 for i in range(len(groups))]
        axes[0].bar(offsets, times, width=0.24, label=algorithm)
        if algorithm != "bfs":
            axes[1].bar(offsets, expansions, width=0.24, label=algorithm)
    for axis in axes:
        axis.set_xticks(range(len(groups)), [f"{p}\n{n} loops" for p, n in groups])
        axis.legend()
    axes[0].set_ylabel("Median of per-task median warm search [us]")
    axes[1].set_ylabel("Mean expanded nodes; BFS counters unavailable")
    figure.suptitle("Minimum-passage paths on small synthetic maps (12 / 19 cells)")
    for extension in ["png", "svg"]:
        figure.savefig(root / f"comparison.{extension}", dpi=150)
    plt.close(figure)
    (root / "summary.json").write_text(json.dumps(summary, indent=2))

    figure, axes = plt.subplots(2, 3, figsize=(12, 8), constrained_layout=True)
    for axis, (profile, loops) in zip(axes.flat, groups):
        geometry = json.loads((root / f"{profile}-seed0-loops{loops}" / "geometry.json").read_text())
        draw_map(axis, geometry)
        axis.set_title(f"{profile}: {loops} loops, seed 0")
    figure.suptitle("Wall geometry and topological passage chords")
    for extension in ["png", "svg"]:
        figure.savefig(root / f"maps.{extension}", dpi=150)
    plt.close(figure)

    figure, axes = plt.subplots(1, 2, figsize=(12, 7.6), constrained_layout=True)
    tracks = []
    for axis, profile in zip(axes, ["hex", "polar"]):
        candidates = [t for t in tasks if t["profile"] == profile and t["seed"] == 0 and t["loops"] == 3]
        task = max(candidates, key=lambda t: t["runs"][2]["cost"])
        path = task["runs"][2]["route_points_m"]
        geometry = json.loads((root / task["map"] / "geometry.json").read_text())
        draw_map(axis, geometry)
        axis.plot(*zip(*path), color="#de861f", linewidth=2)
        marker, = axis.plot([], [], "o", color="#e23333", markersize=10)
        axis.set_title(f'{profile}: A* witness, {task["runs"][2]["cost"]} passages')
        tracks.append((marker, path))
    figure.suptitle("Geometrically checked path witnesses; no robot dynamics")

    def frame(index):
        for marker, path in tracks:
            progress = index / 59 * (len(path) - 1)
            segment = min(int(progress), len(path) - 2)
            fraction = progress - segment
            a, b = path[segment], path[segment + 1]
            marker.set_data([a[0] + fraction * (b[0] - a[0])], [a[1] + fraction * (b[1] - a[1])])
        return [marker for marker, _ in tracks]

    movie = animation.FuncAnimation(figure, frame, frames=60, interval=80, blit=False)
    movie.save(root / "paths.gif", writer=animation.PillowWriter(fps=12), dpi=100)
    plt.close(figure)


if __name__ == "__main__":
    render(Path(sys.argv[1]))
