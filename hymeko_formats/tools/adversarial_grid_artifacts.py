"""Plot canonical campaign records; animate a grid path witness, not robot physics.

Run after adversarial_grid: python3 .../adversarial_grid_artifacts.py OUTPUT
Plotting requires the host's existing matplotlib and Pillow installations.
An all-infeasible campaign still produces a numeric summary without plotting.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def summarize_pairs(rows):
    """Keep infeasible seeds in the denominator; means condition on valid pairs."""
    seeds = [row for row in rows if row.get("baseline") and row.get("selected")]
    paired = {
        "total_seeds": len(rows),
        "seeds_with_feasible_pairs": len(seeds),
        "seeds_without_feasible_pairs": len(rows) - len(seeds),
        "seeds_with_strict_astar_increase": sum(
            row["selected"]["astar"]["expanded_nodes"] > row["baseline"]["astar"]["expanded_nodes"]
            for row in seeds
        ),
        "illustrated_seed": seeds[0]["seed"] if seeds else None,
        "figures_status": "pending" if seeds else "skipped_no_feasible_pairs",
        "interpretation": "Means condition on feasible pairs. Best-of-budget selection for one fixed search configuration; not universal hardness.",
    }
    for search in ("astar", "zero"):
        for arm in ("baseline", "selected"):
            values = [row[arm][search]["expanded_nodes"] for row in seeds]
            paired[f"mean_{search}_{arm}_expansions"] = sum(values) / len(values) if values else None
    return seeds, paired


def draw_layout(ax, sample, title, path_prefix=None):
    from matplotlib.patches import Rectangle

    layout = sample["layout"]
    width, height = layout["width"], layout["height"]
    ax.set(xlim=(-0.5, width - 0.5), ylim=(height - 0.5, -0.5), aspect="equal")
    ax.set_facecolor("#f5f6f8")
    for wall in layout["walls"]:
        ax.add_patch(Rectangle((wall["x"] - 0.5, wall["y"] - 0.5), 1, 1, color="#34495e"))
    path = sample["path"] if path_prefix is None else sample["path"][:path_prefix]
    if path:
        ax.plot([p["x"] for p in path], [p["y"] for p in path], color="#e09125", lw=2.5)
        if path_prefix is not None:
            ax.scatter(path[-1]["x"], path[-1]["y"], color="#e09125", s=110, zorder=6)
    for key, color, marker in [("start", "#16875d", "o"), ("goal", "#d64747", "*")]:
        point = layout[key]
        ax.scatter(point["x"], point["y"], color=color, marker=marker, s=135, zorder=5)
    ax.set_title(title, fontsize=12)
    ax.set_xticks(range(0, width, max(1, width // 8)))
    ax.set_yticks(range(0, height, max(1, height // 8)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    summary = json.loads((args.output / "summary.json").read_text(encoding="utf-8"))
    seeds, paired = summarize_pairs(summary["seeds"])
    summary_path = args.output / "paired_summary.json"
    summary_path.write_text(json.dumps(paired, indent=2) + "\n", encoding="utf-8")
    if not seeds:
        print(json.dumps(paired, indent=2))
        return

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    figures = args.output / "figures"
    figures.mkdir(exist_ok=True)
    labels = [str(row["seed"]) for row in seeds]
    before = [row["baseline"]["astar"]["expanded_nodes"] for row in seeds]
    after = [row["selected"]["astar"]["expanded_nodes"] for row in seeds]
    before_zero = [row["baseline"]["zero"]["expanded_nodes"] for row in seeds]
    after_zero = [row["selected"]["zero"]["expanded_nodes"] for row in seeds]
    fig, axes = plt.subplots(1, 2, figsize=(12.8, 5), constrained_layout=True)
    for ax, a, b, label in zip(axes, [before, before_zero], [after, after_zero],
                              ["Manhattan A* (selection criterion)", "Zero heuristic (secondary measurement)"]):
        ax.bar([i - 0.18 for i in range(len(seeds))], a, width=0.36, label="First feasible", color="#4b89a8")
        ax.bar([i + 0.18 for i in range(len(seeds))], b, width=0.36, label="Selected", color="#e09125")
        ax.set_xticks(range(len(seeds)), labels)
        ax.set(xlabel="Generation seed", ylabel="Expanded nodes", title=label)
        ax.legend()
    fig.suptitle("Matched grid size and wall count; selection cost reported separately")
    fig.savefig(figures / "search_work.png", dpi=150)
    fig.savefig(figures / "search_work.svg")
    plt.close(fig)

    # The first feasible seed is chosen before looking at the size of the effect.
    first = seeds[0]
    fig, axes = plt.subplots(1, 2, figsize=(12.8, 7.2), constrained_layout=True)
    for ax, key, label in zip(axes, ["baseline", "selected"], ["First feasible", "Selected"]):
        sample = first[key]
        draw_layout(ax, sample, f"{label}: {sample['astar']['expanded_nodes']} A* expansions")
    fig.suptitle(f"Seed {first['seed']} | yellow: BFS-checked A* shortest-path witness")
    fig.savefig(figures / "layouts.png", dpi=120)
    plt.close(fig)

    sample = first["selected"]
    fig, ax = plt.subplots(figsize=(12.8, 7.2), dpi=100)
    frames = []
    # Every visited cell in the returned witness is shown; there is no dynamics integration.
    for count in range(1, len(sample["path"]) + 1):
        ax.clear()
        draw_layout(ax, sample, f"Selected grid, seed {first['seed']} | path witness {count}/{len(sample['path'])}", count)
        fig.tight_layout()
        fig.canvas.draw()
        frame = Image.frombuffer("RGBA", fig.canvas.get_width_height(), fig.canvas.buffer_rgba(), "raw", "RGBA", 0, 1)
        frames.append(frame.convert("RGB"))
    plt.close(fig)
    frames[0].save(figures / "path_witness.gif", save_all=True, append_images=frames[1:],
                   duration=110, loop=0)
    paired["figures_status"] = "generated"
    summary_path.write_text(json.dumps(paired, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(paired, indent=2))


if __name__ == "__main__":
    main()
