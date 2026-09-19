"""
The four Lab 4 figures, generated from the planners rather than drawn by hand.

    python3 figures.py out/

Every number and every shaded cell in these comes from an actual run of the
code in planners.py. Nothing is illustrative. If you change a map or a
heuristic, re-run this and the figures change with it, which is the only way a
figure in a lab manual stays true after the third revision.

The four:

    fig_search_shape.png    what each algorithm's expanded set LOOKS like. The
                            single most useful picture in the lab: Dijkstra's
                            disc against A*'s beam, on the same map.
    fig_greedy_trap.png     the dive into the box and the climb back out.
    fig_cost_vs_effort.png  every planner as a point: cells expanded against
                            path length. The trade-off, plotted.
    fig_steps_vs_cost.png   BFS and Dijkstra side by side on the map where
                            fewer steps means more distance.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

from gridmap import GOAL, MAPS, MOVES_8, START

# Consistent across all four figures, so a reader can compare panels.
C_FREE = "#ffffff"
C_WALL = "#33373d"
C_SEEN = "#cfe0f3"
C_PATH = "#d1495b"
C_START = "#2a9d8f"
C_GOAL = "#e9c46a"


def _draw(ax, grid, result=None, title="", shade_expanded=True):
    """One map panel: walls, expanded cells, the path, start and goal.

    `shade_expanded` is off in the figures where the point is the shape of the
    PATH. On a map that a planner explores almost completely, the shading fills
    the panel and hides the very thing the figure is about.
    """
    canvas = np.zeros(grid.shape, dtype=int)
    if result is not None and shade_expanded:
        for cell in result.expanded_cells:
            canvas[cell] = 1
    canvas[grid] = 2

    ax.imshow(canvas, cmap=ListedColormap([C_FREE, C_SEEN, C_WALL]),
              interpolation="nearest", vmin=0, vmax=2)

    if result is not None and result.found:
        rows = [c[0] for c in result.path]
        cols = [c[1] for c in result.path]
        ax.plot(cols, rows, color=C_PATH, linewidth=2.0, solid_capstyle="round")

    ax.plot(START[1], START[0], "o", color=C_START, markersize=7,
            markeredgecolor="white", markeredgewidth=1.0, zorder=5)
    ax.plot(GOAL[1], GOAL[0], "s", color=C_GOAL, markersize=7,
            markeredgecolor="white", markeredgewidth=1.0, zorder=5)
    ax.set_title(title, fontsize=9)
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)


def _label(result):
    if not result.found:
        return f"no path\n{result.expanded} cells expanded"
    return f"length {result.length:.2f}  |  {result.expanded} cells expanded"


def fig_search_shape(planners, out: Path, moves=MOVES_8):
    """Dijkstra's disc against A*'s beam, on the same two maps.

    The left column is what "no idea where the goal is" looks like. The right
    columns are what a heuristic buys you, and the second map shows how much of
    that advantage walls take back.
    """
    labels = ["Dijkstra", "A* euclidean", "A* octile"]
    maps = ["empty_room", "corridors"]
    fig, axes = plt.subplots(len(maps), len(labels), figsize=(9.5, 6.6))

    for row, map_name in enumerate(maps):
        grid = MAPS[map_name]()
        for col, label in enumerate(labels):
            result = planners.PLANNERS[label](grid, START, GOAL, moves)
            _draw(axes[row, col], grid, result,
                  f"{label}\n{_label(result)}")
        axes[row, 0].set_ylabel(map_name, fontsize=10)

    fig.suptitle("The same shortest path, found three ways.\n"
                 "All six panels return the identical distance; "
                 "what differs is how much of the room each one had to look at.",
                 fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    path = out / "fig_search_shape.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def fig_greedy_trap(planners, out: Path, moves=MOVES_8):
    """The box, and three planners meeting it."""
    grid = MAPS["greedy_trap"]()
    labels = ["A* octile", "A* inadmissible", "Greedy best first"]
    fig, axes = plt.subplots(1, 3, figsize=(9.5, 3.8))

    best = planners.PLANNERS["A* octile"](grid, START, GOAL, moves).length
    for ax, label in zip(axes, labels):
        result = planners.PLANNERS[label](grid, START, GOAL, moves)
        excess = 100.0 * (result.length - best) / best
        tag = "shortest" if excess < 1e-6 else f"{excess:.0f}% longer"
        _draw(ax, grid, result, f"{label}\n{_label(result)}\n{tag}")

    fig.suptitle("Every cell inside the box is nearer the goal than the cells "
                 "outside it.\nA search that orders on the heuristic alone "
                 "goes in, and has to come back out.", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    path = out / "fig_greedy_trap.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def fig_cost_vs_effort(planners, out: Path, moves=MOVES_8):
    """The trade-off as a scatter: work done against quality of answer.

    The optimal planners form a flat line along the bottom, differing only in
    how far right they sit. Anything above that line bought its speed with
    distance.
    """
    maps = ["empty_room", "corridors", "greedy_trap", "cluttered", "steps_vs_cost"]

    # DFS is left out. It is 600 percent above optimal on `corridors`, which
    # would compress every other planner onto the zero line and hide the whole
    # point of the figure. Its lesson is in the table, where it belongs.
    shown = [lbl for lbl in planners.PLANNERS if lbl != "DFS"]
    markers = ["o", "s", "^", "D", "v", "P", "X", "*"]
    style = {lbl: markers[i % len(markers)] for i, lbl in enumerate(shown)}

    # Two rows rather than one. Five panels in a strip leaves each one about an
    # inch tall, and a scatter that short cannot show a vertical difference.
    fig, grid_axes = plt.subplots(2, 3, figsize=(10.5, 6.4), sharey=True)
    axes = grid_axes.ravel()
    axes[-1].axis("off")

    for ax, map_name in zip(axes, maps):
        grid = MAPS[map_name]()
        found = {}
        for label in shown:
            r = planners.PLANNERS[label](grid, START, GOAL, moves)
            if r.found:
                found[label] = r
        best = min(v.length for v in found.values())

        for label, r in found.items():
            excess = 100.0 * (r.length - best) / best
            ax.scatter(r.expanded, excess, s=52, marker=style[label],
                       color=C_START if excess < 1e-6 else C_PATH,
                       edgecolor="white", linewidth=0.8, zorder=3,
                       label=label if ax is axes[0] else None)

        ax.axhline(0.0, color=C_START, linewidth=1.0, linestyle="--", zorder=1)
        ax.set_title(map_name, fontsize=9)
        ax.set_xlabel("cells expanded", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(alpha=0.25, zorder=0)

    axes[0].set_ylabel("path length above optimal (%)", fontsize=8)
    axes[0].set_ylim(-2.0, 36.0)
    handles, labels = axes[0].get_legend_handles_labels()
    grid_axes[1, 0].set_ylabel("path length above optimal (%)", fontsize=8)
    axes[-1].legend(handles, labels, loc="center", fontsize=8,
                    frameon=False, markerscale=1.1)
    fig.suptitle("Green sits on the dashed line: it returned the shortest path, "
                 "at whatever cost in search effort.\nRed bought its speed with "
                 "distance. Further left is cheaper to run; lower is a better "
                 "answer. DFS is off this chart at 600 percent.", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    path = out / "fig_cost_vs_effort.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def fig_steps_vs_cost(planners, out: Path, moves=MOVES_8):
    """BFS and Dijkstra on the map where the two currencies disagree."""
    grid = MAPS["steps_vs_cost"]()
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 4.0))

    b = planners.PLANNERS["BFS"](grid, START, GOAL, moves)
    d = planners.PLANNERS["Dijkstra"](grid, START, GOAL, moves)

    _draw(axes[0], grid, b, f"BFS\n{b.steps} steps, length {b.length:.2f}\nfewest moves", shade_expanded=False)
    _draw(axes[1], grid, d, f"Dijkstra\n{d.steps} steps, length {d.length:.2f}\nshortest distance", shade_expanded=False)

    fig.suptitle("BFS wins on the thing it optimises and loses on the thing "
                 "you want.\nIt counts a diagonal as one move; the robot has "
                 "to drive sqrt(2) metres.", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.87))
    path = out / "fig_steps_vs_cost.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def write_all(planners, out_dir, moves=MOVES_8):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    return [fig_search_shape(planners, out, moves),
            fig_greedy_trap(planners, out, moves),
            fig_cost_vs_effort(planners, out, moves),
            fig_steps_vs_cost(planners, out, moves)]


if __name__ == "__main__":
    import planners
    target = sys.argv[1] if len(sys.argv) > 1 else "figures_out"
    for p in write_all(planners, target):
        print(p)
