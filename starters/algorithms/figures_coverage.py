"""
The coverage figures.

    python3 figures_coverage.py out/

    fig_coverage_axis.png     the same room swept two ways: 52 turns against 81
    fig_coverage_spacing.png  the trade between turns and floor actually cleaned
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

try:
    from .gridmap import empty_room
except ImportError:  # Standalone exercise.
    from gridmap import empty_room

C_FREE = "#ffffff"
C_WALL = "#33373d"
C_MISS = "#f3c5cc"
C_PATH = "#d1495b"
C_GOOD = "#2a9d8f"
C_MID = "#e9a13b"

RES = 0.05


def living_room():
    grid = empty_room(80, 120)                     # 4.0 m by 6.0 m
    for (r, c, h, w) in [(20, 30, 14, 20), (50, 70, 16, 24), (12, 90, 10, 14)]:
        grid[r:r + h, c:c + w] = True
    return grid


def _swept_mask(grid, path, tool_width):
    h, w = grid.shape
    swept = np.zeros_like(grid, dtype=bool)
    half = max(0, tool_width // 2)
    for (r, c) in path:
        swept[max(0, r - half):min(h, r + half + 1),
              max(0, c - half):min(w, c + half + 1)] = True
    return swept & ~grid


def _draw(ax, grid, result, tool_width, title):
    """Walls dark, missed floor pink, the driven path on top."""
    swept = _swept_mask(grid, result.path, tool_width)
    canvas = np.zeros(grid.shape, dtype=int)
    canvas[~grid & ~swept] = 1                     # free but never cleaned
    canvas[grid] = 2
    ax.imshow(canvas, cmap=ListedColormap([C_FREE, C_MISS, C_WALL]),
              origin="lower", extent=(0, grid.shape[1] * RES, 0,
                                      grid.shape[0] * RES),
              interpolation="nearest")
    xs = [c * RES for (r, c) in result.path]
    ys = [r * RES for (r, c) in result.path]
    ax.plot(xs, ys, color=C_PATH, linewidth=0.7, alpha=0.9)
    ax.set_title(title, fontsize=9)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)", fontsize=8)
    ax.tick_params(labelsize=7)


def fig_coverage_axis(coverage, out: Path, tool: int = 9):
    grid = living_room()
    results = coverage.compare_axes(grid, tool, tool_width=tool)

    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.4))
    for ax, (name, r) in zip(axes, results.items()):
        _draw(ax, grid, r, tool,
              f"sweep along {name}\ncovered {r.covered*100:.1f}%, "
              f"{r.turns} turns, {r.segments} passes")
    axes[0].set_ylabel("y (m)", fontsize=8)

    saved = results["columns"].turns - results["rows"].turns
    fig.suptitle("The same room, the same tool, the same coverage.\n"
                 f"Sweeping along the long axis saves {saved} turns, because "
                 "turns happen at the ENDS of passes\nand longer passes means "
                 "fewer ends.", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.84))
    p = out / "fig_coverage_axis.png"
    fig.savefig(p, dpi=160)
    plt.close(fig)
    return p


def fig_coverage_spacing(coverage, out: Path, tool: int = 9):
    """Widen the gap between passes: fewer turns, and floor left dirty."""
    grid = living_room()
    spacings = [5, 7, 9, 11, 14, 18]
    covered, turns = [], []
    for s in spacings:
        r = coverage.boustrophedon(grid, s, axis=0, tool_width=tool)
        covered.append(r.covered * 100.0)
        turns.append(r.turns)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.5, 4.4),
                                   gridspec_kw={"width_ratios": [1.2, 1.0]})

    x = [s * RES for s in spacings]
    ax1.plot(x, covered, "o-", color=C_GOOD, linewidth=2, label="covered (%)")
    ax1.axvline(tool * RES, color="#555", linestyle=":", linewidth=1.4)
    ax1.annotate(f"tool width\n{tool * RES:.2f} m", (tool * RES, 55),
                 fontsize=8, ha="center")
    ax1.set_xlabel("spacing between passes (m)", fontsize=9)
    ax1.set_ylabel("floor covered (%)", fontsize=9, color=C_GOOD)
    ax1.tick_params(labelsize=8)
    ax1.grid(alpha=0.25)

    twin = ax1.twinx()
    twin.plot(x, turns, "s--", color=C_MID, linewidth=2)
    twin.set_ylabel("turns", fontsize=9, color=C_MID)
    twin.tick_params(labelsize=8)
    ax1.set_title("Wider passes: less work, and dirty floor", fontsize=10)

    ax2.axis("off")
    lines = [f"{'spacing':>9}{'covered':>10}{'turns':>8}", "-" * 27]
    for s, c, t in zip(x, covered, turns):
        mark = "  <- tool width" if abs(s - tool * RES) < 1e-9 else ""
        lines.append(f"{s:8.2f}m{c:9.1f}%{t:8d}{mark}")
    lines += ["",
              "Spacing equal to the tool is the knee.",
              "",
              "Narrower covers no more floor and",
              "nearly doubles the turns: every pass",
              "is re-cleaning what the last one did.",
              "",
              "Wider is where the stripes appear.",
              "The pink floor in the other figure is",
              "exactly this, and the robot has no",
              "idea it happened.",
              "",
              "Real machines overlap by 10 to 20%,",
              "because wheel slip and localisation",
              "error mean passes that only just",
              "touch in the plan will not touch on",
              "the floor."]
    ax2.text(0.0, 0.98, "\n".join(lines), family="monospace", fontsize=8.5,
             va="top", ha="left", transform=ax2.transAxes)

    fig.suptitle("Coverage is measured against the TOOL width, not the spacing.\n"
                 "Measure it against the spacing and every setting scores "
                 "100 percent while the floor stays dirty.", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    p = out / "fig_coverage_spacing.png"
    fig.savefig(p, dpi=160)
    plt.close(fig)
    return p


def write_all(coverage, out_dir):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    return [fig_coverage_axis(coverage, out), fig_coverage_spacing(coverage, out)]


if __name__ == "__main__":
    import coverage
    target = sys.argv[1] if len(sys.argv) > 1 else "figures"
    for p in write_all(coverage, target):
        print(p)
