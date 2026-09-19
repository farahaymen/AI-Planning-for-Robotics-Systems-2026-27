"""
Lab 4, Exercise 4.3. Run every planner on every map and compare them.

    python3 compare.py                     reference planners, eight-connected
    python3 compare.py --skeleton          your planners from planners_skeleton.py
    python3 compare.py --moves 4           four-connected, where BFS is optimal again
    python3 compare.py --map greedy_trap   one map, with the ASCII path drawn
    python3 compare.py --figures out/      write the three comparison figures

The table prints length, step count, cells expanded, peak frontier size and
runtime, for one reason: a comparison is only honest if every algorithm is
called the same way and measured the same way. You will read claims in papers
that compare a new planner's runtime against someone else's expansion count.
Those claims mean nothing.

Read the "excess" column first. It is the percentage by which a planner's path
exceeds the shortest one on that map, and it is zero for every optimal planner
on every map. The rows where it is not zero are the whole lab.
"""

from __future__ import annotations

import argparse
import importlib
import sys

from gridmap import (GOAL, MAPS, MOVES_4, MOVES_8, SOLVABLE, START,
                     is_valid_path, render)

HEADER = (f"{'planner':22s} {'length':>8s} {'steps':>6s} {'excess':>8s} "
          f"{'expanded':>9s} {'frontier':>9s} {'ms':>8s}")


def table(planners, grid, moves, draw=None):
    rows = []
    for label, run in planners.PLANNERS.items():
        result = run(grid, START, GOAL, moves)
        rows.append((label, result))

    found = [r for _, r in rows if r.found]
    best = min((r.length for r in found), default=0.0)

    print(HEADER)
    print("-" * len(HEADER))
    for label, r in rows:
        if not r.found:
            print(f"{label:22s} {'no path':>8s} {'':>6s} {'':>8s} "
                  f"{r.expanded:9d} {r.max_frontier:9d} {r.runtime_ms:8.2f}")
            continue
        excess = 100.0 * (r.length - best) / best if best else 0.0
        flag = " " if excess < 1e-6 else "*"
        print(f"{label:22s} {r.length:8.2f} {r.steps:6d} {excess:7.2f}%{flag}"
              f"{r.expanded:9d} {r.max_frontier:9d} {r.runtime_ms:8.2f}")
    return dict(rows)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--skeleton", action="store_true",
                    help="use planners_skeleton.py instead of the reference")
    ap.add_argument("--moves", type=int, default=8, choices=(4, 8),
                    help="four- or eight-connected movement (default 8)")
    ap.add_argument("--map", default=None, choices=sorted(MAPS),
                    help="one map only, and draw the paths")
    ap.add_argument("--figures", metavar="DIR", default=None,
                    help="write the comparison figures to DIR")
    args = ap.parse_args(argv)

    planners = importlib.import_module(
        "planners_skeleton" if args.skeleton else "planners")
    moves = MOVES_8 if args.moves == 8 else MOVES_4

    names = [args.map] if args.map else list(MAPS)
    results = {}
    for name in names:
        grid = MAPS[name]()
        print()
        print("=" * len(HEADER))
        print(f"{name}   {args.moves}-connected   "
              f"start {START}  goal {GOAL}  {int(grid.sum())} blocked cells")
        print("=" * len(HEADER))
        results[name] = table(planners, grid, moves)

        if args.map:
            for label in ("Dijkstra", "Greedy best first", "A* inadmissible"):
                r = results[name].get(label)
                if r is None or not r.found:
                    continue
                print(f"\n{label}  length {r.length:.2f}  "
                      f"expanded {r.expanded}")
                print(render(grid, r.path))

    if not args.map:
        print("\n* marks a path longer than the shortest one found on that map.")
        print("Every row with a star is a planner giving something up. Say what,")
        print("and say what it bought, in your Exercise 4.3 write-up.")

    if args.figures:
        from figures import write_all
        paths = write_all(planners, args.figures, moves)
        print(f"\nwrote {len(paths)} figures to {args.figures}")

    # A comparison built on invalid paths is worse than no comparison.
    bad = [(m, lbl) for m, rs in results.items() for lbl, r in rs.items()
           if r.found and not is_valid_path(MAPS[m](), r.path, START, GOAL)]
    if bad:
        print("\nINVALID PATHS RETURNED:")
        for m, lbl in bad:
            print(f"  {lbl} on {m}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
