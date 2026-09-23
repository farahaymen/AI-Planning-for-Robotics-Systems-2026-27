"""Build a simple odometry-based grid from the Lab 2 bag (ROS VM required)."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

import importlib
import os
_mapping = importlib.import_module('starters.lab03.' + os.environ.get('ARC_OCCUPANCY', 'occupancy'))
MappingParams, OccupancyMap = _mapping.MappingParams, _mapping.OccupancyMap


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--stride", type=int, default=3)
    parser.add_argument("--laser-x", type=float, default=0.10)
    parser.add_argument("--resolution", type=float, default=0.05)
    args = parser.parse_args()
    if args.stride < 1 or args.resolution <= 0:
        parser.error("stride and resolution must be positive")
    if args.out.exists():
        parser.error("output directory exists; choose a new name")
    from arc_mapping.bag_reader import read_run, pose_from_odom
    scans, odoms = read_run(args.bag)
    if not scans or not odoms:
        parser.error("bag must contain /scan and /odom")
    if any(msg.header.frame_id != "laser_link" for _, msg in scans):
        parser.error("expected laser_link scans; repair the frame fault before mapping")
    odoms.sort(key=lambda item: stamp_ns(item[1]))
    stamps = np.asarray([stamp_ns(msg) for _, msg in odoms], dtype=np.int64)
    first_scan = scans[0][1]
    params = MappingParams(resolution=args.resolution,
                           max_range=float(first_scan.range_max),
                           min_range=float(first_scan.range_min))
    grid = OccupancyMap(14, 14, origin=(-2.0, -2.0), params=params)
    gaps, used, skipped = [], 0, 0
    for _, scan in scans[::args.stride]:
        t = stamp_ns(scan)
        k = int(np.searchsorted(stamps, t))
        candidates = [j for j in (k - 1, k) if 0 <= j < len(stamps)]
        j = min(candidates, key=lambda index: abs(int(stamps[index]) - t))
        gap = abs(int(stamps[j]) - t) / 1e9
        if gap > 0.10:
            skipped += 1
            continue
        x, y, theta = pose_from_odom(odoms[j][1])
        sensor_pose = (x + args.laser_x * math.cos(theta),
                       y + args.laser_x * math.sin(theta), theta)
        sx, sy, _ = sensor_pose
        if not (-2 <= sx < 12 and -2 <= sy < 12):
            skipped += 1
            continue
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        if (not math.isclose(scan.range_max, params.max_range)
                or not math.isclose(scan.range_min, params.min_range)):
            parser.error("sensor range bounds change within the bag; split the recording")
        grid.integrate_scan(sensor_pose, np.asarray(scan.ranges), angles)
        gaps.append(gap)
        used += 1
    if not used:
        parser.error("no usable scans: inspect header clocks, odometry and map bounds")
    args.out.mkdir(parents=True)
    cells = grid.to_ros_occupancy()
    pixels = np.full(cells.shape, 205, dtype=np.uint8)
    pixels[cells == 0] = 254
    pixels[cells == 100] = 0
    with (args.out / "map.pgm").open("wb") as handle:
        handle.write(f"P5\n{grid.width} {grid.height}\n255\n".encode())
        handle.write(np.flipud(pixels).tobytes())
    metadata = dict(image="map.pgm", resolution=args.resolution,
                    origin=[-2.0, -2.0, 0.0], negate=0,
                    occupied_thresh=0.65, free_thresh=0.25, mode="trinary")
    (args.out / "map.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False))
    np.save(args.out / "log_odds.npy", grid.log_odds)
    fig, ax = plt.subplots(figsize=(7, 7), layout="constrained")
    ax.imshow(np.flipud(pixels), cmap="gray", vmin=0, vmax=255, extent=(-2, 12, -2, 12))
    ax.set(xlabel="odom x (m)", ylabel="odom y (m)", title="Occupancy from measured scans and odometry")
    fig.savefig(args.out / "map.png", dpi=150)
    plt.close(fig)
    report = dict(scans_used=used, scans_skipped=skipped, stride=args.stride,
                  max_header_gap_s=max(gaps), coverage_fraction=grid.coverage(),
                  laser_x_m=args.laser_x, frame="odom",
                  assumptions="fixed planar laser, nearest header-time odometry, no deskew, bounds [-2,12)")
    (args.out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
