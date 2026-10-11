#!/usr/bin/env python3
"""Show Lab 3 occupancy grid mapping LIVE from Gazebo /scan and /odom.

Does not start, use, or require SLAM Toolbox. Uses the supplied course
starters.lab03.occupancy reference inverse sensor model, independently of /map.

Run from ~/arc_ws/src/arc-course with the ROS 2 Jazzy environment sourced:
    python3 lab03_live_ogm.py

Optional:
    python3 lab03_live_ogm.py --resolution 0.10 --scan-stride 3 --beam-stride 4

Keys in the plotting window: s = save PNG, p = probability view,
t = three-state occupancy view, q = quit.
"""
from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path
import math
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from starters.lab03.occupancy import MappingParams, OccupancyMap, sensor_pose_from_base


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def planar_pose(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    return (float(p.x), float(p.y), yaw)


class LiveOGM(Node):
    def __init__(self, args):
        super().__init__("lab03_live_ogm")
        self.args = args
        self.lock = threading.Lock()
        self.odometry = deque(maxlen=250)
        self.grid = None
        self.scans_received = 0
        self.scans_mapped = 0
        self.scans_skipped = 0
        self.pose = None
        self.route = []
        self.create_subscription(Odometry, "/odom", self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self.on_scan,
                                 qos_profile_sensor_data)

    def on_odom(self, msg):
        with self.lock:
            self.odometry.append((stamp(msg), planar_pose(msg)))

    def on_scan(self, msg):
        self.scans_received += 1
        if self.scans_received % self.args.scan_stride:
            return
        with self.lock:
            if not self.odometry:
                self.scans_skipped += 1
                return
            t, base_pose = min(self.odometry,
                               key=lambda item: abs(item[0] - stamp(msg)))
        if abs(t - stamp(msg)) > 0.10:
            self.scans_skipped += 1
            return

        # In this course the laser is 0.10 m in front of the robot body.
        sensor_pose = sensor_pose_from_base(base_pose, 0.10)
        indices = np.arange(0, len(msg.ranges), self.args.beam_stride)
        ranges = np.asarray(msg.ranges, dtype=np.float64)[indices]
        angles = msg.angle_min + indices * msg.angle_increment

        with self.lock:
            if self.grid is None:
                params = MappingParams(resolution=self.args.resolution,
                                       min_range=float(msg.range_min),
                                       max_range=float(msg.range_max))
                self.grid = OccupancyMap(14.0, 14.0, origin=(-2.0, -2.0),
                                         params=params)
            self.grid.integrate_scan(sensor_pose, ranges, angles)
            self.scans_mapped += 1
            self.pose = base_pose
            if not self.route or math.hypot(base_pose[0] - self.route[-1][0],
                                            base_pose[1] - self.route[-1][1]) > 0.03:
                self.route.append(base_pose[:2])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--resolution", type=float, default=0.05,
                        help="map cell size in metres (default 0.05)")
    parser.add_argument("--scan-stride", type=int, default=3,
                        help="integrate one of every N scans (default 3)")
    parser.add_argument("--beam-stride", type=int, default=4,
                        help="integrate one of every N laser beams (default 4)")
    args = parser.parse_args()
    if args.resolution <= 0 or args.scan_stride < 1 or args.beam_stride < 1:
        parser.error("resolution and strides must be positive")

    rclpy.init()
    node = LiveOGM(args)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, ax = plt.subplots(figsize=(7.5, 7))
    fig.canvas.manager.set_window_title("Lab 3: Live Occupancy Grid (NO SLAM)")
    initial_cells = math.ceil(14 / args.resolution)
    plot = ax.imshow(np.full((initial_cells, initial_cells), 205),
                     origin="lower", extent=(-2, 12, -2, 12),
                     vmin=0, vmax=255, cmap="gray", interpolation="nearest")
    (robot_marker,) = ax.plot([], [], "bo", markersize=6, label="Robot (odometry)")
    (trail_line,) = ax.plot([], [], "b-", linewidth=1, alpha=0.75,
                            label="Odometry route")
    ax.set(xlabel="Odometry x (m)", ylabel="Odometry y (m)",
           xlim=(-2, 12), ylim=(-2, 12))
    ax.set_aspect("equal")
    ax.legend(loc="upper right")
    fig.tight_layout()
    probability_mode = False

    def on_key(event):
        nonlocal probability_mode
        if event.key == "p":
            probability_mode = True
        elif event.key == "t":
            probability_mode = False
        elif event.key == "s":
            target = Path.home() / "arc_ws/maps/lab03_live_ogm.png"
            target.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(target, dpi=150)
            print(f"Saved snapshot: {target}", flush=True)
        elif event.key in ("q", "escape"):
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.ion()
    plt.show(block=False)
    print("Lab 3 LIVE OGM: /scan + /odom -> occupancy.py -> Matplotlib", flush=True)
    print("No SLAM and no /map subscription. Keys: p probability, t trinary, s save, q quit.", flush=True)
    print("Start Gazebo and an independent robot driver (arc-drive).", flush=True)

    try:
        while rclpy.ok() and plt.fignum_exists(fig.number):
            with node.lock:
                ready = node.grid is not None
                if ready:
                    if probability_mode:
                        pixels = (255 * (1 - node.grid.probability())).astype(np.uint8)
                    else:
                        occupied = node.grid.to_ros_occupancy()
                        pixels = np.full(occupied.shape, 205, dtype=np.uint8)
                        pixels[occupied == 0] = 254
                        pixels[occupied == 100] = 0
                pose = node.pose
                route = list(node.route)
                received = node.scans_received
                mapped = node.scans_mapped
                skipped = node.scans_skipped
                coverage = node.grid.coverage() if ready else 0

            if ready:
                plot.set_data(pixels)
            if pose is not None:
                robot_marker.set_data([pose[0]], [pose[1]])
            if route:
                trail_line.set_data([p[0] for p in route],
                                    [p[1] for p in route])
            ax.set_title(f"Live Lab 3 OGM ({'probability' if probability_mode else 'free / occupied / unknown'})\n"
                         f"mapped scans: {mapped} / received: {received} | "
                         f"unmatched: {skipped} | observed cells: {coverage:.1%}")
            fig.canvas.draw_idle()
            plt.pause(0.3)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2)
        node.destroy_node()
        plt.close("all")


if __name__ == "__main__":
    main()
