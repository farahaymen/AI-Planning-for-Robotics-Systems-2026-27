#!/usr/bin/env python3
"""Read a bag into plain NumPy arrays. No ROS node, no live simulation.

Supplied because parsing a bag is not the learning objective. Lab 4 Code 4.1.
"""

import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def read_run(path, scan_topic="/scan", odom_topic="/odom"):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    scans, odoms = [], []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic == scan_topic:
            scans.append((stamp_ns, deserialize_message(data, LaserScan)))
        elif topic == odom_topic:
            odoms.append((stamp_ns, deserialize_message(data, Odometry)))
    return scans, odoms


def pose_from_odom(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    theta = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
    return float(p.x), float(p.y), float(theta)


def nearest_pose(scan_stamp_ns, odoms):
    """Odometry runs at 50 Hz and the LiDAR at 10 Hz, so no odometry sample
    exists at exactly the right instant. Nearest neighbour costs up to 10 ms,
    which at 0.5 m/s is 5 mm. Interpolating would be better and is a legitimate
    extension."""
    stamps = np.array([s for s, _ in odoms])
    return pose_from_odom(odoms[int(np.argmin(np.abs(stamps - scan_stamp_ns)))][1])


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "bags/lab03_mapping_run"
    scans, odoms = read_run(path)
    print(f"{len(scans)} scans, {len(odoms)} odometry samples")
    if scans:
        s = scans[0][1]
        print(f"{len(s.ranges)} beams, angle_min {s.angle_min:.3f}, "
              f"increment {s.angle_increment:.5f}, frame '{s.header.frame_id}'")


if __name__ == "__main__":
    main()
