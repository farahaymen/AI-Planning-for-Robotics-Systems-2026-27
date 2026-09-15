#!/usr/bin/env python3
"""Measure odometry drift against ground truth.

The robot does not have access to ground truth. You do, because this is
simulation, and comparing the two is the entire motivation for Lab 5.

    ros2 run arc_lab5 drift_meter
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class DriftMeter(Node):
    def __init__(self):
        super().__init__("drift_meter")
        self.declare_parameter("truth_topic", "/ground_truth")
        self.declare_parameter("report_every_m", 5.0)

        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(
            Odometry, self.get_parameter("truth_topic").value, self.on_truth, 10)

        self.odom = self.truth = None
        self.distance = 0.0
        self.last_odom = None
        self.next_report = self.get_parameter("report_every_m").value

    @staticmethod
    def pose_of(msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        return p.x, p.y, yaw

    def on_odom(self, msg):
        self.odom = self.pose_of(msg)
        if self.last_odom is not None:
            self.distance += math.hypot(self.odom[0] - self.last_odom[0],
                                        self.odom[1] - self.last_odom[1])
        self.last_odom = self.odom
        if self.distance >= self.next_report:
            self.report()
            self.next_report += self.get_parameter("report_every_m").value

    def on_truth(self, msg):
        self.truth = self.pose_of(msg)

    def report(self):
        if self.odom is None or self.truth is None:
            self.get_logger().warn("no ground truth; is it published?")
            return
        error = math.hypot(self.truth[0] - self.odom[0], self.truth[1] - self.odom[1])
        heading = math.degrees(math.atan2(
            math.sin(self.truth[2] - self.odom[2]),
            math.cos(self.truth[2] - self.odom[2])))
        # The percentage is the interesting column: roughly constant means the
        # error is proportional to distance, which is what accumulating random
        # error looks like. Growing means something systematic is present too.
        self.get_logger().info(
            f"driven {self.distance:5.1f} m | position error {error:5.3f} m "
            f"| heading error {heading:6.1f} deg | {100 * error / max(self.distance, 1e-6):4.1f}%")


def main():
    rclpy.init()
    node = DriftMeter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
