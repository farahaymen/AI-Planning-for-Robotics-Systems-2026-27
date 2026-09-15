#!/usr/bin/env python3
"""Subscribes to /range and reports the rate it is actually receiving.

Reporting zero explicitly is the point. A subscriber that says nothing when it
receives nothing is indistinguishable from one that has crashed.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class RangeMonitor(Node):
    def __init__(self):
        super().__init__("range_monitor")
        self.subscription = self.create_subscription(Float32, "range", self.on_range, 10)
        self.count = 0
        self.create_timer(2.0, self.report)

    def on_range(self, msg):
        self.count += 1
        if msg.data < 1.0:
            self.get_logger().warn(f"close range {msg.data:.2f}")

    def report(self):
        self.get_logger().info(f"{self.count / 2.0:.1f} Hz over the last 2 s")
        self.count = 0


def main():
    rclpy.init()
    node = RangeMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
