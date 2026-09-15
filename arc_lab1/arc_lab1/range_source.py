#!/usr/bin/env python3
"""Publishes a fake range reading at 10 Hz, the rate of the real LiDAR.

Exercise 1.2 asks you to change the publisher QoS to best effort and observe
that the default subscriber then receives nothing, with no error anywhere.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class RangeSource(Node):
    def __init__(self):
        super().__init__("range_source")
        # Declared, not hard coded, so `ros2 param set` works at runtime.
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("amplitude", 2.0)
        rate = self.get_parameter("rate_hz").value

        self.publisher = self.create_publisher(Float32, "range", 10)
        # A timer, not a while loop. rclpy.spin hands the thread to an executor;
        # a while loop here starves every other callback and the node looks
        # alive while silently ignoring everything.
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.k = 0
        self.get_logger().info(f"publishing /range at {rate} Hz")

    def tick(self):
        amplitude = self.get_parameter("amplitude").value
        msg = Float32()
        msg.data = float(amplitude * (1.5 + math.sin(self.k * 0.1)))
        self.publisher.publish(msg)
        self.k += 1


def main():
    rclpy.init()
    node = RangeSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
