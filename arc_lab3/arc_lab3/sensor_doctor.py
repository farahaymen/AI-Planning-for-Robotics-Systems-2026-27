#!/usr/bin/env python3
"""Diagnose a sensor topic beyond its average rate.

    ros2 run arc_lab3 sensor_doctor --ros-args -p use_sim_time:=true
    ros2 run arc_lab3 sensor_doctor --ros-args -p use_sim_time:=false

The second invocation is the teaching moment: a node on the wall clock while the
simulator publishes /clock reports a stamp lag of about 1.7 billion seconds.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

from arc_lab3.rate_stats import analyse, stamp_lag

# Sensor data is published best effort. A subscription using the default
# reliable profile connects successfully and receives nothing. Lab 1, Ex 1.2.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class SensorDoctor(Node):
    def __init__(self):
        super().__init__("sensor_doctor")
        self.declare_parameter("topic", "/scan")
        self.declare_parameter("expected_hz", 10.0)
        self.declare_parameter("window", 100)

        self.expected_hz = self.get_parameter("expected_hz").value
        self.window = self.get_parameter("window").value
        self.header_stamps, self.receive_times, self.frame_id = [], [], None

        topic = self.get_parameter("topic").value
        self.create_subscription(LaserScan, topic, self.on_scan, SENSOR_QOS)
        self.get_logger().info(f"watching {topic}, expecting {self.expected_hz} Hz")

    def on_scan(self, msg):
        # Two clocks on purpose: when the sensor says the measurement was taken,
        # and when this node saw it. Comparing them catches use_sim_time errors.
        self.header_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.receive_times.append(self.get_clock().now().nanoseconds / 1e9)
        self.frame_id = msg.header.frame_id
        if len(self.header_stamps) >= self.window:
            self.report()
            self.header_stamps.clear()
            self.receive_times.clear()

    def report(self):
        result = analyse(self.header_stamps, self.expected_hz)
        lag = stamp_lag(self.header_stamps, self.receive_times)
        self.get_logger().info(str(result))
        self.get_logger().info(f"frame_id '{self.frame_id}', mean stamp lag {lag:.4f} s")
        if abs(lag) > 1.0:
            self.get_logger().error(
                "stamp lag over one second. This node and the publisher are on "
                "different clocks. Check use_sim_time.")


def main():
    rclpy.init()
    node = SensorDoctor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
