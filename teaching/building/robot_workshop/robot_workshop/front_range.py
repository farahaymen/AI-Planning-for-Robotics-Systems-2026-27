import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


def front_min(ranges, angle_min, angle_increment, range_min, range_max):
    valid = []
    for index, distance in enumerate(ranges):
        angle = angle_min + index * angle_increment
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(angle) <= math.radians(15):
            if math.isfinite(distance) and range_min <= distance <= range_max:
                valid.append(distance)
    return min(valid) if valid else float('nan')


class FrontRange(Node):
    def __init__(self):
        super().__init__('front_range')
        self.publisher = self.create_publisher(Float32, 'distance', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan):
        message = Float32()
        message.data = float(front_min(scan.ranges, scan.angle_min,
                                      scan.angle_increment, scan.range_min,
                                      scan.range_max))
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = FrontRange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
