import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from robot_workshop.front_range import front_min


class ScanService(Node):
    def __init__(self):
        super().__init__('scan_service')
        self.last_stamp = None
        self.last_valid = False
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.service = self.create_service(Trigger, 'check_scan', self.on_check)

    def on_scan(self, scan):
        value = front_min(scan.ranges, scan.angle_min, scan.angle_increment,
                          scan.range_min, scan.range_max)
        self.last_valid = math.isfinite(value)
        self.last_stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9

    def on_check(self, request, response):
        now = self.get_clock().now().nanoseconds * 1e-9
        age = None if self.last_stamp is None else now - self.last_stamp
        response.success = bool(self.last_valid and age is not None and 0 <= age < 0.5)
        response.message = 'No scan yet' if age is None else f'Scan age: {age:.3f} s'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ScanService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
