import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class DistanceAlert(Node):
    def __init__(self):
        super().__init__('distance_alert')
        self.declare_parameter('threshold_m', 1.0)
        self.subscription = self.create_subscription(
            Float32, 'distance', self.on_distance, 10)

    def on_distance(self, message):
        threshold = float(self.get_parameter('threshold_m').value)
        if not math.isfinite(message.data):
            self.get_logger().warning('No valid distance measurement')
        elif message.data < threshold:
            self.get_logger().warning(f'Close object: {message.data:.2f} m')
        else:
            self.get_logger().info(f'Distance: {message.data:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = DistanceAlert()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
