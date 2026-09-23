import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class DistanceSource(Node):
    def __init__(self):
        super().__init__('distance_source')
        self.publisher = self.create_publisher(Float32, 'distance', 10)
        self.reading = 1.5
        self.timer = self.create_timer(0.5, self.publish_distance)

    def publish_distance(self):
        message = Float32()
        message.data = self.reading
        self.publisher.publish(message)
        self.reading = 0.5 if self.reading > 1.0 else 1.5


def main(args=None):
    rclpy.init(args=args)
    node = DistanceSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
