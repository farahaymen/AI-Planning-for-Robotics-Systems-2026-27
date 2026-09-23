import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


def horizontal_ray(width, height, row, start_col, hit_col):
    if not (0 <= row < height and 0 <= start_col < hit_col < width):
        raise ValueError('The horizontal ray must fit inside the grid')
    data = [-1] * (width * height)
    for col in range(start_col, hit_col):
        data[row * width + col] = 0
    data[row * width + hit_col] = 100
    return data


class SmallMap(Node):
    def __init__(self):
        super().__init__('small_map')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid, 'student_map', qos)
        self.timer = self.create_timer(1.0, self.publish_map)

    def publish_map(self):
        message = OccupancyGrid()
        message.header.frame_id = 'odom'
        message.header.stamp = self.get_clock().now().to_msg()
        message.info.resolution = 0.1
        message.info.width = 20
        message.info.height = 10
        message.info.origin.orientation.w = 1.0
        message.data = horizontal_ray(20, 10, row=4, start_col=2, hit_col=8)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SmallMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
