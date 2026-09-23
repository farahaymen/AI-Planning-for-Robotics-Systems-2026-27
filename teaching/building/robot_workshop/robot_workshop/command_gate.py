import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist


def limited_command(v, w, age_s, timeout_s=0.3):
    if not all(math.isfinite(value) for value in (v, w, age_s)):
        return 0.0, 0.0
    if age_s < 0 or age_s >= timeout_s:
        return 0.0, 0.0
    return max(-0.125, min(0.5, v)), max(-1.8, min(1.8, w))


class CommandGate(Node):
    def __init__(self):
        super().__init__('command_gate')
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.received_at = None
        self.request = (0.0, 0.0)
        self.publisher = self.create_publisher(Twist, 'checked_cmd', 10)
        self.subscription = self.create_subscription(Twist, 'candidate_cmd', self.on_command, 10)
        self.timer = self.create_timer(0.05, self.tick, clock=self.clock)

    def on_command(self, message):
        self.request = (message.linear.x, message.angular.z)
        self.received_at = self.clock.now().nanoseconds * 1e-9

    def tick(self):
        now = self.clock.now().nanoseconds * 1e-9
        age = float('inf') if self.received_at is None else now - self.received_at
        v, w = limited_command(*self.request, age)
        message = Twist()
        message.linear.x = v
        message.angular.z = w
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CommandGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
