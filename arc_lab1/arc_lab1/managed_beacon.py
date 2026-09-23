#!/usr/bin/env python3
"""A managed lifecycle node, used as an optional Lab 6 startup investigation.

After `configure` the publisher EXISTS and the topic appears in `ros2 topic
list`, and nothing is published. A topic existing is not the same as data
flowing, and treating them as the same causes a great deal of confused
debugging from Lab 6 onwards.

    ros2 lifecycle set /managed_beacon configure
    ros2 lifecycle set /managed_beacon activate
"""

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from std_msgs.msg import Float32


class ManagedBeacon(LifecycleNode):
    def __init__(self):
        super().__init__("managed_beacon")
        self.publisher = None
        self.timer = None
        self.k = 0

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.publisher = self.create_lifecycle_publisher(Float32, "beacon", 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.timer.cancel()
        self.get_logger().info("configured: publisher created, nothing published yet")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.timer.reset()
        self.get_logger().info("active: publishing")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.timer.cancel()
        self.get_logger().info("inactive: still running, not publishing")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.publisher)
        return TransitionCallbackReturn.SUCCESS

    def tick(self):
        msg = Float32()
        msg.data = float(self.k)
        self.publisher.publish(msg)
        self.k += 1


def main():
    rclpy.init()
    node = ManagedBeacon()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
