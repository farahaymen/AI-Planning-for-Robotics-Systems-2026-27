#!/usr/bin/env python3
"""
cmd_vel_relay - accept an unstamped Twist on /cmd_vel, drive the controller.

WHY THIS NODE EXISTS

Recent versions of diff_drive_controller subscribe only to
geometry_msgs/msg/TwistStamped. The older `use_stamped_vel` parameter that used
to select the unstamped form has been removed, so there is no configuration that
makes the controller accept a plain Twist.

Almost everything else in this course publishes an unstamped Twist:

    teleop_twist_keyboard          Lab 3 driving, Lab 5 mapping runs
    arc_lab5 drift_meter           Lab 2 and Lab 5 odometry measurement
    arc_rl RosNavEnv               Lab 9 and Lab 10 policy execution
    Nav2 controller_server         Lab 6 onward, on this distribution

Publishing the wrong type is a SILENT failure. The topic exists, both ends
report healthy, `ros2 topic pub` prints happily, and the controller receives
nothing. `ros2 topic info /cmd_vel` is the only place it shows, as two entries
under Type. That is the same class of fault as a QoS mismatch, and it costs
hours to find if you do not already know to look.

So rather than rewriting six places to emit TwistStamped and losing
teleop_twist_keyboard entirely, one small node converts at the boundary:

    anything  --Twist-->  /cmd_vel  --[this node]-->  TwistStamped  -->  controller

The header stamp is filled in here, which is the only information the unstamped
message lacks.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")

        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/diff_drive_controller/cmd_vel")
        self.declare_parameter("frame_id", "base_link")

        self.frame_id = self.get_parameter("frame_id").value
        output_topic = self.get_parameter("output_topic").value
        input_topic = self.get_parameter("input_topic").value

        self.publisher = self.create_publisher(TwistStamped, output_topic, 10)
        self.create_subscription(Twist, input_topic, self.on_twist, 10)

        self.count = 0
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"relaying Twist on {input_topic} to TwistStamped on {output_topic}")

    def on_twist(self, msg: Twist) -> None:
        stamped = TwistStamped()
        # The stamp is the one thing the unstamped message cannot carry. Using
        # the node clock means it follows simulation time when use_sim_time is
        # set, which the controller requires.
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.publisher.publish(stamped)
        self.count += 1

    def report(self) -> None:
        # Reporting zero explicitly matters: a relay that says nothing when it
        # receives nothing looks identical to one that has crashed.
        self.get_logger().info(f"relayed {self.count} command(s) in the last 10 s")
        self.count = 0


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
