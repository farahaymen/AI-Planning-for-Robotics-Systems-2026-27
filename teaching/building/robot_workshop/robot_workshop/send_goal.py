import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def main(args=None):
    rclpy.init(args=args)
    node = Node('student_goal_client')
    node.declare_parameter('x', 0.0)
    node.declare_parameter('y', 0.0)
    node.declare_parameter('yaw', 0.0)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    handle = None
    try:
        if not client.wait_for_server(timeout_sec=10.0):
            node.get_logger().error('Navigation action server is unavailable')
            return
        # Let a subscribed simulation clock update before stamping the goal.
        rclpy.spin_once(node, timeout_sec=1.0)
        x, y, yaw = [float(node.get_parameter(name).value) for name in ('x', 'y', 'yaw')]
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError('Goal coordinates and yaw must be finite')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        def feedback(message):
            remaining = message.feedback.distance_remaining
            node.get_logger().info(f'Remaining path: {remaining:.2f} m')

        future = client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            node.get_logger().warning('Goal rejected')
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result()
        if result is not None:
            success = result.status == GoalStatus.STATUS_SUCCEEDED
            node.get_logger().info(f'Finished: success={success}, status={result.status}')
    except KeyboardInterrupt:
        if handle is not None and rclpy.ok():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=2.0)
    finally:
        client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
