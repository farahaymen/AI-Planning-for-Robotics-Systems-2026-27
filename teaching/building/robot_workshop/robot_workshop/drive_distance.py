"""Lab 2: odometry feedback, bounded forward command, sensor timeout stop."""
from robot_workshop.motion_rule import command_speed
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class DriveDistance(Node):
    def __init__(self):
        super().__init__('drive_distance')
        self.declare_parameter('distance', .6)
        self.declare_parameter('speed', .15)
        self.distance=float(self.get_parameter('distance').value)
        self.speed=float(self.get_parameter('speed').value)
        if not 0 < self.distance <= 2 or not 0 < self.speed <= .3:
            raise ValueError('distance must be (0,2] m and speed (0,.3] m/s')
        self.publisher=self.create_publisher(Twist,'/cmd_vel',10)
        self.start=None;self.travelled=0.;self.odom_time=-math.inf;self.scan_time=-math.inf
        self.clear=False;self.done=False;self.started=time.monotonic()
        self.create_subscription(Odometry,'/odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        self.wall_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05,self.tick,clock=self.wall_clock)

    def odom(self,msg):
        p=msg.pose.pose.position
        if self.start is None:self.start=(p.x,p.y)
        self.travelled=math.hypot(p.x-self.start[0],p.y-self.start[1])
        self.odom_time=time.monotonic()

    def scan(self,msg):
        angles=msg.angle_min+np.arange(len(msg.ranges))*msg.angle_increment
        forward=np.abs(np.arctan2(np.sin(angles),np.cos(angles))) < .45
        ranges=np.asarray(msg.ranges)[forward]
        valid=(np.isfinite(ranges)&(ranges>=msg.range_min)&(ranges<=msg.range_max))|np.isposinf(ranges)
        self.clear=bool(len(ranges)>0 and valid.all() and np.min(ranges)>.45)
        self.scan_time=time.monotonic()

    def tick(self):
        now=time.monotonic();command=Twist()
        if self.travelled >= self.distance and not self.done:
            self.get_logger().info(f'Completed: odometry displacement {self.travelled:.3f} m')
            self.done=True
        if now-self.started > 60 and not self.done:
            self.get_logger().error('Timed out. Check scan, odometry and controllers.');self.done=True
        if not self.done and self.clear and now-self.odom_time<.5 and now-self.scan_time<.5:
            command.linear.x=command_speed(self.travelled,self.distance,self.speed)
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args);node=DriveDistance()
    try:
        while rclpy.ok() and not node.done:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
