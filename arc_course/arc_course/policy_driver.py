"""Lab 9: single ROS/Gazebo episode in the odom frame. No hidden world reset.

Only run with teleop and Nav2 stopped. Restart Gazebo for a fresh episode.
The proximity stop is conservative and is not a certified collision guarantee.
"""
import csv
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from arc_rl.nav_core import scale_action
from arc_rl.baselines import make_gap_follow_policy
from teaching.ros_contract import sample_laser,from_pose


class PolicyDriver(Node):
    def __init__(self):
        super().__init__('policy_driver')
        self.declare_parameter('policy','gap');self.declare_parameter('model','')
        self.declare_parameter('goal_x',1.0);self.declare_parameter('goal_y',0.0)
        self.declare_parameter('duration',60.0);self.declare_parameter('output','policy_episode.csv')
        self.policy_name=str(self.get_parameter('policy').value)
        model=str(self.get_parameter('model').value)
        if self.policy_name=='gap':self.policy=make_gap_follow_policy()
        elif self.policy_name=='dqn':
            from teaching.deep_q import load_policy
            self.policy=load_policy(model)
        elif self.policy_name in ('ppo','hybrid'):
            from stable_baselines3 import PPO
            learned=PPO.load(model,device='cpu')
            if learned.observation_space.shape!=(29,) or learned.action_space.shape!=(2,):raise ValueError('Policy dimensions do not match ARC')
            self.policy=lambda obs:learned.predict(obs,deterministic=True)[0]
            if self.policy_name=='hybrid':
                from starters.lab09.hybrid import HybridController
                self.policy=HybridController(make_gap_follow_policy(),self.policy)
        else:raise ValueError('policy must be gap, dqn, ppo or hybrid')
        self.gx=float(self.get_parameter('goal_x').value);self.gy=float(self.get_parameter('goal_y').value)
        self.duration=float(self.get_parameter('duration').value)
        if not all(math.isfinite(x) for x in (self.gx,self.gy,self.duration)) or self.duration <= 0:
            raise ValueError('goal coordinates must be finite and duration positive')
        output=Path(str(self.get_parameter('output').value)).expanduser()
        output.parent.mkdir(parents=True,exist_ok=True);self.file=output.open('x',newline='')
        self.writer=csv.writer(self.file);self.writer.writerow(['sim_time','x_odom','y_odom','goal_distance','v_command','w_command','status'])
        self.publisher=self.create_publisher(Twist,'/cmd_vel',10)
        self.pose=None;self.beams=None;self.scan_time=self.odom_time=-math.inf
        self.scan_stamp=self.odom_stamp=-math.inf;self.minimum=0.;self.done=False;self.started=time.monotonic()
        self.create_subscription(Odometry,'/odom',self.odom,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        self.wall_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05,self.tick,clock=self.wall_clock)

    def odom(self,msg):
        if msg.header.frame_id.lstrip('/')!='odom':return
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.pose=(p.x,p.y,yaw,msg.twist.twist.linear.x,msg.twist.twist.angular.z)
        self.odom_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;self.odom_time=time.monotonic()

    def scan(self,msg):
        try:
            if msg.header.frame_id.lstrip('/')!='laser_link':raise ValueError('Expected laser_link frame')
            self.beams=sample_laser(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max)
            self.minimum=float(np.min(msg.ranges))
            self.scan_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;self.scan_time=time.monotonic()
        except ValueError as exc:
            self.beams=None;self.get_logger().warning(str(exc),throttle_duration_sec=5.)

    def tick(self):
        now=time.monotonic();sim=self.get_clock().now().nanoseconds*1e-9;command=Twist();status='waiting_for_fresh_sensors'
        fresh=(self.pose is not None and self.beams is not None and now-self.scan_time<.5 and now-self.odom_time<.5 and
               0<=sim-self.scan_stamp<.5 and 0<=sim-self.odom_stamp<.5 and abs(self.scan_stamp-self.odom_stamp)<.2)
        distance=math.inf
        if self.pose is not None:distance=math.hypot(self.gx-self.pose[0],self.gy-self.pose[1])
        if now-self.started>self.duration:status='wall_time_limit';self.done=True
        elif distance<.25:status='goal_reached';self.done=True
        elif len(self.get_publishers_info_by_topic('/cmd_vel')) > 1:
            status='competing_velocity_publisher'
        elif fresh:
            x,y,yaw,v,w=self.pose
            if self.minimum<.40:status='proximity_stop'
            else:
                obs=from_pose(self.beams,x,y,yaw,self.gx,self.gy,v,w)
                action=self.policy(obs,position=(x,y)) if self.policy_name=='hybrid' else self.policy(obs)
                if np.shape(action)==(2,) and np.isfinite(action).all():
                    command.linear.x,command.angular.z=scale_action(action);status='running'
                else:status='invalid_action';self.done=True
        self.publisher.publish(command)
        x,y=self.pose[:2] if self.pose is not None else (math.nan,math.nan)
        self.writer.writerow([sim,x,y,distance,command.linear.x,command.angular.z,status])
        if self.done:self.get_logger().info(status);self.file.flush()


def main(args=None):
    rclpy.init(args=args);node=PolicyDriver()
    try:
        while rclpy.ok() and not node.done:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.publisher.publish(Twist())
        node.file.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
