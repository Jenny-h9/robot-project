#!/usr/bin/env python3
"""Unified ROS 2 navigation node: emergency stop, obstacle avoidance and rejoin."""
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class State(Enum):
    FOLLOW = 1; DECIDE = 2; AVOID = 3; REJOIN = 4; RECOVERY = 5; ESTOP = 6


class UnifiedNavigationNode(Node):
    def __init__(self):
        super().__init__('unified_navigation_node')
        self.d_stop = self.declare_parameter('d_stop', 0.35).value
        self.d_avoid = self.declare_parameter('d_avoid', 0.90).value
        self.max_speed = self.declare_parameter('max_linear_speed', 0.45).value
        self.max_angular = self.declare_parameter('max_angular_speed', 0.8).value
        self.rejoin_tolerance = self.declare_parameter('rejoin_tolerance', 0.30).value
        self.scan_timeout = self.declare_parameter('scan_timeout', 0.25).value
        self.state, self.scan, self.odom, self.global_path = State.ESTOP, None, None, None
        self.last_scan = self.get_clock().now()
        self.create_subscription(LaserScan, '/slamware_ros_sdk_server_node/scan', self.on_scan, 10)
        self.create_subscription(Odometry, '/slamware_ros_sdk_server_node/odom', self.on_odom, 10)
        self.create_subscription(Path, '/plan', self.on_path, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/unified_nav/status', 10)
        self.create_subscription(PoseStamped, '/unified_nav/goal_pose', self.on_goal, 10)
        self.goal_pose = None
        self.create_timer(0.05, self.control_loop)

    def on_scan(self, msg): self.scan, self.last_scan = msg, self.get_clock().now()
    def on_odom(self, msg): self.odom = msg
    def on_path(self, msg): self.global_path = msg
    def on_goal(self, msg): self.goal_pose = msg; self.state = State.FOLLOW

    def min_distance(self):
        if self.scan is None: return math.inf
        vals = [x for x in self.scan.ranges if math.isfinite(x) and self.scan.range_min <= x <= self.scan.range_max]
        return min(vals, default=math.inf)

    def stop(self): self.cmd_pub.publish(Twist())
    def control_loop(self):
        self.status_pub.publish(String(data=self.state.name))
        age = (self.get_clock().now() - self.last_scan).nanoseconds / 1e9
        d = self.min_distance()
        if age > self.scan_timeout or d <= self.d_stop:
            self.state = State.ESTOP
        if self.state == State.ESTOP:
            self.stop()
            if age <= self.scan_timeout and d > self.d_stop + 0.10: self.state = State.DECIDE
            return
        if self.odom is None or self.goal_pose is None:
            self.stop(); return
        dx=self.goal_pose.pose.position.x-self.odom.pose.pose.position.x
        dy=self.goal_pose.pose.position.y-self.odom.pose.pose.position.y
        dist=math.hypot(dx,dy)
        q=self.odom.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        target_yaw=2.0*math.atan2(self.goal_pose.pose.orientation.z,self.goal_pose.pose.orientation.w)
        err=(math.atan2(dy,dx)-yaw+math.pi)%(2*math.pi)-math.pi
        if dist < self.rejoin_tolerance:
            self.stop(); self.status_pub.publish(String(data='ARRIVED')); self.goal_pose=None; return
        if self.global_path is None:
            dx=self.goal_pose.pose.position.x-self.odom.pose.pose.position.x
            dy=self.goal_pose.pose.position.y-self.odom.pose.pose.position.y
            pass
        if self.state == State.FOLLOW and d <= self.d_avoid: self.state = State.DECIDE
        if self.state == State.DECIDE:
            self.state = State.AVOID if d > self.d_stop else State.ESTOP
        if self.state == State.AVOID:
            cmd = Twist(); cmd.linear.x = min(0.20, self.max_speed); cmd.angular.z = 0.45
            self.cmd_pub.publish(cmd)
            if d > self.d_avoid: self.state = State.REJOIN
            return
        if self.state == State.REJOIN:
            if self.global_path is None: self.state = State.RECOVERY; self.stop(); return
            cmd = Twist(); cmd.linear.x = min(0.25, self.max_speed); self.cmd_pub.publish(cmd)
            self.state = State.FOLLOW
            return
        if self.state == State.RECOVERY: self.stop(); self.state = State.DECIDE; return
        cmd = Twist(); cmd.linear.x = min(self.max_speed, 0.6*dist) if abs(err)<1.0 else 0.0
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, 1.5*err))
        self.cmd_pub.publish(cmd)

    def destroy_node(self):
        self.stop()
        super().destroy_node()


def main():
    rclpy.init(); node = UnifiedNavigationNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
