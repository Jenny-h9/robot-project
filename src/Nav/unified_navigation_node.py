#!/usr/bin/env python3
"""Unified ROS 2 navigation node: emergency stop, obstacle avoidance and rejoin."""
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan


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
        self.state, self.scan, self.odom, self.global_path = State.FOLLOW, None, None, None
        self.last_scan = self.get_clock().now()
        self.create_subscription(LaserScan, '/slamware_ros_sdk_server_node/scan', self.on_scan, 10)
        self.create_subscription(Odometry, '/slamware_ros_sdk_server_node/odom', self.on_odom, 10)
        self.create_subscription(Path, '/plan', self.on_path, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.control_loop)

    def on_scan(self, msg): self.scan, self.last_scan = msg, self.get_clock().now()
    def on_odom(self, msg): self.odom = msg
    def on_path(self, msg): self.global_path = msg

    def min_distance(self):
        if self.scan is None: return math.inf
        vals = [x for x in self.scan.ranges if math.isfinite(x) and self.scan.range_min <= x <= self.scan.range_max]
        return min(vals, default=math.inf)

    def stop(self): self.cmd_pub.publish(Twist())
    def control_loop(self):
        age = (self.get_clock().now() - self.last_scan).nanoseconds / 1e9
        d = self.min_distance()
        if age > self.scan_timeout or d <= self.d_stop:
            self.state = State.ESTOP
        if self.state == State.ESTOP:
            self.stop()
            if age <= self.scan_timeout and d > self.d_stop + 0.10: self.state = State.DECIDE
            return
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
        cmd = Twist(); cmd.linear.x = self.max_speed; self.cmd_pub.publish(cmd)


def main():
    rclpy.init(); node = UnifiedNavigationNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
