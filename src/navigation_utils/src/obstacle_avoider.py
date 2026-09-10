#!/usr/bin/env python3
"""
动态避障策略模块
基于激光雷达数据实现实时避障，与Nav2协同工作
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import math
import numpy as np
from typing import Optional, Tuple, List, Dict, Any
from enum import IntEnum

# ROS 2 消息类型
from sensor_msgs.msg import LaserScan, Image, JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

# 自定义消息
from competition_interfaces.msg import ObstacleInfo, AvoidanceCommand


class AvoidanceState(IntEnum):
    """避障状态"""
    NORMAL = 0          # 正常行驶
    DETECTING = 1       # 检测到障碍物
    AVOIDING = 2        # 避障中
    RECOVERING = 3      # 恢复中
    DISABLED = 4        # 禁用状态


class ObstacleAvoider:
    """
    动态避障器
    基于激光雷达数据实现实时避障
    """
    
    def __init__(self, node: Node):
        """
        初始化避障器
        
        参数:
            node: ROS 2 节点实例
        """
        self.node = node
        self.logger = node.get_logger()
        
        # 回调组
        self.mutex_group = MutuallyExclusiveCallbackGroup()
        self.reentrant_group = ReentrantCallbackGroup()
        
        # ---- 订阅传感器数据（使用你的话题） ----
        # 1. 激光雷达（主要避障）
        self.scan_sub = node.create_subscription(
            LaserScan,
            '/slamware_ros_sdk_server_node/scan',  # ← 修改为实际话题
            self.scan_callback,
            10,
            callback_group=self.reentrant_group
        )
        
        # 2. 里程计（速度/位置监控）
        self.odom_sub = node.create_subscription(
            Odometry,
            '/slamware_ros_sdk_server_node/odom',  # ← 修改为实际话题
            self.odom_callback,
            10,
            callback_group=self.reentrant_group
        )
        
        # 3. 头部深度相机（增强避障，检测低矮障碍物）
        self.depth_sub = node.create_subscription(
            Image,
            '/head_camera/aligned_depth_to_color/image_raw',  # ← 新增深度相机
            self.depth_callback,
            10,
            callback_group=self.reentrant_group
        )
        
        # 4. 关节状态（可选，用于检测机械臂状态）
        self.joint_sub = node.create_subscription(
            JointState,
            '/joint_states',  # ← 新增关节状态
            self.joint_callback,
            10,
            callback_group=self.reentrant_group
        )
        
        # ---- 发布速度指令（避障控制） ----
        self.cmd_pub = node.create_publisher(
            Twist,
            '/cmd_vel_avoidance',
            10
        )
        
        # ---- 发布避障信息 ----
        self.obstacle_pub = node.create_publisher(
            ObstacleInfo,
            '/navigation_utils/obstacle_info',
            10
        )
        
        # ---- 状态变量 ----
        self.latest_scan = None
        self.current_odom = None
        self.latest_depth = None
        self.latest_joint = None
        
        self.avoidance_state = AvoidanceState.NORMAL
        self.obstacle_direction = None  # 'left', 'right', 'front'
        self.obstacle_distance = float('inf')
        
        # 深度相机障碍物信息
        self.depth_obstacle_detected = False
        self.depth_obstacle_distance = float('inf')
        
        # ---- 避障启用标志 ----
        self._enabled = False  # 默认禁用，由 NavClient 控制

        # ---- 参数（适配你的传感器） ----
        self.load_parameters()
        
        # ---- 状态发布定时器 ----
        self.status_timer = node.create_timer(
            0.1,
            self.publish_obstacle_info,
            callback_group=self.mutex_group
        )
        
        node.get_logger().info(' ObstacleAvoider 初始化完成')
        node.get_logger().info(f' 订阅话题: /slamware_ros_sdk_server_node/scan')
        node.get_logger().info(f' 订阅话题: /slamware_ros_sdk_server_node/odom')
        node.get_logger().info(f' 订阅话题: /head_camera/aligned_depth_to_color/image_raw')
    
    # ============= 启用/禁用控制 =============
    
    def enable(self):
        """启用避障"""
        self._enabled = True
        self.avoidance_state = AvoidanceState.NORMAL
        self.logger.info(' ObstacleAvoider 已启用')
    
    def disable(self):
        """禁用避障"""
        self._enabled = False
        self.avoidance_state = AvoidanceState.DISABLED
        self.obstacle_direction = None
        self.obstacle_distance = float('inf')
        self.logger.info(' ObstacleAvoider 已禁用')
    
    def is_enabled(self) -> bool:
        """检查是否启用"""
        return self._enabled

    def load_parameters(self):
        """加载配置参数（适配实际传感器）"""
        # 激光雷达参数
        self.node.declare_parameter('obstacle.safe_distance', 0.8)
        self.node.declare_parameter('obstacle.avoid_distance', 0.5)
        self.node.declare_parameter('obstacle.front_threshold', 0.6)
        self.node.declare_parameter('obstacle.side_threshold', 0.8)
        self.node.declare_parameter('obstacle.avoid_angle', 0.5)
        self.node.declare_parameter('obstacle.avoid_speed', 0.2)
        self.node.declare_parameter('obstacle.recover_time', 1.0)
        
        # 深度相机参数（新增）
        self.node.declare_parameter('obstacle.depth_enabled', True)
        self.node.declare_parameter('obstacle.depth_threshold', 0.5)  # 50cm
        self.node.declare_parameter('obstacle.depth_roi_size', 20)    # ROI大小
        
        # 获取参数
        self.safe_distance = self.node.get_parameter('obstacle.safe_distance').value
        self.avoid_distance = self.node.get_parameter('obstacle.avoid_distance').value
        self.front_threshold = self.node.get_parameter('obstacle.front_threshold').value
        self.side_threshold = self.node.get_parameter('obstacle.side_threshold').value
        self.avoid_angle = self.node.get_parameter('obstacle.avoid_angle').value
        self.avoid_speed = self.node.get_parameter('obstacle.avoid_speed').value
        self.recover_time = self.node.get_parameter('obstacle.recover_time').value
        
        self.depth_enabled = self.node.get_parameter('obstacle.depth_enabled').value
        self.depth_threshold = self.node.get_parameter('obstacle.depth_threshold').value
        self.depth_roi_size = self.node.get_parameter('obstacle.depth_roi_size').value
    
    # ==================== 回调函数 ====================
    
    def scan_callback(self, msg: LaserScan):
        """激光雷达回调"""
        self.latest_scan = msg
        self.detect_obstacles_lidar(msg)
    
    def odom_callback(self, msg: Odometry):
        """里程计回调"""
        self.current_odom = msg
        
        # 检测是否被卡住（结合避障状态）
        if self.obstacle_direction is not None:
            current_speed = msg.twist.twist.linear.x
            if abs(current_speed) < 0.01:
                # 可能被卡住，加强避障
                self.avoidance_state = AvoidanceState.RECOVERING
    
    def depth_callback(self, msg: Image):
        """头部深度相机回调（新增）"""
        self.latest_depth = msg
        if self.depth_enabled:
            self.detect_obstacles_depth(msg)
    
    def joint_callback(self, msg: JointState):
        """关节状态回调（新增）"""
        self.latest_joint = msg
        # 可用于检测机械臂是否伸出，影响避障策略
    
    # ==================== 障碍物检测（激光雷达） ====================
    
    def detect_obstacles_lidar(self, scan: LaserScan) -> Dict[str, Any]:
        """
        使用激光雷达检测障碍物
        
        返回:
            dict: 障碍物信息
        """
        if scan is None or len(scan.ranges) == 0:
            return {'detected': False}
        
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        
        # 滤除无效数据
        valid_mask = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) == 0:
            return {'detected': False}
        
        # 分区：左、前、右
        front_mask = (valid_angles > -self.front_threshold) & (valid_angles < self.front_threshold)
        left_mask = (valid_angles > self.front_threshold) & (valid_angles < math.pi/2)
        right_mask = (valid_angles < -self.front_threshold) & (valid_angles > -math.pi/2)
        
        # 检测前方障碍物
        front_ranges = valid_ranges[front_mask]
        if len(front_ranges) > 0:
            min_front = np.min(front_ranges)
            if min_front < self.safe_distance:
                self.obstacle_direction = 'front'
                self.obstacle_distance = min_front
                self.avoidance_state = AvoidanceState.AVOIDING
                return {
                    'detected': True,
                    'direction': 'front',
                    'distance': min_front,
                    'angle': 0.0,
                    'sensor': 'lidar'
                }
        
        # 检测左侧障碍物
        left_ranges = valid_ranges[left_mask]
        if len(left_ranges) > 0:
            min_left = np.min(left_ranges)
            if min_left < self.side_threshold:
                self.obstacle_direction = 'left'
                self.obstacle_distance = min_left
                self.avoidance_state = AvoidanceState.AVOIDING
                return {
                    'detected': True,
                    'direction': 'left',
                    'distance': min_left,
                    'angle': valid_angles[left_mask][np.argmin(left_ranges)],
                    'sensor': 'lidar'
                }
        
        # 检测右侧障碍物
        right_ranges = valid_ranges[right_mask]
        if len(right_ranges) > 0:
            min_right = np.min(right_ranges)
            if min_right < self.side_threshold:
                self.obstacle_direction = 'right'
                self.obstacle_distance = min_right
                self.avoidance_state = AvoidanceState.AVOIDING
                return {
                    'detected': True,
                    'direction': 'right',
                    'distance': min_right,
                    'angle': valid_angles[right_mask][np.argmin(right_ranges)],
                    'sensor': 'lidar'
                }
        
        # 没有检测到障碍物（但检查深度相机）
        if self.depth_obstacle_detected:
            # 使用深度相机的检测结果
            self.obstacle_direction = 'front'
            self.obstacle_distance = self.depth_obstacle_distance
            self.avoidance_state = AvoidanceState.AVOIDING
            return {
                'detected': True,
                'direction': 'front',
                'distance': self.depth_obstacle_distance,
                'angle': 0.0,
                'sensor': 'depth'
            }
        
        self.obstacle_direction = None
        self.obstacle_distance = float('inf')
        self.avoidance_state = AvoidanceState.NORMAL
        return {'detected': False}
    
    # ==================== 障碍物检测（深度相机） ====================
    
    def detect_obstacles_depth(self, msg: Image):
        """
        使用深度相机检测障碍物
        检测前方低矮物体（激光雷达盲区）
        """
        try:
            # 转换为numpy数组（深度数据为uint16，单位毫米）
            depth_array = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width
            )
            
            # 取中心区域ROI
            h, w = msg.height // 2, msg.width // 2
            roi_size = self.depth_roi_size
            roi = depth_array[h-roi_size:h+roi_size, w-roi_size:w+roi_size]
            
            # 提取有效数据（0表示无效）
            valid = roi[roi > 0]
            
            if len(valid) > 0:
                # 计算平均距离和最小距离
                avg_depth = np.mean(valid) / 1000.0  # 转换为米
                min_depth = np.min(valid) / 1000.0
                
                # 如果深度小于阈值，检测到障碍物
                if min_depth < self.depth_threshold:
                    self.depth_obstacle_detected = True
                    self.depth_obstacle_distance = min_depth
                    self.logger.debug(f' 深度相机检测到障碍物: {min_depth:.2f}m')
                    return
                
                # 如果深度逐渐接近，预警
                if avg_depth < self.depth_threshold * 1.5:
                    self.logger.debug(f' 前方物体接近: {avg_depth:.2f}m')
            
            # 没有检测到障碍物
            self.depth_obstacle_detected = False
            self.depth_obstacle_distance = float('inf')
            
        except Exception as e:
            self.logger.error(f' 深度数据处理失败: {e}')
    
    # ==================== 避障控制 ====================
    
    def compute_avoidance_command(self, target_twist: Twist = None) -> Twist:
        """
        计算避障控制指令
        
        参数:
            target_twist: 目标速度指令（来自规划器）
        
        返回:
            Twist: 避障后的速度指令
        """
        # 如果未启用，直接返回原始指令
        if not self._enabled:
            return target_twist if target_twist else Twist()

        # 如果没有检测到障碍物，返回原始指令
        if self.obstacle_direction is None:
            self.avoidance_state = AvoidanceState.NORMAL
            return target_twist if target_twist else Twist()
        
        # 根据障碍物方向生成避障指令
        cmd = Twist()
        self.avoidance_state = AvoidanceState.AVOIDING
        
        # 获取当前速度（用于判断状态）
        current_speed = 0.0
        if self.current_odom:
            current_speed = self.current_odom.twist.twist.linear.x
        
        if self.obstacle_direction == 'front':
            # 前方有障碍物
            if self.obstacle_distance < 0.3:
                # 紧急停止
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.logger.warning(f' 紧急停止！距离: {self.obstacle_distance:.2f}m')
            elif self.obstacle_distance < 0.5:
                # 减速并转向
                cmd.linear.x = 0.05
                # 选择转向方向
                if self._is_left_clear():
                    cmd.angular.z = self.avoid_angle
                elif self._is_right_clear():
                    cmd.angular.z = -self.avoid_angle
                else:
                    cmd.linear.x = -0.1  # 后退
            else:
                # 轻微减速
                cmd.linear.x = max(0.1, target_twist.linear.x * 0.5)
                if self._is_left_clear():
                    cmd.angular.z = self.avoid_angle * 0.5
                elif self._is_right_clear():
                    cmd.angular.z = -self.avoid_angle * 0.5
            
            self.logger.debug(f' 前方障碍物: {self.obstacle_distance:.2f}m')
            
        elif self.obstacle_direction == 'left':
            # 左侧有障碍物，向右避让
            cmd.linear.x = target_twist.linear.x * 0.7 if target_twist else 0.1
            cmd.angular.z = -self.avoid_angle * 0.6
            self.logger.debug(f' 左侧障碍物: {self.obstacle_distance:.2f}m')
            
        elif self.obstacle_direction == 'right':
            # 右侧有障碍物，向左避让
            cmd.linear.x = target_twist.linear.x * 0.7 if target_twist else 0.1
            cmd.angular.z = self.avoid_angle * 0.6
            self.logger.debug(f' 右侧障碍物: {self.obstacle_distance:.2f}m')
        
        return cmd
    
    def publish_avoidance_command(self, target_twist: Twist = None) -> Twist:
        """
    计算并发布避障控制指令
    
    参数:
        target_twist: 目标速度指令
    
    返回:
        Twist: 避障后的速度指令
    """
        cmd = self.compute_avoidance_command(target_twist)
        self.cmd_pub.publish(cmd)
        return cmd
     

    def _is_left_clear(self) -> bool:
        """检查左侧是否畅通"""
        if self.latest_scan is None:
            return True
        
        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(self.latest_scan.angle_min, self.latest_scan.angle_max, len(ranges))
        
        left_mask = (angles > 0.3) & (angles < 1.0)
        left_ranges = ranges[left_mask]
        
        if len(left_ranges) == 0:
            return True
        
        min_left = np.min(left_ranges)
        return min_left > self.side_threshold
    
    def _is_right_clear(self) -> bool:
        """检查右侧是否畅通"""
        if self.latest_scan is None:
            return True
        
        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(self.latest_scan.angle_min, self.latest_scan.angle_max, len(ranges))
        
        right_mask = (angles < -0.3) & (angles > -1.0)
        right_ranges = ranges[right_mask]
        
        if len(right_ranges) == 0:
            return True
        
        min_right = np.min(right_ranges)
        return min_right > self.side_threshold
    
    # ==================== 状态发布 ====================
    
    def publish_obstacle_info(self):
        """发布障碍物信息"""
        info = ObstacleInfo()
        info.detected = self.obstacle_direction is not None
        info.direction = self.obstacle_direction if self.obstacle_direction else 'none'
        info.distance = float(self.obstacle_distance)
        info.state = int(self.avoidance_state)
        info.timestamp = self.node.get_clock().now().to_msg()
        self.obstacle_pub.publish(info)
    
    def get_avoidance_state(self) -> AvoidanceState:
        """获取当前避障状态"""
        return self.avoidance_state
    
    def get_obstacle_info(self) -> Dict[str, Any]:
        """获取当前障碍物信息"""
        info = {
            'detected': self.obstacle_direction is not None,
            'direction': self.obstacle_direction,
            'distance': self.obstacle_distance,
            'state': int(self.avoidance_state)
        }
        
        # 添加深度相机信息
        if self.depth_enabled:
            info['depth_detected'] = self.depth_obstacle_detected
            info['depth_distance'] = self.depth_obstacle_distance
        
        # 添加里程计信息
        if self.current_odom:
            info['current_speed'] = self.current_odom.twist.twist.linear.x
        
        return info
    
    def is_obstacle_detected(self) -> bool:
        """检查是否检测到障碍物"""
        return self.obstacle_direction is not None