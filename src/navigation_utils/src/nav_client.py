#!/usr/bin/env python3
"""
Nav2 导航客户端封装
提供统一的导航接口，支持：
- 单点导航 (NavigateToPose)
- 多点导航 (NavigateThroughPoses)
- 导航状态监控
- 导航取消
- 超时控制
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future

import math
import time
from typing import Optional, List, Callable, Dict, Any
from enum import IntEnum

# ROS 2 消息类型
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion,Twist
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

# 自定义消息
from competition_interfaces.msg import NavigationStatus, NavigationFeedback
# __init__.py
from obstacle_avoider import ObstacleAvoider, AvoidanceState


class NavState(IntEnum):
    """导航状态枚举"""
    IDLE = 0            # 空闲
    NAVIGATING = 1      # 导航中
    SUCCEEDED = 2       # 成功
    FAILED = 3          # 失败
    CANCELLED = 4       # 已取消
    TIMEOUT = 5         # 超时


class NavClient:
    """
    Nav2 导航客户端封装
    提供同步和异步导航接口
    """
    
    def __init__(self, node: Node):
        """
   #====     初始化导航客户端
        
        参数:
            node: ROS 2 节点实例
        """
        self.node = node
        self.logger = node.get_logger()
        
        # 回调组
        self.mutex_group = MutuallyExclusiveCallbackGroup()
        self.reentrant_group = ReentrantCallbackGroup()
        
        # ---- 创建 Action 客户端 ----
        self.nav_to_pose_client = ActionClient(
            node,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.reentrant_group
        )
        
        
        self.nav_through_poses_client = ActionClient(
            node,
            NavigateThroughPoses,
            'navigate_through_poses',
            callback_group=self.reentrant_group
        )
        
        # ---- 初始化避障器（默认启用） ----
        self.obstacle_avoider = ObstacleAvoider(node)
        
        # ---- 新增：避障启用标志 ----
        self.avoidance_enabled = False  # 默认关闭


        # ---- 状态变量 ----
        self.current_goal = None
        self.current_state = NavState.IDLE
        self.goal_handle = None
        self.result_future = None
        self.feedback_callback = None
        
        # ---- 状态发布 ----
        self.status_pub = node.create_publisher(
            NavigationStatus,
            '/navigation_utils/status',
            10
        )
        self.feedback_pub = node.create_publisher(
            NavigationFeedback,
            '/navigation_utils/feedback',
            10
        )
        
        # ---- 等待服务器就绪 ----
        node.get_logger().info('等待 Nav2 服务器就绪...')
        self.wait_for_servers()
        
        node.get_logger().info('NavClient 初始化完成')
    
    def wait_for_servers(self, timeout: float = 10.0) -> bool:
        """等待所有 Nav2 服务器就绪"""
        start = time.time()
        while time.time() - start < timeout:
            if self.nav_to_pose_client.wait_for_server(timeout_sec=0.5):
                return True
            self.node.get_logger().info('等待 Nav2 服务器...')
        return False

        
        #=======================避障接口======================
    def enable_avoidance(self):
        """启用避障（取货→送货区时调用）"""
        self.avoidance_enabled = True
        self.logger.info(' 避障策略已启用')
    
    def disable_avoidance(self):
        """禁用避障（其他导航时调用）"""
        self.avoidance_enabled = False
        self.logger.info(' 避障策略已禁用')
    
    def set_avoidance(self, enabled: bool):
        """设置避障开关"""
        if enabled:
            self.enable_avoidance()
        else:
            self.disable_avoidance()
    
    def is_avoidance_enabled(self) -> bool:
        """检查避障是否启用"""
        return self.avoidance_enabled

       # ==================== 导航接口 ====================
    def _navigate_generic(
        self,
        goal_msg: Any,
        client: ActionClient,
        timeout: float,
        feedback_callback: Optional[Callable] = None,
        is_through: bool = False
     ) -> NavState:
        nav_type = "多点" if is_through else "单点"
        self.logger.info(f'{nav_type}导航开始')
         
        self.current_state = NavState.NAVIGATING
        self.feedback_callback = feedback_callback
    
     # 发送目标
        send_future = client.send_goal_async(
        goal_msg,
        feedback_callback=self._feedback_callback
        )
    
     # 等待接受
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        result = send_future.result()
        if not result:
            return self._handle_failure("Goal send failed")
    
        self.goal_handle = result
        if not self.goal_handle.accepted:
            return self._handle_failure("Goal rejected")
    
      # 等待完成
        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, self.result_future, timeout_sec=timeout)
    
        result = self.result_future.result()
        if not result:
            return self._handle_failure("No result")
    
        if result.result:
            return self._handle_success(f"{nav_type}导航完成")
        else:
            return self._handle_failure(f"{nav_type}导航失败")
    def navigate_to_pose(
        self,
        pose: PoseStamped,
        timeout: float = 60.0,
        feedback_callback: Optional[Callable] = None
    ) -> NavState:
        """单点导航（同步）"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        return self._navigate_generic(
            goal_msg,
            self.nav_to_pose_client,
            timeout,
            feedback_callback,
            is_through=False
        )

    def navigate_through_poses(
        self,
        poses: List[PoseStamped],
        timeout: float = 120.0,
        feedback_callback: Optional[Callable] = None
    ) -> NavState:
        """多点导航（同步）"""
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        return self._navigate_generic(
            goal_msg, 
            self.nav_through_poses_client, 
            timeout, 
            feedback_callback, 
            is_through=True
            )
       # =========================避障查询接口=====================

    def get_avoidance_command(self, target_twist: Twist = None) -> Twist:
        """
        获取速度指令（根据避障开关决定是否处理）
        """
        if self.avoidance_enabled and self.obstacle_avoider:
            # 避障启用 → 应用避障策略
            return self.obstacle_avoider.compute_avoidance_command(target_twist)
        else:
            # 避障禁用 → 直接返回原始指令
            return target_twist if target_twist else Twist()
    
    def is_obstacle_detected(self) -> bool:
        """检查是否检测到障碍物（仅在避障启用时有效）"""
        if self.avoidance_enabled and self.obstacle_avoider:
            return self.obstacle_avoider.is_obstacle_detected()
        return False
    
    def get_obstacle_info(self) -> dict:
        """获取障碍物信息"""
        if self.avoidance_enabled and self.obstacle_avoider:
            return self.obstacle_avoider.get_obstacle_info()
        return {'detected': False, 'enabled': False}
        #==========================================================

    def cancel_navigation(self) -> bool:
        """取消当前导航"""
        if self.goal_handle and self.goal_handle.active:
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=2.0)
            self.current_state = NavState.CANCELLED
            self._publish_status(self.current_state, "Cancelled by user")
            self.logger.info('导航已取消')
            return True
        return False
    
    def is_navigating(self) -> bool:
        """检查是否正在导航"""
        return self.current_state == NavState.NAVIGATING
    
    def get_current_state(self) -> NavState:
        """获取当前导航状态"""
        return self.current_state


    def _handle_failure(self, message: str) -> NavState:
        """统一的失败处理"""
        self.current_state = NavState.FAILED
        self._publish_status(self.current_state, message)
        self.logger.error(message)
        return self.current_state

    def _handle_success(self, message: str) -> NavState:
        """统一的成功处理"""
        self.current_state = NavState.SUCCEEDED
        self._publish_status(self.current_state, message)
        self.logger.info(message)
        return self.current_state
    
    # ==================== 回调处理 ====================
    
    def _feedback_callback(self, feedback_msg):
        """处理导航反馈"""
        # 处理 NavigateToPose 或 NavigateThroughPoses 的反馈
        feedback = feedback_msg.feedback
        
        # 创建自定义反馈消息
        nav_feedback = NavigationFeedback()
        nav_feedback.state = int(self.current_state)
        nav_feedback.distance_remaining = getattr(feedback, 'distance_remaining', 0.0)
        nav_feedback.current_pose = getattr(feedback, 'current_pose', None)
        nav_feedback.estimated_time_remaining = getattr(feedback, 'estimated_time_remaining', 0.0)
        nav_feedback.timestamp = self.node.get_clock().now().to_msg()
        self.feedback_pub.publish(nav_feedback)
        
        # 调用用户回调
        if self.feedback_callback:
            self.feedback_callback(feedback_msg)
    
    # ==================== 状态发布 ====================
    
    def _publish_status(self, state: NavState, message: str):
        """发布导航状态"""
        status_msg = NavigationStatus()
        status_msg.state = int(state)
        status_msg.message = message
        status_msg.timestamp = self.node.get_clock().now().to_msg()
        self.status_pub.publish(status_msg)
    
    # ====================工具函数（创建导航目标） ====================
    
    @staticmethod
    def create_pose(x: float, y: float, yaw: float = 0.0, frame_id: str = 'map') -> PoseStamped:
        """
        创建位姿消息
        
        参数:
            x: X坐标 (米)
            y: Y坐标 (米)
            yaw: 朝向角 (弧度)
            frame_id: 坐标系名称
        
        返回:
            PoseStamped: 位姿消息
        """
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position = Point(x=x, y=y, z=0.0)
        
        # 欧拉角转四元数
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        
        return pose
    
    @staticmethod
    def create_poses_from_list(pose_list: List[tuple], frame_id: str = 'map') -> List[PoseStamped]:
        """
        从坐标列表创建位姿列表
        
        参数:
            pose_list: [(x, y, yaw), ...] 坐标列表
            frame_id: 坐标系名称
        
        返回:
            List[PoseStamped]: 位姿列表
        """
        poses = []
        for x, y, yaw in pose_list:
            poses.append(NavClient.create_pose(x, y, yaw, frame_id))
        return poses