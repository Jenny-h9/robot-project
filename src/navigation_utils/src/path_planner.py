#!/usr/bin/env python3
"""
路径规划优化模块
提供路径平滑、路径裁剪、路径检查等辅助功能
"""

import rclpy
from rclpy.node import Node

import math
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
from enum import IntEnum

# ROS 2 消息类型
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point


class PathPlanner:
    """
    路径规划优化器
    提供路径平滑、裁剪、检查等功能
    """
    
    def __init__(self, node: Node):
        """
        初始化路径规划器
        
        参数:
            node: ROS 2 节点实例
        """
        self.node = node
        self.logger = node.get_logger()
        
        # 参数
        node.declare_parameter('path_planner.max_curvature', 0.5)
        node.declare_parameter('path_planner.waypoint_distance', 0.3)
        node.declare_parameter('path_planner.smooth_iterations', 10)
        node.declare_parameter('path_planner.min_path_length', 0.5)
        
        self.max_curvature = node.get_parameter('path_planner.max_curvature').value
        self.waypoint_distance = node.get_parameter('path_planner.waypoint_distance').value
        self.smooth_iterations = node.get_parameter('path_planner.smooth_iterations').value
        self.min_path_length = node.get_parameter('path_planner.min_path_length').value
        
        node.get_logger().info('PathPlanner 初始化完成')
    
    # ==================== 路径平滑 ====================
    
    def smooth_path(self, path: Path, iterations: int = None) -> Path:
        """
        平滑路径（使用简单平滑算法）
        
        参数:
            path: 原始路径
            iterations: 迭代次数
        
        返回:
            Path: 平滑后的路径
        """
        if iterations is None:
            iterations = self.smooth_iterations
        
        if len(path.poses) < 3:
            return path
        
        # 提取点坐标
        points = self._path_to_points(path)
        
        # 平滑处理
        smoothed = self._smooth_points(points, iterations)
        
        # 转换为路径消息
        smoothed_path = Path()
        smoothed_path.header = path.header
        for point in smoothed:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position = Point(x=point[0], y=point[1], z=0.0)
            smoothed_path.poses.append(pose)
        
        return smoothed_path
    
    def _path_to_points(self, path: Path) -> List[Tuple[float, float]]:
        """将路径转换为点列表"""
        points = []
        for pose in path.poses:
            points.append((pose.pose.position.x, pose.pose.position.y))
        return points
    
    def _smooth_points(self, points: List[Tuple[float, float]], iterations: int) -> List[Tuple[float, float]]:
        """
        平滑点序列（使用平均滤波）
        
        参数:
            points: 原始点列表
            iterations: 迭代次数
        
        返回:
            List[Tuple]: 平滑后的点列表
        """
        if len(points) < 3:
            return points
        
        smoothed = [list(p) for p in points]
        
        for _ in range(iterations):
            for i in range(1, len(smoothed) - 1):
                # 计算相邻点的平均
                smoothed[i][0] = (smoothed[i-1][0] + smoothed[i][0] + smoothed[i+1][0]) / 3.0
                smoothed[i][1] = (smoothed[i-1][1] + smoothed[i][1] + smoothed[i+1][1]) / 3.0
        
        return [(p[0], p[1]) for p in smoothed]
    
    # ==================== 路径裁剪 ====================
    
    def crop_path(self, path: Path, start_idx: int = 0, end_idx: int = None) -> Path:
        """
        裁剪路径
        
        参数:
            path: 原始路径
            start_idx: 起始索引
            end_idx: 结束索引
        
        返回:
            Path: 裁剪后的路径
        """
        if end_idx is None:
            end_idx = len(path.poses)
        
        if start_idx >= len(path.poses) or end_idx <= start_idx:
            return path
        
        cropped_path = Path()
        cropped_path.header = path.header
        cropped_path.poses = path.poses[start_idx:end_idx]
        return cropped_path
    
    def crop_path_by_distance(self, path: Path, distance: float) -> Path:
        """
        按距离裁剪路径
        
        参数:
            path: 原始路径
            distance: 裁剪距离（从起点算起）
        
        返回:
            Path: 裁剪后的路径
        """
        if len(path.poses) < 2:
            return path
        
        # 计算累计距离
        cum_dist = 0.0
        cut_idx = 0
        prev_point = (path.poses[0].pose.position.x, path.poses[0].pose.position.y)
        
        for i in range(1, len(path.poses)):
            point = (path.poses[i].pose.position.x, path.poses[i].pose.position.y)
            cum_dist += self._distance(prev_point, point)
            if cum_dist >= distance:
                cut_idx = i
                break
            prev_point = point
        
        if cut_idx == 0:
            return path
        
        return self.crop_path(path, 0, cut_idx + 1)
    
    # ==================== 路径检查 ====================
    
    def check_path_validity(self, path: Path) -> Dict[str, Any]:
        """
        检查路径有效性
        
        参数:
            path: 待检查的路径
        
        返回:
            dict: {
                'valid': bool,
                'length': float,
                'num_poses': int,
                'issues': list
            }
        """
        issues = []
        
        # 检查是否为空
        if not path.poses or len(path.poses) < 2:
            issues.append("Path has less than 2 poses")
            return {'valid': False, 'length': 0.0, 'num_poses': 0, 'issues': issues}
        
        # 计算路径长度
        total_length = 0.0
        for i in range(1, len(path.poses)):
            p1 = (path.poses[i-1].pose.position.x, path.poses[i-1].pose.position.y)
            p2 = (path.poses[i].pose.position.x, path.poses[i].pose.position.y)
            total_length += self._distance(p1, p2)
        
        # 检查路径长度
        if total_length < self.min_path_length:
            issues.append(f"Path too short: {total_length:.2f}m < {self.min_path_length}m")
        
        # 检查曲率（急转弯）
        for i in range(1, len(path.poses) - 1):
            curvature = self._calculate_curvature(
                path.poses[i-1].pose.position,
                path.poses[i].pose.position,
                path.poses[i+1].pose.position
            )
            if curvature > self.max_curvature:
                issues.append(f"High curvature at pose {i}: {curvature:.2f}")
        
        # 检查点间距
        for i in range(1, len(path.poses)):
            p1 = (path.poses[i-1].pose.position.x, path.poses[i-1].pose.position.y)
            p2 = (path.poses[i].pose.position.x, path.poses[i].pose.position.y)
            dist = self._distance(p1, p2)
            if dist > self.waypoint_distance * 3:
                issues.append(f"Large gap at pose {i}: {dist:.2f}m")
        
        return {
            'valid': len(issues) == 0,
            'length': total_length,
            'num_poses': len(path.poses),
            'issues': issues
        }
    
    def _distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """计算两点距离"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def _calculate_curvature(self, p1