#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_client import NavClient

def main():
    rclpy.init()
    node = Node('test_nav')
    
    nav = NavClient(node)
    
    if not nav.wait_for_servers():
        print(" Nav2 服务器未就绪")
        return
    
    print("NavClient 初始化成功")
    
    # 创建一个目标点（在仿真中，假设地图原点附近）
    goal = NavClient.create_pose(1.0, 1.0, 0.0)
    
    print(f" 导航到: ({goal.pose.position.x}, {goal.pose.position.y})")
    result = nav.navigate_to_pose(goal, timeout=30.0)
    
    print(f" 导航结果: {result}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()