# robot-project

面向智慧零售自主服务机器人的 ROS2 开发项目。当前仓库包含基础 ROS2 示例节点，以及一个独立的连通货架图像采集调度器，用于在仿真环境中完成货架定位、激光靠近、分层拍照和数据落盘。

## 当前功能

### `supermarket_capture`

- 支持 5 组连通货架，按连续 15 列遍历
- 每列拍摄 3 层，目标高度为 0.50 m、0.85 m、1.19 m
- 使用二维激光雷达判断与货架的距离，并连续 3 帧确认停车
- 使用 `/slamware_ros_sdk_server_node/odom` 判断相邻列之间约 0.45 m 的移动距离
- 使用头部 RGB 相机保存 JPG 图像
- 自动生成 `manifest.json`，记录货架、列、层、高度、路径和时间戳
- 异常时向 `/cmd_vel` 发布零速度

当前版本聚焦图像采集调度，不包含机械臂抓取、商品放置或 YOLO 训练流程。

## 目录结构

```text
src/
├── hello_robot/
│   └── ROS2 基础示例节点
└── supermarket_capture/
    ├── supermarket_capture/scheduler_node.py
    ├── config/capture.yaml
    ├── launch/capture.launch.py
    ├── package.xml
    ├── setup.py
    └── setup.cfg
```

## 环境要求

- Ubuntu 22.04 / WSL2
- ROS2 Humble
- Python 3
- `rclpy`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`cv_bridge`
- 仿真 Server 发布对应的 ROS2 传感器话题

## 构建

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

本项目实际使用的 WSL 工作区为 `/home/h9/robot_ws`；在比赛 Client 容器中对应挂载路径为 `/workspace/student`：

```bash
cd /home/h9/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 启动采集节点

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch supermarket_capture capture.launch.py
```

节点启动后默认自动进入 `LASER_APPROACH`。即使自动启动，只有仿真 Server 同时运行并发布新鲜的雷达、里程计、关节状态和头部 RGB 图像，才会继续运动。发生异常时节点自动停止并进入错误状态；重新开始任务需要重启节点。

## 使用的 ROS2 接口

### 订阅

| 用途 | 话题 | 类型 |
|---|---|---|
| 激光雷达 | `/slamware_ros_sdk_server_node/scan` | `sensor_msgs/msg/LaserScan` |
| 底盘位姿 | `/slamware_ros_sdk_server_node/odom` | `nav_msgs/msg/Odometry` |
| 头部 RGB | `/head_camera/color/image_raw` | `sensor_msgs/msg/Image` |

### 发布

| 用途 | 话题 | 类型 |
|---|---|---|
| 底盘控制 | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| 阶段调试 | `/supermarket_capture/stage` | `std_msgs/msg/String` |

状态阶段包括：

```text
LASER_APPROACH
CAPTURE
MOVE_NEXT_COLUMN
FINISHED
ERROR
```

## 配置

默认配置位于 `src/supermarket_capture/config/capture.yaml`，主要参数如下：

```yaml
stop_distance: 0.75
approach_speed: 0.80
stop_confirm_count: 3
column_spacing: 0.45
turn_speed: 0.30
level_heights_m: [0.50, 0.85, 1.19]
camera_settle_time: 0.5
```

列间距、停车距离、靠近速度和相机稳定时间均可在现场调试时修改。当前层高映射为目标高度 `[0.50, 0.85, 1.19] m` 对应升降柱值 `[0.69, 0.35, 0.00]`；机器人默认正对货架，按左转 90°、横向移动、右转回原始朝向遍历。头部控制数组按 `head_yaw_joint, head_pitch_joint` 顺序发送。

## 输出数据

每次运行会创建类似目录：

```text
captures/run_YYYYMMDD_HHMMSS/
├── images/
│   ├── shelf_01_col_01_level_01.jpg
│   └── ...
└── manifest.json
```

完整采集模式预期生成 45 张图片。`manifest.json` 保存每张图片的货架编号、列编号、层编号、目标高度、相对路径和时间戳。

## 仿真注意事项

- 五组货架被视为一排连通结构，机器人只在排端完成一次激光靠近。
- 机器人随后沿货架方向连续移动，不在组间重复靠近。
- `SUPERMARKET_TASKS=all` 适合 45 商品的开发压力测试；正式比赛任务数量由比赛 Server 下发，不应将 45 个货位硬编码为比赛任务。
- 随机障碍物场景需要进一步接入完整路径规划；当前节点的固定距离移动主要用于第一阶段联调。
- 没有新鲜有效的雷达、里程计、关节或图像数据时，节点保持零速度或等待，不使用旧缓存继续运动或拍照。
- 层高映射已按当前仿真约定配置为 `level_mapping_calibrated: true`；若更换模型，必须重新标定后再启用。
- 横移方向由 `shelf_traverse_direction` 配置为 `left` 或 `right`；横移时若正前方距离小于 `emergency_stop_distance`，立即进入错误状态。

## 开发状态

当前开发分支为 `dev`，已完成采集调度器的基础构建和启动验证。后续计划包括：

1. 接入实际货架导航目标和路径规划
2. 在更换模型时复核升降柱/头部姿态映射
3. 增加 ROS2 Action Result 返回图片路径列表
4. 基于采集并标注的图像训练兼容 `kele.pt` 的 Ultralytics YOLO 模型
