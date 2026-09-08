"""独立的货架图像采集状态机骨架。

只负责调度、激光靠近、固定距离移动和图像落盘；不包含抓取或 YOLO 代码。
"""

import json
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


class Stage:
    IDLE = 'IDLE'
    APPROACH = 'LASER_APPROACH'
    CAPTURE = 'CAPTURE'
    MOVE_NEXT = 'MOVE_NEXT_COLUMN'
    NEXT_SHELF = 'NEXT_SHELF'
    FINISHED = 'FINISHED'
    ERROR = 'ERROR'


class CaptureScheduler(Node):
    def __init__(self):
        super().__init__('supermarket_capture')
        self.declare_parameters('', [
            ('output_dir', 'captures'), ('shelf_count', 5),
            ('column_count', 3), ('level_count', 3),
            ('level_heights_m', [0.50, 0.85, 1.19]),
            ('stop_distance', 0.75), ('approach_speed', 0.10),
            ('front_angle_deg', 10.0), ('stop_confirm_count', 3),
            ('column_spacing', 0.45), ('camera_settle_time', 0.5),
            ('approach_timeout', 60.0), ('move_timeout', 30.0),
        ])
        self.shelf_count = int(self.get_parameter('shelf_count').value)
        self.column_count = int(self.get_parameter('column_count').value)
        self.level_count = int(self.get_parameter('level_count').value)
        self.level_heights_m = [float(v) for v in self.get_parameter('level_heights_m').value]
        if len(self.level_heights_m) != self.level_count:
            raise ValueError('level_heights_m 长度必须等于 level_count')
        self.total_columns = self.shelf_count * self.column_count
        root = Path(str(self.get_parameter('output_dir').value))
        run = datetime.now().strftime('run_%Y%m%d_%H%M%S')
        self.run_dir = root / run
        self.image_dir = self.run_dir / 'images'
        self.image_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_scan = None
        self.odom = None
        self.stage = Stage.IDLE
        self.shelf = self.column = self.level = 0
        self.global_column = 0
        self.stop_count = 0
        self.move_origin = None
        self.stage_started = time.monotonic()
        self.paths = []
        self.manifest = []
        self.stage_pub = self.create_publisher(String, '/supermarket_capture/stage', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/slamware_ros_sdk_server_node/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/slamware_ros_sdk_server_node/odom', self.odom_cb, 10)
        self.create_subscription(Image, '/head_camera/color/image_raw', self.image_cb, 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.set_stage(Stage.APPROACH)

    def scan_cb(self, msg):
        self.latest_scan = msg

    def odom_cb(self, msg):
        self.odom = msg.pose.pose.position

    def image_cb(self, msg):
        self.latest_image = msg

    def set_stage(self, stage):
        self.stage, self.stage_started = stage, time.monotonic()
        self.stage_pub.publish(String(data=stage))
        self.get_logger().info(
            f'stage={stage} shelf={self.shelf + 1} '
            f'column={self.column + 1} level={self.level + 1}'
        )

    def stop(self):
        self.cmd_pub.publish(Twist())

    def front_distance(self):
        if self.latest_scan is None:
            return None
        half = math.radians(float(self.get_parameter('front_angle_deg').value))
        vals = []
        for i, value in enumerate(self.latest_scan.ranges):
            angle = self.latest_scan.angle_min + i * self.latest_scan.angle_increment
            if abs(angle) <= half and math.isfinite(value) and self.latest_scan.range_min <= value <= self.latest_scan.range_max:
                vals.append(value)
        return sorted(vals)[len(vals) // 2] if vals else None

    def tick(self):
        if self.stage == Stage.APPROACH:
            distance = self.front_distance()
            if time.monotonic() - self.stage_started > float(self.get_parameter('approach_timeout').value):
                self.fail('激光靠近超时')
            elif distance is not None and distance <= float(self.get_parameter('stop_distance').value):
                self.stop_count += 1
                if self.stop_count >= int(self.get_parameter('stop_confirm_count').value):
                    self.stop(); self.set_stage(Stage.CAPTURE)
            else:
                self.stop_count = 0
                cmd = Twist(); cmd.linear.x = float(self.get_parameter('approach_speed').value); self.cmd_pub.publish(cmd)
        elif self.stage == Stage.CAPTURE:
            if self.latest_image is not None and time.monotonic() - self.stage_started >= float(self.get_parameter('camera_settle_time').value):
                if not self.save_image():
                    return
                if self.level < self.level_count:
                    self.set_stage(Stage.CAPTURE)
                elif self.global_column + 1 < self.total_columns:
                    self.set_stage(Stage.MOVE_NEXT)
                else:
                    self.stop()
                    self.write_manifest()
                    self.set_stage(Stage.FINISHED)
                    self.timer.cancel()
        elif self.stage == Stage.MOVE_NEXT:
            if self.move_origin is None and self.odom is not None:
                self.move_origin = (self.odom.x, self.odom.y)
            if self.move_origin and self.odom:
                distance = math.hypot(self.odom.x - self.move_origin[0], self.odom.y - self.move_origin[1])
                if distance >= float(self.get_parameter('column_spacing').value):
                    self.stop()
                    self.global_column += 1
                    self.shelf = self.global_column // self.column_count
                    self.column = self.global_column % self.column_count
                    self.level = 0
                    self.move_origin = None
                    self.set_stage(Stage.CAPTURE)
                else:
                    cmd = Twist(); cmd.linear.x = float(self.get_parameter('approach_speed').value); self.cmd_pub.publish(cmd)
            if time.monotonic() - self.stage_started > float(self.get_parameter('move_timeout').value):
                self.fail('列间移动超时')
        elif self.stage == Stage.NEXT_SHELF:
            # 五组货架连成一排：组间继续沿同一方向移动，不再次靠近。
            self.set_stage(Stage.MOVE_NEXT)

    def save_image(self):
        cv = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        name = f'shelf_{self.shelf + 1:02d}_col_{self.column + 1:02d}_level_{self.level + 1:02d}.jpg'
        path = self.image_dir / name
        import cv2
        if not cv2.imwrite(str(path), cv):
            self.fail(f'图片保存失败: {path}')
            return False
        rel = str(path.relative_to(self.run_dir))
        self.paths.append(str(path)); self.manifest.append({'shelf': self.shelf + 1, 'column': self.column + 1, 'level': self.level + 1, 'height_m': self.level_heights_m[self.level], 'path': rel, 'timestamp': time.time()})
        self.level += 1
        return True

    def write_manifest(self):
        (self.run_dir / 'manifest.json').write_text(json.dumps({'run_id': self.run_dir.name, 'count': len(self.paths), 'images': self.manifest}, ensure_ascii=False, indent=2), encoding='utf-8')

    def fail(self, message):
        self.stop(); self.get_logger().error(message); self.set_stage(Stage.ERROR); self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = CaptureScheduler()
    try:
        rclpy.spin(node)
    finally:
        node.stop(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
