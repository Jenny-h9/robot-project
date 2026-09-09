"""安全优先的连通货架图像采集调度器。"""

import json
import math
import csv
import time
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, LaserScan
from std_msgs.msg import Float64MultiArray, String


class Stage:
    IDLE = 'IDLE'
    APPROACH = 'LASER_APPROACH'
    CAPTURE = 'CAPTURE'
    MOVE_NEXT = 'MOVE_NEXT_COLUMN'
    TURN_LEFT = 'TURN_LEFT_90'
    MOVE_LATERAL = 'MOVE_LATERAL'
    TURN_RIGHT = 'TURN_RIGHT_90'
    FINISHED = 'FINISHED'
    ERROR = 'ERROR'
    INTERRUPTED = 'INTERRUPTED'


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class CaptureScheduler(Node):
    def __init__(self):
        super().__init__('supermarket_capture')
        self.declare_parameters('', [
            ('output_dir', 'captures'), ('shelf_count', 5),
            ('column_count', 3), ('level_count', 3),
            ('connected_shelves', True),
            ('level_heights_m', [0.50, 0.85, 1.19]),
            # 控制器实际目标值需现场标定；height 仅用于记录和校验。
            ('spine_positions_m', [0.0, 0.35, 0.69]),
            ('head_pitch_positions_rad', [0.0, 0.0, 0.0]),
            ('head_yaw_position_rad', 0.0), ('joint_tolerance', 0.03),
            ('level_mapping_calibrated', False), ('shelf_traverse_direction', 'left'),
            ('stop_distance', 0.75), ('approach_speed', 0.08),
            ('approach_brake_distance', 1.50), ('approach_emergency_distance', 0.35),
            ('front_angle_deg', 20.0), ('stop_confirm_count', 3),
            ('column_spacing', 0.45), ('camera_settle_time', 0.5),
            ('joint_motion_timeout', 10.0), ('image_timeout', 3.0),
            ('capture_timeout', 20.0),
            ('turn_speed', 0.30), ('turn_tolerance_deg', 5.0),
            ('emergency_stop_distance', 0.30),
            ('sensor_timeout', 0.5), ('approach_timeout', 60.0),
            ('max_approach_distance', 3.0),
            ('move_timeout', 30.0), ('max_lateral_error', 0.12),
            ('max_heading_error_deg', 12.0), ('enabled', True),
        ])
        self.shelf_count = int(self.get_parameter('shelf_count').value)
        self.column_count = int(self.get_parameter('column_count').value)
        self.level_count = int(self.get_parameter('level_count').value)
        self.connected_shelves = bool(self.get_parameter('connected_shelves').value)
        self.level_heights_m = [float(v) for v in self.get_parameter('level_heights_m').value]
        self.spine_positions = [float(v) for v in self.get_parameter('spine_positions_m').value]
        self.head_pitch_positions = [float(v) for v in self.get_parameter('head_pitch_positions_rad').value]
        if not (len(self.level_heights_m) == len(self.spine_positions) == len(self.head_pitch_positions) == self.level_count):
            raise ValueError('层高、升降柱和头部目标值长度必须等于 level_count')
        if str(self.get_parameter('shelf_traverse_direction').value).lower() not in ('left', 'right'):
            raise ValueError('shelf_traverse_direction 只能是 left 或 right')
        if any(value < 0.0 or value > 0.91 for value in self.spine_positions):
            raise ValueError('spine_positions_m 必须位于升降柱 0.00~0.91m 行程内')
        if float(self.get_parameter('approach_speed').value) <= 0.0 or float(self.get_parameter('turn_speed').value) <= 0.0:
            raise ValueError('approach_speed 和 turn_speed 必须为正数')
        self.total_columns = self.shelf_count * self.column_count if self.connected_shelves else self.column_count
        root = Path(str(self.get_parameter('output_dir').value))
        run = datetime.now().strftime('run_%Y%m%d_%H%M%S')
        self.run_dir, self.image_dir = root / run, root / run / 'images'
        self.image_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.latest_scan = self.latest_odom = self.latest_image = self.latest_joints = None
        self.last_scan_mono = self.last_odom_mono = self.last_image_mono = self.last_joint_mono = None
        self.last_scan_stamp_ns = self.last_image_stamp_ns = 0
        self.front_distance_value = None
        self.front_min_distance_value = None
        self.scan_close_count = 0
        self.stage = Stage.IDLE
        self.enabled = bool(self.get_parameter('enabled').value)
        self.shelf = self.column = self.level = self.global_column = 0
        self.move_origin = self.move_heading = None
        self.capture_required_image_ns = 0
        self.capture_started = 0.0
        self.capture_pose_reached_at = None
        self.capture_image_deadline = None
        self.turn_target = None
        self.shelf_facing_heading = None
        self.capture_waiting_new = False
        self.paths, self.manifest = [], []
        self.failure_message = ''
        self.approach_origin = None
        self.control_log = self.run_dir / 'control_log.csv'
        with self.control_log.open('w', newline='', encoding='utf-8') as stream:
            csv.writer(stream).writerow(['time', 'stage', 'front_median_m', 'front_min_m', 'odom_x', 'odom_y', 'yaw_rad', 'cmd_linear_x', 'cmd_angular_z'])
        self.stage_pub = self.create_publisher(String, '/supermarket_capture/stage', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.spine_pub = self.create_publisher(Float64MultiArray, '/spine_forward_position_controller/commands', 10)
        self.head_pub = self.create_publisher(Float64MultiArray, '/head_forward_position_controller/commands', 10)
        self.create_subscription(LaserScan, '/slamware_ros_sdk_server_node/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/slamware_ros_sdk_server_node/odom', self.odom_cb, 10)
        self.create_subscription(Image, '/head_camera/color/image_raw', self.image_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.set_stage(Stage.IDLE)
        if self.enabled and bool(self.get_parameter('level_mapping_calibrated').value):
            self.set_stage(Stage.APPROACH)
            self.get_logger().info('auto-start enabled; sensor safety checks remain active')
        else:
            self.enabled = False
            self.get_logger().error('level mapping is not calibrated; refusing to move')

    def scan_cb(self, msg):
        now = time.monotonic()
        if self.last_scan_stamp_ns and stamp_ns(msg.header.stamp) <= self.last_scan_stamp_ns:
            return
        self.latest_scan, self.last_scan_mono = msg, now
        self.last_scan_stamp_ns = stamp_ns(msg.header.stamp)
        self.front_distance_value = self._front_distance(msg)
        self.front_min_distance_value = self._front_min_distance(msg)
        self.scan_close_count = self.scan_close_count + 1 if self.front_distance_value is not None and self.front_distance_value <= float(self.get_parameter('stop_distance').value) else 0

    def odom_cb(self, msg):
        self.latest_odom, self.last_odom_mono = msg, time.monotonic()

    def image_cb(self, msg):
        self.latest_image, self.last_image_mono = msg, time.monotonic()

    def joint_cb(self, msg):
        self.latest_joints, self.last_joint_mono = msg, time.monotonic()

    def set_stage(self, stage):
        self.stage, self.stage_started = stage, time.monotonic()
        try:
            self.stage_pub.publish(String(data=stage))
        except Exception:
            pass
        self.get_logger().info(f'stage={stage} shelf={self.shelf + 1} column={self.column + 1} level={self.level + 1}')
        if stage == Stage.CAPTURE:
            self.capture_started = time.monotonic()
            self.capture_pose_reached_at = None
            self.capture_image_deadline = self.capture_started + float(self.get_parameter('image_timeout').value)
            self.capture_required_image_ns = self.last_image_stamp_ns
            # 清除上一层缓存；必须等待相机送来新的 ROS 时间戳。
            self.latest_image = None
            self.capture_waiting_new = False
            self.publish_level_target()
        elif stage == Stage.APPROACH:
            self.approach_origin = None

    def stop(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    def _front_distance(self, scan):
        half = math.radians(float(self.get_parameter('front_angle_deg').value))
        vals = [v for i, v in enumerate(scan.ranges) if abs(scan.angle_min + i * scan.angle_increment) <= half and math.isfinite(v) and scan.range_min <= v <= scan.range_max]
        return sorted(vals)[len(vals) // 2] if vals else None

    def _front_min_distance(self, scan):
        half = math.radians(float(self.get_parameter('front_angle_deg').value))
        vals = [v for i, v in enumerate(scan.ranges) if abs(scan.angle_min + i * scan.angle_increment) <= half and math.isfinite(v) and scan.range_min <= v <= scan.range_max]
        return min(vals) if vals else None

    def sensors_fresh(self, require_joints=False):
        now, limit = time.monotonic(), float(self.get_parameter('sensor_timeout').value)
        ok = self.last_scan_mono is not None and self.front_distance_value is not None and now - self.last_scan_mono <= limit
        ok = ok and self.last_odom_mono is not None and now - self.last_odom_mono <= limit
        return ok and (not require_joints or (self.last_joint_mono is not None and now - self.last_joint_mono <= limit))

    def tick(self):
        if not self.enabled:
            self.stop(); return
        if self.stage == Stage.APPROACH:
            if time.monotonic() - self.stage_started > float(self.get_parameter('approach_timeout').value):
                self.fail('激光靠近超时')
            elif not self.sensors_fresh():
                self.stop()
            elif self.approach_origin is None:
                self.approach_origin = self.latest_odom.pose.pose.position
            elif self.front_min_distance_value <= float(self.get_parameter('emergency_stop_distance').value):
                self.fail('靠近阶段单帧触发前向紧急停车'); return
            elif self.approach_origin is not None and self.latest_odom is not None:
                p = self.latest_odom.pose.pose.position
                if math.hypot(p.x - self.approach_origin.x, p.y - self.approach_origin.y) > float(self.get_parameter('max_approach_distance').value):
                    self.fail('靠近阶段超过最大前进距离'); return
            elif self.scan_close_count >= int(self.get_parameter('stop_confirm_count').value):
                self.stop(); self.set_stage(Stage.CAPTURE)
            else:
                # 高速远距离靠近，进入刹车区后线性降速，减少控制延迟造成的冲过停车点。
                distance = self.front_distance_value
                if self.front_min_distance_value is not None and self.front_min_distance_value <= float(self.get_parameter('approach_emergency_distance').value):
                    self.fail('靠近阶段前方进入紧急停车距离'); return
                brake = float(self.get_parameter('approach_brake_distance').value)
                speed = float(self.get_parameter('approach_speed').value)
                if distance is not None and distance < brake:
                    stop_distance = float(self.get_parameter('stop_distance').value)
                    speed = 0.0 if distance <= stop_distance else speed * (distance - stop_distance) / max(0.01, brake - stop_distance)
                cmd = Twist(); cmd.linear.x = min(speed, float(self.get_parameter('approach_speed').value)); self.cmd_pub.publish(cmd); self.log_control(cmd)
        elif self.stage == Stage.CAPTURE:
            self.stop()
            now = time.monotonic()
            if now - self.capture_started > float(self.get_parameter('capture_timeout').value):
                self.fail('CAPTURE 阶段总超时'); return
            if now - self.capture_started > float(self.get_parameter('joint_motion_timeout').value) and self.capture_pose_reached_at is None:
                self.fail('关节到位超时'); return
            if not self.sensors_fresh(True) or not self.joints_at_level():
                # 姿态曾到位后又偏离时，必须重新到位并重新稳定。
                self.capture_pose_reached_at = None
                self.capture_waiting_new = False
                self.latest_image = None
                return
            if self.capture_pose_reached_at is None:
                self.capture_pose_reached_at = now; self.capture_image_deadline = now + float(self.get_parameter('image_timeout').value); return
            if now - self.capture_pose_reached_at < float(self.get_parameter('camera_settle_time').value): return
            if not self.capture_waiting_new:
                self.latest_image = None
                self.capture_required_image_ns = self.last_image_stamp_ns
                self.capture_waiting_new = True
                self.capture_image_deadline = now + float(self.get_parameter('image_timeout').value)
                return
            if self.latest_image is None or stamp_ns(self.latest_image.header.stamp) <= self.capture_required_image_ns:
                if now > self.capture_image_deadline: self.fail('稳定后未收到新图像')
                return
            if not self.save_image(): return
            if self.level + 1 < self.level_count:
                self.level += 1; self.set_stage(Stage.CAPTURE)
            elif self.global_column + 1 < self.total_columns:
                self.level = 0
                if self.shelf_facing_heading is None:
                    self.shelf_facing_heading = self.yaw_from_odom(self.latest_odom)
                direction = str(self.get_parameter('shelf_traverse_direction').value).lower()
                turn_sign = 1.0 if direction == 'left' else -1.0
                self.turn_target = self.normalize_angle(self.shelf_facing_heading + turn_sign * math.pi / 2.0)
                self.set_stage(Stage.TURN_LEFT)
            else:
                self.finish()
        elif self.stage in (Stage.TURN_LEFT, Stage.TURN_RIGHT):
            if time.monotonic() - self.stage_started > float(self.get_parameter('move_timeout').value): self.fail('转向超时'); return
            if not self.sensors_fresh(): self.stop(); return
            yaw = self.yaw_from_odom(self.latest_odom)
            err = math.atan2(math.sin(self.turn_target - yaw), math.cos(self.turn_target - yaw))
            if abs(math.degrees(err)) <= float(self.get_parameter('turn_tolerance_deg').value):
                self.stop()
                if self.stage == Stage.TURN_LEFT:
                    self.move_origin = None; self.move_heading = self.turn_target; self.set_stage(Stage.MOVE_LATERAL)
                else:
                    self.global_column += 1; self.shelf = self.global_column // self.column_count if self.connected_shelves else 0; self.column = self.global_column % self.column_count; self.move_origin = self.move_heading = None; self.set_stage(Stage.CAPTURE)
            else:
                # 根据角度误差决定旋转方向，避免越过目标后继续绕整圈。
                cmd = Twist(); cmd.angular.z = math.copysign(float(self.get_parameter('turn_speed').value), err); self.cmd_pub.publish(cmd)
        elif self.stage == Stage.MOVE_LATERAL:
            if time.monotonic() - self.stage_started > float(self.get_parameter('move_timeout').value): self.fail('横向移动超时'); return
            if not self.sensors_fresh(): self.stop(); return
            if self.move_origin is None:
                self.move_origin = self.latest_odom.pose.pose.position
                # move_heading 在左转完成时已保存为货架横向方向。
            if not self.odom_direction_ok(): self.fail('里程计方向/航向偏差超限'); return
            p = self.latest_odom.pose.pose.position
            dx, dy = p.x - self.move_origin.x, p.y - self.move_origin.y
            forward = dx * math.cos(self.move_heading) + dy * math.sin(self.move_heading)
            lateral = abs(-dx * math.sin(self.move_heading) + dy * math.cos(self.move_heading))
            if self.front_min_distance_value is not None and self.front_min_distance_value <= float(self.get_parameter('emergency_stop_distance').value):
                self.fail('横移时前方障碍物进入紧急停车距离')
            elif lateral > float(self.get_parameter('max_lateral_error').value) or forward < -0.05:
                self.fail('移动方向不符合预期')
            elif forward >= float(self.get_parameter('column_spacing').value):
                self.stop(); self.turn_target = self.shelf_facing_heading; self.set_stage(Stage.TURN_RIGHT)
            else:
                cmd = Twist(); cmd.linear.x = float(self.get_parameter('approach_speed').value); self.cmd_pub.publish(cmd); self.log_control(cmd)

    @staticmethod
    def yaw_from_odom(odom):
        q = odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def odom_direction_ok(self):
        if self.latest_odom is None or self.move_heading is None: return False
        err = math.atan2(math.sin(self.yaw_from_odom(self.latest_odom) - self.move_heading), math.cos(self.yaw_from_odom(self.latest_odom) - self.move_heading))
        return abs(math.degrees(err)) <= float(self.get_parameter('max_heading_error_deg').value)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def publish_level_target(self):
        self.spine_pub.publish(Float64MultiArray(data=[self.spine_positions[self.level]]))
        self.head_pub.publish(Float64MultiArray(data=[float(self.get_parameter('head_yaw_position_rad').value), self.head_pitch_positions[self.level]]))
        self.get_logger().info(f'level target height={self.level_heights_m[self.level]:.2f}m spine={self.spine_positions[self.level]:.2f}m pitch={self.head_pitch_positions[self.level]:.3f}rad')

    def log_control(self, cmd):
        p = self.latest_odom.pose.pose.position if self.latest_odom else None
        yaw = self.yaw_from_odom(self.latest_odom) if self.latest_odom else None
        with self.control_log.open('a', newline='', encoding='utf-8') as stream:
            csv.writer(stream).writerow([time.time(), self.stage, self.front_distance_value, self.front_min_distance_value, p.x if p else None, p.y if p else None, yaw, cmd.linear.x, cmd.angular.z])

    def joints_at_level(self):
        if self.latest_joints is None: return False
        lookup = dict(zip(self.latest_joints.name, self.latest_joints.position))
        if 'slide_joint' not in lookup or 'head_pitch_joint' not in lookup: return False
        tol = float(self.get_parameter('joint_tolerance').value)
        return abs(lookup['slide_joint'] - self.spine_positions[self.level]) <= tol and abs(lookup['head_pitch_joint'] - self.head_pitch_positions[self.level]) <= tol

    def save_image(self):
        try:
            image_ns = stamp_ns(self.latest_image.header.stamp)
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            path = self.image_dir / f'shelf_{self.shelf + 1:02d}_col_{self.column + 1:02d}_level_{self.level + 1:02d}.jpg'
            if not cv2.imwrite(str(path), cv_image): self.fail(f'图片保存失败: {path}'); return False
            pose = self.latest_odom.pose.pose if self.latest_odom else None
            self.paths.append(str(path)); self.manifest.append({'shelf': self.shelf + 1, 'column': self.column + 1, 'level': self.level + 1, 'height_m': self.level_heights_m[self.level], 'path': str(path.relative_to(self.run_dir)), 'image_stamp_ns': image_ns, 'scan_distance_m': self.front_distance_value, 'odom': {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z} if pose else None, 'spine_target_m': self.spine_positions[self.level], 'head_pitch_target_rad': self.head_pitch_positions[self.level], 'timestamp': time.time()})
            self.last_image_stamp_ns = image_ns; self.write_manifest(); return True
        except Exception as exc:
            self.fail(f'图片处理失败: {exc}'); return False

    def write_manifest(self):
        payload = {'run_id': self.run_dir.name, 'status': self.stage, 'error': self.failure_message, 'count': len(self.paths), 'expected_count': self.total_columns * self.level_count, 'images': self.manifest}
        tmp = self.run_dir / 'manifest.json.tmp'; tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8'); tmp.replace(self.run_dir / 'manifest.json')

    def finish(self):
        self.stop(); self.set_stage(Stage.FINISHED); self.write_manifest(); self.timer.cancel()

    def fail(self, message):
        self.failure_message = message; self.enabled = False; self.stop(); self.get_logger().error(message); self.set_stage(Stage.ERROR); self.write_manifest()


def main(args=None):
    rclpy.init(args=args)
    node = CaptureScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断，保存部分 manifest')
        node.failure_message = '用户 Ctrl+C 中断任务'
        node.stage = Stage.INTERRUPTED
    finally:
        node.stop(); node.write_manifest(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
