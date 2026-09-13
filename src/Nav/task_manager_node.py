#!/usr/bin/env python3
import json, math
from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped

try:
    from arm_pick import ArmPicker
except ImportError:
    ArmPicker = None

class Phase(Enum): PREPARE='prepare'; SCAN='scan'; PICK='pick'; DELIVER='deliver'; FETCH='fetch'; DONE='done'; RECOVERY='recovery'

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.phase=Phase.PREPARE; self.tasks=[]; self.mapping={}; self.detections=None; self.current=None
        self.scan_speed=float(self.declare_parameter('scan_speed', .18).value)
        self.scan_height=[float(x) for x in self.declare_parameter('scan_heights',[.30,.75,1.20]).value]
        self.a=(1.920,-3.170,0.0); self.cabinet=(1.920,2.60,math.pi/2); self.b=(-1.88,-2.80,-math.pi/2)
        self.create_subscription(String,'/supermarket_sorting/task',self.task_cb,10)
        self.create_subscription(Detection3DArray,'/kele/detections',self.detection_cb,10)
        self.nav_goal=self.create_publisher(PoseStamped,'/unified_nav/goal_pose',10)
        self.spine=self.create_publisher(Float64MultiArray,'/spine_forward_position_controller/commands',10)
        self.create_timer(.2,self.loop)
        self.picker=ArmPicker(self) if ArmPicker else None

    def task_cb(self,msg):
        data=json.loads(msg.data); self.tasks=[dict(x) for x in data.get('targets',[])]; self.phase=Phase.PREPARE
    def detection_cb(self,msg): self.detections=msg
    def goal(self,p):
        g=PoseStamped(); g.header.frame_id='map'; g.header.stamp=self.get_clock().now().to_msg(); g.pose.position.x,p_y=p[0],p[1]
        g.pose.position.y=p[1]; g.pose.orientation.z=math.sin(p[2]/2); g.pose.orientation.w=math.cos(p[2]/2); self.nav_goal.publish(g)
    def loop(self):
        if not self.tasks and self.phase not in (Phase.PREPARE,Phase.DONE): self.phase=Phase.DONE
        if self.phase==Phase.DONE: return
        if self.phase==Phase.PREPARE: self.goal(self.cabinet); self.phase=Phase.SCAN; self.level=0; self.col=0; return
        if self.phase==Phase.SCAN:
            self.spine.publish(Float64MultiArray(data=[self.scan_height[self.level]]))
            self.col += 1
            if self.detections: self.mapping.update(self.extract_mapping(self.detections))
            if self.col>=5:
                self.level += 1; self.col=0
                if self.level>=len(self.scan_height): self.phase=Phase.FETCH
            return
        if self.phase==Phase.FETCH:
            known=[t for t in self.tasks if t.get('kind') in self.mapping]
            if not known: self.phase=Phase.RECOVERY; return
            self.current=min(known,key=lambda t:self.mapping[t['kind']].get('cost',0.0)); self.goal(self.mapping[self.current['kind']]['pose']); self.phase=Phase.PICK; return
        if self.phase==Phase.PICK:
            if not self.picker: self.get_logger().error('arm_pick package unavailable'); self.phase=Phase.RECOVERY; return
            result=self.picker.pick(self.current['kind'])
            if getattr(result,'success',False): self.phase=Phase.DELIVER
            else: self.phase=Phase.RECOVERY
            return
        if self.phase==Phase.DELIVER:
            self.goal(self.b); self.tasks=[t for t in self.tasks if t is not self.current]; self.current=None
            self.phase=Phase.FETCH if self.tasks else Phase.DONE
        elif self.phase==Phase.RECOVERY: self.get_logger().warn('navigation/vision recovery required'); self.phase=Phase.SCAN

    def extract_mapping(self,msg): return {}

def main():
    rclpy.init(); n=TaskManager(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
