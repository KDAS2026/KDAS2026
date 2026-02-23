#!/usr/bin/env python3
"""
pp_viz.py
RViz visualization helper for Simulink Pure Pursuit
- Frame: base_link (Vehicle frame)
- Coordinate Correction: Rotates inputs -90deg (CW) to fix orientation
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class PPViz(Node):
    def __init__(self):
        super().__init__('pp_viz')

        # --------------------
        # Parameters
        # --------------------
        # [핵심 1] 맵(map) 대신 차량(base_link) 기준으로 변경
        self.declare_parameter('frame_id', 'base_link') 
        self.declare_parameter('circle_points', 72)
        self.declare_parameter('circle_line_width', 0.04)
        self.declare_parameter('target_scale', 0.20)
        self.declare_parameter('ld_fixed', 1.0)
        self.declare_parameter('draw_fixed_ld', True)

        self.frame_id = self.get_parameter('frame_id').value
        self.circle_points = int(self.get_parameter('circle_points').value)
        self.circle_line_width = float(self.get_parameter('circle_line_width').value)
        self.target_scale = float(self.get_parameter('target_scale').value)
        self.ld_fixed = float(self.get_parameter('ld_fixed').value)
        self.draw_fixed_ld = bool(self.get_parameter('draw_fixed_ld').value)

        # --------------------
        # State
        # --------------------
        # [핵심 2] base_link 기준이므로 차량 현재 위치는 항상 (0,0)
        self.cur_pos = (0.0, 0.0) 
        self.target = None
        self.mode = None

        # --------------------
        # Publishers
        # --------------------
        self.pub_target = self.create_publisher(Marker, '/pp_target_marker', 5)
        self.pub_circle = self.create_publisher(Marker, '/pp_lookahead_circle', 5)
        self.pub_fixed_ld = self.create_publisher(Marker, '/pp_fixed_ld_circle', 5)

        # --------------------
        # Subscribers
        # --------------------
        self.create_subscription(Point, '/location', self.cb_location, 20)
        self.create_subscription(Point, '/pp_target_selected', self.cb_target, 20)

        # Timer
        self.create_timer(0.05, self.timer_publish)  # 20 Hz

    # --------------------
    # Callbacks
    # --------------------
    def cb_location(self, msg: Point):
        # base_link 모드에서는 location(전역 좌표)을 시각화 중심점으로 쓰지 않습니다.
        # 차량 중심(0,0)을 유지합니다.
        pass

    def cb_target(self, msg: Point):
        # [핵심 3] 시계방향 90도 회전 (CW Rotation)
        # 반시계로 90도 돌아가 보인다면, 시계방향으로 90도 돌려야 정면이 맞습니다.
        # Simulink (Y=Forward) -> ROS (X=Forward) 변환
        x_rotated = msg.y
        y_rotated = -msg.x
        
        self.target = (float(x_rotated), float(y_rotated))
        try:
            self.mode = int(msg.z)
        except Exception:
            self.mode = None

    # --------------------
    # Marker builders
    # --------------------
    def _make_target_marker(self, stamp):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp

        m.ns = 'pp_target'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        # 회전 보정된 좌표 사용
        m.pose.position.x = self.target[0]
        m.pose.position.y = self.target[1]
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0

        m.scale.x = self.target_scale
        m.scale.y = self.target_scale
        m.scale.z = self.target_scale

        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.lifetime.sec = 0
        m.lifetime.nanosec = int(0.2 * 1e9)
        return m

    def _make_circle_marker(self, stamp, radius, ns, color):
        # 원의 중심은 항상 차량 중심 (0,0)
        cx, cy = (0.0, 0.0)

        c = Marker()
        c.header.frame_id = self.frame_id
        c.header.stamp = stamp
        c.ns = ns
        c.id = 0
        c.type = Marker.LINE_STRIP
        c.action = Marker.ADD
        c.scale.x = self.circle_line_width
        c.color.r, c.color.g, c.color.b, c.color.a = color

        n = max(12, self.circle_points)
        for i in range(n + 1):
            th = 2.0 * math.pi * i / n
            p = Point()
            p.x = cx + radius * math.cos(th)
            p.y = cy + radius * math.sin(th)
            p.z = 0.0
            c.points.append(p)

        c.lifetime.sec = 0
        c.lifetime.nanosec = int(0.2 * 1e9)
        return c

    # --------------------
    # Timer
    # --------------------
    def timer_publish(self):
        if self.target is None:
            return

        stamp = self.get_clock().now().to_msg()

        # 1. Target Marker
        self.pub_target.publish(self._make_target_marker(stamp))

        # 2. Lookahead Circle (Blue)
        # 반지름 = 차량 중심(0,0)에서 타겟까지의 거리
        dist_to_target = math.hypot(self.target[0], self.target[1])
        
        self.pub_circle.publish(
            self._make_circle_marker(
                stamp,
                dist_to_target,
                'pp_target_dist',
                (0.0, 0.0, 1.0, 1.0)
            )
        )

        # 3. Fixed LD Circle (Green)
        if self.draw_fixed_ld:
            self.pub_fixed_ld.publish(
                self._make_circle_marker(
                    stamp,
                    self.ld_fixed,
                    'pp_fixed_ld',
                    (0.0, 1.0, 0.0, 0.8)
                )
            )

def main():
    rclpy.init()
    node = PPViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
