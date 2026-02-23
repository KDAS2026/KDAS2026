#!/usr/bin/env python3
import rclpy, json, os
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool
from geometry_msgs.msg import Point
import numpy as np

class WaypointGeneratorNode(Node):
    def __init__(self):
        super().__init__('waypoint_generator')

        # --- 차량 및 장애물 파라미터
        self.car_half   = 0.2    # 차량 반폭
        self.car_length = 0.4    # 차량 길이 (예비)
        self.obs_length = 0.5    # 장애물 길이

        # --- 경로 맵 정의
        self.paths = {
            1: np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.2], [3.0, 0.0], [4.0, 0.0]]),
            2: np.array([[4.0, 0.0], [5.0, 0.0], [6.0, 0.1], [7.0, 0.0]])
        }

        # --- 경로 시퀀스(JSON) 로드
        self.path_id_sequence = []
        self.current_path_idx = None
        seq_file = self.declare_parameter(
            'sequence_file', os.path.join(os.getcwd(), 'dqn_paths.json')
        ).value
        if os.path.exists(seq_file):
            with open(seq_file, 'r') as f:
                data = json.load(f)
            self.path_id_sequence = data.get('total', [])
            self.get_logger().info(f"Loaded sequence: {self.path_id_sequence}")
        else:
            self.get_logger().error(f"Sequence file not found: {seq_file}")

        # --- 오프셋 파라미터
        self.delta           = 0.05
        self.max_k           = 20
        self.reach_threshold = 0.2

        # --- 상태 변수
        self.selected_id  = None
        self.obs_x        = None
        self.obs_y        = None
        self.obs_radius   = None
        self.current_pose = None  # (x, y, heading)
        self.avoiding     = False
        self.offset_path  = None

        # --- 타이머 발행 버퍼
        self.current_buffer = []
        self.current_index  = 0
        self.active         = False

        # --- 구독자: 장애물 정보, 위치
        self.create_subscription(
            Float32MultiArray, '/obstacle_info', self.obstacle_info_cb, 10
        )
        self.create_subscription(
            Point, '/location', self.location_cb, 10
        )

        # --- 퍼블리셔: 회피 경로 및 이벤트
        self.pub_x           = self.create_publisher(Float32, '/path_x',    10)
        self.pub_y           = self.create_publisher(Float32, '/path_y',    10)
        self.pub_avoid_start = self.create_publisher(Bool,    '/avoid_start',10)
        self.pub_avoid_done  = self.create_publisher(Bool,    '/avoid_done', 10)

        # --- 주기적 발행 타이머
        self.create_timer(0.2, self.timer_publish)

    def update_selected_id(self):
        if self.current_pose is None or not self.path_id_sequence:
            return
        x, y, _ = self.current_pose
        thr = 0.5
        for idx, pid in enumerate(self.path_id_sequence):
            pts = self.paths.get(pid)
            if pts is None:
                continue
            if np.min(np.linalg.norm(pts - [x, y], axis=1)) < thr:
                self.selected_id      = pid
                self.current_path_idx = idx
                return

    def obstacle_info_cb(self, msg: Float32MultiArray):
        self.obs_x      = float(msg.data[0])
        self.obs_y      = float(msg.data[1])
        self.obs_radius = float(msg.data[2])
        self.avoiding    = False
        self.offset_path = None
        self.try_generate()

    def location_cb(self, msg: Point):
        self.current_pose = (msg.x, msg.y, msg.z)
        self.update_selected_id()

        # 회피 완료 체크
        if self.avoiding and self.offset_path is not None:
            last_x, last_y, _ = self.offset_path[-1]
            if np.hypot(msg.x - last_x, msg.y - last_y) < self.reach_threshold:
                self.get_logger().info("회피 종료: 마지막 웨이포인트 도달")
                self.avoiding = False
                self.pub_avoid_done.publish(Bool(data=True))

        self.try_generate()

    def try_generate(self):
        if self.avoiding:
            return
        if None in (
            self.obs_x, self.obs_y, self.obs_radius,
            self.current_pose, self.selected_id,
            self.current_path_idx
        ):
            return
        if self.selected_id not in self.paths:
            self.get_logger().warn(f"[!] 알 수 없는 경로 ID: {self.selected_id}")
            return

        cur_pts = self.paths[self.selected_id]
        if self.current_path_idx + 1 < len(self.path_id_sequence):
            next_id  = self.path_id_sequence[self.current_path_idx + 1]
            next_pts = self.paths[next_id]
            full_pts = np.vstack([cur_pts, next_pts[1:]])
            end_pt   = next_pts[-1]
        else:
            full_pts = cur_pts
            end_pt   = cur_pts[-1]

        start_pt = np.array(self.current_pose[:2])
        c_obs    = np.array([self.obs_x, self.obs_y])
        r_obs    = self.obs_radius

        offset_pts = self.compute_offset_points(
            full_pts, start_pt, end_pt, c_obs, r_obs
        )
        self.update_path_buffer(offset_pts)
        self.avoiding = True
        self.pub_avoid_start.publish(Bool(data=True))

    def compute_offset_points(self, pts, start_pt, end_pt, c_obs, r_obs):
        N = len(pts)
        offset = np.zeros((N, 2))
        offset[0]  = start_pt
        offset[-1] = end_pt

        idx_obs = np.argmin(np.linalg.norm(pts - c_obs, axis=1))
        if 0 < idx_obs < N-1:
            tangent = pts[idx_obs+1] - pts[idx_obs-1]
            dir_vec = tangent / np.linalg.norm(tangent)
        else:
            dir_vec = np.array([1.0, 0.0])
        half_len = self.obs_length / 2.0
        a = c_obs - half_len * dir_vec
        b = c_obs + half_len * dir_vec
        thresh = r_obs + self.car_half

        def point_to_segment_dist(p, a_pt, b_pt):
            ap = p - a_pt
            ab = b_pt - a_pt
            t  = np.dot(ap, ab) / np.dot(ab, ab)
            t  = np.clip(t, 0.0, 1.0)
            proj = a_pt + t * ab
            return np.linalg.norm(p - proj)

        for i in range(1, N-1):
            p_curr = pts[i]
            if point_to_segment_dist(p_curr, a, b) < thresh:
                p_prev, p_next = pts[i-1], pts[i+1]
                # 법선 벡터 계산
                normal = np.array([-(p_next - p_prev)[1], (p_next - p_prev)[0]])
                nrm    = np.linalg.norm(normal)
                if nrm > 1e-6:
                    normal /= nrm
                    d = 0.0
                    # 항상 왼쪽(+) 방향으로만 offset 적용
                    for k in range(1, self.max_k+1):
                        cand = p_curr + k * self.delta * normal  # 왼쪽 방향만
                        if point_to_segment_dist(cand, a, b) > thresh:
                            d = k * self.delta
                            break
                    offset[i] = p_curr + d * normal
                else:
                    offset[i] = p_curr
            else:
                offset[i] = p_curr

        result = []
        for j in range(N):
            x, y = offset[j]
            if j < N-1:
                dx, dy = offset[j+1] - offset[j]
                yaw     = np.arctan2(dy, dx)
            else:
                yaw = result[-1][2] if result else 0.0
            result.append((x, y, yaw))
        return result

    def update_path_buffer(self, path_tuples):
        self.offset_path    = path_tuples
        self.current_buffer = [(x, y) for x, y, _ in path_tuples]
        self.current_index  = 0
        self.active         = True
        self.get_logger().info(f"Offset 경로 저장 완료: {len(path_tuples)} points")

    def timer_publish(self):
        if not self.active or self.current_index >= len(self.current_buffer):
            return
        for i in range(4):
            idx = self.current_index + i
            if idx < len(self.current_buffer):
                x, y = self.current_buffer[idx]
                self.pub_x.publish(Float32(data=x))
                self.pub_y.publish(Float32(data=y))
            else:
                break
        self.current_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = WaypointGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
