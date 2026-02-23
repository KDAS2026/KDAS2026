#!/usr/bin/env python3
import os
import json
import math
import rclpy
from collections import deque
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Bool, Int32, Char, Float64MultiArray, MultiArrayDimension
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

shared = get_package_share_directory('path_planning')


class HelperPathSender(Node):
    """
    Integrated helper.py (A+B merge):
      - Keeps high-rate publish + distance-based waypoint consumption (B)
      - Keeps robust stops/pickup/dropoff parsing + reset on resume (A)
      - Adds multi-node forward window prefetch (queue of node waypoints)
      - Adds stop->resume re-anchor (nearest within a bounded search window)

    Topics (unchanged):
      pub:  path_x(Float32), path_y(Float32), path_mode(Int32),
            pickup_dropoff(Int32), stop(Int32), dqn_done(Bool)
      sub:  location(Point), /yolo_stop(Int32), /yolo_traffic_light(Int32), ride(Char)

    Added (for RViz):
      pub:  /helper_path (nav_msgs/Path)

    Added (for Simulink Pure Pursuit input stability):
      pub:  /pp_path (nav_msgs/Path)                    # same content as /helper_path
      pub:  /pp_waypoints (std_msgs/Float64MultiArray)  # 64x2 flattened: [x1,y1,x2,y2,...] (length 128)
      pub:  /pp_waypoints_len (Int32)                   # valid waypoint count (0~64)
    """

    def __init__(self):
        super().__init__('helper_path_sender')

        # Publishers
        self.pub_x = self.create_publisher(Float32, 'path_x', 100)
        self.pub_y = self.create_publisher(Float32, 'path_y', 100)
        self.pub_path_mode = self.create_publisher(Int32, 'path_mode', 100)
        self.pub_pick_drop = self.create_publisher(Int32, 'pickup_dropoff', 10)
        self.pub_stop = self.create_publisher(Int32, 'stop', 10)
        self.pub_dqn_done = self.create_publisher(Bool, 'dqn_done', 10)

        # RViz visualization (path window)
        self.pub_path = self.create_publisher(NavPath, '/helper_path', 5)

        # Simulink input (avoid nav_msgs/Path variable-length array limits inside Simulink)
        # NOTE: We still publish /pp_path for debugging/compatibility, but the recommended
        #       control input is /pp_waypoints (fixed 64x2 flattened).
        self.pub_pp_path = self.create_publisher(NavPath, '/pp_path', 5)
        self.pub_pp_waypoints = self.create_publisher(Float64MultiArray, '/pp_waypoints', 10)
        self.pub_pp_waypoints_len = self.create_publisher(Int32, '/pp_waypoints_len', 10)

        # Subscribers
        self.create_subscription(Point, 'location', self.cb_location, 20)
        self.create_subscription(Int32, '/yolo_stop', self.cb_stop, 20)
        self.create_subscription(Int32, '/yolo_traffic_light', self.cb_TLight, 10)
        self.create_subscription(Char, 'ride', self.cb_ride, 10)

        # Parameters / paths
        self.wp_dir = os.path.join(shared, 'waypoints')

        # Node -> path_mode mapping (kept)
        self.path_mode_map = {
            0: 1, 1: 0, 2: 0, 3: 1, 4: 0, 5: 0, 6: 1, 7: 1, 8: 1, 9: 1,
            10: 0, 11: 0, 12: 1, 13: 0, 14: 0, 15: 1, 16: 0, 17: 1, 18: 0, 19: 1,
            20: 1, 21: 0, 22: 1, 23: 1, 24: 0, 25: 1, 26: 1, 27: 0, 28: 1, 29: 0,
            30: 1, 31: 1, 32: 0, 33: 1, 34: 1, 35: 1, 36: 1, 37: 1, 38: 0, 39: 1,
            40: 0, 41: 1, 42: 1, 43: 1, 44: 0, 45: 1, 46: 0, 47: 0, 48: 1, 49: 0,
            50: 1, 51: 0, 52: 1, 53: 1, 54: 1, 55: 1, 56: 0, 57: 2, 58: 1, 59: 1,
            60: 1, 61: 1
        }

        # Runtime state
        self.started = False
        self.ride = None
        self.pos = (0.0, 0.0)

        # Stop handling
        self.stop_flag = 0
        self.stop_counter = 0
        self.stops_index = 0
        self.stops = []

        # Pickup/Dropoff mapping (A-style)
        self.pickup_set = set()
        self.dropoff_set = set()

        # Node queue & segment index
        self.segment_idx = 0
        self.node_queue = deque()

        # Multi-node buffers (forward window)
        # buffers[i] corresponds to node_queue[segment_idx + i]1.1
        self.buffers = []  # list[list[[x,y],...]]
        self.buffer_node_ids = []  # list[node_id]
        self.cur_buf = 0   # buffer index within buffers
        self.cur_idx = 0   # waypoint index within buffers[cur_buf]

        # Timers
        self.loop_time = 0.01         # ~100 Hz
        self.path_loop_time = 0.75    # stop tick loop
        self.path_pub_time = 0.1      # 10 Hz path publish for RViz

        # Stop timing
        self.TICKS_TO_WAIT = int(math.ceil(3.0 / self.path_loop_time))
        self.PICKDROP_TICKS = int(math.ceil(5.0 / self.path_loop_time))  # pickup/dropoff hold (legacy)
        self.YOLO_COOLDOWN_TICKS = int(math.ceil(7.0 / self.path_loop_time))
        self.yolo_cooldown_counter = 0

        # Distance-based consumption tuning
        self.REACH_TOL = 0.08               # meters (tune vs waypoint spacing)
        self.MAX_ADVANCE_PER_TICK = 5       # cap to prevent jumpy consumption at 100 Hz

        # Forward window config (multi-node prefetch)
        self.WINDOW_LEN_M = 1.5             # maintain at least this much path ahead (arc-length approx)
        self.MAX_PREFETCH_NODES = 6         # hard cap on nodes loaded ahead

        # Re-anchor config (stop->resume)
        self.REANCHOR_W_BACK = 30           # search back points in current buffer
        self.REANCHOR_W_FWD = 200           # search forward points across buffers
        self.REANCHOR_SHIFT_TOL = 0.35      # if close to segment end, allow shifting segment

        # Goal-based finish (kept)
        self.GOAL_X = -1.3
        self.GOAL_Y = -0.56
        self.GOAL_ENTER_R = 0.1
        self.GOAL_EXIT_R = 0.0
        self.goal_armed = False

        # Start state: RUNNING (do not force /stop=1 at init)
        # self.pub_stop.publish(Int32(data=1))  # disabled

    # ---------------------------
    # Utility / lifecycle helpers
    # ---------------------------

    def finish_ride(self, reason: str = ""):
        if reason:
            self.get_logger().info(f"ride done: {reason}")
        else:
            self.get_logger().info("ride done")
        self.pub_dqn_done.publish(Bool(data=False))
        self.pub_stop.publish(Int32(data=1))
        self.started = False

    def _is_last_segment(self) -> bool:
        return (len(self.node_queue) > 0) and (self.segment_idx >= (len(self.node_queue) - 1))

    def check_goal_finish(self):
        if not self.started:
            return
        if not self._is_last_segment():
            return

        x, y = self.pos
        d = math.hypot(x - self.GOAL_X, y - self.GOAL_Y)

        if not self.goal_armed:
            if d > self.GOAL_EXIT_R:
                self.goal_armed = True
                self.get_logger().info(f"[goal] armed (d={d:.3f})")
            return

        if d <= self.GOAL_ENTER_R:
            self.finish_ride(reason=f"goal reached at ({self.GOAL_X}, {self.GOAL_Y}), d={d:.3f}")

    def _resolve_paths_json(self, filename: str) -> str:
        p = Path(filename)
        if p.is_absolute():
            return str(p)

        tried = []
        share = Path(get_package_share_directory('path_planning'))

        cand1 = share / 'paths' / filename
        tried.append(str(cand1))
        if cand1.is_file():
            return str(cand1)

        cand2 = Path.cwd() / 'src' / 'path_planning' / 'paths' / filename
        tried.append(str(cand2))
        if cand2.is_file():
            return str(cand2)

        raise FileNotFoundError(
            f"Cannot find '{filename}'. Tried:\n  - " + "\n  - ".join(tried)
        )

    # ---------------------------
    # ROS callbacks
    # ---------------------------

    def cb_location(self, msg: Point):
        self.pos = (msg.x, msg.y)

    def cb_stop(self, msg: Int32):
        """
        Stop-sign handling (YOLO):
          - msg.data == 1: trigger a fixed-time stop (e.g., 3 seconds) handled in path_loop()

        Design:
          - /yolo_stop is treated as an EVENT (edge/pulse).
          - helper publishes /stop=1 immediately, holds for TICKS_TO_WAIT, then releases /stop=0 and re-anchors.
        """
        if msg.data != 1:
            return

        # Do not override traffic-light stop. If we're already stopped for any reason, ignore.
        if self.stop_flag != 0:
            return

        if self.yolo_cooldown_counter == 0:
            self.stop_flag = 1  # stop-sign stop
            self.stop_counter = 0  # arm counter in path_loop()
            self.pub_stop.publish(Int32(data=1))
            self.yolo_cooldown_counter = self.YOLO_COOLDOWN_TICKS
            self.get_logger().info("YOLO Stop Sign -> STOP (fixed duration)")
        else:
            self.get_logger().info("YOLO Stop Sign ignored (cooldown)")

    def cb_TLight(self, msg: Int32):
        """
        Traffic light handling (YOLO):
          - msg.data == 1: Red  -> stop immediately and hold until green
          - msg.data == 0: Green -> release stop and re-anchor once

        NOTE:
          This node (helper) is the ONLY publisher of /stop in this architecture.
        """
        if msg.data == 1:
            # Red light: enter traffic-light stop state (hold until green)
            if self.stop_flag != 2:
                self.stop_flag = 2
                self.stop_counter = 0  # not used for TL stop
                self.pub_stop.publish(Int32(data=1))
                self.get_logger().info("YOLO Red Light -> STOP (traffic light)")
        elif msg.data == 0:
            # Green light: release if we were holding for TL
            if self.stop_flag == 2:
                self.stop_flag = 0
                self.stop_counter = 0
                self.pub_stop.publish(Int32(data=0))
                self.pub_pick_drop.publish(Int32(data=0))
                self.get_logger().info("YOLO Green Light -> RESUME + re_anchor")
                self.re_anchor()
            else:
                # If not in TL-stop state, ignore (avoid toggling /stop unexpectedly)
                pass
    def cb_ride(self, msg: Char):
        if not self.started:
            self.ride = msg.data
            self.get_logger().info(f"Received /ride command: {self.ride}")
            self.pub_dqn_done.publish(Bool(data=True))
            self.started = True
            self.start()
        else:
            self.get_logger().info("Ride command received again, but node already started.")

    # ---------------------------
    # Path / stop config loading
    # ---------------------------

    def load_waypoints(self, node_id) -> list:
        if node_id is None:
            return []
        coords = []

        filename = f'waypoints_{node_id}.json'

        path = os.path.join(self.wp_dir, f'waypoints_{node_id}.json')

        self.get_logger().info(f"------------ Reading JSON file: {filename} ------------")
        try:
            with open(path, 'r') as f:
                pts = json.load(f)
            for p in pts:
                coords.append([float(p[0]), float(p[1])])
            self.get_logger().info(f"[helper] loaded waypoints for node {node_id}: {len(coords)} points")
        except Exception as e:
            self.get_logger().error(f"load_waypoints({node_id}) failed: {e}")
            return []
        return coords

    def _parse_stops_config_Astyle(self, stops_cfg):
        """
        A-style robust parsing:
          - legacy list: [10, 16, ...] or list-of-lists
          - dict:
              {
                "pickup_nodes": [...],
                "dropoff_nodes": [...],
                "stops": [...]   # optional ordered list
              }
        Behavior:
          - If dict has explicit stops list, use it as stop order.
          - Else, stops = union(pickup, dropoff).
          - If legacy list and non-empty: first stop => pickup, last stop => dropoff.
        """
        self.pickup_set = set()
        self.dropoff_set = set()
        stops_list = []

        if isinstance(stops_cfg, dict):
            self.pickup_set = set(map(int, stops_cfg.get('pickup_nodes', [])))
            self.dropoff_set = set(map(int, stops_cfg.get('dropoff_nodes', [])))

            if 'stops' in stops_cfg and isinstance(stops_cfg['stops'], list):
                stops_list = [int(x) for x in stops_cfg['stops']]
            else:
                stops_list = list(self.pickup_set | self.dropoff_set)

        else:
            # Legacy list (or list-of-lists)
            stops = stops_cfg if stops_cfg else []
            if any(isinstance(s, list) for s in stops):
                flat = []
                for s in stops:
                    flat.extend(s if isinstance(s, list) else [s])
                stops = flat
            stops_list = [int(x) for x in stops] if stops else []

            if stops_list:
                self.pickup_set = {int(stops_list[0])}
                self.dropoff_set = {int(stops_list[-1])}

        self.stops = stops_list

        self.get_logger().info(f"Loaded stops: {self.stops}")
        self.get_logger().info(f"Loaded pickup_nodes: {sorted(list(self.pickup_set))}")
        self.get_logger().info(f"Loaded dropoff_nodes: {sorted(list(self.dropoff_set))}")

    # ---------------------------
    # Multi-node forward window
    # ---------------------------

    def _reset_buffers(self):
        self.buffers = []
        self.buffer_node_ids = []
        self.cur_buf = 0
        self.cur_idx = 0

    def _append_node_buffer(self, node_id):
        pts = self.load_waypoints(node_id)
        self.buffers.append(pts)
        self.buffer_node_ids.append(node_id)

    def _ensure_forward_window(self):
        """
        Ensure we have enough path ahead (approx arc-length) by prefetching more nodes.
        This uses current cursor (cur_buf, cur_idx) as the start point.
        """
        if not self.buffers:
            return

        def buffers_can_grow():
            return (len(self.buffers) < self.MAX_PREFETCH_NODES) and ((self.segment_idx + len(self.buffers)) < len(self.node_queue))

        def ahead_length_m() -> float:
            length = 0.0
            last_pt = None
            for b in range(self.cur_buf, len(self.buffers)):
                pts = self.buffers[b]
                if not pts:
                    continue
                start_i = self.cur_idx if (b == self.cur_buf) else 0
                if start_i >= len(pts):
                    continue
                for i in range(start_i, len(pts)):
                    pt = pts[i]
                    if last_pt is not None:
                        length += math.hypot(pt[0] - last_pt[0], pt[1] - last_pt[1])
                    last_pt = pt
                    if length >= self.WINDOW_LEN_M:
                        return length
            return length

        while ahead_length_m() < self.WINDOW_LEN_M and buffers_can_grow():
            next_node = self.node_queue[self.segment_idx + len(self.buffers)]
            self.get_logger().info(f"[window] prefetch node {next_node}")
            self._append_node_buffer(next_node)

    def _shift_segment(self, k: int = 1):
        for _ in range(k):
            self.segment_idx += 1
            
            # 현재 처리 중인 버퍼(cur_buf)가 0번(삭제될 버퍼)인지 확인
            is_popping_current = (self.cur_buf == 0)

            if self.buffers:
                self.buffers.pop(0)
                self.buffer_node_ids.pop(0)
            
            if self.cur_buf > 0:
                self.cur_buf -= 1
            elif is_popping_current:
                # [수정] 현재 주행 중이던 버퍼가 사라졌으므로,
                # 다음 버퍼(새로운 0번)의 처음부터 시작하도록 리셋합니다.
                self.cur_buf = 0
                self.cur_idx = 0

        if not self.buffers:
            self.cur_buf = 0
            self.cur_idx = 0
            return
        
        # 인덱스 범위 안전장치 (기존 로직 유지하되 필요시 리셋 후 재조정)
        self.cur_buf = max(0, min(self.cur_buf, len(self.buffers) - 1))
        if self.buffers[self.cur_buf]:
            self.cur_idx = max(0, min(self.cur_idx, len(self.buffers[self.cur_buf]) - 1))
        else:
            self.cur_idx = 0

        self._ensure_forward_window()
        # 현재 로드된 버퍼가 있고, 로봇 위치 정보가 있다면 연결성 체크
        if self.buffers and self.pos != (0.0, 0.0):
            try:
                # 현재 로봇 위치
                rx, ry = self.pos
                
                # 새로 바뀐 경로의 첫 번째 점
                start_pt = self.buffers[self.cur_buf][0] 
                sx, sy = float(start_pt[0]), float(start_pt[1])
                
                # 거리 계산
                dist = math.hypot(sx - rx, sy - ry)
                
                # 현재 로드한 노드 번호 확인 (node_queue와 segment_idx 이용)
                curr_node_id = self.node_queue[self.segment_idx]
                
                self.get_logger().warn(f"⚠️ Segment Changed to Node {curr_node_id}")
                self.get_logger().warn(f"   - Robot Pos: ({rx:.2f}, {ry:.2f})")
                self.get_logger().warn(f"   - New Start: ({sx:.2f}, {sy:.2f})")
                self.get_logger().warn(f"   - Jump Dist: {dist:.2f} m")

                if dist > 2.0: # 2미터 이상 차이나면 에러급 경고
                    self.get_logger().error(f"🚨🚨 PATH DISCONTINUITY DETECTED! (Jump > 2m) 🚨🚨")
            except Exception as e:
                self.get_logger().error(f"Debug print failed: {e}")

    # ---------------------------
    # Target selection & consumption
    # ---------------------------

    def _get_current_target(self):
        if not self.buffers:
            return None

        b = self.cur_buf
        i = self.cur_idx

        while b < len(self.buffers):
            pts = self.buffers[b]
            if not pts:
                b += 1
                i = 0
                continue
            if i < len(pts):
                self.cur_buf = b
                self.cur_idx = i
                return (b, i, pts[i])
            b += 1
            i = 0

        return None

    def _advance_cursor_one(self):
        tgt = self._get_current_target()
        if tgt is None:
            return False
        b, i, _pt = tgt
        pts = self.buffers[b]
        if i + 1 < len(pts):
            self.cur_buf = b
            self.cur_idx = i + 1
            return True
        else:
            self.cur_buf = b + 1
            self.cur_idx = 0
            return True

    def _advance_if_reached(self, tol: float):
        if self.stop_flag != 0:
            return

        advanced = 0
        while advanced < self.MAX_ADVANCE_PER_TICK:
            tgt = self._get_current_target()
            if tgt is None:
                return
            _b, _i, pt = tgt
            x, y = self.pos
            d = math.hypot(x - pt[0], y - pt[1])
            if d > tol:
                return
            if not self._advance_cursor_one():
                return
            advanced += 1

    def publish_path_mode(self):
        if self.segment_idx >= len(self.node_queue):
            return
        key = int(self.node_queue[self.segment_idx])
        if key in self.path_mode_map:
            self.pub_path_mode.publish(Int32(data=int(self.path_mode_map[key])))

    # ---------------------------
    # Segment transition & stopping
    # ---------------------------

    def end_trigger(self, tol: float = 0.28) -> bool:
        if not self.buffers:
            return False
        if self.segment_idx + 1 >= len(self.node_queue):
            return False
        seg_pts = self.buffers[0]
        if not seg_pts:
            return False
        x_target, y_target = seg_pts[-1]
        x, y = self.pos
        return math.hypot(x - x_target, y - y_target) <= tol

    def _publish_pickdrop_for_current_stop(self, current_node: int):
        role = 0
        if self.pickup_set or self.dropoff_set:
            if current_node in self.pickup_set:
                role = 1
            elif current_node in self.dropoff_set:
                role = 2
        else:
            if self.stops_index == 0:
                role = 1
            elif self.stops_index == len(self.stops) - 1:
                role = 2

        if role != 0:
            self.pub_pick_drop.publish(Int32(data=int(role)))

    # ---------------------------
    # Re-anchor (stop->resume)
    # ---------------------------

    def re_anchor(self):
        if not self.buffers:
            return

        x, y = self.pos

        candidates = []
        tgt = self._get_current_target()
        if tgt is None:
            return
        b0, i0, _ = tgt

        b = b0
        pts = self.buffers[b] if b < len(self.buffers) else []
        if pts:
            lo = max(0, i0 - self.REANCHOR_W_BACK)
            hi = min(len(pts), i0 + self.REANCHOR_W_FWD)
            for i in range(lo, hi):
                candidates.append((b, i))

        remaining = self.REANCHOR_W_FWD
        for bb in range(b0 + 1, min(len(self.buffers), b0 + 1 + self.MAX_PREFETCH_NODES)):
            pts2 = self.buffers[bb]
            if not pts2:
                continue
            take = min(len(pts2), remaining)
            for i in range(0, take):
                candidates.append((bb, i))
            remaining -= take
            if remaining <= 0:
                break

        best = None
        best_d = 1e9
        for (bb, ii) in candidates:
            pt = self.buffers[bb][ii]
            d = math.hypot(x - pt[0], y - pt[1])
            if d < best_d:
                best_d = d
                best = (bb, ii)

        if best is None:
            return

        best_b, best_i = best
        self.get_logger().info(f"[re_anchor] cursor -> buf={best_b}, idx={best_i}, d={best_d:.3f}")
        self.cur_buf = best_b
        self.cur_idx = best_i

        if best_b > 0 and self.end_trigger(tol=self.REANCHOR_SHIFT_TOL):
            self.get_logger().info(f"[re_anchor] shifting segment by {best_b} (near segment end)")
            self._shift_segment(best_b)
    #----------------------------
    # ---------------------------
    # target point problem
    # ---------------------------
    # [추가 1] 안전하게 타겟 포인트 가져오기
    def _get_target_at(self, b_idx, p_idx):
        if b_idx >= len(self.buffers): return None
        pts = self.buffers[b_idx]
        if not pts: return None
        if p_idx >= len(pts): return None
        return (b_idx, p_idx, pts[p_idx])

    # [추가 2] 다음 인덱스 위치 미리보기
    def _peek_next_pos(self, b, i):
        if b >= len(self.buffers): return None
        pts = self.buffers[b]
        if i + 1 < len(pts):
            return (b, i + 1)
        else:
            if b + 1 < len(self.buffers):
                return (b + 1, 0)
            else:
                return None

    # [추가 3] LD 없이 '따라잡기(Catch-up)'만 수행하는 함수
    def _update_target_catchup(self):
        if self.stop_flag != 0 or not self.buffers:
            return

        x, y = self.pos
        
        # 반복문을 돌며 다음 점이 더 가까운지 확인
        for _ in range(self.MAX_ADVANCE_PER_TICK):
            curr_tgt = self._get_target_at(self.cur_buf, self.cur_idx)
            if not curr_tgt: break
            
            next_idx = self._peek_next_pos(self.cur_buf, self.cur_idx)
            if not next_idx: break 
            
            next_tgt = self._get_target_at(next_idx[0], next_idx[1])
            if not next_tgt: break

            d_curr = math.hypot(x - curr_tgt[2][0], y - curr_tgt[2][1])
            d_next = math.hypot(x - next_tgt[2][0], y - next_tgt[2][1])

            # [핵심] 다음 점이 현재 점보다 내 차에 더 가깝다면? -> 이미 지났으므로 스킵!
            if d_next < d_curr:
                self.cur_buf, self.cur_idx = next_idx
            else:
                # 다음 점이 더 멀다면? -> 현재 점이 맞음.
                # 하지만 현재 점에 충분히 가까이 갔다면(REACH_TOL) 통과시킴
                if d_curr < self.REACH_TOL:
                    self.cur_buf, self.cur_idx = next_idx
                else:
                    break


    #----------------------------
    # ---------------------------
    # RViz publishing (Path)
    # ---------------------------

    def publish_path(self):
        """
        Publish a nav_msgs/Path for RViz visualization.

        - Topic: /helper_path
        - Content: a forward window of waypoints starting from the current cursor
        - Rate: controlled by self.path_pub_time timer
        """
        if not self.started:
            return
        if self.stop_flag != 0:
            return
        if not self.buffers:
            return

        msg = NavPath()
        msg.header.stamp = self.get_clock().now().to_msg()

        # IMPORTANT: Set this to the SAME fixed frame you use in RViz (map/odom).
        # If your /location is already in 'map', keep 'map'. If it's 'odom', change it.
        msg.header.frame_id = 'map'

        max_points = 400

        count = 0
        b = self.cur_buf
        i = self.cur_idx

        while b < len(self.buffers) and count < max_points:
            pts = self.buffers[b]
            if not pts:
                b += 1
                i = 0
                continue

            start_i = i if b == self.cur_buf else 0
            for k in range(start_i, len(pts)):
                if count >= max_points:
                    break
                p = PoseStamped()
                p.header = msg.header
                p.pose.position.x = float(pts[k][0])
                p.pose.position.y = float(pts[k][1])
                p.pose.position.z = 0.0
                p.pose.orientation.w = 1.0
                msg.poses.append(p)
                count += 1

            b += 1
            i = 0

        # Publish to both topics (RViz + Simulink subscriber expecting nav_msgs/Path)
        self.pub_path.publish(msg)
        # Mirror for Simulink models that still subscribe to nav_msgs/Path.
        self.pub_pp_path.publish(msg)

    # ---------------------------
    # Simulink-friendly publishing (fixed-size numeric array)
    # ---------------------------

    def publish_waypoints(self):
        """Publish a fixed-size waypoint window for Simulink.

        Why this exists:
          - Simulink ROS 2 Subscribe often clamps variable-length arrays (like nav_msgs/Path.poses)
            to a small maximum (commonly 16), which breaks Pure Pursuit inputs.
          - std_msgs/Float64MultiArray.data is a plain numeric vector and is reliably handled.

        This helper intentionally publishes **a capped number of points** because Simulink's
        ROS 2 Subscribe block can clamp variable-length arrays. In your current Simulink model,
        the observed maximum for Float64MultiArray.data is 128 elements.

        Therefore we publish:
          - N = 64 waypoints (x,y pairs)
          - Flattened vector length = 128 ( [x1,y1,x2,y2,...] )

        Topics:
          - /pp_waypoints: Float64MultiArray, flattened as [x1,y1,x2,y2,...] length 128
          - /pp_waypoints_len: Int32, valid waypoint count (0~64)
        """
        if not self.started:
            return
        if self.stop_flag != 0:
            return
        if not self.buffers:
            return

        # NOTE:
        #   Simulink (R2024b) in your setup inferred Float64MultiArray.data as length 128.
        #   That corresponds to 64 (x,y) waypoint pairs. Keep this aligned to avoid
        #   "Port width mismatch" during model compile.
        MAX_N = 64

        # Collect up to MAX_N points starting from the current cursor across buffers.
        pts_out = []  # list[(x,y)]
        b = self.cur_buf
        i = self.cur_idx

        while b < len(self.buffers) and len(pts_out) < MAX_N:
            pts = self.buffers[b]
            if not pts:
                b += 1
                i = 0
                continue

            start_i = i if b == self.cur_buf else 0
            for k in range(start_i, len(pts)):
                if len(pts_out) >= MAX_N:
                    break
                pts_out.append((float(pts[k][0]), float(pts[k][1])))

            b += 1
            i = 0

        n_valid = len(pts_out)

        # Flatten into a fixed-length (MAX_N*2) vector with zero-padding.
        data = [0.0] * (MAX_N * 2)
        for idx, (x, y) in enumerate(pts_out):
            j = 2 * idx
            data[j] = x
            data[j + 1] = y

        msg = Float64MultiArray()
        msg.data = data

        # Optional: attach layout metadata (not required for Simulink, but helps humans).
        dim0 = MultiArrayDimension()
        dim0.label = 'waypoints'
        dim0.size = MAX_N
        dim0.stride = MAX_N * 2

        dim1 = MultiArrayDimension()
        dim1.label = 'xy'
        dim1.size = 2
        dim1.stride = 2

        msg.layout.dim = [dim0, dim1]
        msg.layout.data_offset = 0

        self.pub_pp_waypoints.publish(msg)
        self.pub_pp_waypoints_len.publish(Int32(data=int(n_valid)))

    # ---------------------------
    # Main loops
    # ---------------------------

    def loop(self):
        # Main high-rate streaming loop for path_x/path_y.
        self.check_goal_finish()
        if not self.started:
            return

        # During ANY stop state, pause path streaming (requirement: stop 중 publish 중단).
        if self.stop_flag != 0:
            return

        # Segment transition (consume node queue).
        # Keep legacy pickup/dropoff stops (from stops_file), but ignore other coordinate-based stops.
        if self.end_trigger():
            stop_loc = self.stops[self.stops_index] if self.stops_index < len(self.stops) else None
            cur_node = int(self.node_queue[self.segment_idx]) if self.segment_idx < len(self.node_queue) else None
            self.get_logger().info(f"Trigger: node {cur_node}, stop location {stop_loc}")

            if stop_loc is not None and cur_node is not None and int(stop_loc) == int(cur_node):
                # Determine whether this stop is PICKUP(1) / DROPOFF(2).
                role = 0
                if self.pickup_set or self.dropoff_set:
                    if cur_node in self.pickup_set:
                        role = 1
                    elif cur_node in self.dropoff_set:
                        role = 2
                else:
                    # Legacy behavior: first stop = pickup, last stop = dropoff
                    if self.stops_index == 0:
                        role = 1
                    elif self.stops_index == len(self.stops) - 1:
                        role = 2

                if role != 0:
                    self.get_logger().info("Stop at PICK/DROP stop point")
                    self.pub_pick_drop.publish(Int32(data=int(role)))
                    self.pub_stop.publish(Int32(data=1))
                    self.stop_flag = 3
                    self.stop_counter = 0
                else:
                    self.get_logger().info("Stop point matched but role==0 -> ignored (coord stops disabled)")

                # Advance stop cursor regardless (prevents repeated matching on the same node).
                self.stops_index += 1

            if self.segment_idx + 1 >= len(self.node_queue):
                self.finish_ride(reason="path consumed")
                return

            self._shift_segment(1)
            return


        self.publish_path_mode()

        self._ensure_forward_window()
        # self._advance_if_reached(tol=self.REACH_TOL)  # legacy
        self._update_target_catchup()

        tgt = self._get_target_at(self.cur_buf, self.cur_idx)
        if tgt is None:
            self.finish_ride(reason="waypoints consumed")
            return

        _b, _i, pt = tgt
        self.pub_x.publish(Float32(data=float(pt[0])))
        self.pub_y.publish(Float32(data=float(pt[1])))

    def path_loop(self):
        # Slow-rate stop-state manager (ticks at self.path_loop_time).
        if self.yolo_cooldown_counter > 0:
            self.yolo_cooldown_counter -= 1

        # stop_flag meanings:
        #   0: running
        #   1: stop-sign fixed-duration stop (countdown here)
        #   2: traffic-light stop (held until green in cb_TLight)
        #   3: pickup/dropoff stop (from stops_file)

        if self.stop_flag == 2:
            # Traffic light stop: hold until cb_TLight receives green and releases.
            return

        if self.stop_flag == 3:
            # Pickup/Dropoff (from stops_file): fixed-duration hold (legacy behavior)
            if self.stop_counter == 0:
                self.stop_counter = self.PICKDROP_TICKS
                self.get_logger().info(f"Pick/Drop hold armed: {self.PICKDROP_TICKS} ticks")

            self.stop_counter -= 1
            if self.stop_counter <= 0:
                self.get_logger().info("Pick/Drop hold done -> RESUME + re_anchor")
                self.stop_flag = 0
                self.stop_counter = 0
                self.pub_stop.publish(Int32(data=0))
                self.pub_pick_drop.publish(Int32(data=0))
                self.re_anchor()
            return

        if self.stop_flag == 1:
            # Stop sign: fixed-duration hold
            if self.stop_counter == 0:
                self.stop_counter = self.TICKS_TO_WAIT
                self.get_logger().info(f"Stop Sign hold armed: {self.TICKS_TO_WAIT} ticks")

            self.stop_counter -= 1
            if self.stop_counter <= 0:
                self.get_logger().info("Stop Sign hold done -> RESUME + re_anchor")
                self.stop_flag = 0
                self.stop_counter = 0
                self.pub_stop.publish(Int32(data=0))
                self.pub_pick_drop.publish(Int32(data=0))
                self.re_anchor()
            return
    # ---------------------------
    # Start
    # ---------------------------

    def start(self):
        ride_char = chr(self.ride)

        self.declare_parameter('ride_file', f'path_{ride_char}.json')
        ride_file = self.get_parameter('ride_file').value
        ride_path = self._resolve_paths_json(ride_file)
        self.get_logger().info(f"[helper] ride_file: {ride_file} -> {ride_path}")
        with open(ride_path, 'r') as f:
            total = json.load(f)

        self.declare_parameter('stops_file', f'stop_{ride_char}.json')
        stops_file = self.get_parameter('stops_file').value
        stops_path = self._resolve_paths_json(stops_file)
        self.get_logger().info(f"[helper] stops_file: {stops_file} -> {stops_path}")
        with open(stops_path, 'r') as f:
            stops_cfg = json.load(f)

        self._parse_stops_config_Astyle(stops_cfg)

        if not total:
            self.get_logger().error("No path found.")
            self.node_queue = deque()
            return

        self.node_queue = deque(total)
        self.get_logger().info(f"Loaded nodes: {self.node_queue}")

        self.segment_idx = 0
        self.stops_index = 0
        self.stop_flag = 0
        self.stop_counter = 0
        self.goal_armed = False
        self._reset_buffers()

        first_node = self.node_queue[self.segment_idx]
        self._append_node_buffer(first_node)
        self._ensure_forward_window()

        self.cur_buf = 0
        self.cur_idx = 0

        self.create_timer(self.loop_time, self.loop)
        self.create_timer(self.path_loop_time, self.path_loop)

        # Publish forward path window for RViz
        self.create_timer(self.path_pub_time, self.publish_path)

        # Publish fixed-size 64x2 waypoints for Simulink (recommended control input)
        self.create_timer(self.path_pub_time, self.publish_waypoints)

        self.get_logger().info("Helper node initialized.")
        self.pub_stop.publish(Int32(data=0))


def main():
    rclpy.init()
    node = HelperPathSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
