#!/usr/bin/env python3
import os
import json
import time
import math
import rclpy
from collections import deque
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Bool, Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory

shared = get_package_share_directory('path_planning')

class HelperPathSender(Node):
    def __init__(self):
        super().__init__('helper_path_sender')
        # QoS Configuration
        dqn_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Publishers and Subscribers
        self.pub_x = self.create_publisher(Float32, 'path_x', 100)
        self.pub_y = self.create_publisher(Float32, 'path_y', 100)
        self.pub_path_mode = self.create_publisher(Int32, 'path_mode', 100)
        self.pub_pick_drop = self.create_publisher(Int32, 'pickup_dropoff', 10)
        self.pub_stop = self.create_publisher(Int32, 'stop', 10)

        self.create_subscription(Point, 'location', self.cb_location, 20)
        self.create_subscription(Int32, 'stop', self.cb_stop, 20)
        self.create_subscription(Bool, 'dqn_done', self.cb_dqn_done, dqn_qos)

        # Parameters
        self.WINDOW         = 4      # publish in groups of 4 points
        self.COOLDOWN_SEC   = 1.0    # lock for 1 second after transition
        self.wp_dir         = os.path.join(shared, 'waypoints')

        self.path_mode_map = {
            0: 2, 1: 0, 2: 0, 3: 1, 4: 0, 5: 0, 6: 1, 7: 1, 8: 1, 9: 1,
            10: 0, 11: 0, 12: 1, 13: 0, 14: 0, 15: 1, 16: 0, 17: 1, 18: 0, 19: 1,
            20: 1, 21: 0, 22: 1, 23: 1, 24: 0, 25: 1, 26: 1, 27: 0, 28: 1, 29: 0,
            30: 1, 31: 1, 32: 0, 33: 1, 34: 1, 35: 1, 36: 1, 37: 1, 38: 0, 39: 1,
            40: 0, 41: 1, 42: 1, 43: 0, 44: 0, 45: 1, 46: 0, 47: 0, 48: 1, 49: 0,
            50: 1, 51: 0, 52: 1, 53: 1, 54: 1, 55: 1, 56: 0, 57: 2, 58: 1, 59: 0
        }
        # 0: straight, 1: curve, 2: start/finish

        # ── State variables ─────────────────────────────────────────────────────────
        self.pos = (0.0, 0.0)
        self.stop_flag = 0
        self.stops_index = 0
        self.stops = []

        self.segment_idx = 0
        self.buffer_path = []
        self.next_buffer = []
        self.current_index = 0
        self.next_index = 0
        self.last_trigger_time = 0.0
        self.path_loop_time = 0.75

        # tick-counter in place of sleep(5)
        self.stop_counter = 0
        # path_loop timer is self.path_loop_times ⇒ ~5s/self.path_loop_times ≈ 9.1 ⇒ round up to 10 ticks
        self.TICKS_TO_WAIT = int(math.ceil(5.0 / self.path_loop_time))

    def start(self):
        # ── Parameters ─────────────────────────────────────────
        self.declare_parameter('dqn_file', 'dqn_paths.json')
        with open(self.get_parameter('dqn_file').value, 'r') as f:
            total = json.load(f)['total']

        # load stops
        self.declare_parameter('stops_file', 'stops.json')
        with open(self.get_parameter('stops_file').value, 'r') as f:
            stops = json.load(f)

        # flatten nested lists if needed
        if any(isinstance(s, list) for s in stops):
            flat = []
            for s in stops:
                if isinstance(s, list):
                    flat.extend(s)
                else:
                    flat.append(s)
            stops = flat

        if not stops:
            self.get_logger().error("No valid stops found.")
            self.stops = []
        else:
            self.stops = stops
            self.stops.append(59)
            self.get_logger().info(f"loaded stops: {self.stops}")

        # Load first path sequence
        if not total:
            self.get_logger().error("No path found.")
            self.node_queue = []
        else:
            self.node_queue = deque(total)
            self.get_logger().info(f"loaded nodes: {self.node_queue}")

        self.preload_buffer(self.segment_idx)

        # Timers
        self.create_timer(0.01, self.loop)
        self.create_timer(self.path_loop_time, self.path_loop)
        self.get_logger().info("Helper node initialized.")

    # ── Callbacks ────────────────────────────────────────────────────────────────
    def cb_location(self, msg: Point):
        self.pos = (msg.x, msg.y)

    def cb_stop(self, msg: Int32) -> None:
        if msg.data == 1:
            self.stop_flag = 1
            self.get_logger().info('Manual Stop Activated')

    def cb_dqn_done(self, msg: Bool) -> None:
        if msg.data:
            self.start()
            self.pub_stop.publish(Int32(data=0))

    # ── File loading utilities ──────────────────────────────────────────────────
    def load_waypoints(self, index: int) -> list:
        node_id = self.node_queue[index] if isinstance(index, int) else index
        coords = []
        path = os.path.join(self.wp_dir, f'waypoints_{node_id}.json')
        try:
            with open(path, 'r') as f:
                pts = json.load(f)
            for p in pts:
                self.get_logger().info(f"load_waypoints({node_id})")
                coords.append([float(p[0]), float(p[1])])
        except Exception as e:
            self.get_logger().error(f"load_waypoints({node_id}) failed: {e}")
            return []
        return coords

    def preload_buffer(self, idx: int):
        key1 = self.node_queue[idx]
        key2 = self.node_queue[idx+1] if idx+1 < len(self.node_queue) else None

        self.buffer_path = self.load_waypoints(key1)
        self.next_buffer = self.load_waypoints(key2) if key2 else []

        self.next_index = 0
        self.get_logger().info(f"Loaded [{key1}]")
        if key2:
            self.get_logger().info(f"Preloaded [{key2}]")

    # ── Publishing ───────────────────────────────────────────────────────────────
    def publish_waypoint(self):
        if self.stop_flag == 0:
            if not self.buffer_path:
                self.get_logger().warn("buffer is empty")
                return

            if self.current_index < len(self.buffer_path):
                pt = self.buffer_path[self.current_index]
                self.pub_x.publish(Float32(data=pt[0]))
                self.pub_y.publish(Float32(data=pt[1]))
                self.current_index += 1
                self.get_logger().info(f"path {self.node_queue[self.segment_idx]}: ({pt[0]}, {pt[1]})")
            elif self.next_index < len(self.next_buffer):
                pt = self.next_buffer[self.next_index]
                self.pub_x.publish(Float32(data=pt[0]))
                self.pub_y.publish(Float32(data=pt[1]))
                self.next_index += 1
                self.get_logger().info(f"preloaded {self.node_queue[self.segment_idx+1]}: ({pt[0]}, {pt[1]})")

    def publish_path_mode(self) -> None:
        if self.segment_idx in self.path_mode_map:
            mode = self.path_mode_map[self.segment_idx]
            self.pub_path_mode.publish(Int32(data=mode))

    def end_trigger(self, tol: float = 0.28) -> bool:
        next_idx = self.segment_idx + 1
        if next_idx >= len(self.node_queue):
            return False
        x_target, y_target = self.buffer_path[-1]
        x, y = self.pos
        distance = math.hypot(x - x_target, y - y_target)
        return distance <= tol

    # ── Main loops ───────────────────────────────────────────────────────────────
    def loop(self):
        now = time.monotonic()
        if self.end_trigger():
            self.get_logger().info(f"Trigger: node {self.node_queue[self.segment_idx]}, stop location {self.stops[self.stops_index]}")
            if int(self.stops[self.stops_index]) == int(self.node_queue[self.segment_idx]):
                self.get_logger().info("Stop at stop point")
                if self.stops_index == 0:
                    self.pub_pick_drop.publish(Int32(data=1))  # pickup
                elif self.stops_index == len(self.stops) - 1:
                    self.pub_pick_drop.publish(Int32(data=2))  # dropoff
                self.pub_stop.publish(Int32(data=1))
                self.stop_flag = 1
                self.stops_index += 1

            self.segment_idx += 1
            self.buffer_path = self.next_buffer
            self.current_index = self.next_index
            self.preload_buffer(self.segment_idx)
            self.last_trigger_time = now
            return

        self.publish_path_mode()

    def path_loop(self):
        if self.stop_flag == 1:
            # first tick: preload and start counter
            if self.stop_counter == 0:
                self.get_logger().info("stop detected")
                self.preload_buffer(self.segment_idx)
                self.stop_counter = self.TICKS_TO_WAIT

            # decrement tick counter
            self.stop_counter -= 1

            # when counter expires, resume
            if self.stop_counter <= 0:
                self.get_logger().info("start")
                self.stop_flag = 0
                self.stop_counter = 0
                self.pub_stop.publish(Int32(data=0))
            else:
                return
        else:
            self.publish_waypoint()

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
