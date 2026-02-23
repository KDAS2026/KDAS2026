#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Int32MultiArray, Bool, String
import numpy as np
import random
import networkx as nx
import collections
import json
import threading
import time
from collections import deque

# ======== Graph Setup ========
graph = {
    0: [1],    1: [2],    2: [3],    3: [4,42],    4: [5],
    5: [6],    6: [7,10], 7: [8,36], 8: [9,39], 9: [6],
    10: [11], 11: [12],  12: [13],  13: [14],  14: [15,56],
    15: [16], 16: [17,18,19], 17: [55], 18: [29], 19: [34],
    20: [29], 21: [34], 22: [44],  23: [34], 24: [44],
    25: [55], 26: [44], 27: [55], 28: [29], 29: [30,58],
    30: [31], 31: [32], 32: [33],  33: [20,21,22], 34: [35],
    35: [9,39],36: [37], 37: [38],  38: [26,27,28], 39: [40],
    40: [41,51],41: [43],42: [43], 43: [23,24,25], 44: [45],
    45: [46], 46: [47,52],47: [48], 48: [49], 49: [50],
    50: [8,36],51: [31],52: [53],  53: [54],  54: [47,52],
    55: [2],  56: [57], 57: [59], 58: [5], 59: []
}
num_nodes = max(graph.keys()) + 1

# ======== DQN Parameters ========
episodes       = 5000
max_steps      = 100
alpha          = 0.001
gamma          = 0.9
epsilon_start  = 0.5
hidden_dim     = 64

def bfs_shortest_path(s, g):
    dq = deque([[s]])
    seen = {s}
    while dq:
        path = dq.popleft()
        u = path[-1]
        if u == g:
            return path
        for v in graph.get(u, []):
            if v not in seen:
                seen.add(v)
                dq.append(path + [v])
    return [s]

def get_reward_dynamic(goal):
    branch_nodes = {n for n, outs in graph.items() if len(outs) > 1}
    DG = nx.DiGraph(graph)
    loop_nodes = {
        node
        for comp in nx.strongly_connected_components(DG)
        if len(comp) > 1
        for node in comp
    }
    def reward_fn(current, next_node, step):
        if next_node == goal:
            return 100
        if next_node in loop_nodes:
            return -1000
        if next_node in branch_nodes:
            return 5
        return -1
    return reward_fn

class SimpleDQN:
    def __init__(self, in_dim, hid_dim, out_dim):
        self.W1 = np.random.randn(in_dim, hid_dim) * 0.01
        self.b1 = np.zeros((1, hid_dim))
        self.W2 = np.random.randn(hid_dim, out_dim) * 0.01
        self.b2 = np.zeros((1, out_dim))
    def forward(self, x):
        self.z1 = x @ self.W1 + self.b1
        self.a1 = np.maximum(self.z1, 0)
        self.z2 = self.a1 @ self.W2 + self.b2
        return self.z2
    def train(self, x, target, lr):
        out = self.forward(x)
        dloss = 2 * (out - target) / target.shape[0]
        dW2 = self.a1.T @ dloss
        db2 = dloss.sum(axis=0, keepdims=True)
        da1 = dloss @ self.W2.T
        dz1 = da1 * (self.z1 > 0)
        dW1 = x.T @ dz1
        db1 = dz1.sum(axis=0, keepdims=True)
        self.W1 -= lr * dW1; self.b1 -= lr * db1
        self.W2 -= lr * dW2; self.b2 -= lr * db2

def get_state_vector(n):
    v = np.zeros((1, num_nodes))
    v[0, n] = 1
    return v

def dqn_path(start, goal):
    reward_fn = get_reward_dynamic(goal)
    model     = SimpleDQN(num_nodes, hidden_dim, num_nodes)
    PENALTY_P = 10
    for ep in range(episodes):
        epsilon = epsilon_start * (1 - ep/(episodes-1))
        state   = start
        visit_counts = collections.Counter([state])
        for step in range(max_steps):
            vec    = get_state_vector(state)
            q_vals = model.forward(vec)[0]
            acts   = graph[state]
            if not acts: break
            if random.random() < epsilon:
                a = random.choice(acts)
            else:
                a = acts[np.argmax([q_vals[x] for x in acts])]
            r = reward_fn(state, a, step)
            cnt = visit_counts[a]
            if cnt > 0:
                r -= cnt * PENALTY_P
            next_q = model.forward(get_state_vector(a))[0]
            maxn   = max([next_q[x] for x in graph[a]] + [0]) if graph[a] else 0
            target = q_vals.copy()
            target[a] = r + gamma * maxn
            model.train(vec, target.reshape(1, -1), alpha)
            state = a
            visit_counts[state] += 1
            if state == goal:
                break
    path = [start]
    state = start
    for _ in range(200):
        q_vals = model.forward(get_state_vector(state))[0]
        acts   = graph[state]
        if not acts:
            break
        a = acts[np.argmax([q_vals[x] for x in acts])]
        if a in path:
            break
        path.append(a)
        state = a
        if state == goal:
            break
    if path[-1] != goal:
        path = bfs_shortest_path(start, goal)
    return path

def compute_dqn_paths(points):
    if points[-1] != 59:
        points.append(59)
    final = points[-1]
    paths, total = [], []
    for i in range(len(points)-1):
        p = dqn_path(points[i], points[i+1])
        paths.append(p)
        for node in p:
            if total and total[-1] == node:
                continue
            total.append(node)
            if node == final:
                return paths, total
    return paths, total

# ======== ROS2 Node ========
class DQNPathPlanner(Node):
    def __init__(self):
        super().__init__('dqn_path_planner')
        custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.subscriber = self.create_subscription(Int32MultiArray, 'dqn_path_input', self.input_callback, custom_qos)
        self.dqn_done_publisher = self.create_publisher(Bool, 'dqn_done', custom_qos)
        self.path_result_publisher = self.create_publisher(String, 'dqn_path_result', custom_qos)

        self.computing = False

        startup_msg = Bool()
        startup_msg.data = False
        self.dqn_done_publisher.publish(startup_msg)

        self.get_logger().info("DQN Path Planner Node Started (input: /dqn_path_input → output: /dqn_done, /dqn_path_result)")

    def input_callback(self, msg):
        if self.computing:
            self.get_logger().warn("Path computation already in progress, ignoring new request.")
            return
        stops = list(msg.data)
        if not all(isinstance(x, int) and 0 <= x <= 59 for x in stops):
            self.get_logger().error("Invalid stop point(s). Must be integers from 0 to 59.")
            return
        full_points = [0] + stops + [59]
        self.get_logger().info(f"Computing path for waypoints: {full_points}")
        self.computing = True
        threading.Thread(target=self.run_dqn_thread, args=(full_points,), daemon=True).start()

    def run_dqn_thread(self, points):
        try:
            start_time = time.time()
            paths, total = compute_dqn_paths(points)
            result = {f"p{idx+1}": p for idx, p in enumerate(paths)}
            result["total"] = total
            json_result = json.dumps(result, indent=2)

            # Save to file
            save_path = 'dqn_paths.json'
            try:
                with open(save_path, 'w') as f:
                    json.dump(result, f, indent=2)
                self.get_logger().info(f"Saved DQN path result to: {save_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to save JSON file: {e}")

            # Publish Done Message
            done_msg = Bool()
            done_msg.data = True
            self.dqn_done_publisher.publish(done_msg)

            self.path_result_publisher.publish(String(data=json_result))
            elapsed = time.time() - start_time
            self.get_logger().info(f"DQN path computation completed in {elapsed:.2f}s and published.")
        except Exception as e:
            self.get_logger().error(f"Error during path computation: {str(e)}")
        finally:
            self.computing = False

def main(args=None):
    rclpy.init(args=args)
    node = DQNPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
