#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Float32

import time
import math
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class LivePlot(Node):
    def __init__(self):
        super().__init__('live_plot')

        # ----- buffers
        self.t_buf = deque(maxlen=800)

        self.v_cur_buf = deque(maxlen=800)  # estimated current speed
        self.v_cmd_buf = deque(maxlen=800)  # commanded speed (/simulinkOut.x)

        self.x_buf = deque(maxlen=800)
        self.y_buf = deque(maxlen=800)

        self.target_x = None
        self.target_y = None

        self.v_cmd = None

        # speed estimation state
        self.last_x = None
        self.last_y = None
        self.last_t = None

        self.v_ema = 0.0
        self.ema_alpha = 0.2  # 0~1 (0.1~0.3 권장)

        self.t0 = None

        # ----- subscribers
        self.create_subscription(Point, 'location', self.cb_location, 50)
        self.create_subscription(Point, '/simulinkOut', self.cb_simulink_out, 50)
        self.create_subscription(Float32, 'path_x', self.cb_path_x, 50)
        self.create_subscription(Float32, 'path_y', self.cb_path_y, 50)

        # ----- plots
        self.fig1, self.ax1 = plt.subplots()
        self.ax1.set_title("Speed: estimated current vs command")
        self.ax1.set_xlabel("t [s]")
        self.ax1.set_ylabel("speed [m/s]")
        (self.line_vcur,) = self.ax1.plot([], [], label="v_current_est")
        (self.line_vcmd,) = self.ax1.plot([], [], label="v_command")
        self.ax1.legend(loc="upper right")
        self.ax1.grid(True)

        self.fig2, self.ax2 = plt.subplots()
        self.ax2.set_title("XY: current trajectory + target point")
        self.ax2.set_xlabel("x [m]")
        self.ax2.set_ylabel("y [m]")
        (self.line_xy,) = self.ax2.plot([], [], label="current_traj")
        self.sc_target = self.ax2.scatter([], [], marker="x", label="target")
        self.ax2.legend(loc="upper right")
        self.ax2.grid(True)
        self.ax2.axis("equal")

        self.ani1 = FuncAnimation(self.fig1, self.update_speed_plot, interval=100)
        self.ani2 = FuncAnimation(self.fig2, self.update_xy_plot, interval=100)

        self.get_logger().info("LivePlot started: speed is estimated by differentiating /location")

    # ----- callbacks
    def cb_simulink_out(self, msg: Point):
        # /simulinkOut.x = output_speed
        self.v_cmd = float(msg.x)

    def cb_path_x(self, msg: Float32):
        self.target_x = float(msg.data)

    def cb_path_y(self, msg: Float32):
        self.target_y = float(msg.data)

    def cb_location(self, msg: Point):
        now = time.time()
        if self.t0 is None:
            self.t0 = now

        x = float(msg.x)
        y = float(msg.y)

        # --- estimate current speed from position derivative
        v_est = float('nan')
        if self.last_x is not None and self.last_t is not None:
            dt = now - self.last_t

            # dt guard: reject absurd dt to avoid spikes
            if 1e-3 < dt < 1.0:
                dx = x - self.last_x
                dy = y - self.last_y
                v_raw = math.hypot(dx, dy) / dt

                # EMA low-pass
                self.v_ema = self.ema_alpha * v_raw + (1.0 - self.ema_alpha) * self.v_ema
                v_est = self.v_ema

        self.last_x, self.last_y, self.last_t = x, y, now

        # --- push buffers
        self.t_buf.append(now - self.t0)
        self.x_buf.append(x)
        self.y_buf.append(y)

        self.v_cur_buf.append(v_est)
        self.v_cmd_buf.append(self.v_cmd if self.v_cmd is not None else float('nan'))

    # ----- plot updaters
    def update_speed_plot(self, _):
        if len(self.t_buf) < 2:
            return (self.line_vcur, self.line_vcmd)

        t = list(self.t_buf)
        vcur = list(self.v_cur_buf)
        vcmd = list(self.v_cmd_buf)

        self.line_vcur.set_data(t, vcur)
        self.line_vcmd.set_data(t, vcmd)

        self.ax1.relim()
        self.ax1.autoscale_view()
        return (self.line_vcur, self.line_vcmd)

    def update_xy_plot(self, _):
        if len(self.x_buf) < 2:
            return (self.line_xy, self.sc_target)

        xs = list(self.x_buf)
        ys = list(self.y_buf)
        self.line_xy.set_data(xs, ys)

        if self.target_x is not None and self.target_y is not None:
            self.sc_target.set_offsets([[self.target_x, self.target_y]])

        self.ax2.relim()
        self.ax2.autoscale_view()
        self.ax2.axis("equal")
        return (self.line_xy, self.sc_target)


def main():
    rclpy.init()
    node = LivePlot()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
