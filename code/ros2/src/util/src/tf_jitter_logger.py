#!/usr/bin/env python3
import csv
import math
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

class TFJitterLogger(Node):
    def __init__(self):
        super().__init__('tf_jitter_logger')

        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('output', 'tf_jitter.csv')
        self.declare_parameter('rate_hz', 20.0)

        self.parent = self.get_parameter('parent_frame').value
        self.child = self.get_parameter('child_frame').value
        self.rate = self.get_parameter('rate_hz').value

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.file = open(
            self.get_parameter('output').value,
            'w',
            newline=''
        )
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time', 'x', 'y', 'yaw'])

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(
            1.0 / self.rate,
            self.timer_cb
        )

        self.get_logger().info(
            f'Logging TF {self.parent} -> {self.child}'
        )

    def timer_cb(self):
        try:
            tf = self.buffer.lookup_transform(
                self.parent,
                self.child,
                rclpy.time.Time()
            )

            t = tf.transform.translation
            q = tf.transform.rotation

            yaw = euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )[2]

            now = self.get_clock().now()
            t_sec = (now - self.start_time).nanoseconds * 1e-9

            # ★ 절대 반올림/캐스팅 없음 ★
            self.writer.writerow([
                t_sec,
                t.x,
                t.y,
                yaw
            ])

        except Exception:
            pass

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = TFJitterLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
