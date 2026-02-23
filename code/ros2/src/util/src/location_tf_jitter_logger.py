#!/usr/bin/env python3
import csv
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener

class LocationTfJitterLogger(Node):
    def __init__(self):
        super().__init__('location_tf_jitter_logger')

        # Parameters
        self.declare_parameter('location_topic', 'location')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('output', 'location_vs_tf_jitter.csv')
        self.declare_parameter('rate_hz', 20.0)

        self.location_topic = self.get_parameter('location_topic').value
        self.parent = self.get_parameter('parent_frame').value
        self.child = self.get_parameter('child_frame').value
        self.rate = float(self.get_parameter('rate_hz').value)
        self.output = self.get_parameter('output').value

        # TF
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # Latest location
        self.last_loc = None  # (x, y, yaw)
        self.last_loc_time = None

        # Subscription: location
        self.create_subscription(Point, self.location_topic, self.location_cb, 10)

        # CSV
        self.file = open(self.output, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            't',  # logger time [s] since start
            'loc_x', 'loc_y', 'loc_yaw',
            'tf_x', 'tf_y', 'tf_yaw',
            'loc_age_s',  # how old the latest location is at logging time
            'tf_ok', 'loc_ok'
        ])

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate, self.timer_cb)

        self.get_logger().info(
            f"Logging location('{self.location_topic}') and TF({self.parent}->{self.child}) to {self.output}"
        )

    def location_cb(self, msg: Point):
        # NOTE: Do not round/truncate. Keep raw float values.
        now = self.get_clock().now()
        self.last_loc = (float(msg.x), float(msg.y), float(msg.z))
        self.last_loc_time = now

    def _quat_to_yaw(self, q):
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def timer_cb(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9

        # Read TF
        tf_ok = 0
        tf_x = tf_y = tf_yaw = float('nan')
        try:
            tf = self.buffer.lookup_transform(self.parent, self.child, rclpy.time.Time())
            tr = tf.transform.translation
            qr = tf.transform.rotation
            tf_x, tf_y = float(tr.x), float(tr.y)
            tf_yaw = float(self._quat_to_yaw(qr))
            tf_ok = 1
        except Exception:
            pass

        # Read latest location
        loc_ok = 0
        loc_x = loc_y = loc_yaw = float('nan')
        loc_age_s = float('nan')
        if self.last_loc is not None and self.last_loc_time is not None:
            loc_x, loc_y, loc_yaw = self.last_loc
            loc_age_s = (now - self.last_loc_time).nanoseconds * 1e-9
            loc_ok = 1

        self.writer.writerow([t, loc_x, loc_y, loc_yaw, tf_x, tf_y, tf_yaw, loc_age_s, tf_ok, loc_ok])

    def destroy_node(self):
        self.file.close()
        self.get_logger().info("CSV file closed and data saved.")
        super().destroy_node()

def main():
    rclpy.init()
    node = LocationTfJitterLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
