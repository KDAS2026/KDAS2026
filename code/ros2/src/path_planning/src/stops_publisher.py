#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Int32MultiArray
import json

class JsonListPublisherNode(Node):
    def __init__(self):
        super().__init__('stop_list_publisher_node')

        # Define custom QoS profile
        custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )   

        self.publisher_ = self.create_publisher(Int32MultiArray, 'dqn_path_input', custom_qos)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.json_path = 'stops.json'
        self.data_list = self.load_json_list(self.json_path)
        self.index = 0

        if not self.data_list:
            self.get_logger().error("JSON file loaded but empty or invalid!")

    def load_json_list(self, path):
        try:
            with open(path, 'r') as f:
                data = json.load(f)
                if isinstance(data, list):
                    return data
                else:
                    self.get_logger().error("JSON root must be a list.")
                    return []
        except Exception as e:
            self.get_logger().error(f"Failed to load JSON file: {e}")
            return []

    def timer_callback(self):
        if self.index < len(self.data_list):
            row = self.data_list[self.index]
            if isinstance(row, list) and all(isinstance(x, int) for x in row):
                msg = Int32MultiArray()
                msg.data = row
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published row {self.index}: {row}")
            else:
                self.get_logger().warn(f"Row {self.index} is not a list of integers: {row}")
            self.index += 1
        else:
            self.get_logger().info("All rows have been published.")
            self.timer.cancel()  # Stop publishing

def main(args=None):
    rclpy.init(args=args)
    node = JsonListPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
