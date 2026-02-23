#!/usr/bin/env python3
"""
LED Strip Controller (ROS2 Humble compatible)

Controls QCar2 LED strip via:
  /qcar2_hardware/set_parameters  (rcl_interfaces/srv/SetParameters)

Rule mapping:
  - Moving                       -> GREEN
  - Pick-Up (stopped)            -> BLUE
  - Drop-Off (stopped)           -> YELLOW
  - Otherwise stopped            -> RED

qcar2_hardware led_color_id:
  0=red, 1=green, 2=blue, 3=yellow
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterValue, ParameterType


class LEDController(Node):
    def __init__(self):
        super().__init__('led_controller')

        self.target_node = '/qcar2_hardware'
        self.param_name = 'led_color_id'
        self.service_name = f'{self.target_node}/set_parameters'

        self.stop = True
        self.pickdrop = 0

        self.last_led = None
        self.pending = False

        self.create_subscription(Int32, '/stop', self.cb_stop, 10)
        self.create_subscription(Int32, '/pickup_dropoff', self.cb_pickdrop, 10)

        self.cli = self.create_client(SetParameters, self.service_name)

        self.create_timer(0.1, self.update_led)   # 10 Hz
        # self.create_timer(1.0, self.debug_print)  # DEBUG: 비활성화

        self.get_logger().info(
            f"LEDController running. Using service '{self.service_name}'"
        )

    def cb_stop(self, msg: Int32):
        self.stop = (msg.data == 1)

    def cb_pickdrop(self, msg: Int32):
        self.pickdrop = int(msg.data)

    # DEBUG 전용 – 필요 시만 활성화
    # def debug_print(self):
    #     self.get_logger().info(
    #         f"[dbg] stop={self.stop}, pickdrop={self.pickdrop}, "
    #         f"last_led={self.last_led}, pending={self.pending}"
    #     )

    def compute_led(self) -> int:
        if not self.stop:
            return 1  # GREEN
        if self.pickdrop == 1:
            return 2  # BLUE
        if self.pickdrop == 2:
            return 3  # YELLOW
        return 0      # RED

    def update_led(self):
        led = self.compute_led()

        if led == self.last_led:
            return
        if self.pending:
            return
        if not self.cli.service_is_ready():
            # self.get_logger().warn("set_parameters service not ready")
            return

        self.pending = True
        self.call_set_parameter_int(led)

    def call_set_parameter_int(self, value: int):
        req = SetParameters.Request()

        p = ParameterMsg()
        p.name = self.param_name

        pv = ParameterValue()
        pv.type = ParameterType.PARAMETER_INTEGER
        pv.integer_value = int(value)

        p.value = pv
        req.parameters = [p]

        future = self.cli.call_async(req)
        future.add_done_callback(lambda f: self.on_set_done(f, value))

    def on_set_done(self, future, value: int):
        self.pending = False
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Set failed: {e}")
            return

        if res.results and res.results[0].successful:
            self.last_led = value
            # self.get_logger().info(f"LED set -> {value}")
        else:
            reason = (
                res.results[0].reason
                if (res.results and len(res.results) > 0)
                else "no result"
            )
            self.get_logger().error(f"LED parameter rejected: {reason}")


def main():
    rclpy.init()
    node = LEDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
