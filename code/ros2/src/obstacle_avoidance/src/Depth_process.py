#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
from skimage import measure

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obs_detector')
        # 카메라 파라미터
        self.fx, self.fy = 385.6, 385.6
        self.cx, self.cy = 321.9, 237.3

        # 차량 크기 (m)
        self.car_length = 0.4

        # 구독 & 퍼블리셔
        self.sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.cb, 10)
        self.pub = self.create_publisher(
            Float32MultiArray, '/obstacle_info', 10)
        self.bridge = CvBridge()

        # ROI: 도로 아래 절반
        self.roi_xmin, self.roi_xmax = 0, 640
        self.roi_ymin, self.roi_ymax = 240, 480

    def cb(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg)  # float32, m
        roi   = depth[self.roi_ymin:self.roi_ymax,
                      self.roi_xmin:self.roi_xmax]

        mask  = (roi < 2.0).astype(np.uint8)
        mask  = measure.label(mask)
        props = measure.regionprops(mask)
        if not props or props[0].area < 100:
            return

        region = props[0]
        coords   = region.coords
        left_idx = np.argmin(coords[:,1])
        local_row, local_col = coords[left_idx]
        img_row = int(local_row) + self.roi_ymin
        img_col = int(local_col) + self.roi_xmin

        z_front = depth[img_row, img_col]
        if not np.isfinite(z_front) or z_front <= 0.0:
            return

        x_front = (img_col - self.cx) * z_front / self.fx
        y_front = ((region.centroid[0] + self.roi_ymin) - self.cy) * z_front / self.fy

        # NEW: pixel distance from image center to leftmost obstacle pixel
        r_pix = abs(img_col - self.cx)

        # If you want a “real-world” lateral offset in metres instead:
        # r_m = r_pix * z_front / self.fx

        x_obs = x_front + self.car_length/2.0
        y_obs = y_front

        out = Float32MultiArray(data=[
            float(x_obs),
            float(y_obs),
            float(r_pix)      # or float(r_m)
        ])
        self.pub.publish(out)
        self.get_logger().debug(
            f"Published obstacle_info: x_obs={x_obs:.2f}, "
            f"y_obs={y_obs:.2f}, r_pix={r_pix:.1f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
