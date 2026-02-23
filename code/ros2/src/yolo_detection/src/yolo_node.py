#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from ultralytics import YOLO


class QCarDetectionNode(Node):
    """
    YOLO based detection node for QCar.

    Publishes (perception-only):
      * /yolo_stop (Int32): stop-sign EVENT (pulse)
      * /yolo_traffic_light (Int32): traffic light state (1=RED, 0=NOT-RED)

    Added visualization:
      * /yolo/annotated (sensor_msgs/Image): camera frame with bbox/label/conf overlay

    Subscribes:
      - /camera/color_image (sensor_msgs/Image)   [configurable via camera_topic]
    """

    def __init__(self):
        super().__init__('qcar_detection_node')

        # -----------------------
        # 1) Parameters (Launch 연동)
        # -----------------------
        self.declare_parameter('model_path', '')
        self.declare_parameter('camera_topic', '/camera/color_image')
        self.declare_parameter('stop_cooldown_sec', 10.0)
        self.declare_parameter('publish_annotated', True)          # <-- added
        self.declare_parameter('annotated_topic', '/yolo/annotated')  # <-- added

        model_path_param = self.get_parameter('model_path').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.stop_cooldown_sec = float(self.get_parameter('stop_cooldown_sec').get_parameter_value().double_value)

        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)
        self.annotated_topic = self.get_parameter('annotated_topic').get_parameter_value().string_value

        # If model_path is empty (e.g., running without launch), try package default.
        if not model_path_param:
            try:
                pkg_share = get_package_share_directory('yolo_detection')
                model_path_param = os.path.join(pkg_share, 'models', '0126yolo.pt')
            except Exception:
                pass

        # Fallback: try local file
        if not os.path.exists(model_path_param):
            self.get_logger().warn(f"Model not found at: {model_path_param}")
            model_path_param = '0126yolo.pt'

        self.get_logger().info(f"Loading YOLO model from: {model_path_param}")
        self.get_logger().info(f"Camera topic: {self.camera_topic}")
        self.get_logger().info(f"Publish annotated: {self.publish_annotated} -> {self.annotated_topic}")

        # -----------------------
        # 2) YOLO model
        # -----------------------
        try:
            self.model = YOLO(model_path_param)
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise

        # -----------------------
        # 3) Detection config
        # -----------------------
        self.fps = 10.0
        self.threshold_sec = 4.0     # require continuous detection for robustness
        self.conf_thres = 0.6
        self.imgsz = (480, 640)

        # Class IDs (dataset-dependent)
        self.CLS_STOP = 0
        self.CLS_RED = 1

        # Red-latch release: how many consecutive "miss frames" to release red state
        self.red_release_miss_frames = 10

        self.threshold_frames = max(1, int(round(self.fps * self.threshold_sec)))
        self.stop_cooldown_ns = int(self.stop_cooldown_sec * 1e9)

        # -----------------------
        # 4) ROS I/O
        # -----------------------
        self.sub_image = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10
        )
        self.pub_yolo_stop = self.create_publisher(Int32, '/yolo_stop', 10)
        self.pub_tl = self.create_publisher(Int32, '/yolo_traffic_light', 10)

        # (added) annotated image publisher
        self.pub_annot = None
        if self.publish_annotated:
            self.pub_annot = self.create_publisher(Image, self.annotated_topic, 10)

        self.bridge = CvBridge()
        self.latest_frame = None

        # -----------------------
        # 5) Internal state
        # -----------------------
        self.count_stop = 0
        self.count_red = 0

        self.red_latched = False
        self.red_miss_count = 0

        self.stop_cooldown_until_ns = 0

        self.timer = self.create_timer(1.0 / self.fps, self.process_frame)
        self.get_logger().info("QCarDetectionNode started.")

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            self.latest_frame = None

    def publish_yolo_stop(self, value: int):
        msg = Int32()
        msg.data = int(value)
        self.pub_yolo_stop.publish(msg)

    def publish_traffic_light(self, value: int):
        msg = Int32()
        msg.data = int(value)
        self.pub_tl.publish(msg)

    def publish_annotated_image(self, bgr_img):
        if self.pub_annot is None:
            return
        try:
            msg = self.bridge.cv2_to_imgmsg(bgr_img, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_annot.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish annotated image: {e}")

    def process_frame(self):
        now = self.now_ns()

        if self.latest_frame is None:
            # No image -> output safe defaults
            self.publish_yolo_stop(0)
            self.publish_traffic_light(0)
            return

        # (important) copy frame to avoid being overwritten mid-inference
        frame = self.latest_frame.copy()

        # YOLO inference
        try:
            results = self.model(frame, imgsz=self.imgsz, conf=self.conf_thres, verbose=False)
        except Exception as e:
            self.get_logger().error(f"YOLO inference error: {e}")
            self.publish_yolo_stop(0)
            self.publish_traffic_light(0)
            return

        # (added) visualization: draw boxes/labels/conf on frame
        if self.publish_annotated and results is not None and len(results) > 0:
            try:
                annotated = results[0].plot()  # returns BGR numpy image
                self.publish_annotated_image(annotated)
            except Exception as e:
                self.get_logger().warn(f"Annotate/plot failed: {e}")

        detected = set()
        for r in results:
            if r.boxes is None:
                continue
            for cls_idx in r.boxes.cls:
                detected.add(int(cls_idx))

        has_stop = (self.CLS_STOP in detected)
        has_red = (self.CLS_RED in detected)

        # --- Stop-sign "stability" counter (event generation)
        self.count_stop = self.count_stop + 1 if has_stop else 0

        stop_event = 0
        if now >= self.stop_cooldown_until_ns:
            if self.count_stop >= self.threshold_frames:
                stop_event = 1
                self.stop_cooldown_until_ns = now + self.stop_cooldown_ns
                self.count_stop = 0

        # --- Traffic light red latch (state output)
        self.count_red = self.count_red + 1 if has_red else 0

        if not self.red_latched:
            if self.count_red >= self.threshold_frames:
                self.red_latched = True
                self.red_miss_count = 0
        else:
            if has_red:
                self.red_miss_count = 0
            else:
                self.red_miss_count += 1
                if self.red_miss_count >= self.red_release_miss_frames:
                    self.red_latched = False
                    self.red_miss_count = 0
                    self.count_red = 0

        tl_out = 1 if self.red_latched else 0

        # Final outputs
        self.publish_yolo_stop(stop_event)
        self.publish_traffic_light(tl_out)


def main(args=None):
    rclpy.init(args=args)
    node = QCarDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
