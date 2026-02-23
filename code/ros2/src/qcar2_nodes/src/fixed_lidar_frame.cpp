#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class SyncFrameBroadcaster : public rclcpp::Node
{
public:
  SyncFrameBroadcaster()
  : Node("synced_lidar_frame")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&SyncFrameBroadcaster::scan_callback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/qcar2_imu", 10, std::bind(&SyncFrameBroadcaster::imu_callback, this, std::placeholders::_1));

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/qcar2_scan", 10);
    clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_header_ = msg->header;
    imu_time_available_ = true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!imu_time_available_) {
      RCLCPP_WARN(this->get_logger(), "IMU timestamp not yet available, skipping scan");
      return;
    }

    // ── TF: base_link → base_scan with IMU time ───────────────────
    geometry_msgs::msg::TransformStamped t;
    t.header = last_imu_header_;
    t.header.frame_id = "base_link";
    t.child_frame_id = "base_scan";

    t.transform.translation.x = 0.1;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 3.14);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);

    // ── Clock message ──────────────────────────────────────────────
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = last_imu_header_.stamp;
    clock_pub_->publish(clock_msg);

    // ── Publish /scan → /qcar2_scan with IMU header ───────────────
    auto scan_with_imu_header = *msg;  // copy original scan message
    scan_with_imu_header.header = last_imu_header_;
    scan_pub_->publish(scan_with_imu_header);
  }

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // State
  std_msgs::msg::Header last_imu_header_;
  bool imu_time_available_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
