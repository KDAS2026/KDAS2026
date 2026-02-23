#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;  // ✅ synced with scan time
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
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
