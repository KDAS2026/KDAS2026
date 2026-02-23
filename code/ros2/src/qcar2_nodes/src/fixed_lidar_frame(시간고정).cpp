#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

// 정적 TF 브로드캐스터
#include "tf2_ros/static_transform_broadcaster.h"

class SyncFrameBroadcaster : public rclcpp::Node
{
public:
  SyncFrameBroadcaster()
  : Node("synced_lidar_frame")
  {
    // 정적 TF 브로드캐스터 생성 (/tf_static)
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // base_link -> base_scan 정적 TF는 "여기서 1번만" publish
    publish_static_tf_once();

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&SyncFrameBroadcaster::scan_callback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/qcar2_imu", 10,
      std::bind(&SyncFrameBroadcaster::imu_callback, this, std::placeholders::_1));

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/qcar2_scan", 10);
  }

private:
  void publish_static_tf_once()
  {
    geometry_msgs::msg::TransformStamped t;

    // 정적 TF는 /tf_static에 올라가면 "시간에 의존하지 않게" 취급됩니다.
    // stamp는 관례적으로 now를 넣습니다.
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id  = "base_scan";

    t.transform.translation.x = 0.1;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 3.14);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    static_tf_broadcaster_->sendTransform(t);

    RCLCPP_INFO(this->get_logger(), "Published static TF on /tf_static: base_link -> base_scan");
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_header_ = msg->header;
    imu_time_available_ = true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // IMU는 계속 쓰되, scan 자체를 막고 싶지 않으면 아래 return을 빼도 됩니다.
    // 지금은 너 코드 의도를 존중해서 유지.
    if (!imu_time_available_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "IMU timestamp not yet available, skipping scan");
      return;
    }

    // scan은 stamp 유지, frame_id만 base_scan으로 통일
    auto out = *msg;
    out.header.frame_id = "base_scan";
    scan_pub_->publish(out);
  }

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  // Static TF broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

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
