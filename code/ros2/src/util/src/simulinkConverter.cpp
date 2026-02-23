#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"

#include <csignal>
#include <chrono>
#include <thread>
#include <algorithm>  // for std::clamp

class SimulinkOutSubscriber : public rclcpp::Node
{
public:
  SimulinkOutSubscriber()
  : Node("simulink_out_subscriber"),
    throttle(0.0),
    steering_angle(0.0)
  {
    // Declare and retrieve parameter for per-vehicle steering correction
    this->declare_parameter<double>("steering_offset", 0.0);
    this->get_parameter("steering_offset", steering_offset);

    // Subscription to Simulink output
    subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/simulinkOut", 100,
      std::bind(&SimulinkOutSubscriber::topic_callback, this, std::placeholders::_1)
    );

    // Publisher to motor command interface
    command_publisher_ = this->create_publisher<qcar2_interfaces::msg::MotorCommands>(
      "/qcar2_motor_speed_cmd", 100
    );
  }

  void topic_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    // Update values
    throttle = msg->x;
    steering_angle = msg->y;

    // Apply offset and clamp
    double corrected_throttle = std::clamp(throttle, -1.0, 1.0);
    double corrected_steering = std::clamp(steering_angle + steering_offset, -1.0, 1.0);

    // Publish corrected command
    auto msgOut = qcar2_interfaces::msg::MotorCommands();
    msgOut.motor_names = {"steering_angle", "motor_throttle"};
    msgOut.values = {corrected_steering, corrected_throttle};

    command_publisher_->publish(msgOut);
  }

  void gradual_stop_and_shutdown()
  {
    RCLCPP_WARN(this->get_logger(), "Interrupt received: gradually stopping the vehicle...");

    double current_throttle = throttle;
    double current_steering = steering_angle;

    const int steps = 20;
    const double step_throttle = current_throttle / steps;
    const double step_steering = current_steering / steps;

    for (int i = 0; i < steps; ++i)
    {
      current_throttle -= step_throttle;
      current_steering -= step_steering;

      double corrected_throttle = std::clamp(current_throttle, -1.0, 1.0);
      double corrected_steering = std::clamp(current_steering + steering_offset, -1.0, 1.0);

      auto msgOut = qcar2_interfaces::msg::MotorCommands();
      msgOut.motor_names = {"steering_angle", "motor_throttle"};
      msgOut.values = {corrected_steering, corrected_throttle};
      command_publisher_->publish(msgOut);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Final zero command
    auto msgOut = qcar2_interfaces::msg::MotorCommands();
    msgOut.motor_names = {"steering_angle", "motor_throttle"};
    msgOut.values = {0.0, 0.0};
    command_publisher_->publish(msgOut);

    RCLCPP_INFO(this->get_logger(), "Shutdown complete.");
  }

private:
  double throttle;
  double steering_angle;
  double steering_offset;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
  rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr command_publisher_;
};

// Global pointer for signal handler access
std::shared_ptr<SimulinkOutSubscriber> g_node = nullptr;

void signal_handler(int /* signal */)
{
  if (g_node) {
    g_node->gradual_stop_and_shutdown();
  }
  rclcpp::shutdown();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  g_node = std::make_shared<SimulinkOutSubscriber>();

  std::signal(SIGINT, signal_handler);

  rclcpp::spin(g_node);
  g_node.reset();

  return 0;
}

