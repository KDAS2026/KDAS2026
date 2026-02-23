#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class SimTimePublisher : public rclcpp::Node
{
public:
    SimTimePublisher()
    : Node("sim_time_publisher"), start_publishing_(false)
    {
        // Publisher to /sim_time topic
        sim_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("sim_time", 10);

        // Subscriber to /dqn_done
        dqn_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "dqn_done",
            10,
            std::bind(&SimTimePublisher::dqnDoneCallback, this, std::placeholders::_1)
        );

        // Subscriber to /clock
        clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock",
            10,
            std::bind(&SimTimePublisher::clockCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "SimTimePublisher initialized. Waiting for dqn_done = true...");
    }

private:
    void dqnDoneCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !start_publishing_) {
            RCLCPP_INFO(this->get_logger(), "dqn_done = true. Publishing /sim_time now.");
            start_publishing_ = true;
        } else if (!msg->data && start_publishing_) {
            RCLCPP_INFO(this->get_logger(), "dqn_done = false. Halting /sim_time publishing.");
            start_publishing_ = false;
        }
    }

    void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        if (!start_publishing_) return;

        std_msgs::msg::Float64 sim_time_msg;
        sim_time_msg.data = rclcpp::Time(msg->clock).seconds();
        sim_time_pub_->publish(sim_time_msg);
    }

    bool start_publishing_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sim_time_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dqn_done_sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimTimePublisher>());
    rclcpp::shutdown();
    return 0;
}
