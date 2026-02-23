#include "rclcpp/rclcpp.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"
#include "qcar2_interfaces/msg/boolean_leds.hpp"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <cmath>

class KeyboardController : public rclcpp::Node
{
public:
    KeyboardController() 
    : Node("keyboard_qcar_controller"),
      throttle(0.0),
      steering(0.0),
      reset_requested(false)
    {
        // Declare and read the steering offset parameter
	// if needed, can be modified via run parameter
        this->declare_parameter<double>("steering_offset", 0.0);
        this->get_parameter("steering_offset", steering_offset);

        command_publisher_ = this->create_publisher<qcar2_interfaces::msg::MotorCommands>("qcar2_motor_speed_cmd", 10);
        led_publisher_ = this->create_publisher<qcar2_interfaces::msg::BooleanLeds>("qcar2_led_cmd", 10);

        set_nonblocking_input();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&KeyboardController::control_loop, this));
    }

    ~KeyboardController() {
        reset_terminal();
    }

private:
    void set_nonblocking_input()
    {
        tcgetattr(STDIN_FILENO, &original_termios);
        struct termios new_termios = original_termios;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }

    void reset_terminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
    }

    void control_loop()
    {
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            switch (c)
            {
                case 'w': throttle += 0.1; reset_requested = false; break;
                case 's': throttle -= 0.1; reset_requested = false; break;
                case 'a': steering += 0.05; reset_requested = false; break;
                case 'd': steering -= 0.05; reset_requested = false; break;
                case ' ': reset_requested = true; break;
                case 'q': rclcpp::shutdown(); break;
                default: break;
            }
        }

        // Gradual reset if requested
        if (reset_requested) {
            const double decay_step_throttle = 0.05;
            const double decay_step_steering = 0.05;

            throttle = std::abs(throttle) > decay_step_throttle ?
                throttle - std::copysign(decay_step_throttle, throttle) : 0.0;

            steering = std::abs(steering) > decay_step_steering ?
                steering - std::copysign(decay_step_steering, steering) : 0.0;
        }

        // Clamp values
        throttle = std::clamp(throttle, -1.0, 1.0);
        steering = std::clamp(steering, -1.0, 1.0);

        // Apply hardware correction offset
        double corrected_steering = std::clamp(steering + steering_offset, -1.0, 1.0);

        // Publish motor command
        auto msg = qcar2_interfaces::msg::MotorCommands();
        msg.motor_names = {"steering_angle", "motor_throttle"};
        msg.values = {corrected_steering, throttle};
        command_publisher_->publish(msg);

        // Dummy LED publish
        auto leds = qcar2_interfaces::msg::BooleanLeds();
        leds.led_names = {"left_outside_headlight", "right_outside_headlight"};
        leds.values = {false, false};
        led_publisher_->publish(leds);

        // Console output
        std::cout << "\rThrottle: " << throttle << " | Steering (logical): " << steering 
                  << " | Sent (corrected): " << corrected_steering << "        " << std::flush;
    }

    rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr command_publisher_;
    rclcpp::Publisher<qcar2_interfaces::msg::BooleanLeds>::SharedPtr led_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double throttle;
    double steering;
    double steering_offset;
    bool reset_requested;
    struct termios original_termios;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardController>());
    rclcpp::shutdown();
    return 0;
}

