#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoystickController : public rclcpp::Node
{
public:
    JoystickController() : Node("joystick_controller")
    {
        // Declare and get the gain parameter
        this->declare_parameter<double>("gain", 1.0);
        this->declare_parameter<std::string>("vel_topic", "/base_vel_cmd");
        gain_ = this->get_parameter("gain").as_double();
        vel_topic_ = this->get_parameter("vel_topic").as_string();
        default_gain_ = gain_; // Store the default gain value

        // Subscribe to the joystick topic
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoystickController::joy_callback, this, std::placeholders::_1));

        // Publisher for the velocity command
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(vel_topic_, 10);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Check if the X button (usually index 0) is pressed
        if (msg->buttons[0] == 1)
        {
            gain_ = default_gain_ * 2; // Multiply gain by 2
        }
        // Check if the Y button (usually index 3) is pressed
        else if (msg->buttons[3] == 1)
        {
            gain_ = default_gain_ * 4; // Multiply gain by 4
        }
        else
        {
            gain_ = default_gain_; // Reset to the default gain
        }

        auto cmd_vel = geometry_msgs::msg::Twist();

        // Map joystick axes to robot velocity and apply gain
        cmd_vel.linear.x = gain_ * msg->axes[1];  // Forward/backward on left stick Y-axis
        cmd_vel.angular.z = gain_ * msg->axes[0]; // Left/right on left stick X-axis

        vel_pub_->publish(cmd_vel);
    }

    double gain_;
    std::string vel_topic_;
    double default_gain_; // Store the initial gain value
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
