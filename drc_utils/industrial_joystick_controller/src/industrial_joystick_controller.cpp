#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "siemens_et200sp_modbus_driver_msgs/msg/siemens_etsp_modbus_driver.hpp"

#include <chrono>
#include <iostream>

class IndustrialJoystickController : public rclcpp::Node
{
public:
    IndustrialJoystickController() : Node("industrial_joystick_controller")
    {
        // Declare parameters
        this->declare_parameter<double>("max_linear_speed", 0.2);
        this->declare_parameter<double>("max_angular_speed", 0.3);
        this->declare_parameter<double>("soft_start_time", 2.0);
        this->declare_parameter<std::string>("vel_topic", "/base_vel_cmd");
        this->declare_parameter<std::string>("siemens_et200sp_output_topic", "/siemens_et200sp");

        // Read parameters from launch file
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        soft_start_time_ = this->get_parameter("soft_start_time").as_double();
        vel_topic_ = this->get_parameter("vel_topic").as_string();

        // Calculate rates for soft start of speed commands
        linear_speed_rate_ = max_linear_speed_ / soft_start_time_;
        angular_speed_rate_ = max_angular_speed_ / soft_start_time_;

        // Declare and get the ET200SP output topic parameter
        siemens_et200sp_output_topic_ = this->get_parameter("siemens_et200sp_output_topic").as_string();

        // Subscribe to ET200SP output topic
        siemens_et200sp_data_sub_ = this->create_subscription<siemens_et200sp_modbus_driver_msgs::msg::SiemensEtspModbusDriver>(
                                                                siemens_et200sp_output_topic_,
                                                                rclcpp::SystemDefaultsQoS(),
                                                                std::bind(&IndustrialJoystickController::siemens_et200sp_callback, this, std::placeholders::_1));

        // Publisher for the velocity command
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(vel_topic_, 10);

        // Init time variables
        last_start_callback_time_ = std::chrono::high_resolution_clock::now();
        start_callback_time_ = std::chrono::high_resolution_clock::now();
    }

private:
    void siemens_et200sp_callback(const siemens_et200sp_modbus_driver_msgs::msg::SiemensEtspModbusDriver::SharedPtr msg)
    {
        // Save timestamp of the last callback to calculate elapsed
        last_start_callback_time_ = start_callback_time_;
        start_callback_time_ = std::chrono::high_resolution_clock::now();

        // Calculate the time elapsed since last callback
        elapsed_time_ = std::chrono::duration<double>(start_callback_time_ - last_start_callback_time_).count();
        if(elapsed_time_ > 1.0)     elapsed_time_ = 1.0;

        // Verify if PLC is connected
        if(msg->plc_connected){
            // Verify if operation mode is manual
            if(msg->man_sw){
                // Verify if operation mode is not automatic
                if(!msg->auto_sw){
                    // Verify jostick connection
                    if(msg->joy_connected){
                        // Correct operation mode and joystick connection are valid

                        // Verify joystick up/down button is pressed
                        if(msg->joy_up > 0){
                            if(msg->joy_down == 0){
                                if(linear_speed_ < 0)       linear_speed_ = 0.0;
                                else                        linear_speed_ += (linear_speed_rate_ * elapsed_time_);
                            }
                            else{
                                linear_speed_ = 0.0;
                            }
                        }
                        else if(msg->joy_down > 0){
                            if(linear_speed_ > 0)           linear_speed_ = 0.0;
                            else                            linear_speed_ -= (linear_speed_rate_ * elapsed_time_);
                        }
                        else{
                            linear_speed_ = 0.0;
                        }

                        // Verify joystick left/right button is pressed
                        if(msg->joy_right > 0){
                            if(msg->joy_left == 0){
                                if(angular_speed_ > 0)      angular_speed_ = 0.0;
                                else                        angular_speed_ -= (angular_speed_rate_ * elapsed_time_);
                            }
                            else{
                                angular_speed_ = 0.0;
                            }
                        }
                        else if(msg->joy_left > 0){
                            if(angular_speed_ < 0)          angular_speed_ = 0.0;
                            else                            angular_speed_ += (angular_speed_rate_ * elapsed_time_);
                        }
                        else{
                            angular_speed_ = 0.0;
                        }

                        // Limit the maximum speed of robot base
                        if(linear_speed_ > max_linear_speed_)           linear_speed_ = max_linear_speed_;
                        else if(linear_speed_ < -max_linear_speed_)     linear_speed_ = -max_linear_speed_;
                        if(angular_speed_ > max_angular_speed_)         angular_speed_ = max_angular_speed_;
                        else if(angular_speed_ < -max_angular_speed_)   angular_speed_ = -max_angular_speed_;

                        // Publish velocity command to robot base
                        auto cmd_vel = geometry_msgs::msg::Twist();
                        cmd_vel.linear.x = linear_speed_;
                        cmd_vel.angular.z = angular_speed_;
                                
                        vel_pub_->publish(cmd_vel);
                    }
                    else{
                        RCLCPP_ERROR(this->get_logger(), "Industrial joystick not connected in manual mode!");
                    }
                }
                else{
                    RCLCPP_ERROR(this->get_logger(), "Automatic and manual modes configured at the same time!");
                }
            }    
        }
    }

    // Base speed variables and rates
    double max_linear_speed_;
    double linear_speed_;
    double linear_speed_rate_;
    double max_angular_speed_;
    double angular_speed_;
    double angular_speed_rate_;
    double soft_start_time_;

    // Time variables for soft start and elapsed time calculation
    std::chrono::high_resolution_clock::time_point start_callback_time_;
    std::chrono::high_resolution_clock::time_point last_start_callback_time_;
    double elapsed_time_;

    std::string vel_topic_;
    std::string siemens_et200sp_output_topic_;
    rclcpp::Subscription<siemens_et200sp_modbus_driver_msgs::msg::SiemensEtspModbusDriver>::SharedPtr siemens_et200sp_data_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IndustrialJoystickController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
