#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include "drc_flexnav_controllers_msgs/msg/dynamics_controller_status.hpp"
#include "drc_flexnav_controllers_msgs/msg/mrac_controller_status.hpp"
#include "drc_flexnav_pose_estimation/extended_kalman_filter.hpp"

class DynamicsControllerStatusToAccelStamped : public rclcpp::Node {
public:
  DynamicsControllerStatusToAccelStamped()
      : Node("dynamics_controller_status_to_accel_stamped") {
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Using clock type: " << this->get_clock()->get_clock_type());
    status_topic_name_ = declare_parameter<std::string>(
        "status_topic_name", "/dynamics_controller/status");
    controller_type_ = declare_parameter<std::string>(
        "controller_type", "dynamics_controller");
    accel_stamped_output_topic_name_ =
        declare_parameter<std::string>("accel_stamped_output_topic_name", "/mu");

    if (controller_type_ == "dynamics_controller"){
      dynamics_controller_status_sub_ = this->create_subscription<drc_flexnav_controllers_msgs::msg::DynamicsControllerStatus>(
        status_topic_name_, 10, std::bind(&DynamicsControllerStatusToAccelStamped::dynamics_controller_status_callback, this, std::placeholders::_1));

    } else if (controller_type_ == "mrac_controller"){
      mrac_controller_status_sub_ = this->create_subscription<drc_flexnav_controllers_msgs::msg::MRACControllerStatus>(
        status_topic_name_, 10, std::bind(&DynamicsControllerStatusToAccelStamped::mrac_controller_status_callback, this, std::placeholders::_1));
    }

    accel_stamped_pub_ = this->create_publisher<geometry_msgs::msg::AccelStamped>(accel_stamped_output_topic_name_, 10);
  }

private:
    void dynamics_controller_status_callback(const drc_flexnav_controllers_msgs::msg::DynamicsControllerStatus::SharedPtr msg) {
        geometry_msgs::msg::AccelStamped accel_s;
        accel_s.header = msg->header;
        accel_s.accel = msg->accel_set_point;

        accel_stamped_pub_->publish(accel_s);
    }
    void mrac_controller_status_callback(const drc_flexnav_controllers_msgs::msg::MRACControllerStatus::SharedPtr msg) {
        geometry_msgs::msg::AccelStamped accel_s;
        accel_s.header = msg->header;
        accel_s.accel = msg->accel_set_point;

        accel_stamped_pub_->publish(accel_s);
    }

    rclcpp::Subscription<drc_flexnav_controllers_msgs::msg::DynamicsControllerStatus>::SharedPtr dynamics_controller_status_sub_;
    rclcpp::Subscription<drc_flexnav_controllers_msgs::msg::MRACControllerStatus>::SharedPtr mrac_controller_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_stamped_pub_;
    std::string status_topic_name_, accel_stamped_output_topic_name_, controller_type_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicsControllerStatusToAccelStamped>());
  rclcpp::shutdown();
  return 0;
}
