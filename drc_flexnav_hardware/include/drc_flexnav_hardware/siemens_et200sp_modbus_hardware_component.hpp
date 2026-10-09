#ifndef DRC_FLEXNAV_HARDWARE__SIEMENS_ET200SP_MODBUS_HARDWARE_COMPONENT_HPP_
#define DRC_FLEXNAV_HARDWARE__SIEMENS_ET200SP_MODBUS_HARDWARE_COMPONENT_HPP_

#include <chrono>
#include <iostream>
#include <modbus/modbus.h>
#include <unordered_map>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "drc_flexnav_hardware/visbility_control.h"
#include "hardware_interface/system_interface.hpp"

#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rclcpp/subscription.hpp>
#include "drc_flexnav_controllers_msgs/msg/plc_command.hpp"
#include "drc_flexnav_controllers_msgs/msg/plc_mission_response.hpp"
#include "drc_flexnav_controllers_msgs/msg/plc_request.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "siemens_et200sp_modbus_driver.hpp"
#include <rclcpp/rclcpp.hpp>

#include <odometry_arc/diff_odometry.hpp>

namespace drc_flexnav_hardware {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

struct Vector3 {
  double x;
  double y;
  double z;
};
struct Quatertion {
  double x;
  double y;
  double z;
  double w;
};

struct Odom {
  struct Pose {
    Vector3 position;
    Quatertion orientation;
  } pose;
  struct Twist {
    Vector3 linear;
    Vector3 angular;
  } twist;
  double &operator[](std::string member) {
    if (member == "pose.position.x")
      return pose.position.x;
    else if (member == "pose.position.y")
      return pose.position.y;
    else if (member == "pose.position.z")
      return pose.position.z;
    else if (member == "pose.orientation.x")
      return pose.orientation.x;
    else if (member == "pose.orientation.y")
      return pose.orientation.y;
    else if (member == "pose.orientation.z")
      return pose.orientation.z;
    else if (member == "pose.orientation.w")
      return pose.orientation.w;
    else if (member == "twist.linear.x")
      return twist.linear.x;
    else if (member == "twist.linear.y")
      return twist.linear.y;
    else if (member == "twist.linear.z")
      return twist.linear.z;
    else if (member == "twist.angular.x")
      return twist.angular.x;
    else if (member == "twist.angular.y")
      return twist.angular.y;
    else if (member == "twist.angular.z")
      return twist.angular.z;
    throw std::invalid_argument(member + "is not a member of struct Odom");
  }
  const double &operator[](std::string member) const {
    if (member == "pose.position.x")
      return pose.position.x;
    else if (member == "pose.position.y")
      return pose.position.y;
    else if (member == "pose.position.z")
      return pose.position.z;
    else if (member == "pose.orientation.x")
      return pose.orientation.x;
    else if (member == "pose.orientation.y")
      return pose.orientation.y;
    else if (member == "pose.orientation.z")
      return pose.orientation.z;
    else if (member == "pose.orientation.w")
      return pose.orientation.w;
    else if (member == "twist.linear.x")
      return twist.linear.x;
    else if (member == "twist.linear.y")
      return twist.linear.y;
    else if (member == "twist.linear.z")
      return twist.linear.z;
    else if (member == "twist.angular.x")
      return twist.angular.x;
    else if (member == "twist.angular.y")
      return twist.angular.y;
    else if (member == "twist.angular.z")
      return twist.angular.z;
    throw std::invalid_argument(member + "is not a member of struct Odom");
  }
};

class Siemens_ET200SP_Modbus_HW_Component
    : public hardware_interface::SystemInterface {
public:
  Siemens_ET200SP_Modbus_HW_Component();

  DRC_FLEXNAV_HARDWARE_PUBLIC CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  /*DRC_FLEXNAV_HARDWARE_PUBLIC CallbackReturn*/
  /*on_error(const rclcpp_lifecycle::State &previous_state) override;*/

  DRC_FLEXNAV_HARDWARE_PUBLIC CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  DRC_FLEXNAV_HARDWARE_PUBLIC std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  // NOTE: Optional methods:
  //
  /* hDRC_FLEXNAV_HARDWARE_PUBLIC ardware_interface::return_type
   * prepare_command_mode_switch( */
  /*     const std::vector<std::string> &start_interfaces, */
  /*     const std::vector<std::string> &stop_interfaces) override; */
  /**/
  /* hDRC_FLEXNAV_HARDWARE_PUBLIC ardware_interface::return_type
   * perform_command_mode_switch( */
  /*     const std::vector<std::string> &start_interfaces, */
  /*     const std::vector<std::string> &stop_interfaces) override; */

  DRC_FLEXNAV_HARDWARE_PUBLIC hardware_interface::return_type
  read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC void
  reset_state_interfaces(const std::string &joint);

  DRC_FLEXNAV_HARDWARE_PUBLIC void
  reset_command_interfaces(const std::string &joint);

  DRC_FLEXNAV_HARDWARE_PUBLIC void
  reset_gpio_interfaces(const std::string &gpio_interface);

  DRC_FLEXNAV_HARDWARE_PUBLIC hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  Siemens_ET200SP_Modbus_Driver siemens_et200sp;
  int connection_count_ = 0;
  bool is_connected_flag_ = false;

  std::unordered_map<std::string, double> command_interface_position,
      command_interface_velocity, gpio_command_interface;

  std::unordered_map<std::string, double> state_interface_position,
      state_interface_velocity, gpio_state_interface;

  std::string plc_ip_;
  int modbus_communication_port_;
  std::string output_topic_;
  std::string input_prefix_;
  int frequency_;
  std::chrono::milliseconds plc_connection_timeout_, plc_connection_retry_;

  std::string left_wheel_joint_name_, right_wheel_joint_name_;
  int encoder_max_counts_;
  double gear_ratio_;
  bool export_position_interface;
  std::string gpio_name_;
  std::vector<std::string> gpio_interfaces_, gpio_state_interfaces_;
  int64_t last_left_encoder{0}, last_right_encoder{0};
  int64_t last_timestamp{0};

  std::string odometer_name_;
  Odom odometry_values_;
  odometry_arc::DiffOdometry odom_;
  double wheel_to_robot_center_{0}, wheel_radius_{0};
  // phi[0] is the left wheel angular displacement
  // phi[1] is the right wheel angular displacement
  Eigen::Vector2d phi_, deltaPhi_, phiDot_;
  bool use_joint_vel_{true};

  // Missions variables
  rclcpp::Subscription<drc_flexnav_controllers_msgs::msg::PlcCommand>::SharedPtr command_subscription_;
  rclcpp::Subscription<drc_flexnav_controllers_msgs::msg::PlcMissionResponse>::SharedPtr missions_subscription_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<drc_flexnav_controllers_msgs::msg::PlcCommand>> plc_command_response_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<drc_flexnav_controllers_msgs::msg::PlcMissionResponse>> plc_missions_response_;
  bool plc_command_response_flag_ = false;
  bool plc_missions_response_flag_ = false;

  // Node on thread
  rclcpp::Node::SharedPtr hw_node_;
  std::thread hw_node_spin_thread_;

  // Publisher to plc_request
  std::shared_ptr<rclcpp::Publisher<drc_flexnav_controllers_msgs::msg::PlcRequest>> plc_request_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<drc_flexnav_controllers_msgs::msg::PlcRequest>> rt_plc_request_pub_;
};
} // namespace drc_flexnav_hardware

#endif // DRC_FLEXNAV_HARDWARE__SIEMENS_ET200SP_MODBUS_HARDWARE_COMPONENT_HPP_
