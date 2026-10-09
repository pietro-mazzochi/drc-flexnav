#ifndef DRC_FLEXNAV_HARDWARE__CURTIS_ACF2A_CAN_HARDWARE_COMPONENT_HPP_
#define DRC_FLEXNAV_HARDWARE__CURTIS_ACF2A_CAN_HARDWARE_COMPONENT_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"

#include "drc_flexnav_hardware/visbility_control.h"
#include "hardware_interface/system_interface.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "curtis_acf2a_can_driver/canopen_interface.hpp"
#include "curtis_acf2a_can_driver/curtis_acf2a_can_driver_lib.hpp"

namespace drc_flexnav_hardware {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Curtis_ACF2A_Can_HW_Component
    : public hardware_interface::SystemInterface {
public:
  Curtis_ACF2A_Can_HW_Component();

  DRC_FLEXNAV_HARDWARE_PUBLIC CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC void start_resume_cfg_thread();

  DRC_FLEXNAV_HARDWARE_PUBLIC void pause_cfg_thread();

  DRC_FLEXNAV_HARDWARE_PUBLIC void stop_cfg_thread();

  DRC_FLEXNAV_HARDWARE_PUBLIC void config_motors();

  DRC_FLEXNAV_HARDWARE_PUBLIC CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  /* DRC_FLEXNAV_HARDWARE_PUBLIC CallbackReturn */
  /* on_error(const rclcpp_lifecycle::State &previous_state) override; */

  DRC_FLEXNAV_HARDWARE_PUBLIC std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  DRC_FLEXNAV_HARDWARE_PUBLIC std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  // NOTE: Optional methods:
  //
  /* DRC_FLEXNAV_HARDWARE_PUBLIC hardware_interface::return_type
   * prepare_command_mode_switch( */
  /*     const std::vector<std::string> &start_interfaces, */
  /*     const std::vector<std::string> &stop_interfaces) override; */
  /**/
  /* DRC_FLEXNAV_HARDWARE_PUBLIC hardware_interface::return_type
   * perform_command_mode_switch( */
  /*     const std::vector<std::string> &start_interfaces, */
  /*     const std::vector<std::string> &stop_interfaces) override; */

  DRC_FLEXNAV_HARDWARE_PUBLIC hardware_interface::return_type
  read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC void
  reset_state_interfaces(const std::string &joint);

  DRC_FLEXNAV_HARDWARE_PUBLIC void
  reset_command_interfaces(const std::string &joint);

  DRC_FLEXNAV_HARDWARE_PUBLIC hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  std::unordered_map<std::string, double> command_interface_position,
      command_interface_velocity, command_interface_effort;

  std::unordered_map<std::string, double> state_interface_position,
      state_interface_velocity, state_interface_effort, state_interface_nmt,
      state_interface_motor_configured;

  CANOPEN_Interface canopen_left;
  CANOPEN_Interface canopen_right;

  Curtis_ACF2A_Can_Driver left_motor;
  Curtis_ACF2A_Can_Driver right_motor;

  bool left_motor_configured = false;
  bool right_motor_configured = false;
  bool motors_configured_flag = false;

  std::string left_wheel_joint_name_, right_wheel_joint_name_;
  std::string can_left_port_, can_right_port_;
  int left_motor_id_, right_motor_id_, left_motor_swap_dir_,
      right_motor_swap_dir_;
  int left_motor_control_mode_, right_motor_control_mode_;
  int64_t encoder_max_counts_;
  double gear_ratio_;
  bool export_position_interface = true;
  bool use_gear_ratio_ = false;

  float left_motor_torque_ref_ = 0;
  float right_motor_torque_ref_ = 0;
};
} // namespace drc_flexnav_hardware

#endif // DRC_FLEXNAV_HARDWARE__CURTIS_ACF2A_CAN_HARDWARE_COMPONENT_HPP_
