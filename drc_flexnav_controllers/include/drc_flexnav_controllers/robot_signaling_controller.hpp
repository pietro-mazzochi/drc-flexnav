// Copyright (c) 2024, ISI SIM/DALCA
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef DRC_FLEXNAV_CONTROLLERS__ROBOT_SIGNALING_CONTROLLER_HPP_
#define DRC_FLEXNAV_CONTROLLERS__ROBOT_SIGNALING_CONTROLLER_HPP_

#include <std_msgs/msg/int32.hpp>

#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <vector>

#include "drc_flexnav_controllers/utils.hpp"
#include "drc_flexnav_controllers/visibility_control.h"

namespace drc_flexnav_controllers {
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RobotSignalingController
    : public controller_interface::ControllerInterface {
public:
  RobotSignalingController(void);

  DRC_FLEXNAV_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration
  command_interface_configuration(void) const override;

  DRC_FLEXNAV_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration
  state_interface_configuration(void) const override;

  DRC_FLEXNAV_CONTROLLERS__VISIBILITY_PUBLIC
  CallbackReturn on_init(void) override;

  DRC_FLEXNAV_CONTROLLERS__VISIBILITY_PUBLIC
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_CONTROLLERS__VISIBILITY_PUBLIC
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_CONTROLLERS__VISIBILITY_PUBLIC
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  std::string gpio_name_;
  std::vector<std::string> gpio_interfaces_, gpio_state_interfaces_;

  std::string green_front_light_topic_, green_rear_light_topic_,
      red_front_light_topic_, red_rear_light_topic_, left_light_topic_,
      right_light_topic_, buzzer_topic_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr green_front_light_sub_,
      green_rear_light_sub_, red_front_light_sub_, red_rear_light_sub_,
      left_light_sub_, right_light_sub_, buzzer_sub_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Int32>>
      rt_green_front_light_, rt_green_rear_light_, rt_red_front_light_,
      rt_red_rear_light_, rt_left_light_, rt_right_light_, rt_buzzer_;
};
} // namespace drc_flexnav_controllers

#endif // DRC_FLEXNAV_CONTROLLERS__ROBOT_SIGNALING_CONTROLLER_HPP_
