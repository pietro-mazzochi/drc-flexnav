// Copyright (c) 2024, ISI SIM/DALCA
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef DRC_FLEXNAV_CONTROLLERS__ODOMETRY_BROADCASTER_HPP_
#define DRC_FLEXNAV_CONTROLLERS__ODOMETRY_BROADCASTER_HPP_

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <vector>

#include "drc_flexnav_controllers/utils.hpp"
#include "drc_flexnav_controllers/visibility_control.h"

namespace drc_flexnav_controllers {
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class OdometryBroadcaster : public controller_interface::ControllerInterface {
public:
  OdometryBroadcaster(void);

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
  std::string sensor_name_;
  std::vector<std::string> sensor_interfaces_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
      rt_odom_pub_;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> tf_odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
      rt_tf_odom_pub_;

  std::string odom_topic_, tf_topic_;
  std::string odom_frame_id_, base_frame_id_;
  bool publish_odom_to_tf_{true};

  std::string gpio_name_;
  std::vector<std::string> gpio_state_interfaces_;

  bool use_siemens_time_stamp_;
};
} // namespace drc_flexnav_controllers

#endif // DRC_FLEXNAV_CONTROLLERS__ODOMETRY_BROADCASTER_HPP_
