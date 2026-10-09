// Copyright (c) 2024, ISI SIM/DALCA
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef DRC_FLEXNAV_CONTROLLERS__SIEMENS_ET200SP_BROADCASTER_HPP_
#define DRC_FLEXNAV_CONTROLLERS__SIEMENS_ET200SP_BROADCASTER_HPP_

#include <std_msgs/msg/float64_multi_array.hpp>

#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>

#include "drc_flexnav_controllers/utils.hpp"
#include "drc_flexnav_controllers/visibility_control.h"

#include "siemens_et200sp_modbus_driver_msgs/msg/siemens_etsp_modbus_driver.hpp"

namespace drc_flexnav_controllers {
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Siemens_ET200SP_Broadcaster
    : public controller_interface::ControllerInterface {
public:
  Siemens_ET200SP_Broadcaster(void);

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

  std::shared_ptr<rclcpp::Publisher<
      siemens_et200sp_modbus_driver_msgs::msg::SiemensEtspModbusDriver>>
      siemens_et200sp_data_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<
      siemens_et200sp_modbus_driver_msgs::msg::SiemensEtspModbusDriver>>
      rt_siemens_et200sp_data_pub_;
  std::string siemens_et200sp_output_topic_;

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>
      is_emergency_lock_data_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>
      rt_is_emergency_lock_data_pub_;
  std::string is_emergency_lock_output_topic_;
};
} // namespace drc_flexnav_controllers

#endif // DRC_FLEXNAV_CONTROLLERS__SIEMENS_ET200SP_BROADCASTER_HPP_
