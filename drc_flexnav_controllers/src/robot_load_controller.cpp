// Copyright (c) 2024, ISI SIM/DALCA
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

#include "drc_flexnav_controllers/robot_load_controller.hpp"

using std::placeholders::_1;

namespace drc_flexnav_controllers {
RobotLoadController::RobotLoadController(void) {}

CallbackReturn RobotLoadController::on_init(void) {
  try {
    gpio_name_ = auto_declare<std::string>("gpio_name", "siemens_et200sp_gpio");
    gpio_interfaces_ = auto_declare<std::vector<std::string>>(
        "gpio_interfaces", std::vector<std::string>{});
    gpio_state_interfaces_ = auto_declare<std::vector<std::string>>(
        "gpio_state_interfaces", std::vector<std::string>{});
    load_unload_timeout_seconds_topic_ = auto_declare<std::string>(
        "load_unload_timeout_seconds_topic", "load_unload_timeout_seconds");
    load_command_topic_ =
        auto_declare<std::string>("load_command_topic", "load_command");
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(
        get_node()->get_logger(),
        "Exception thrown in on_init() with message: " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotLoadController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  try {
    load_unload_timeout_seconds_sub_ =
        get_node()->create_subscription<std_msgs::msg::Int32>(
            load_unload_timeout_seconds_topic_, 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
              rt_load_unload_timeout_seconds_.writeFromNonRT(msg);
            });
    load_command_sub_ = get_node()->create_subscription<std_msgs::msg::Int32>(
        load_command_topic_, 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
          rt_load_command_.writeFromNonRT(msg);
        });
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Exception thrown in on_configure(): " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RobotLoadController::command_interface_configuration(void) const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto &command_interface : gpio_interfaces_) {
    config.names.push_back(gpio_name_ + "/" + command_interface);
  }

  return config;
}

controller_interface::InterfaceConfiguration
RobotLoadController::state_interface_configuration(void) const {
  controller_interface::InterfaceConfiguration config;

  for (const auto &state_interface : gpio_interfaces_) {
    config.names.push_back(gpio_name_ + "/" + state_interface);
  }
  for (const auto &state_interface : gpio_state_interfaces_) {
    config.names.push_back(gpio_name_ + "/" + state_interface);
  }

  return config;
}

CallbackReturn RobotLoadController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}
CallbackReturn RobotLoadController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
RobotLoadController::update(const rclcpp::Time & /*time*/,
                            const rclcpp::Duration & /*period*/) {
  auto buffer = rt_load_unload_timeout_seconds_.readFromRT();
  if (buffer && *buffer) {
    set_command_if_exists((*buffer)->data,
                          gpio_name_ + "/" + "load_unload_timeout_seconds",
                          command_interfaces_);
  }
  buffer = rt_load_command_.readFromRT();
  if (buffer && *buffer) {
    set_command_if_exists((*buffer)->data, gpio_name_ + "/" + "load_command",
                          command_interfaces_);
  }
  return controller_interface::return_type::OK;
}

} // namespace drc_flexnav_controllers

PLUGINLIB_EXPORT_CLASS(drc_flexnav_controllers::RobotLoadController,
                       controller_interface::ControllerInterface)
