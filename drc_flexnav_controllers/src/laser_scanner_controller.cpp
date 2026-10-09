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

#include "drc_flexnav_controllers/laser_scanner_controller.hpp"

using std::placeholders::_1;

namespace drc_flexnav_controllers {
LaserScannerController::LaserScannerController(void) {}

CallbackReturn LaserScannerController::on_init(void) {
  try {
    gpio_name_ = auto_declare<std::string>("gpio_name", "siemens_et200sp_gpio");
    gpio_interfaces_ = auto_declare<std::vector<std::string>>(
        "gpio_interfaces", std::vector<std::string>{});
    gpio_state_interfaces_ = auto_declare<std::vector<std::string>>(
        "gpio_state_interfaces", std::vector<std::string>{});
    ls_config_topic_ =
        auto_declare<std::string>("ls_config_topic", "ls_config");
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(
        get_node()->get_logger(),
        "Exception thrown in on_init() with message: " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn LaserScannerController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  try {
    ls_config_sub_ = get_node()->create_subscription<std_msgs::msg::Int32>(
        ls_config_topic_, 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
          rt_ls_config_.writeFromNonRT(msg);
        });
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Exception thrown in on_configure(): " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
LaserScannerController::command_interface_configuration(void) const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto &command_interface : gpio_interfaces_) {
    config.names.push_back(gpio_name_ + "/" + command_interface);
  }

  return config;
}

controller_interface::InterfaceConfiguration
LaserScannerController::state_interface_configuration(void) const {
  controller_interface::InterfaceConfiguration config;

  for (const auto &state_interface : gpio_interfaces_) {
    config.names.push_back(gpio_name_ + "/" + state_interface);
  }
  for (const auto &state_interface : gpio_state_interfaces_) {
    config.names.push_back(gpio_name_ + "/" + state_interface);
  }

  return config;
}

CallbackReturn LaserScannerController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}
CallbackReturn LaserScannerController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
LaserScannerController::update(const rclcpp::Time & /*time*/,
                               const rclcpp::Duration & /*period*/) {
  auto buffer = rt_ls_config_.readFromRT();
  if (buffer && *buffer) {
    set_command_if_exists((*buffer)->data, gpio_name_ + "/" + "ls_config",
                          command_interfaces_);
  }
  return controller_interface::return_type::OK;
}

} // namespace drc_flexnav_controllers

PLUGINLIB_EXPORT_CLASS(drc_flexnav_controllers::LaserScannerController,
                       controller_interface::ControllerInterface)
