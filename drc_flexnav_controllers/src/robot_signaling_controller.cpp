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

#include "drc_flexnav_controllers/robot_signaling_controller.hpp"

using std::placeholders::_1;

namespace drc_flexnav_controllers {
RobotSignalingController::RobotSignalingController(void) {}

CallbackReturn RobotSignalingController::on_init(void) {
  try {
    gpio_name_ = auto_declare<std::string>("gpio_name", "siemens_et200sp_gpio");
    gpio_interfaces_ = auto_declare<std::vector<std::string>>(
        "gpio_interfaces", std::vector<std::string>{});
    gpio_state_interfaces_ = auto_declare<std::vector<std::string>>(
        "gpio_state_interfaces", std::vector<std::string>{});
    green_front_light_topic_ = auto_declare<std::string>(
        "green_front_light_topic", "green_front_light");
    green_rear_light_topic_ =
        auto_declare<std::string>("green_rear_light_topic", "green_rear_light");
    red_front_light_topic_ =
        auto_declare<std::string>("red_front_light_topic", "red_front_light");
    red_rear_light_topic_ =
        auto_declare<std::string>("red_rear_light_topic", "red_rear_light");
    left_light_topic_ =
        auto_declare<std::string>("left_light_topic", "left_light");
    right_light_topic_ =
        auto_declare<std::string>("right_light_topic", "right_light");
    buzzer_topic_ = auto_declare<std::string>("buzzer_topic", "buzzer");
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(
        get_node()->get_logger(),
        "Exception thrown in on_init() with message: " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSignalingController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  try {
    green_front_light_sub_ =
        get_node()->create_subscription<std_msgs::msg::Int32>(
            green_front_light_topic_, 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
              rt_green_front_light_.writeFromNonRT(msg);
            });
    green_rear_light_sub_ =
        get_node()->create_subscription<std_msgs::msg::Int32>(
            green_rear_light_topic_, 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
              rt_green_rear_light_.writeFromNonRT(msg);
            });
    red_front_light_sub_ =
        get_node()->create_subscription<std_msgs::msg::Int32>(
            red_front_light_topic_, 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
              rt_red_front_light_.writeFromNonRT(msg);
            });
    red_rear_light_sub_ = get_node()->create_subscription<std_msgs::msg::Int32>(
        red_rear_light_topic_, 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
          rt_red_rear_light_.writeFromNonRT(msg);
        });
    left_light_sub_ = get_node()->create_subscription<std_msgs::msg::Int32>(
        left_light_topic_, 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
          rt_left_light_.writeFromNonRT(msg);
        });
    right_light_sub_ = get_node()->create_subscription<std_msgs::msg::Int32>(
        right_light_topic_, 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
          rt_right_light_.writeFromNonRT(msg);
        });
    buzzer_sub_ = get_node()->create_subscription<std_msgs::msg::Int32>(
        right_light_topic_, 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
          rt_buzzer_.writeFromNonRT(msg);
        });
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Exception thrown in on_configure(): " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RobotSignalingController::command_interface_configuration(void) const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto &command_interface : gpio_interfaces_) {
    config.names.push_back(gpio_name_ + "/" + command_interface);
  }

  return config;
}

controller_interface::InterfaceConfiguration
RobotSignalingController::state_interface_configuration(void) const {
  controller_interface::InterfaceConfiguration config;

  for (const auto &state_interface : gpio_interfaces_) {
    config.names.push_back(gpio_name_ + "/" + state_interface);
  }
  for (const auto &state_interface : gpio_state_interfaces_) {
    config.names.push_back(gpio_name_ + "/" + state_interface);
  }

  return config;
}

CallbackReturn RobotSignalingController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}
CallbackReturn RobotSignalingController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
RobotSignalingController::update(const rclcpp::Time & /*time*/,
                                 const rclcpp::Duration & /*period*/) {

  auto buffer = rt_green_front_light_.readFromRT();
  if (buffer && *buffer) {
    set_command_if_exists((*buffer)->data,
                          gpio_name_ + "/" + "green_front_light",
                          command_interfaces_);
  }
  buffer = rt_green_rear_light_.readFromRT();
  if (buffer && *buffer) {
    set_command_if_exists((*buffer)->data,
                          gpio_name_ + "/" + "green_rear_light",
                          command_interfaces_);
  }
  buffer = rt_red_front_light_.readFromRT();
  if (buffer && *buffer) {
    set_command_if_exists((*buffer)->data, gpio_name_ + "/" + "red_front_light",
                          command_interfaces_);
  }
  buffer = rt_red_rear_light_.readFromRT();
  if (buffer && *buffer) {
    set_command_if_exists((*buffer)->data, gpio_name_ + "/" + "red_rear_light",
                          command_interfaces_);
  }
  buffer = rt_left_light_.readFromRT();
  if (buffer && *buffer) {
    set_command_if_exists((*buffer)->data, gpio_name_ + "/" + "left_light",
                          command_interfaces_);
  }
  buffer = rt_right_light_.readFromRT();
  if (buffer && *buffer) {
    set_command_if_exists((*buffer)->data, gpio_name_ + "/" + "right_light",
                          command_interfaces_);
  }
  buffer = rt_buzzer_.readFromRT();
  if (buffer && *buffer) {
    set_command_if_exists((*buffer)->data, gpio_name_ + "/" + "buzzer",
                          command_interfaces_);
  }
  return controller_interface::return_type::OK;
}

} // namespace drc_flexnav_controllers

PLUGINLIB_EXPORT_CLASS(drc_flexnav_controllers::RobotSignalingController,
                       controller_interface::ControllerInterface)
