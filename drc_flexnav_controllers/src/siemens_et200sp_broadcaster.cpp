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

#include "drc_flexnav_controllers/siemens_et200sp_broadcaster.hpp"

using std::placeholders::_1;

namespace drc_flexnav_controllers {
Siemens_ET200SP_Broadcaster::Siemens_ET200SP_Broadcaster(void) {}

CallbackReturn Siemens_ET200SP_Broadcaster::on_init(void) {
  try {
    gpio_name_ = auto_declare<std::string>("gpio_name", "siemens_et200sp_gpio");
    gpio_interfaces_ = auto_declare<std::vector<std::string>>(
        "gpio_interfaces", std::vector<std::string>{});
    gpio_state_interfaces_ = auto_declare<std::vector<std::string>>(
        "gpio_state_interfaces", std::vector<std::string>{});
    siemens_et200sp_output_topic_ = auto_declare<std::string>(
        "siemens_et200sp_output_topic", "siemens_et200sp");

    is_emergency_lock_output_topic_ = auto_declare<std::string>(
        "is_emergency_lock_output_topic", "/is_emergency_lock");
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(
        get_node()->get_logger(),
        "Exception thrown in on_init() with message: " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn Siemens_ET200SP_Broadcaster::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  try {

    siemens_et200sp_data_pub_ =
        get_node()
            ->create_publisher<siemens_et200sp_modbus_driver_msgs::msg::
                                   SiemensEtspModbusDriver>(
                siemens_et200sp_output_topic_, rclcpp::SystemDefaultsQoS());
    rt_siemens_et200sp_data_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<
            siemens_et200sp_modbus_driver_msgs::msg::SiemensEtspModbusDriver>>(
            siemens_et200sp_data_pub_);

    is_emergency_lock_data_pub_ =
        get_node()->create_publisher<std_msgs::msg::Bool>(
            is_emergency_lock_output_topic_, rclcpp::SystemDefaultsQoS());
    rt_is_emergency_lock_data_pub_ = std::make_shared<
        realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
        is_emergency_lock_data_pub_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Exception thrown in on_configure(): " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Siemens_ET200SP_Broadcaster::command_interface_configuration(void) const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  return config;
}

controller_interface::InterfaceConfiguration
Siemens_ET200SP_Broadcaster::state_interface_configuration(void) const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::ALL;

  for (const auto &state_interface : gpio_interfaces_) {
    config.names.push_back(gpio_name_ + "/" + state_interface);
  }
  for (const auto &state_interface : gpio_state_interfaces_) {
    config.names.push_back(gpio_name_ + "/" + state_interface);
  }

  return config;
}

CallbackReturn Siemens_ET200SP_Broadcaster::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}
CallbackReturn Siemens_ET200SP_Broadcaster::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
Siemens_ET200SP_Broadcaster::update(const rclcpp::Time &time,
                                    const rclcpp::Duration & /*period*/) {
  if (rt_siemens_et200sp_data_pub_ && rt_siemens_et200sp_data_pub_->trylock()) {

    rt_siemens_et200sp_data_pub_->msg_.header.stamp = time;
    rt_siemens_et200sp_data_pub_->msg_.time_stamp =
        return_state_if_exists(rt_siemens_et200sp_data_pub_->msg_.time_stamp,
                               gpio_name_ + "/time_stamp", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.delta_t =
        return_state_if_exists(rt_siemens_et200sp_data_pub_->msg_.delta_t,
                               gpio_name_ + "/delta_t", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.left_encoder =
        return_state_if_exists(rt_siemens_et200sp_data_pub_->msg_.left_encoder,
                               gpio_name_ + "/left_encoder", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.right_encoder = return_state_if_exists(
        rt_siemens_et200sp_data_pub_->msg_.right_encoder,
        gpio_name_ + "/right_encoder", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.left_encoder_rad =
        return_state_if_exists(
            rt_siemens_et200sp_data_pub_->msg_.left_encoder_rad,
            gpio_name_ + "/left_encoder_rad", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.right_encoder_rad =
        return_state_if_exists(
            rt_siemens_et200sp_data_pub_->msg_.right_encoder_rad,
            gpio_name_ + "/right_encoder_rad", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.left_encoder_rpm =
        return_state_if_exists(
            rt_siemens_et200sp_data_pub_->msg_.left_encoder_rpm,
            gpio_name_ + "/left_encoder_rpm", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.right_encoder_rpm =
        return_state_if_exists(
            rt_siemens_et200sp_data_pub_->msg_.right_encoder_rpm,
            gpio_name_ + "/right_encoder_rpm", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.joy_up =
        return_state_if_exists(rt_siemens_et200sp_data_pub_->msg_.joy_up,
                               gpio_name_ + "/joy_up", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.joy_down =
        return_state_if_exists(rt_siemens_et200sp_data_pub_->msg_.joy_down,
                               gpio_name_ + "/joy_down", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.joy_left =
        return_state_if_exists(rt_siemens_et200sp_data_pub_->msg_.joy_left,
                               gpio_name_ + "/joy_left", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.joy_right =
        return_state_if_exists(rt_siemens_et200sp_data_pub_->msg_.joy_right,
                               gpio_name_ + "/joy_right", state_interfaces_);
    /* rt_siemens_et200sp_data_pub_->msg_.docked_front = */
    /*     return_state_if_exists( */
    /*         rt_siemens_et200sp_data_pub_->msg_.docked_front, gpio_name_ + "/"
     * + "docked_front",
     */
    /*         state_interfaces_); */
    /* rt_siemens_et200sp_data_pub_->msg_.docked_rear = */
    /*     return_state_if_exists( */
    /*         rt_siemens_et200sp_data_pub_->msg_.docked_rear, gpio_name_ + "/"
     * + "docked_rear", */
    /*         state_interfaces_); */
    rt_siemens_et200sp_data_pub_->msg_.charging = return_state_if_exists(
        rt_siemens_et200sp_data_pub_->msg_.charging, gpio_name_ + "/charging",
        state_interfaces_, false);
    rt_siemens_et200sp_data_pub_->msg_.front_ls_violation =
        return_state_if_exists(
            rt_siemens_et200sp_data_pub_->msg_.front_ls_violation,
            gpio_name_ + "/front_ls_violation", state_interfaces_, true);
    rt_siemens_et200sp_data_pub_->msg_.rear_ls_violation =
        return_state_if_exists(
            rt_siemens_et200sp_data_pub_->msg_.rear_ls_violation,
            gpio_name_ + "/rear_ls_violation", state_interfaces_, true);
    rt_siemens_et200sp_data_pub_->msg_.reset_wait = return_state_if_exists(
        rt_siemens_et200sp_data_pub_->msg_.reset_wait,
        gpio_name_ + "/reset_wait", state_interfaces_, true);
    rt_siemens_et200sp_data_pub_->msg_.undefined_error = return_state_if_exists(
        rt_siemens_et200sp_data_pub_->msg_.undefined_error,
        gpio_name_ + "/undefined_error", state_interfaces_, true);
    rt_siemens_et200sp_data_pub_->msg_.restart_detected =
        return_state_if_exists(
            rt_siemens_et200sp_data_pub_->msg_.restart_detected,
            gpio_name_ + "/restart_detected", state_interfaces_, true);
    rt_siemens_et200sp_data_pub_->msg_.left_motor_error =
        return_state_if_exists(
            rt_siemens_et200sp_data_pub_->msg_.left_motor_error,
            gpio_name_ + "/left_motor_error", state_interfaces_, true);
    rt_siemens_et200sp_data_pub_->msg_.right_motor_error =
        return_state_if_exists(
            rt_siemens_et200sp_data_pub_->msg_.right_motor_error,
            gpio_name_ + "/right_motor_error", state_interfaces_, true);
    rt_siemens_et200sp_data_pub_->msg_.table_error = return_state_if_exists(
        rt_siemens_et200sp_data_pub_->msg_.table_error,
        gpio_name_ + "/table_error", state_interfaces_, true);
    rt_siemens_et200sp_data_pub_->msg_.load_error = return_state_if_exists(
        rt_siemens_et200sp_data_pub_->msg_.load_error,
        gpio_name_ + "/load_error", state_interfaces_, true);
    rt_siemens_et200sp_data_pub_->msg_.unload_error = return_state_if_exists(
        rt_siemens_et200sp_data_pub_->msg_.unload_error,
        gpio_name_ + "/unload_error", state_interfaces_, true);
    rt_siemens_et200sp_data_pub_->msg_.battery_error = return_state_if_exists(
        rt_siemens_et200sp_data_pub_->msg_.battery_error,
        gpio_name_ + "/battery_error", state_interfaces_, true);
    rt_siemens_et200sp_data_pub_->msg_.docking_error = return_state_if_exists(
        rt_siemens_et200sp_data_pub_->msg_.docking_error,
        gpio_name_ + "/docking_error", state_interfaces_, true);
    rt_siemens_et200sp_data_pub_->msg_.auto_sw =
        return_state_if_exists(rt_siemens_et200sp_data_pub_->msg_.auto_sw,
                               gpio_name_ + "/auto_sw", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.man_sw =
        return_state_if_exists(rt_siemens_et200sp_data_pub_->msg_.man_sw,
                               gpio_name_ + "/man_sw", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.reset_sw =
        return_state_if_exists(rt_siemens_et200sp_data_pub_->msg_.reset_sw,
                               gpio_name_ + "/reset_sw", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.table_front_load_sensor =
        return_state_if_exists(
            rt_siemens_et200sp_data_pub_->msg_.table_front_load_sensor,
            gpio_name_ + "/table_front_load_sensor", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.table_rear_load_sensor =
        return_state_if_exists(
            rt_siemens_et200sp_data_pub_->msg_.table_rear_load_sensor,
            gpio_name_ + "/table_rear_load_sensor", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.joy_connected = return_state_if_exists(
        rt_siemens_et200sp_data_pub_->msg_.joy_connected,
        gpio_name_ + "/joy_connected", state_interfaces_, false);
    rt_siemens_et200sp_data_pub_->msg_.is_emergency_lock =
        return_state_if_exists(
            rt_siemens_et200sp_data_pub_->msg_.is_emergency_lock,
            gpio_name_ + "/is_emergency_lock", state_interfaces_, true);
    rt_siemens_et200sp_data_pub_->msg_.loaded = return_state_if_exists(
        rt_siemens_et200sp_data_pub_->msg_.loaded, gpio_name_ + "/loaded",
        state_interfaces_, false);
    rt_siemens_et200sp_data_pub_->msg_.communication_faults =
        return_state_if_exists(
            rt_siemens_et200sp_data_pub_->msg_.communication_faults,
            gpio_name_ + "/communication_faults", state_interfaces_);
    rt_siemens_et200sp_data_pub_->msg_.plc_connected = return_state_if_exists(
        rt_siemens_et200sp_data_pub_->msg_.plc_connected,
        gpio_name_ + "/plc_connected", state_interfaces_, false);
    rt_siemens_et200sp_data_pub_->msg_.last_plc_read_write_time =
        return_state_if_exists(
            rt_siemens_et200sp_data_pub_->msg_.last_plc_read_write_time,
            gpio_name_ + "/last_plc_read_write_time", state_interfaces_);
    rt_siemens_et200sp_data_pub_->unlockAndPublish();
    if (rt_is_emergency_lock_data_pub_ &&
        rt_is_emergency_lock_data_pub_->trylock()) {
      rt_is_emergency_lock_data_pub_->msg_.data =
          rt_siemens_et200sp_data_pub_->msg_.is_emergency_lock;
      rt_is_emergency_lock_data_pub_->unlockAndPublish();
    }
  }

  return controller_interface::return_type::OK;
}

} // namespace drc_flexnav_controllers

PLUGINLIB_EXPORT_CLASS(drc_flexnav_controllers::Siemens_ET200SP_Broadcaster,
                       controller_interface::ControllerInterface)
