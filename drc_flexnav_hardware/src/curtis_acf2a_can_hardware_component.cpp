#include "drc_flexnav_hardware/curtis_acf2a_can_hardware_component.hpp"
#include "drc_flexnav_hardware/utils.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include <limits>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <stdexcept>

namespace drc_flexnav_hardware {

Curtis_ACF2A_Can_HW_Component::Curtis_ACF2A_Can_HW_Component()
    : left_motor(canopen_left), right_motor(canopen_right) {}

CallbackReturn Curtis_ACF2A_Can_HW_Component::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("Curtis_ACF2A_Can_HW_Component"),
                        "Error occured during Curtis_ACF2A_Can_HW_Component "
                        "hardware's init while calling "
                        "hardware_interface::SystemInterface::on_init(info)");
    return hardware_interface::CallbackReturn::ERROR;
  }

  left_wheel_joint_name_ = info_.hardware_parameters["left_motor_joint_name"];
  right_wheel_joint_name_ = info_.hardware_parameters["right_motor_joint_name"];
  can_left_port_ = info_.hardware_parameters["can_left_port"];
  can_right_port_ = info_.hardware_parameters["can_right_port"];
  left_motor_id_ = static_cast<uint8_t>(
      std::stoi(info_.hardware_parameters["left_motor_id"]));
  right_motor_id_ = static_cast<uint8_t>(
      std::stoi(info_.hardware_parameters["right_motor_id"]));
  left_motor_swap_dir_ = static_cast<uint8_t>(
      std::stoi(info_.hardware_parameters["left_motor_swap_dir"]));
  right_motor_swap_dir_ = static_cast<uint8_t>(
      std::stoi(info_.hardware_parameters["right_motor_swap_dir"]));
  left_motor_control_mode_ =
      std::stoi(info_.hardware_parameters["left_motor_control_mode"]);
  right_motor_control_mode_ =
      std::stoi(info_.hardware_parameters["right_motor_control_mode"]);
  encoder_max_counts_ =
      std::stoi(info_.hardware_parameters["encoder_max_counts"]);
  gear_ratio_ = std::stod(info_.hardware_parameters["gear_ratio"]);
  std::string use_gear_ratio_param =
      info_.hardware_parameters["use_gear_ratio"];
  std::istringstream(use_gear_ratio_param) >> std::boolalpha >> use_gear_ratio_;
  std::string export_position_interface_param =
      info_.hardware_parameters["export_position_interface"];
  std::istringstream(export_position_interface_param) >> std::boolalpha >>
      export_position_interface;

  // Return a callback error if the actuated joints are not present in the UDRF
  if (std::find_if(info_.joints.begin(), info_.joints.end(),
                   [&](const auto &joint) {
                     return joint.name == left_wheel_joint_name_;
                   }) == info_.joints.end()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("Curtis_ACF2A_Can_HW_Component"),
                        "Error occured during Curtis_ACF2A_Can_HW_Component "
                        "hardware's init while calling "
                        "because left_wheel_joint_name: "
                            << left_wheel_joint_name_
                            << " is not present in UDRF.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (std::find_if(info_.joints.begin(), info_.joints.end(),
                   [&](const auto &joint) {
                     return joint.name == right_wheel_joint_name_;
                   }) == info_.joints.end()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("Curtis_ACF2A_Can_HW_Component"),
                        "Error occured during Curtis_ACF2A_Can_HW_Component "
                        "hardware's init while calling "
                        "because right_wheel_joint_name: "
                            << right_wheel_joint_name_
                            << " is not present in UDRF.");

    return hardware_interface::CallbackReturn::ERROR;
  }
  /* rclcpp::NodeOptions node_opt; */
  /* node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>( */
  /*     controller_name, node_namespace, node_options, */
  /*     false); // disable LifecycleNode service interfaces */
  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn Curtis_ACF2A_Can_HW_Component::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  // Init CANOpen stack
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("Curtis_ACF2A_Can_HW_Component"),
                      "can_left_port: " << can_left_port_);
  if (!canopen_left.init(can_left_port_.c_str()))
    return hardware_interface::CallbackReturn::ERROR;
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("Curtis_ACF2A_Can_HW_Component"),
                      "can_right_port: " << can_right_port_);

  if (!canopen_right.init(can_right_port_.c_str()))
    return hardware_interface::CallbackReturn::ERROR;

  left_motor.init(left_motor_id_, left_motor_swap_dir_);
  right_motor.init(right_motor_id_, right_motor_swap_dir_);

  config_motors();

  std::cerr << "on_configure finished" << std::endl;

  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn Curtis_ACF2A_Can_HW_Component::on_cleanup(
    const rclcpp_lifecycle::State &previous_state) {
  auto callback_return = hardware_interface::CallbackReturn::SUCCESS;

  if (!canopen_left.deinit())
    callback_return = hardware_interface::CallbackReturn::ERROR;

  if (!canopen_right.deinit())
    callback_return = hardware_interface::CallbackReturn::ERROR;

  return callback_return;
}

CallbackReturn Curtis_ACF2A_Can_HW_Component::on_shutdown(
    const rclcpp_lifecycle::State &previous_state) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn Curtis_ACF2A_Can_HW_Component::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  auto callback_return = hardware_interface::CallbackReturn::SUCCESS;
  // Init motors
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("Curtis_ACF2A_Can_HW_Component"),
                      "on_activate");
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("Curtis_ACF2A_Can_HW_Component"),
                      "on_activate motors init ended");

  return callback_return;
}

// Config motors. WARNING: This function must be executed in parallel to
// can_read_parse_loop() to capture the responses from the Driver.
void Curtis_ACF2A_Can_HW_Component::config_motors() {
  //  config motor parameters
  left_motor.set_config_control_mode(left_motor_control_mode_);
  left_motor_configured = left_motor.config("verbose");

  right_motor.set_config_control_mode(right_motor_control_mode_);
  right_motor_configured = right_motor.config("error");

  motors_configured_flag = left_motor_configured && right_motor_configured;
}

CallbackReturn Curtis_ACF2A_Can_HW_Component::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {

  std::cerr << "on_deactivate" << std::endl;

  bool left_success{true}, right_success{true};
  left_motor_torque_ref_ = 0;
  right_motor_torque_ref_ = 0;

  if (motors_configured_flag) {
    left_success = left_motor.set_torque(left_motor_torque_ref_);
    right_success = right_motor.set_torque(right_motor_torque_ref_);
  }

  if (left_success && right_success)
    return hardware_interface::CallbackReturn::SUCCESS;

  return hardware_interface::CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface>
Curtis_ACF2A_Can_HW_Component::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto &joint : info_.joints) {
    if (joint.name == left_wheel_joint_name_ ||
        joint.name == right_wheel_joint_name_) {
      for (const auto &state_interface : joint.state_interfaces) {
        if (state_interface.name == hardware_interface::HW_IF_POSITION) {
          if (export_position_interface) {
            state_interfaces.push_back(hardware_interface::StateInterface(
                joint.name, hardware_interface::HW_IF_POSITION,
                &state_interface_position[joint.name]));
          }
          state_interface_position.insert(
              {joint.name, std::numeric_limits<double>::quiet_NaN()});
        } else if (state_interface.name == hardware_interface::HW_IF_VELOCITY) {
          state_interface_velocity.insert(
              {joint.name, std::numeric_limits<double>::quiet_NaN()});
          state_interfaces.push_back(hardware_interface::StateInterface(
              joint.name, hardware_interface::HW_IF_VELOCITY,
              &state_interface_velocity[joint.name]));
        } else if (state_interface.name == hardware_interface::HW_IF_EFFORT) {
          state_interface_effort.insert(
              {joint.name, std::numeric_limits<double>::quiet_NaN()});
          state_interfaces.push_back(hardware_interface::StateInterface(
              joint.name, hardware_interface::HW_IF_EFFORT,
              &state_interface_effort[joint.name]));
        } else if (state_interface.name == "nmt_state") {
          state_interface_nmt.insert(
              {joint.name, std::numeric_limits<double>::quiet_NaN()});
          state_interfaces.push_back(hardware_interface::StateInterface(
              joint.name, "nmt_state", &state_interface_nmt[joint.name]));
        } else if (state_interface.name == "motor_configured") {
          state_interface_motor_configured.insert(
            {joint.name, std::numeric_limits<double>::quiet_NaN()});
          state_interfaces.push_back(hardware_interface::StateInterface(
              joint.name, "motor_configured", &state_interface_motor_configured[joint.name]));
        }
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Curtis_ACF2A_Can_HW_Component::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto &joint : info_.joints) {
    if (joint.name == left_wheel_joint_name_ ||
        joint.name == right_wheel_joint_name_) {
      for (const auto &command_interface : joint.command_interfaces) {
        if (command_interface.name == hardware_interface::HW_IF_POSITION) {
          command_interface_position.insert(
              {joint.name, std::numeric_limits<double>::quiet_NaN()});
          command_interfaces.push_back(hardware_interface::CommandInterface(
              joint.name, hardware_interface::HW_IF_POSITION,
              &command_interface_position[joint.name]));
        } else if (command_interface.name ==
                   hardware_interface::HW_IF_VELOCITY) {
          command_interface_velocity.insert(
              {joint.name, std::numeric_limits<double>::quiet_NaN()});
          command_interfaces.push_back(hardware_interface::CommandInterface(
              joint.name, hardware_interface::HW_IF_VELOCITY,
              &command_interface_velocity[joint.name]));
        } else if (command_interface.name == hardware_interface::HW_IF_EFFORT) {
          command_interface_effort.insert(
              {joint.name, std::numeric_limits<double>::quiet_NaN()});
          command_interfaces.push_back(hardware_interface::CommandInterface(
              joint.name, hardware_interface::HW_IF_EFFORT,
              &command_interface_effort[joint.name]));
        }
      }
    }
  }
  return command_interfaces;
}

// NOTE: Optional methods:
//
/* hardware_interface::return_type
 * Curtis_ACF2A_Can_HW_Component::prepare_command_mode_switch(
 */
/*     const std::vector<std::string> &start_interfaces, */
/*     const std::vector<std::string> &stop_interfaces) ; */
/**/
/* hardware_interface::return_type
 * Curtis_ACF2A_Can_HW_Component::perform_command_mode_switch(
 */
/*     const std::vector<std::string> &start_interfaces, */
/*     const std::vector<std::string> &stop_interfaces) ; */

hardware_interface::return_type
Curtis_ACF2A_Can_HW_Component::read(const rclcpp::Time &time,
                                    const rclcpp::Duration &period) {
  auto hw_interface_return = hardware_interface::return_type::OK;

  update_map_if_key_exists(state_interface_motor_configured, left_wheel_joint_name_,
                           static_cast<double>(left_motor_configured));
  update_map_if_key_exists(state_interface_motor_configured, right_wheel_joint_name_,
                             static_cast<double>(right_motor_configured));

  auto left_motor_parse_result = left_motor.get_state();
  auto right_motor_parse_result = right_motor.get_state();

  update_map_if_key_exists(state_interface_nmt, left_wheel_joint_name_,
                           static_cast<double>(left_motor_parse_result));
  update_map_if_key_exists(state_interface_nmt, right_wheel_joint_name_,
                           static_cast<double>(right_motor_parse_result));

  if (left_motor_parse_result == COMM_TIMEOUT) {
    // timeout left motor
    left_motor_torque_ref_ = 0;
    reset_state_interfaces(left_wheel_joint_name_);
    /*hw_interface_return = hardware_interface::return_type::ERROR;*/
  } else {
    update_map_if_key_exists(
        state_interface_position, left_wheel_joint_name_,
        encoder2pos(left_motor.get_encoder(), encoder_max_counts_,
                    static_cast<double>(gear_ratio_), use_gear_ratio_));
    update_map_if_key_exists(
        state_interface_velocity, left_wheel_joint_name_,
        static_cast<double>(motor_rpm_to_rads(left_motor.get_speed(),
                                              gear_ratio_, use_gear_ratio_)));
    update_map_if_key_exists(state_interface_effort, left_wheel_joint_name_,
                             static_cast<double>(left_motor.get_torque()));
  }
  if (right_motor_parse_result == COMM_TIMEOUT) {
    // timeout right motor
    right_motor_torque_ref_ = 0;
    reset_state_interfaces(right_wheel_joint_name_);
    /*hw_interface_return = hardware_interface::return_type::ERROR;*/
  } else {
    update_map_if_key_exists(
        state_interface_position, right_wheel_joint_name_,
        encoder2pos(right_motor.get_encoder(), encoder_max_counts_,
                    static_cast<double>(gear_ratio_), use_gear_ratio_));
    update_map_if_key_exists(
        state_interface_velocity, right_wheel_joint_name_,
        static_cast<double>(
            motor_rpm_to_rads(right_motor.get_speed(), gear_ratio_),
            use_gear_ratio_));
    update_map_if_key_exists(state_interface_effort, right_wheel_joint_name_,
                             static_cast<double>(right_motor.get_torque()));
  }
  return hw_interface_return;
}

void Curtis_ACF2A_Can_HW_Component::reset_state_interfaces(
    const std::string &joint) {
  update_map_if_key_exists(state_interface_position, joint,
                           std::numeric_limits<double>::quiet_NaN());
  update_map_if_key_exists(state_interface_velocity, joint,
                           std::numeric_limits<double>::quiet_NaN());
  update_map_if_key_exists(state_interface_effort, joint,
                           std::numeric_limits<double>::quiet_NaN());
}

void Curtis_ACF2A_Can_HW_Component::reset_command_interfaces(
    const std::string &joint) {
  update_map_if_key_exists(command_interface_position, joint,
                           std::numeric_limits<double>::quiet_NaN());
  update_map_if_key_exists(command_interface_velocity, joint,
                           std::numeric_limits<double>::quiet_NaN());
  update_map_if_key_exists(command_interface_effort, joint,
                           std::numeric_limits<double>::quiet_NaN());
}

hardware_interface::return_type
Curtis_ACF2A_Can_HW_Component::write(const rclcpp::Time &time,
                                     const rclcpp::Duration &period) {
  bool left_success{false}, right_success{false};

  if (motors_configured_flag == false)
    return hardware_interface::return_type::OK;

  if (right_motor.get_state() != OPERATIONAL ||
      left_motor.get_state() != OPERATIONAL)
    return hardware_interface::return_type::OK;

  auto it = command_interface_effort.find(left_wheel_joint_name_);
  if (it != command_interface_effort.end()) {
    left_motor_torque_ref_ = static_cast<float>(it->second);
    left_success = left_motor.set_torque(left_motor_torque_ref_);
  }

  it = command_interface_effort.find(right_wheel_joint_name_);
  if (it != command_interface_effort.end()) {
    right_motor_torque_ref_ = static_cast<float>(it->second);
    right_success = right_motor.set_torque(right_motor_torque_ref_);
  }

  if (left_success && right_success)
    return hardware_interface::return_type::OK;

  return hardware_interface::return_type::OK;
}

} // namespace drc_flexnav_hardware

PLUGINLIB_EXPORT_CLASS(drc_flexnav_hardware::Curtis_ACF2A_Can_HW_Component,
                       hardware_interface::SystemInterface)
