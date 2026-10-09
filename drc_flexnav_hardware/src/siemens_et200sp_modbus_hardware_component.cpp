#include "drc_flexnav_hardware/siemens_et200sp_modbus_hardware_component.hpp"
#include "drc_flexnav_hardware/utils.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include <limits>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>

namespace drc_flexnav_hardware {

Siemens_ET200SP_Modbus_HW_Component::Siemens_ET200SP_Modbus_HW_Component()
    : odom_(wheel_to_robot_center_,
            std::vector<double>{wheel_radius_, wheel_radius_}) {}

CallbackReturn Siemens_ET200SP_Modbus_HW_Component::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),
        "Error occured during Siemens_ET200SP_Modbus_HW_Component "
        "hardware's init while calling "
        "hardware_interface::SystemInterface::on_init(info)");
    return hardware_interface::CallbackReturn::ERROR;
  }

  plc_ip_ = info_.hardware_parameters["plc_ip"];
  modbus_communication_port_ =
      std::stoi(info_.hardware_parameters["modbus_communication_port"]);
  output_topic_ = info_.hardware_parameters["output_topic"];
  input_prefix_ = info_.hardware_parameters["input_prefix"];
  frequency_ = std::stoi(info_.hardware_parameters["plc_frequency"]);
  int plc_connection_timeout_ms =
      std::stoi(info_.hardware_parameters["plc_connection_timeout_ms"]);
  plc_connection_timeout_ =
      std::chrono::milliseconds(plc_connection_timeout_ms);
  int plc_connection_retry_ms =
      std::stoi(info_.hardware_parameters["plc_connection_retry_ms"]);
  plc_connection_retry_ = std::chrono::milliseconds(plc_connection_retry_ms);

  left_wheel_joint_name_ = info_.hardware_parameters["left_encoder_joint_name"];
  right_wheel_joint_name_ =
      info_.hardware_parameters["right_encoder_joint_name"];
  encoder_max_counts_ =
      std::stoi(info_.hardware_parameters["wheel_encoder_max_counts"]);
  gear_ratio_ = std::stod(info_.hardware_parameters["gear_ratio"]);
  std::string export_position_interface_param =
      info_.hardware_parameters["export_position_interface"];
  std::istringstream(export_position_interface_param) >> std::boolalpha >>
      export_position_interface;

  gpio_name_ = info_.hardware_parameters["gpio_name"];

  std::string gpio_interfaces = info_.hardware_parameters["gpio_interfaces"];
  std::stringstream gpio_interfaces_ss(gpio_interfaces);
  std::string gpio_interface;
  while (gpio_interfaces_ss >> gpio_interface)
    gpio_interfaces_.push_back(gpio_interface);

  std::string gpio_state_interfaces =
      info_.hardware_parameters["gpio_state_interfaces"];
  std::stringstream gpio_state_interfaces_ss(gpio_state_interfaces);
  std::string gpio_state_interface;
  while (gpio_state_interfaces_ss >> gpio_state_interface)
    gpio_state_interfaces_.push_back(gpio_state_interface);

  odometer_name_ = info_.hardware_parameters["odometer_name"];
  wheel_to_robot_center_ =
      std::stod(info_.hardware_parameters["wheel_to_robot_center"]);
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);

  std::string use_joint_vel_param =
      info_.hardware_parameters["odom_use_joint_vel"];
  std::istringstream(use_joint_vel_param) >> std::boolalpha >> use_joint_vel_;

  // Return a callback error if the actuated joints are not present in the UDRF
  if (std::find_if(info_.joints.begin(), info_.joints.end(),
                   [&](const auto &joint) {
                     return joint.name == left_wheel_joint_name_;
                   }) == info_.joints.end()) {
    RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),
        "Error occured during Siemens_ET200SP_Modbus_HW_Component "
        "hardware's init while calling "
        "because left_wheel_joint_name: "
            << left_wheel_joint_name_ << " is not present in UDRF.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (std::find_if(info_.joints.begin(), info_.joints.end(),
                   [&](const auto &joint) {
                     return joint.name == right_wheel_joint_name_;
                   }) == info_.joints.end()) {
    RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),
        "Error occured during Siemens_ET200SP_Modbus_HW_Component "
        "hardware's init while calling "
        "because right_wheel_joint_name: "
            << right_wheel_joint_name_ << " is not present in UDRF.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (std::find_if(info_.gpios.begin(), info_.gpios.end(),
                   [&](const auto &gpio) { return gpio.name == gpio_name_; }) ==
      info_.gpios.end()) {
    RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),
        "Error occured during Siemens_ET200SP_Modbus_HW_Component "
        "hardware's init while calling "
        "because gpio: "
            << gpio_name_ << " is not present in UDRF.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (std::find_if(info_.gpios.begin(), info_.gpios.end(),
                   [&](const auto &gpio) { return gpio.name == gpio_name_; }) ==
      info_.gpios.end()) {
    RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),
        "Error occured during Siemens_ET200SP_Modbus_HW_Component "
        "hardware's init while calling "
        "because gpio: "
            << gpio_name_ << " is not present in UDRF.");

    return hardware_interface::CallbackReturn::ERROR;
  }
  if (std::find_if(info_.sensors.begin(), info_.sensors.end(),
                   [&](const auto &sensor) {
                     return sensor.name == odometer_name_;
                   }) == info_.sensors.end()) {
    RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),
        "Error occured during Siemens_ET200SP_Modbus_HW_Component "
        "hardware's init while calling "
        "because odometer: "
            << odometer_name_ << " is not present in UDRF.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  // Create ROS2 node
  hw_node_ = std::make_shared<rclcpp::Node>("siemens_et200sp_hw_interface_node");

  // Command response to CLP
  std::string to_plc_command_topic_ = "to_plc_command";
  try {
    command_subscription_ = hw_node_->create_subscription<drc_flexnav_controllers_msgs::msg::PlcCommand>(
        to_plc_command_topic_, 10,
        [this](const drc_flexnav_controllers_msgs::msg::PlcCommand::SharedPtr msg) {
          plc_command_response_.writeFromNonRT(msg);
          plc_command_response_flag_ = true;
        });
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),
                        "Exception thrown in on_configure(): " << e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Mission response to CLP
  std::string to_plc_mission_topic_ = "to_plc_mission";
  try {
    missions_subscription_ = hw_node_->create_subscription<drc_flexnav_controllers_msgs::msg::PlcMissionResponse>(
        to_plc_mission_topic_, 10,
        [this](const drc_flexnav_controllers_msgs::msg::PlcMissionResponse::SharedPtr msg) {
          plc_missions_response_.writeFromNonRT(msg);
          plc_missions_response_flag_ = true;
        });
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),
                        "Exception thrown in on_configure(): " << e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Publisher to plc_response
  std::string plc_request_topic_ = "plc_request";
  plc_request_pub_ = hw_node_->create_publisher<drc_flexnav_controllers_msgs::msg::PlcRequest>(plc_request_topic_, rclcpp::SystemDefaultsQoS());
  rt_plc_request_pub_ = std::make_shared<realtime_tools::RealtimePublisher<drc_flexnav_controllers_msgs::msg::PlcRequest>>(plc_request_pub_);

  // Run node on thread
  hw_node_spin_thread_ = std::thread([this]() {
     rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(hw_node_);
      RCLCPP_INFO(hw_node_->get_logger(), "hw_node_ running!");
      exec.spin();
  });

  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn Siemens_ET200SP_Modbus_HW_Component::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  std::cerr << "Siemens_ET200SP_Modbus_HW_Component on_configure, plc_ip_: "
            << plc_ip_.c_str()
            << " modbus_communication_port_: " << modbus_communication_port_
            << std::endl;
  if (!siemens_et200sp.is_connected()) {
    std::cerr << "Siemens_ET200SP_Modbus_HW_Component on_configure: plc not "
                 "connected"
              << std::endl;
    is_connected_flag_ = false;
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),
        "Trying connect to " << plc_ip_.c_str() << ":"
                             << modbus_communication_port_);

    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      auto elapsed_time = std::chrono::steady_clock::now() - start_time;
      if (elapsed_time > plc_connection_timeout_) {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),
            "Siemens_ET200SP connection timeout ");
        // TODO: implement on_error
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (siemens_et200sp.is_connected())
        break;

      siemens_et200sp.disconnect();
      siemens_et200sp.connect(
          plc_ip_.c_str(), static_cast<uint16_t>(modbus_communication_port_),
          1000000 / frequency_); // parameter timeout in microseconds
      //

      std::this_thread::sleep_for(plc_connection_retry_);
    }
  }
  std::cerr << "Siemens_ET200SP_Modbus_HW_Component on_configure: plc connected"
            << std::endl;
  is_connected_flag_ = true;
  siemens_et200sp.syncro();

  odom_.setParams(wheel_to_robot_center_,
                  std::vector<double>{wheel_radius_, wheel_radius_});
  phi_.setZero();
  deltaPhi_.setZero();
  phiDot_.setZero();

  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn Siemens_ET200SP_Modbus_HW_Component::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  auto callback_return = hardware_interface::CallbackReturn::SUCCESS;

  siemens_et200sp.disconnect();
  is_connected_flag_ = false;

  return callback_return;
}

CallbackReturn Siemens_ET200SP_Modbus_HW_Component::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  // Stop hw_node thread
  rclcpp::shutdown();
  if (hw_node_spin_thread_.joinable()) {
    hw_node_spin_thread_.join();
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn Siemens_ET200SP_Modbus_HW_Component::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  auto callback_return = hardware_interface::CallbackReturn::SUCCESS;

  RCLCPP_ERROR_STREAM(rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),
                      "on_activate");

  if (siemens_et200sp.is_connected()) {
    siemens_et200sp.syncro();
    if (!is_connected_flag_) {
      is_connected_flag_ = true;
      RCLCPP_INFO_STREAM(
          rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),
          "Connected to " << plc_ip_.c_str() << ":"
                          << modbus_communication_port_);
      std::this_thread::sleep_for(plc_connection_retry_);
    }
  } else {
    callback_return = hardware_interface::CallbackReturn::ERROR;
  }

  last_timestamp = siemens_et200sp.get_message_timestamp_ms();
  siemens_et200sp.syncro();

  RCLCPP_ERROR_STREAM(rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),
                      "on_activate motors init ended");
  return callback_return;
}

CallbackReturn Siemens_ET200SP_Modbus_HW_Component::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Siemens_ET200SP_Modbus_HW_Component::export_state_interfaces() {
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
        }
      }
    }
  }

  std::vector<std::string>::iterator gpio_it, gpio_end;
  for (auto &gpio : info_.gpios) {
    if (gpio.name == gpio_name_) {
      for (const auto &state_interface : gpio.state_interfaces) {
        gpio_it = find(gpio_interfaces_.begin(), gpio_interfaces_.end(),
                       state_interface.name);
        gpio_end = gpio_interfaces_.end();
        if (gpio_it == gpio_end) {
          gpio_it = find(gpio_state_interfaces_.begin(),
                         gpio_state_interfaces_.end(), state_interface.name);
          gpio_end = gpio_state_interfaces_.end();
        }
        if (gpio_it != gpio_end) {
          gpio_state_interface.insert(
              {state_interface.name, std::numeric_limits<double>::quiet_NaN()});
          state_interfaces.push_back(hardware_interface::StateInterface(
              gpio.name, state_interface.name,
              &gpio_state_interface[state_interface.name]));
        }
      }
    }
  }
  for (auto &sensor : info_.sensors) {
    if (sensor.name == odometer_name_) {
      try {
        for (const auto &state_interface : sensor.state_interfaces) {
          odometry_values_[state_interface.name] = 0.0;
          state_interfaces.push_back(hardware_interface::StateInterface(
              odometer_name_, state_interface.name,
              &odometry_values_[state_interface.name]));
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),
            "Error occured when trying to export odometer '"
                << odometer_name_
                << "' state interfaces during "
                   "Siemens_ET200SP_Modbus_HW_Component "
                   "export_state_interfaces: "
                << e.what());
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Siemens_ET200SP_Modbus_HW_Component::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  std::vector<std::string>::iterator gpio_it;
  for (auto &gpio : info_.gpios) {
    if (gpio.name == gpio_name_) {
      for (const auto &command_interface : gpio.command_interfaces) {
        gpio_it = find(gpio_interfaces_.begin(), gpio_interfaces_.end(),
                       command_interface.name);
        if (gpio_it != gpio_interfaces_.end()) {
          gpio_command_interface.insert(
              {command_interface.name,
               std::numeric_limits<double>::quiet_NaN()});
          command_interfaces.push_back(hardware_interface::CommandInterface(
              gpio.name, command_interface.name,
              &gpio_command_interface[command_interface.name]));
        }
      }
    }
  }
  return command_interfaces;
}

// NOTE: Optional methods:
//
/* hardware_interface::return_type
 * Siemens_ET200SP_Modbus_HW_Component::prepare_command_mode_switch(
 */
/*     const std::vector<std::string> &start_interfaces, */
/*     const std::vector<std::string> &stop_interfaces) ; */
/**/
/* hardware_interface::return_type
 * Siemens_ET200SP_Modbus_HW_Component::perform_command_mode_switch(
 */
/*     const std::vector<std::string> &start_interfaces, */
/*     const std::vector<std::string> &stop_interfaces) ; */

hardware_interface::return_type
Siemens_ET200SP_Modbus_HW_Component::read(const rclcpp::Time &time,
                                          const rclcpp::Duration &period) {
  auto hw_interface_return = hardware_interface::return_type::OK;

  siemens_et200sp.read();

  auto left_encoder = siemens_et200sp.get_left_encoder_counts();
  auto right_encoder = siemens_et200sp.get_right_encoder_counts();

  update_map_if_key_exists(
      state_interface_position, left_wheel_joint_name_,
      convert_count_to_rad(left_encoder, encoder_max_counts_));
  update_map_if_key_exists(
      state_interface_velocity, left_wheel_joint_name_,
      convert_count_to_rad_s(left_encoder, last_left_encoder,
                             encoder_max_counts_, period.nanoseconds() / 1e6));
  update_map_if_key_exists(
      state_interface_position, right_wheel_joint_name_,
      convert_count_to_rad(right_encoder, encoder_max_counts_));
  update_map_if_key_exists(
      state_interface_velocity, right_wheel_joint_name_,
      convert_count_to_rad_s(right_encoder, last_right_encoder,
                             encoder_max_counts_, period.nanoseconds() / 1e6));
  last_left_encoder = left_encoder;
  last_right_encoder = right_encoder;

  auto plc_msg_time_stamp = siemens_et200sp.get_message_timestamp_ms();
  auto delta_t = plc_msg_time_stamp - last_timestamp;
  last_timestamp = plc_msg_time_stamp;

  deltaPhi_ = -phi_;
  if (std::isnan(state_interface_position[left_wheel_joint_name_]) ||
      std::isnan(state_interface_position[right_wheel_joint_name_]) ||
      std::isnan(state_interface_velocity[left_wheel_joint_name_]) ||
      std::isnan(state_interface_velocity[right_wheel_joint_name_])) {
    odom_.resetVelocity();
  } else {
    phi_[0] = state_interface_position[left_wheel_joint_name_];
    phiDot_[0] = state_interface_velocity[left_wheel_joint_name_];

    phi_[1] = state_interface_position[right_wheel_joint_name_];
    phiDot_[1] = state_interface_velocity[right_wheel_joint_name_];

    deltaPhi_ += phi_;

    if (use_joint_vel_)
      odom_.update(deltaPhi_, phiDot_);
    else
      odom_.update(deltaPhi_, period);
  }
  Eigen::Vector3d x;
  Eigen::Vector2d u;
  odom_.getPose(x);
  odom_.getVelocity(u);

  odometry_values_["pose.position.x"] = x[0];
  odometry_values_["pose.position.y"] = x[1];
  odometry_values_["pose.position.z"] = 0.0;

  auto quat = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), x[2]));

  odometry_values_["pose.orientation.x"] = quat.x;
  odometry_values_["pose.orientation.y"] = quat.y;
  odometry_values_["pose.orientation.z"] = quat.z;
  odometry_values_["pose.orientation.w"] = quat.w;

  odometry_values_["twist.linear.x"] = u[0] * cos(x[2]);
  odometry_values_["twist.linear.y"] = u[0] * sin(x[2]);
  odometry_values_["twist.linear.z"] = 0;
  odometry_values_["twist.angular.x"] = 0;
  odometry_values_["twist.angular.y"] = 0;
  odometry_values_["twist.angular.z"] = u[1];

  update_map_if_key_exists(gpio_state_interface, std::string("time_stamp"),
                           static_cast<double>(plc_msg_time_stamp));
  update_map_if_key_exists(gpio_state_interface, std::string("delta_t"),
                           static_cast<double>(delta_t));
  update_map_if_key_exists(
      gpio_state_interface, std::string("left_encoder"),
      static_cast<double>(siemens_et200sp.get_left_encoder_counts()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("right_encoder"),
      static_cast<double>(siemens_et200sp.get_right_encoder_counts()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("left_encoder_rad"),
      static_cast<double>(convert_count_to_rad(
          siemens_et200sp.get_left_encoder_counts(), encoder_max_counts_)));
  update_map_if_key_exists(
      gpio_state_interface, std::string("right_encoder_rad"),
      static_cast<double>(convert_count_to_rad(
          siemens_et200sp.get_right_encoder_counts(), encoder_max_counts_)));
  update_map_if_key_exists(
      gpio_state_interface, std::string("left_encoder_rpm"),
      static_cast<double>(convert_count_to_rpm(
          siemens_et200sp.get_left_encoder_counts(), last_left_encoder,
          encoder_max_counts_, delta_t)));
  update_map_if_key_exists(
      gpio_state_interface, std::string("right_encoder_rpm"),
      static_cast<double>(convert_count_to_rpm(
          siemens_et200sp.get_left_encoder_counts(), last_right_encoder,
          encoder_max_counts_, delta_t)));
  update_map_if_key_exists(
      gpio_state_interface, std::string("joy_up"),
      static_cast<double>(siemens_et200sp.get_joystick_up()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("joy_down"),
      static_cast<double>(siemens_et200sp.get_joystick_down()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("joy_left"),
      static_cast<double>(siemens_et200sp.get_joystick_left()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("joy_right"),
      static_cast<double>(siemens_et200sp.get_joystick_right()));
  /* update_map_if_key_exists( */
  /*     gpio_state_interface, std::string("docked_front"), */
  /*     static_cast<double>((siemens_et200sp.getDockingStatus() & */
  /*                          siemens_et200sp.ROBOT_DOCKED_FRONT) > 0)); */
  /* update_map_if_key_exists( */
  /*     gpio_state_interface, std::string("docked_rear"), */
  /*     static_cast<double>((siemens_et200sp.getDockingStatus() & */
  /*                          siemens_et200sp.ROBOT_DOCKED_REAR) > 0)); */
  update_map_if_key_exists(gpio_state_interface, std::string("charging"),
                           static_cast<double>(siemens_et200sp.is_charging()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("front_ls_violation"),
      static_cast<double>(
          siemens_et200sp.get_front_laser_scanner_active_zone()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("rear_ls_violation"),
      static_cast<double>(
          siemens_et200sp.get_rear_laser_scanner_active_zone()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("reset_wait"),
      static_cast<double>(siemens_et200sp.is_waiting_user_reset_error_codes()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("undefined_error"),
      static_cast<double>(siemens_et200sp.has_undefined_error()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("restart_detected"),
      static_cast<double>(siemens_et200sp.has_initial_error_code()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("left_motor_error"),
      static_cast<double>(siemens_et200sp.has_left_motor_error()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("right_motor_error"),
      static_cast<double>(siemens_et200sp.has_right_motor_error()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("table_error"),
      static_cast<double>(siemens_et200sp.has_table_motor_error()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("load_error"),
      static_cast<double>(siemens_et200sp.has_loading_error()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("unload_error"),
      static_cast<double>(siemens_et200sp.has_unloading_error()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("battery_error"),
      static_cast<double>(siemens_et200sp.has_battery_error()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("docking_error"),
      static_cast<double>(siemens_et200sp.has_docking_error()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("auto_sw"),
      static_cast<double>(siemens_et200sp.is_automatic_mode_selected()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("man_sw"),
      static_cast<double>(siemens_et200sp.is_manual_mode_selected()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("reset_sw"),
      static_cast<double>(siemens_et200sp.is_reset_button_pressed()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("table_front_load_sensor"),
      static_cast<double>(siemens_et200sp.get_front_load_sensor_state()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("table_rear_load_sensor"),
      static_cast<double>(siemens_et200sp.get_rear_load_sensor_state()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("joy_connected"),
      static_cast<double>(siemens_et200sp.is_joystick_connected()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("is_emergency_lock"),
      static_cast<double>(siemens_et200sp.is_emergency_lock()));
  update_map_if_key_exists(gpio_state_interface, std::string("loaded"),
                           static_cast<double>(siemens_et200sp.is_loaded()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("communication_faults"),
      static_cast<double>(siemens_et200sp.get_error_count()));
  update_map_if_key_exists(gpio_state_interface, std::string("plc_connected"),
                           static_cast<double>(siemens_et200sp.is_connected()));
  update_map_if_key_exists(
      gpio_state_interface, std::string("last_plc_read_write_time"),
      static_cast<double>(siemens_et200sp.get_last_read_write_time()));

  // Check if plc_request publish
  static robot_command_t siemens_et200sp_last_received_command_ = siemens_et200sp.get_command();
  if(siemens_et200sp.get_command() != siemens_et200sp_last_received_command_){
    // New command from PLC
    switch (siemens_et200sp.get_command())
    {
    case ROBOT_SHUTDOWN:
      // Publish plc_request
      if (rt_plc_request_pub_ && rt_plc_request_pub_->trylock()) {
          rt_plc_request_pub_->msg_.header.stamp = time;
          rt_plc_request_pub_->msg_.command = "shutdown";
          rt_plc_request_pub_->msg_.mission_label = "\0";
          rt_plc_request_pub_->msg_.mission_type = "\0";
          rt_plc_request_pub_->msg_.mission_direction = "\0";
          rt_plc_request_pub_->unlockAndPublish();
      }
      break;
    
    default:
      break;
    }

    siemens_et200sp_last_received_command_ = siemens_et200sp.get_command();
  }
  else{
    // No PLC command. Check mission command
    if(siemens_et200sp.is_new_mission_cmd()){
      if(siemens_et200sp.is_mission_cancel_cmd()){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"), "Mission cancel!");
        // Publish plc_request (mission_cancel)
        if (rt_plc_request_pub_ && rt_plc_request_pub_->trylock()) {
            rt_plc_request_pub_->msg_.header.stamp = time;
            rt_plc_request_pub_->msg_.command = "mission_cancel";
            rt_plc_request_pub_->msg_.mission_label = "\0";
            rt_plc_request_pub_->msg_.mission_type = "\0";
            rt_plc_request_pub_->msg_.mission_direction = "\0";
            rt_plc_request_pub_->unlockAndPublish();
        }
      }
      else{
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"), "Mission start!");
        // Publish plc_request (mission_start)
        if (rt_plc_request_pub_ && rt_plc_request_pub_->trylock()) {
          rt_plc_request_pub_->msg_.header.stamp = time;

          // Set command
          rt_plc_request_pub_->msg_.command = "mission_start";

          // Set label
          char mission_label[5];
          siemens_et200sp.get_mission_node_label(mission_label);
          rt_plc_request_pub_->msg_.mission_label = mission_label;

          if(siemens_et200sp.is_mission_move_only())               rt_plc_request_pub_->msg_.mission_type = "transfer";
          else if(siemens_et200sp.is_mission_load_at_dest())       rt_plc_request_pub_->msg_.mission_type = "loading";
          else if(siemens_et200sp.is_mission_unload_at_dest())     rt_plc_request_pub_->msg_.mission_type = "unloading";

          // Set direction
          if(siemens_et200sp.is_mission_dock_from_rear())     rt_plc_request_pub_->msg_.mission_direction = "rear";
          else                                                rt_plc_request_pub_->msg_.mission_direction = "front";
          
          rt_plc_request_pub_->unlockAndPublish();
        }
      }
    }
  }

  return hw_interface_return;
}

void Siemens_ET200SP_Modbus_HW_Component::reset_state_interfaces(
    const std::string &joint) {
  update_map_if_key_exists(state_interface_position, joint,
                           std::numeric_limits<double>::quiet_NaN());
  update_map_if_key_exists(state_interface_velocity, joint,
                           std::numeric_limits<double>::quiet_NaN());
}

void Siemens_ET200SP_Modbus_HW_Component::reset_command_interfaces(
    const std::string &joint) {
  update_map_if_key_exists(command_interface_position, joint,
                           std::numeric_limits<double>::quiet_NaN());
  update_map_if_key_exists(command_interface_velocity, joint,
                           std::numeric_limits<double>::quiet_NaN());
}
void Siemens_ET200SP_Modbus_HW_Component::reset_gpio_interfaces(
    const std::string &gpio_interface) {
  update_map_if_key_exists(gpio_state_interface, gpio_interface,
                           std::numeric_limits<double>::quiet_NaN());
  update_map_if_key_exists(gpio_command_interface, gpio_interface,
                           std::numeric_limits<double>::quiet_NaN());
}

hardware_interface::return_type Siemens_ET200SP_Modbus_HW_Component::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  try {
    if (mapped_value_is_valid(gpio_command_interface,
                              std::string("green_front_light"))) {
      siemens_et200sp.set_front_green_light_state(
          static_cast<robot_signaling_status_t>(
              std::floor(gpio_command_interface["green_front_light"])));
    }
    if (mapped_value_is_valid(gpio_command_interface,
                              std::string("green_rear_light"))) {
      siemens_et200sp.set_rear_green_light_state(
          static_cast<robot_signaling_status_t>(
              std::floor(gpio_command_interface["green_rear_light"])));
    }
    if (mapped_value_is_valid(gpio_command_interface,
                              std::string("red_front_light"))) {
      siemens_et200sp.set_front_red_light_state(
          static_cast<robot_signaling_status_t>(
              std::floor(gpio_command_interface["red_front_light"])));
    }
    if (mapped_value_is_valid(gpio_command_interface,
                              std::string("red_rear_light"))) {
      siemens_et200sp.set_rear_red_light_state(
          static_cast<robot_signaling_status_t>(
              std::floor(gpio_command_interface["red_rear_light"])));
    }
    if (mapped_value_is_valid(gpio_command_interface,
                              std::string("left_light"))) {
      siemens_et200sp.set_front_red_light_state(
          static_cast<robot_signaling_status_t>(
              std::floor(gpio_command_interface["left_light"])));
    }
    if (mapped_value_is_valid(gpio_command_interface,
                              std::string("right_light"))) {
      siemens_et200sp.set_rear_red_light_state(
          static_cast<robot_signaling_status_t>(
              std::floor(gpio_command_interface["right_light"])));
    }
    if (mapped_value_is_valid(gpio_command_interface, std::string("buzzer"))) {
      siemens_et200sp.set_buzzer_state(static_cast<robot_signaling_status_t>(
          std::floor(gpio_command_interface["buzzer"])));
    }
    if (mapped_value_is_valid(gpio_command_interface,
                              std::string("load_unload_timeout_seconds"))) {
      siemens_et200sp.set_load_unload_timeout_seconds(static_cast<uint8_t>(
          std::floor(gpio_command_interface["load_unload_timeout_seconds"])));
    }
    if (mapped_value_is_valid(gpio_command_interface,
                              std::string("load_command"))) {
      siemens_et200sp.send_command(static_cast<robot_command_t>(
          std::floor(gpio_command_interface["load_command"])));
    }
    if (mapped_value_is_valid(gpio_command_interface,
                              std::string("ls_config"))) {
      siemens_et200sp.set_laser_scanner_config(static_cast<uint8_t>(
          std::floor(gpio_command_interface["ls_config"])));
    }

    // Send command to PLC
    auto to_plc_command_buffer = plc_command_response_.readFromRT();
    if (to_plc_command_buffer && *to_plc_command_buffer && plc_command_response_flag_) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),"Send mission command to PLC!");
      siemens_et200sp.send_command((robot_command_t)(*to_plc_command_buffer)->command);
      plc_command_response_flag_ = false;
    }

    // Send mission response to PLC
    auto to_plc_mission_buffer = plc_missions_response_.readFromRT();
    if (to_plc_mission_buffer && *to_plc_mission_buffer && plc_missions_response_flag_) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),"Send mission status to PLC!");
      char mission_message[21];
      std::snprintf(mission_message, sizeof(mission_message), "%s", (*to_plc_mission_buffer)->message.c_str());
      siemens_et200sp.set_mission_status((mission_status_tp)(*to_plc_mission_buffer)->mission_status);
      siemens_et200sp.send_message(mission_message, (message_tp)(*to_plc_mission_buffer)->message_type, (*to_plc_mission_buffer)->message_code);
      plc_missions_response_flag_ = false;
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("Siemens_ET200SP_Modbus_HW_Component"),
        "Error occured during Siemens_ET200SP_Modbus_HW_Component write: "
            << e.what());
  }
  siemens_et200sp.write();
  return hardware_interface::return_type::OK;
}

} // namespace drc_flexnav_hardware

PLUGINLIB_EXPORT_CLASS(
    drc_flexnav_hardware::Siemens_ET200SP_Modbus_HW_Component,
    hardware_interface::SystemInterface)
