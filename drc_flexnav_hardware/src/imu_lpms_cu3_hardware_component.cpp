#include "drc_flexnav_hardware/imu_lpms_cu3_hardware_component.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace drc_flexnav_hardware {

ImuLpmsCu3HardwareComponent::ImuLpmsCu3HardwareComponent() : imu_states_({}) {}

CallbackReturn ImuLpmsCu3HardwareComponent::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SensorInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("ImuLpmsCu3HardwareComponent"),
                 "Failed to initialize hardware interface");
    return CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  try {
    imu_port_ = info_.hardware_parameters.at("serial_port");
    imu_id_ = std::stoi(info_.hardware_parameters.at("identifier"));
    update_rate_ = std::stoi(info_.hardware_parameters.at("update_rate"));
    double_precision_ =
        info_.hardware_parameters.at("double_precision") == "true";
    serial_baudrate_ =
        std::stoi(info_.hardware_parameters.at("serial_baudrate"));
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("ImuLpmsCu3HardwareComponent"),
                 "Failed to parse IMU parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("ImuLpmsCu3HardwareComponent"),
              "Successfully initialized IMU hardware interface");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ImuLpmsCu3HardwareComponent::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("ImuLpmsCu3HardwareComponent"),
              "Configuring IMU...");

  if (!configure_imu()) {
    return CallbackReturn::ERROR;
  }

  start_resume_imu_thread();

  return CallbackReturn::SUCCESS;
}

CallbackReturn ImuLpmsCu3HardwareComponent::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  stop_imu_thread();
  reset_state_interfaces();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImuLpmsCu3HardwareComponent::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("ImuLpmsCu3HardwareComponent"),
              "Activating IMU...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImuLpmsCu3HardwareComponent::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("ImuLpmsCu3HardwareComponent"),
              "Deactivating IMU...");
  pause_imu_thread();
  return CallbackReturn::SUCCESS;
}

CallbackReturn drc_flexnav_hardware::ImuLpmsCu3HardwareComponent::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("ImuLpmsCu3HardwareComponent"),
              "Shutting down IMU hardware interface...");

  // Optional: Perform any cleanup actions needed during shutdown
  stop_imu_thread();
  reset_state_interfaces();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ImuLpmsCu3HardwareComponent::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Orientation quaternion
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "orientation.w", &imu_states_.orientation[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "orientation.x", &imu_states_.orientation[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "orientation.y", &imu_states_.orientation[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "orientation.z", &imu_states_.orientation[3]));

  // Angular velocity
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.x", &imu_states_.angular_velocity[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.y", &imu_states_.angular_velocity[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.z", &imu_states_.angular_velocity[2]));

  // Linear acceleration
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("imu_sensor", "linear_acceleration.x",
                                         &imu_states_.linear_acceleration[0]));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("imu_sensor", "linear_acceleration.y",
                                         &imu_states_.linear_acceleration[1]));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("imu_sensor", "linear_acceleration.z",
                                         &imu_states_.linear_acceleration[2]));

  // Magnetic field
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "mag_sensor", "magnetic_field.x", &imu_states_.magnetic_field[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "mag_sensor", "magnetic_field.y", &imu_states_.magnetic_field[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "mag_sensor", "magnetic_field.z", &imu_states_.magnetic_field[2]));

  return state_interfaces;
}

hardware_interface::return_type
ImuLpmsCu3HardwareComponent::read(const rclcpp::Time & /*read*/,
                                  const rclcpp::Duration & /*period*/) {

  // Data is updated in the imu_read_loop thread
  return hardware_interface::return_type::OK;
}

void ImuLpmsCu3HardwareComponent::start_resume_imu_thread() {
  if (!thread_started_) {
    stop_thread_ = false;
    imu_read_thread_ =
        std::thread(&ImuLpmsCu3HardwareComponent::imu_read_loop, this);
    thread_started_ = true;
  } else if (pause_thread_) {
    std::unique_lock<std::mutex> lock(mutex_);
    pause_thread_ = false;
    cv_.notify_one();
  }
}

void ImuLpmsCu3HardwareComponent::pause_imu_thread() {
  std::unique_lock<std::mutex> lock(mutex_);
  pause_thread_ = true;
}

void ImuLpmsCu3HardwareComponent::stop_imu_thread() {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    stop_thread_ = true;
    pause_thread_ = false;
  }
  cv_.notify_one();

  if (imu_read_thread_.joinable()) {
    imu_read_thread_.join();
    thread_started_ = false;
  }
}

void ImuLpmsCu3HardwareComponent::imu_read_loop() {
  while (!stop_thread_) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this]() { return !pause_thread_ || stop_thread_; });
      if (stop_thread_) {
        break;
      }
    }

    if (imu_interface_.get_stream_loop()) {
      // Update IMU data
      imu_states_.orientation[0] = imu_interface_.get_quaternion_w();
      imu_states_.orientation[1] = imu_interface_.get_quaternion_x();
      imu_states_.orientation[2] = imu_interface_.get_quaternion_y();
      imu_states_.orientation[3] = imu_interface_.get_quaternion_z();

      imu_states_.angular_velocity[0] = imu_interface_.get_angular_vel_x();
      imu_states_.angular_velocity[1] = imu_interface_.get_angular_vel_y();
      imu_states_.angular_velocity[2] = imu_interface_.get_angular_vel_z();

      imu_states_.linear_acceleration[0] = imu_interface_.get_accel_x();
      imu_states_.linear_acceleration[1] = imu_interface_.get_accel_y();
      imu_states_.linear_acceleration[2] = imu_interface_.get_accel_z();

      imu_states_.magnetic_field[0] = imu_interface_.get_mag_x();
      imu_states_.magnetic_field[1] = imu_interface_.get_mag_y();
      imu_states_.magnetic_field[2] = imu_interface_.get_mag_z();
    }
  }
}

void ImuLpmsCu3HardwareComponent::reset_state_interfaces() {
  for (int i = 0; i < 4; i++) {
    imu_states_.orientation[i] = 0.0;
  }
  for (int i = 0; i < 3; i++) {
    imu_states_.angular_velocity[i] = 0.0;
    imu_states_.linear_acceleration[i] = 0.0;
    imu_states_.magnetic_field[i] = 0.0;
  }
}

bool ImuLpmsCu3HardwareComponent::configure_imu() {
  if (!imu_interface_.init(imu_id_, imu_port_, serial_baudrate_)) {
    RCLCPP_ERROR(rclcpp::get_logger("ImuLpmsCu3HardwareComponent"),
                 "Failed to initialize IMU");
    return false;
  }

  imu_interface_.set_32bits_precision(double_precision_);

  if (!imu_interface_.set_stream_frequency(update_rate_)) {
    RCLCPP_ERROR(rclcpp::get_logger("ImuLpmsCu3HardwareComponent"),
                 "Failed to set IMU update rate");
    return false;
  }

  if (imu_interface_.set_operating_mode(FLEXNAV_IMU_OPERATING_MODE_STREAM)) {
    RCLCPP_INFO(rclcpp::get_logger("ImuLpmsCu3HardwareComponent"),
                "IMU set to stream mode");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ImuLpmsCu3HardwareComponent"),
                 "Failed to set IMU to stream mode");
    return false;
  }

  return true;
}

} // namespace drc_flexnav_hardware

PLUGINLIB_EXPORT_CLASS(drc_flexnav_hardware::ImuLpmsCu3HardwareComponent,
                       hardware_interface::SensorInterface)
