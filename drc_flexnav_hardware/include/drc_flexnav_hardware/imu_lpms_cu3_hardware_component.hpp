#ifndef DRC_FLEXNAV_HARDWARE__IMU_LPMS_CU3_HARDWARE_COMPONENT_HPP_
#define DRC_FLEXNAV_HARDWARE__IMU_LPMS_CU3_HARDWARE_COMPONENT_HPP_

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "drc_flexnav_hardware/visbility_control.h"
#include "imu_lpms_cu3/imu_lpms_cu3_sensor.hpp"
#include <condition_variable>

namespace drc_flexnav_hardware {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ImuLpmsCu3HardwareComponent : public hardware_interface::SensorInterface {
public:
  ImuLpmsCu3HardwareComponent();

  DRC_FLEXNAV_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  DRC_FLEXNAV_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DRC_FLEXNAV_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  struct imu_state_interfaces {
    double orientation[4];        // Quaternion w,x,y,z
    double angular_velocity[3];   // x,y,z
    double linear_acceleration[3];// x,y,z
    double magnetic_field[3];     // x,y,z
  } imu_states_;

  // Thread control
  std::atomic<bool> stop_thread_{false};
  bool pause_thread_{false};
  bool thread_started_{false};
  std::thread imu_read_thread_;
  std::mutex mutex_;
  std::condition_variable cv_;

  drc_flexnav_sensors::Inertial_measurement_unit_and_compass imu_interface_;

  // Configuration parameters
  std::string imu_port_;
  int imu_id_;
  int update_rate_;
  bool double_precision_;
  int serial_baudrate_;

  DRC_FLEXNAV_HARDWARE_PUBLIC
  void start_resume_imu_thread();

  DRC_FLEXNAV_HARDWARE_PUBLIC
  void pause_imu_thread();

  DRC_FLEXNAV_HARDWARE_PUBLIC
  void stop_imu_thread();

  DRC_FLEXNAV_HARDWARE_PUBLIC
  void imu_read_loop();

  // Utility methods
  void reset_state_interfaces();
  bool configure_imu();
};

} // namespace drc_flexnav_hardware

#endif // DRC_FLEXNAV_HARDWARE__IMU_LPMS_CU3_HARDWARE_COMPONENT_HPP_
