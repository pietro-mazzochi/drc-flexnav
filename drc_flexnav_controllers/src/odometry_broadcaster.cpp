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

#include "drc_flexnav_controllers/odometry_broadcaster.hpp"

using std::placeholders::_1;

namespace drc_flexnav_controllers {
OdometryBroadcaster::OdometryBroadcaster(void) {}

CallbackReturn OdometryBroadcaster::on_init(void) {
  try {
    sensor_name_ = auto_declare<std::string>("sensor_name", "odometer");
    sensor_interfaces_ = auto_declare<std::vector<std::string>>(
        "sensor_interfaces", std::vector<std::string>{});
    odom_topic_ = auto_declare<std::string>("odom_topic", "/odom");
    odom_frame_id_ = auto_declare<std::string>("odom_frame_id", "odom");
    base_frame_id_ = auto_declare<std::string>("base_frame_id", "base");
    tf_topic_ = auto_declare<std::string>("tf_topic", "/tf");
    publish_odom_to_tf_ = auto_declare<bool>("publish_odom_to_tf", true);
    gpio_name_ = auto_declare<std::string>("gpio_name", "siemens_et200sp_gpio");
    gpio_state_interfaces_ = auto_declare<std::vector<std::string>>(
        "gpio_state_interfaces", std::vector<std::string>{});
    use_siemens_time_stamp_ =
        auto_declare<bool>("use_siemens_time_stamp", false);
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(
        get_node()->get_logger(),
        "Exception thrown in on_init() with message: " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn OdometryBroadcaster::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  try {

    odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SystemDefaultsQoS());
    rt_odom_pub_ = std::make_shared<
        realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_);
    rt_odom_pub_->msg_.header.frame_id = odom_frame_id_;
    rt_odom_pub_->msg_.child_frame_id = base_frame_id_;
    rt_odom_pub_->msg_.pose.pose.position.z = 0;
    rt_odom_pub_->msg_.pose.pose.orientation.x = 0;
    rt_odom_pub_->msg_.pose.pose.orientation.y = 0;

    // Fake covariance
    double pose_cov[] = {1e-6, 1e-6, 1e-16, 1e-16, 1e-16, 1e-9};
    rt_odom_pub_->msg_.pose.covariance = {
        pose_cov[0], 0, 0, 0, 0, 0, 0, pose_cov[1], 0, 0, 0, 0, 0, 0,
        pose_cov[2], 0, 0, 0, 0, 0, 0, pose_cov[3], 0, 0, 0, 0, 0, 0,
        pose_cov[4], 0, 0, 0, 0, 0, 0, pose_cov[5]};

    rt_odom_pub_->msg_.twist.twist.linear.z = 0;
    rt_odom_pub_->msg_.twist.twist.angular.x = 0;
    rt_odom_pub_->msg_.twist.twist.angular.y = 0;

    // Fake covariance
    double twist_cov[] = {1e-6, 1e-6, 1e-16, 1e-16, 1e-16, 1e-9};
    rt_odom_pub_->msg_.twist.covariance = {
        twist_cov[0], 0, 0, 0, 0, 0, 0, twist_cov[1], 0, 0, 0, 0, 0, 0,
        twist_cov[2], 0, 0, 0, 0, 0, 0, twist_cov[3], 0, 0, 0, 0, 0, 0,
        twist_cov[4], 0, 0, 0, 0, 0, 0, twist_cov[5]};
    if (publish_odom_to_tf_) {

      tf_odom_pub_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
          tf_topic_, rclcpp::SystemDefaultsQoS());
      rt_tf_odom_pub_ = std::make_shared<
          realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
          tf_odom_pub_);
      rt_tf_odom_pub_->msg_.transforms.resize(1);
      rt_tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
      rt_tf_odom_pub_->msg_.transforms[0].transform.rotation.x = 0.0;
      rt_tf_odom_pub_->msg_.transforms[0].transform.rotation.y = 0.0;
      rt_tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
      rt_tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Exception thrown in on_configure(): " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
OdometryBroadcaster::command_interface_configuration(void) const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  return config;
}

controller_interface::InterfaceConfiguration
OdometryBroadcaster::state_interface_configuration(void) const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto &state_interface : sensor_interfaces_) {
    config.names.push_back(sensor_name_ + "/" + state_interface);
  }
  for (const auto &state_interface : gpio_state_interfaces_) {
    config.names.push_back(gpio_name_ + "/" + state_interface);
  }

  return config;
}

CallbackReturn OdometryBroadcaster::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}
CallbackReturn OdometryBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
OdometryBroadcaster::update(const rclcpp::Time &time,
                            const rclcpp::Duration & /*period*/) {

  if (rt_odom_pub_ && rt_odom_pub_->trylock()) {
    rt_odom_pub_->msg_.header.stamp = time;

    rt_odom_pub_->msg_.pose.pose.position.x = return_state_if_exists(
        rt_odom_pub_->msg_.pose.pose.position.x,
        sensor_name_ + "/pose.position.x", state_interfaces_);
    rt_odom_pub_->msg_.pose.pose.position.y = return_state_if_exists(
        rt_odom_pub_->msg_.pose.pose.position.y,
        sensor_name_ + "/pose.position.y", state_interfaces_);
    rt_odom_pub_->msg_.pose.pose.orientation.z = return_state_if_exists(
        rt_odom_pub_->msg_.pose.pose.orientation.z,
        sensor_name_ + "/pose.orientation.z", state_interfaces_);
    rt_odom_pub_->msg_.pose.pose.orientation.w = return_state_if_exists(
        rt_odom_pub_->msg_.pose.pose.orientation.w,
        sensor_name_ + "/pose.orientation.w", state_interfaces_);

    rt_odom_pub_->msg_.twist.twist.linear.x = return_state_if_exists(
        rt_odom_pub_->msg_.twist.twist.linear.x,
        sensor_name_ + "/twist.linear.x", state_interfaces_);
    rt_odom_pub_->msg_.twist.twist.linear.y = return_state_if_exists(
        rt_odom_pub_->msg_.twist.twist.linear.y,
        sensor_name_ + "/twist.linear.y", state_interfaces_);
    rt_odom_pub_->msg_.twist.twist.angular.z = return_state_if_exists(
        rt_odom_pub_->msg_.twist.twist.angular.z,
        sensor_name_ + "/twist.angular.z", state_interfaces_);

    rt_odom_pub_->unlockAndPublish();
  }

  if (publish_odom_to_tf_) {
    if (rt_tf_odom_pub_ && rt_tf_odom_pub_->trylock()) {
      geometry_msgs::msg::TransformStamped &odom_frame =
          rt_tf_odom_pub_->msg_.transforms[0];

      odom_frame.header.stamp = time;
      if (use_siemens_time_stamp_) {
        for (const auto &interface : state_interfaces_) {
          if (interface.get_name() == (gpio_name_ + "/time_stamp")) {
            odom_frame.header.stamp = rclcpp::Time(interface.get_value());
          }
        }
      }
      odom_frame.transform.translation.x = return_state_if_exists(
          odom_frame.transform.translation.x, sensor_name_ + "/pose.position.x",
          state_interfaces_);
      odom_frame.transform.translation.y = return_state_if_exists(
          odom_frame.transform.translation.x, sensor_name_ + "/pose.position.y",
          state_interfaces_);
      odom_frame.transform.rotation.z = return_state_if_exists(
          odom_frame.transform.rotation.z, sensor_name_ + "/pose.orientation.z",
          state_interfaces_);
      odom_frame.transform.rotation.w = return_state_if_exists(
          odom_frame.transform.rotation.z, sensor_name_ + "/pose.orientation.w",
          state_interfaces_);

      rt_tf_odom_pub_->unlockAndPublish();
    }
  }

  return controller_interface::return_type::OK;
}

} // namespace drc_flexnav_controllers

PLUGINLIB_EXPORT_CLASS(drc_flexnav_controllers::OdometryBroadcaster,
                       controller_interface::ControllerInterface)
