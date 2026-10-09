// Copyright (c) 2024, ISI SIM/DALCA
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef DRC_FLEXNAV_CONTROLLERS__MRAC_CONTROLLER_HPP_
#define DRC_FLEXNAV_CONTROLLERS__MRAC_CONTROLLER_HPP_

#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "drc_flexnav_controllers/utils.hpp"
#include "drc_flexnav_controllers/visibility_control.h"
#include "drc_flexnav_controllers_msgs/msg/mrac_controller_status.hpp"
#include "curtis_acf2a_can_driver/canopen_interface.hpp"

namespace drc_flexnav_controllers
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class MRACController : public controller_interface::ControllerInterface
  {
  public:
    MRACController(void);

    DRC_FLEXNAV_CONTROLLERS__VISIBILITY_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration(void) const override;

    DRC_FLEXNAV_CONTROLLERS__VISIBILITY_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration(void) const override;

    DRC_FLEXNAV_CONTROLLERS__VISIBILITY_PUBLIC
    CallbackReturn on_init(void) override;

    DRC_FLEXNAV_CONTROLLERS__VISIBILITY_PUBLIC
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    DRC_FLEXNAV_CONTROLLERS__VISIBILITY_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    DRC_FLEXNAV_CONTROLLERS__VISIBILITY_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    DRC_FLEXNAV_CONTROLLERS__VISIBILITY_PUBLIC
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    std::vector<std::string> state_joints_, command_joints_;

    std::shared_ptr<rclcpp::Publisher<drc_flexnav_controllers_msgs::msg::MRACControllerStatus>> status_publisher_;
    std::shared_ptr<realtime_tools::RealtimePublisher<drc_flexnav_controllers_msgs::msg::MRACControllerStatus>> rt_status_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_command_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_is_emergency_lock_;

    std::vector<double> effort_limit_;
    double linear_velocity_limit_;
    double angular_velocity_limit_;
    rclcpp::Time lastSamplingTime_;

    Eigen::Matrix2d Ginv_, initialGinv_;
    Eigen::Matrix<double, 2, 4> F_, initialF_;

    bool use_odom_topic_;
    std::string odom_topic_, odometer_name_;
    std::vector<std::string> odometer_interfaces_;
    std::string nmt_interface_name_, motor_configured_interface_name_;

    std::string is_emergency_lock_topic_name_;

    bool check_motor_;
    bool is_locked_;

    Eigen::Matrix<double, 2, 1> vel_ref_;
    Eigen::Matrix<double, 2, 1> nu_ref_;
    Eigen::Vector2d u_;
    Eigen::Vector2d torque_;

    Eigen::Vector2d uModel_;
    Eigen::DiagonalMatrix<double, 2> alpha_;

    bool adaptive_;
    Eigen::Matrix<double, 2, 6> Wf_;
    Eigen::DiagonalMatrix<double, 2> epsilon_;
    Eigen::Vector<double, 6> thetaEst_;
    Eigen::Vector2d FWthetaEst_;
    Eigen::Vector2d ef_, e1_;
    Eigen::Matrix<double, 6, 6> P_, initialP_;

    int priority_;

    realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> rt_command_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<nav_msgs::msg::Odometry>> rt_odom_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Bool>> rt_is_emergency_lock_;

    void commandCB(const geometry_msgs::msg::Twist::SharedPtr command);
    void odomCB(const nav_msgs::msg::Odometry::SharedPtr odom);
    void isEmergencyLockCB(const std_msgs::msg::Bool::SharedPtr is_emergency_lock);

    void logStatus(const rclcpp::Time& time, const rclcpp::Duration& dt);

    void reset();
    void resetParams();
  };
} // namespace drc_flexnav_controllers

#endif // DRC_FLEXNAV_CONTROLLERS__MRAC_CONTROLLER_HPP_
