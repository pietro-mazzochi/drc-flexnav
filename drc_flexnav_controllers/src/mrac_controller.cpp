// Copyright (c) 2024, ISI SIM/DALCA
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <cmath>
#include <sys/mman.h>
#include <algorithm>
#include <array>
#include <vector>
#include <type_traits>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/logging.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "drc_flexnav_controllers/mrac_controller.hpp"

using std::placeholders::_1;

#define sqr(x) ((x) * (x))

namespace drc_flexnav_controllers
{
  MRACController::MRACController(void)
    : is_locked_(true)
  {}

  CallbackReturn MRACController::on_init(void)
  {
    try
    {
      auto_declare<std::vector<std::string>>("command_joints", command_joints_);
      odometer_name_ = auto_declare<std::string>("odometer_name", "odometer");
      odometer_interfaces_ = auto_declare<std::vector<std::string>>(
        "odometer_interfaces", std::vector<std::string>{});
      odom_topic_ = auto_declare<std::string>("odom_topic", "/odom");
      use_odom_topic_ = auto_declare<bool>("use_odom_topic", false);

      check_motor_ = auto_declare<bool>("check_motor", true);

      if (check_motor_) {
        is_emergency_lock_topic_name_ = auto_declare<std::string>("is_emergency_lock_topic_name", "/is_emergency_lock");
        nmt_interface_name_ = auto_declare<std::string>("nmt_interface_name", "nmt_state");
        motor_configured_interface_name_ = auto_declare<std::string>("motor_configured_interface_name", "motor_configured");
      }

      auto_declare<std::string>("odom_frame_id", "odom");
      auto_declare<std::string>("base_frame_id", "base_link");
      auto_declare<double>("wheel_to_robot_center", 0.0);
      auto_declare<std::vector<double>>("wheel_radius", std::vector<double>());

      auto_declare<std::vector<double>>("model_parameters", std::vector<double>());
      auto_declare<std::vector<double>>("parameters_covariance", std::vector<double>());
      auto_declare<std::vector<double>>("alpha", std::vector<double>());
      auto_declare<std::vector<double>>("epsilon", std::vector<double>());
      auto_declare<bool>("adaptive", false);

      auto_declare<int>("priority", sched_get_priority_max(SCHED_FIFO));
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception thrown in on_init() with message: " << e.what());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MRACController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    try
    {
      command_joints_ = get_node()->get_parameter("command_joints").as_string_array();
      if (command_joints_.empty())
      {
        throw std::runtime_error("'command_joints' parameter was empty,");
      }
      std::array<std::string, 3> odometer_mandatory_interfaces = {"twist.linear.x", "twist.linear.y", "twist.angular.z"};
      if ( !(use_odom_topic_ || contains_subcontainer(odometer_interfaces_, odometer_mandatory_interfaces)) ) {
        RCLCPP_ERROR_STREAM(get_node()->get_logger(), "When using odometer, the following interfaces are mandatory: " << odometer_mandatory_interfaces);
        return CallbackReturn::ERROR;
      }

      auto odom_frame_id = get_node()->get_parameter("odom_frame_id").as_string();
      auto base_frame_id = get_node()->get_parameter("base_frame_id").as_string();

      status_publisher_ = get_node()->create_publisher<drc_flexnav_controllers_msgs::msg::MRACControllerStatus>("~/status", rclcpp::SystemDefaultsQoS());
      rt_status_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<drc_flexnav_controllers_msgs::msg::MRACControllerStatus>>(status_publisher_);
      rt_status_publisher_->msg_.header.frame_id = base_frame_id;
      rt_status_publisher_->msg_.accel_set_point.linear.x = 0.0;
      rt_status_publisher_->msg_.accel_set_point.linear.y = 0.0;
      rt_status_publisher_->msg_.accel_set_point.linear.z = 0.0;
      rt_status_publisher_->msg_.accel_set_point.angular.x = 0.0;
      rt_status_publisher_->msg_.accel_set_point.angular.y = 0.0;
      rt_status_publisher_->msg_.accel_set_point.angular.z = 0.0;
      rt_status_publisher_->msg_.velocity.linear.x = 0.0;
      rt_status_publisher_->msg_.velocity.linear.y = 0.0;
      rt_status_publisher_->msg_.velocity.linear.z = 0.0;
      rt_status_publisher_->msg_.velocity.angular.x = 0.0;
      rt_status_publisher_->msg_.velocity.angular.y = 0.0;
      rt_status_publisher_->msg_.velocity.angular.z = 0.0;
      rt_status_publisher_->msg_.velocity_set_point.linear.x = 0.0;
      rt_status_publisher_->msg_.velocity_set_point.linear.y = 0.0;
      rt_status_publisher_->msg_.velocity_set_point.linear.z = 0.0;
      rt_status_publisher_->msg_.velocity_set_point.angular.x = 0.0;
      rt_status_publisher_->msg_.velocity_set_point.angular.y = 0.0;
      rt_status_publisher_->msg_.velocity_set_point.angular.z = 0.0;

      sub_command_ = get_node()->create_subscription<geometry_msgs::msg::Twist>("~/command", 1,
        std::bind(&MRACController::commandCB, this, _1));

      sub_odom_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 1,
        std::bind(&MRACController::odomCB, this, _1));

      if (check_motor_) {
        sub_is_emergency_lock_ = get_node()->create_subscription<std_msgs::msg::Bool>(is_emergency_lock_topic_name_, 1,
          std::bind(&MRACController::isEmergencyLockCB, this, _1));
      }

      if (!get_node()->get_parameter("effort_limit", effort_limit_))
      {
        throw std::runtime_error("No 'effort_limit' parameter.");
      }
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "Joint effort limits [min | max]: " << effort_limit_[0] << " Nm | " << effort_limit_[1] << " Nm");

      // model_parameters = [kv kw k5 k6 k7 k8]
      // F = [kv 0  0  k5
      //      0  kw k6 0 ]
      // Ginv = 0.5*[1/k7  1/k8
      //             1/k7 -1/k8]
      std::vector<double> model_parameters;
      if (!get_node()->get_parameter("model_parameters", model_parameters))
      {
        throw std::runtime_error("No 'model_parameters' parameter.");
      }
      initialF_ << model_parameters[0], 0, 0, model_parameters[2],
                   0, model_parameters[1], model_parameters[3], 0;
      initialGinv_ << 0.5/model_parameters[4],  0.5/model_parameters[5],
                      0.5/model_parameters[4], -0.5/model_parameters[5];

      std::vector<double> alphaVec;
      if (!get_node()->get_parameter("alpha", alphaVec))
        throw std::runtime_error("No 'alpha' parameter.");
      alpha_.diagonal() = Eigen::Map<Eigen::Vector2d>(alphaVec.data(), 2);

      adaptive_ = get_node()->get_parameter("adaptive").as_bool();

      if (adaptive_)
      {
        std::vector<double> covarianceVec;
        if (!get_node()->get_parameter("parameters_covariance", covarianceVec))
          throw std::runtime_error("No 'parameters_covariance' parameter.");
        initialP_.diagonal() = Eigen::Map<Eigen::Vector<double, 6>>(covarianceVec.data(), 6);

        std::vector<double> epsilonVec;
        if (!get_node()->get_parameter("epsilon", epsilonVec))
          throw std::runtime_error("No 'epsilon' parameter.");
        epsilon_.diagonal() = Eigen::Map<Eigen::Vector2d>(epsilonVec.data(), 2);
      }

      if (!get_node()->get_parameter("linear_velocity_limit", linear_velocity_limit_))
      {
        throw std::runtime_error("No 'linear_velocity_limit' parameter.");
      }
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "Linear velocity limit: " << linear_velocity_limit_ << " m/s");

      if (!get_node()->get_parameter("angular_velocity_limit", angular_velocity_limit_))
      {
        throw std::runtime_error("No 'angular_velocity_limit' parameter.");
      }
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "Angular velocity limit: " << angular_velocity_limit_ << " rad/s");
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception thrown in on_configure(): " << e.what());
      return CallbackReturn::ERROR;
    }

    if (!get_node()->get_parameter("priority", priority_))
    {
      RCLCPP_WARN(get_node()->get_logger(), "No 'priority' configured for controller. Using highest possible priority.");
    }

    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration MRACController::command_interface_configuration(void) const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& joint : command_joints_)
    {
      config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    }

    return config;
  }

  controller_interface::InterfaceConfiguration MRACController::state_interface_configuration(void) const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    if(!use_odom_topic_){
      for (const auto& state_interface : odometer_interfaces_) {
        config.names.push_back(odometer_name_ + "/" + state_interface);
      }
    }

    if (check_motor_) {
      for (const auto& joint : command_joints_){
        config.names.push_back(joint + "/" + nmt_interface_name_);
      }
      for (const auto& joint : command_joints_){
        config.names.push_back(joint + "/" + motor_configured_interface_name_);
      }
    }

    return config;
  }

  CallbackReturn MRACController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    nu_ref_.setZero();
    vel_ref_.setZero();
    u_.setZero();
    uModel_.setZero();
    torque_.setZero();
    lastSamplingTime_ = get_node()->get_clock()->now();

    F_ = initialF_;
    Ginv_ = initialGinv_;

    if (adaptive_)
    {
      // thetaEst = [kv kw k5 k6 k7 k8]^T
      // F = [kv 0  0  k5
      //      0  kw k6 0 ]
      // Ginv = 0.5*[1/k7  1/k8
      //             1/k7 -1/k8]
      thetaEst_ << F_(0,0), F_(1,1), F_(0,3), F_(1,2), 0.5/Ginv_(0,0), 0.5/Ginv_(0,1);
      P_ = initialP_;

      Wf_.setZero();
      FWthetaEst_.setZero();
      ef_.setZero();
      e1_.setZero();
    }

    struct sched_param param;
    param.sched_priority = priority_;

    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to set real-time scheduler.");
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to lock memory.");
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MRACController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    for (unsigned int i = 0; i < command_joints_.size(); i++)
    {
      command_interfaces_[i].set_value(0);
    }

    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type MRACController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
  {
    rclcpp::Time time_ = get_node()->get_clock()->now();
    auto dt_ = (time_ - lastSamplingTime_);

    if (dt_.seconds() < std::numeric_limits<double>::epsilon())
      return controller_interface::return_type::OK;

    lastSamplingTime_ = time_;
    torque_.setZero();

    if (check_motor_) {
      std::vector<double> motor_state_values, motor_configured_values;
      std::vector<nmt_state> motor_states;
      std::vector<bool> motor_configured;
      motor_state_values.reserve(command_joints_.size());
      motor_states.reserve(command_joints_.size());
      motor_configured_values.reserve(command_joints_.size());
      motor_configured.reserve(command_joints_.size());

      try {
        for (unsigned int i = 0; i < command_joints_.size(); i++){
          motor_state_values[i] = return_state_if_exists(motor_state_values[i], command_joints_[i] + "/" + nmt_interface_name_, state_interfaces_);
          motor_states[i] = static_cast<nmt_state>(static_cast<std::underlying_type_t<nmt_state>>(std::round(motor_state_values[i])));

          motor_configured_values[i] = return_state_if_exists(motor_configured_values[i], command_joints_[i] + "/" + motor_configured_interface_name_, state_interfaces_);
          motor_configured[i] = static_cast<bool>(std::round(motor_configured_values[i]));
        }
      } catch (const std::exception& e){
        RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception thrown in update() with message: " << e.what());
        return controller_interface::return_type::ERROR;
      }

      auto emergency = rt_is_emergency_lock_.readFromRT();
      if (emergency && *emergency){
        is_locked_ = (*emergency)->data;
      }
      for (unsigned int i = 0; i < command_joints_.size(); i++){
        is_locked_ = is_locked_ || (motor_states[i] != nmt_state::OPERATIONAL) || !motor_configured[i];
      }
    }
    else {
      is_locked_ = false;
    }

    if (is_locked_) {
      reset();
      // resetParams();
      for (unsigned int i = 0; i < command_joints_.size(); i++){
        command_interfaces_[i].set_value(0);
      }
      logStatus(time_, dt_);
      return controller_interface::return_type::OK;
    }

    //Get odometry
    if (use_odom_topic_) {
      auto odom = rt_odom_.readFromRT();
      if (odom && *odom){
        const double vx = (*odom)->twist.twist.linear.x;
        const double vy = (*odom)->twist.twist.linear.y;
        const double w  = (*odom)->twist.twist.angular.z;

        tf2::Quaternion tf2Orientation;
        const auto orientation = (*odom)->pose.pose.orientation;
        tf2::fromMsg(orientation, tf2Orientation);

        u_[0] = getLongitudinalVelocity(vx, vy, tf2::getYaw(tf2Orientation));
        u_[1] = w;
      }
    } else {
      double vx, vy, w;
      double qx, qy, qz, qw;
      vx = return_state_if_exists(vx, odometer_name_ + "/twist.linear.x"    , state_interfaces_);
      vy = return_state_if_exists(vy, odometer_name_ + "/twist.linear.y"    , state_interfaces_);
      w  = return_state_if_exists(w , odometer_name_ + "/twist.angular.z"   , state_interfaces_);
      qx = return_state_if_exists(qx, odometer_name_ + "/pose.orientation.x", state_interfaces_);
      qy = return_state_if_exists(qy, odometer_name_ + "/pose.orientation.y", state_interfaces_);
      qz = return_state_if_exists(qz, odometer_name_ + "/pose.orientation.z", state_interfaces_);
      qw = return_state_if_exists(qw, odometer_name_ + "/pose.orientation.w", state_interfaces_);

      u_[0] = getLongitudinalVelocity(vx, vy, tf2::getYaw(tf2::Quaternion(qx, qy, qz, qw)));
      u_[1] = w;
    }

    auto command = rt_command_.readFromRT();
    if (command && *command)
    {
      vel_ref_[0] = (*command)->linear.x;
      vel_ref_[1] = (*command)->angular.z;
    }

    vel_ref_[0] = std::clamp(vel_ref_[0], -linear_velocity_limit_, linear_velocity_limit_);
    vel_ref_[1] = std::clamp(vel_ref_[1], -angular_velocity_limit_, angular_velocity_limit_);

    if (adaptive_)
    {
      // thetaEst = [kv kw k5 k6 k7 k8]^T
      // F = [kv 0  0  k5
      //      0  kw k6 0 ]
      // Ginv = 0.5*[1/k7  1/k8
      //             1/k7 -1/k8]
      F_(0,0) = thetaEst_[0];
      F_(1,1) = thetaEst_[1];
      F_(0,3) = thetaEst_[2];
      F_(1,2) = thetaEst_[3];
      Ginv_(0,0) =  0.5/thetaEst_[4];
      Ginv_(1,0) =  0.5/thetaEst_[4];
      Ginv_(0,1) =  0.5/thetaEst_[5];
      Ginv_(1,1) = -0.5/thetaEst_[5];
    }

    // Compute reference model
    auto duModel = alpha_ * (vel_ref_ - uModel_);
    nu_ref_ = alpha_ * (uModel_ - u_) + duModel;

    // uf = [v, w, v*w, w^2]
    Eigen::Vector4d uf(u_[0], u_[1], u_[0]*u_[1], sqr(u_[1]));
    auto tau_ = Ginv_ * (nu_ref_ - F_ * uf);

    // Control action saturation + un-swap right-left wheels torque
    torque_ << std::clamp(tau_[1], effort_limit_[0], effort_limit_[1]),
               std::clamp(tau_[0], effort_limit_[0], effort_limit_[1]);

    // Apply torques
    for (unsigned int i = 0; i < command_joints_.size(); i++)
      command_interfaces_[i].set_value(torque_[i]);

    if (adaptive_)
    { // Parameter estimation
      Eigen::Matrix<double, 2, 6> W;
      W << u_[0], 0, sqr(u_[1]), 0, torque_[1] + torque_[0], 0,
           0, u_[1], 0, u_[0]*u_[1], 0, torque_[1] - torque_[0];

      auto dWf = epsilon_.inverse() * (W - Wf_);

      auto dFWthetaEst = epsilon_.inverse() * (W * thetaEst_ - FWthetaEst_);

      auto e = u_ - uModel_;

      auto def = epsilon_.inverse() * (e - ef_);

      auto WfPhi = def + alpha_ * ef_ + FWthetaEst_ - Wf_ * thetaEst_;

      auto de1 = WfPhi - alpha_ * e1_;

      auto dthetaEst = 0.5 * P_ * Wf_.transpose() * WfPhi + 0.5 * P_ * Wf_.transpose() * alpha_ * e1_;

      auto dP = - P_ * Wf_.transpose() * Wf_ * P_;

      // Update filters
      Wf_         += dWf * dt_.seconds();
      FWthetaEst_ += dFWthetaEst * dt_.seconds();
      ef_         += def * dt_.seconds();
      e1_         += de1 * dt_.seconds();
      thetaEst_   += dthetaEst * dt_.seconds();
      P_          += dP * dt_.seconds();

      // Prevent catastrophic numerical failure
      if (
        isinf(Wf_.array()).any()         || isnan(Wf_.array()).any()         ||
        isinf(FWthetaEst_.array()).any() || isnan(FWthetaEst_.array()).any() ||
        isinf(ef_.array()).any()         || isnan(ef_.array()).any()         ||
        isinf(e1_.array()).any()         || isnan(e1_.array()).any()         ||
        isinf(thetaEst_.array()).any()   || isnan(thetaEst_.array()).any()   ||
        isinf(P_.array()).any()          || isnan(P_.array()).any()          || (P_.diagonal().array() < 0).any()
      )
      {
        RCLCPP_WARN(get_node()->get_logger(), "NaN or Inf found in MRAC adaptation. Resetting controller.");
        Wf_.setZero();
        FWthetaEst_.setZero();
        ef_.setZero();
        e1_.setZero();
        resetParams();
      }
    }

    // update reference model
    uModel_ += duModel * dt_.seconds();

    logStatus(time_, dt_);
    return controller_interface::return_type::OK;
  }

  void MRACController::commandCB(const geometry_msgs::msg::Twist::SharedPtr command)
  {
    rt_command_.writeFromNonRT(command);
  }
  void MRACController::odomCB(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    rt_odom_.writeFromNonRT(odom);
  }
  void MRACController::isEmergencyLockCB(const std_msgs::msg::Bool::SharedPtr is_emergency_lock)
  {
    rt_is_emergency_lock_.writeFromNonRT(is_emergency_lock);
  }

  void MRACController::logStatus(const rclcpp::Time& time, const rclcpp::Duration& dt)
  {
    if (rt_status_publisher_ && rt_status_publisher_->trylock())
    {
      rt_status_publisher_->msg_.header.stamp = time;

      rt_status_publisher_->msg_.accel_set_point.linear.x = nu_ref_[0];
      rt_status_publisher_->msg_.accel_set_point.angular.z = nu_ref_[1];

      rt_status_publisher_->msg_.velocity.linear.x = u_[0];
      rt_status_publisher_->msg_.velocity.angular.z = u_[1];

      rt_status_publisher_->msg_.velocity_set_point.linear.x = vel_ref_[0];
      rt_status_publisher_->msg_.velocity_set_point.angular.z = vel_ref_[1];

      rt_status_publisher_->msg_.velocity_error[0] = vel_ref_[0] - u_[0];
      rt_status_publisher_->msg_.velocity_error[1] = vel_ref_[1] - u_[1];

      rt_status_publisher_->msg_.velocity_model[0] = uModel_[0];
      rt_status_publisher_->msg_.velocity_model[1] = uModel_[1];

      rt_status_publisher_->msg_.time_step_ms = static_cast<double>(1000 * dt.seconds());

      for (unsigned int i = 0; i < torque_.size(); i++)
      {
        rt_status_publisher_->msg_.command[i] = torque_[i];
      }

      for (unsigned int i = 0; i < thetaEst_.size(); i++)
      {
        rt_status_publisher_->msg_.parameters[i] = thetaEst_[i];
        rt_status_publisher_->msg_.covariance[i] = P_(i,i);
      }

      rt_status_publisher_->msg_.locked = is_locked_;

      rt_status_publisher_->unlockAndPublish();
    }
  }

  void MRACController::reset()
  {
    nu_ref_.setZero();
    vel_ref_.setZero();
    u_.setZero();
    uModel_.setZero();
    torque_.setZero();
    lastSamplingTime_ = get_node()->get_clock()->now();

    if (adaptive_)
    {
      Wf_.setZero();
      FWthetaEst_.setZero();
      ef_.setZero();
      e1_.setZero();
    }
  }

  void MRACController::resetParams()
  {
    // thetaEst = [kv kw k5 k6 k7 k8]^T
    // F = [kv 0  0  k5
    //      0  kw k6 0 ]
    // Ginv = 0.5*[1/k7  1/k8
    //             1/k7 -1/k8]
    F_ = initialF_;
    Ginv_ = initialGinv_;

    thetaEst_ << F_(0,0), F_(1,1), F_(0,3), F_(1,2), 0.5/Ginv_(0,0), 0.5/Ginv_(0,1);
    P_ = initialP_;
  }
}

PLUGINLIB_EXPORT_CLASS(drc_flexnav_controllers::MRACController, controller_interface::ControllerInterface)
