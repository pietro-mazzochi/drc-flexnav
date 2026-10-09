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
#include <rclcpp/logging.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <type_traits>
#include <cmath>

#include "drc_flexnav_controllers/dynamics_controller.hpp"

using std::placeholders::_1;

namespace drc_flexnav_controllers
{
  DynamicsController::DynamicsController(void)
    : is_locked_(true)
  {}

  CallbackReturn DynamicsController::on_init(void)
  {
    try
    {
      auto_declare<std::vector<std::string>>("command_joints", command_joints_);
      odometer_name_ = auto_declare<std::string>("odometer_name", "odometer");
      odometer_interfaces_ = auto_declare<std::vector<std::string>>(
        "odometer_interfaces", std::vector<std::string>{});
      odom_topic_ = auto_declare<std::string>("odom_topic", "/odom");
      use_odom_topic_ = auto_declare<bool>("use_odom_topic", false);
      is_emergency_lock_topic_name_ = auto_declare<std::string>("is_emergency_lock_topic_name", "/is_emergency_lock");
      nmt_interface_name_ = auto_declare<std::string>("nmt_interface_name", "nmt_state");
      motor_configured_interface_name_ = auto_declare<std::string>("motor_configured_interface_name", "motor_configured");

      auto_declare<std::string>("base_frame_id", "base_link");

      auto_declare<std::vector<double>>("model_parameters", std::vector<double>());

      auto_declare<std::vector<double>>("linear_vel_pid", std::vector<double>());
      auto_declare<std::vector<double>>("angular_vel_pid", std::vector<double>());

      auto_declare<int>("priority", sched_get_priority_max(SCHED_FIFO));
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception thrown in on_init() with message: " << e.what());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn DynamicsController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
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

      auto base_frame_id = get_node()->get_parameter("base_frame_id").as_string();

      status_publisher_ = get_node()->create_publisher<drc_flexnav_controllers_msgs::msg::DynamicsControllerStatus>("~/status", rclcpp::SystemDefaultsQoS());
      rt_status_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<drc_flexnav_controllers_msgs::msg::DynamicsControllerStatus>>(status_publisher_);
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
                                            std::bind(&DynamicsController::commandCB, this, _1));

      sub_odom_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 1,
                                            std::bind(&DynamicsController::odomCB, this, _1));
      sub_is_emergency_lock_ = get_node()->create_subscription<std_msgs::msg::Bool>(is_emergency_lock_topic_name_, 1,
                                            std::bind(&DynamicsController::isEmergencyLockCB, this, _1));

      if (!get_node()->get_parameter("effort_limit", effort_limit_))
      {
        throw std::runtime_error("No 'effort_limit' parameter.");
      }
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "Joint effort limits [min | max]: " << effort_limit_[0] << " Nm | " << effort_limit_[1] << " Nm");

      if (!get_node()->get_parameter("model_parameters", model_parameters_))
      {
        throw std::runtime_error("No 'model_parameters' parameter.");
      }
      ModelParametersVector param_vec = Eigen::Map<ModelParametersVector>(model_parameters_.data(),
                                        MODEL_PARAMETER_INDEX_SIZE, 1);
      linearizing_controller_ = std::make_shared<StateFeedbackLinearization>(param_vec);

      std::vector<double> linear_vel_pid_params_vector_;
      if (!get_node()->get_parameter("linear_vel_pid", linear_vel_pid_params_vector_))
      {
        throw std::runtime_error("No 'linear_vel_pid' parameter.");
      }

      std::vector<double> angular_vel_pid_params_vector_;
      if (!get_node()->get_parameter("angular_vel_pid", angular_vel_pid_params_vector_))
      {
        throw std::runtime_error("No 'angular_vel_pid' parameter.");
      }

      linear_vel_pid_ = std::make_unique<PIDControllerType>(linear_vel_pid_params_vector_[ANALOG_PID_KP_INDEX],
                                  linear_vel_pid_params_vector_[ANALOG_PID_KI_INDEX],
                                  linear_vel_pid_params_vector_[ANALOG_PID_KD_INDEX],
                                  linear_vel_pid_params_vector_[ANALOG_PID_UMIN_INDEX],
                                  linear_vel_pid_params_vector_[ANALOG_PID_UMAX_INDEX],
                                  linear_vel_pid_params_vector_[ANALOG_PID_TAU_INDEX],
                                  "Linear Velocity PID");
      angular_vel_pid_ = std::make_unique<PIDControllerType>(angular_vel_pid_params_vector_[ANALOG_PID_KP_INDEX],
                                   angular_vel_pid_params_vector_[ANALOG_PID_KI_INDEX],
                                   angular_vel_pid_params_vector_[ANALOG_PID_KD_INDEX],
                                   angular_vel_pid_params_vector_[ANALOG_PID_UMIN_INDEX],
                                   angular_vel_pid_params_vector_[ANALOG_PID_UMAX_INDEX],
                                   angular_vel_pid_params_vector_[ANALOG_PID_TAU_INDEX],
                                   "Angular Velocity PID");

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

  controller_interface::InterfaceConfiguration DynamicsController::command_interface_configuration(void) const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& joint : command_joints_){
      config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    }

    return config;
  }

  controller_interface::InterfaceConfiguration DynamicsController::state_interface_configuration(void) const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    if(!use_odom_topic_){
      for (const auto& state_interface : odometer_interfaces_) {
        config.names.push_back(odometer_name_ + "/" + state_interface);
      }
    }
    for (const auto& joint : command_joints_){
      config.names.push_back(joint + "/" + nmt_interface_name_);
    }
    for (const auto& joint : command_joints_){
      config.names.push_back(joint + "/" + motor_configured_interface_name_);
    }

    return config;
  }

  CallbackReturn DynamicsController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    nu_ref_.setZero();
    vel_ref_.setZero();
    u_.setZero();
    torque_.setZero();
    lastSamplingTime_ = get_node()->get_clock()->now();

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

  CallbackReturn DynamicsController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    for (unsigned int i = 0; i < command_joints_.size(); i++)
    {
      command_interfaces_[i].set_value(0);
    }

    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type DynamicsController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
  {
    rclcpp::Time time_ = get_node()->get_clock()->now();
    auto dt_ = (time_ - lastSamplingTime_);

    if (dt_.seconds() < std::numeric_limits<double>::epsilon())
      return controller_interface::return_type::OK;

    lastSamplingTime_ = time_;
    torque_.setZero();

    std::vector<double> motor_state_values, motor_configured_values;
    std::vector<nmt_state> motor_states;
    std::vector<bool> motor_configured;
    motor_state_values.reserve(command_joints_.size());
    motor_states.reserve(command_joints_.size());
    motor_configured_values.reserve(command_joints_.size());
    motor_configured.reserve(command_joints_.size());

    try {
      for (unsigned int i = 0; i < command_joints_.size(); i++){
        motor_state_values[i] = return_state_if_exists(motor_state_values[i], command_joints_[i] + nmt_interface_name_, state_interfaces_);
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
      is_locked_ = is_locked_ || (motor_states[i] != nmt_state::OPERATIONAL || !motor_configured[i]);
    }

    if (is_locked_) {
      reset();
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
        const auto orientation = (*odom)->pose.pose.orientation;

        u_[0] = getLongitudinalVelocity(vx, vy, tf2::getYaw(orientation));
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

    // Compute Velocity PID control actions
    auto command = rt_command_.readFromRT();
    if (command && *command)
    {
      vel_ref_[0] = (*command)->linear.x;
      vel_ref_[1] = (*command)->angular.z;
    }

    vel_ref_[0] = std::clamp(vel_ref_[0], -linear_velocity_limit_, linear_velocity_limit_);
    vel_ref_[1] = std::clamp(vel_ref_[1], -angular_velocity_limit_, angular_velocity_limit_);

    nu_ref_[0] = linear_vel_pid_->compute(vel_ref_[0], u_[0], dt_.seconds());
    nu_ref_[1] = angular_vel_pid_->compute(vel_ref_[1], u_[1], dt_.seconds());

    // Uncomment to bypass velocity controller
    // nu_ref_[0] = vel_ref_[0];
    // nu_ref_[1] = vel_ref_[1];

    // Compute Feedback Linearization
    // Linearizing Controller considers swapped wheels order
    // - tau_[0] => right_wheel
    // - tau_[1] => left_wheel
    Eigen::Vector2d tau_ = linearizing_controller_->compute_system_inputs(nu_ref_, u_);

    // Control action saturation + un-swap wheels order
    torque_ << std::clamp(tau_[1], effort_limit_[0], effort_limit_[1]),
               std::clamp(tau_[0], effort_limit_[0], effort_limit_[1]);

    // Apply torques
    for (unsigned int i = 0; i < command_joints_.size(); i++)
      command_interfaces_[i].set_value(torque_[i]);

    logStatus(time_, dt_);
    return controller_interface::return_type::OK;
  }

  void DynamicsController::commandCB(const geometry_msgs::msg::Twist::SharedPtr command)
  {
    rt_command_.writeFromNonRT(command);
  }
  void DynamicsController::odomCB(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    rt_odom_.writeFromNonRT(odom);
  }
  void DynamicsController::isEmergencyLockCB(const std_msgs::msg::Bool::SharedPtr is_emergency_lock)
  {
    rt_is_emergency_lock_.writeFromNonRT(is_emergency_lock);
  }

  void DynamicsController::logStatus(const rclcpp::Time& time, const rclcpp::Duration& dt)
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

      rt_status_publisher_->msg_.velocity_error[0] = linear_vel_pid_->get_error();
      rt_status_publisher_->msg_.velocity_error[1] = angular_vel_pid_->get_error();

      rt_status_publisher_->msg_.time_step_ms = ((double)(1000 * dt.seconds()));

      for (unsigned int i = 0; i < torque_.size(); i++)
      {
        rt_status_publisher_->msg_.command[i] = torque_[i];
      }

      rt_status_publisher_->msg_.locked = is_locked_;

      rt_status_publisher_->unlockAndPublish();
    }
  }

  void DynamicsController::reset()
  {
    nu_ref_.setZero();
    vel_ref_.setZero();
    u_.setZero();
    lastSamplingTime_ = get_node()->get_clock()->now();

    linear_vel_pid_->reset();
    angular_vel_pid_->reset();
  }
}

PLUGINLIB_EXPORT_CLASS(drc_flexnav_controllers::DynamicsController, controller_interface::ControllerInterface)
