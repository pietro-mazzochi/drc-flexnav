#include "rclcpp/rclcpp.hpp"

#include <cmath>
#include <cstdint>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <drc_flexnav_pose_estimation/extended_kalman_filter.hpp>

class ExtendedKalmanFilterNode : public rclcpp::Node {
public:
  ExtendedKalmanFilterNode()
      : Node("extended_kalman_filter_node"),
        last_time_{rclcpp::Time(0, 0, this->get_clock()->get_clock_type())} {
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Using clock type: " << this->get_clock()->get_clock_type());

    wheel_base_ = declare_parameter<double>("wheel_separation", 1.0945);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.125);
    theta_C_mag_ = declare_parameter<double>("theta_C_mag", 2.56e-01);
    epsilon_ = declare_parameter<double>("epsilon", 1.0e-40);

    estimation_topic_name_ = declare_parameter<std::string>(
        "estimation_topic_name", "/kalman_estimation");
    estimation_frame_id_ =
        declare_parameter<std::string>("estimation_frame_id", "kalman");
    base_frame_id_ =
        declare_parameter<std::string>("base_frame_id", "base_link");
    queue_size_ = declare_parameter<int>("queue_size", 100);
    publish_odom_to_tf_ = declare_parameter<bool>("publish_odom_to_tf", true);

    double x_initial = declare_parameter<double>("x_initial", 0.0);
    double y_initial = declare_parameter<double>("y_initial", 0.0);
    double theta_initial = declare_parameter<double>("theta_initial", 0.0);
    double v_initial = declare_parameter<double>("v_initial", 0.0);
    double omega_initial = declare_parameter<double>("omega_initial", 0.0);

    double baccx = declare_parameter<double>("baccx", 0.0);
    double baccy = declare_parameter<double>("baccy", 0.0);
    double baccz = declare_parameter<double>("baccz", 0.0);
    double bgirox = declare_parameter<double>("bgirox", 0.0);
    double bgiroy = declare_parameter<double>("bgiroy", 0.0);
    double bgiroz = declare_parameter<double>("bgiroz", 0.0);

    left_wheel_name_ =
        declare_parameter<std::string>("left_wheel_name", "left_wheel");
    right_wheel_name_ =
        declare_parameter<std::string>("right_wheel_name", "right_wheel");
    joint_states_topic_name_ = declare_parameter<std::string>(
        "joint_states_topic_name", "/joint_states");
    magnetometer_topic_name_ = declare_parameter<std::string>(
        "magnetometer_topic_name", "/magnetometer/data");
    imu_topic_name_ =
        declare_parameter<std::string>("imu_topic_name", "/imu/out");
    accel_topic_name_ =
        declare_parameter<std::string>("accel_topic_name", "/mu");

    RCLCPP_INFO_STREAM(this->get_logger(), "Initializing ExtendedKalmanFilter");
    filter_ = std::make_unique<DRC_PoseEstimation::ExtendedKalmanFilter>(
        wheel_radius_, wheel_base_ / 2.0, theta_C_mag_, x_initial, y_initial,
        theta_initial, v_initial, omega_initial, baccx, baccy, baccz, bgirox,
        bgiroy, bgiroz);
    filter_->set_epsilon(epsilon_);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "ExtendedKalmanFilter sucessfully initialized");

    estimation_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(estimation_topic_name_, 20);

    if (publish_odom_to_tf_) {
      tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    magnetometer_sub_.subscribe(this, magnetometer_topic_name_);
    imu_sub_.subscribe(this, imu_topic_name_);
    accel_sub_.subscribe(this, accel_topic_name_);
    joint_states_sub_.subscribe(this, joint_states_topic_name_);

    uint32_t queue_size = static_cast<uint32_t>(queue_size_);

    sensors_sync_w_joint_states_ =
        std::make_shared<message_filters::Synchronizer<
            message_filters::sync_policies::ApproximateTime<
                sensor_msgs::msg::JointState, sensor_msgs::msg::MagneticField,
                sensor_msgs::msg::Imu, geometry_msgs::msg::AccelStamped>>>(
            message_filters::sync_policies::ApproximateTime<
                sensor_msgs::msg::JointState, sensor_msgs::msg::MagneticField,
                sensor_msgs::msg::Imu, geometry_msgs::msg::AccelStamped>(
                queue_size),
            joint_states_sub_, magnetometer_sub_, imu_sub_, accel_sub_);

    sensors_sync_w_joint_states_->setAgePenalty(0.1);
    sensors_sync_w_joint_states_->registerCallback(std::bind(
        &ExtendedKalmanFilterNode::sensors_sync_cb, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    RCLCPP_INFO_STREAM(this->get_logger(), "end");
  }

private:
  void sensors_sync_cb(
      const sensor_msgs::msg::JointState::ConstSharedPtr &joint_states,
      const sensor_msgs::msg::MagneticField::ConstSharedPtr &magnetic_field,
      const sensor_msgs::msg::Imu::ConstSharedPtr &imu,
      const geometry_msgs::msg::AccelStamped::ConstSharedPtr &accel) {

    rclcpp::Time node_time = this->get_clock()->now();

    if (last_time_.seconds() == 0 && last_time_.nanoseconds() == 0) {
      last_time_ = node_time;
      return;
    }

    double left_wheel_vel, right_wheel_vel;

    for (unsigned int joint_index = 0; joint_index < joint_states->name.size();
         joint_index++) {
      if (joint_states->name[joint_index] == left_wheel_name_)
        left_wheel_vel = joint_states->velocity[joint_index];
      if (joint_states->name[joint_index] == right_wheel_name_)
        right_wheel_vel = joint_states->velocity[joint_index];
    }
    double v = static_cast<double>(
        sqrt(accel->accel.linear.x * accel->accel.linear.x +
             accel->accel.linear.y * accel->accel.linear.y));

    rclcpp::Duration T = node_time - last_time_;

    try {
      filter_->compute(
          imu->linear_acceleration.x, -imu->linear_acceleration.y,
          -imu->linear_acceleration.z, imu->angular_velocity.x,
          -imu->angular_velocity.y, -imu->angular_velocity.z,
          magnetic_field->magnetic_field.x, magnetic_field->magnetic_field.y,
          magnetic_field->magnetic_field.z, left_wheel_vel, right_wheel_vel, v,
          accel->accel.angular.z, T.to_chrono<std::chrono::duration<double>>());
      publish_odometry();
    } catch (const std::exception &e) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Caught exception: " << e.what());
    }
    last_time_ = node_time;
  }

  void publish_odometry() {
    Eigen::Vector<double, 11> x = filter_->get_x_state_corr();
    Eigen::Matrix<double, 11, 11> P = filter_->get_P_current();

    nav_msgs::msg::Odometry odomMsg;
    odomMsg.header.stamp = last_time_;
    odomMsg.header.frame_id = estimation_frame_id_;
    odomMsg.child_frame_id = base_frame_id_;

    odomMsg.pose.pose.position.x = x[0];
    odomMsg.pose.pose.position.y = x[1];
    odomMsg.pose.pose.position.z = 0.0;

    odomMsg.pose.pose.orientation =
        tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), x[2]));

    // Fake covariance
    // TODO: use estimated covariance
    double pose_cov[] = {1e-6, 1e-6, 1e-16, 1e-16, 1e-16, 1e-9};
    odomMsg.pose.covariance = {
        pose_cov[0], 0, 0, 0, 0, 0, 0, pose_cov[1], 0, 0, 0, 0, 0, 0,
        pose_cov[2], 0, 0, 0, 0, 0, 0, pose_cov[3], 0, 0, 0, 0, 0, 0,
        pose_cov[4], 0, 0, 0, 0, 0, 0, pose_cov[5]};

    odomMsg.twist.twist.linear.x = x[3] * cos(x[2]);
    odomMsg.twist.twist.linear.y = x[3] * sin(x[2]);
    odomMsg.twist.twist.linear.z = 0;
    odomMsg.twist.twist.angular.x = 0;
    odomMsg.twist.twist.angular.y = 0;
    odomMsg.twist.twist.angular.z = x[4];

    // Fake covariance
    // TODO: use estimated covariance
    double twist_cov[] = {1e-6, 1e-6, 1e-16, 1e-16, 1e-16, 1e-9};
    odomMsg.twist.covariance = {
        twist_cov[0], 0, 0, 0, 0, 0, 0, twist_cov[1], 0, 0, 0, 0, 0, 0,
        twist_cov[2], 0, 0, 0, 0, 0, 0, twist_cov[3], 0, 0, 0, 0, 0, 0,
        twist_cov[4], 0, 0, 0, 0, 0, 0, twist_cov[5]};

    estimation_pub_->publish(odomMsg);

    if (publish_odom_to_tf_) {
      geometry_msgs::msg::TransformStamped odomFrame;
      odomFrame.header.stamp = last_time_;
      odomFrame.header.frame_id = estimation_frame_id_;
      odomFrame.child_frame_id = base_frame_id_;
      odomFrame.transform.translation.x = x[0];
      odomFrame.transform.translation.y = x[1];
      odomFrame.transform.translation.z = 0.0;
      odomFrame.transform.rotation =
          tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), x[2]));

      tfBroadcaster_->sendTransform(odomFrame);
    }
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimation_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::JointState, sensor_msgs::msg::MagneticField,
          sensor_msgs::msg::Imu, geometry_msgs::msg::AccelStamped>>>
      sensors_sync_w_joint_states_;
  message_filters::Subscriber<sensor_msgs::msg::JointState> joint_states_sub_;
  message_filters::Subscriber<sensor_msgs::msg::MagneticField>
      magnetometer_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
  message_filters::Subscriber<geometry_msgs::msg::AccelStamped> accel_sub_;

  std::unique_ptr<DRC_PoseEstimation::ExtendedKalmanFilter> filter_;

  std::string left_wheel_name_, right_wheel_name_;
  double wheel_radius_, wheel_base_, theta_C_mag_, epsilon_;

  bool publish_odom_to_tf_;
  std::string estimation_frame_id_;
  std::string estimation_topic_name_;
  int queue_size_;
  std::string base_frame_id_;
  std::string joint_states_topic_name_, magnetometer_topic_name_,
      imu_topic_name_, accel_topic_name_;
  rclcpp::Time last_time_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExtendedKalmanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
