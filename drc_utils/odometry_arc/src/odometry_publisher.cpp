#include <limits>
#include <rclcpp/rclcpp.hpp>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <std_srvs/srv/empty.hpp>

#include <odometry_arc/diff_odometry.hpp>

class OdometryPublisher : public rclcpp::Node {
public:
  OdometryPublisher(void);
  void publish(void);

private:
  Eigen::Vector2d phi_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub_;

  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<std_msgs::msg::Float64,
                                                      std_msgs::msg::Float64>>>
      joint_sync_;
  message_filters::Subscriber<std_msgs::msg::Float64> left_encoder_sub_;
  message_filters::Subscriber<std_msgs::msg::Float64> right_encoder_sub_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_odom_service_;

  bool use_joint_states_, use_joint_vel_;

  std::string left_wheel_name_, right_wheel_name_;
  std::vector<double> wheelRadius_;
  double wheel_to_robot_center;

  odometry_arc::DiffOdometry odom_;

  std::string odom_frame_id_;
  std::string odom_topic_name_;
  std::string joint_states_topic_name_;
  std::string base_frame_id_;
  std::string left_encoder_topic_name_, right_encoder_topic_name_;
  std::string reset_odom_srv_name_;

  rclcpp::Time lastSamplingTime_;
  rclcpp::TimerBase::SharedPtr timer_;

  void jointStateCB(const sensor_msgs::msg::JointState::SharedPtr jointState);
  void
  encodersSyncCB(const std_msgs::msg::Float64::ConstSharedPtr &left_encoder,
                 const std_msgs::msg::Float64::ConstSharedPtr &right_encoder);

  void
  resetOdometry(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                std::shared_ptr<std_srvs::srv::Empty::Response> response);
};

OdometryPublisher::OdometryPublisher(void)
    : Node("odometry_publisher"), wheelRadius_(2),
      odom_(wheel_to_robot_center, wheelRadius_) {

  use_joint_states_ = declare_parameter<bool>("use_joint_states", true);
  use_joint_vel_ = declare_parameter<bool>("use_joint_vel", false);
  left_wheel_name_ =
      declare_parameter<std::string>("left_wheel_name", "left_wheel");
  right_wheel_name_ =
      declare_parameter<std::string>("right_wheel_name", "right_wheel");
  wheel_to_robot_center =
      declare_parameter<double>("wheel_to_robot_center", 1.0);
  wheelRadius_ = declare_parameter<std::vector<double>>(
      "wheel_radius", std::vector<double>{0.1, 0.1});
  odom_topic_name_ = declare_parameter<std::string>("odom_topic_name", "odom");
  joint_states_topic_name_ =
      declare_parameter<std::string>("joint_states_topic_name", "joint_states");
  odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
  left_encoder_topic_name_ =
      declare_parameter<std::string>("left_encoder_topic_name", "left_encoder");
  right_encoder_topic_name_ = declare_parameter<std::string>(
      "right_encoder_topic_name", "right_encoder");
  reset_odom_srv_name_ =
      declare_parameter<std::string>("reset_odom_srv_name", "reset_odom");

  odom_.setParams(wheel_to_robot_center, wheelRadius_);
  phi_.setZero();

  odomPub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 100);

  tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  lastSamplingTime_ = this->get_clock()->now();

  if (use_joint_states_) {
    jointStateSub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_name_, 100,
        std::bind(&OdometryPublisher::jointStateCB, this,
                  std::placeholders::_1));
  } else {
    uint32_t queue_size = 10;

    left_encoder_sub_.subscribe(this, left_encoder_topic_name_);
    right_encoder_sub_.subscribe(this, right_encoder_topic_name_);

    joint_sync_ = std::make_shared<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            std_msgs::msg::Float64, std_msgs::msg::Float64>>>(
        message_filters::sync_policies::ApproximateTime<std_msgs::msg::Float64,
                                                        std_msgs::msg::Float64>(
            queue_size),
        left_encoder_sub_, right_encoder_sub_);

    joint_sync_->setAgePenalty(0.05);
    joint_sync_->registerCallback(std::bind(&OdometryPublisher::encodersSyncCB,
                                            this, std::placeholders::_1,
                                            std::placeholders::_2));
  }
  int rate = 100;
  declare_parameter("publish_rate", rate);
  get_parameter("publish_rate", rate);

  timer_ = rclcpp::create_timer(this, this->get_clock(),
                                rclcpp::Duration::from_seconds(1.0 / rate),
                                std::bind(&OdometryPublisher::publish, this));

  reset_odom_service_ = this->create_service<std_srvs::srv::Empty>(
      reset_odom_srv_name_,
      std::bind(&OdometryPublisher::resetOdometry, this, std::placeholders::_1,
                std::placeholders::_2));
}

void OdometryPublisher::jointStateCB(
    const sensor_msgs::msg::JointState::SharedPtr jointState) {
  rclcpp::Time time = jointState->header.stamp;

  // Incremental encoders sense angular displacement and
  // not velocity
  // phi[0] is the left wheel angular displacement
  // phi[1] is the right wheel angular displacement
  Eigen::Vector2d deltaPhi = -phi_;
  Eigen::Vector2d phiDot;
  for (unsigned int joint_index = 0; joint_index < jointState->name.size();
       joint_index++) {
    if (jointState->name[joint_index] == left_wheel_name_) {
      if (std::isnan(jointState->position[joint_index]) ||
          std::isnan(jointState->velocity[joint_index])) {
        odom_.resetVelocity();
        return;
      }
      phi_[0] = jointState->position[joint_index];
      phiDot[0] = jointState->velocity[joint_index];
    }
    if (jointState->name[joint_index] == right_wheel_name_) {
      if (std::isnan(jointState->position[joint_index]) ||
          std::isnan(jointState->velocity[joint_index])) {
        odom_.resetVelocity();
        return;
      }
      phi_[1] = jointState->position[joint_index];
      phiDot[1] = jointState->velocity[joint_index];
    }
  }
  deltaPhi += phi_;

  if (use_joint_vel_)
    odom_.update(deltaPhi, phiDot);
  else
    odom_.update(deltaPhi, time - lastSamplingTime_);
  lastSamplingTime_ = time;
}

void OdometryPublisher::encodersSyncCB(
    const std_msgs::msg::Float64::ConstSharedPtr &left_encoder,
    const std_msgs::msg::Float64::ConstSharedPtr &right_encoder) {
  rclcpp::Time time = this->get_clock()->now();

  // Incremental encoders sense angular displacement and
  Eigen::Vector2d deltaPhi;
  phi_[0] = -left_encoder->data;
  phi_[1] = -right_encoder->data;

  deltaPhi = phi_;

  odom_.update(deltaPhi, time - lastSamplingTime_);
  lastSamplingTime_ = time;
}

void OdometryPublisher::publish(void) {
  rclcpp::Time time = this->get_clock()->now();

  Eigen::Vector3d x;
  Eigen::Vector2d u;
  odom_.getPose(x);
  odom_.getVelocity(u);

  nav_msgs::msg::Odometry odomMsg;
  odomMsg.header.stamp = time;
  odomMsg.header.frame_id = odom_frame_id_;
  odomMsg.child_frame_id = base_frame_id_;

  odomMsg.pose.pose.position.x = x[0];
  odomMsg.pose.pose.position.y = x[1];
  odomMsg.pose.pose.position.z = 0.0;

  odomMsg.pose.pose.orientation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), x[2]));

  // Fake covariance
  double pose_cov[] = {1e-6, 1e-6, 1e-16, 1e-16, 1e-16, 1e-9};
  odomMsg.pose.covariance = {
      pose_cov[0], 0, 0, 0, 0, 0, 0, pose_cov[1], 0, 0, 0, 0, 0, 0,
      pose_cov[2], 0, 0, 0, 0, 0, 0, pose_cov[3], 0, 0, 0, 0, 0, 0,
      pose_cov[4], 0, 0, 0, 0, 0, 0, pose_cov[5]};

  odomMsg.twist.twist.linear.x = u[0] * cos(x[2]);
  odomMsg.twist.twist.linear.y = u[0] * sin(x[2]);
  odomMsg.twist.twist.linear.z = 0;
  odomMsg.twist.twist.angular.x = 0;
  odomMsg.twist.twist.angular.y = 0;
  odomMsg.twist.twist.angular.z = u[1];

  // Fake covariance
  double twist_cov[] = {1e-6, 1e-6, 1e-16, 1e-16, 1e-16, 1e-9};
  odomMsg.twist.covariance = {
      twist_cov[0], 0, 0, 0, 0, 0, 0, twist_cov[1], 0, 0, 0, 0, 0, 0,
      twist_cov[2], 0, 0, 0, 0, 0, 0, twist_cov[3], 0, 0, 0, 0, 0, 0,
      twist_cov[4], 0, 0, 0, 0, 0, 0, twist_cov[5]};

  odomPub_->publish(odomMsg);

  geometry_msgs::msg::TransformStamped odomFrame;
  odomFrame.header.stamp = time;
  odomFrame.header.frame_id = odom_frame_id_;
  odomFrame.child_frame_id = base_frame_id_;
  odomFrame.transform.translation.x = x[0];
  odomFrame.transform.translation.y = x[1];
  odomFrame.transform.translation.z = 0.0;
  odomFrame.transform.rotation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), x[2]));

  tfBroadcaster_->sendTransform(odomFrame);
}

void OdometryPublisher::resetOdometry(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  odom_.resetOdometry();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
