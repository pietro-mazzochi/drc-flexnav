#ifndef SENSOR_SUBSCRIBER_HPP
#define SENSOR_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <frequency_check_msg/msg/frequency_check.hpp>

#include <deque>
#include <unordered_map>
#include <chrono>
#include <string>
#include <vector>
#include <map>

class SensorSubscriber : public rclcpp::Node {
public:
    SensorSubscriber();

private:
    void rear_scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void rear_lidar_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void front_scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void front_lidar_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void magnetometer_callback(sensor_msgs::msg::MagneticField::SharedPtr msg);
    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);
    void joint_states_callback(sensor_msgs::msg::JointState::SharedPtr msg);  // ← Novo callback

    void process_message(const std::string &sensor_name);
    void log_and_check_frequency();
    bool check_frequency(const std::string &sensor_name);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rear_lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr front_lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr magnetometer_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;     // ← Novo subscriber

    rclcpp::Publisher<frequency_check_msg::msg::FrequencyCheck>::SharedPtr frequency_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unordered_map<std::string, std::deque<std::chrono::steady_clock::time_point>> message_times_;
    size_t window_size_;
    std::chrono::milliseconds timeout_duration_;
    std::unordered_map<std::string, std::pair<double, double>> frequency_ranges_;
    std::vector<std::string> all_sensor_names_;
    std::vector<std::string> active_sensors_;

    std::string rear_scan_topic_name, rear_lidar_topic_name, front_scan_topic_name,
                front_lidar_topic_name, magnetometer_topic_name, imu_topic_name,
                joint_states_topic_name;
};

#endif // SENSOR_SUBSCRIBER_HPP
