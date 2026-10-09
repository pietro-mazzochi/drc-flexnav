#include "sensor_subscriber.hpp"
#include <algorithm>
#include <stdexcept>

SensorSubscriber::SensorSubscriber() : Node("frequency_check")
{
  try {
    this->declare_parameter<std::string>("rear_scan_topic", "/rear_scan");
    this->declare_parameter<std::string>("rear_lidar_topic", "/rear_lidar");
    this->declare_parameter<std::string>("front_scan_topic", "/front_scan");
    this->declare_parameter<std::string>("front_lidar_topic", "/front_lidar");
    this->declare_parameter<std::string>("magnetometer_topic", "/imu_sensor/magnetic_field");
    this->declare_parameter<std::string>("imu_topic", "/imu_sensor/imu");
    this->declare_parameter<std::string>("joint_states_topic", "/joint_states");   // ← Novo parâmetro
    this->declare_parameter<int>("window_size", 100);
    this->declare_parameter<int>("timeout_ms", 3000);

    this->declare_parameter<std::vector<std::string>>("sensors.names",
      std::vector<std::string>{"imu", "magnetometer", "front_lidar", "front_scan", "rear_lidar", "rear_scan", "joint_states"});
    this->declare_parameter<std::vector<double>>("sensors.min_frequency",
      std::vector<double>{10.0, 10.0, 4.0, 4.0, 4.0, 4.0, 5.0});
    this->declare_parameter<std::vector<double>>("sensors.max_frequency",
      std::vector<double>{120.0, 120.0, 10.0, 10.0, 10.0, 10.0, 50.0});
    this->declare_parameter<std::vector<std::string>>("sensors.active_sensors", std::vector<std::string>{});

    this->get_parameter("rear_scan_topic", rear_scan_topic_name);
    this->get_parameter("rear_lidar_topic", rear_lidar_topic_name);
    this->get_parameter("front_scan_topic", front_scan_topic_name);
    this->get_parameter("front_lidar_topic", front_lidar_topic_name);
    this->get_parameter("magnetometer_topic", magnetometer_topic_name);
    this->get_parameter("imu_topic", imu_topic_name);
    this->get_parameter("joint_states_topic", joint_states_topic_name);   // ← Obtém o nome do tópico

    int window_size_int = 0, timeout_ms = 0;
    this->get_parameter("window_size", window_size_int);
    this->get_parameter("timeout_ms", timeout_ms);
    if(window_size_int <= 0){
      throw std::invalid_argument("window_size must be greater than 0");
    }
    if(timeout_ms <= 0){
      throw std::invalid_argument("timeout_ms must be greater than 0");
    }
    window_size_ = static_cast<size_t>(window_size_int);
    timeout_duration_ = std::chrono::milliseconds(timeout_ms);

    std::vector<std::string> sensor_names;
    std::vector<double> min_freqs, max_freqs;
    this->get_parameter("sensors.names", sensor_names);
    this->get_parameter("sensors.min_frequency", min_freqs);
    this->get_parameter("sensors.max_frequency", max_freqs);

    if (sensor_names.size() != min_freqs.size() ||
        sensor_names.size() != max_freqs.size())
    {
      throw std::invalid_argument("sensors.names, sensors.min_frequency, and sensors.max_frequency must have the same length");
    }
    all_sensor_names_ = sensor_names;

    std::vector<std::string> active_sensors;
    this->get_parameter("sensors.active_sensors", active_sensors);
    if(active_sensors.empty()){
      active_sensors_ = sensor_names;
    } else {
      active_sensors_ = active_sensors;
    }

    // frequency_ranges_ only of "active sensors"
    for (size_t i = 0; i < sensor_names.size(); ++i) {
      if (std::find(active_sensors_.begin(), active_sensors_.end(), sensor_names[i]) == active_sensors_.end()) {
        continue;
      }
      if (min_freqs[i] <= 0 || max_freqs[i] <= 0 || min_freqs[i] > max_freqs[i]) {
         throw std::invalid_argument("Invalid frequency range for sensor: " + sensor_names[i]);
      }
      frequency_ranges_[sensor_names[i]] = std::make_pair(min_freqs[i], max_freqs[i]);
    }

    for (const auto &sensor : frequency_ranges_) {
      const std::string &sensor_name = sensor.first;
      if (sensor_name == "rear_scan") {
        rear_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          rear_scan_topic_name, 10,
          std::bind(&SensorSubscriber::rear_scan_callback, this, std::placeholders::_1));
      } else if (sensor_name == "rear_lidar") {
        rear_lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          rear_lidar_topic_name, 10,
          std::bind(&SensorSubscriber::rear_lidar_callback, this, std::placeholders::_1));
      } else if (sensor_name == "front_scan") {
        front_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          front_scan_topic_name, 10,
          std::bind(&SensorSubscriber::front_scan_callback, this, std::placeholders::_1));
      } else if (sensor_name == "front_lidar") {
        front_lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          front_lidar_topic_name, 10,
          std::bind(&SensorSubscriber::front_lidar_callback, this, std::placeholders::_1));
      } else if (sensor_name == "magnetometer") {
        magnetometer_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
          magnetometer_topic_name, 10,
          std::bind(&SensorSubscriber::magnetometer_callback, this, std::placeholders::_1));
      } else if (sensor_name == "imu") {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
          imu_topic_name, 10,
          std::bind(&SensorSubscriber::imu_callback, this, std::placeholders::_1));
      } else if (sensor_name == "joint_states") {   // ← Adiciona a assinatura para joint states
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
          joint_states_topic_name, 10,
          std::bind(&SensorSubscriber::joint_states_callback, this, std::placeholders::_1));
      } else {
        RCLCPP_WARN(this->get_logger(), "No subscription configured for sensor: %s", sensor_name.c_str());
      }
    }

    frequency_pub_ = this->create_publisher<frequency_check_msg::msg::FrequencyCheck>("frequency_check", 10);

    // creates a timer to log and check frequency every X seconds
    timer_ = this->create_wall_timer(std::chrono::seconds(2),
      std::bind(&SensorSubscriber::log_and_check_frequency, this));
  }
  catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Parameter validation failed: %s", e.what());
    rclcpp::shutdown();
  }
}

void SensorSubscriber::rear_scan_callback(sensor_msgs::msg::LaserScan::SharedPtr)
{
  process_message("rear_scan");
}

void SensorSubscriber::rear_lidar_callback(sensor_msgs::msg::PointCloud2::SharedPtr)
{
  process_message("rear_lidar");
}

void SensorSubscriber::front_scan_callback(sensor_msgs::msg::LaserScan::SharedPtr)
{
  process_message("front_scan");
}

void SensorSubscriber::front_lidar_callback(sensor_msgs::msg::PointCloud2::SharedPtr)
{
  process_message("front_lidar");
}

void SensorSubscriber::magnetometer_callback(sensor_msgs::msg::MagneticField::SharedPtr)
{
  process_message("magnetometer");
}

void SensorSubscriber::imu_callback(sensor_msgs::msg::Imu::SharedPtr)
{
  process_message("imu");
}

void SensorSubscriber::joint_states_callback(sensor_msgs::msg::JointState::SharedPtr)
{
  process_message("joint_states");
}

void SensorSubscriber::process_message(const std::string &sensor_name)
{
  auto now = std::chrono::steady_clock::now();
  message_times_[sensor_name].push_back(now);
  if (message_times_[sensor_name].size() > window_size_) {
    message_times_[sensor_name].pop_front();
  }
}

void SensorSubscriber::log_and_check_frequency()
{
  frequency_check_msg::msg::FrequencyCheck frequency_msg;
  frequency_msg.alert = false;
  frequency_msg.frequency_error = "";

  for (const auto &sensor_name : all_sensor_names_) {
    bool sensor_ok = check_frequency(sensor_name);
    bool is_monitored = (std::find(active_sensors_.begin(), active_sensors_.end(), sensor_name) != active_sensors_.end());

    if (is_monitored && !sensor_ok) {
      frequency_msg.frequency_error += sensor_name + ", ";
      RCLCPP_WARN(this->get_logger(), "%s frequency out of range or timed out.", sensor_name.c_str());
    }
    if (sensor_name == "imu") {
      frequency_msg.imu_publishing = sensor_ok;
    } else if (sensor_name == "magnetometer") {
      frequency_msg.mag_publishing = sensor_ok;
    } else if (sensor_name == "front_lidar") {
      frequency_msg.front_lidar_publishing = sensor_ok;
    } else if (sensor_name == "front_scan") {
      frequency_msg.front_scan_publishing = sensor_ok;
    } else if (sensor_name == "rear_lidar") {
      frequency_msg.rear_lidar_publishing = sensor_ok;
    } else if (sensor_name == "rear_scan") {
      frequency_msg.rear_scan_publishing = sensor_ok;
    } else if (sensor_name == "joint_states") {   // ← Mapeia o status para joint_states
      frequency_msg.joint_states_publishing = sensor_ok;
    }
  }

  if (!frequency_msg.frequency_error.empty()) {
    frequency_msg.frequency_error = frequency_msg.frequency_error.substr(0, frequency_msg.frequency_error.size() - 2);
    frequency_msg.alert = true;
  }

  frequency_pub_->publish(frequency_msg);
}

bool SensorSubscriber::check_frequency(const std::string &sensor_name)
{
  if (message_times_.find(sensor_name) == message_times_.end() ||
      message_times_[sensor_name].size() < 2)
  {
    return false;
  }

  auto now = std::chrono::steady_clock::now();
  auto time_since_last_message = now - message_times_[sensor_name].back();
  if (time_since_last_message > timeout_duration_) {
    return false;
  }

  double duration = std::chrono::duration_cast<std::chrono::duration<double>>(
                      now - message_times_[sensor_name].front()).count();
  if (duration <= 0.0) {
    return false;
  }
  double frequency = (message_times_[sensor_name].size() - 1) / duration;

  auto range = frequency_ranges_.find(sensor_name);
  if (range != frequency_ranges_.end()) {
    double min_freq = range->second.first;
    double max_freq = range->second.second;
    return (frequency >= min_freq) && (frequency <= max_freq);
  }
  return false;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
