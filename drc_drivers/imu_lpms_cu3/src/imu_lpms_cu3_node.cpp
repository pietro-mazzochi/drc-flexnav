#include "imu_lpms_cu3/imu_lpms_cu3_sensor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include <chrono>

using namespace std::chrono_literals;

drc_flexnav_sensors::Inertial_measurement_unit_and_compass FlexnavIMU;

class ImuPublisher : public rclcpp::Node {
public:
  ImuPublisher() : Node("imu_publisher") {

    RCLCPP_INFO(this->get_logger(), "Initializing IMU driver");

    this->declare_parameter<int>("identifier", 1);
    this->declare_parameter<int>("update_rate", 100);
    this->declare_parameter<bool>("double_precision", true);
    this->declare_parameter<int>("serial_baudrate", 1000000);
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");

    int imu_identifier_;
    if (this->get_parameter("identifier", imu_identifier_)) {
      RCLCPP_INFO(this->get_logger(), "identifier: %i", imu_identifier_);
    }

    int imu_update_rate_;
    if (this->get_parameter("update_rate", imu_update_rate_)) {
      RCLCPP_INFO(this->get_logger(), "update_rate: %i Hz", imu_update_rate_);
    }

    bool imu_precision_;
    if (this->get_parameter("double_precision", imu_precision_)) {
      RCLCPP_INFO(this->get_logger(), "Use double precision: %s",
                  (true == imu_precision_) ? "true" : "false");
    }

    std::string imu_serial_port_;
    if (this->get_parameter("serial_port", imu_serial_port_)) {
      RCLCPP_INFO(this->get_logger(), "serial_port: %s",
                  imu_serial_port_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Parameter not found, using default value: %s",
                  imu_serial_port_.c_str());
    }

    int imu_serial_baudrate_;
    if (this->get_parameter("serial_baudrate", imu_serial_baudrate_)) {
      RCLCPP_INFO(this->get_logger(), "serial_baudrate: %i",
                  imu_serial_baudrate_);
    }

    if (!FlexnavIMU.init(imu_identifier_, imu_serial_port_,
                         imu_serial_baudrate_)) {
      RCLCPP_ERROR(this->get_logger(), "Can't init IMU driver.");
      exit(-1);
    } else {
    }

    FlexnavIMU.flush();
    if (imu_identifier_ < 0 && FlexnavIMU.detect_id() < 0) {
      RCLCPP_ERROR(this->get_logger(), "Can't get ID from IMU. Using ID = 0.");
      FlexnavIMU.set_id(0);
    }

    if (FlexnavIMU.set_operating_mode(FLEXNAV_IMU_OPERATING_MODE_COMMAND)) {
      RCLCPP_INFO(this->get_logger(), "Command mode OK. ");
    } else {
      RCLCPP_ERROR(this->get_logger(), "GOTO_COMMAND_MODE fail!!!");
    }
    if (FlexnavIMU.set_stream_frequency(imu_update_rate_)) {
      RCLCPP_INFO(this->get_logger(), "SET FREQUENCY OK. ");
    } else {
      RCLCPP_ERROR(this->get_logger(), "SET FREQUENCY fail!!!");
    }

    if (imu_precision_) {
      if (FlexnavIMU.set_32bits_precision(true))
        RCLCPP_INFO(this->get_logger(), "Precision set succef.");
      else
        RCLCPP_ERROR(this->get_logger(), "Fail to set data precision.");
    } else {
      if (FlexnavIMU.set_32bits_precision(false))
        RCLCPP_INFO(this->get_logger(), "Precision unset succef.");
      else
        RCLCPP_ERROR(this->get_logger(), "Fail to unset data precision.");
    }

    if (FlexnavIMU.get_current_transmit_data())
      RCLCPP_INFO(this->get_logger(), "Current transmit data ok.");
    else
      RCLCPP_ERROR(this->get_logger(), "Fail when get transmit data profile.");
    if (FlexnavIMU.set_operating_mode(FLEXNAV_IMU_OPERATING_MODE_STREAM))
      RCLCPP_INFO(this->get_logger(), "Stream mode OK. ");
    else
      RCLCPP_ERROR(this->get_logger(), "GOTO_STREAM_MODE fail!!!");

    publisherImu_ =
        this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    publisherMag_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
        "magnetic_field/data", 10);
    timer_ = this->create_wall_timer(
        1000 / imu_update_rate_ * 1ms,
        std::bind(&ImuPublisher::publish_imu_data, this));
  }

private:
  void publish_imu_data() {
    auto messageImu = sensor_msgs::msg::Imu();
    auto messageMagno = sensor_msgs::msg::MagneticField();

    if (FlexnavIMU
            .get_stream_loop()) // publish only when have a new data from IMU
    {
      uint32_t timestamp = FlexnavIMU.get_timestamp();
      messageImu.header.stamp.sec = timestamp / 500;
      messageImu.header.stamp.nanosec = (timestamp % 500) * 2000000;
      messageImu.header.frame_id = "imu_frame";
      messageImu.orientation.x = FlexnavIMU.get_quaternion_x();
      messageImu.orientation.y = FlexnavIMU.get_quaternion_y();
      messageImu.orientation.z = FlexnavIMU.get_quaternion_z();
      messageImu.orientation.w = FlexnavIMU.get_quaternion_w();

      messageImu.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

      messageImu.angular_velocity.x = FlexnavIMU.get_angular_vel_x();
      messageImu.angular_velocity.y = FlexnavIMU.get_angular_vel_y();
      messageImu.angular_velocity.z = FlexnavIMU.get_angular_vel_z();

      messageImu.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

      messageImu.linear_acceleration.x = FlexnavIMU.get_accel_raw_x();
      messageImu.linear_acceleration.y = FlexnavIMU.get_accel_raw_y();
      messageImu.linear_acceleration.z = FlexnavIMU.get_accel_raw_z();

      messageImu.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

      publisherImu_->publish(messageImu);

      // publish magnetometer
      messageMagno.header.stamp.sec = messageImu.header.stamp.sec;
      messageMagno.header.stamp.nanosec = messageImu.header.stamp.nanosec;
      messageMagno.header.frame_id = "magnetometer_frame";

      messageMagno.magnetic_field.x = FlexnavIMU.get_mag_raw_x();
      messageMagno.magnetic_field.y = FlexnavIMU.get_mag_raw_y();
      messageMagno.magnetic_field.z = FlexnavIMU.get_mag_raw_z();

      messageMagno.magnetic_field_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

      publisherMag_->publish(messageMagno);
    }
  }
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisherImu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisherMag_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPublisher>());
  rclcpp::shutdown();
  return 0;
}
