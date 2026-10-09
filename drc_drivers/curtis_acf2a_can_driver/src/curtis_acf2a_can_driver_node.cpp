#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int64.hpp"

#include "curtis_acf2a_can_driver/curtis_acf2a_can_driver_lib.hpp"

// Defines to config motor
#define LOG_LEVEL_DEFAULT "verbose"
#define CAN_READ_RATE_DEFAULT 1000
#define CAN_WRITE_RATE_DEFAULT 100
#define LEFT_MOTOR_ID_DEFAULT 0x25
#define RIGHT_MOTOR_ID_DEFAULT 0x26
#define LEFT_CAN_INTERFACE_DEFAULT "can0"
#define RIGHT_CAN_INTERFACE_DEFAULT "can1"
#define LEFT_MOTOR_SWAP_DIR_DEFAULT 0
#define RIGHT_MOTOR_SWAP_DIR_DEFAULT 1
#define GEAR_RATIO_DEFAULT 35.91
#define ENC_MAX_COUNTS_DEFAULT 4096

// Define DEBUG_ON. Uncomment to print errors and motor states
#define DEBUG_ON

// Create CAN1 interface object
CANOPEN_Interface canopen_left;
CANOPEN_Interface canopen_right;

// Create Curtis motor intefrace object
Curtis_ACF2A_Can_Driver left_motor(canopen_left);   // Motor left
Curtis_ACF2A_Can_Driver right_motor(canopen_right); // Motor right

class Curtis_ACF2A_Can_Driver_Node : public rclcpp::Node {
public:
  Curtis_ACF2A_Can_Driver_Node()
      : Node("curtis_ACF2A_can_driver_node"), stop_thread_(false) {
    if ((log_level == "info") | (log_level == "verbose")) {
      RCLCPP_INFO(this->get_logger(), "Initializing Curtis Motor driver");
    }

    // Declare parameters
    std::string can_left_port_;
    std::string can_right_port_;
    log_level =
        this->declare_parameter<std::string>("log_level", LOG_LEVEL_DEFAULT);
    can_read_rate_ = static_cast<uint16_t>(this->declare_parameter<uint16_t>(
        "can_read_rate", CAN_READ_RATE_DEFAULT));
    can_write_rate_ = static_cast<uint16_t>(this->declare_parameter<uint16_t>(
        "can_write_rate", CAN_WRITE_RATE_DEFAULT));
    left_motor_id_ = static_cast<uint8_t>(this->declare_parameter<uint8_t>(
        "left_motor_id", LEFT_MOTOR_ID_DEFAULT));
    right_motor_id_ = static_cast<uint8_t>(this->declare_parameter<uint8_t>(
        "right_motor_id", RIGHT_MOTOR_ID_DEFAULT));
    left_motor_swap_dir_ =
        static_cast<uint8_t>(this->declare_parameter<uint8_t>(
            "left_motor_swap_dir", LEFT_MOTOR_SWAP_DIR_DEFAULT));
    right_motor_swap_dir_ =
        static_cast<uint8_t>(this->declare_parameter<uint8_t>(
            "right_motor_swap_dir", RIGHT_MOTOR_SWAP_DIR_DEFAULT));
    can_left_port_ = this->declare_parameter<std::string>(
        "can_port_left", LEFT_CAN_INTERFACE_DEFAULT);
    can_right_port_ = this->declare_parameter<std::string>(
        "can_port_right", RIGHT_CAN_INTERFACE_DEFAULT);
    joint_state_topic_name_ = this->declare_parameter<std::string>(
        "joint_state_topic_name", "/joint_states");
    left_wheel_joint_name_ = this->declare_parameter<std::string>(
        "left_wheel_joint_name", "left_wheel");
    right_wheel_joint_name_ = this->declare_parameter<std::string>(
        "right_wheel_joint_name", "right_wheel");
    gear_ratio =
        this->declare_parameter<float>("gear_ratio", GEAR_RATIO_DEFAULT);
    encoder_max_counts = this->declare_parameter<int>("encoder_max_counts",
                                                      ENC_MAX_COUNTS_DEFAULT);
    left_motor_control_mode_ = this->declare_parameter<int>(
        "left_motor_control_mode", CONFIG_CONTROL_MODE_DEFAULT);
    right_motor_control_mode_ = this->declare_parameter<int>(
        "right_motor_control_mode", CONFIG_CONTROL_MODE_DEFAULT);

    // Get parameters
    // can_read_rate_ =
    // static_cast<uint16_t>(this->get_parameter("can_read_rate").as_int());
    // can_write_rate_ =
    // static_cast<uint16_t>(this->get_parameter("can_write_rate").as_int());
    // left_motor_id_ =
    // static_cast<uint8_t>(this->get_parameter("left_motor_id").as_int());
    // right_motor_id_ =
    // static_cast<uint8_t>(this->get_parameter("right_motor_id").as_int());
    // can_left_port_ =
    // static_cast<std::string>(this->get_parameter("can_port_left").as_string());
    // can_right_port_ =
    // static_cast<std::string>(this->get_parameter("can_port_right").as_string());
    // joint_state_topic_name_ =
    // static_cast<std::string>(this->get_parameter("joint_state_topic_name").as_string());
    // left_wheel_joint_name_ =
    // static_cast<std::string>(this->get_parameter("left_wheel_joint_name").as_string());
    // right_wheel_joint_name_ =
    // static_cast<std::string>(this->get_parameter("right_wheel_joint_name").as_string());

    if (log_level == "verbose") {
      RCLCPP_INFO(this->get_logger(), "can_read_rate: %u", can_read_rate_);
      RCLCPP_INFO(this->get_logger(), "can_write_rate: %u", can_write_rate_);
      RCLCPP_INFO(this->get_logger(), "left_motor_id: 0x%x", left_motor_id_);
      RCLCPP_INFO(this->get_logger(), "right_motor_id: 0x%x", right_motor_id_);
      RCLCPP_INFO(this->get_logger(), "can_port_left: %s",
                  can_left_port_.c_str());
      RCLCPP_INFO(this->get_logger(), "can_port_right: %s",
                  can_right_port_.c_str());
    }

    // Shutdown callback
    rclcpp::on_shutdown(
        std::bind(&Curtis_ACF2A_Can_Driver_Node::on_shutdown_callback, this));

    // Init CANOpen stack
    if (!canopen_left.init(can_left_port_.c_str()))
      return;
    if (!canopen_right.init(can_right_port_.c_str()))
      return;

    // Create the subscribers
    left_motor_torque_ref_subscriber_ =
        this->create_subscription<std_msgs::msg::Float32>(
            "/left_motor/throttle", 10,
            std::bind(&Curtis_ACF2A_Can_Driver_Node::throttleleft_motorCallback,
                      this, std::placeholders::_1));
    right_motor_torque_ref_subscriber_ =
        this->create_subscription<std_msgs::msg::Float32>(
            "/right_motor/throttle", 10,
            std::bind(
                &Curtis_ACF2A_Can_Driver_Node::throttleright_motorCallback,
                this, std::placeholders::_1));

    // Create the publishers
    left_motor_encoder_publisher_ =
        this->create_publisher<std_msgs::msg::Int64>(
            "/left_motor/encoder_count", 10);
    right_motor_encoder_publisher_ =
        this->create_publisher<std_msgs::msg::Int64>(
            "/right_motor/encoder_count", 10);
    left_motor_speed_publisher_ =
        this->create_publisher<std_msgs::msg::Int16>("/left_motor/speed", 10);
    right_motor_speed_publisher_ =
        this->create_publisher<std_msgs::msg::Int16>("/right_motor/speed", 10);
    left_motor_torque_publisher_ =
        this->create_publisher<std_msgs::msg::Float32>("/left_motor/torque",
                                                       10);
    right_motor_torque_publisher_ =
        this->create_publisher<std_msgs::msg::Float32>("/right_motor/torque",
                                                       10);
    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>(
            joint_state_topic_name_, 10);

    // Timer CAN read
    can_read_timer_ = this->create_wall_timer(
        std::chrono::microseconds(1000000 / can_read_rate_),
        std::bind(&Curtis_ACF2A_Can_Driver_Node::can_read_parse_loop, this));

    // Init motors
    left_motor.init(left_motor_id_, left_motor_swap_dir_);
    right_motor.init(right_motor_id_, right_motor_swap_dir_);

    // Config motors. WARNING: This function must be executed in parallel with
    // can_read_parse_loop() to capture the responses from the Driver.
    cfg_thread_ =
        std::thread(&Curtis_ACF2A_Can_Driver_Node::configMotors, this);
  }

  ~Curtis_ACF2A_Can_Driver_Node() {
    // Join threads on destructor
    if (cfg_thread_.joinable()) {
      cfg_thread_.join();
    }

    stop_thread_ = true;
    if (can_write_thread_.joinable()) {
      can_write_thread_.join();
    }
  }

private:
  std::thread cfg_thread_;
  void configMotors() {
    // config motor parameters
    left_motor.set_config_control_mode(left_motor_control_mode_);
    left_motor.config(log_level);

    right_motor.set_config_control_mode(right_motor_control_mode_);
    right_motor.config(log_level);

    // Start real-time thread for CAN writes
    can_write_thread_ =
        std::thread(&Curtis_ACF2A_Can_Driver_Node::can_write_loop, this);

    // ROS publish data
    publish_ros_data_timer_ = this->create_wall_timer(
        std::chrono::microseconds(1000000 / can_write_rate_),
        std::bind(&Curtis_ACF2A_Can_Driver_Node::publish_ros_data, this));
  }

  // Left motor throttle callback
  void throttleleft_motorCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    left_motor_torque_ref_ = msg->data;

    if (log_level == "verbose") {
      RCLCPP_INFO(this->get_logger(), "Left motor torque reference: %f",
                  left_motor_torque_ref_);
    }
  }

  // Right motor throttle callback
  void
  throttleright_motorCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    right_motor_torque_ref_ = msg->data;

    if (log_level == "verbose") {
      RCLCPP_INFO(this->get_logger(), "Right motor torque reference: %f",
                  right_motor_torque_ref_);
    }
  }

  // CAN read and parse
  void can_read_parse_loop() {
    static nmt_state last_NMT_state_right = COMM_TIMEOUT;
    static nmt_state last_NMT_state_left = COMM_TIMEOUT;

    // canopen_left.read_loop();
    // canopen_right.read_loop();

    // nmt_state left_motor_state = left_motor.parse_loop();
    // nmt_state right_motor_state = right_motor.parse_loop();
    nmt_state left_motor_state = left_motor.get_state();
    nmt_state right_motor_state = right_motor.get_state();

    if (left_motor_state == COMM_TIMEOUT) {
      // timeout left motor
      left_motor_torque_ref_ = 0;
    }
    if (right_motor_state == COMM_TIMEOUT) {
      // timeout right motor
      right_motor_torque_ref_ = 0;
    }

    if (log_level == "verbose") {
      if (right_motor_state != last_NMT_state_right) {
        RCLCPP_INFO(this->get_logger(), "Right motor state: %s",
                    right_motor.get_state_message());
      }
      if (left_motor_state != last_NMT_state_left) {
        RCLCPP_INFO(this->get_logger(), "Left motor state: %s",
                    left_motor.get_state_message());
      }

      last_NMT_state_right = right_motor_state;
      last_NMT_state_left = left_motor_state;
    }
  }

  // ROS publish drivers data to topics
  void publish_ros_data() {
    std_msgs::msg::Int64 encoder_msg;
    std_msgs::msg::Int16 speed_msg;
    std_msgs::msg::Float32 torque_msg;
    encoder_msg.data = left_motor.get_encoder();
    left_motor_encoder_publisher_->publish(encoder_msg);
    encoder_msg.data = right_motor.get_encoder();
    right_motor_encoder_publisher_->publish(encoder_msg);
    speed_msg.data = left_motor.get_speed();
    left_motor_speed_publisher_->publish(speed_msg);
    speed_msg.data = right_motor.get_speed();
    right_motor_speed_publisher_->publish(speed_msg);
    torque_msg.data = left_motor.get_torque();
    left_motor_torque_publisher_->publish(torque_msg);
    torque_msg.data = right_motor.get_torque();
    right_motor_torque_publisher_->publish(torque_msg);

    sensor_msgs::msg::JointState joint_msg;
    std_msgs::msg::Header header;

    header.stamp = this->get_clock()->now();
    joint_msg.header = header;

    joint_msg.name.resize(2);
    joint_msg.name[0] = left_wheel_joint_name_;
    joint_msg.name[1] = right_wheel_joint_name_;

    joint_msg.position.resize(2);
    joint_msg.position[0] =
        encoder2pos(left_motor.get_encoder(), encoder_max_counts, gear_ratio);
    /*joint_msg.position[0] = left_motor_encoder;*/
    joint_msg.position[1] =
        encoder2pos(right_motor.get_encoder(), encoder_max_counts, gear_ratio);
    /*joint_msg.position[1] = right_motor_encoder;*/

    joint_msg.velocity.resize(2);
    joint_msg.velocity[0] =
        motor_rpm_to_rads(left_motor.get_speed(), gear_ratio);
    joint_msg.velocity[1] =
        motor_rpm_to_rads(right_motor.get_speed(), gear_ratio);

    joint_msg.effort.resize(2);
    joint_msg.effort[0] = left_motor.get_torque();
    joint_msg.effort[1] = right_motor.get_torque();

    joint_state_publisher_->publish(joint_msg);
  }

  // CAN write and publish
  void can_write_loop() {
    rclcpp::Rate rate(CAN_WRITE_RATE_DEFAULT);

    while (!stop_thread_ && rclcpp::ok()) {
      left_motor.set_torque(left_motor_torque_ref_);
      right_motor.set_torque(right_motor_torque_ref_);
      rate.sleep();
    }
  }

  void on_shutdown_callback() {
    if ((log_level == "info") | (log_level == "verbose")) {
      RCLCPP_INFO(this->get_logger(),
                  "Shutting down. Setting throttle commands to zero.");
    }

    left_motor_torque_ref_ = 0;
    right_motor_torque_ref_ = 0;

    left_motor.set_torque(left_motor_torque_ref_);
    right_motor.set_torque(right_motor_torque_ref_);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Variables for convertion to SI and publish as JointState
  std::string joint_state_topic_name_, left_wheel_joint_name_,
      right_wheel_joint_name_;
  float gear_ratio;
  int64_t encoder_max_counts;

  // Variables to store the throttle value for each motor
  float left_motor_torque_ref_ = 0;
  float right_motor_torque_ref_ = 0;

  // Publishers for motor encoders
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr
      left_motor_encoder_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr
      right_motor_encoder_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr
      left_motor_speed_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr
      right_motor_speed_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      left_motor_torque_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      right_motor_torque_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;

  // Subscriber for motor throttle commands
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      left_motor_torque_ref_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      right_motor_torque_ref_subscriber_;

  std::thread can_write_thread_;
  bool stop_thread_;

  // Parameters variables
  uint16_t can_read_rate_ = 0;
  uint16_t can_write_rate_ = 0;
  uint8_t left_motor_id_ = 0;
  uint8_t right_motor_id_ = 0;
  uint8_t left_motor_swap_dir_ = 0;
  uint8_t right_motor_swap_dir_ = 0;
  int left_motor_control_mode_, right_motor_control_mode_;

  // Timer CAN read and parse
  rclcpp::TimerBase::SharedPtr can_read_timer_;

  // Timer CAN write
  rclcpp::TimerBase::SharedPtr can_write_timer_;

  // Timer publish ROS data
  rclcpp::TimerBase::SharedPtr publish_ros_data_timer_;

  // Debug level
  std::string log_level;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Curtis_ACF2A_Can_Driver_Node>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
