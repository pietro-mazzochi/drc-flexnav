#define _USE_MATH_DEFINES
#include <cmath>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "siemens_et200sp_modbus_driver.hpp"
#include "siemens_et200sp_modbus_driver_msgs/msg/siemens_etsp_modbus_driver.hpp"


Siemens_ET200SP_Modbus_Driver siemens_et200sp;

class Siemens_ET200SP_Modbus_Driver_Node : public rclcpp::Node
{
public:

    Siemens_ET200SP_Modbus_Driver_Node() : Node("siemens_et200sp_modbus_driver_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing PLC driver ...");

        plc_ip_ = this->declare_parameter<std::string>("plc_ip_", "192.168.10.1");
        modbus_communication_port_ = this->declare_parameter<int>("modbus_communication_port_", 502);
        output_topic_ = this->declare_parameter<std::string>("output_topic", "et200sp");
        input_prefix_ = this->declare_parameter<std::string>("input_prefix", "");
        frequency_ = this->declare_parameter<int>("freq");
        gear_ratio_ = this->declare_parameter<double>("gear_ratio_", 35.91);
        encoder_max_counts_ = this->declare_parameter<int>("encoder_max_counts", 40000);
        left_encoder_topic_ = this->declare_parameter<std::string>("left_encoder_topic", "/left_encoder");
        right_encoder_topic_ = this->declare_parameter<std::string>("right_encoder_topic", "/right_encoder");
        left_wheel_vel_topic_ = this->declare_parameter<std::string>("left_wheel_vel_topic", "/left_wheel_vel");
        right_wheel_vel_topic_ = this->declare_parameter<std::string>("right_wheel_vel_topic", "/right_wheel_vel");

        RCLCPP_WARN(this->get_logger(), "Parameter 'modbus ip': %s", plc_ip_.c_str());
        RCLCPP_WARN(this->get_logger(), "Parameter 'modbus port': %i", modbus_communication_port_);
        RCLCPP_WARN(this->get_logger(), "Parameter 'output_topic': %s", output_topic_.c_str());
        RCLCPP_WARN(this->get_logger(), "Parameter 'input_prefix': %s", input_prefix_.c_str());
        RCLCPP_WARN(this->get_logger(), "Parameter 'frequency': %i Hz", frequency_);
        RCLCPP_WARN(this->get_logger(), "Parameter 'gear_ratio': %f", gear_ratio_);
        RCLCPP_WARN(this->get_logger(), "Parameter 'encoder_max_counts': %i", encoder_max_counts_);

        // do   {
                // RCLCPP_WARN(this->get_logger(), "\nTrying connect to %s:%i\n", plc_ip_.c_str(), modbusPort);
                // siemens_et200sp.connect(plc_ip_.c_str(), modbusPort, 1000000/frequency_); // parameter timeout in microseconds
                // siemens_et200sp.syncro();
            //  } while (!siemens_et200sp.is_connected());

        // Publicadores para os encoders
 
        siemens_et200sp_data_pub_ = this->create_publisher<siemens_et200sp_modbus_driver_msgs::msg::SiemensEtspModbusDriver>(output_topic_, 10);
        left_encoder_pub_ = this->create_publisher<std_msgs::msg::Float64>(left_encoder_topic_, 10);
        right_encoder_pub_ = this->create_publisher<std_msgs::msg::Float64>(right_encoder_topic_, 10);
        left_wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(left_wheel_vel_topic_, 10);
        right_wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(right_wheel_vel_topic_, 10);

        // Temporizador para publicação periódica dos valores dos encoders (10ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000/frequency_),
            std::bind(&Siemens_ET200SP_Modbus_Driver_Node::publish_plc_values, this));

        const std::string green_front_light_topic_     = input_prefix_ + "/green_front_light";
        const std::string green_red_light_topic_      = input_prefix_ + "/green_red_light";
        const std::string red_front_light_topic_       = input_prefix_ + "/red_front_light";
        const std::string red_rear_light_topic_        = input_prefix_ + "/red_rear_light";
        const std::string left_light_topic_          = input_prefix_ + "/left_light";
        const std::string right_light_topic_         = input_prefix_ + "/right_light";
        const std::string buzzer_topic_              = input_prefix_ + "/buzzer";
        const std::string load_unload_timeout_topic_ = input_prefix_ + "/load_unload_timeout_seconds";
        const std::string load_command_topic_        = input_prefix_ + "/load_command";
        const std::string ls_config_topic_           = input_prefix_ + "/ls_config";

        RCLCPP_INFO(this->get_logger(), "SUBSCRIBING %s", green_front_light_topic_.c_str()      );
        RCLCPP_INFO(this->get_logger(), "SUBSCRIBING %s", green_red_light_topic_.c_str()       );
        RCLCPP_INFO(this->get_logger(), "SUBSCRIBING %s", red_front_light_topic_.c_str()        );
        RCLCPP_INFO(this->get_logger(), "SUBSCRIBING %s", red_rear_light_topic_.c_str()         );
        RCLCPP_INFO(this->get_logger(), "SUBSCRIBING %s", left_light_topic_.c_str()           );
        RCLCPP_INFO(this->get_logger(), "SUBSCRIBING %s", right_light_topic_.c_str()          );
        RCLCPP_INFO(this->get_logger(), "SUBSCRIBING %s", buzzer_topic_.c_str()               );
        RCLCPP_INFO(this->get_logger(), "SUBSCRIBING %s", load_unload_timeout_topic_.c_str()  );
        RCLCPP_INFO(this->get_logger(), "SUBSCRIBING %s", load_command_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "SUBSCRIBING %s", ls_config_topic_.c_str());

        // Assinantes para os tópicos de controle das luzes
        green_front_light_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            green_front_light_topic_, 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                siemens_et200sp.set_front_green_light_state((robot_signaling_status_t)msg->data);
                RCLCPP_INFO(this->get_logger(), "Changing Front Green Light status to %i", msg->data);
            });
        green_rear_light_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            green_red_light_topic_, 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                siemens_et200sp.set_rear_green_light_state((robot_signaling_status_t)msg->data);
                RCLCPP_INFO(this->get_logger(), "Changing Rear Green Light status to %i", msg->data);
            });
        red_front_light_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            red_front_light_topic_, 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                siemens_et200sp.set_front_red_light_state((robot_signaling_status_t)msg->data);
                RCLCPP_INFO(this->get_logger(), "Changing Front Red Light status to %i", msg->data);
            });
        red_rear_light_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            red_rear_light_topic_, 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                siemens_et200sp.set_rear_red_light_state((robot_signaling_status_t)msg->data);
                RCLCPP_INFO(this->get_logger(), "Changing Rear Red Light status to %i", msg->data);
            });
        left_light_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            left_light_topic_, 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                siemens_et200sp.set_left_light_state((robot_signaling_status_t)msg->data);
                RCLCPP_INFO(this->get_logger(), "Changing Left Light status to %i", msg->data);
            });
        right_light_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            right_light_topic_, 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                siemens_et200sp.set_right_light_state((robot_signaling_status_t)msg->data);
                RCLCPP_INFO(this->get_logger(), "Changing Right Light status to %i", msg->data);
            });
        buzzer_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            buzzer_topic_, 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                siemens_et200sp.set_buzzer_state((robot_signaling_status_t)msg->data);
                RCLCPP_INFO(this->get_logger(), "Changing Buzzer status to %i", msg->data);
            });
        load_unload_timeout_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            load_unload_timeout_topic_, 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                siemens_et200sp.set_load_unload_timeout_seconds((uint8_t)msg->data);
                RCLCPP_INFO(this->get_logger(), "Changing Load/Unload timeout to %i", msg->data);
            });
        load_command_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            load_command_topic_, 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                siemens_et200sp.send_command((robot_command_t)msg->data);
                RCLCPP_INFO(this->get_logger(), "Sending load/unload command %02xH", msg->data);
            });
        ls_config_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            ls_config_topic_, 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                siemens_et200sp.set_laser_scanner_config((uint8_t)msg->data);
                RCLCPP_INFO(this->get_logger(), "Seting LS zone to %02xH", msg->data);
            });
        // rear_ls_config_sub_      = this->create_subscription<std_msgs::msg::Int32>(
        //                                 rear_ls_config_topic_     , 10,
        //                                 [this](const std_msgs::msg::Int32::SharedPtr msg)
        //                                     {
        //                                     siemens_et200sp.setRearLaserScannerConfig((uint8_t)msg->data);
        //                                     RCLCPP_INFO(this->get_logger(), "Seting rear LS zone to %02xH", msg->data);
        //                                     });
    }

private:
    std::string plc_ip_;
    std::string output_topic_, input_prefix_, left_encoder_topic_, right_encoder_topic_, left_wheel_vel_topic_, right_wheel_vel_topic_;
    int frequency_;
    int modbus_communication_port_;
    double gear_ratio_;
    int encoder_max_counts_;
    int connection_count_ = 0;
    bool is_connected_flag_ = false;

    void publish_plc_values()
    {

        auto plc_msg = siemens_et200sp_modbus_driver_msgs::msg::SiemensEtspModbusDriver();
//        siemens_et200sp.syncro();
        if (!siemens_et200sp.is_connected() && connection_count_ == 0) 
            {
            is_connected_flag_ = false;
            RCLCPP_INFO(this->get_logger(), "Trying connect to %s:%i", plc_ip_.c_str(), modbus_communication_port_);
            siemens_et200sp.disconnect();
            siemens_et200sp.connect(plc_ip_.c_str(), modbus_communication_port_, 1000000/frequency_); // parameter timeout in microseconds
            connection_count_ = 100;
            } 
        else
            {
            siemens_et200sp.syncro();
            }

        if (siemens_et200sp.is_connected())
           {
            if (!is_connected_flag_)
               {
               is_connected_flag_ = true;
               RCLCPP_INFO(this->get_logger(), "Connected to %s:%i", plc_ip_.c_str(), modbus_communication_port_);
               }
           }

        if (connection_count_ > 0) connection_count_ --;

        static bool flag = 0;
        static int64_t last_left_encoder;
        static int64_t last_right_encoder;
        static int64_t last_timestamp;

        plc_msg.header.stamp.sec = now().nanoseconds() / 1000000000;
        plc_msg.header.stamp.nanosec = now().nanoseconds() % 1000000000;
        // 
        if(flag == 0){
            last_left_encoder = 0; 
            last_right_encoder = 0;
            last_timestamp = siemens_et200sp.get_message_timestamp_ms();
            flag = 1;
        }
        
        plc_msg.time_stamp = siemens_et200sp.get_message_timestamp_ms();

        int64_t delta_t = plc_msg.time_stamp - last_timestamp;
        last_timestamp = plc_msg.time_stamp;

        plc_msg.delta_t                   = delta_t;
        plc_msg.left_encoder              = siemens_et200sp.get_left_encoder_counts();
        plc_msg.right_encoder             = siemens_et200sp.get_right_encoder_counts();
        plc_msg.left_encoder_rad          = convert_count_to_rad(plc_msg.left_encoder, encoder_max_counts_);
        plc_msg.right_encoder_rad         = convert_count_to_rad(plc_msg.right_encoder, encoder_max_counts_);
        plc_msg.left_encoder_rpm          = convert_count_to_rpm(plc_msg.left_encoder, last_left_encoder, encoder_max_counts_, delta_t);
        plc_msg.right_encoder_rpm         = convert_count_to_rpm(plc_msg.right_encoder, last_right_encoder, encoder_max_counts_, delta_t);
        plc_msg.joy_up                    = siemens_et200sp.get_joystick_up();
        plc_msg.joy_down                  = siemens_et200sp.get_joystick_down();
        plc_msg.joy_left                  = siemens_et200sp.get_joystick_left();
        plc_msg.joy_right                 = siemens_et200sp.get_joystick_right();
        // plc_msg.docked_front              = (siemens_et200sp.getDockingStatus() & siemens_et200sp.ROBOT_DOCKED_FRONT) > 0;
        // plc_msg.docked_rear               = (siemens_et200sp.getDockingStatus() & siemens_et200sp.ROBOT_DOCKED_REAR) > 0;
        plc_msg.charging                  = siemens_et200sp.is_charging();
        plc_msg.front_ls_violation        = siemens_et200sp.get_front_laser_scanner_active_zone();
        plc_msg.rear_ls_violation         = siemens_et200sp.get_rear_laser_scanner_active_zone();
        plc_msg.reset_wait                = siemens_et200sp.is_waiting_user_reset_error_codes();
        plc_msg.undefined_error           = siemens_et200sp.has_undefined_error();
        plc_msg.restart_detected          = siemens_et200sp.has_initial_error_code();
        plc_msg.left_motor_error          = siemens_et200sp.has_left_motor_error();
        plc_msg.right_motor_error         = siemens_et200sp.has_right_motor_error();
        plc_msg.table_error               = siemens_et200sp.has_table_motor_error();
        plc_msg.load_error                = siemens_et200sp.has_loading_error();
        plc_msg.unload_error              = siemens_et200sp.has_unloading_error();
        plc_msg.battery_error             = siemens_et200sp.has_battery_error();
        plc_msg.docking_error             = siemens_et200sp.has_docking_error();
        plc_msg.auto_sw                   = siemens_et200sp.is_automatic_mode_selected();
        plc_msg.man_sw                    = siemens_et200sp.is_manual_mode_selected();
        plc_msg.reset_sw                  = siemens_et200sp.is_reset_button_pressed();
        plc_msg.table_front_load_sensor   = siemens_et200sp.get_front_load_sensor_state();
        plc_msg.table_rear_load_sensor    = siemens_et200sp.get_rear_load_sensor_state();
        plc_msg.joy_connected             = siemens_et200sp.is_joystick_connected();
        plc_msg.is_emergency_lock         = siemens_et200sp.is_emergency_lock();
        plc_msg.loaded                    = siemens_et200sp.is_loaded();
        plc_msg.communication_faults      = siemens_et200sp.get_error_count();
        plc_msg.plc_connected             = siemens_et200sp.is_connected();
        plc_msg.last_plc_read_write_time  = siemens_et200sp.get_last_read_write_time();

        last_left_encoder = plc_msg.left_encoder; 
        last_right_encoder = plc_msg.right_encoder;
        
        siemens_et200sp_data_pub_->publish(plc_msg);

        std_msgs::msg::Float64 left_encoder_msg, right_encoder_msg;

        left_encoder_msg.data = plc_msg.left_encoder_rad;
        right_encoder_msg.data = plc_msg.right_encoder_rad;
        
        left_encoder_pub_->publish(left_encoder_msg);
        right_encoder_pub_->publish(right_encoder_msg);

        std_msgs::msg::Float64MultiArray left_wheel_vel_msg, right_wheel_vel_msg;

        left_wheel_vel_msg.data.resize(1);
        right_wheel_vel_msg.data.resize(1);

        left_wheel_vel_msg.data[0] = convert_count_to_rad_s(plc_msg.left_encoder, last_left_encoder, encoder_max_counts_, delta_t); // <<= MODIFY to use delta_t
        right_wheel_vel_msg.data[0] = convert_count_to_rad_s(plc_msg.right_encoder, last_right_encoder, encoder_max_counts_, delta_t); // <<== MODIFY to use delta_t
        
        left_wheel_vel_pub_->publish(left_wheel_vel_msg);
        right_wheel_vel_pub_->publish(right_wheel_vel_msg);

        // pub 
    }

    // Membros para publicação e assinatura dos tópicos
    rclcpp::Publisher<siemens_et200sp_modbus_driver_msgs::msg::SiemensEtspModbusDriver>::SharedPtr siemens_et200sp_data_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_encoder_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_encoder_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_wheel_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_wheel_vel_pub_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr green_front_light_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr green_rear_light_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr red_front_light_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr red_rear_light_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_light_sub_;         
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_light_sub_;        
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr buzzer_sub_;             
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr load_unload_timeout_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr load_command_sub_;       
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ls_config_sub_;    
//    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rear_ls_config_sub_;     

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Siemens_ET200SP_Modbus_Driver_Node>());
    rclcpp::shutdown();
    return 0;
}
