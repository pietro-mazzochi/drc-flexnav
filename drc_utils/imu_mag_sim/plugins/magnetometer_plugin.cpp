#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>

namespace gazebo
{
  class MagnetometerPlugin : public SensorPlugin
  {
  public:
    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
    {
      this->magnetometerSensor = std::dynamic_pointer_cast<sensors::MagnetometerSensor>(_sensor);
      if (!this->magnetometerSensor)
      {
        gzerr << "MagnetometerPlugin requires a MagnetometerSensor.\n";
        return;
      }

      if (!rclcpp::ok())
      {
        rclcpp::init(0, nullptr);
      }

      this->rosNode = rclcpp::Node::make_shared("magnetometer_plugin");
      this->magnetometerPublisher = this->rosNode->create_publisher<sensor_msgs::msg::MagneticField>("magnetometer/data", 10);

      this->updateConnection = this->magnetometerSensor->ConnectUpdated(std::bind(&MagnetometerPlugin::OnUpdate, this));
      this->rosSpinThread = std::thread(std::bind(&MagnetometerPlugin::Spin, this));
    }

  private:
    void OnUpdate()
    {
      sensor_msgs::msg::MagneticField msg;
      msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(this->magnetometerSensor->LastUpdateTime());
      msg.magnetic_field.x = this->magnetometerSensor->MagneticField().X();
      msg.magnetic_field.y = this->magnetometerSensor->MagneticField().Y();
      msg.magnetic_field.z = this->magnetometerSensor->MagneticField().Z();
      this->magnetometerPublisher->publish(msg);
    }

    void Spin()
    {
      rclcpp::spin(this->rosNode);
    }

    sensors::MagnetometerSensorPtr magnetometerSensor;
    event::ConnectionPtr updateConnection;
    rclcpp::Node::SharedPtr rosNode;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetometerPublisher;
    std::thread rosSpinThread;
  };

  GZ_REGISTER_SENSOR_PLUGIN(MagnetometerPlugin)
}
