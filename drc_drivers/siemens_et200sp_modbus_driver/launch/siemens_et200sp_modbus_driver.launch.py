import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    plc_ip_arg = DeclareLaunchArgument(
        "ip", default_value="192.168.10.1", description="IP address of the PLC"
    )
    modbus_port_arg = DeclareLaunchArgument(
        "port", default_value="502", description="Modbus port of the PLC"
    )
    output_topic_arg = DeclareLaunchArgument(
        "output_topic",
        default_value="et200sp",
        description="ROS2 topic to publish data incomming from PLC",
    )
    input_prefix_arg = DeclareLaunchArgument(
        "input_prefix",
        default_value="",
        description="ROS2 topic to subscribe data and send to PLC",
    )
    left_encoder_topic = DeclareLaunchArgument(
        "left_encoder_topic",
        default_value="/left_encoder",
        description="ROS2 topic to left encoder angular displacement",
    )
    right_encoder_topic = DeclareLaunchArgument(
        "right_encoder_topic",
        default_value="/right_encoder",
        description="ROS2 topic to right encoder angular displacement",
    )
    left_wheel_vel_topic = DeclareLaunchArgument(
        "left_wheel_vel_topic",
        default_value="/left_wheel_pid/measurement",
        description="ROS2 topic to left wheel angular velocity",
    )
    right_wheel_vel_topic = DeclareLaunchArgument(
        "right_wheel_vel_topic",
        default_value="/right_wheel_pid/measurement",
        description="ROS2 topic to right wheel angular velocity",
    )
    frequency = DeclareLaunchArgument(
        "freq",
        default_value="100",
#        default_value="80", # when use S7-1200
        description="ROS2 frequency (Hz) - read plc - publish topic",
    )

    return LaunchDescription(
        [
            plc_ip_arg,
            modbus_port_arg,
            output_topic_arg,
            input_prefix_arg,
            left_encoder_topic,
            right_encoder_topic,
            left_wheel_vel_topic,
            right_wheel_vel_topic,
            frequency,
            Node(
                package="siemens_et200sp_modbus_driver",
                executable="siemens_et200sp_modbus_driver_node",
                name="siemens_et200sp_modbus_driver_node",
                output="screen",
                parameters=[
                    {"device_ip": LaunchConfiguration("ip")},
                    {"modbus_port": LaunchConfiguration("port")},
                    {"output_topic": LaunchConfiguration("output_topic")},
                    {"input_prefix": LaunchConfiguration("input_prefix")},
                    {"left_encoder_topic": LaunchConfiguration("left_encoder_topic")},
                    {"right_encoder_topic": LaunchConfiguration("right_encoder_topic")},
                    {
                        "left_wheel_vel_topic": LaunchConfiguration(
                            "left_wheel_vel_topic"
                        )
                    },
                    {
                        "right_wheel_vel_topic": LaunchConfiguration(
                            "right_wheel_vel_topic"
                        )
                    },
                    {"freq": LaunchConfiguration("freq")},
                ],
            ),
        ]
    )
