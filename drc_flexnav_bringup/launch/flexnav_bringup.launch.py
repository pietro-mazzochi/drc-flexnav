from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    GroupAction
)
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python import get_package_share_path
from launch.conditions import IfCondition, UnlessCondition

def get_paths():
    """Get all launch and configuration file paths"""
    # Launch paths
    description_path = get_package_share_path('drc_flexnav_description') / 'launch' / 'flexnav.launch.xml'
    controller_path = get_package_share_path('drc_flexnav_bringup') / 'launch' / 'mrac_controller.launch.xml'
    velocity_converter_path = get_package_share_path('base2joint_velocity_converter') / 'launch' / 'velocity_converter_node.launch.xml'
    siemens_et200sp_modbus_driver_path = get_package_share_path('siemens_et200sp_modbus_driver') / 'launch' / 'siemens_et200sp_modbus_driver.launch.py'
    motor_launch_path = get_package_share_path("curtis_acf2a_can_driver") / "launch" / "curtis_acf2a_can_driver.launch.xml"
    lidar_path = get_package_share_path("livox_ros_driver2") / "launch_ROS2" / "lidar.launch.py"
    imu_path = get_package_share_path("drc_flexnav_bringup") / "launch" / "imu_mag_broadcaster.launch.xml"
    laser_scanner_controller_path = get_package_share_path("drc_flexnav_controllers") / "launch" / "laser_scanner_controller.launch.xml"
    odometry_broadcaster_path = get_package_share_path("drc_flexnav_controllers") / "launch" / "odometry_broadcaster.launch.xml"
    robot_load_controller_path = get_package_share_path("drc_flexnav_controllers") / "launch" / "robot_load_controller.launch.xml"
    robot_signaling_controller_path = get_package_share_path("drc_flexnav_controllers") / "launch" / "robot_signaling_controller.launch.xml"
    siemens_et200sp_broadcaster_path = get_package_share_path("drc_flexnav_controllers") / "launch" / "siemens_et200sp_broadcaster.launch.xml"
    kalman_filter_path = get_package_share_path("drc_flexnav_pose_estimation") / "launch" / "extended_kalman_filter.launch.py"
    industrial_joystick_path = get_package_share_path("industrial_joystick_controller") / "launch" / "industrial_joystick_control.launch.xml"

    # Configuration paths
    odometry_config_path = get_package_share_path("drc_flexnav_bringup") / "config" / "odometry_publisher.yaml"
    velocity_converter_config = get_package_share_path("drc_flexnav_bringup") / "config" / "joint_velocity_converter_real_robot.yaml"
    velocity_converter_config_hw_interface = get_package_share_path("drc_flexnav_bringup") / "config" / "joint_velocity_converter_real_robot_hw_interface.yaml"
    curtis_config_path = get_package_share_path("curtis_acf2a_can_driver") / "config" / "curtis_acf2a_can_driver_params.yaml"
    controller_manager_config_path = get_package_share_path("drc_flexnav_bringup") / "config" / "controller_manager.yaml"
    rviz2_config = get_package_share_path("drc_flexnav_bringup") / "rviz" / "flexnav1500.rviz"
    lidar_config = get_package_share_path("drc_flexnav_bringup") / "config" / "lidar_config.json"
    laser_scanner_config = get_package_share_path("drc_flexnav_controllers") / "config" / "laser_scanner_controller.yaml"
    odometry_broadcaster_config = get_package_share_path("drc_flexnav_controllers") / "config" / "odometry_broadcaster.yaml"
    robot_load_config = get_package_share_path("drc_flexnav_controllers") / "config" / "robot_load_controller.yaml"
    robot_signaling_config = get_package_share_path("drc_flexnav_controllers") / "config" / "robot_signaling_controller.yaml"
    siemens_et200sp_broadcaster_config = get_package_share_path("drc_flexnav_controllers") / "config" / "siemens_et200sp_broadcaster.yaml"
    kalman_filter_config = get_package_share_path("drc_flexnav_pose_estimation") / "config" / "extended_kalman_filter.yaml"
    
    return {
        'launch': {
            'description': description_path,
            'controller': controller_path,
            'velocity_converter': velocity_converter_path,
            'siemens_driver': siemens_et200sp_modbus_driver_path,
            'motor': motor_launch_path,
            'lidar': lidar_path,
            'imu': imu_path,
            'laser_scanner': laser_scanner_controller_path,
            'odom_broadcaster': odometry_broadcaster_path,
            'robot_load': robot_load_controller_path,
            'robot_signaling': robot_signaling_controller_path,
            'siemens_et200sp_broadcaster': siemens_et200sp_broadcaster_path,
            'kalman_filter': kalman_filter_path,
            'industrial_joystick': industrial_joystick_path
        },
        'config': {
            'odometry': odometry_config_path,
            'velocity_converter': velocity_converter_config,
            'velocity_converter_hw': velocity_converter_config_hw_interface,
            'curtis': curtis_config_path,
            'controller_manager': controller_manager_config_path,
            'rviz': rviz2_config,
            'lidar': lidar_config,
            'laser_scanner_config' : laser_scanner_config,
            'odometry_broadcaster_config' : odometry_broadcaster_config,
            'robot_load_config' : robot_load_config,
            'robot_signaling_config' : robot_signaling_config,
            'siemens_et200sp_config' : siemens_et200sp_broadcaster_config,
            'kalman_filter_config':kalman_filter_config
        }
    }

def declare_launch_arguments():
    paths = get_paths()
    """Define all launch arguments"""
    
    return [
        DeclareLaunchArgument("robot", default_value="flexnav1500"),
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("ignition", default_value="False"),
        DeclareLaunchArgument("odom_config", default_value=str(paths['config']['odometry'])),
        DeclareLaunchArgument("controller_manager_config", default_value=str(paths['config']['controller_manager'])),
        DeclareLaunchArgument("sim", default_value="False"),
        DeclareLaunchArgument("use_hw_interface", default_value="False"),
        DeclareLaunchArgument("odom", default_value="True", description="Odometry flag"),
        DeclareLaunchArgument("laser_scanner_controller_config", default_value=str(paths['config']['laser_scanner_config']) ),
        DeclareLaunchArgument("odometry_broadcaster_config", default_value=str(paths['config']['odometry_broadcaster_config']) ),
        DeclareLaunchArgument("robot_load_controller_config", default_value=str(paths['config']['robot_load_config']) ),
        DeclareLaunchArgument("robot_signaling_controller_config", default_value=str(paths['config']['robot_signaling_config']) ),
        DeclareLaunchArgument("siemens_broadcaster_config", default_value=str(paths['config']['siemens_et200sp_config']) ),
        DeclareLaunchArgument("kalman_filter_config", default_value=str(paths['config']['kalman_filter_config']) ),


        # Livox configurations
        DeclareLaunchArgument("xfer_format", default_value="0", 
            description="Transfer format (0-Pointcloud2, 1-customized)"),
        DeclareLaunchArgument("multi_topic", default_value="1",
            description="Multi-topic configuration (0-All LiDARs share same topic, 1-One LiDAR one topic)"),
        DeclareLaunchArgument("data_src", default_value="0",
            description="Data source configuration (0-lidar, others-invalid)"),
        DeclareLaunchArgument("publish_freq", default_value="10.0",
            description="Publish frequency"),
        DeclareLaunchArgument("output_type", default_value="0",
            description="Output type"),
        DeclareLaunchArgument("lvx_frame_id", default_value="livox_frame",
            description="Frame ID"),
        DeclareLaunchArgument("lvx_file_path", default_value="/home/livox/livox_test.lvx",
            description="Path to LVX file"),
        DeclareLaunchArgument("cmdline_bd_code", default_value="livox0000000001",
            description="Command line board code"),
        DeclareLaunchArgument("lidar_config", default_value=str(paths['config']['lidar']),
            description="Path to the user config file")
    ]

def generate_launch_description():
    """Generate the launch description with all components"""
    paths = get_paths()
    
    ld = LaunchDescription(declare_launch_arguments())
    
    # Grupo 1: Componentes Base (t=0s)
    base_components = GroupAction([
        # Robot description
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(paths['launch']['description'])),
            launch_arguments={
                "robot": LaunchConfiguration("robot"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "ignition": LaunchConfiguration("ignition"),
                "sim": LaunchConfiguration("sim"),
                "use_hw_interface": LaunchConfiguration("use_hw_interface")
            }.items()
        ),
        # Controller manager
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            remappings=[("/controller_manager/robot_description", "/robot_description")],
            condition=IfCondition(LaunchConfiguration("use_hw_interface")),
            parameters=[LaunchConfiguration("controller_manager_config")]
        ),
        # Velocity converter
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(paths['launch']['velocity_converter'])),
            condition=IfCondition(LaunchConfiguration("use_hw_interface")),
            launch_arguments={"config_file": str(paths['config']['velocity_converter_hw'])}.items()
        )
    ])

    # Controller node
    controller_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(paths['launch']['controller'])),
        condition=UnlessCondition(LaunchConfiguration("sim"))
    )

    controller_node_sim = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(paths['launch']['controller'])),
        condition=IfCondition(LaunchConfiguration("sim"))
    )

    siemens_driver = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(paths['launch']['siemens_driver'])),
            condition=UnlessCondition(LaunchConfiguration("use_hw_interface")),
            launch_arguments={"ip": "192.168.10.1"}.items()
        )
    ])

    siemens_hw_interface = GroupAction([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(paths['launch']['laser_scanner'])),
            condition=IfCondition(LaunchConfiguration("use_hw_interface"))
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(paths['launch']['odom_broadcaster'])),
            condition=IfCondition(LaunchConfiguration("use_hw_interface"))
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(paths['launch']['robot_load'])),
            condition=IfCondition(LaunchConfiguration("use_hw_interface"))
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(paths['launch']['robot_signaling'])),
            condition=IfCondition(LaunchConfiguration("use_hw_interface"))
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(paths['launch']['siemens_et200sp_broadcaster'])),
            condition=IfCondition(LaunchConfiguration("use_hw_interface"))
        ),
    ])

    curtis_driver = GroupAction([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(paths['launch']['motor'])),
            condition=UnlessCondition(LaunchConfiguration("use_hw_interface")),
            launch_arguments={
                "config_file": str(paths['config']['curtis']),
                "velocity_converter_config": str(paths['config']['velocity_converter'])
            }.items()
        )
    ])

    lidar = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(paths['launch']['lidar'])),
            launch_arguments={
                "publish_freq": LaunchConfiguration("publish_freq"),
                "frame_id": LaunchConfiguration("lvx_frame_id"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "user_config_path": LaunchConfiguration("lidar_config")
            }.items()
        )
    ])

    imu = GroupAction([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(paths['launch']['imu']))
        )
    ])

    odometry = GroupAction([
        Node(
            package="odometry_arc",
            executable="odometry_publisher",
            name="odometry_publisher",
            condition=IfCondition(PythonExpression([
                "('", LaunchConfiguration("odom"), "' == 'true' or '",
                LaunchConfiguration("odom"), "' == 'True') and not ('",
                LaunchConfiguration("use_hw_interface"), "' == 'true' or '",
                LaunchConfiguration("use_hw_interface"), "' == 'True')"
            ])),
            parameters=[
                LaunchConfiguration("odom_config"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"use_joint_states": False}
            ]
        ),
    ])

    kalman = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(paths['launch']['kalman_filter'])),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items()
        )
    ])

    industrial_joystick = GroupAction([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(paths['launch']['industrial_joystick']))
        )
    ])

    ld.add_action(base_components)
    ld.add_action(controller_node_sim)
    ld.add_action(TimerAction(period=2.0, actions=[controller_node]))
    ld.add_action(TimerAction(period=3.0, actions=[curtis_driver]))
    ld.add_action(TimerAction(period=5.0, actions=[siemens_driver]))
    ld.add_action(TimerAction(period=5.0, actions=[odometry]))
    ld.add_action(TimerAction(period=3.0, actions=[siemens_hw_interface]))
    ld.add_action(TimerAction(period=8.0, actions=[imu]))
    ld.add_action(TimerAction(period=10.0, actions=[kalman]))
    ld.add_action(TimerAction(period=10.0, actions=[lidar]))
    ld.add_action(TimerAction(period=10.0, actions=[industrial_joystick]))

    return ld
