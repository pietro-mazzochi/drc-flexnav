from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python import get_package_share_path
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart


def generate_launch_description():

    # launcher paths
    description_path = (
        get_package_share_path("drc_flexnav_description")
        / "launch"
        / "flexnav.launch.xml"
    )
    controller_path = (
        get_package_share_path("drc_flexnav_bringup")
        / "launch"
        / "joint_effort_controller.launch.xml"
    )
    velocity_converter_path = (
        get_package_share_path("base2joint_velocity_converter")
        / "launch"
        / "velocity_converter_node.launch.xml"
    )
    # amrplc_path = (
    #     get_package_share_path("siemens_et200sp_modbus_driver")
    #     / "launch"
    #     / "siemens_et200sp_modbus_driver.launch.py"
    # )
    # TODO: change the motor node for hardware interfaces
    motor_launch_path = (
        get_package_share_path("curtis_acf2a_can_driver")
        / "launch"
        / "curtis_acf2a_can_driver.launch.xml"
    )
    lidar_path = (
        get_package_share_path("livox_ros_driver2") / "launch_ROS2" / "lidar.launch.py"
    )

    imu_path = (
        get_package_share_path("openzen_driver") / "launch" / "openzen_lpms.launch.py"
    )

    # configuration paths
    odometry_config_path = str(
        get_package_share_path("drc_flexnav_bringup")
        / "config"
        / "odometry_publisher.yaml"
    )
    velocity_converter_config = (
        get_package_share_path("drc_flexnav_bringup")
        / "config"
        / "joint_velocity_converter_real_robot.yaml"
    )
    velocity_converter_config_hw_interface = (
        get_package_share_path("drc_flexnav_bringup")
        / "config"
        / "joint_velocity_converter_real_robot_hw_interface_with_pid.yaml"
    )

    curtis_config_path = (
        get_package_share_path("curtis_acf2a_can_driver")
        / "config"
        / "curtis_acf2a_can_driver_params.yaml"
    )
    controller_manager_config_path = str(
        get_package_share_path("drc_flexnav_bringup")
        / "config"
        / "controller_manager.yaml"
    )
    rviz2_config = str(
        get_package_share_path("drc_flexnav_bringup") / "rviz" / "flexnav1500.rviz"
    )

    joint_velocity_broadcaster_config_path = str(get_package_share_path("base2joint_velocity_converter") / "config" / "joint_vel_broadcaster.yaml")

    joint_velocity_broadcaster_path = str(get_package_share_path("base2joint_velocity_converter") / "launch" / "joint_velocity_broadcaster_node.launch.xml" )

    wheelset_pids_config_path = str(get_package_share_path("drc_flexnav_bringup") / "config" / "flexnav1500_pid_controller.yaml" )

    wheelset_pids_path = str(get_package_share_path("drc_flexnav_bringup") / "launch" / "flexnav1500_digital_pid_controller.launch.xml" )

    declare_arguments = [
        DeclareLaunchArgument("robot", default_value="flexnav1500"),
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("ignition", default_value="False"),
        DeclareLaunchArgument("odom_config", default_value=odometry_config_path),
        DeclareLaunchArgument(
            "controller_manager_config", default_value=controller_manager_config_path
        ),
        DeclareLaunchArgument("sim", default_value="False"),
        DeclareLaunchArgument("use_hw_interface", default_value="False"),
        DeclareLaunchArgument(
            "odom", default_value="True", description="Odometry flag"
        ),
        # livox configurations
        DeclareLaunchArgument(
            "xfer_format",
            default_value="0",
            description="Transfer format (0-Pointcloud2, 1-customized)",
        ),
        DeclareLaunchArgument(
            "multi_topic",
            default_value="1",
            description="Multi-topic configuration (0-All LiDARs share same topic, 1-One LiDAR one topic)",
        ),
        DeclareLaunchArgument(
            "data_src",
            default_value="0",
            description="Data source configuration (0-lidar, others-invalid)",
        ),
        DeclareLaunchArgument(
            "publish_freq", default_value="10.0", description="Publish frequency"
        ),
        DeclareLaunchArgument(
            "output_type", default_value="0", description="Output type"
        ),
        DeclareLaunchArgument(
            "lvx_frame_id", default_value="livox_frame", description="Frame ID"
        ),
        DeclareLaunchArgument(
            "lvx_file_path",
            default_value="/home/livox/livox_test.lvx",
            description="Path to LVX file",
        ),
        DeclareLaunchArgument(
            "cmdline_bd_code",
            default_value="livox0000000001",
            description="Command line board code",
        ),
        DeclareLaunchArgument(
            "lidar_config",
            default_value=str(
                get_package_share_path("drc_flexnav_bringup")
                / "config"
                / "lidar_config.json"
            ),
            description="Path to the user config file",
        ),
    ]
    # robot description launch
    flexnav_description = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(description_path)),
        launch_arguments={
            "robot": LaunchConfiguration("robot"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "ignition": LaunchConfiguration("ignition"),
            "sim": LaunchConfiguration("sim"),
            "use_hw_interface": LaunchConfiguration("use_hw_interface"),
        }.items(),
    )
    # controller manager node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        condition=IfCondition(LaunchConfiguration("use_hw_interface")),
        parameters=[LaunchConfiguration("controller_manager_config")],
    )

    # controller node launch
    controller_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(controller_path)),
        condition=UnlessCondition(LaunchConfiguration("sim")),
    )

    controller_node_delayed = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[TimerAction(period=2.0, actions=[controller_node])],
        ),
    )

    controller_node_sim = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(controller_path)),
        condition=IfCondition(LaunchConfiguration("sim")),
    )

    # velocity converter node
    velocity_converter_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(velocity_converter_path)),
        condition=IfCondition(LaunchConfiguration("use_hw_interface")),
        launch_arguments={"config_file": str(velocity_converter_config_hw_interface)}.items(),
    )

    joint_velocity_broadcaster_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(joint_velocity_broadcaster_path),
        launch_arguments={"config_file": joint_velocity_broadcaster_config_path}.items(),
    )

    wheelset_pid_nodes = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(wheelset_pids_path),
        launch_arguments={"config_file": wheelset_pids_config_path}.items(),        
    )


    odometry_node = Node(
        package="odometry_arc",
        executable="odometry_publisher",
        name="odometry_publisher",
        condition=IfCondition(PythonExpression(["('",LaunchConfiguration("odom"),"' == 'true' or '",LaunchConfiguration("odom"),"' == 'True') \
                and not ('", LaunchConfiguration("use_hw_interface"),"' == 'true' or '",LaunchConfiguration("use_hw_interface"),"' == 'True')"])),
        parameters=[
            LaunchConfiguration("odom_config"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"use_joint_states": False},
        ],
    )

    odometry_node_joint_states = Node(
        package="odometry_arc",
        executable="odometry_publisher",
        name="odometry_publisher",
        condition=IfCondition(PythonExpression(["('",LaunchConfiguration("odom"),"' == 'true' or '",LaunchConfiguration("odom"),"' == 'True') \
                and ('", LaunchConfiguration("use_hw_interface"),"' == 'true' or '",LaunchConfiguration("use_hw_interface"),"' == 'True')"])),
        parameters=[
            LaunchConfiguration("odom_config"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"use_joint_states": True},
            {"use_joint_vel": True},
        ],
    )

    # amrplc launch
    # amrplc = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(str(amrplc_path)),
    #     launch_arguments={"ip": "192.168.10.1"}.items(),
    # )

    # imu launch
    imu_openzen = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(imu_path)),
    )

    # curtis driver node
    motor_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(motor_launch_path)),
        condition=UnlessCondition(LaunchConfiguration("use_hw_interface")),
        launch_arguments={"config_file": str(curtis_config_path), "velocity_converter_config": str(velocity_converter_config)}.items(),
    )

    # lidar node
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(lidar_path)),
        launch_arguments={
            "publish_freq": LaunchConfiguration("publish_freq"),
            "frame_id": LaunchConfiguration("lvx_frame_id"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "user_config_path": LaunchConfiguration("lidar_config"),
        }.items(),
    )

    # rviz2
    # rviz = Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         output='screen',
    #         arguments=['--display-config', str(rviz2_config)]
    #     )

    ld = LaunchDescription(declare_arguments)
    ld.add_action(flexnav_description)
    ld.add_action(controller_node_delayed)
    ld.add_action(controller_node_sim)
    ld.add_action(velocity_converter_node)
    ld.add_action(odometry_node)
    ld.add_action(odometry_node_joint_states)
    # ld.add_action(amrplc)
    # ld.add_action(lidar_node)
    ld.add_action(motor_node)
    ld.add_action(controller_manager)
    # ld.add_action(imu_openzen)
    ld.add_action(wheelset_pid_nodes)
    ld.add_action(joint_velocity_broadcaster_node)
    # ld.add_action(rviz)

    # TODO: add magnetometer
    # , imu
    # , move lidar node to correct package

    return ld
