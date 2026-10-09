from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node, SetParameter
from launch.substitutions import (
    LaunchConfiguration,
)

from ament_index_python.packages import (
    get_package_share_directory,
    get_package_share_path,
)


def generate_launch_description():
    drc_flexnav_pose_estimation = get_package_share_path("drc_flexnav_pose_estimation")

    ekf_config_arg = DeclareLaunchArgument(
        "ekf_config",
        default_value=str(
            drc_flexnav_pose_estimation / "config" / "extended_kalman_filter.yaml"
        ),
        description="EKF configuration file path",
    )
    ekf_config = LaunchConfiguration("ekf_config")

    use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn the kalman filter node automaticaly",
    )
    use_respawn = LaunchConfiguration("use_respawn")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation time",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    SetParameter("use_sim_time", use_sim_time)

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )
    log_level = LaunchConfiguration("log_level")

    ekf_node = Node(
        package="drc_flexnav_pose_estimation",
        executable="extended_kalman_filter_node",
        name="extended_kalman_filter_node",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[ekf_config],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    mu_broadcaster = Node(
        package="drc_flexnav_pose_estimation",
        executable="dynamics_controller_status_to_accel_stamped",
        name="mu_broadcaster",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[{"status_topic_name": "/mrac_controller/status"}, {"controller_type": "mrac_controller"}],
    )

    ld = [
        ekf_config_arg,
        use_respawn_arg,
        use_sim_time_arg,
        log_level_arg,
        ekf_node,
        mu_broadcaster,
    ]

    return LaunchDescription(ld)
