import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_imu_mag_sim = get_package_share_directory('imu_mag_sim')
    urdf_file = os.path.join(pkg_imu_mag_sim, 'urdf', 'imu_mag_robot.urdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_imu_mag_sim, 'worlds', 'empty.world')],
            description='SDF world file'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')]),
            launch_arguments={'world': LaunchConfiguration('world'), 'verbose':'true'}.items(),
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-entity', 'imu_mag_robot', '-file', urdf_file, '-x', '0.0', '-y', '0.0', '-z', '0.0'],
            output='screen'
        ),
    ])
