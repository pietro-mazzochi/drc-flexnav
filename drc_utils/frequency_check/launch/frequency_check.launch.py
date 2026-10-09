#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('frequency_check')
    default_config = os.path.join(pkg_share, 'config', 'config.yaml')

    config_arg = DeclareLaunchArgument(
        'frequency_check_config',
        default_value=default_config,
        description='Caminho para o arquivo de configuração do node frequency_check'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usa simulação de tempo'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Nível de log'
    )

    frequency_check_node = Node(
        package='frequency_check',
        executable='frequency_check',
        name='frequency_check',
        output='screen',
        parameters=[
            LaunchConfiguration('frequency_check_config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    ld = LaunchDescription([
        config_arg,
        use_sim_time_arg,
        log_level_arg,
        frequency_check_node,
    ])

    return ld
