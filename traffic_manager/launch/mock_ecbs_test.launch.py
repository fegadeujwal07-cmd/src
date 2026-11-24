#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('traffic_manager')
    default_config = os.path.join(pkg_share, 'config', 'traffic_manager.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='traffic_manager parameters',
    )

    mock_ecbs = Node(
        package='traffic_manager',
        executable='mock_ecbs_eval_server.py',
        name='mock_ecbs_eval_server',
        output='screen',
    )

    traffic_manager = Node(
        package='traffic_manager',
        executable='traffic_manager_node',
        name='traffic_manager',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_file_arg,
        mock_ecbs,
        traffic_manager,
    ])
