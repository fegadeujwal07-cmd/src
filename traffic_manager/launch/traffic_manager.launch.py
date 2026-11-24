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
        description='YAML parameters for traffic_manager (query_horizon_m, topics, ECBS service)',
    )

    traffic_manager_node = Node(
        package='traffic_manager',
        executable='traffic_manager_node',
        name='traffic_manager',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_file_arg,
        traffic_manager_node,
    ])
