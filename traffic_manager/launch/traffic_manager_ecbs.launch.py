#!/usr/bin/env python3
"""traffic_manager + real ecbs_planner evaluate_path_server (no mock)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tm_share = get_package_share_directory('traffic_manager')
    default_config = os.path.join(tm_share, 'config', 'traffic_manager.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='traffic_manager parameters',
    )
    ecbs_service_arg = DeclareLaunchArgument(
        'evaluate_path_service',
        default_value='/evaluate_path_service',
        description='Must match traffic_manager evaluate_path_service param',
    )

    evaluate_path_server = Node(
        package='ecbs_planner',
        executable='evaluate_path_server',
        name='evaluate_path_server',
        output='screen',
        parameters=[{
            'service_name': LaunchConfiguration('evaluate_path_service'),
        }],
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
        ecbs_service_arg,
        evaluate_path_server,
        traffic_manager,
    ])
