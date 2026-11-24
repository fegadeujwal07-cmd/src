#!/usr/bin/env python3

"""
Complete RMF System Launch File - Essential Core Services Only.
This launches only the essential RMF core services needed for the full control fleet adapter.

ESSENTIAL SERVICES:
- rmf_traffic_schedule: Essential for fleet coordination (REQUIRED for full control adapter)
- rmf_traffic_blockade: Essential for traffic management and collision prevention
- rmf_task_dispatcher: Essential for task management
- building_map_server: Essential for map visualization and navigation

OPTIONAL SERVICES (commented out):
- websocket_server: Not required for full control adapter

REQUIREMENTS:
- MQTT broker (Mosquitto) must be running for robot communication
- Full control fleet adapter can be launched separately

Use RMF's official RViz: ros2 launch rmf_visualization visualization.launch.xml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Generate launch description for essential RMF core services only"""
    
    # Declare launch arguments
    fleet_name_arg = DeclareLaunchArgument(
        'fleet_name',
        default_value='amr_fleet',
        description='Name of the AMR fleet'
    )
    
    nav_graph_file_arg = DeclareLaunchArgument(
        'nav_graph_file',
        default_value='config/nav_graphs/0.yaml',
        description='Path to nav_graph.yaml file'
    )
    
    building_file_arg = DeclareLaunchArgument(
        'building_file',
        default_value='config/R3.building.yaml',
        description='Path to building.yaml file'
    )
    
    mqtt_broker_host_arg = DeclareLaunchArgument(
        'mqtt_broker_host',
        default_value='localhost',
        description='MQTT broker host for robot communication'
    )
    
    # NOTE: SERVER_URL IS CURRENTLY UNREACHABLE
    # Issue: 20.20.0.201 is not accessible from this machine (100% packet loss)
    # The fleet adapter will fail to poll robot states and show connection timeout errors
    # To fix: change to localhost:8000 or fix network connectivity to 20.20.0.201
    
    # Start RMF Traffic Schedule (ESSENTIAL for full control fleet adapter)
    # This is the only service that the full control adapter absolutely needs
    rmf_traffic_schedule = Node(
        package='rmf_traffic_ros2',
        executable='rmf_traffic_schedule',
        name='rmf_traffic_schedule',
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', 'rmf_traffic_schedule:=info'
        ]
    )
    
    # Start RMF Traffic Blockade (ESSENTIAL for traffic management and collision prevention)
    rmf_traffic_blockade = Node(
        package='rmf_traffic_ros2',
        executable='rmf_traffic_blockade',
        name='rmf_traffic_blockade',
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', 'rmf_traffic_blockade:=info'
        ]
    )
    
    # Start RMF Task Dispatcher (ESSENTIAL for task management)
    rmf_task_dispatcher = Node(
        package='rmf_task_ros2',
        executable='rmf_task_dispatcher',
        name='rmf_task_dispatcher',
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', 'rmf_task_dispatcher:=info'
        ]
    )
    
    # RMF Building Map Server (ESSENTIAL for map visualization and navigation)
    building_map_server = Node(
        package='rmf_building_map_tools',
        executable='building_map_server',
        name='building_map_server',
        arguments=[
            os.path.join(os.getcwd(), 'src/execution/config/R3.building.yaml'),
            '--ros-args',
            '--log-level', 'building_map_server:=info'
        ],
        remappings=[('building_map', '/building_map')],
        output='screen'
    )
    
    # OPTIONAL: RMF WebSocket Server (NOT required for full control adapter)
    # Uncomment if you need websocket functionality
    # websocket_server = Node(
    #     package='rmf_websocket',
    #     executable='websocket_server',
    #     name='rmf_websocket_server',
    #     output='screen',
    #     arguments=[
    #         '--ros-args',
    #         '--log-level', 'rmf_websocket_server:=info'
    #     ]
    # )
    
    # Create launch description
    return LaunchDescription([
        # Launch arguments
        fleet_name_arg,
        nav_graph_file_arg,
        building_file_arg,
        mqtt_broker_host_arg,
        
        # Log startup information
        LogInfo(msg="🚀 Starting Essential RMF Core Services..."),
        LogInfo(msg="This will launch only the essential services:"),
        LogInfo(msg="  1. RMF Traffic Schedule (REQUIRED for full control adapter)"),
        LogInfo(msg="  2. RMF Traffic Blockade (ESSENTIAL for collision avoidance)"),
        LogInfo(msg="  3. RMF Task Dispatcher (ESSENTIAL for task management)"),
        LogInfo(msg="  4. RMF Building Map Server (ESSENTIAL for map visualization)"),
        LogInfo(msg=""),
        LogInfo(msg="✅ These services are sufficient for the full control fleet adapter"),
        LogInfo(msg=""),
        LogInfo(msg="To launch the full control fleet adapter separately, run:"),
        LogInfo(msg="  ros2 launch execution execution_full_control.launch.py"),
        LogInfo(msg=""),
        LogInfo(msg="To launch RMF's official RViz, run:"),
        LogInfo(msg="  ros2 launch rmf_visualization visualization.launch.xml"),
        
        # Start essential RMF core services
        rmf_traffic_schedule,  # REQUIRED for full control adapter
        rmf_traffic_blockade,  # Essential for traffic management
        rmf_task_dispatcher,   # Essential for task management
        building_map_server,   # Essential for map visualization
        #ould I check your launch files to ensure the navgraph visualizer is included, or help you start it manually?
        # Optional services (commented out)
        # websocket_server,
    ])
