#!/usr/bin/env python3
"""
Working Launch file for RMF Full Control Integration
This launch file properly configures the full control fleet adapter with correct absolute paths.
"""

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the workspace directory
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    
    # Declare launch arguments
    fleet_name_arg = DeclareLaunchArgument(
        'fleet_name',
        default_value='amr_fleet',
        description='Name of the fleet'
    )
    
    # server_uri for rmf-web API server connection (uses C++ adapter's built-in WebSocket)
    server_uri_arg = DeclareLaunchArgument(
        'server_uri',
        default_value='ws://localhost:8000/_internal',
        description='WebSocket URI of the rmf-web API server (e.g., ws://localhost:8000/_internal or ws://192.168.1.166:8000/_internal). Must use ws:// protocol. Empty string means no connection.'
    )
    
    # MQTT broker for robot communication
    mqtt_broker_host_arg = DeclareLaunchArgument(
        'mqtt_broker_host',
        default_value='localhost',
        description='MQTT broker host (e.g., localhost or 192.168.1.166). Run Mosquitto on RMF laptop.'
    )
    mqtt_broker_port_arg = DeclareLaunchArgument(
        'mqtt_broker_port',
        default_value='1883',
        description='MQTT broker port (default 1883).'
    )
    
    # api_server_url for RMF API server (for task state bridge to get real-time task status)
    api_server_url_arg = DeclareLaunchArgument(
        'api_server_url',
        default_value='http://localhost:8000',
        description='HTTP URL of the RMF API server for task state updates (e.g., http://localhost:8000). Used by task_state_bridge to get real-time task status.'
    )
    
    nav_graph_file_arg = DeclareLaunchArgument(
        'nav_graph_file',
        default_value=os.path.join(
            get_package_share_directory('execution'),
            'config', 'nav_graphs', '0.yaml'
        ),
        description='Absolute path to the navigation graph file'
    )

    traffic_manager_config_arg = DeclareLaunchArgument(
        'traffic_manager_config',
        default_value=os.path.join(
            get_package_share_directory('traffic_manager'),
            'config', 'traffic_manager.yaml'
        ),
        description='traffic_manager YAML (query_horizon_m, ECBS service, topics)',
    )

    evaluate_path_service_arg = DeclareLaunchArgument(
        'evaluate_path_service',
        default_value='/evaluate_path_service',
        description='EvaluatePath service (ecbs_planner server + traffic_manager client)',
    )
    enable_safety_field_arg = DeclareLaunchArgument(
        'enable_safety_field',
        default_value='false',
        description='Launch monitor/safety_field_node (proximity startPause via zone_manager). '
                    'Off by default: parked robots beside the aisle deadlock against passing robots.',
    )
    pkg_share = get_package_share_directory('execution')
    config_file = os.path.join(pkg_share, 'config', 'config.yaml')

    # Get launch configurations
    fleet_name = LaunchConfiguration('fleet_name')
    server_uri = LaunchConfiguration('server_uri')
    api_server_url = LaunchConfiguration('api_server_url')
    nav_graph_file = LaunchConfiguration('nav_graph_file')
    mqtt_broker_host = LaunchConfiguration('mqtt_broker_host')
    mqtt_broker_port = LaunchConfiguration('mqtt_broker_port')
    traffic_manager_config = LaunchConfiguration('traffic_manager_config')
    evaluate_path_service = LaunchConfiguration('evaluate_path_service')
    enable_safety_field = LaunchConfiguration('enable_safety_field')

    full_control_adapter = Node(
        package='rmf_fleet_adapter',
        executable='full_control',
        name='full_control_fleet_adapter',
        parameters=[{
            'use_sim_time': False,  # Use real time (not simulation time)
            'fleet_name': fleet_name,
            'nav_graph_file': nav_graph_file,
            'enable_responsive_wait': False,
            'discovery_timeout': 60.0,
            'update_frequency': 2.0,
            'publish_fleet_state': False,
            'fleet_state_topic': '/fleet_states',
            'path_request_topic': '/robot_path_requests',
            'mode_request_topic': '/robot_mode_requests',
            # evaluate_path_server owns ScheduleCache + /from_zone_manager (traffic_manager client).
            'ecbs_external_evaluate_server': True,
            'perform_deliveries': True,   # 👈 enable delivery task acceptance
            'perform_loop': True,        # 👈 enable loop task acceptance
            #'perform_patrol': True,        # 👈 enable patrol task acceptance
            'server_uri': server_uri,            # WebSocket connection to rmf-web API server
            
            # Vehicle Traits Parameters (from config.yaml rmf_fleet.limits and profile)
            #'linear_velocity': 0.2,              # m/s - max forward speed (from config.yaml limits.linear[0])
            #'linear_acceleration': 0.2,          # m/s² - max acceleration (from config.yaml limits.linear[1])
            #'angular_velocity': 0.2,             # rad/s - max rotation speed (from config.yaml limits.angular[0])
            #'angular_acceleration': 0.2,         # rad/s² - max rotation acceleration (from config.yaml limits.angular[1])
            #'footprint_radius': 0.3,             # m - robot footprint radius (from config.yaml profile.footprint)
            #'vicinity_radius': 0.5,              # m - safety zone radius (from config.yaml profile.vicinity)
            #'reversible': True,                  # Can robot move backwards (from config.yaml reversible)
            
            # Battery System Parameters (from config.yaml battery_system)
            #'battery_voltage': 12.0,             # V - battery voltage (from config.yaml battery_system.voltage)
            #'battery_capacity': 24.0,            # Ah - battery capacity (from config.yaml battery_system.capacity)
            #'battery_charging_current': 5.0,     # A - charging current (from config.yaml battery_system.charging_current)
            
            # Mechanical System Parameters (from config.yaml mechanical_system)
            #'mass': 20.0,                        # kg - robot mass (from config.yaml mechanical_system.mass)
            #'inertia': 10.0,                     # kg·m² - moment of inertia (from config.yaml mechanical_system.moment_of_inertia)
            #'friction_coefficient': 0.22,        # friction coefficient (from config.yaml mechanical_system.friction_coefficient)
            
            # Power System Parameters (from config.yaml ambient_system and tool_system)
            #'ambient_power_drain': 20.0,         # W - base power consumption (from config.yaml ambient_system.power)
            #'tool_power_drain': 0.0,              # W - tool power consumption (from config.yaml tool_system.power)
            
            # Battery Management Parameters (from config.yaml)
            #'drain_battery': True,               # Account for battery drain (from config.yaml account_for_battery_drain)
            #'recharge_threshold': 0.10,          # Battery level threshold (from config.yaml recharge_threshold)
            #'recharge_soc': 1.0,                 # Charge to 100% SOC (from config.yaml recharge_soc)
            
            # Task Capabilities (from config.yaml task_capabilities)
            #'perform_deliveries': True,          # Enable delivery tasks (from config.yaml task_capabilities.delivery)
            #'perform_loop': True,                # Enable loop tasks (from config.yaml task_capabilities.loop)
            #'finishing_request': 'nothing',      # What to do after tasks (from config.yaml task_capabilities.finishing_request)
            
            # Adapter Behavior
            #'enable_responsive_wait': True,
            #'discovery_timeout': 60.0,
            #'update_frequency': 10.0,            # Update frequency (from config.yaml publish_fleet_state)
            #'publish_fleet_state': False,
           #'delay_threshold': 60.0,             # Allow up to 60 seconds of cumulative delay before itinerary refresh
            #'fleet_state_topic': '/fleet_states',
            #'path_request_topic': '/robot_path_requests',
           # 'mode_request_topic': '/robot_mode_requests',
           #'server_uri': server_uri,            # WebSocket connection to rmf-web API server
        }],
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', 'fleet_adapter:=info'
        ]
    )

    custom_adapter = Node(
        package='execution',
        executable='execution_full_control',
        name='rmf_integrated_fleet_adapter',
        parameters=[{
            'use_sim_time': False,  # Use real time (not simulation time)
            'fleet_name': fleet_name,
            'config_path': config_file,
            'nav_graph_path': nav_graph_file,
            # 10 Hz. Drives poll_amr_states_tick -> publish_fleet_state(), so this
            # is the rate RMF's full_control runs update_state() at, once PER ROBOT
            # per tick — the only stage in the chain whose cost multiplies by fleet
            # size. At 25 Hz, 10 robots meant 250 update_state() calls/s.
            #
            # Safe to sample because the state path is a latest-value cache, not a
            # queue: on_robot_state() overwrites latest_robot_state_ and this timer
            # reads whatever is newest, so the pose is at most one period old and
            # the lag does not accumulate.
            #
            # Only this FMS-side stage is lowered. The two robot-side stages
            # (hardware_interface.py's 0.04 s timer and the bridge's
            # state_publish_hz) stay at 25 Hz, so worst-case staleness grows by one
            # period (~100 ms), not three. Floor is RMF's 200 ms path-request
            # resend (full_control/main.cpp:495) — 10 Hz keeps 2 samples inside it,
            # 5 Hz would leave none.
            'state_poll_interval': 0.1,
            'map_conversion_mode': 'auto',
            'server_uri': server_uri,  # WebSocket connection to rmf-web API server (port 8000)
            'mqtt_broker_host': mqtt_broker_host,
            'mqtt_broker_port': mqtt_broker_port,
            'base_extension_request_topic': '/BaseRequests',
        }],
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', 'rmf_integrated_fleet_adapter:=info'
        ]
    )

    # RMF Building Map Server (ESSENTIAL for map visualization in dashboard)
    # This publishes the building map to /map topic, which the rmf-web API server subscribes to
    building_map_file = os.path.join(pkg_share, 'config', 'R3.building.yaml')
    building_map_server = Node(
        package='rmf_building_map_tools',
        executable='building_map_server',
        name='building_map_server',
        parameters=[{
            'use_sim_time': False,  # Use real time (not simulation time)
        }],
        arguments=[
            building_map_file,
            '--ros-args',
            '--log-level', 'building_map_server:=info'
        ],
        # Note: building_map_server publishes to /map by default
        # The rmf-web API server subscribes to /map and stores it in the database
        # The dashboard then fetches the map from the API server
        output='screen'
    )

    # RMF Visualization (ESSENTIAL for trajectory visualization in dashboard)
    # This launches the trajectory server on port 8006 which the dashboard connects to
    # Use the existing visualization.launch.xml file instead of launching nodes directly
    # This is cleaner and matches the office demo approach
    # 
    # Note: Set headless:=false if you want RViz to launch (for debugging/visualization)
    #       Set headless:=true if you only want the web dashboard (no RViz GUI)
    #       The trajectory server (port 8006) works for both RViz and dashboard
    try:
        # Get the visualization launch file path
        viz_pkg_share = get_package_share_directory('rmf_visualization')
        viz_launch_file = os.path.join(viz_pkg_share, 'visualization.launch.xml')
        
        if os.path.exists(viz_launch_file):
            # Use ExecuteProcess to launch the XML launch file
            # This is the cleanest way to include an XML launch file from Python
            # headless:=false launches RViz, headless:=true only launches nodes (no GUI)
            visualization_launch = ExecuteProcess(
                cmd=['ros2', 'launch', 'rmf_visualization', 'visualization.launch.xml',
                     'use_sim_time:=false',
                     'map_name:=L1',
                     'headless:=false',  # Set to false to launch RViz (works with dashboard too!)
                     'websocket_port:=8006'],
                output='screen',
                name='rmf_visualization_launch'
            )
        else:
            raise FileNotFoundError(f"Visualization launch file not found: {viz_launch_file}")
    except (KeyError, FileNotFoundError) as e:
        # Visualization package not available
        visualization_launch = None
        LogInfo(msg=f"⚠️  WARNING: Could not launch visualization: {e}")
        LogInfo(msg="         Trajectory server will not be started.")
        LogInfo(msg="         To enable: Build rmf_visualization package")

    # Task State Bridge - connects to RMF API server and publishes task state updates to ROS topic
    # This enables the fleet adapter to know which task is currently "underway" vs "queued"
    task_state_bridge = Node(
        package='execution',
        executable='execution_task_state_bridge.py',
        name='task_state_bridge',
        parameters=[{
            'api_server_url': api_server_url,
            'reconnect_interval': 5.0,
        }],
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', 'task_state_bridge:=info'
        ]
    )

    # Start custom adapter with delay to ensure full_control is ready
    custom_adapter_delayed = TimerAction(
        period=2.0,
        actions=[custom_adapter]
    )
    
    # Start task state bridge with delay to ensure API server is ready
    task_state_bridge_delayed = TimerAction(
        period=3.0,
        actions=[task_state_bridge]
    )

    # ---- Zone stack (Phase 2/3) -------------------------------------------
    # zone_monitor  : nav graph zones -> /robot_zone_status, /zone_catalog
    # zone_manager  : conflict detection -> /zone_conflicts (pause/resume)
    #                 and /replan_candidate (which robot should replan)
    #
    # The custom adapter consumes both: it pauses robots on /zone_conflicts and
    # closes lanes FOR ONE ROBOT on /replan_candidate, which makes RMF replan
    # only that robot. zone_manager owns resume.
    #
    # NOTE: zone_manager autostarts its own lifecycle (CONFIGURE -> ACTIVATE in
    # its main.cpp), so no external lifecycle manager is required.
    zone_monitor_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('monitor'),
                         'launch', 'zone_monitor.launch.xml')),
    )

    zone_manager_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('zone_manager'),
                         'launch', 'zone_manager.launch.xml')),
    )

    # ECBS EvaluatePath server (traffic_manager calls mode=evaluate).
    ecbs_evaluate_path_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ecbs_planner'),
                         'launch', 'evaluate_path_service.launch.xml')),
        launch_arguments={
            'service_name': evaluate_path_service,
        }.items(),
    )

    # Path gate: full_control publishes PathIntent; traffic_manager replies with
    # PathRelease before PathRequest is sent to execution.
    traffic_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('traffic_manager'),
                         'launch', 'traffic_manager.launch.py')),
        launch_arguments={
            'config_file': traffic_manager_config,
        }.items(),
    )

    # Rectangular safety field per AMR, centred on the MEASURED pose from
    # /fleet_states and turned to the MEASURED heading, so it is carried by the
    # robot rather than lying on the map under it. Publishes
    # /safety_field_markers for RViz and, as per-robot deltas,
    # /safety_field_conflict — which zone_manager turns into startPause /
    # stopPause on /zone_conflicts.
    #
    # safety_field_margin is the number to turn. It is the clearance kept around
    # the body on all four sides, so the field is 1.353+2*margin long and
    # 1.403+2*margin wide. At 0.40 m two fields touch with 0.80 m of clear floor
    # between the bodies: 2.153 m between centres nose-to-tail, 2.203 m
    # side-by-side, and up to 3.080 m corner-to-corner — the rectangle catches a
    # diagonal approach almost a metre earlier than the 2.2 m circle it replaced,
    # which is the point of the shape.
    #
    # One rectangle, no hysteresis and no dwell: the conflict is published the
    # moment the fields touch and cleared the moment they part. Because the
    # field rotates, two robots turning on the spot can cross that boundary in
    # both directions, and each crossing is a real startPause / stopPause.
    safety_field_node = Node(
        package='monitor',
        executable='safety_field_node',
        name='safety_field_node',
        output='screen',
        condition=IfCondition(enable_safety_field),
        parameters=[{
            'safety_field_length': 1.353,      # m, AMR body nose to tail
            'safety_field_width': 1.403,       # m, AMR body side to side
            'safety_field_margin': 0.20,       # m, clearance added on each side
            'evaluate_rate_hz': 10.0,
            'level_name': 'L1',
        }],
        arguments=[
            '--ros-args',
            '--log-level', 'safety_field_node:=warn'
        ]
    )

    # zone_monitor needs the nav graph + fleet states; zone_manager needs
    # zone_monitor's catalog. safety_field_node needs /fleet_states, and
    # zone_manager must be up to act on the conflicts it reports — though
    # /safety_field_conflict is latched, so a late zone_manager still receives
    # the most recent transition rather than missing it entirely. Stagger them
    # all after the adapter is up.
    zone_stack_delayed = TimerAction(
        period=5.0,
        actions=[zone_monitor_launch, zone_manager_launch, safety_field_node]
    )
    
    # Information messages
    info_messages = [
        LogInfo(msg="🚀 Starting RMF Full Control Integration (Working)"),
        LogInfo(msg="✅ Full Control Fleet Adapter: Listens to /fleet_states topic"),
        LogInfo(msg="📤 Custom Fleet Adapter: Publishes FleetState messages"),
        LogInfo(msg="🔄 Integration: Custom adapter -> FleetState -> Full Control adapter"),
        LogInfo(msg="🤖 Robots will be automatically added to full control adapter"),
        LogInfo(msg=""),
        LogInfo(msg="🗺️  Building Map Server: Publishing map to /map topic"),
        LogInfo(msg="   - Map file: " + building_map_file),
        LogInfo(msg="   - API server subscribes to /map and serves it to dashboard"),
        LogInfo(msg=""),
        LogInfo(msg="📈 Trajectory Server: WebSocket server on port 8006"),
        LogInfo(msg="   - Provides robot trajectory/path visualization"),
        LogInfo(msg="   - Dashboard connects to: http://localhost:8006"),
        LogInfo(msg=""),
        LogInfo(msg="Key Topics:"),
        LogInfo(msg="  - /fleet_states: FleetState messages"),
        LogInfo(msg="  - /map: Building map (for dashboard visualization)"),
        LogInfo(msg="  - /robot_mode_requests: Mode requests to robots"),
        LogInfo(msg="  - /traffic_manager/path_intent: Path legs awaiting release"),
        LogInfo(msg="  - /traffic_manager/path_release: Approved path prefix for robots"),
        LogInfo(msg="  - /BaseRequests: VDA newBaseRequest / ack → traffic_manager"),
        LogInfo(msg="  - /evaluate_path_service: ECBS horizon evaluate (ecbs_planner)"),        
        LogInfo(msg=""),
        LogInfo(msg="Navigation Graph: " + os.path.join(pkg_share, 'config', 'nav_graphs', '0.yaml')),
        LogInfo(msg=""),
        LogInfo(msg="To test the integration:"),
        LogInfo(msg="  1. Launch this file"),
        LogInfo(msg="  2. Check /fleet_states topic for robot states"),
        LogInfo(msg="  3. Use RMF Dashboard or CLI to send delivery tasks"),
        LogInfo(msg=""),
        LogInfo(msg="Dashboard Integration (Built-in WebSocket):"),
        LogInfo(msg="  - For local: ros2 launch ... server_uri:=\"ws://localhost:8000/_internal\""),
        LogInfo(msg="  - For network: ros2 launch ... server_uri:=\"ws://192.168.1.166:8000/_internal\""),
        LogInfo(msg="  - Ensure rmf-web API server is running on port 8000"),
        LogInfo(msg="  - The full_control C++ adapter handles WebSocket connection automatically"),
        LogInfo(msg="  - Building map server publishes to /map for dashboard visualization"),
        LogInfo(msg="📡 MQTT broker:"),
        LogInfo(msg="  - Run Mosquitto on RMF laptop: sudo apt install mosquitto && sudo systemctl start mosquitto"),
        LogInfo(msg="  - mqtt_broker_host:=192.168.1.166 (connects to laptop's broker)"),
        LogInfo(msg=""),
        LogInfo(msg="🌉 Task State Bridge:"),
        LogInfo(msg="  - Connects to RMF API server via Socket.IO"),
        LogInfo(msg="  - Publishes task state updates to /rmf_task_state_updates"),
        LogInfo(msg="  - Fleet adapter uses this to trigger pickup when task is 'underway'"),
        LogInfo(msg="  - Default API server URL: http://localhost:8000"),
        LogInfo(msg="  - Override with: ros2 launch ... api_server_url:=\"http://your-server:8000\""),
    ]
    
    return LaunchDescription([
        # Launch arguments
        fleet_name_arg,
        server_uri_arg,
        mqtt_broker_host_arg,
        mqtt_broker_port_arg,
        api_server_url_arg,
        nav_graph_file_arg,
        traffic_manager_config_arg,
        evaluate_path_service_arg,
        enable_safety_field_arg,
        
        # Information messages
        *info_messages,
        
        # Start building map server first (needed for dashboard visualization)
        building_map_server,
        
        # Start visualization (trajectory server on port 8006) if available
        *([visualization_launch] if visualization_launch is not None else []),

        # ECBS service then path gate (before full_control sends path intents)
        ecbs_evaluate_path_launch,
        traffic_manager_launch,
        
        # Start full control adapter
        full_control_adapter,
        
        # Start custom adapter with delay
        custom_adapter_delayed,
        
        # Start task state bridge with delay (connects to RMF API server for task status)
        task_state_bridge_delayed,

        # Zone conflict detection + per-robot replan (Phase 2/3),
        # plus the safety field that feeds proximity conflicts into it
        zone_stack_delayed,
    ])
