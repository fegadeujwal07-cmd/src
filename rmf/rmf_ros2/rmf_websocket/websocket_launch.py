# websocket_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rmf_websocket',
            executable='rmf_websocket_server',  # name may vary, double-check package
            name='rmf_websocket_server',
            output='screen',
            parameters=[{'port': 7878}],  # adjust as needed
        )
    ])
