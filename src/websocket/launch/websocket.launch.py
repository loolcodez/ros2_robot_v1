import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
#    config = os.path.join(get_package_share_directory('websocket'), 'config', 'websocket.yaml')

    return LaunchDescription([
        Node(
            package='websocket',
            executable='websocket_node',
            name='websocket',
            output='screen'
#            parameters=[config]
        )
    ])
