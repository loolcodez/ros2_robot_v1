import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
#    config = os.path.join(get_package_share_directory('power_control'), 'config', 'driver.yaml')

    return LaunchDescription([
        Node(
            package='power_control',
            executable='power_control_node',
            name='power_control',
            output='screen'
#            parameters=[config]
        )
    ])
