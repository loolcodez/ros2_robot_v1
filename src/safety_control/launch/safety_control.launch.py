from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='safety_control',
            executable='safety_control_node',
            name='safety_control_node',
            output='screen'
#            parameters=[config]
        )
    ])
