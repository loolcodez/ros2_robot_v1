from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='odometer',
            executable='odometer_node',
            name='odometer_node',
            output='screen'
#            parameters=[config]
        )
    ])
