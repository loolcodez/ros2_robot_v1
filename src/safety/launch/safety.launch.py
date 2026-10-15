from launch import LaunchDescription
from launch_ros.actions import Node
#from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
#    config = os.path.join(get_package_share_directory('proximity_sensor'), 'config', 'driver.yaml')

    return LaunchDescription([
        Node(
            package='safety',
            executable='safety_node',
            name='safety',
            output='screen'
#            parameters=[config]
        )
    ])
