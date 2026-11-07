import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Get robot_bringup package's config directory to find config/ekf.yaml
    pkg_share = get_package_share_directory('robot_bringup')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    return LaunchDescription([
        Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        )
    ])
