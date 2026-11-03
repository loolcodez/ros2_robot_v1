import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_share = get_package_share_directory('robot_bringup')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # Change xacro model to URDF model
    robot_description_raw = Command(['xacro', ' ', xacro_file])

    # Force to string parameter type
    robot_description_param = ParameterValue(robot_description_raw, value_type=str)

    # Create robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_param}]
    )

    return LaunchDescription([
        node_robot_state_publisher
    ])
