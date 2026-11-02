import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
#from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command #, LaunchSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Haetaan robot_bringup-paketin share-hakemiston polku
    pkg_share = get_package_share_directory('robot_bringup')

    # Määritetään polku urdf-tiedostoon
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')


    # Käytetään xacro-komentoa muuttamaan .xacro-malli puhtaaksi URDF-rakenteeksi
    robot_description_raw = Command(['xacro', ' ', xacro_file])

    # KÄÄRITÄÄN ARVO PARAMETERVALUE-MÄÄRITELMÄÄN (Pakotetaan tyypiksi string)
    robot_description_param = ParameterValue(robot_description_raw, value_type=str)

    # Konfiguroidaan robot_state_publisher -solmu
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_param}]
    )

    # Palautetaan LaunchDescription, joka käynnistää solmun
    return LaunchDescription([
        node_robot_state_publisher
    ])
