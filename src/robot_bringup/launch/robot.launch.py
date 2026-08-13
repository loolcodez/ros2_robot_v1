import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    driver_launch_dir = os.path.join(get_package_share_directory('driver'), 'launch')
    websocket_launch_dir = os.path.join(get_package_share_directory('websocket'), 'launch')
    power_launch_dir = os.path.join(get_package_share_directory('power_control'), 'launch')
#    camera_launch_dir = os.path.join(get_package_share_directory('camera'), 'launch')

    launch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(driver_launch_dir, 'driver.launch.py'))
    )
    
    launch_websocket = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(websocket_launch_dir, 'websocket.launch.py'))
    )
    
    launch_power = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(power_launch_dir, 'power_control.launch.py'))
    )
    
 #   launch_camera = IncludeLaunchDescription(
 #       PythonLaunchDescriptionSource(os.path.join(camera_launch_dir, 'camera.launch.py'))
 #   )

    return LaunchDescription([
        launch_driver,
        launch_websocket,
        launch_power
  #      launch_camera
    ])
