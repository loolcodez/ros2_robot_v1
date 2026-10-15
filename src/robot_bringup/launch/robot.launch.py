import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    safety_launch_dir = os.path.join(get_package_share_directory('safety'), 'launch')
    power_launch_dir = os.path.join(get_package_share_directory('power_control'), 'launch')
    proximity_launch_dir = os.path.join(get_package_share_directory('proximity_sensor'), 'launch')
    light_launch_dir = os.path.join(get_package_share_directory('light_control'), 'launch')
    websocket_launch_dir = os.path.join(get_package_share_directory('websocket'), 'launch')
    driver_launch_dir = os.path.join(get_package_share_directory('driver'), 'launch')
#    camera_launch_dir = os.path.join(get_package_share_directory('camera'), 'launch')

    launch_safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(safety_launch_dir, 'safety.launch.py'))
    )

    launch_power = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(power_launch_dir, 'power_control.launch.py'))
    )

    proximity_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(proximity_launch_dir, 'proximity_sensor.launch.py'))
    )
    launch_light = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(light_launch_dir, 'light_control.launch.py'))
    )

 #   launch_camera = IncludeLaunchDescription(
 #       PythonLaunchDescriptionSource(os.path.join(camera_launch_dir, 'camera.launch.py'))
 #   )

    launch_websocket = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(websocket_launch_dir, 'websocket.launch.py'))
    )

    launch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(driver_launch_dir, 'driver.launch.py'))
    )

    return LaunchDescription([
        launch_safety,
        launch_power,
        proximity_sensor,
        launch_light,
  #      launch_camera
        launch_websocket,
        launch_driver
    ])
