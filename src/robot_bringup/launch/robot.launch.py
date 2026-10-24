import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    lc_manager_launch_dir = os.path.join(get_package_share_directory('lc_manager'), 'launch')
    safety_launch_dir = os.path.join(get_package_share_directory('safety_control'), 'launch')
    power_launch_dir = os.path.join(get_package_share_directory('power_control'), 'launch')
    light_launch_dir = os.path.join(get_package_share_directory('light_control'), 'launch')
    proximity_launch_dir = os.path.join(get_package_share_directory('proximity_sensor'), 'launch')
#    camera_launch_dir = os.path.join(get_package_share_directory('camera'), 'launch')
    websocket_launch_dir = os.path.join(get_package_share_directory('websocket'), 'launch')
    odometer_launch_dir = os.path.join(get_package_share_directory('odometer'), 'launch')
    driver_launch_dir = os.path.join(get_package_share_directory('driver'), 'launch')

    launch_lc_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lc_manager_launch_dir, 'lc_manager.launch.py'))
    )

    launch_safety_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(safety_launch_dir, 'safety_control.launch.py'))
    )

    launch_power_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(power_launch_dir, 'power_control.launch.py'))
    )

    launch_light_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(light_launch_dir, 'light_control.launch.py'))
    )

    proximity_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(proximity_launch_dir, 'proximity_sensor.launch.py'))
    )

 #   launch_camera = IncludeLaunchDescription(
 #       PythonLaunchDescriptionSource(os.path.join(camera_launch_dir, 'camera.launch.py'))
 #   )

    launch_websocket = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(websocket_launch_dir, 'websocket.launch.py'))
    )

    launch_odometer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(odometer_launch_dir, 'odometer.launch.py'))
    )

    launch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(driver_launch_dir, 'driver.launch.py'))
    )

    return LaunchDescription([
        launch_lc_manager,
        launch_safety_control,
        launch_power_control,
        launch_light_control,
        proximity_sensor,
  #      launch_camera
        launch_websocket,
        launch_odometer,
        launch_driver,
    ])
