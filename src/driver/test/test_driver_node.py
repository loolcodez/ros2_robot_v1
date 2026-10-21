# Run like this: colcon test --packages-select driver --event-handlers console_cohesion+ # ????
# colcon build --symlink-install
# source install/setup.bash
# pytest src/driver/test/test_driver_node.py

from unittest.mock import MagicMock, patch
import pytest
import rclpy
from geometry_msgs.msg import Twist
from driver.driver_node import DriverNode

@pytest.fixture(autouse=True)
def setup_teardown():
    rclpy.init()
    yield
    rclpy.shutdown()

@patch('driver.driver_node.DriverNode.initialize_driver')
def test_driver_node_speed_mapping(mock_init):

    # Fake serial connection creation
    mock_init.return_value = True

    node = DriverNode()
    node.driver = MagicMock()

    # Make simulated joystick message. Drive forward with speed 50
    msg = Twist()
    msg.linear.x = 50.0
    msg.angular.z = 0.0

    node.handle_speed_control_command(msg)

    # Check expected motor speeds [50, -50, -50, 50], invert = [False, True, True, False]
    # Left_front, right_front, left_rear, right_rear
    assert node.target_motor_speeds == [50, -50, -50, 50]

@patch('driver.driver_node.DriverNode.initialize_driver')
def test_driver_node_sensor_processing(mock_init):

    # Fake serial connection creation
    mock_init.return_value = True

    node = DriverNode()
    mock_driver = MagicMock()

    # Define what faked diver card return for encoders
    mock_driver.get_motor_encoder.return_value = (1000.0, 1000.0, 1000.0, 1000.0)
    mock_driver.get_accelerometer_data.return_value = (0.0, 0.0, -1.0) # G-force value
    mock_driver.get_gyroscope_data.return_value = (0.0, 0.0, 0.0)
    node.driver = mock_driver

    # Set initial state for encoders. We do not want delta to be zero in the beginning
    node.previous_encoders = [900.0, 900.0, 900.0, 900.0]

    # Set filtered target speed what PID control tried to achieve
    node.filtered_motor_speeds = [10.0, 10.0, 10.0, 10.0]

    node.handle_sensors_and_control()

    # Check that the node called the driver card API calls to get data
    mock_driver.get_motor_encoder.assert_called_once()
    mock_driver.get_gyroscope_data.assert_called_once()

    # Assert that previous_encoders was updated to new value
    assert node.previous_encoders == [1000.0, 1000.0, 1000.0, 1000.0]

    # Check that new command was sent to driver card API to update motors speed
    mock_driver.set_motor.assert_called_once()
