# Run like this: colcon test --packages-select driver --event-handlers console_cohesion+ # ????
# colcon build --symlink-install
# source install/setup.bash
# pytest src/driver/test/test_driver_node.py -s

from unittest.mock import MagicMock, patch
import unittest
import pytest
import rclpy
from geometry_msgs.msg import Twist
from driver.driver_node import DriverNode
from rclpy.lifecycle import LifecycleState

@pytest.fixture(autouse=True)
def setup_teardown():
    rclpy.init()
    yield
    rclpy.shutdown()

class TestProximitySensorNode(unittest.TestCase):

    def setUp(self):
        """Init node and its internal HW objects"""
        # Patch initialize_driver before on_configure call to avoid error prints
        with patch('driver.driver_node.DriverNode.initialize_driver', return_value=True):
            self.node = DriverNode()
            state = self.node.on_configure(LifecycleState("", 1))
        self.shutdown = False

    @patch('driver.driver_node.DriverNode.initialize_driver')
    def test_driver_node_speed_mapping(self, mock_init):

        # Fake serial connection creation
        self.node.driver = MagicMock()
        mock_init.return_value = True

        # Make simulated joystick message. Drive forward with speed 50
        msg = Twist()
        msg.linear.x = 50.0
        msg.angular.z = 0.0

        self.node.handle_speed_control_command(msg)

        # Check expected motor speeds [50, -50, -50, 50], invert = [False, True, True, False]
        # Left_front, right_front, left_rear, right_rear
        assert self.node.target_motor_speeds == [50, -50, -50, 50]

    @patch('driver.driver_node.DriverNode.initialize_driver')
    def test_driver_node_sensor_processing(self, mock_init):

        # Fake serial connection creation
        self.node.driver = MagicMock()
        mock_init.return_value = True

        # Define what faked diver card return for encoders
        self.node.driver.get_motor_encoder.return_value = (1000.0, 1000.0, 1000.0, 1000.0)
        self.node.driver.get_accelerometer_data.return_value = (0.0, 0.0, -1.0) # G-force value
        self.node.driver.get_gyroscope_data.return_value = (0.0, 0.0, 0.0)
  
        # Set initial state for encoders. We do not want delta to be zero in the beginning
        self.node.previous_encoders = [900.0, 900.0, 900.0, 900.0]

        # Set filtered target speed what PID control tried to achieve
        self.node.filtered_motor_speeds = [10.0, 10.0, 10.0, 10.0]

        self.node.handle_sensors_and_control()

        # Check that the node called the driver card API calls to get data
        self.node.driver.get_motor_encoder.assert_called_once()
        self.node.driver.get_gyroscope_data.assert_called_once()

        # Assert that previous_encoders was updated to new value
        assert self.node.previous_encoders == [1000.0, 1000.0, 1000.0, 1000.0]

        # Verify that new command was sent to driver card API to update motors speed
        # Assert that set_motor method was called 4 times
        assert self.node.driver.set_motor.call_count == 4

        # Verify calls and their arguments in order
        from unittest.mock import call
        self.node.driver.set_motor.assert_has_calls([
            call(17, 0, 0, 0),
            call(17, 17, 0, 0),
            call(17, 17, 17, 0),
            call(17, 17, 17, 17)
        ], any_order=False)
    