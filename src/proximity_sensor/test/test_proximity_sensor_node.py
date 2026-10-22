# colcon build --symlink-install
# source install/setup.bash
# pytest src/proximity_sensor/test/test_proximity_sensor_node.py -s

import unittest
from unittest.mock import patch
import json
import pytest
import rclpy
from test_fake_hardware_interface import FakeVL53L1XHardwareInterface
from proximity_sensor.proximity_sensor_node import ProximitySensorNode, String

@pytest.fixture(autouse=True)
def setup_teardown():
    rclpy.init()
    yield
    rclpy.shutdown()

class TestProximitySensorNode(unittest.TestCase):

    def setUp(self):
        """Init node and its internal HW objects"""

        # Hijack node internal import and replace it with the fake HW class
        self.patcher = patch(
            'proximity_sensor.proximity_sensor_node.VL53L1XHardwareInterface',
            new=FakeVL53L1XHardwareInterface
        )
        self.patcher.start()

        # Create tested node
        self.node = ProximitySensorNode()

    def test_initial_state_and_sensor_setup(self):

        back_right_sensor = self.node.sensors.get('back_right')
        assert back_right_sensor is not None

        back_right_sensor = self.node.sensors.get('back_right')
        assert back_right_sensor is not None

        # Assert that face class was imported
        assert isinstance(back_right_sensor.sensor, FakeVL53L1XHardwareInterface)

    def test_measure_distance_while_moving_backward(self):

        self.node.set_move_direction("Backward")
        # fake new distance measurement 0mm
        self.node.sensors['back_right'].sensor.current_distance = 201
        self.node.sensors['back_left'].sensor.current_distance = 200

        # Check that backward sensors were started ranging
        assert self.node.sensors['back_right'].sensor.ranging_started is True
        assert self.node.sensors['back_left'].sensor.ranging_started is True

        # Create spy to detect published messages
        published_messages = []
        def mock_publish(msg):
            published_messages.append(msg)
        self.node.pub_range.publish = mock_publish

        self.node.read_and_publish()

        # Two sensors should be read and results published (back_left ja back_right)
        assert len(published_messages) == 2

        # check back_left sensor message content
        back_left_msg = next((json.loads(m.data) for m in published_messages if '"sensor": "back_left"' in m.data), None)
        assert back_left_msg is not None
        assert back_left_msg["distance"] == 200.0

        # check back_right sensor message content
        back_right_msg = next((json.loads(m.data) for m in published_messages if '"sensor": "back_right"' in m.data), None)
        assert back_right_msg is not None
        assert back_right_msg["distance"] == 201.0

    def test_move_direction_callback(self):
        msg = String(data="Forward")
        self.node.set_move_direction_callback(msg)

        assert self.node.move_direction == "Forward"

    def test_move_direction_change(self):
        self.node.move_direction = "Forward"

        msg = String(data="Backward")
        self.node.set_move_direction_callback(msg)

        assert self.node.move_direction == "Backward"

if __name__ == '__main__':
    unittest.main()
