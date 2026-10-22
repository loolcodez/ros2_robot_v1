# colcon build --symlink-install
# source install/setup.bash
# pytest src/proximity_sensor/test/test_VL53L1X_class.py -s

import unittest
import json
from unittest.mock import MagicMock
from test_fake_hardware_interface import FakeVL53L1XHardwareInterface
from proximity_sensor.proximity_sensor_node import VL53L1X

class TestSensorWithCleanArchitecture(unittest.TestCase):

    def setUp(self):
        """Initialize plain sensor class using fake logger"""
        self.mock_node = MagicMock()  # Logger is used from Node
        self.fake_hw = FakeVL53L1XHardwareInterface("back_left")

        # Create sensor class with fake HW
        self.sensor = VL53L1X(self.mock_node, "back_left", self.fake_hw)
        self.sensor.start_ranging()

    def test_filter_invalid_measurements(self):
        # Ensure that, invalid distance values are dropped (<1 tai >4000)

        # Test lower limit (0 mm)
        # fake new distance measurement 0mm
        self.fake_hw.current_distance = 0

        res = self.sensor.read_sensor()
        assert res is None  # Should be dropped

        # Test upper limit (4000 mm)
        # fake new distance measurement 0mm
        self.fake_hw.current_distance = 4001

        res = self.sensor.read_sensor()
        assert res is None  # Should be dropped

    def test_moving_average_calculation(self):
        """Test that averaging works correctly with ring buffer"""

        # Input two distances in a row: 100mm ja 200mm
        self.fake_hw.current_distance = 100
        self.sensor.read_sensor()

        self.fake_hw.current_distance = 200
        msg = self.sensor.read_sensor()

        # Average should be (100 + 200) / 2 = 150
        assert msg is not None
        data = json.loads(msg.data)
        assert data["distance"] == 150.0

    def test_cleanup(self):
        self.sensor.cleanup()

if __name__ == '__main__':
    unittest.main()
