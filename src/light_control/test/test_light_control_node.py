# colcon build --symlink-install
# source install/setup.bash
# pytest src/light_control/test/test_light_control_node.py

import unittest
from unittest.mock import MagicMock
import pytest
import rclpy
from light_control.light_control_node import LightsControlNode, LightState, String

@pytest.fixture(autouse=True)
def setup_teardown():
    rclpy.init()
    yield
    rclpy.shutdown()

class TestLightsControlNode(unittest.TestCase):

    def setUp(self):
        """Init node before each test case run"""
        self.node = LightsControlNode()
        self.gpio_object = MagicMock()

    def send_command(self, command_str: str):
        """Help function to inject commands to subscriber callback function"""
        msg = String()
        msg.data = command_str
        self.node.command_callback(msg)

    def test_front_lights_on_and_off(self):
        self.send_command("lights/front_on")
        self.assertEqual(self.node.lights['front_left'].state, LightState.ON)
        self.assertEqual(self.node.lights['front_right'].state, LightState.ON)

        self.send_command("lights/front_off")
        self.assertEqual(self.node.lights['front_left'].state, LightState.OFF)
        self.assertEqual(self.node.lights['front_right'].state, LightState.OFF)

    def test_back_lights_on_and_off(self):
        self.send_command("lights/back_on")
        self.assertEqual(self.node.lights['back_left'].state, LightState.ON)
        self.assertEqual(self.node.lights['back_right'].state, LightState.ON)

        self.send_command("lights/back_off")
        self.assertEqual(self.node.lights['back_left'].state, LightState.OFF)
        self.assertEqual(self.node.lights['back_right'].state, LightState.OFF)

    def test_emergency_lights_on_and_off(self):
        self.send_command("lights/front_on")

        # Emergency lights ON
        self.send_command("lights/emergency_on")
        self.assertTrue(self.node.light_blinking)
        self.assertEqual(self.node.lights['front_left'].state, LightState.BLINKING_AND_ON)
        self.assertEqual(self.node.lights['back_left'].state, LightState.BLINKING_AND_OFF)

        # Emergency lights OFF -> lights should restore to original state
        self.send_command("lights/emergency_off")
        self.assertFalse(self.node.light_blinking)
        self.assertEqual(self.node.lights['front_left'].state, LightState.ON)
        self.assertEqual(self.node.lights['back_left'].state, LightState.OFF)

    def test_all_lights_off(self):
        # All front lights ON then emergency lights ON
        self.send_command("lights/front_on")
        self.send_command("lights/emergency_on")

        # All off
        self.send_command("lights/all_off")
        self.assertFalse(self.node.light_blinking)
        for name, light in self.node.lights.items():
            self.assertEqual(light.state, LightState.OFF, f"Light {name} should have turned OFF")

if __name__ == '__main__':
    unittest.main()
