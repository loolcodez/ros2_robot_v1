# colcon build --symlink-install
# source install/setup.bash
# pytest src/light_control/test/test_light_control_node.py

# import sys
import unittest
from unittest.mock import MagicMock, patch
import pytest
import rclpy
from light_control.light_control_node import LightsControlNode, LightState, String

@pytest.fixture(autouse=True)
def setup_teardown():
    rclpy.init()
    yield
    rclpy.shutdown()

# Mokataan 'periphery' -kirjasto ennen lataamista, jotta testi ei kaadu puuttuvaan laitteistoon
#sys.modules['periphery'] = MagicMock()

# Tuodaan testattavat luokat (oletetaan että koodisi tiedoston nimi on lights_node.py)
# Korvaa 'lights_node' omalla tiedostonimelläsi, jos se on eri

class TestLightsControlNode(unittest.TestCase):

    @patch('light_control.lights_node.rclpy')  # Mokataan rclpy, jotta testi ei vaadi käynnissä olevaa ROS2-alustaa
    @patch('light_control.lights_node.GPIO')   # Mokataan GPIO-pinnit
    def setUp(self, mock_gpio, mock_rclpy):
        """Alustetaan testattava node puhtaalla tilalla ennen jokaista testiä."""
        self.node = LightsControlNode()
        self.gpio_object = MagicMock()

    def send_command(self, command_str: str):
        """Apufunktio komentojen syöttämiseen suoraan callback-funktioon."""
        msg = String()
        msg.data = command_str
        self.node.command_callback(msg)

    def test_front_lights_on_and_off(self):
        """Testataan 'lights/front_on' ja 'lights/front_off' komennot."""
        self.send_command("lights/front_on")
        self.assertEqual(self.node.lights['front_left'].state, LightState.ON)
        self.assertEqual(self.node.lights['front_right'].state, LightState.ON)

        self.send_command("lights/front_off")
        self.assertEqual(self.node.lights['front_left'].state, LightState.OFF)
        self.assertEqual(self.node.lights['front_right'].state, LightState.OFF)

    def test_back_lights_on_and_off(self):
        """Testataan 'lights/back_on' ja 'lights/back_off' komennot."""
        self.send_command("lights/back_on")
        self.assertEqual(self.node.lights['back_left'].state, LightState.ON)
        self.assertEqual(self.node.lights['back_right'].state, LightState.ON)

        self.send_command("lights/back_off")
        self.assertEqual(self.node.lights['back_left'].state, LightState.OFF)
        self.assertEqual(self.node.lights['back_right'].state, LightState.OFF)

    def test_emergency_lights_on_and_off(self):
        """Testataan hätävilkkujen kytkeytyminen ja vilkutustilan aktivointi."""
        # Laitetaan ensin etuvalot päälle, jotta nähdään muuttuuko tila oikein BLINKING_AND_ON
        self.send_command("lights/front_on")

        # Hätävilkut päälle
        self.send_command("lights/emergency_on")
        self.assertTrue(self.node.light_blinking)
        self.assertEqual(self.node.lights['front_left'].state, LightState.BLINKING_AND_ON)
        self.assertEqual(self.node.lights['back_left'].state, LightState.BLINKING_AND_OFF)

        # Hätävilkut pois päältä -> pitäisi palautua alkuperäisiin tiloihin
        self.send_command("lights/emergency_off")
        self.assertFalse(self.node.light_blinking)
        self.assertEqual(self.node.lights['front_left'].state, LightState.ON)
        self.assertEqual(self.node.lights['back_left'].state, LightState.OFF)

    def test_all_lights_off(self):
        """Testataan 'lights/all_off' komento, joka sammuttaa kaiken ja pysäyttää vilkun."""
        # Laitetaan kaikki sekaisin päälle
        self.send_command("lights/front_on")
        self.send_command("lights/emergency_on")

        # Kaikki pois
        self.send_command("lights/all_off")
        self.assertFalse(self.node.light_blinking)
        for name, light in self.node.lights.items():
            self.assertEqual(light.state, LightState.OFF, f"Valon {name} piti sammua")

if __name__ == '__main__':
    unittest.main()
