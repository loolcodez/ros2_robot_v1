# colcon test --packages-select safety_control
# colcon build --symlink-install
# source install/setup.bash
# pytest src/safety_control/test/test_safety_control_node.py

import json
import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from safety_control.safety_control_node import SafetyControlNode

@pytest.fixture(scope="session", autouse=True)
def rclpy_init():
    """Init rclpy context once"""
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def test_node():
    """Create new node for every test case"""
    node = SafetyControlNode()
    yield node
    node.destroy_node()

def test_initial_state(node):
    """Verify initial values"""
    assert node.speed_limit_factor == 1
    assert node.current_distance_mm == 9999
    assert node.safety_distance_mm == 300

def test_range_callback_updates_distance(node):
    """Verify that proximity sensor updates internal state of the nope"""
    msg = String()
    msg.data = json.dumps({"sensor": "front_left_tof", "distance": 150})
    node.range_callback(msg)

    assert node.sensor == "front_left_tof"
    assert node.current_distance_mm == 150

@pytest.mark.parametrize("alert_status, expected_factor", [
    ("Low", 0.5),
    ("Very Low", 0.0),
    ("Critical", 0.0),
    ("Normal", 1.0) # Test return to normal
])
def test_power_alert_callback(node, alert_status, expected_factor):
    """Verify low battery alert"""
    msg = String(data=alert_status)
    node.power_alert_callback(msg)
    assert node.speed_limit_factor == expected_factor

def test_cmd_vel_in_callback_safety_stop(node):
    """Verify safety stop"""
    # Set road block inside safety marginal (300mm)
    node.current_distance_mm = 200

    # Create a message which request running forward
    twist_msg = Twist()
    twist_msg.linear.x = 20.0
    twist_msg.angular.z = 5.0

    node.cmd_vel_in_callback(twist_msg)

    # Road block ahead. Speed should go to zero
    assert twist_msg.linear.x == 0.0
    assert twist_msg.angular.z == 0.0

def test_cmd_vel_in_callback_safe_distance(node):
    """Verify that not effect if road block is outside safety marginal"""
    node.current_distance_mm = 500

    twist_msg = Twist()
    twist_msg.linear.x = 20.0
    twist_msg.angular.z = 5.0

    node.cmd_vel_in_callback(twist_msg)

    # Values should not change (speed_limit_factor = 1.0)
    assert twist_msg.linear.x == 20.0
    assert twist_msg.angular.z == 5.0
