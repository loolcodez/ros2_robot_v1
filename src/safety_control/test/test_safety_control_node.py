# colcon test --packages-select safety_control
# colcon build --symlink-install
# source install/setup.bash
# pytest src/safety_control/test/test_safety_control_node.py -s

import json
import unittest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from safety_control.safety_control_node import SafetyControlNode
from rclpy.lifecycle import LifecycleState
class TestSafetyControlNode(unittest.TestCase):
        
    def setUp(self):
        rclpy.init()
        self.node = SafetyControlNode()
        self.node.on_configure(LifecycleState("", 1))
        self.node.on_activate(LifecycleState("", 3))

    def tearDown(self):
        # Destroy node after each test case run
        if hasattr(self, 'node') and self.node:
            self.node.on_deactivate(LifecycleState("", 4))
            self.node.on_cleanup(LifecycleState("", 2))
            self.node.destroy_node()
        rclpy.shutdown()

    def test_initial_state(self):
        """Verify initial values"""
        assert self.node.speed_limit_factor == 1
        assert self.node.current_distance_mm == 9999
        assert self.node.safety_distance_mm == 300

    def test_range_callback_updates_distance(self):
        """Verify that proximity sensor updates internal state of the nope"""
        msg = String()
        msg.data = json.dumps({"sensor": "front_left_tof", "distance": 150})
        self.node.range_callback(msg)

        assert self.node.sensor == "front_left_tof"
        assert self.node.current_distance_mm == 150

    def test_power_alert_callback(self):
        """Verify low battery alert"""
        test_cases = [
            ("Low", 0.5),
            ("Very Low", 0.0),
            ("Critical", 0.0),
            ("Good", 1.0),
            ("Full", 1.0)
        ]
        for alert_status, expected_factor in test_cases:
            # subtest runs all test even some fails
            with self.subTest(alert_status=alert_status):
                msg = String(data=alert_status)
                self.node.power_alert_callback(msg)
                self.assertEqual(self.node.speed_limit_factor, expected_factor)

    def test_cmd_vel_in_callback_safety_stop(self):
        """Verify safety stop"""
        # Set road block inside safety marginal (300mm)
        self.node.current_distance_mm = 200

        # Create a message which request running forward
        twist_msg = Twist()
        twist_msg.linear.x = 20.0
        twist_msg.angular.z = 5.0

        self.node.cmd_vel_in_callback(twist_msg)

        # Road block ahead. Speed should go to zero
        assert twist_msg.linear.x == 0.0
        assert twist_msg.angular.z == 0.0

    def test_cmd_vel_in_callback_safe_distance(self):
        """Verify that not effect if road block is outside safety marginal"""
        self.node.current_distance_mm = 500

        twist_msg = Twist()
        twist_msg.linear.x = 20.0
        twist_msg.angular.z = 5.0

        self.node.cmd_vel_in_callback(twist_msg)

        # Values should not change (speed_limit_factor = 1.0)
        assert twist_msg.linear.x == 20.0
        assert twist_msg.angular.z == 5.0
