# Run like this: colcon test --packages-select odometer <- does not work

# Run like this:
# colcon build --symlink-install
# source install/setup.bash
# pytest src/odometer/test/test_odometer_node.py -s

import math
import unittest
from unittest.mock import patch
import pytest
import rclpy
from std_msgs.msg import Int32MultiArray
from odometer.odometer_node import OdometerNode
from rclpy.lifecycle import LifecycleState

@pytest.fixture(autouse=True)
def setup_teardown():
    rclpy.init()
    yield
    rclpy.shutdown()

class TestOdometerNode(unittest.TestCase):

    def setUp(self):
        """Init node and its internal HW objects"""
        self.node = OdometerNode()
        self.node.on_configure(LifecycleState("",1))
        self.node.is_activated = True

    def test_odometer_straight_line_forward(self):

        # Generate first fake message. Odometer compares two values!
        start_msg = Int32MultiArray()
        start_msg.data = [0, 0, 0, 0]
        self.node.process_encoders(start_msg)

        # Generate fake message: Robot runs straight line
        # All wheels should spin just one round (1320 pulses)
        msg = Int32MultiArray()
        msg.data = [1320, 1320, 1320, 1320]

        # Call calculation function directly
        self.node.process_encoders(msg)

        # Distance of one wheel turn = pii * diameter (0.065m) = 0.2041 meters
        expected_distance = math.pi * 0.065
        
        # Assert that code calculated the X-coordinate correctly with small tolerance
        assert pytest.approx(self.node.x, abs=0.001) == expected_distance
        assert self.node.y == 0.0  # When running straight Y value should be zero
        assert self.node.th == 0.0 # When running straight angle should be zero
        # X-coordinate must be positive as robot moves forward
        assert self.node.x > 0.0

    def test_odometer_straight_line_backward(self):

        # Generate first fake message Odometer compares two values!
        start_msg = Int32MultiArray()
        start_msg.data = [0, 0, 0, 0]
        self.node.process_encoders(start_msg)

        # Generate fake message: Robot runs straight line
        # All wheels should spin just one round (1320 pulses)
        msg = Int32MultiArray()
        msg.data = [-1320, -1320, -1320, -1320]
        self.node.process_encoders(msg)

        # Distance of one wheel turn = pii * diameter (0.065m) = n. 0.2041 meters
        expected_distance = math.pi * 0.065

        # Assert that code calculated the X-coordinate correctly with small tolerance
        assert pytest.approx(-self.node.x, abs=0.001) == expected_distance
        assert self.node.y == 0.0  # When running straight Y value should be zero
        assert self.node.th == 0.0 # When running straight angle should be zero
        # X-coordinate must be negative as robot moves backward
        assert self.node.x < 0.0

    def test_odometer_turn_left_forward(self):

        # Generate first fake message. Odometer compares two values!
        init_msg = Int32MultiArray()
        init_msg.data = [0, 0, 0, 0]
        self.node.process_encoders(init_msg)

        # Make a steering message that makes robot move forward left.
        # Right side wheels (index 1 ja 3) runs more pulses than left side wheels (index 0 ja 2).
        # Example: Left side 900 pulses, light side 1100 pulses. (R = 1.0m)
        msg = Int32MultiArray()
        msg.data = [900, 1100, 900, 1100]
        self.node.process_encoders(msg)

        # Calculate expected movement in meters
        pulse_to_meter = (math.pi * self.node.wheel_diameter) / self.node.counts_per_revolution
        d_lf = msg.data[0] * pulse_to_meter  # Left Front
        d_rf = msg.data[1] * pulse_to_meter  # Right Front
        d_lr = msg.data[2] * pulse_to_meter  # Left Rear
        d_rr = msg.data[3] * pulse_to_meter  # Right Rear

        expected_d_center = (d_lf + d_rf + d_lr + d_rr) / 4.0
        W = self.node.wheel_separation  # Left - right distance
        L = self.node.wheel_base        # Front - back axis distance
        expected_d_theta = ((d_rf + d_rr) - (d_lf + d_lr)) / (2.0 * (W + L)) 

        # Check that calculation result is expected
        assert self.node.x == pytest.approx(expected_d_center, abs=0.001)
        assert self.node.y == pytest.approx(0.0, abs=0.001)
        # X-coordinate must be positive as robot moves forward
        assert self.node.x > 0.0
        # Angle (theta) must be positive as robot turns left
        assert self.node.th == pytest.approx(expected_d_theta, abs=0.001)
        assert self.node.th > 0.0

    def test_odometer_turn_left_backward(self):

        # Generate first fake message Odometer compares two values!
        init_msg = Int32MultiArray()
        init_msg.data = [0, 0, 0, 0]
        self.node.process_encoders(init_msg)

        # Make a steering message that makes robot move forward left.
        # Right side wheels (index 1 ja 3) runs more pulses than left side wheels (index 0 ja 2).
        # Example: Left side -900 pulses, light side -1100 pulses. Pulses are negative when moved backwards.
        msg = Int32MultiArray()
        msg.data = [-900, -1100, -900, -1100]
        self.node.process_encoders(msg)

        pulse_to_meter = (math.pi * self.node.wheel_diameter) / self.node.counts_per_revolution
        d_lf = msg.data[0] * pulse_to_meter  # Left Front
        d_rf = msg.data[1] * pulse_to_meter  # Right Front
        d_lr = msg.data[2] * pulse_to_meter  # Left Rear
        d_rr = msg.data[3] * pulse_to_meter  # Right Rear

        expected_d_center = (d_lf + d_rf + d_lr + d_rr) / 4.0
        W = self.node.wheel_separation  # Left - right distance
        L = self.node.wheel_base        # Front - back axis distance
        expected_d_theta = ((d_rf + d_rr) - (d_lf + d_lr)) / (2.0 * (W + L)) 

        # Check that calculation result is expected
        assert self.node.x == pytest.approx(expected_d_center, abs=0.001)
        assert self.node.y == pytest.approx(0.0, abs=0.001)
        # X-coordinate must be negative as robot moves backward
        assert self.node.x < 0.0
        # Angle (theta) must be negative as robot turns right
        assert self.node.th == pytest.approx(expected_d_theta, abs=0.001)
        assert self.node.th < 0.0

    def test_odometer_turn_right_forward(self):

        # Generate first fake message. Odometer compares two values!
        init_msg = Int32MultiArray()
        init_msg.data = [0, 0, 0, 0]
        self.node.process_encoders(init_msg)

        # Make a steering message that makes robot move forward right.
        # Left side wheels (index 2 ja 4) runs more pulses than right side wheels (index 1 ja 3).
        # Example: Right side 900 pulses, left side 1100 pulses.
        msg = Int32MultiArray()
        msg.data = [1100, 900, 1100, 900]
        self.node.process_encoders(msg)

        # Calculate expected movement in meters
        pulse_to_meter = (math.pi * self.node.wheel_diameter) / self.node.counts_per_revolution
        d_lf = msg.data[0] * pulse_to_meter  # Left Front
        d_rf = msg.data[1] * pulse_to_meter  # Right Front
        d_lr = msg.data[2] * pulse_to_meter  # Left Rear
        d_rr = msg.data[3] * pulse_to_meter  # Right Rear

        expected_d_center = (d_lf + d_rf + d_lr + d_rr) / 4.0
        W = self.node.wheel_separation  # Left - right distance
        L = self.node.wheel_base        # Front - back axis distance
        expected_d_theta = ((d_rf + d_rr) - (d_lf + d_lr)) / (2.0 * (W + L)) 

        # Check that calculation result is expected
        assert self.node.x == pytest.approx(expected_d_center, abs=0.001)
        assert self.node.y == pytest.approx(0.0, abs=0.001)
        # X-coordinate must be positive as robot moves forward
        assert self.node.x > 0.0
        # Angle (theta) must be negative as robot turns right
        assert self.node.th == pytest.approx(expected_d_theta, abs=0.001)
        assert self.node.th < 0.0

    def test_odometer_turn_right_backward(self):

        # Generate first fake message. Odometer compares two values!
        init_msg = Int32MultiArray()
        init_msg.data = [0, 0, 0, 0]
        self.node.process_encoders(init_msg)

        # Make a steering message that makes robot move forward right.
        # Left side wheels (index 2 ja 4) runs more pulses than right side wheels (index 1 ja 3).
        # Example: Right side 900 pulses, left side 1100 pulses.  Pulses are negative when moved backwards.
        msg = Int32MultiArray()
        msg.data = [-1100, -900, -1100, -900]
        self.node.process_encoders(msg)

        # Calculate expected movement in meters
        pulse_to_meter = (math.pi * self.node.wheel_diameter) / self.node.counts_per_revolution
        d_lf = msg.data[0] * pulse_to_meter  # Left Front
        d_rf = msg.data[1] * pulse_to_meter  # Right Front
        d_lr = msg.data[2] * pulse_to_meter  # Left Rear
        d_rr = msg.data[3] * pulse_to_meter  # Right Rear

        expected_d_center = (d_lf + d_rf + d_lr + d_rr) / 4.0
        W = self.node.wheel_separation  # Left - right distance
        L = self.node.wheel_base        # Front - back axis distance
        expected_d_theta = ((d_rf + d_rr) - (d_lf + d_lr)) / (2.0 * (W + L)) 

        # Check that calculation result is expected
        assert self.node.x == pytest.approx(expected_d_center, abs=0.001)
        assert self.node.y == pytest.approx(0.0, abs=0.001)
        # X-coordinate must be negative as robot moves forward
        assert self.node.x < 0.0
        # Angle (theta) must be positive as robot turns right
        assert self.node.th == pytest.approx(expected_d_theta, abs=0.001)
        assert self.node.th > 0.0

    def test_odometer_arc_turn_r1_forward(self):

        self.node.wheel_separation = 0.20  # 20 cm
        self.node.wheel_base = 0.155 # 15.5 cm
        # Init odometer
        init_msg = Int32MultiArray()
        init_msg.data = [0, 0, 0, 0]
        self.node.process_encoders(init_msg)

        # Simulate driving with 5 encoder steps turning left. At every encoder count we add movement +900
        # for left wheel and +1100 for right wheel
        steps = [
            [900, 1100, 900, 1100],
            [1800, 2200, 1800, 2200],
            [2700, 3300, 2700, 3300],
            [3600, 4400, 3600, 4400],
            [4500, 5500, 4500, 5500]
        ]

        # Check robot's angle (theta) and position (X, Y) after 5 encoder steps
        # Calculate one step movement in meters
        pulse_to_meter = (math.pi * self.node.wheel_diameter) / self.node.counts_per_revolution
        
        # Calculate one step movement and change of angle
        d_lf_step = 900 * pulse_to_meter
        d_rf_step = 1100 * pulse_to_meter
        d_lr_step = 900 * pulse_to_meter
        d_rr_step = 1100 * pulse_to_meter

        d_center_step = (d_lf_step + d_rf_step + d_lr_step + d_rr_step) / 4.0
        
        W = self.node.wheel_separation
        L = self.node.wheel_base if hasattr(self.node, 'wheel_base') else 0.0
        d_theta_step = ((d_rf_step + d_rr_step) - (d_lf_step + d_lr_step)) / (2.0 * (W + L))

        # Simulate 5 separate steps from start point (0,0,0) using midpoint method
        sim_x, sim_y, sim_th = 0.0, 0.0, 0.0
        for _ in range(5):
            th_mid = sim_th + (d_theta_step / 2.0)
            sim_x += d_center_step * math.cos(th_mid)
            sim_y += d_center_step * math.sin(th_mid)
            sim_th += d_theta_step
            sim_th = math.atan2(math.sin(sim_th), math.cos(sim_th))

        # Run steps
        for step in steps:
            msg = Int32MultiArray()
            msg.data = step
            self.node.process_encoders(msg)

        # In asserts we accept small error as the odometer way to calculate movement does not exactly match with simulated movement
        assert self.node.x == pytest.approx(sim_x, abs=0.02)
        assert self.node.y == pytest.approx(sim_y, abs=0.04)
        assert self.node.th == pytest.approx(sim_th, abs=0.001)

        print(f"Ideal expected_x: {sim_x}") # 0.7494857324780642
        print(f"Ideal expected_y: {sim_y}") # 0.16593734744069122
        print(f"Odometer x: {self.node.x}") # 0.756003037013165
        print(f"Odometer y: {self.node.y}") # 0.1331295402632495
