# Run like this: colcon test --packages-select odometer <- does not work
# Run like this: pytest src/odometer/test/test_odometer.py

import math
import pytest
import rclpy
from std_msgs.msg import Int32MultiArray
from odometer.odometer_node import OdometerNode

@pytest.fixture(autouse=True)
def setup_teardown():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_odometer_straight_line_forward():
    node = OdometerNode()

    # Generate first fake message. Odometer compares two values!
    start_msg = Int32MultiArray()
    start_msg.data = [0, 0, 0, 0]
    node.process_encoders(start_msg)

    # Generate fake message: Robot runs straight line
    # All wheels should spin just one round (1320 pulses)
    msg = Int32MultiArray()
    msg.data = [1320, 1320, 1320, 1320]

    # Call calculation function directly
    node.process_encoders(msg)

    # Distance of one wheel turn = pii * diameter (0.065m) = n. 0.2041 meters
    expected_distance = math.pi * 0.065

    # Assert that code calculated the X-coordinate correctly with small tolerance
    assert pytest.approx(node.x, abs=0.001) == expected_distance
    assert node.y == 0.0  # When running straight Y value should be zero
    assert node.th == 0.0 # When running straight angle should be zero
    # X-coordinate must be positive as robot moves forward
    assert node.x > 0.0

def test_odometer_straight_line_backward():
    node = OdometerNode()

    # Generate first fake message Odometer compares two values!
    start_msg = Int32MultiArray()
    start_msg.data = [0, 0, 0, 0]
    node.process_encoders(start_msg)

    # Generate fake message: Robot runs straight line
    # All wheels should spin just one round (1320 pulses)
    msg = Int32MultiArray()
    msg.data = [-1320, -1320, -1320, -1320]

    # Call calculation function directly
    node.process_encoders(msg)

    # Distance of one wheel turn = pii * diameter (0.065m) = n. 0.2041 meters
    expected_distance = math.pi * 0.065

    # Assert that code calculated the X-coordinate correctly with small tolerance
    assert pytest.approx(-node.x, abs=0.001) == expected_distance
    assert node.y == 0.0  # When running straight Y value should be zero
    assert node.th == 0.0 # When running straight angle should be zero
    # X-coordinate must be negative as robot moves backward
    assert node.x < 0.0

def test_odometer_turn_left_forward():
    node = OdometerNode()

    # Generate first fake message. Odometer compares two values!
    init_msg = Int32MultiArray()
    init_msg.data = [0, 0, 0, 0]
    node.process_encoders(init_msg)

    # Make a steering message that makes robot move forward left.
    # Right side wheels (index 1 ja 3) runs more pulses than left side wheels (index 0 ja 2).
    # Example: Left side 900 pulses, light side 1100 pulses. (R = 1.0m)
    msg = Int32MultiArray()
    msg.data = [900, 1100, 900, 1100]
    node.process_encoders(msg)

    # Calculate expected movement in meters
    pulse_to_meter = (math.pi * node.wheel_diameter) / node.counts_per_revolution
    d_left = 900 * pulse_to_meter
    d_right = 1100 * pulse_to_meter

    expected_d_center = (d_right + d_left) / 2.0
    expected_d_theta = (d_right - d_left) / node.wheel_separation


    # Check that calculation result is expected
    assert node.x == pytest.approx(expected_d_center, abs=0.001)
    assert node.y == pytest.approx(0.0, abs=0.001)
    # X-coordinate must be positive as robot moves forward
    assert node.x > 0.0
    # Angle (theta) must be positive as robot turns left
    assert node.th == pytest.approx(expected_d_theta, abs=0.001)
    assert node.th > 0.0

def test_odometer_turn_left_backward():
    node = OdometerNode()

    # Generate first fake message Odometer compares two values!
    init_msg = Int32MultiArray()
    init_msg.data = [0, 0, 0, 0]
    node.process_encoders(init_msg)

    # Make a steering message that makes robot move forward left.
    # Right side wheels (index 1 ja 3) runs more pulses than left side wheels (index 0 ja 2).
    # Example: Left side -900 pulses, light side -1100 pulses. Pulses are negative when moved backwards.
    msg = Int32MultiArray()
    msg.data = [-900, -1100, -900, -1100]

    node.process_encoders(msg)

    pulse_to_meter = (math.pi * node.wheel_diameter) / node.counts_per_revolution
    d_left = -900 * pulse_to_meter
    d_right = -1100 * pulse_to_meter

    expected_d_center = (d_right + d_left) / 2.0
    expected_d_theta = (d_right - d_left) / node.wheel_separation

    # Check that calculation result is expected
    assert node.x == pytest.approx(expected_d_center, abs=0.001)
    assert node.y == pytest.approx(0.0, abs=0.001)
    # X-coordinate must be negative as robot moves backward
    assert node.x < 0.0
    # Angle (theta) must be negative as robot turns right
    assert node.th == pytest.approx(expected_d_theta, abs=0.001)
    assert node.th < 0.0

def test_odometer_turn_right_forward():
    node = OdometerNode()

    # Generate first fake message. Odometer compares two values!
    init_msg = Int32MultiArray()
    init_msg.data = [0, 0, 0, 0]
    node.process_encoders(init_msg)

    # Make a steering message that makes robot move forward right.
    # Left side wheels (index 2 ja 4) runs more pulses than right side wheels (index 1 ja 3).
    # Example: Right side 900 pulses, left side 1100 pulses.
    msg = Int32MultiArray()
    msg.data = [1100, 900, 1100, 900]

    node.process_encoders(msg)

    # Calculate expected movement in meters
    pulse_to_meter = (math.pi * node.wheel_diameter) / node.counts_per_revolution
    d_left = 1100 * pulse_to_meter
    d_right = 900 * pulse_to_meter

    expected_d_center = (d_right + d_left) / 2.0
    expected_d_theta = (d_right - d_left) / node.wheel_separation


    # Check that calculation result is expected
    assert node.x == pytest.approx(expected_d_center, abs=0.001)
    assert node.y == pytest.approx(0.0, abs=0.001)
    # X-coordinate must be positive as robot moves forward
    assert node.x > 0.0
    # Angle (theta) must be negative as robot turns right
    assert node.th == pytest.approx(expected_d_theta, abs=0.001)
    assert node.th < 0.0

def test_odometer_turn_right_forward():
    node = OdometerNode()

    # Generate first fake message. Odometer compares two values!
    init_msg = Int32MultiArray()
    init_msg.data = [0, 0, 0, 0]
    node.process_encoders(init_msg)

    # Make a steering message that makes robot move forward right.
    # Left side wheels (index 2 ja 4) runs more pulses than right side wheels (index 1 ja 3).
    # Example: Right side 900 pulses, left side 1100 pulses.  Pulses are negative when moved backwards.
    msg = Int32MultiArray()
    msg.data = [-1100, -900, -1100, -900]

    node.process_encoders(msg)

    # Calculate expected movement in meters
    pulse_to_meter = (math.pi * node.wheel_diameter) / node.counts_per_revolution
    d_left = -1100 * pulse_to_meter
    d_right = -900 * pulse_to_meter

    expected_d_center = (d_right + d_left) / 2.0
    expected_d_theta = (d_right - d_left) / node.wheel_separation

    # Check that calculation result is expected
    assert node.x == pytest.approx(expected_d_center, abs=0.001)
    assert node.y == pytest.approx(0.0, abs=0.001)
    # X-coordinate must be negative as robot moves forward
    assert node.x < 0.0
    # Angle (theta) must be positive as robot turns right
    assert node.th == pytest.approx(expected_d_theta, abs=0.001)
    assert node.th > 0.0
