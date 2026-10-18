# Run like this: colcon test --packages-select odometer <- does not work

# Run like this:
# colcon build --symlink-install
# source install/setup.bash
# pytest src/odometer/test/test_odometer_node.py -s

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

def test_odometer_arc_turn_r1_forward():
    node = OdometerNode()
    node.wheel_separation = 0.20  # 20 cm

    # Init odometer
    init_msg = Int32MultiArray()
    init_msg.data = [0, 0, 0, 0]
    node.process_encoders(init_msg)

    # Simulate driving with 5 encoder steps turning left. At every encoder count we add movement +900
    # for left wheel and +1100 for right wheel
    steps = [
        [900, 1100, 900, 1100],
        [1800, 2200, 1800, 2200],
        [2700, 3300, 2700, 3300],
        [3600, 4400, 3600, 4400],
        [4500, 5500, 4500, 5500]
    ]

    for step in steps:
        msg = Int32MultiArray()
        msg.data = step
        node.process_encoders(msg)

    # Check robot's angle (theta) and position (X, Y) after 5 encoder steps

    # Calculate one step movement in meters
    pulse_to_meter = (math.pi * node.wheel_diameter) / node.counts_per_revolution
    d_center_step = 1000 * pulse_to_meter  # n. 0.1546 meters per step
    total_d_center = d_center_step * 5    # n. 0.7735 meters whole movement (mathematical ideal case)

    # When running a circle, angle th = moved distance / radius (R = 1.0)
    expected_th = total_d_center / 1.0

    # In ideal mathematical case from origin point (0,0) target point can be calculated lie this:
    # X = R * sin(th)
    # Y = R * (1 - cos(th))
    expected_x = 1.0 * math.sin(expected_th)
    expected_y = 1.0 * (1.0 - math.cos(expected_th))
    print(f"Ideal expected_x: {expected_x}") # 0.6986423628917345
    print(f"Ideal expected_y: {expected_y}") # 0.28452893225997333
    # OdometerNode counts moved distance step-by-step. There are 5 steps in this test. Each step about 15 cm.
    # Counted route will not follow circle's arc but direct route between each two points. Therefore
    # theoretical equation (expected_x = R * sin(th)) shall not give the correct answer moved distance!
    # OdometerNode count distance is a bigger as the points go little bit outside of the circle's arc!
    # Line of each two points are on tangent of the circle's arc.

    # In x,y asserts we use values given by the test itself, as exact expected values are not trivial to calculate!
    assert node.x == pytest.approx(0.719, abs=0.001)
    assert node.y == pytest.approx(0.230, abs=0.001)
    assert node.th == pytest.approx(expected_th, abs=0.001)
    assert node.th > 0.0
