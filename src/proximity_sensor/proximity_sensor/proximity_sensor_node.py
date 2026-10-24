# i2c_bus = 0
# SDA 22 (TWI0_SDA) Green wire
# SCL 15 (TWI0_SCL) Yellow wire
# XSHUT front-left 29/PI0/256 Gray wire
# XSHUT front-right 31/PI15/271 Gray wire
# XSHUT back-left 35/PI2/258 Gray wire
# XSHUT back-right 37/PI16/272 Gray wire

import json
from enum import Enum
from typing import Final, Dict
from collections import deque
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecyclePublisher
from std_msgs.msg import String
from proximity_sensor.hardware_interface import VL53L1XHardwareInterface

class GPIO_pins(Enum):
    FRONT_LEFT = 256
    FRONT_RIGHT = 271
    BACK_LEFT = 258
    BACK_RIGHT = 272

class VL53L1X():
    def __init__(self, node: LifecycleNode, name: str, hardware_interface):
        self.sensor = hardware_interface
        self.name: Final[str] = name
        self.logger = node.get_logger()

        # Sensor physical limits
        self.min_dist = 1
        self.max_dist = 4000

        # Ring buffer for 10 last measurements
        self.history = deque(maxlen=10)

    def start_ranging(self):
        self.logger.info(f"starting ranging on sensor {self.name}")
        try:
            # Set measurement profile (1 = Short range, 2 = Long range)
            self.sensor.start_ranging()
            #self.sensor.start_ranging(2)
        except Exception as e:
            self.logger.error(f"starting ranging on sensor {self.name} failed: {e}")

    def stop_ranging(self):
        self.logger.info(f"stopping ranging on sensor {self.name}")
        try:
            self.sensor.stop_ranging()
        except Exception as e:
            self.logger.error(f"stopping ranging on sensor {self.name} failed: {e}")

    def read_sensor(self):
        try:
            # # Read distance in millimeters
            distance = self.sensor.get_distance()
            if isinstance(distance, str):
                self.logger.error(f"Sensor {self.name} HW error: {distance}")
                return None

            # Filter invalid values
            if distance <= self.min_dist or distance >= self.max_dist:
                self.logger.warn(f"Invalid measurement detected: {distance} mm. Measurement dropped")
                return None

            # Add value in ring buffer
            self.history.append(distance)

            # Calculate moving average of 10 last measurements. # Round to zero decimal
            moving_average = round(sum(self.history) / len(self.history),0)

            msg = String()
            distance = {"sensor": self.name, "distance": moving_average}
            msg.data = json.dumps(distance)
            self.logger.info(f"L: {distance} mm")
            return msg

        except Exception as e:
            self.logger.error(f"Error in reading sensor: {self.name} {e}")
            return None

    def cleanup(self):
        self.sensor.cleanup()

class ProximitySensorNode(LifecycleNode):
    def __init__(self):
        super().__init__('vl53l1x_node')
        self.is_activated = False
        self.move_direction = "Forward"
        self.sensors : Dict[str, VL53L1X]
        self.move_direction_sub = None
        self.pub_range : LifecyclePublisher
        self.timer = None

    def on_configure(self, state):
        self.get_logger().info("IN on_configure state")

        # Create sensors
        hw_sensor_front_right = VL53L1XHardwareInterface('front_right', GPIO_pins.FRONT_RIGHT.value, 0x32)
        hw_sensor_front_left = VL53L1XHardwareInterface('front_left', GPIO_pins.FRONT_LEFT.value, 0x31)
        hw_sensor_back_right = VL53L1XHardwareInterface('back_right', GPIO_pins.BACK_RIGHT.value, 0x30)
        hw_sensor_back_left = VL53L1XHardwareInterface('back_left', GPIO_pins.BACK_LEFT.value, 0x29)

        self.sensors = {
            'back_left': VL53L1X(self, 'back_left', hw_sensor_back_left),
            'back_right': VL53L1X(self, 'back_right', hw_sensor_back_right),
            'front_left': VL53L1X(self, 'front_left', hw_sensor_front_left),
            'front_right': VL53L1X(self, 'front_right', hw_sensor_front_right)
        }
#        self.set_move_direction("Forward")
        self.set_move_direction("Backward")

        self.move_direction_sub = self.create_subscription(String, '/move/direction', self.set_move_direction_callback, 10)
        self.pub_range = self.create_lifecycle_publisher(String, 'range', 10) # type: ignore reportAttributeAccessIssue

        # Create timer which expires after every 0.1 second (10 Hz) and publishes sensor data
        self.timer = self.create_timer(0.1, self.read_and_publish)

        self.get_logger().info('Returning SUCCESS')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info("IN on_active state")
        self.is_activated = True
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info("IN on_deactive state")
        self.is_activated = False
        return super().on_activate(state)

    def set_move_direction_callback(self, msg: String):
        if not self.is_activated:
            return
        self.set_move_direction(msg.data)

    def set_move_direction(self, direction: str):
        self.get_logger().info(f"Setting move direction to: {direction}")
        if direction in {"Forward", "Backward"}:
            self.move_direction = direction
            if direction == "Forward":
                active_sensors = {'front_left', 'front_right'}
                inactive_sensors = {'back_left', 'back_right'}
            else:
                inactive_sensors = {'front_left', 'front_right'}
                active_sensors = {'back_left', 'back_right'}

            # Stop ranging
            for name in inactive_sensors:
                sensor = self.sensors.get(name)
                if sensor:
                    sensor.stop_ranging()

            # Start ranging
            for name in active_sensors:
                sensor = self.sensors.get(name)
                if sensor:
                    sensor.start_ranging()
        else:
            self.get_logger().error(f"invalid in move direction: {direction}")

    def read_and_publish(self):

        if self.move_direction == "Forward":
            active_sensors = {'front_left', 'front_right'}
        else:
            active_sensors = {'back_left', 'back_right'}

        for name in active_sensors:
            sensor = self.sensors.get(name)
            if sensor:
                msg = sensor.read_sensor()
                if msg is not None:
                    self.pub_range.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ProximitySensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _, sensor in node.sensors.items():
            sensor.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
