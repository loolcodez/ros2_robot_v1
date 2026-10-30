# i2c_bus = 0
# SDA 22 (TWI0_SDA) Green wire
# SCL 15 (TWI0_SCL) Yellow wire
# XSHUT front-left 29/PI0/256 Gray wire
# XSHUT front-right 31/PI15/271 Gray wire
# XSHUT back-left 35/PI2/258 Gray wire
# XSHUT back-right 37/PI16/272 Gray wire

import time
import json
from enum import Enum
from typing import Final, Dict
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# HW library imports
from periphery import GPIO
import VL53L1X as ToFSensor

class GPIO_pins(Enum):
    FRONT_LEFT = 256
    FRONT_RIGHT = 271
    BACK_LEFT = 258
    BACK_RIGHT = 272

class VL53L1X():
    def __init__(self, node: Node, name: str, gpio_pin: int, address: int = 0x29):
        self.name: Final[str] = name
        self.address: Final[int] = address
        self.logger = node.get_logger()

        # Sensor physical limits
        self.min_dist = 1
        self.max_dist = 4000

        # Ring buffer for 10 last measurements
        self.history = deque(maxlen=10)

         # Init XSHUT pins and set it OUTPUT-state
        self.logger.info(f"VL53L1X: {self.name} Initializing XSHUT pins...")
        try:
            self.xshut = GPIO(gpio_pin, "out")
            self.xshut.write(False) # Set pin low
            time.sleep(0.5)  # Wait until sensor is ready
        except Exception as e:
            self.logger.error(f"VL53L1X: {self.name} GPIO initialization failed: {e}")
            return

        # Initialize I2C-bus
        i2c_bus = 0
        self.logger.info(f"VL53L1X: {self.name} connecting VL53L1X sensor on I2C-bus {i2c_bus}...")

        try:
            # ----------------------------------------------------
            # Init back-left sensor
            # ----------------------------------------------------
            self.logger.info(f"VL53L1X: Waking up {self.name} sensor...")
            self.xshut.write(True) # Set pin high
            time.sleep(0.5)  # Wait until sensor is ready

            try:
                # Sensor wakes up using default 0x29 address
                self.tof = ToFSensor.VL53L1X(i2c_bus=i2c_bus, i2c_address=0x29)
                if address != 0x29:
                    # Switch back left sensor address to new address
                    self.tof.open()
                    self.tof.change_address(self.address)
                    self.tof.close()
                    # Open using new address
                    self.tof = ToFSensor.VL53L1X(i2c_bus=i2c_bus, i2c_address=self.address)

                self.tof.open()

                # Set timing 50ms budget and 55ms inter measurement budget
                # Reading of the sensor must be faster that the timer which triggers the reading
                self.tof.set_timing(50000, 55)
                self.logger.info(f"VL53L1X: {self.name} sensor initialized successfully on address {self.address:#x}!")

            except Exception as e:
                self.logger.error(f"VL53L1X: {self.name} sensor initialization failed: {e}")
                return
        except Exception as e:
            self.logger.error(f"VL53L1X: {self.name} sensor sequential initialization failed: {e}")
            self.cleanup()
            return

    def start_ranging(self):
        self.logger.error(f"VL53L1X: starting ranging on sensor {self.name}")
        try:
            # Set measurement profile (1 = Short range, 2 = Long range)
            self.tof.start_ranging(2)
        except Exception as e:
            self.logger.error(f"VL53L1X: starting ranging on sensor {self.name} failed: {e}")

    def stop_ranging(self):
        self.logger.error(f"VL53L1X: stopping ranging on sensor {self.name}")
        try:
            self.tof.stop_ranging()
        except Exception as e:
            self.logger.error(f"VL53L1X: stopping ranging on sensor {self.name} failed: {e}")

    def read_sensor(self):
        try:
            # # Read distance in millimeters
            distance = self.tof.get_distance()

            # Filter invalid values
            if distance <= self.min_dist or distance >= self.max_dist:
                self.logger.warn(f"VL53L1X: Invalid measurement detected: {distance} mm. Measurement dropped")
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
            self.logger.error(f"VL53L1X: Error in reading sensor: {self.name} {e}")
            return None

    def cleanup(self):
        try:
            self.tof.stop_ranging()
        except Exception as e:
            self.logger.error(f"Error in stopping measuring sensor: {self.name} {e}")

        if self.xshut:
            try:
                self.xshut.write(False)
                self.xshut.close()
            except Exception as e:
                self.logger.error(f"Error in closing {self.name} sensor: {e}")

class VL53L1XNode(Node):
    def __init__(self):
        super().__init__('vl53l1x_node')

        # Create sensors
        self.sensors : Dict[str, VL53L1X] = {
            'back_right': VL53L1X(self, 'back_right', GPIO_pins.BACK_LEFT.value, 0x29)
            #'back_left': VL53L1X(self, 'back_left', GPIO_pins.BACK_RIGHT.value, 0x30,),
            #'front_right': VL53L1X(self, 'front_right', GPIO_pins.FRONT_LEFT.value, 0x31),
            #'front_left': VL53L1X(self, 'front_left', GPIO_pins.FRONT_RIGHT.value, 0x32)
        }

#        self.set_move_direction("forward")
#        self.set_move_direction("backward")

        self.move_direction_sub = self.create_subscription(String, '/move/direction', self.set_move_direction_callback, 10)
        self.pub_range = self.create_publisher(String, 'range', 10)

        # Create timer which expires after every 0.1 second (10 Hz) and publishes sensor data
        self.timer = self.create_timer(0.1, self.read_and_publish)

    def set_move_direction_callback(self, msg: String):
        self. set_move_direction(msg.data)

    def set_move_direction(self, direction: str):
        self.get_logger().error(f"VL53L1XNode: Setting move direction to: {direction}")
        if direction in {"forward", "backward"}:
            self.move_direction = direction
            if direction == "forward":
                active_sensors = {'front_left', 'front_right'}
                inactive_sensors = {'back_left', 'back_right'}
            else:
                inactive_sensors = {'front_left', 'front_right'}
                active_sensors = {'back_left', 'back_right'}

            for name in inactive_sensors:
                sensor = self.sensors.get(name)
                if sensor:
                    sensor.stop_ranging()

            for name in active_sensors:
                sensor = self.sensors.get(name)
                if sensor:
                    sensor.start_ranging()
        else:
            self.get_logger().error(f"VL53L1XNode: invalid in move direction: {direction}")

    def read_and_publish(self):

        if self.move_direction == "forward":
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
    node = VL53L1XNode()
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
