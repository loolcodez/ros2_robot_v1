# Asensin tällä
# sudo pip install smbus2 vl53l1x python-periphery --break-system-packages
# sudo pip install smbus2 vl53l1x python-periphery --target=/usr/lib/python3/dist-packages --break-system-packages


# sudo groupadd -f gpio
# sudo usermod -aG gpio,i2c orangepi
# sudo nano /etc/udev/rules.d/99-gpio.rules
# https://github.com/Joshua-Riek/ubuntu-rockchip/issues/222


# su - orangepi

#!/usr/bin/env python3
#import sys
#sys.path.insert(0, '/usr/local/lib/python3.12/dist-packages')

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

# HW library imports
#from periphery import GPIO

from wiringpi import GPIO
import VL53L1X as ToFSensor

class VL53L1XTestNode(Node):
    def __init__(self):
        super().__init__('vl53l1x_test_node')

        self.publisher_ = self.create_publisher(Int32, 'range_data', 10)

        # Define XSHUT pin (GPIO 256) and set it OUTPUT-state
        # Set XSHUT High to start sensor
        self.get_logger().info("Initializing XSHUT pin (GPIO 256)...")
        try:
            self.xshut = GPIO(256, "out")
            self.xshut.write(True) # Set pin high -> Sensor wakes up
            time.sleep(0.2)        # Wait until sensor is ready
        except Exception as e:
            self.get_logger().error(f"GPIO initialization failed: {e}")
            return

        # Initialize I2C-bus (Default bus is 1)
        i2c_bus = 1
        self.get_logger().info(f"Connecting VL53L1X sensor on I2C-bus {i2c_bus}...")

        try:
            # Sensors default address is 0c29
            self.tof = ToFSensor.VL53L1X(i2c_bus=i2c_bus, i2c_address=0x29)
            self.tof.open()

            # Set measurement profile (1 = Short range, 2 = Long range)
            self.tof.start_ranging(2)
            self.get_logger().info("VL53L1X initialized successfully!")
        except Exception as e:
            self.get_logger().error(f"Sensor initialization failed: {e}")
            return

        # Create timer which expires after every 0.1 second and publishes sensor data
        self.timer = self.create_timer(0.1, self.read_and_publish)

    def read_and_publish(self):
        try:
            # Read distance in millimeters
            distance_mm = self.tof.get_distance()

            if distance_mm > 0:
                msg = Int32()
                msg.data = distance_mm
                self.publisher_.publish(msg)
                self.get_logger().info(f"Distance: {distance_mm} mm")
            else:
                self.get_logger().warn("Error in sensor measurement or target too far")

        except Exception as e:
            self.get_logger().error(f"Error in sensor reading: {e}")

    def destroy_node(self):
        # Set XSHUT low to close the sensor
        try:
            self.tof.stop_ranging()
            self.xshut.write(False)
            self.xshut.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VL53L1XTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
