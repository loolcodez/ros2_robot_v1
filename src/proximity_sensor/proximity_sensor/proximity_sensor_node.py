# i2c_bus = 0
# SDA 22 (TWI0_SDA) Green wire
# SCL 15 (TWI0_SCL) Yellow wire
# XSHUT front-left 29/PI0/256 Gray wire
# XSHUT front-right 31/PI15/271 Gray wire
# XSHUT back-left 35/PI2/258 Gray wire
# XSHUT back-right 37/PI16/272 Gray wire

# import time
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32
# from smbus2 import SMBus, i2c_msg
# from periphery import GPIO

# # VL53L1X Rekisteriosoitteet (16-bit)
# REG_MODEL_ID = 0x010F
# REG_I2C_SLAVE_DEVICE_ADDRESS = 0x0001
# REG_SYS_START = 0x0087
# REG_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = 0x0096

# class VL53L1XNode(Node):
#     def __init__(self):
#         super().__init__('proximity_sensor')

#         self.pub_back_left = self.create_publisher(Int32, 'range/back_left', 10)
#         self.pub_back_right = self.create_publisher(Int32, 'range/back_right', 10)

#         self.get_logger().info("Initializing XSHUT pins...")
#         try:
#             self.xshut_back_left = GPIO(258, "out")   # back-left (Pin 35)
#             self.xshut_back_right = GPIO(272, "out")  # back-right (Pin 37)

#             # Resetoidaan molemmat anturit (Sammutetaan)
#             self.xshut_back_left.write(False)
#             time.sleep(0.5)
#             self.xshut_back_right.write(False)
#             time.sleep(0.5)
#         except Exception as e:
#             self.get_logger().error(f"GPIO initialization failed: {e}")
#             return

#         self.i2c_bus = 0
#         self.get_logger().info(f"Connecting VL53L1X sensors on I2C-bus {self.i2c_bus}...")

#         try:
#             self.bus = SMBus(self.i2c_bus)

#             # ----------------------------------------------------
#             # 1. HERÄTETÄÄN JA ALUSTETAAN BACK-LEFT -> 0x30
#             # ----------------------------------------------------
#             self.get_logger().info("Waking up Back-Left sensor...")
#             self.xshut_back_left.write(True)
#             time.sleep(0.5)

#             # Vaihdetaan oletusosoitteesta 0x29 osoitteeseen 0x30
#             self.write_8bit_reg(0x29, REG_I2C_SLAVE_DEVICE_ADDRESS, 0x30)
#             time.sleep(0.1)

#             # Käynnistetään mittaus uudella osoitteella 0x30
#             # Kirjoitetaan 0x40 (Continuous measurement) järjestelmän käynnistysrekisteriin
#             self.write_8bit_reg(0x30, REG_SYS_START, 0x40)
#             self.get_logger().info("Back-Left initialized successfully on address 0x30!")
#             time.sleep(0.5)

#             # ----------------------------------------------------
#             # 2. HERÄTETÄÄN JA ALUSTETAAN BACK-RIGHT -> 0x29
#             # ----------------------------------------------------
#             self.get_logger().info("Waking up Back-Right sensor...")
#             self.xshut_back_right.write(True)
#             time.sleep(0.5)

#             # Käynnistetään mittaus oletusosoitteella 0x29
#             self.write_8bit_reg(0x29, REG_SYS_START, 0x40)
#             self.get_logger().info("Back-Right initialized successfully on address 0x29!")

#         except Exception as e:
#             self.get_logger().error(f"Sensor sequential initialization failed: {e}")
#             return

#         # Luetaan antureita 1 Hz tai 10 Hz tahdilla
#         self.timer = self.create_timer(0.1, self.read_and_publish)

#     def write_8bit_reg(self, device_addr, reg_addr, val):
#         """Kirjoittaa 8-bittisen arvon 16-bittiseen rekisteriosoitteeseen"""
#         write_msg = i2c_msg.write(device_addr, [reg_addr >> 8, reg_addr & 0xFF, val])
#         self.bus.i2c_rdwr(write_msg)

#     def read_16bit_val_from_reg(self, device_addr, reg_addr):
#         """Lukee 16-bittisen arvon (kuten etäisyyden) 16-bittisestä rekisteriosoitteesta"""
#         write_msg = i2c_msg.write(device_addr, [reg_addr >> 8, reg_addr & 0xFF])
#         read_msg = i2c_msg.read(device_addr, 2) # Luetaan 2 tavua (High ja Low byte)
#         self.bus.i2c_rdwr(write_msg, read_msg)

#         data = list(read_msg)
#         return (data[0] << 8) | data[1]

#     def read_and_publish(self):
#         try:
#             # Luetaan etäisyysdata suoraan rekisteristä 0x0096 molemmille osoitteille
#             dist_left = self.read_16bit_val_from_reg(0x30, REG_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0)
#             dist_right = self.read_16bit_val_from_reg(0x29, REG_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0)

#             # Julkaistaan vasen
#             msg_left = Int32()
#             msg_left.data = dist_left
#             self.pub_back_left.publish(msg_left)

#             # Julkaistaan oikea
#             msg_right = Int32()
#             msg_right.data = dist_right
#             self.pub_back_right.publish(msg_right)

#             self.get_logger().info(f"L: {dist_left} mm | R: {dist_right} mm")

#         except Exception as e:
#             self.get_logger().error(f"Error reading sensors: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = VL53L1XNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if hasattr(node, 'bus'):
#             node.bus.close()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


## =======================================

import time
import json
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# HW library imports
from periphery import GPIO
import VL53L1X as ToFSensor

class VL53L1XNode(Node):
    def __init__(self):
        super().__init__('vl53l1x_node')

        # Sensor physical limits
        self.min_dist = 40
        self.max_dist = 4000
        # Ring buffer for 10 last measurements
        self.history_back_left = deque(maxlen=10)

        # self.pub_back_left = self.create_publisher(Int32, 'range/back_left', 10)
        # self.pub_back_right = self.create_publisher(Int32, 'range/back_right', 10)
        self.pub_range = self.create_publisher(String, 'range', 10)

         # Init XSHUT pins and set it OUTPUT-state
         # Both must be LOW (SHUT) (False) in the beginning
        self.get_logger().info("Initializing XSHUT pins...")
        try:
            self.xshut_back_left = GPIO(258, "out")   # back-left (Pin 35)
            self.xshut_back_right = GPIO(272, "out")  # back-right (Pin 37)
            self.xshut_back_left.write(False) # Set pin low
            self.xshut_back_right.write(False) # Set pin low
            time.sleep(0.5)  # Wait until sensor is ready
        except Exception as e:
            self.get_logger().error(f"GPIO initialization failed: {e}")
            return

        # Initialize I2C-bus
        i2c_bus = 0
        self.get_logger().info(f"Connecting VL53L1X sensor on I2C-bus {i2c_bus}...")

        try:
            # ----------------------------------------------------
            # Init back-left sensor
            # ----------------------------------------------------
            self.get_logger().info("Waking up Back-Left sensor...")
            self.xshut_back_left.write(True) # Set pin high
            time.sleep(0.5)  # Wait until sensor is ready

            try:
                # Back-right sensor wakes up using default 0x29 address
                self.tof_back_left = ToFSensor.VL53L1X(i2c_bus=i2c_bus, i2c_address=0x29)
                self.tof_back_left.open()
            except Exception as e:
                self.get_logger().error(f"Back-Left initialization failed: {e}")
                return

            # Switch back left sensor address to 0x30
            # Huom: python-vl53l1x -kirjastossa metodi on yleensä nimeltään 'change_address'
            self.tof_back_left.change_address(0x30)
            self.tof_back_left.close()

            # Open using new address
            self.tof_back_left = ToFSensor.VL53L1X(i2c_bus=i2c_bus, i2c_address=0x30)
            self.tof_back_left.open()

            # Set timing 50ms budget and 55ms inter measurement budget
            # Reading of the sensor must be faster that the timer which triggers the reading
            self.tof_back_left.set_timing(50000, 55)

            # Set measurement profile (1 = Short range, 2 = Long range)
            self.tof_back_left.start_ranging(2)
            self.get_logger().info("Back-Left initialized successfully on address 0x30!")

            # ----------------------------------------------------
            # Init back-right sensor
            # ----------------------------------------------------
            # self.get_logger().info("Waking up Back-Right sensor...")
            # self.xshut_back_right.write(True) # Set pin high
            # time.sleep(0.5)  # Wait until sensor is ready

            # # Back-left sensor wakes up using default 0x29 address
            # self.tof_back_right = ToFSensor.VL53L1X(i2c_bus=i2c_bus, i2c_address=0x29)
            # # # Set measurement profile (1 = Short range, 2 = Long range)
            # self.tof_back_right.open()
            # # Set measurement profile (1 = Short range, 2 = Long range)
            # self.tof_back_right.start_ranging(2)
            # self.get_logger().info("Back-Right initialized successfully on address 0x29!")
        except Exception as e:
            self.get_logger().error(f"Sensor sequential initialization failed: {e}")
            #self.cleanup()
            #return

        # Create timer which expires after every 0.1 second (10 Hz) and publishes sensor data
#        self.timer = self.create_timer(0.1, self.read_and_publish)
        self.timer = self.create_timer(0.1, self.read_and_publish_left)
#        self.timer = self.create_timer(1, self.read_and_publish_right)

    def read_and_publish_left(self):
        try:
            # # Read distance in millimeters
            distance = self.tof_back_left.get_distance()

            # Filter invalid values
            if distance <= self.min_dist or distance >= self.max_dist:
                self.get_logger().warn(f"Invalid measurement detected: {distance} mm. Measurement dropped")
                return

            # Add value in ring buffer
            self.history_back_left.append(distance)

            # Calculate moving average of 10 last measurements
            moving_average = sum(self.history_back_left) / sum(self.history_back_left) / len(self.history_back_left)

            msg_back_left = String()
            distance_back_left = {"sensor": "back_left", "distance": moving_average}
            msg_back_left.data = json.dumps(distance_back_left)
            self.pub_range.publish(msg_back_left.data)

            self.get_logger().info(f"L: {distance_back_left} mm")
        except Exception as e:
            self.get_logger().error(f"Error reading sensors: {e}")

    # def read_and_publish_right(self):
    #     try:
    #         # Read distance in millimeters
    #         distance_back_right = self.tof_back_right.get_distance()

    #         # Publish results
    #         msg_back_right = Int32()
    #         msg_back_right.data = distance_back_right
    #         self.pub_back_right.publish(msg_back_right)

    #         self.get_logger().info(f"R: {distance_back_right} mm")

    #     except Exception as e:
    #         self.get_logger().error(f"Error reading sensors: {e}")

    # def read_and_publish(self):
    #     try:
    #         # Read distance in millimeters
    #         distance_back_left = self.tof_back_left.get_distance()
    #         distance_back_right = self.tof_back_right.get_distance()

    #         # Publish results
    #         msg_back_left = Int32()
    #         msg_back_left.data = distance_back_left
    #         self.pub_back_left.publish(msg_back_left)

    #         msg_back_right = Int32()
    #         msg_back_right.data = distance_back_right
    #         self.pub_back_right.publish(msg_back_right)

    #         self.get_logger().info(f"L: {distance_back_left} mm | R: {distance_back_right} mm")

    #     except Exception as e:
    #         self.get_logger().error(f"Error reading sensors: {e}")


    # def read_and_publish(self):
    #     try:
    #         # Read distance in millimeters
    #         distance_mm = self.tof.get_distance()

    #         if distance_mm > 0:
    #             msg = Int32()
    #             msg.data = distance_mm
    #             self.publisher_.publish(msg)
    #             self.get_logger().info(f"Distance: {distance_mm} mm")
    #         else:
    #             self.get_logger().warn("Error in sensor measurement or target too far")

    #     except Exception as e:
    #         self.get_logger().error(f"Error in sensor reading: {e}")


    def cleanup(self):
        try:
            if hasattr(self, 'tof_left'):
                self.tof_back_left.stop_ranging()
            if hasattr(self, 'tof_right'):
                self.tof_back_right.stop_ranging()
        except Exception as e:
            self.get_logger().info(f"Error in stopping measuring sensors: {e}")

        if self.xshut_back_left:
            try:
                self.xshut_back_left.write(False)
                self.xshut_back_left.close()
            except Exception as e:
                self.get_logger().info(f"Error in closing back_left sensor: {e}")

        if self.xshut_back_right:
            try:
                self.xshut_back_right.write(False)
                self.xshut_back_right.close()
            except Exception as e:
                self.get_logger().info(f"Error in closing back_right sensor: {e}")
        super().destroy_node()

    # def destroy_node(self):
    #     # Set XSHUT low to close the sensor
    #     try:
    #         self.tof.stop_ranging()
    #     except:
    #         pass

    #     if self.xshut:
    #         try:
    #             self.xshut.write(False)
    #             self.xshut.close()
    #         except:
    #             pass
    #     super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VL53L1XNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

## =======================================

# # Asensin tällä
# # sudo pip install smbus2 vl53l1x python-periphery --break-system-packages
# # sudo pip install smbus2 vl53l1x python-periphery --target=/usr/lib/python3/dist-packages --break-system-packages


# # sudo groupadd -f gpio
# # sudo usermod -aG gpio,i2c orangepi
# # sudo nano /etc/udev/rules.d/99-gpio.rules
# # https://github.com/Joshua-Riek/ubuntu-rockchip/issues/222


# # su - orangepi

# #!/usr/bin/env python3
# #import sys
# #sys.path.insert(0, '/usr/local/lib/python3.12/dist-packages')

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32
# import time

# # HW library imports
# from periphery import GPIO
# #from wiringpi import GPIO
# import VL53L1X as ToFSensor

# class VL53L1XTestNode(Node):
#     def __init__(self):
#         super().__init__('vl53l1x_test_node')

#         self.publisher_ = self.create_publisher(Int32, 'range_data', 10)

#         # Define XSHUT pin (GPIO 256) and set it OUTPUT-state
#         # Set XSHUT High to start sensor
#         self.get_logger().info("Initializing XSHUT pin (GPIO 256)...")
#         try:
#             self.xshut = GPIO(256, "out")
#             self.xshut.write(True) # Set pin high -> Sensor wakes up
#             time.sleep(0.2)        # Wait until sensor is ready
#         except Exception as e:
#             self.get_logger().error(f"GPIO initialization failed: {e}")
#             return

#         # Initialize I2C-bus (Default bus is 1)
#         i2c_bus = 1
#         self.get_logger().info(f"Connecting VL53L1X sensor on I2C-bus {i2c_bus}...")

#         try:
#             # Sensors default address is 0c29
#             self.tof = ToFSensor.VL53L1X(i2c_bus=i2c_bus, i2c_address=0x29)
#             self.tof.open()

#             # Set measurement profile (1 = Short range, 2 = Long range)
#             self.tof.start_ranging(2)
#             self.get_logger().info("VL53L1X initialized successfully!")
#         except Exception as e:
#             self.get_logger().error(f"Sensor initialization failed: {e}")
#             return

#         # Create timer which expires after every 0.1 second and publishes sensor data
#         self.timer = self.create_timer(0.1, self.read_and_publish)

#     def read_and_publish(self):
#         try:
#             # Read distance in millimeters
#             distance_mm = self.tof.get_distance()

#             if distance_mm > 0:
#                 msg = Int32()
#                 msg.data = distance_mm
#                 self.publisher_.publish(msg)
#                 self.get_logger().info(f"Distance: {distance_mm} mm")
#             else:
#                 self.get_logger().warn("Error in sensor measurement or target too far")

#         except Exception as e:
#             self.get_logger().error(f"Error in sensor reading: {e}")

#     def destroy_node(self):
#         # Set XSHUT low to close the sensor
#         try:
#             self.tof.stop_ranging()
#             self.xshut.write(False)
#             self.xshut.close()
#         except:
#             pass
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = VL53L1XTestNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
