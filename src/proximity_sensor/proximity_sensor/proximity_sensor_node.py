# i2c0 bus0
# SCL 261 (TWI0) Yellow wire, SDA 262 (TWI0) Green wire

# i2c0 bus2
# SCL 265 (TWI2) Yellow wire, SDA 266 (TWI2) Green wire

# Enable pins
# LPn (XSHUT) front-left 29/PI0/256 Gray wire
# LPn (XSHUT) front-right 31/PI15/271 Gray wire
# LPn (XSHUT) back-left 35/PI2/258 Gray wire
# LPn (XSHUT) back-right 37/PI16/272 Gray wire

import time
import math
from enum import Enum
from typing import Final, Dict
import threading
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecyclePublisher
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from smbus2 import SMBus
from proximity_sensor.hardware_interface import VL53L5CXHardwareInterface

class GPIO_LPn_pins(Enum):
    FRONT_LEFT = 256
    FRONT_RIGHT = 271
    BACK_LEFT = 258
    BACK_RIGHT = 272

class VL53L5CX():
    def __init__(self, node: LifecycleNode, name: str, hardware_interface, bus_lock: threading.Lock):
        self.node = node
        self.sensor = hardware_interface
        self.bus_lock = bus_lock
        self.name: Final[str] = name
        self.logger = node.get_logger()
        self.timer = None
        self.get_data_count = 0
        self.data_received_count = 0

        # Sensor physical limits
        self.min_dist = 1
        self.max_dist = 4000

        # Sensor's installation parameters and picture angle (FoV)
        self.fov_deg = 45.0
        self.fov_rad = math.radians(self.fov_deg)

        # Calculate angle per picture in (8x8 matrix with 7 middle spaces)
        self.angle_step = self.fov_rad / 7.0

        # Create callback group for sensor reading
        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()

        # Create timer which run read_and_publish function in own MultiThreadedExecutor
        self.timer = node.create_timer(0.1, self.read_and_publish, callback_group=self.sensor_callback_group)

        # /tof/front_left/points  sensor_msgs/msg/PointCloud2 Proximity → Nav2
        # /tof/front_right/points sensor_msgs/msg/PointCloud2 Proximity → Nav2
        # /tof/back_left/points   sensor_msgs/msg/PointCloud2 Proximity → Nav2
        # /tof/back_right/points  sensor_msgs/msg/PointCloud2 Proximity → Nav2
        self.point_cloud_pub = node.create_lifecycle_publisher(PointCloud2, f'tof/{self.name}', 10) # type: ignore reportAttributeAccessIssue

        self.logger.info(f"VL53L5CX {self.name} created")

    def activate(self):
        self.logger.info(f"starting ranging on sensor {self.name}")
        if self.timer is not None:
            try:
                self.timer = self.node.create_timer(
                    0.1,
                    self.read_and_publish,
                    callback_group=self.sensor_callback_group
                )
                self.sensor.start_ranging()
            except Exception as e:
                self.logger.error(f"starting ranging on sensor {self.name} failed: {e}")

    def deactivate(self):
        self.logger.info(f"stopping ranging on sensor {self.name}")
        if self.timer is not None:
            try:
                self.sensor.stop_ranging()
                self.timer.destroy()
                self.timer = None
                self.logger.info(f"VL53L5CX {self.name} timer deactivated.")
            except Exception as e:
                self.logger.error(f"stopping ranging on sensor {self.name} failed: {e}")

    def read_and_publish(self):

        # Fetch sensor data
        distances, status_info, status = self.read_sensor()
        if status != 'OK' or distances is None or status_info is None:
            return

        points = []
        rows, cols = 8, 8

        # Loop through 8x8 matrix
        for r in range(rows):
            for c in range(cols):
                idx = r * cols + c

                # Filtering for Nav2. Accept only valid measurements status codes (5 ja 9)
                # If status is something else skip the measurement value
                if status_info[idx] not in {5, 9}:
                    continue

                # Convert distance values to millimeters
                d_meters = distances[idx] / 1000.0

                # Skip measurement values out of valid range
                if d_meters <= 0.02 or d_meters > 4.0:
                    continue

                # Calculate place of the point related to center point of the sensor
                # r-3.5 and c-3.5 concentrates points in between -3.5 ... +3.5
                horizontal_angle = (3.5 - c) * self.angle_step
                vertical_angle = (3.5 - r) * self.angle_step

                # Project values to 3D space (ROS standard: X=forward, Y=left, Z=right)
                x = d_meters * math.cos(vertical_angle) * math.cos(horizontal_angle)
                y = d_meters * math.cos(vertical_angle) * math.sin(horizontal_angle)
                z = d_meters * math.sin(vertical_angle)

                points.append([x, y, z])

        # Create header for PointCloud2 message
        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = (
            f'tof_{self.name}_link'  # URDF/TF tree name
        )

        # Define PointCloud2 message fields (X, Y, Z float32)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Create and publish the message
        msg = point_cloud2.create_cloud(header, fields, points)
        if msg is not None:
            self.point_cloud_pub.publish(msg)

    def read_sensor(self):
        try:
            with self.bus_lock:
                start = time.perf_counter()
                self.get_data_count = self.get_data_count + 1

                # Read distance in millimeters
                distances, status_info, status = self.sensor.get_distances_and_statuses()
                if status == "NO_NEW_DATA":
                    return None, None, status

                back_elapsed = time.perf_counter()
                self.data_received_count = self.data_received_count + 1

                # Less CPU is used if this check is inside the lock with current reading settings (set_ranging_frequency_hz(15))!
                if isinstance(distances, str):
                    self.logger.error(f"Sensor {self.name} HW error: {status}")
                    return None, None, status

            self.logger.info(
                f"{self.name}: {(back_elapsed - start)*1000:.1f} ms, get_data_count: {self.get_data_count}, data_received_count: {self.data_received_count}",
                throttle_duration_sec=1.0)

            return distances, status_info, status
        except Exception as e:
            self.logger.error(f"Error in reading sensor: {self.name} {e}")
            return None, None, f"ERROR: {e}"

    def cleanup(self):
        self.sensor.cleanup()

class ProximitySensorNode(LifecycleNode):
    def __init__(self):
        super().__init__('proximity_sensor_node')
        self.is_activated = False
        self.move_direction = "stop"
        self.sensors : Dict[str, VL53L5CX]
        self.active_sensors = {}
        self.move_direction_sub = None
        self.pub_range : LifecyclePublisher
        self.smbus = None
        self.timer = None
        self.smbus0 = SMBus(0) # i2c0 / TWI0
        self.smbus3 = SMBus(3) # i2c2 / TWI2

        lock_bus0 = threading.Lock()
        lock_bus3 = threading.Lock()

        # Create HW sensor objects
        try:
            hw_sensor_front_left = VL53L5CXHardwareInterface('front_left', GPIO_LPn_pins.FRONT_LEFT.value) # i2c2
            hw_sensor_front_right = VL53L5CXHardwareInterface('front_right', GPIO_LPn_pins.FRONT_RIGHT.value) # i2c2
            hw_sensor_back_right = VL53L5CXHardwareInterface('back_right', GPIO_LPn_pins.BACK_RIGHT.value)  # i2c0
            hw_sensor_back_left = VL53L5CXHardwareInterface('back_left', GPIO_LPn_pins.BACK_LEFT.value) # i2c0
        except Exception as e:
             self.get_logger().error(f"Error in HW interface object creation: {e}")

        # Initialize sensors
        try:
            status = hw_sensor_front_left.initialize_sensor(self.smbus3, 0x30) # i2c2
            self.get_logger().info(f"hw_sensor_front_left: {status}")
            status = hw_sensor_front_right.initialize_sensor(self.smbus3, 0x29) # i2c2
            self.get_logger().info(f"hw_sensor_front_right: {status}")
            status = hw_sensor_back_right.initialize_sensor(self.smbus0, 0x30) # i2c0
            self.get_logger().info(f"hw_sensor_back_right: {status}")
            status = hw_sensor_back_left.initialize_sensor(self.smbus0, 0x29) # i2c0
            self.get_logger().info(f"hw_sensor_back_left: {status}")
        except Exception as e:
            self.get_logger().error(f"Error in initializing sensor: {e}")

        # Create sensor objects
        try:
            self.sensors = {
                'front_left': VL53L5CX(self, 'front_left', hw_sensor_front_left, lock_bus3), # i2c2
                'front_right': VL53L5CX(self, 'front_right', hw_sensor_front_right, lock_bus3), # i2c2
                'back_right': VL53L5CX(self, 'back_right', hw_sensor_back_right, lock_bus0), # i2c0
                'back_left': VL53L5CX(self, 'back_left', hw_sensor_back_left, lock_bus0), # i2c0
            }
        except Exception as e:
            self.get_logger().error(f"Error in VL53L5CX object creation: {e}")

        self.get_logger().info("ProximitySensorNode created")

    def on_configure(self, state):
        self.get_logger().info("IN on_configure state")
        self.get_logger().info('Returning SUCCESS')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info("IN on_active state")
        self.is_activated = True
        for _, sensor in self.sensors.items():
            sensor.activate()

        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info("IN on_deactivate state")
        self.is_activated = False
        for _, sensor in self.sensors.items():
            sensor.deactivate()
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.get_logger().info("IN on_cleanup state")
        for _, sensor in self.sensors.items():
            sensor.cleanup()
        self.sensors.clear()
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    node = ProximitySensorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'sensors'):
            for _, sensor in node.sensors.items():
                sensor.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
