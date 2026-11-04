# i2c_bus = 0
# SDA 22 (TWI0_SDA) Green wire
# SCL 15 (TWI0_SCL) Yellow wire
# XSHUT front-left 29/PI0/256 Gray wire
# XSHUT front-right 31/PI15/271 Gray wire
# XSHUT back-left 35/PI2/258 Gray wire
# XSHUT back-right 37/PI16/272 Gray wire

import time
import math
from enum import Enum
from typing import Final, Dict
import rclpy
#from rclpy.node import Node # timer
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecyclePublisher
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String, Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from smbus2 import SMBus
from proximity_sensor.hardware_interface import VL53L5CXHardwareInterface

class GPIO_pins(Enum):
    FRONT_LEFT = 256
    FRONT_RIGHT = 271
    BACK_LEFT = 258
    BACK_RIGHT = 272

class VL53L5CX():
    def __init__(self, node: LifecycleNode, name: str, hardware_interface):
        self.node = node
        self.sensor = hardware_interface
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
            start = time.perf_counter()
            self.get_data_count = self.get_data_count + 1

            # Read distance in millimeters
            distances, status_info, status = self.sensor.get_distances_and_statuses()
            if status == "NO_NEW_DATA":
                return None, None, status

            back_elapsed = time.perf_counter()
            self.data_received_count = self.data_received_count + 1

            self.logger.info(
                f"Back {self.name}: {(back_elapsed - start)*1000:.1f} ms, get_data_count: {self.get_data_count}, data_received_count: {self.data_received_count}",
                throttle_duration_sec=1.0)

            if isinstance(distances, str):
                self.logger.error(f"Sensor {self.name} HW error: {status}")
                return None, None, status

            # self.logger.info(f"Sensor: {self.name}, distances: {distances}",
            #                  throttle_duration_sec=1.0) # Print in log once per second
            # self.logger.info(f"Sensor: {self.name}, statuses: {status_info}",
            #                  throttle_duration_sec=1.0) # Print in log once per second
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
        self.smbus0 = SMBus(0)
        self.smbus3 = SMBus(3)
        self.get_logger().info("I2C SMBus-väylät avattu onnistuneesti __init__-vaiheessa.")


        # Create sensors
        #hw_sensor_front_left = VL53L5CXHardwareInterface('front_left', GPIO_pins.FRONT_LEFT.value)
        hw_sensor_front_right = VL53L5CXHardwareInterface('front_right', GPIO_pins.FRONT_RIGHT.value)
        #hw_sensor_back_right = VL53L5CXHardwareInterface('back_right', GPIO_pins.BACK_RIGHT.value)
        hw_sensor_back_left = VL53L5CXHardwareInterface('back_left', GPIO_pins.BACK_LEFT.value)
        self.get_logger().info('Step3')

        try:
            # status = hw_sensor_front_left.initialize_sensor(self.smbus3, 0x30)
            # self.get_logger().info(f"hw_sensor_front_left: {status}")
            status = hw_sensor_front_right.initialize_sensor(self.smbus3, 0x29)
            self.get_logger().info(f"hw_sensor_front_right: {status}")
#            status = hw_sensor_back_right.initialize_sensor(self.smbus0, 0x29)
            #status = hw_sensor_back_right.initialize_sensor(self.smbus0, 0x30)
            #self.get_logger().info(f"hw_sensor_back_right: {status}")
            status = hw_sensor_back_left.initialize_sensor(self.smbus0, 0x29)
            self.get_logger().info(f"hw_sensor_back_left: {status}")
        except Exception as e:
            return f"Error in initializing sensor: {e}"

        self.get_logger().info('Step4')

        self.sensors = {
            #'front_left': VL53L5CX(self, 'front_left', hw_sensor_front_left),
            'front_right': VL53L5CX(self, 'front_right', hw_sensor_front_right),
            #'back_right': VL53L5CX(self, 'back_right', hw_sensor_back_right)
            'back_left': VL53L5CX(self, 'back_left', hw_sensor_back_left),
        }

        self.get_logger().info("ProximitySensorNode created")

    def on_configure(self, state):
        self.get_logger().info("IN on_configure state")

        # Define bus for I2C communication. Defined at /boot/orangepiEnv.txt (overlays=pi-i2c0 pi-i2c2)
        # smbus0 = SMBus(0) # sudo i2cdetect -y 0
        # smbus3 = SMBus(3) # sudo i2cdetect -y 3
        # self.get_logger().info('Step2')

        # # Create sensors
        # #hw_sensor_front_left = VL53L5CXHardwareInterface('front_left', GPIO_pins.FRONT_LEFT.value)
        # #hw_sensor_front_right = VL53L5CXHardwareInterface('front_right', GPIO_pins.FRONT_RIGHT.value)
        # hw_sensor_back_right = VL53L5CXHardwareInterface('back_right', GPIO_pins.BACK_RIGHT.value)
        # hw_sensor_back_left = VL53L5CXHardwareInterface('back_left', GPIO_pins.BACK_LEFT.value)
        # self.get_logger().info('Step3')

        # try:
        #     # status = hw_sensor_front_left.initialize_sensor(smbus3, 0x30)
        #     # self.get_logger().info(f"hw_sensor_front_left: {status}")
        #     # status = hw_sensor_front_right.initialize_sensor(smbus3, 0x29)
        #     # self.get_logger().info(f"hw_sensor_front_right: {status}")
        #     status = hw_sensor_back_right.initialize_sensor(self.smbus0, 0x29)
        #     #status = hw_sensor_back_right.initialize_sensor(smbus0, 0x30)
        #     self.get_logger().info(f"hw_sensor_back_right: {status}")
        #     status = hw_sensor_back_left.initialize_sensor(self.smbus3, 0x29)
        #     self.get_logger().info(f"hw_sensor_back_left: {status}")
        # except Exception as e:
        #     return f"Error in initializing sensor: {e}"
        # self.get_logger().info('Step4')

        # self.sensors = {
        #     #'front_left': VL53L5CX(self, 'front_left', hw_sensor_front_left),
        #     #'front_right': VL53L5CX(self, 'front_right', hw_sensor_front_right),
        #     'back_left': VL53L5CX(self, 'back_left', hw_sensor_back_left),
        #     'back_right': VL53L5CX(self, 'back_right', hw_sensor_back_right)
        # }

#        self.set_move_direction("backward")
#        self.set_move_direction("forward")

#        self.move_direction_sub = self.create_subscription(String, '/move/direction', self.set_move_direction_callback, 10)

        self.get_logger().info('Returning SUCCESS')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info("IN on_active state")
        self.is_activated = True
        for _, sensor in self.sensors.items():
            sensor.activate()

        # Create timer which expires after every ~0.066 second (15 Hz) and publishes sensor data
#        self.timer = self.create_timer(0.066, self.read_and_publish)
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info("IN on_deactive state")
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

    # def set_move_direction_callback(self, msg: String):
    #     if not self.is_activated:
    #         return
#        self.set_move_direction(msg.data)

    # def set_move_direction(self, direction: str):
    #     self.get_logger().info(f"Setting move direction to: {direction}")
    #     self.active_sensors = {'front_left', 'front_right','back_left', 'back_right'}
    #     for name in self.active_sensors:
    #         sensor = self.sensors.get(name)
    #         if sensor:
    #             sensor.start_ranging()


        # if direction in {"stop", "forward", "backward"}:
        #     self.move_direction = direction
        #     inactive_sensors = {}
        #     if direction == "forward":
        #         self.active_sensors = {'front_left', 'front_right'}
        #         inactive_sensors = {'back_left', 'back_right'}
        #     elif direction == "backward":
        #         inactive_sensors = {'front_left', 'front_right'}
        #         self.active_sensors = {'back_left', 'back_right'}
        #     elif direction == "stop":
        #         for name in self.active_sensors:
        #             sensor = self.sensors.get(name)
        #             if sensor:
        #                 sensor.stop_ranging()
        #         self.active_sensors = {}
        #         return

        # #     # Stop ranging
        #     for name in inactive_sensors:
        #         sensor = self.sensors.get(name)
        #         if sensor:
        #             sensor.stop_ranging()

        #     # Start ranging
        #     for name in self.active_sensors:
        #         sensor = self.sensors.get(name)
        #         if sensor:
        #             sensor.start_ranging()
        # else:
        #    self.get_logger().error(f"invalid in move direction: {direction}")

#    def read_and_publish(self):
        # if self.move_direction == "forward":
        #     active_sensors = {'front_left', 'front_right'}
        # else:
        #     active_sensors = {'back_left', 'back_right'}

        # for name in active_sensors:
        #     sensor = self.sensors.get(name)
        #     if sensor:
        #         sensor.read_and_publish()

#        start = time.perf_counter()

        # self.sensors['front_left'].read_and_publish()
        # self.sensors['front_right'].read_and_publish()

        # front_elapsed = time.perf_counter()

 #       self.sensors['back_left'].read_and_publish()
 #       self.sensors['back_right'].read_and_publish()

#        back_elapsed = time.perf_counter()

 #       self.get_logger().info(
            #f"Front: {(front_elapsed - start)*1000:.1f} ms, "
            #f"Back: {(back_elapsed - front_elapsed)*1000:.1f} ms",
#            f"Back: {(back_elapsed - start)*1000:.1f} ms",
#            throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = ProximitySensorNode()

#    executor = MultiThreadedExecutor(num_threads=4)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Puhdistetaan sensorit
        if hasattr(node, 'sensors'):
            for _, sensor in node.sensors.items():
                sensor.cleanup()
        node.destroy_node()
        rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     node = ProximitySensorNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         for _, sensor in node.sensors.items():
#             sensor.cleanup()
#         node.destroy_node()
#         rclpy.shutdown()

if __name__ == '__main__':
    main()
