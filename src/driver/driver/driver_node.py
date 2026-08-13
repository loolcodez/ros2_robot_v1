import time
import math
from typing import List
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Standard message for robot movement
from std_msgs.msg import Float32, Int32, String
from std_msgs.msg import Int32MultiArray
#from odometer import ODOMeter

class DriverNode(Node):
    def __init__(self):
        super().__init__('driver_node')
        self.shutdown = False
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        #self.declare_parameter('publish_rate_hz', 10.0)
        #self.declare_parameter('command_timeout_sec', 1.0)
        self.declare_parameter('invert_motor_1', False) # left front
        self.declare_parameter('invert_motor_2', True) # right front
        self.declare_parameter('invert_motor_3', True) # left_rear
        self.declare_parameter('invert_motor_4', False) # right rear
        # Set speed correction value. Value range received from Joystick is -1 .. 1
        self.declare_parameter('max_linear', 60.0) # speed. Must be in the range `[-100, 100]`
        self.declare_parameter('max_angular', 60.0) # turning. Must be in the range `[-100, 100]`
        self.counts_per_revolution = 1320
        self.wheel_diameter = 0.065

        self.encoding_time_ms = 20 #ms
        self.previous_encoders: list[float] = [0.0] * 4
        self.target_motor_speeds: list[float] = [0.0] * 4
        self.previous_target_motor_speeds: list[float] = [0.0] * 4

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        #self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        #self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)

        self.invert = [
            bool(self.get_parameter('invert_motor_1').value), # left front
            bool(self.get_parameter('invert_motor_2').value), # right front
            bool(self.get_parameter('invert_motor_3').value), # left_rear
            bool(self.get_parameter('invert_motor_4').value), # right rear
        ]

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.driver = None
        #self.odometer = ODOMeter()

        message = Twist()
        float(message.linear.x)
        float(message.angular.z)

        self.manual_control_sub = self.create_subscription(message, 'cmd_vel_manual', self.handle_speed_control_command, 10)
        self.battery_voltage_pub = self.create_publisher(Float32, '/power/battery_voltage', 10)
        self.power_alert_sub = self.create_subscription(Int32, '/power/alert', self.beep, 10)
        self.power_shutdown_sub = self.create_subscription(String, '/power/shutdown', self.destroy_node, 10)
        #self.encoder_pub = self.create_publisher(Int32MultiArray, '/wheel_encoders', 10)

        if not self.initialize_driver():
            self.get_logger().error('Failed to initialize Rosmaster driver')
            raise RuntimeError('Rosmaster init failed')

        # Read motor encoders
        period = 1 / self.encoding_time_ms
        self.encoders_reader_timer = self.create_timer(period, self.handle_encoders)

        # Publish initial  battery status
        self.publish_battery()
        # Set timer which publishes battery status periodically
        period = 10.0 # 10s
        self.battery_publish_timer = self.create_timer(period, self.publish_battery)

        self.get_logger().info('DriveNode created')

    def initialize_driver(self) -> bool:
        """
        Adjust the import below if your Rosmaster library uses a different module path.
        """

        try:
            # Example import style. Change this if your local library differs.
            from Rosmaster_Lib import Rosmaster  # type: ignore
        except Exception as exc:
            self.get_logger().error(f'Driver: Failed to import Rosmaster library: {exc}')
            self.get_logger().error('Driver: Install or copy the vendor Rosmaster library so Python can import it.')
            return False

        try:
            # Some vendor libraries accept com/port, some do not.
            # Start with the simplest constructor first.
            try:
                self.driver = Rosmaster(com=self.port, debug=False)
            except TypeError:
                self.driver = Rosmaster()

            self.driver.create_receive_threading()
            self.driver.clear_auto_report_data()

            self.get_logger().info(f'Driver: Rosmaster initialized on port {self.port}')
            return True

        except Exception as exc:
            self.get_logger().error(f'Driver: Rosmaster initialization error: {exc}')
            return False

    def beep(self, beep_count):
        # Control the buzzer
        # `on_time=0` turns it off
        # `on_time=1` keeps it on continuously
        # `on_time>=10` turns it on for the given number of milliseconds
        # `on_time` must be a multiple of 10 for timed operation
        for _ in range(beep_count.data):
            on_time = 100 # 100ms
            self.driver.set_beep(on_time)
            time.sleep(1)

    def handle_speed_control_command(self, msg):
        self.get_logger().info(f"Driver: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")
        if self.shutdown is True:
            return

        # Add speed correction --> from -1 .. 1 to -60 .. 60
        msg.linear.x = msg.linear.x * self.max_linear
        msg.angular.z = -msg.angular.z * self.max_angular

        # Differential control X = forward/backward, z = turning)
        left_speed = msg.linear.x - msg.angular.z
        right_speed = msg.linear.x + msg.angular.z

        # Copy speeds for all 4 motors
        # left_front,  right_front, left_rear, right rear
        speeds = [int(left_speed), int(right_speed), int(left_speed), int(right_speed)]

        # Invert other side motor direction
        for i in range(4):
            if self.invert[i]:
                speeds[i] = -speeds[i]

        # Limit values to maximum values. # Each `speed_X` must be in the range `[-100, 100]`
        self.target_motor_speeds = [self.clamp_motor_value(v) for v in speeds]

 #       if self.target_motor_speeds != self.previous_target_motor_speeds:
            #reset_pid_if_needed()
            #pass

#        self.previous_target_motor_speeds = self.target_motor_speeds

    def handle_encoders(self):
        current_encoders = self.read_wheel_encoders()
        delta_encoders = [curr - prev for curr, prev in zip(current_encoders, self.previous_encoders)]

        self.previous_encoders = current_encoders
        current_motor_speeds = self.calculate_wheel_speeds(delta_encoders, self.encoding_time_ms)

        motors = [0, 1, 2, 3]
        corrected_motor_speeds: list[float] = [
            self.target_motor_speeds[motor] + self.pid_control(self.target_motor_speeds[motor], current_motor_speeds[motor])
            for motor in motors
        ]
        self.get_logger().info(f"Driver: corrected_motor_speeds: {
            corrected_motor_speeds[0], corrected_motor_speeds[1],
            corrected_motor_speeds[2], corrected_motor_speeds[3]}")

        # Limit values to maximum values. # Each `speed_X` must be in the range `[-100, 100]`
        corrected_motor_speeds = [self.clamp_motor_value(v) for v in corrected_motor_speeds]
        self.set_motor_speeds(corrected_motor_speeds)

    def calculate_wheel_speeds(self, delta_encoders, delta_time):
        # Calculate current speed for every wheel in the list
        wheel_factor = (math.pi * self.wheel_diameter) / (self.counts_per_revolution * delta_time)
        speeds = [delta * wheel_factor for delta in delta_encoders]
        return speeds

    def read_wheel_encoders(self) -> None:
        if self.driver is None:
            self.get_logger().warning("Driver: self.driver is None")
            return

        try:
            encoders = self.driver.get_motor_encoder()
            self.get_logger().info(f"Driver: Encoders: {encoders[0], encoders[1], encoders[2], encoders[3]}")
            #self.odometer.process_encoders(encoders)
            #msg = Int32MultiArray()
            #msg.data = [int(v) for v in encoders]
            #self.encoder_pub.publish(msg)
            return encoders
        except Exception as exc:
            self.get_logger().warning(f'Driver: Failed to read motor encoders: {exc}')

    def pid_control(self, target_speed, current_speed):
        # Simple P control (add later I and D if needed):
        Kp = 0.5
        error = target_speed - current_speed
        correction = error * Kp
        return correction

    def set_motor_speeds(self, speeds: List[float]) -> bool:
        if self.driver is None:
            return False

        try:
            self.driver.set_motor(
                int(speeds[0]),
                int(speeds[1]),
                int(speeds[2]),
                int(speeds[3]),
            )
            return True
        except Exception as exc:
            self.get_logger().error(f'Driver: Failed to send motor command: {exc}')
            return False

    @staticmethod
    def clamp_motor_value(value: float) -> float:
        return max(-100.0, min(100.0, value))

    def destroy_node(self, string_msg) -> bool:
        self.get_logger().info(string_msg.data)
        try:
            self.get_logger().info('Driver: Stopping motors before shutdown')
            self.set_motor_speeds([0, 0, 0, 0])
        except Exception:
            pass
        return super().destroy_node()

    def publish_battery(self) -> None:
        if self.driver is None:
            return

        try:
            voltage = float(self.driver.get_battery_voltage())
            msg = Float32()
            msg.data = voltage
            self.battery_voltage_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warning(f'Driver: Failed to read battery voltage: {exc}')

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            shutdown_msg = String()
            shutdown_msg.data = "Startup failure"
            node.destroy_node(shutdown_msg)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
