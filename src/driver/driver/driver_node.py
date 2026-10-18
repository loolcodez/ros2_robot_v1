import time
import math
from typing import List
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Standard message type for robot movement
from sensor_msgs.msg import Imu   # Standard message type for IMU data
from std_msgs.msg import Float32, String, Int32MultiArray

class DriverNode(Node):
    def __init__(self):
        super().__init__('driver_node')
        self.shutdown = False
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('invert_motor_1', False) # left front
        self.declare_parameter('invert_motor_2', True) # right front
        self.declare_parameter('invert_motor_3', True) # left_rear
        self.declare_parameter('invert_motor_4', False) # right rear
        # Set speed correction value. Value range received from Joystick is -1 .. 1
        self.declare_parameter('max_linear', 60.0) # Speed. Must be in the range `[-100, 100]`
        self.declare_parameter('max_angular', 60.0) # Turning. Must be in the range `[-100, 100]`
        self.counts_per_revolution = 1320
        self.wheel_diameter = 0.065

        self.encoding_time_ms = 20 #ms
        self.previous_encoders: list[float] = [0.0] * 4
        self.target_motor_speeds: list[float] = [0.0] * 4
        self.previous_target_motor_speeds: list[float] = [0.0] * 4

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.invert = [
            bool(self.get_parameter('invert_motor_1').get_parameter_value().bool_value), # left front
            bool(self.get_parameter('invert_motor_2').get_parameter_value().bool_value), # right front
            bool(self.get_parameter('invert_motor_3').get_parameter_value().bool_value), # left_rear
            bool(self.get_parameter('invert_motor_4').get_parameter_value().bool_value), # right rear
        ]
        self.max_linear = float(self.get_parameter('max_linear').get_parameter_value().double_value)
        self.max_angular = float(self.get_parameter('max_angular').get_parameter_value().double_value)

        self.driver = None


        # Ohjaustila: 'joystick_pid' (enkooderikorjaus käytössä) tai 'direct' (itseajava/suora)
        self.control_mode = 'joystick_pid' 

        # PID-säätimen muuttujat kullekin 4 moottorille
        self.pid_kp = 0.5
        self.pid_ki = 0.1
        self.pid_kd = 0.05
        self.integral_error = [0.0] * 4
        self.previous_error = [0.0] * 4

        # Suodatusta varten
        self.target_motor_speeds = [0.0] * 4      # Mihin suuntaan joystick pyytää
        self.filtered_motor_speeds = [0.0] * 4    # Suodatettu/tasattu tavoitenopeus
        self.filter_alpha = 0.3                   # Mitä pienempi, sitä tasaisempi kiihtyvyys (0.0 - 1.0)

        # Enkooderipohjainen nopeuden seuranta
        self.previous_encoders = [0.0] * 4


        # message = Twist()
        # float(message.linear.x)
        # float(message.angular.z)

        # ROS2 communication
#        self.manual_control_sub = self.create_subscription(message, 'cmd_vel_out', self.handle_speed_control_command, 10)
        self.manual_control_sub = self.create_subscription(Twist, 'cmd_vel_out', self.handle_speed_control_command, 10)
        self.battery_voltage_pub = self.create_publisher(Float32, '/power/battery_voltage', 10)
        self.power_alert_sub = self.create_subscription(String, '/power/alert', self.beep, 10)
        self.power_shutdown_sub = self.create_subscription(String, '/power/shutdown', self.shutdown_callback, 10)

        # Sensor publishers
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/wheel_encoders', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)

        if not self.initialize_driver():
            self.get_logger().error('Failed to initialize Rosmaster driver')
            raise RuntimeError('Rosmaster init failed')

        # Read motor encoders
        period = 1 / self.encoding_time_ms
        #self.encoders_reader_timer = self.create_timer(period, self.handle_encoders)
        self.sensor_loop_timer = self.create_timer(0.05, self.handle_sensors_and_control)

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

    def beep(self, msg):
        beep_count = 0
        if msg.data == "Low":
            beep_count = 1
        if msg.data == "Very Low":
            beep_count = 5

        # Control the buzzer
        # `on_time=0` turns it off
        # `on_time=1` keeps it on continuously
        # `on_time>=10` turns it on for the given number of milliseconds
        # `on_time` must be a multiple of 10 for timed operation
        for _ in range(beep_count):
            on_time = 100 # 100ms
            self.driver.set_beep(on_time)
            time.sleep(1)

    def handle_speed_control_command(self, msg):
        self.get_logger().info(f"Driver: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")
        if self.shutdown is True:
            return

        # Differential control X = forward/backward, z = turning)
        left_speed = msg.linear.x - msg.angular.z
        right_speed = msg.linear.x + msg.angular.z

        # Copy speeds for all 4 motors
        # left_front, right_front, left_rear, right rear
        speeds = [int(left_speed), int(right_speed), int(left_speed), int(right_speed)]

        # Invert other side motor direction
        for i in range(4):
            if self.invert[i]:
                speeds[i] = -speeds[i]

        # Store new speed request
        # Limit values to maximum values. # Each `speed_X` must be in the range `[-100, 100]`
        self.target_motor_speeds = [self.clamp_motor_value(v) for v in speeds]

    def handle_sensors_and_control(self):
        """Pyörii itsenäisesti 20 Hz taajuudella. Hoitaa suodatuksen, PID-korjauksen ja anturit."""
        if self.driver is None or self.shutdown:
            return

        try:
            # --- 1. ANTURIDATAN LUKU JA JULKAISU ---
            current_encoders = self.driver.get_motor_encoder()

            enc_msg = Int32MultiArray()
            enc_msg.data = [int(v) for v in current_encoders]
            self.encoder_pub.publish(enc_msg)

            # (IMU-luenta ja julkaisu tähän väliin...)

            # --- 2. OHJAUSKOMENTOJEN SUODATUS (Low-pass filter) ---
            # Tasataan joystickin 100ms välein tulevia hyppäyksiä 20Hz taajuudella
            for i in range(4):
                self.filtered_motor_speeds[i] = (self.filter_alpha * self.target_motor_speeds[i] + 
                                                 (1.0 - self.filter_alpha) * self.filtered_motor_speeds[i])

            # --- 3. OHJAUKSEN KORJAUS (TILA-KOHTAINEN) ---
            if self.control_mode == 'joystick_pid':
                # Lasketaan todelliset nopeudet pulssien muutoksesta
                delta_encoders = [curr - prev for curr, prev in zip(current_encoders, self.previous_encoders)]
                current_speeds = self.calculate_wheel_speeds(delta_encoders, 0.05)

                final_commands = []
                for i in range(4):
                    # Verrataan suodatettua tavoitetta ja todellista mitattua nopeutta
                    error = self.filtered_motor_speeds[i] - current_speeds[i]

                    # PID-matematiikka ilman ulkoisia paketteja
                    self.integral_error[i] += error * 0.05
                    # Rajoitetaan integraalin karkaamista (Anti-windup)
                    self.integral_error[i] = max(-20.0, min(20.0, self.integral_error[i]))

                    derivative = (error - self.previous_error[i]) / 0.05
                    self.previous_error[i] = error

                    pid_output = (self.pid_kp * error) + (self.pid_ki * self.integral_error[i]) + (self.pid_kd * derivative)

                    # Lopullinen moottorikomento = Tavoite + PID-korjaus
                    final_cmd = self.filtered_motor_speeds[i] + pid_output
                    final_commands.append(self.clamp_motor_value(int(final_cmd)))

                self.driver.set_motor(*final_commands)

            else:
                # 'direct'-tilassa (itseajava) moottorit ajetaan suoraan ilman sisäistä PID-korjausta
                direct_commands = [self.clamp_motor_value(int(v)) for v in self.filtered_motor_speeds]
                self.driver.set_motor(*direct_commands)

            # Päivitetään enkooderipohja seuraavaa kierrosta varten
            self.previous_encoders = current_encoders

        except Exception as exc:
            self.get_logger().warning(f'Driver: Virhe sensor/control loopissa: {exc}')

    def calculate_wheel_speeds(self, delta_encoders, delta_time):
        # Calculate current speed for every wheel in the list
        wheel_factor = (math.pi * self.wheel_diameter) / (self.counts_per_revolution * delta_time)
        speeds = [delta * wheel_factor for delta in delta_encoders]
        return speeds

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

    def shutdown_callback(self, string_msg) -> None:
        self.get_logger().info(f"Shutdown trigger received: {string_msg.data}")
        self.stop_motors_and_destroy()

    def stop_motors_and_destroy(self) -> None:
        try:
            self.get_logger().info('Driver: Stopping motors before shutdown')
            self.set_motor_speeds([0, 0, 0, 0])
        except Exception as e:
            self.get_logger().error(f"Failed to stop motors: {e}")
        super().destroy_node()

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
            node.get_logger().info("Driver: Startup failure - shutting down")
            node.stop_motors_and_destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
