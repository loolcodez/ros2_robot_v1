import json
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecyclePublisher
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SafetyControlNode(LifecycleNode):
    def __init__(self):
        super().__init__('safety_control_node')
        self.is_activated = False
        self.move_direction = None

        # Set speed correction value. Value range received from Joystick is -1 .. 1
        self.declare_parameter('max_linear', 60.0) # Speed. Must be in the range `[-100, 100]`
        self.declare_parameter('max_angular', 60.0) # Turning. Must be in the range `[-100, 100]`

        self.speed_limit_factor = 1
        self.safety_distance_mm = 300  # Safety distance in millimeters
        self.current_distance_mm = 9999  # Default range when object is not detected
        self.sensor = ""
        self.max_linear = float(self.get_parameter('max_linear').get_parameter_value().double_value)
        self.max_angular = float(self.get_parameter('max_angular').get_parameter_value().double_value)

        self.range_sub = None
        self.manual_control_sub = None
        self.manual_control_sub = None
        self.power_alert_sub = None
        self.cmd_vel_out_pub : LifecyclePublisher

        self.get_logger().info(f"Safety Node created. Safety distance: {self.safety_distance_mm} mm")

    def on_configure(self, state):
        self.get_logger().info("IN on_configure state")
        # Listener for distance sensors
        self.range_sub = self.create_subscription(String, 'range', self.range_callback, 10)

        # Listener for incoming control commands from joystick
        self.manual_control_sub = self.create_subscription(String, 'cmd_vel_manual', self.cmd_vel_manual_callback, 10)

        # Listener for incoming control commands from AI
        self.manual_control_sub = self.create_subscription(Twist, 'cmd_vel_in', self.cmd_vel_in_callback, 10)

        # Listener for power alert
        self.power_alert_sub = self.create_subscription(String, '/power/alert', self.power_alert_callback, 10)

        # Publisher for checked command
        self.cmd_vel_out_pub = self.create_lifecycle_publisher(Twist, 'cmd_vel_out', 10) # type: ignore reportAttributeAccessIssue

        self.direction_pub = self.create_lifecycle_publisher(String, '/move/direction', 10) # type: ignore reportAttributeAccessIssue

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

    def range_callback(self, msg):
        """Update latest distance information"""
        if not self.is_activated:
            return

        json_message = json.loads(msg.data)
        self.sensor = json_message.get('sensor')
        self.current_distance_mm = json_message.get('distance')
        self.get_logger().info(f"SafetyNode: Sensor: {self.sensor}, Distance: {self.current_distance_mm} mm")

    def power_alert_callback(self, msg):
        if not self.is_activated:
            return
        self.speed_limit_factor = 1
        if msg.data == "Low":
            self.speed_limit_factor = 0.5
        if msg.data in ("Very Low", "Critical"):
            self.speed_limit_factor = 0.0
        self.get_logger().info(f"SafetyNode: speed_limit_factor: {self.speed_limit_factor}")

    def cmd_vel_manual_callback(self, msg):
        """Check safety of incoming drive command"""
        if not self.is_activated:
            return

        # Get control data from JSON message
        json_message = json.loads(msg.data)
        self.get_logger().info(f"SafetyNode: Received message: {json_message}")
        sender = json_message.get('sender')
        if sender == 'Joystick':
            # Create standard ROS 2 move message
            # Minus sign changes direction of movement
            twist_msg = Twist()
            twist_msg.linear.x = float(json_message.get('yOffset', 0.0))
            twist_msg.angular.z = -float(json_message.get('xOffset', 0.0))

            # If control command is the same as previous drop it.
            #if (self.previous_twist_msg.linear.x != twist_msg.linear.x or
            #    self.previous_twist_msg.angular.z != twist_msg.angular.z):

            #self.previous_twist_msg.linear.x = twist_msg.linear.x
            #self.previous_twist_msg.angular.z = twist_msg.angular.z

            self.get_logger().info(
                f"SafetyNode: received cmd_vel_manual -> "
                f"linear.x: {twist_msg.linear.x:.2f}, angular.z: {twist_msg.angular.z:.2f}"
            )

            if twist_msg.linear.x > 0 and self.move_direction != "Forward":
                self.move_direction = "Forward"
                self.direction_pub.publish(self.move_direction)
            elif twist_msg.linear.x < 0 and self.move_direction != "Backward":
                self.move_direction = "Backward"
                self.direction_pub.publish(self.move_direction)

            # Add speed correction --> from -1 .. 1 to -60 .. 60
            twist_msg.linear.x = twist_msg.linear.x * self.max_linear
            twist_msg.angular.z = -twist_msg.angular.z * self.max_angular

            self.cmd_vel_in_callback(twist_msg)

    def cmd_vel_in_callback(self, twist_msg):
        """Check safety of incoming drive command"""
        if not self.is_activated:
            return

        # Ensure maximum values are not exceed
        twist_msg.linear.x = max(-self.max_linear, min(self.max_linear, twist_msg.linear.x))
        twist_msg.angular.z = max(-self.max_angular, min(self.max_angular, twist_msg.angular.z))

        # Limit speed when low battery
        twist_msg.linear.x = twist_msg.linear.x * self.speed_limit_factor
        twist_msg.angular.z = twist_msg.angular.z * self.speed_limit_factor

        if twist_msg.linear.x > 0 or twist_msg.linear.x < 0:
            if self.current_distance_mm < self.safety_distance_mm:
                # Road block ahead: Force drive speed to zero
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0 # Turning may not need to be limited
                self.get_logger().warn(
                    f"SafetyNode: Moving stopped! Road blocked: {self.current_distance_mm} mm. Drive command suspended",
                    throttle_duration_sec=1.0 # Print in log once per second
                )
        self.get_logger().info(
            f"SafetyNode: sending cmd_vel_out -> "
            f"linear.x: {twist_msg.linear.x:.2f}, angular.z: {twist_msg.angular.z:.2f}",
            throttle_duration_sec=1.0 # Print in log once per second
        )

        # Publish filtered command to driver node
        self.cmd_vel_out_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
