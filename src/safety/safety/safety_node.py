import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # Set speed correction value. Value range received from Joystick is -1 .. 1
        self.declare_parameter('max_linear', 60.0) # Speed. Must be in the range `[-100, 100]`
        self.declare_parameter('max_angular', 60.0) # Turning. Must be in the range `[-100, 100]`

        self.safety_distance_mm = 300  # Safety distance in millimeters
        self.current_distance_mm = 9999  # Default range when object is not detected
        self.sensor = ""
        self.max_linear = float(self.get_parameter('max_linear').get_parameter_value().double_value)
        self.max_angular = float(self.get_parameter('max_angular').get_parameter_value().double_value)

        # Listener distance sensors
        self.range_sub = self.create_subscription(String, 'range', self.range_callback, 10)

        # Listener for incoming control commands from joystick and AI
        self.manual_control_sub = self.create_subscription(String, 'cmd_vel_manual', self.cmd_vel_manual_callback, 10)

        # Publisher for checked command
        self.cmd_vel_out_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)

        self.get_logger().info(f"Safety Node started. Safety distance: {self.safety_distance_mm} mm")

    def range_callback(self, msg):
        """Update latest distance information"""

        json_message = json.loads(msg.data)
        self.sensor = json_message.get('sensor')
        self.current_distance_mm = json_message.get('distance')
        self.get_logger().info(f"SafetyNode: Sensor: {self.sensor}, Distance: {self.current_distance_mm} mm")

    def cmd_vel_manual_callback(self, msg):
        """Check safety of incoming drive command"""

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

            # Add speed correction --> from -1 .. 1 to -60 .. 60
            twist_msg.linear.x = twist_msg.linear.x * self.max_linear
            twist_msg.angular.z = -twist_msg.angular.z * self.max_angular

            # Ensure maximum values are not exceed
            twist_msg.linear.x = max(-self.max_linear, min(self.max_linear, twist_msg.linear.x))
            twist_msg.angular.z = max(-self.max_angular, min(self.max_angular, twist_msg.angular.z))

            # Store turning speed (No need to prevent turning usually)
            if twist_msg.linear.x > 0 or twist_msg.linear.x < 0:
                if self.current_distance_mm < self.safety_distance_mm:
                    # Road block ahead: Force drive speed to zero
                    twist_msg.linear.x = 0.0
                    self.get_logger().warn(
                        f"Moving stopped! Road blocked: {self.current_distance_mm} mm. Drive command suspended",
                        throttle_duration_sec=1.0  # Print in log once per second
                    )
            # Publish filtered command to driver node
            self.cmd_vel_out_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
