import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry

class WheelOdometerNode(Node):
    def __init__(self):
        super().__init__('wheel_odometer_node')

        # Physical dimensions
        self.counts_per_revolution = 1320.0 # Encoder counts
        self.wheel_diameter = 0.065 # 65mm
        self.wheel_separation = 0.30 # 0.3M left - right, measured middle of wheels

        self.previous_counts = None
        self.last_time = self.get_clock().now()

        # Robots place (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.encoder_sub = self.create_subscription(
            Int32MultiArray, '/wheel_encoders', self.process_encoders, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom_wheel', 10)

    def process_encoders(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        current_counts = msg.data # [M0, M1, M2, M3]

        if self.previous_counts is None:
            self.previous_counts = current_counts
            self.last_time = current_time
            return

        # 1. Count change in pulses (delta) for every motor
        deltas = [curr - prev for curr, prev in zip(current_counts, self.previous_counts)]

        # 2. Average left and right side (Example: M0 and M2 left, M1 and M3 right)
        delta_left = (deltas[0] + deltas[2]) / 2.0
        delta_right = (deltas[1] + deltas[3]) / 2.0

        # 3. Change pulses to meters
        pulse_to_meter = (math.pi * self.wheel_diameter) / self.counts_per_revolution
        d_left = delta_left * pulse_to_meter
        d_right = delta_right * pulse_to_meter

        # 4. Averaged robots movement (linear distance and and direction change)
        d_center = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation

        # 5. Update place in the world (Dead Reckoning)
        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)
        self.th += d_theta

        # 6. Publish odometry message (/odom_wheel)
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        cy = math.cos(self.th * 0.5)
        sy = math.sin(self.th * 0.5)
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy

        # Speeds (m/s and rad/s)
        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = d_theta / dt
        self.get_logger().info(f"OdoMeter: Speed linear.x {odom.twist.twist.linear.x}")
        self.get_logger().info(f"OdoMeter: Speed angular.z {odom.twist.twist.angular.z}")
        self.get_logger().info(f"OdoMeter: Position.x {odom.pose.pose.position.x}")
        self.get_logger().info(f"OdoMeter: Position.z {odom.pose.pose.position.z}")

        self.odom_pub.publish(odom)

        self.previous_counts = current_counts
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelOdometerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.get_logger().info("wheel_odometry_node startup failure - shutting down")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
