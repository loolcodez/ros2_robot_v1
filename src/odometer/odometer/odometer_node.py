import math
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecyclePublisher
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry

class OdometerNode(LifecycleNode):
    def __init__(self):
        super().__init__('odometer_node')
        self.is_activated = False

        # Physical dimensions
        self.counts_per_revolution = 1320.0 # Encoder counts
        self.wheel_diameter = 0.065 # 65mm
        self.wheel_separation = 0.20 # 0.2M left - right, measured middle of wheels

        self.previous_counts = None
        self.last_time = self.get_clock().now()

        # Robots place (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.encoder_sub = None
        self.odom_pub : LifecyclePublisher
        self.get_logger().info('OdometerNode created')

    def on_configure(self, state):
        self.get_logger().info("IN on_configure state")

        self.encoder_sub = self.create_subscription(Int32MultiArray, '/wheel_encoders', self.process_encoders, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom_wheel', 10) # type: ignore
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

    def process_encoders(self, msg):
        if not self.is_activated:
            return
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        # left_front, right_front, left_rear, right rear
        current_counts = msg.data # [M0, M1, M2, M3]

        if self.previous_counts is None:
            self.previous_counts = current_counts
            self.last_time = current_time
            return

        # Count change in pulses (delta) for every motor
        deltas = [curr - prev for curr, prev in zip(current_counts, self.previous_counts)]

        # Average left and right side (Example: M0 and M2 left, M1 and M3 right)
        delta_left = (deltas[0] + deltas[2]) / 2.0
        delta_right = (deltas[1] + deltas[3]) / 2.0

        # Change pulses to meters
        pulse_to_meter = (math.pi * self.wheel_diameter) / self.counts_per_revolution
        d_left = delta_left * pulse_to_meter
        d_right = delta_right * pulse_to_meter

        # Averaged robots movement (linear distance and and direction change)
        d_center = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation

        # Update place in the world (Dead Reckoning)
        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)
        self.th += d_theta

        # Publish odometry message (/odom_wheel)
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
        self.get_logger().info(f"Speed linear.x {odom.twist.twist.linear.x}")
        self.get_logger().info(f"Speed angular.z {odom.twist.twist.angular.z}")
        self.get_logger().info(f"Position.x {odom.pose.pose.position.x}")
        self.get_logger().info(f"Position.y {odom.pose.pose.position.y}")

        self.odom_pub.publish(odom)

        self.previous_counts = current_counts
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OdometerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.get_logger().info("Startup failure - shutting down")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
