import json
from collections import deque
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class PowerControlNode(Node):
    def __init__(self):
        super().__init__('power_control')
        self.beep_alert_sent = False
        self.history_voltage = deque(maxlen=10) # Ring buffer for las 6 measurements (1 minute)

        self.battery_voltage_sub = self.create_subscription(Float32, '/power/battery_voltage', self.handle_battery_voltage, 10)
        self.battery_status_pub = self.create_publisher(String, '/power/battery_status', 10)
        self.power_shutdown_pub = self.create_publisher(String, '/power/shutdown', 10)
        self.power_alert_pud = self.create_publisher(String, '/power/alert', 10)

    def handle_battery_voltage(self, float32_msg):
        current_voltage = float32_msg.data
        self.history_voltage.append(current_voltage)
        moving_average = sum(self.history_voltage) / len(self.history_voltage)
        status = "Full"
        if moving_average < 11.5:
            status = "Good"
        if moving_average < 11.2:
            status = "Low"
            msg = String()
            msg.data = status
            self.power_alert_pud.publish(msg)
        if moving_average < 11.0:
            status = "Very Low"
            msg = String()
            msg.data = status
            self.power_alert_pud.publish(msg)
        if moving_average < 10.8:
            status = "Critical"
            msg = String()
            msg.data = status
            self.power_alert_pud.publish(msg)
        if moving_average < 10.6:
            status = "Shutdown"
            msg = String()
            msg.data = status
            self.power_alert_pud.publish(msg)
            shutdown_msg = String()
            shutdown_msg.data = "Shutdown voltage. Shutdown in 1 minute"
            self.power_shutdown_pub.publish(shutdown_msg)
            try:
                # Schedule a shutdown in 1 minute
                subprocess.run(["sudo", "shutdown", "-h", "+1"], check=True)
                self.get_logger().info("Shutdown scheduled successfully.")
            except subprocess.CalledProcessError as e:
                self.get_logger().info(f"Failed to schedule shutdown: {e}")

        status = f"{current_voltage:.2f}V {status}"
        msg = String()
        msg.data = json.dumps({"sender": "Rover", "type": "control", "battery_status": status})
        self.get_logger().info(f"PowerControlNode: Sending battery status: {msg}")
        self.battery_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PowerControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
