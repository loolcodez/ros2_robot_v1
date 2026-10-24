import json
from collections import deque
import subprocess
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecyclePublisher
from std_msgs.msg import Float32, String

class PowerControlNode(LifecycleNode):
    def __init__(self):
        super().__init__('power_control')
        self.is_activated = False
        self.beep_alert_sent = False
        self.history_voltage = deque(maxlen=10) # Ring buffer for las 6 measurements (1 minute)

        self.connection_status_sub = None
        self.battery_voltage_sub = None
        self.battery_status_pub : LifecyclePublisher
        self.power_shutdown_pub : LifecyclePublisher
        self.power_alert_pud : LifecyclePublisher

    def on_configure(self, state):
        self.get_logger().info("IN on_configure state")

        # Create subscribers
        self.connection_status_sub = self.create_subscription(String, '/connection/status', self.handle_connection_status, 10)
        self.battery_voltage_sub = self.create_subscription(Float32, '/power/battery_voltage', self.handle_battery_voltage, 10)

        # Create publishers
        self.battery_status_pub = self.create_lifecycle_publisher(String, '/power/battery_status', 10) # type: ignore reportAttributeAccessIssue
        self.power_shutdown_pub = self.create_lifecycle_publisher(String, '/power/shutdown', 10) # type: ignore reportAttributeAccessIssue
        self.power_alert_pud = self.create_lifecycle_publisher(String, '/power/alert', 10) # type: ignore reportAttributeAccessIssue

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

    def handle_connection_status(self, msg):
        if not self.is_activated:
            return
        if msg.data == "connected":
            self.get_logger().info('New websocket connection')
            moving_average = 12
            if len(self.history_voltage) > 0:
                moving_average = round(sum(self.history_voltage) / len(self.history_voltage),1) # Round to one decimal
            status = self.get_battery_status(moving_average)
            status = f"{moving_average:.2f}V {status}"
            msg = String()
            msg.data = json.dumps({"sender": "Rover", "type": "control", "battery_status": status})
            self.get_logger().info(f"Sending battery status: {msg.data}")
            self.battery_status_pub.publish(msg)

    def handle_battery_voltage(self, float32_msg):
        if not self.is_activated:
            return
        current_voltage = float32_msg.data
        self.history_voltage.append(current_voltage)
        moving_average = round(sum(self.history_voltage) / len(self.history_voltage),1) # Round to one decimal
        status = self.get_battery_status(moving_average)
        msg = String()
        msg.data = status
        if status in {"Low", "Very Low", "Critical", "Shutdown"}:
            self.power_alert_pud.publish(msg)
        if status == "Shutdown":
            shutdown_msg = String()
            shutdown_msg.data = "Shutdown voltage. Shutdown in 1 minute"
            self.power_shutdown_pub.publish(shutdown_msg)
            try:
                # Schedule a shutdown in 1 minute
                subprocess.run(["sudo", "shutdown", "-h", "+1"], check=True)
                self.get_logger().info("Shutdown scheduled successfully.")
            except subprocess.CalledProcessError as e:
                self.get_logger().info(f"Failed to schedule shutdown: {e}")

        status = f"{moving_average:.2f}V {status}"
        msg = String()
        msg.data = json.dumps({"sender": "Rover", "type": "control", "battery_status": status})
        self.get_logger().info(f"Sending battery status: {msg.data}")
        self.battery_status_pub.publish(msg)

    def get_battery_status(self, moving_average):
        status = "Full"
        if moving_average < 11.5:
            status = "Good"
        if moving_average < 11.2:
            status = "Low"
        if moving_average < 11.0:
            status = "Very Low"
        if moving_average < 10.8:
            status = "Critical"
        if moving_average < 10.6:
            status = "Shutdown"
        return status

def main(args=None):
    rclpy.init(args=args)
    node = PowerControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
