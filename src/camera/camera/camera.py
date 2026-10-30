# orangepi@orangepizero2w:~$ lsusb
# Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
# Bus 001 Device 002: ID 05e3:0610 Genesys Logic, Inc. Hub
# Bus 001 Device 003: ID 0edc:2050 YJXU502S USB Camera
# Bus 001 Device 004: ID 0edc:2050 YJXU502S USB Camera
# Bus 001 Device 005: ID 1a86:7523 QinHeng Electronics CH340 serial converter
# Bus 002 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
# Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
# Bus 004 Device 001: ID 1d6b:0001 Linux Foundation 1.1 root hub
# Bus 005 Device 001: ID 1d6b:0001 Linux Foundation 1.1 root hub
# Bus 006 Device 001: ID 1d6b:0001 Linux Foundation 1.1 root hub

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecyclePublisher
from std_msgs.msg import String

class CameraNode(LifecycleNode):
    def __init__(self):
        super().__init__('camera_node')
        self.is_activated = False
        self.camera_status = None
        self.camera_status_sub = None

        self.get_logger().info("Camera Node created")

    def on_configure(self, state):
        self.get_logger().info("IN on_configure state")
        # Front_ON, Front_OFF, Back_ON, Back_OFF, All_OFF
        self.camera_status_sub = self.create_subscription(String, '/camera/status', self.handle_camera_status, 10)

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

    def handle_camera_status(self, msg: String):
        if not self.is_activated:
            return

        command = msg.data
        if command not in {"Front_ON", "Front_OFF", "Back_ON", "Back_OFF", "All_OFF"}:
            self.get_logger().error(f"Unknown command: {command}")
            return

        if command == "Front_ON":
            # Activate front camera
            pass
        if command == "Front_OFF":
            # Deactivate front camera
            pass
        if command == "Back_ON":
            # Activate back camera
            pass
        if command == "Back_OFF":
            # Deactivate back camera
            pass
        if command == "All_OFF":
            # Deactivate  front and back camera
            pass
        self.camera_status = command

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
