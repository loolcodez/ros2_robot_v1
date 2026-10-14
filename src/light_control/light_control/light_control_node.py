# Set
# ros2 topic pub --once /cmd_lights std_msgs/msg/String "{data: 'lights/front_on'}"
# ros2 topic pub --once /cmd_lights std_msgs/msg/String "{data: 'lights/front_off'}"
# ros2 topic pub --once /cmd_lights std_msgs/msg/String "{data: 'lights/back_on'}"
# ros2 topic pub --once /cmd_lights std_msgs/msg/String "{data: 'lights/back_off'}"
# ros2 topic pub --once /cmd_lights std_msgs/msg/String "{data: 'lights/emergency_on'}"
# ros2 topic pub --once /cmd_lights std_msgs/msg/String "{data: 'lights/emergency_off'}"

# Listen
# ros2 topic echo /lights_status

import time
from enum import Enum
from typing import Final, Dict
from threading import Thread
from periphery import GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# GPIO pins
GPIO_PINS = {
    'front_left': 257,
    'front_right': 228,
    'back_left': 260,
    'back_right': 259
}

class LightState(Enum):
    LIGHT_ON = 1
    LIGHT_OFF = 2
    LIGHT_BLINKING = 3

class Light:
    def __init__(self, name: str, gpio_pin: int, initial_state: LightState = LightState.LIGHT_OFF):
        self.name: Final[str] = name
        self.gpio_pin: Final[int] = gpio_pin
        self.state: LightState = initial_state
        self.gpio_object: GPIO | None = None  # Here is stored periphery.GPIO-object later

    def set_pin_state(self, new_state: LightState):
        """Changes internal state and updates physical pin state, except when blinking"""
        self.state = new_state

        # If pin initialized successfully, write pin value
        if self.gpio_object is not None:
            if new_state == LightState.LIGHT_ON:
                self.gpio_object.write(True)
            elif new_state == LightState.LIGHT_OFF:
                self.gpio_object.write(False)
            elif new_state == LightState.LIGHT_BLINKING:
                self.gpio_object.write(False)
        print(f"Light '{self.name}' (GPIO {self.gpio_pin}) changed to: {self.state.name}")

    # def read_pin_state(self) -> bool:
    #     if self.gpio_object is not None:
    #         pin_value = self.gpio_object.read()
    #         print(f"Light '{self.name}' (GPIO {self.gpio_pin}) value: {pin_value}")
    #         return pin_value
    #     return False

class LightsControlNode(Node):
    def __init__(self):
        super().__init__('lights_control_node')

        # Subscribe light setting commands
        # "lights/front_on"
        # "lights/front_off"
        # "lights/back_on"
        # "lights/back_off"
        # "lights/emergency_on"
        # "lights/emergency_off"
        self.command_sub = self.create_subscription(String, 'cmd_lights', self.command_callback, 10)

        #self.status_pub = self.create_publisher(String, 'lights_status', 10)

        self.lights : Dict[str, Light] = {
            'front_left': Light("Front Left", GPIO_PINS['front_left']),
            'front_right': Light("Front Right", GPIO_PINS['front_right']),
            'back_left': Light("Back Left", GPIO_PINS['back_left']),
            'back_right': Light("Back Right", GPIO_PINS['back_right'])
        }

        self.light_blinking = False
        self.blink_thread = None

        self.init_gpios()
        self.get_logger().info('Lights Control Node started')

    def init_gpios(self):
        """Initialize GPIO objects for Light objects"""
        for name, light_object in self.lights.items():
            try:
                # Store GPIO object in Light object
                light_object.gpio_object = GPIO(light_object.gpio_pin, "out")
                # Set initial state LOW/OFF
                light_object.set_pin_state(LightState.LIGHT_OFF)
            except Exception as e:
                self.get_logger().error(f'Failed to set name {name}, pin {light_object.gpio_pin} as output: {e}')

    def set_front_lights(self, new_state: LightState):
        self.lights['front_left'].set_pin_state(new_state)
        self.lights['front_right'].set_pin_state(new_state)

    def set_back_lights(self, new_state: LightState):
        self.lights['back_left'].set_pin_state(new_state)
        self.lights['back_right'].set_pin_state(new_state)

    def set_emergency_lights(self, new_state: LightState):
        for name in self.lights:
            self.set_light(name, new_state)

    def set_light(self, name, new_state: LightState):
        self.lights[name].set_pin_state(new_state)
        #self.set_pin_value(self.lights[name].gpio_pin, new_state)

    def command_callback(self, msg: String):
        cmd = msg.data.lower()
        self.get_logger().info(f"Received command: {cmd}")
        if cmd == "lights/front_on":
            self.get_logger().info("Front lights ON")
            self.set_front_lights(LightState.LIGHT_ON)
            return
        if cmd == "lights/front_off":
            self.get_logger().info("Front lights OFF")
            self.set_front_lights(LightState.LIGHT_OFF)
            return
        if cmd == "lights/back_on":
            self.get_logger().info("Back lights ON")
            self.set_back_lights(LightState.LIGHT_ON)
            return
        if cmd == "lights/back_off":
            self.get_logger().info("Back lights OFF")
            self.set_back_lights(LightState.LIGHT_OFF)
            return
        if cmd == "lights/emergency_on":
            if self.light_blinking is False:
                self.get_logger().info("Emergency lights ON")
                self.set_emergency_lights(LightState.LIGHT_BLINKING)
                self.light_blinking = True
                self.start_blinking()
            return
        if cmd == "lights/emergency_off":
            if self.light_blinking is True:
                self.get_logger().info("Emergency lights OFF")
                self.light_blinking = False
                self.set_emergency_lights(LightState.LIGHT_OFF)
            return
        else:
            self.get_logger().error(f"Unkonwn command: {cmd}")

    def start_blinking(self):
        self.blink_thread = Thread(target=self._blink_loop)
        self.blink_thread.daemon = True # This ensures that thread is terminated if ROS2 node gets closed
        self.blink_thread.start()

    def _blink_loop(self):
        state_toggle = False
        while self.light_blinking and rclpy.ok():
            state_toggle = not state_toggle  # Changes between True ja False

            for _, light in self.lights.items():
                if light.state == LightState.LIGHT_BLINKING and light.gpio_object is not None:
                    light.gpio_object.write(state_toggle)
            time.sleep(0.5) # Blink rate 0.5s ON and OFF

    def publish_status(self):
        import json
        status = {
            'lights': self.lights,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LightsControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_blinking()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
