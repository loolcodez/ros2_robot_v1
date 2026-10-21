from typing import Final

class FakeVL53L1XHardwareInterface:
    def __init__(self, name: str, gpio_pin: int= 0, address: int=29):
        self.name: Final[str] = name
        self.gpio_pin = gpio_pin
        self.address = address
        self.current_distance = -1
        self.ranging_started = False
        self.status = "OK"

    def start_ranging(self):
        if self.status != "OK":
            return self.status
        self.ranging_started = True
        return self.status

    def stop_ranging(self):
        if self.status != "OK":
            return self.status
        self.ranging_started = False
        return self.status

    def get_distance(self):
        if self.status != "OK":
            return self.status
        return self.current_distance

    def cleanup(self):
        if self.status != "OK":
            return self.status
        self.ranging_started = False
        return self.status
