# This is used only in real HW
# Separate HW class is easy to mock in unit tests or simulation class when real HW is not used

import time
from typing import Final

# HW library imports
from periphery import GPIO
import VL53L1X as ToFSensor

class VL53L1XHardwareInterface:

    """Hardware interface class"""
    def __init__(self, name: str, gpio_pin: int, address: int):
        self.name: Final[str] = name
        self.status = "OK"

        # Init XSHUT pins and set it OUTPUT-state
        try:
            self.xshut = GPIO(gpio_pin, "out")
            self.xshut.write(False)
            time.sleep(0.5)
            self.xshut.write(True)
            time.sleep(0.5)
        except Exception as e:
            self.status = f"{self.name} GPIO initialization failed on gpio_pin: {gpio_pin}: {e}"

        try:
            # Sensor wakes up using default 0x29 address
            self.tof = ToFSensor.VL53L1X(i2c_bus=0, i2c_address=0x29)
            if address != 0x29:
                self.tof.open()
                self.tof.change_address(address)
                self.tof.close()
                self.tof = ToFSensor.VL53L1X(i2c_bus=0, i2c_address=address)
            self.tof.open()
            self.tof.set_timing(50000, 55)
        except Exception as e:
            self.status = f"{self.name} sensor initialization failed on address: {address:#x}: {e}"

    def start_ranging(self):
        if self.status != "OK":
            return self.status
        try:
            # Set measurement profile (1 = Short range, 2 = Long range)
            self.tof.start_ranging(2)
            return self.status
        except Exception as e:
            return f"Error in starting ranging on sensor {self.name} failed: {e}"

    def stop_ranging(self):
        if self.status != "OK":
            return self.status
        try:
            self.tof.stop_ranging()
            return self.status
        except Exception as e:
            return f"Error in stopping ranging on sensor {self.name} failed: {e}"

    def get_distance(self):
        if self.status != "OK":
            return self.status
        try:
            return self.tof.get_distance()
        except Exception as e:
            return f"Error in reading sensor: {self.name} {e}"

    def cleanup(self):
        if self.status != "OK":
            return self.status
        try:
            self.tof.stop_ranging()
            self.xshut.write(False)
            self.xshut.close()
            return self.status
        except Exception as e:
            return f"Error in sensor cleanup: {self.name} {e}"
