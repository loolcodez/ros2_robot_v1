# This is used only in real HW
# Separate HW class is easy to mock in unit tests or simulation class when real HW is not used

import time
from typing import Final, Tuple, List, Optional
from periphery import GPIO
from vl53l5cx.vl53l5cx import VL53L5CX

class VL53L5CXHardwareInterface:
    """Hardware interface class for VL53L5CX 8x8 ToF Sensor"""

    def __init__(self, name: str, lp_pin: int):
        self.name: Final[str] = name
        self.status = "OK"
        self.tof: Optional[VL53L5CX] = None

        # Init LPn (XSHUT) pin and set it OUTPUT-state. All sensor'ss LPn pins must be LOW before initialization
        try:
            self.lpn = GPIO(lp_pin, "out")
            self.lpn.write(False) # LPn pin to LOW
            time.sleep(0.2)
        except Exception as e:
            self.status = f"{self.name} GPIO initialization failed on lp_pin: {lp_pin}: {e}"

    def initialize_sensor(self, target_address: int = 0x29, i2c_bus: int = 0):
        if self.status != "OK":
            return self.status

        self.lpn.write(True) # LPn pin to HIGH
        time.sleep(0.2)
        try:
            if target_address != 0x29:
                temp_tof = VL53L5CX(i2c_bus=i2c_bus, i2c_address=0x29)
                temp_tof.set_i2c_address(target_address)
                self.tof = VL53L5CX(i2c_bus=i2c_bus, i2c_address=target_address)
            else:
                self.tof = VL53L5CX(i2c_bus=i2c_bus, i2c_address=0x29)

            # Configure sensor to 8x8 state (64 areas) ja 15 Hz sample speed
            self.tof.set_resolution(8 * 8) # type: ignore
            self.tof.set_ranging_frequency_hz(15) # type: ignore
        except Exception as e:
            self.status = f"{self.name} sensor initialization failed on bus {i2c_bus}, address {target_address:#x}: {e}"
        return self.status

    def start_ranging(self) -> str:
        if self.status != "OK" or self.tof is None:
            return self.status
        try:
            self.tof.start_ranging() # type: ignore
            return self.status
        except Exception as e:
            return f"Error in starting ranging on sensor {self.name}: {e}"

    def stop_ranging(self) -> str:
        if self.status != "OK" or self.tof is None:
            return self.status
        try:
            self.tof.stop_ranging() # type: ignore
            return self.status
        except Exception as e:
            return f"Error in stopping ranging on sensor {self.name}: {e}"

    def get_distances_and_statuses(self) -> Tuple[Optional[List[int]], Optional[List[int]], str]:
        if self.status != "OK" or self.tof is None:
            return None, None, self.status
        try:
            if self.tof.data_ready():
                data = self.tof.get_data()
                return data.distance_mm, data.target_status, "OK"
            return None, None, "NO_NEW_DATA"
        except Exception as e:
            return None, None, f"Error in reading sensor {self.name}: {e}"

    def cleanup(self) -> str:
        self.status = "OK"
        if self.tof:
            try:
                self.tof.stop_ranging()
            except Exception:
                pass
        try:
            self.lpn.write(False)
            self.lpn.close()
        except Exception as e:
            return f"Error in sensor cleanup: {self.name} {e}"
        return self.status
