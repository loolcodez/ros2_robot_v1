# This is used only in real HW
# Separate HW class is easy to mock in unit tests or simulation class when real HW is not used

import time
from typing import Final, Tuple, List, Optional
from smbus2 import SMBus
from periphery import GPIO
import vl53l5cx_ctypes as vl53l5cx
import numpy

class VL53L5CXHardwareInterface:
    """Hardware interface class for vl53l5cx 8x8 ToF Sensor"""

    def __init__(self, name: str, lp_pin: int):
        self.name: Final[str] = name
        self.status = "OK"
        self.tof = None
        self.target_address = None

        # Init LPn (XSHUT) pin and set it OUTPUT-state. All sensor'ss LPn pins must be LOW before initialization
        try:
            self.lpn = GPIO(lp_pin, "out")
            self.lpn.write(False) # LPn pin to LOW
            time.sleep(0.5)
        except Exception as e:
            self.status = f"{self.name} GPIO initialization failed on lp_pin: {lp_pin}: {e}"

    def initialize_sensor(self, smbus: SMBus, target_address: int = 0x29)-> str:
        if self.status != "OK":
            return self.status

        self.target_address = target_address
        self.lpn.write(True) # LPn pin to HIGH
        time.sleep(0.3)
        try:
            if target_address != 0x29:
                temp_tof = vl53l5cx.VL53L5CX(i2c_addr=0x29, i2c_dev=smbus)
                temp_tof.set_i2c_address(target_address)
                time.sleep(0.1)

                self.tof = vl53l5cx.VL53L5CX(i2c_addr=target_address, i2c_dev=smbus)
            else:
                self.tof = vl53l5cx.VL53L5CX(i2c_addr=0x29, i2c_dev=smbus)

            # Configure sensor to 8x8 state (64 areas) ja 15 Hz sample speed
            self.tof.set_resolution(64) # type: ignore
            self.tof.set_ranging_frequency_hz(15) # type: ignore
        except Exception as e:
            self.status = f"{self.name} sensor initialization failed on bus {smbus}, address {target_address:#x}: {e}"
            #self.lpn.write(False) # LPn pin to LOW
            time.sleep(0.5)
        return self.status

    def restore_sensor_addres(self):
        self.tof.set_i2c_address(0x29) # type: ignore
        time.sleep(0.1)

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
                # Muutetaan ctypes-datamassa NumPyn kautta tavallisiksi 1D-listoiksi.
                # flipud kääntää matriisin pystysuunnassa (kuten Pimoronin esimerkki vaatii).
                # .flatten() varmistaa, että palautus on suora 64 alkion jono 8x8-matriisin sijaan.
                dist_grid = numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8))).flatten()
                stat_grid = numpy.flipud(numpy.array(data.target_status).reshape((8, 8))).flatten()

                # Muutetaan listoiksi, jotta tyyppitarkistukset ja ROS2-noden silmukat toimivat heti
                distances = [int(d) for d in dist_grid]
                status_info = [int(s) for s in stat_grid]
                return distances, status_info, "OK"
            return None, None, "NO_NEW_DATA"
        except Exception as e:
            return None, None, f"Error in reading sensor {self.name}: {e}"

    def cleanup(self) -> str:
        self.status = "OK"
        if self.tof:
            try:
                self.tof.stop_ranging()
#                self.tof.restore_sensor_addres()
            except Exception:
                pass
        try:
            if self.target_address != 0x29:
                self.lpn.write(False)
                self.lpn.close()
        except Exception as e:
            return f"Error in sensor cleanup: {self.name} {e}"
        return self.status
