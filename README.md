# ros2_robot_v1

# Rosmaster Base Workspace

ROS 2 Jazzy workspace for a Python driver that connects a Yahboom Rosmaster motor control board to ROS 2 topics and services.

## What this package provides

- Launch file: `ros2 launch rosmaster_base driver.launch.py`
- Battery voltage publisher: `/battery_voltage`
- Wheel encoder publisher: `/wheel_encoders`
- Raw motor command topic: `/cmd_raw_motors`
- Stop-motors service: `/stop_motors`

## Workspace layout

```text
rosmaster_base_ws/
├── src/rosmaster_base/
├── build/
├── install/
└── log/
```

## Prerequisites

- Ubuntu with ROS 2 Jazzy installed
- Python 3
- `colcon`
- Yahboom Rosmaster vendor Python library
- USB connection to the Rosmaster board

## 1. Install the vendor Rosmaster library

Download the Rosmaster Python library from Yahboom:

```text
https://www.yahboom.net/study/ROS-Driver-Board
```

On the Yahboom page, open the code/software download for the Rosmaster board, then install the library and `pyserial`:

```bash
python3 setup.py install --user
sudo apt install python3-serial
```

## 2. Check the USB connection

Confirm that the board is visible over USB:

```bash
lsusb
```

Example output:

```text
Bus 004 Device 002: ID 1a86:7523 QinHeng Electronics CH340 serial converter
```

Check the serial device:

```bash
ls -l /dev/ttyUSB0
```

Example printout:
crwxrwxrwx 1 root dialout 188, 0 Jul 31 17:00 /dev/ttyUSB0


If you use a custom symlink such as `/dev/myserial`, verify it with:

```bash
ls -l /dev/myserial
```

Example printout:
lrwxrwxrwx 1 root root 7 Jul 31 17:00 /dev/myserial -> ttyUSB0


## 3. Optional: create a persistent udev rule

If the device path changes between reboots, create a udev rule:

```bash
sudo nano /etc/udev/rules.d/myserial.rules
```

Use this content:

```text
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="myserial"
```

Reload udev rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 4. Configure the serial port

The default driver configuration currently uses:

```yaml
port: "/dev/ttyUSB0"
```

That value is defined in [`driver.yaml`](/home/orangepi/Documents/ros2_robot_v1/rosmaster_base_ws/src/rosmaster_base/config/driver.yaml).

If you want to use `/dev/myserial`, update the `port` value there before launching the node.

## 5. Build the workspace

```bash
cd ~/Documents/ros2_robot_v1/rosmaster_base_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## 6. Run the driver

```bash
cd ~/Documents/ros2_robot_v1/rosmaster_base_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rosmaster_base driver.launch.py
```

If the workspace has already been built, only the `source` and `ros2 launch` commands are needed.

## Interfaces

### Published topics

```text
/battery_voltage    std_msgs/msg/Float32
/wheel_encoders     std_msgs/msg/Int32MultiArray
```

### Subscribed topics

```text
/cmd_raw_motors     std_msgs/msg/Int16MultiArray
```

Expected motor command format:

```text
data: [M1, M2, M3, M4]
```

Each value is clamped to the range `-100` to `100`.

### Services

```text
/stop_motors        std_srvs/srv/Trigger
```

## Test commands

Open a second terminal and source the workspace first:

```bash
cd ~/Documents/ros2_robot_v1/rosmaster_base_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Check battery voltage:

```bash
ros2 topic echo /battery_voltage
```

Check wheel encoders:

```bash
ros2 topic echo /wheel_encoders
```

Drive all motors forward:

```bash
ros2 topic pub --once /cmd_raw_motors std_msgs/msg/Int16MultiArray "{data: [20, 20, 20, 20]}"
```

Drive all motors in reverse:

```bash
ros2 topic pub --once /cmd_raw_motors std_msgs/msg/Int16MultiArray "{data: [-20, -20, -20, -20]}"
```

Turn in place:

```bash
ros2 topic pub --once /cmd_raw_motors std_msgs/msg/Int16MultiArray "{data: [20, -20, 20, -20]}"
```

Stop the motors:

```bash
ros2 service call /stop_motors std_srvs/srv/Trigger "{}"
```

## Notes

- The node name is `rosmaster_driver`.
- The launch file loads parameters from `src/rosmaster_base/config/driver.yaml`.
- If the vendor Python package uses a different import path, update the import in [`driver_node.py`](/home/orangepi/Documents/ros2_robot_v1/rosmaster_base_ws/src/rosmaster_base/rosmaster_base/driver_node.py).














# Return the current battery voltage.
get_battery_voltage(self)

# Clear cached data that was automatically sent by the MCU.
clear_auto_report_data(self)

# Start the background thread that receives and processes incoming data.
create_receive_threading(self)

# Return encoder data for all four motors.
get_motor_encoder(self)

# Set raw motor PWM speed values without encoder-based speed feedback.
# Each `speed_X` must be in the range `[-100, 100]`.
set_motor(self, speed_1, speed_2, speed_3, speed_4)




from Rosmaster_Lib import Rosmaster

bot = Rosmaster()

# Start the receive thread first, then give the board time to auto-report fresh data.
bot.create_receive_threading()

# get_motor_encoder() returns cumulative counts since boot.
print('before:', bot.get_motor_encoder())

# Run with speed 10 motor 1
bot.set_motor(10, 0, 0, 0)
time.sleep(0.3)
print('running:', bot.get_motor_encoder())

time.sleep(2)
print('after 2s:', bot.get_motor_encoder())

# Stop
bot.set_motor(0, 0, 0, 0)
time.sleep(0.2)
print('stopped:', bot.get_motor_encoder())



sudo apt update
sudo apt install python3-websockets

cd src

ros2 pkg create --build-type ament_python --node-name robot_bringup robot_bringup
ros2 pkg create --build-type ament_python --node-name websocket websocket
ros2 pkg create --build-type ament_python --node-name driver driver
ros2 pkg create --build-type ament_python --node-name power_control power_control
ros2 pkg create --build-type ament_python --node-name light_control light_control
ros2 pkg create --build-type ament_python --node-name proximity_sensor proximity_sensor
ros2 pkg create --build-type ament_python --node-name safety_control safety_control
ros2 pkg create --build-type ament_python --node-name odometer odometer
ros2 pkg create --build-type ament_python --node-name lc_manager lc_manager
ros2 pkg create --build-type ament_python --node-name camera camera

sudo usermod -aG gpio orangepi

# 1. Luodaan udev-sääntötiedosto GPIO-oikeuksille
sudo nano /etc/udev/rules.d/99-gpio.rules

SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:gpio /sys/class/gpio/export /sys/class/gpio/unexport ; chmod 220 /sys/class/gpio/export /sys/class/gpio/unexport'"
SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:gpio /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value ; chmod 660 /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value'"

sudo udevadm control --reload-rules && sudo udevadm trigger


cd /home/orangepi/Documents/ros2_robot_v1
rosdep install -i --from-path src --rosdistro jazzy -y


# Build the workspace

cd ~/Documents/ros2_robot_v1
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Run all nodes
source install/setup.bash
ros2 launch robot_bringup robot.launch.py

# Run only light control
source install/setup.bash
ros2 launch light_control light_control.launch.py

# Run only proximity sensor
source install/setup.bash
ros2 launch proximity_sensor proximity_sensor.launch.py

# Run only odometer
source install/setup.bash
ros2 launch odometer odometer.launch.py

# Run only driver
source install/setup.bash
ros2 launch driver driver.launch.py


# I2C bus settings. OrangePi zero 2w have pins for three I2C busses 
sudo ls -la/boot


# Add this line to enable tree I2C busses
overlays=pi-i2c0 pi-i2c1 pi-i2c2

sudo nano /boot/orangepiEnv.txt
save
verify
sudo cat /boot/orangepiEnv.txt
boot

# VL53L5CX sensor default address is 0x29. Attach sensor wires in SCL/SDA pins

SCL 261 (TWI0) yellow, SDA 262 (TWI0) green
SCL 263 (TWI1) yellow, SDA 264 (TWI1) green
SCL 265 (TWI2) yellow, SDA 266 (TWI2) green

# Note that system allocated I2C bus numbers
# Use sudo i2cdetect -y 0 command to find 29 number in there printouts to find correct values for SMBUS
# These are 0, 3, 4 in this case. Use SMBUS(num) in python code

orangepi@orangepizero2w:~/Documents/robot/ros2_robot_v1$ sudo i2cdetect -y 0
[sudo] password for orangepi: 
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- 29 -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --                         
orangepi@orangepizero2w:~/Documents/robot/ros2_robot_v1$ sudo i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- UU -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --                         
orangepi@orangepizero2w:~/Documents/robot/ros2_robot_v1$ sudo i2cdetect -y 2
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: 30 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --                         
orangepi@orangepizero2w:~/Documents/robot/ros2_robot_v1$ sudo i2cdetect -y 3
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- 29 -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --                         
orangepi@orangepizero2w:~/Documents/robot/ros2_robot_v1$ sudo i2cdetect -y 4
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- 29 -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --                         
orangepi@orangepizero2w:~/Documents/robot/ros2_robot_v1$

# Asennetaan paketit suoraan viralliseen järjestelmäpolkuun, jota ROS 2 ei voi ohittaa
sudo pip install smbus2 vl53l1x python-periphery --target=/usr/lib/python3/dist-packages --break-system-packages

# Test lifecycle manager with driver node
source install/setup.bash
ros2 launch driver driver.launch.py
ros2 launch lc_manager lc_manager.py



python3 install vl53l5cx-ctypes


sudo apt update
sudo apt install ros-jazzy-xacro