
# Allow usage of GPIO interface

sudo usermod -aG gpio orangepi

# Create udev file for GPIO access rights

sudo nano /etc/udev/rules.d/99-gpio.rules

SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:gpio /sys/class/gpio/export /sys/class/gpio/unexport ; chmod 220 /sys/class/gpio/export /sys/class/gpio/unexport'"
SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:gpio /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value ; chmod 660 /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value'"

# Reload settings or just reboot OrangePi

sudo udevadm control --reload-rules && sudo udevadm trigger

# Install library for Vl53l5CX sensor

python3 install vl53l5cx-ctypes


# VL53L5CX sensor default address is 0x29. Attach sensor wires in SCL/SDA pins
OrangePi have three I2C buses which can be taken into use

# First attach sensors in GPIO pins. Sensors need to be also powered
SCL 261 (TWI0) yellow wire, SDA 262 (TWI0) green wire (I2C bus 0)
SCL 263 (TWI1) yellow wire, SDA 264 (TWI1) green wire (I2C bus 1)
SCL 265 (TWI2) yellow wire, SDA 266 (TWI2) green wire (I2C bus 2)

LPn_pins. After boot sensor is active even these pins would not be connected
- FRONT_LEFT = 256
- FRONT_RIGHT = 271
- BACK_LEFT = 258
- BACK_RIGHT = 272

# I2C bus settings. OrangePi zero 2w have pins for three I2C busses 

sudo ls -la/boot

# Add this line to enable /boot/orangepiEnv.txt file to enable three I2C busses

sudo nano /boot/orangepiEnv.txt

overlays=pi-i2c0 pi-i2c1 pi-i2c2

Save the file (Ctrl+O, Enter) and exit (Ctrl+X)

verify changes

sudo cat /boot/orangepiEnv.txt

Reboot OrangePi

sudo reboot

Note that system allocates the I2C bus numbers! # Use sudo i2cdetect -y 0 command to find 29 number in three printouts to find correct values for SMBUS to be used in the code.
Below example 0, 3, 4 are the one. Use these numbers for SMBUS(num) in python code
Use bus number in the code depends on the overlays configuration line

sudo i2cdetect -y 0
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


# How to change increase i2c bus frequency in Orange Pi zero 2w running Ubuntu22.04

If three I2C busses would used line would be this: overlays=pi-i2c0 pi-i2c1 pi-i2c2

We are using these buses: overlays=pi-i2c0 pi-i2c2

Step 1: Create a Custom Device Tree Source File

First, open a terminal and create a new Device Tree Source (.dts) file using your text editor
nano i2c-sensor-fastmode.dts

Paste the following configuration into the file. This targeting script ensures the default clock-frequency properties are overwritten with 400000 Hz (400 kHz):

--

/dts-v1/;
/plugin/;

/ {
    compatible = "allwinner,sun50i-h616";

    fragment@0 {
        target = <&i2c0>;
        __overlay__ {
            clock-frequency = <400000>;
        };
    };

    fragment@1 {
        target = <&i2c1>;
        __overlay__ {
            clock-frequency = <400000>;
        };
    };

    fragment@2 {
        target = <&i2c2>;
        __overlay__ {
            clock-frequency = <400000>;
        };
    };
};

--

--

/dts-v1/;
/plugin/;

/ {
    compatible = "allwinner,sun50i-h616";

    fragment@0 {
        target = <&i2c0>;
        __overlay__ {
            clock-frequency = <400000>;
        };
    };

    fragment@2 {
        target = <&i2c2>;
        __overlay__ {
            clock-frequency = <400000>;
        };
    };
};

--


(Note: If you only need to change the frequency of a single specific bus, such as /dev/i2c-2, you can keep just the fragment target block corresponding to that specific bus number).


Step 2: Compile the Device Tree Overlay

Install compiler if not already installed
sudo apt update && sudo apt install -y device-tree-compiler xxd

Compile the text .dts file into a binary .dtbo overlay using the device tree compiler (dtc):

dtc -I dts -O dtb -o i2c-sensor-fastmode.dtbo i2c-sensor-fastmode.dts

Step 3: Move the Overlay into Boot Storage

Copy your freshly compiled overlay file to your Orange Pi overlay management folder:

sudo cp i2c-sensor-fastmode.dtbo /boot/overlay-user/

Verify file existence

ls -la /boot/overlay-user/

drwxr-xr-x 2 root root 4096 Nov  5 20:26 .
drwxr-xr-x 4 root root 4096 Nov  5 18:14 ..
-rw-r--r-- 1 root root  501 Nov  5 20:26 i2c-sensor-fastmode.dtbo

Step 4: Activating the Overlay via Environment Configuration

Open your environment boot profile file:

sudo nano /boot/orangepiEnv.txt

Add user_overlays= line at the end of file:

user_overlays=i2c-sensor-fastmode

Save the file (Ctrl+O, Enter) and exit (Ctrl+X):

Verify file content:

sudo cat /boot/orangepiEnv.txt

Reboot your hardware for the change to take effect:

sudo reboot

If successful you can see initializing and reading of or sensors happening faster