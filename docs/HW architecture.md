How to change increase i2c bus frequency in Orange Pi zero 2w running Ubuntu22.04


Step 1: Create a Custom Device Tree Source File

First, open a terminal and create a new Device Tree Source (.dts) file using your text editor
nano i2c-fastmode.dts

Paste the following configuration into the file. This targeting script ensures the default clock-frequency properties are overwritten with 400000 Hz (400 kHz):
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

    fragment@3 {
        target = <&i2c3>;
        __overlay__ {
            clock-frequency = <400000>;
        };
    };
};

(Note: If you only need to change the frequency of a single specific bus, such as /dev/i2c-3, you can keep just the fragment target block corresponding to that specific bus number).


Step 2: Compile the Device Tree Overlay

Compile the text .dts file into a binary .dtbo overlay using the device tree compiler (dtc):

dtc -I dts -O dtb -o i2c-fastmode.dtbo i2c-fastmode.dts

(If the terminal warns you that dtc is not installed, install it first via sudo apt install device-tree-compiler)


Step 3: Move the Overlay into Boot Storage

Move your freshly compiled overlay file to your Orange Pi overlay management folder:

sudo cp i2c-fastmode.dtbo /boot/overlay-user/

(If the folder /boot/overlay-user/ does not exist on your configuration, check /boot/dtb/allwinner/overlay/ instead).

Step 4: Activating the Overlay via Environment Configuration

Open your environment boot profile file:

sudo nano /boot/orangepiEnv.txt

Look for the line that starts with user_overlays=. If it is missing, create it. Add your overlay name to the parameter:

user_overlays=i2c-fastmode

(If you are already applying other user overlays, separate their names with spaces)
Save the file (Ctrl+O, Enter) and exit (Ctrl+X). Reboot your hardware for the change to take effect:

sudo reboot


Step 5: Verify Your Frequencies

Once the Orange Pi finishes rebooting, you can run a quick test using the kernel debug attributes to check if your clock rate increased:

echo 0x$(sudo xxd /sys/class/i2c-adapter/i2c-3/of_node/clock-frequency | cut -f 2,3 -d ' ' --output-delimiter='') | xargs printf "%d Hz\n"

(Swap out i2c-3 for whatever bus index number your target hardware resides on).

If successful, the terminal will print 400000 Hz.



