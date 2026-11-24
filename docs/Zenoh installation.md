
# rmw_zenoh_cpp


1. Install rmw_zenoh_cpp in robot

sudo apt update
sudo apt install ros-jazzy-rmw-zenoh-cpp


2. Set environment variables

To use Zenoh is used by default between all nodes add RMW_IMPLEMENTATION in ~/.bashrc file:

echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> ~/.bashrc
source ~/.bashrc

Make sure that following line is before that in the same file

source /opt/ros/jazzy/setup.bash

3. Define Zenoh Router (rmw_zenohd) for the network

In order that Flutter app in tablet and external AI server can connect to robot over WiFi, make Zenoh router to listen at address 0.0.0.0, port 7447.


3.1. Copy default configuration:

mkdir -p ~/zenoh_config
cp /opt/ros/jazzy/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5 ~/zenoh_config/router_config.json5

3.2. Check listening address in file ~/zenoh_config/router_config.json5:

Make sure that listen block has tcp/0.0.0.0:7447 (Most Jazzy versions this is already default: tcp/[::]:7447, which listens both IPv4  and IPv6 addresses).

3.3. Switch ON shared memory to improve robot's internal communication speed:

This can be done either in configuration file or exporting environment variable:

export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'


4. Test installation

Open three terminals in robot and test that messages goes through Zenoh:

• In terminal 1 start Zenoh router
ros2 run rmw_zenoh_cpp rmw_zenohd

• In terminal 2 start listener
ros2 run demo_nodes_cpp listener

• In terminal 3 start sender
ros2 run demo_nodes_cpp talker

You should see "I heard: [Hello World: ...]" messages in terminal 2, when it works correctly

5. Start router automatic in robot's boot (Systemd)

Create background service for rmw_zenohd process:

sudo nano /etc/systemd/system/zenoh-router.service


Add following content in the file (Replace ubuntu robot's own username):
ini
[Unit]
Description=ROS 2 Zenoh Router
After=network.target

[Service]
Type=simple
User=ubuntu
Environment="RMW_IMPLEMENTATION=rmw_zenoh_cpp"
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && ros2 run rmw_zenoh_cpp rmw_zenohd"
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target


Take service in use:

sudo systemctl daemon-reload
sudo systemctl enable zenoh-router.service
sudo systemctl start zenoh-router.service


Kun olet saanut reitittimen pyörimään, haluatko seuraavaksi konfiguraation client-noodeille (esim. AI-palvelimen tai tabletin yhdistäminen robotin IP-osoitteeseen) vai koodipohjan safety-noodille, joka ottaa vastaan 8 tavun joystick-viestejä?