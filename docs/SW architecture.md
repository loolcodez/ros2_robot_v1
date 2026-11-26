
# Robot states

 UNCONFIGURED
  - Robot is waiting to be configured

 IDLE
  - Robot is configured but not actively observing surrounding environment
  - Robot does not send proximity sensor data to Nav2 packet
  - Robot cannot send pictures to AI

 ACTIVE
  - Robot is actively observing surrounding environment but it is not able to run
  - Robot sends proximity sensor data to Nav2 packet
  - Robot can send pictures to AI from front or back camera

 RUNNING
  - Robot is actively observing surrounding environment and able to run but not necessary moving
  - Robot sends proximity sensor data to Nav2 packet
  - Robot sends odometer data to localization packet
  - Robot sends IMU sensor data to localization packet
  - Robot sends pictures to AI from camera facing move direction

 ERROR
  - Robot is stopped but it still can be observing surrounding environment
  - Robot can send proximity sensor data to Nav2 packet
  - Robot can send odometer data to localization packet
  - Robot can send IMU sensor data to localization packet
  - Robot can send pictures to AI from camera facing to move direction


# Self made ROS2 packages
- Robot_bringup
- Websocket
- Safety
- ProximitySensor
- PowerControl
- ODOMeter
- LightControl
- LCManager
- Driver
- Camera

# Self made ROS2 Nodes
- WebsocketNode
- SafetyNode
- ProximitySensorNode
- PowerControlNode
- ODOMeterNode
- LightControlNode
- LCManagerNode
- DriverNode
- CameraNode

# Imported ROS2 packages
- Nav2
- robot_state_publisher
- ekf_robot_localization

# Node descriptions

## WebsocketNode
  This is old interface for simple Joystick control running in mobile

  Published topics:
   - robot/cmd_vel_manual
       Drive control command coming from Joystick is published

   - robot/connection/status
     Websocket connection status change is published

  Subscribed topics:
   - robot/power/battery_status
     Battery status is forwarder over websocket to mobile

##  SafetyControlNode

  Published topics:
   -  robot/cmd_vel
      Publishes robot drive commands consumed by driver node
    
   -  robot/move/direction
      Publishes robot move direction based on control data. Currently not used 

  Subscribed topics:
    - robot/distance-range
      Subscribes robot distance sensor data. Currently not use and not published in this topic

    - robot/cmd_vel_manual
      Manual control commands coming from Joystick node

    - control/cmd_vel
      Manual control commands coming from mobile Control

    - ai/cmd_vel
      AI control commands coming form AI server

    - robot/power/alert
      Power alerts coming from power control node

## ProximitySensorNode

  Published topics:
   - robot/tof/front_left
   - robot/tof/front_right
   - robot/tof/back_left
   - robot/tof/back_right
      Publishes ToF sensor data to robot_state_publisher and ekf_robot_localization nodes

  Subscribed topics:
    - None

## PowerControlNode

  Published topics:
   - robot/power/battery_status
     Publishes robotäs battery status. Joystick and Control are consumers

   - robot/power/shutdown
     Publishes robot shutdown down command. Driver is consumer

   - robot/power/alert
     Publishes power alert. Diver node is consumer

  Subscribed topics:
   - robot/connection/status
   - robot/power/battery_voltage

## ODOMeterNode

  Published topics:
  Subscribed topics:

## LightControlNode

  Published topics:
  Subscribed topics:

## LCManagerNode

  Published topics:
  Subscribed topics:

## DriverNode

  Published topics:
  Subscribed topics:

## CameraNode

  Published topics:
  Subscribed topics:

# Life cycle management

# Startup

# State control



