
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

# Node descriptions

## WebsocketNode


  Published topics
  Subscribed topics

##  SafetyNode


  Published topics
  Subscribed topics

## ProximitySensorNode


  Published topics
  Subscribed topics

## PowerControlNode


  Published topics
  Subscribed topics

## ODOMeterNode


  Published topics
  Subscribed topics

## LightControlNode


  Published topics
  Subscribed topics

## LCManagerNode


  Published topics
  Subscribed topics

## DriverNode


  Published topics
  Subscribed topics

## CameraNode


  Published topics
  Subscribed topics





# Life cycle management

# Startup


# State control



