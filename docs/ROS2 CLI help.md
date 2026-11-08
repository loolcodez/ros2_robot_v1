# Get the current statePermalink
ros2 lifecycle get /driver_node

# List available transitionsPermalink
ros2 lifecycle list /driver_node

# Configure the node
ros2 lifecycle set /driver_node configure

# Activate it
ros2 lifecycle set /driver_node activate

# Deactivate
ros2 lifecycle set /driver_node deactivate

# Cleanup (back to Unconfigured)
ros2 lifecycle set /driver_node cleanup

# Shutdown
ros2 lifecycle set /driver_node shutdown

# Watch state transitions in real time
ros2 topic echo /driver_node/transition_event


# Listen topics

# Websocket connection status change
ros2 topic echo /connection/status

# ODO meter
ros2 topic echo /wheel_encoders

# Proximity sensors to Nav2
ros2 topic echo /tof/front_left
ros2 topic echo /tof/front_right
ros2 topic echo /tof/back_left
ros2 topic echo /tof/back_right

# IMU data
ros2 topic echo /imu/data_raw

# Power status
ros2 topic echo /power/alert
ros2 topic echo /power/shutdown
ros2 topic echo /power/battery_voltage

# Steering data
ros2 topic echo /move/direction
ros2 topic echo cmd_vel_manual
ros2 topic echo cmd_vel_in
ros2 topic echo cmd_vel_out
ros2 topic echo range <-- not used

# Lights
ros2 topic echo cmd_lights
ros2 topic echo lights_status

# Puhlish on a topic
ros2 topic pub --once /move/direction std_msgs/msg/String "{data: "backward"}"


# Listen sensor publish statistics
ros2 topic hz /tof/front_left
ros2 topic hz /tof/front_right
ros2 topic hz /tof/back_left
ros2 topic hz /tof/back_right


# How to Publish & Subscribe to a ROS2 Topic Using Command line
https://www.theconstruct.ai/publish-subscribe-ros2-topic

# ROS2 topic Command Line Tool – Debug ROS2 Topics From the Terminal
https://roboticsbackend.com/ros2-topic-cmd-line-tool-debug-ros2-topics-from-the-terminal

ros2 topic list

/camera/status
/camera_node/transition_event
/cmd_lights
/cmd_vel_in
/cmd_vel_manual
/cmd_vel_out
/connection/status
/diagnostics
/driver_node/transition_event
/imu/data_raw
/joint_states
/light_control_node/transition_event
/lights_status
/move/direction
/odom_wheel
/odometer_node/transition_event
/odometry/filtered
/parameter_events
/power/alert
/power/battery_status
/power/battery_voltage
/power/shutdown
/power_control_node/transition_event
/proximity_sensor_node/transition_event
/range
/robot_description
/rosout
/safety_control_node/transition_event
/set_pose
/tf
/tf_static
/tof/back_left
/tof/back_right
/tof/front_left
/tof/front_right
/websocket_node/transition_event
/wheel_encoders

ros2 topic echo /odometry/filtered --once

ros2 topic hz /odometry

ros2 topic echo /odometry/filtered --field twist.twist


ros2 topic echo /wheel_encoders