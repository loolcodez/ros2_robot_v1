TODO:

Joystick:
- Joystick should re-activate websocket connection when screen is re-opened if connection was open
- There could be configurable time when re-open is applied
- Joystick should be able to switch on/off driving lights
- Fix joystick picture clipping problem.
- Joystick should should send basic configure and activate and deactivate messages
- Add activation/deactivation of self drive mode

UTs:
- Fix unit tests to work with lifecycle management
- Add more unit tests

Robot control:
- Add Robot control node
- It should subscribe activate and deactivate messages and have complete control of robot state inside the robot

Camera:
- Complete camera node
- Make it able to send images to AI
- Add AI in loop

Light control:
- Light control should subscribe that and switch off lights when websocket connection closes if not in self drive mode
- Add AI controlled light switching.

Navigation:
- Make ROS2 topics support 3rd part nodes
- Add localization node
- Add Nav2 node
- Add anything needed in between for self drive!
- Make robot driving in its own
