
.PHONY: build run clean

build: clean
	bash -lc "source /opt/ros/$(ROS_DISTRO)/setup.bash && colcon build --symlink-install"

run:
	bash -lc "source /opt/ros/$(ROS_DISTRO)/setup.bash && source install/setup.bash && ros2 launch robot_bringup robot.launch.py"

clean:
	rm -rf build install log
