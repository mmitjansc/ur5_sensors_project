# UR5 control with Xbox360 joystick

To enable the Xbox joystick, first connect it via USB to the PC. Then run the following lines on the terminal:

	$ sudo xboxdrv --silent --detach-kernel-driver

If a joystick different than `js0` is set, we need to tell ROS which joystick to use:

	$ rosparam set joy_node/dev "/dev/input/jsX"

Now we can run the ROS joy node:

	$ rosrun joy joy_node
	
The `joy_node` node will publish messages of type `sensor_msgs/Joy` to the topic `/joy`, consisting of a header, a float32 *axis* array and an int32 *buttons* array. All values go from -1.0 to 1.0.

