# UR5 control with Xbox360 joystick

To enable the Xbox joystick, first connect it via USB to the PC. Then run the following lines on the terminal:

	$ sudo xboxdrv --silent --detach-kernel-driver

Now we can run the ROS launch file:

	$ roslaunch ur5_joy_control ur5_joy_control.launch
	
The `joy_node` node will publish messages of type `sensor_msgs/Joy` to the topic `/joy`, consisting of a header, a float32 *axis* array and an int32 *buttons* array. Values go from -1.0 to 1.0.

