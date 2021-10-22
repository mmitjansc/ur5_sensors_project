# UR5 control with Xbox360 joystick

To enable the Xbox joystick, first connect it via USB to the PC. Then run the following lines on the terminal:

	$ sudo xboxdrv --silent --detach-kernel-driver

Now we can run the ROS launch file:

	$ roslaunch ur5_joy_control ur5_launch.launch #To "bring up" the UR5
	$ roslaunch ur5_joy_control navigate.launch
	
The `joy_node` node will publish messages of type `sensor_msgs/Joy` to the topic `/joy`, consisting of a header, a float32 *axis* array and an int32 *buttons* array. Values go from -1.0 to 1.0.

There is the possibility to create workspace limits consisting of overlapped boxes. To activate this
function, run the following file:

	$ roslaunch ur5_joy_control set_limits.launch
	
By following the instructions on the screen, move the robot to the 4 positions of the base of each workspace box, and then set the hight of the box. Repeat this for as many boxes as needed. 

Then, to control the robot with the set workspace, launch the following file:

	$ roslaunch ur5_joy_control navigate_with_limits.launch
