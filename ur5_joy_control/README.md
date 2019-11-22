# UR5 control with Xbox360 joystick

To enable the Xbox joystick, first connect it via USB to the PC. Then run the following lines on the terminal:

	$ sudo xboxdrv --silent --detach-kernel-driver
	$ rosrun joy joy\_node (\_dev\_name:=/dev/input/js0)
