# ur5_ros_arduino

Arduino code with ROS for Aleks' sensors.

If any msg is changed in the package, and after `catkin_make` is run, it should be built for Arduino as well:

    $ cd ~/Arduino/libraries
    $ rm -r ros_lib
    $ rosrun rosserial_arduino make_libraries.py .
  
These lines will rebuild all ROS libraries for Arduino, including the header files for your new messages.

## Run

    $ rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=50500
    
The baud rate is defined inside `arduino_code.ino`, and needs to match the argument `_baud`. This node will make the Arduino Mega publish to the topic */pressure* the data obtained from the sensors.

Note that the value for the `_baud` parameter has been set to closely match Robotiq's force sensor publishing rate (~62 Hz). This value was obtained empirically, as the publishing rate depends both on the baud rate of the Arduino and the loop rate of the code.

The node `ros_ard.cpp` writes in a .csv file the obtained data.
