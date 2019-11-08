# ur5_ros_arduino

Arduino code with ROS for Aleks' sensors.

If any msg is changed in the package, and after *catkin_make* is run, it should be built for Arduino as well:

    $ cd ~/Arduino/libraries
    $ rm -r ros_lib
    $ rosrun rosserial_arduino make_libraries.py .
  
These lines will rebuild all ROS libraries for Arduino, including the header files for your new messages.

## Run

    $ rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
    
The baud rate is defined inside ```arduino_code.ino```, and needs to match the argument *_baud*. This node will make the Arduino Mega publish to the topic */pressure* the data obtained from the sensors.

The node ```ros_ard.cpp``` writes in a .csv file this data.
