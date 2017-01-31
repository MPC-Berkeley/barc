# rebuild ROS workspace
cd ~/barc/workspace
rm -rf devel/ build/
catkin_make clean
catkin_make

# configure ROS environment
source /home/odroid/barc/workspace/devel/setup.bash

# rebuild arduino libraries
cd ~/sketchbook/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
cd

# download and install ano (arduino) utility if not already installed 
# check if arduino libraries connected
if ! hash ano 2>/dev/null; then
    cd ~/barc/scripts/
    source install_apps.sh
fi

# flash arduino if connected
if [ -e /dev/ttyUSB0 ]; then
    cd ~/barc/arduino/.arduino_nano328_node
    cp ~/barc/arduino/arduino_nano328_node/arduino_nano328_node.ino src/
    ano clean
    ano build -m nano328
    ano upload -m nano328 -p /dev/ttyUSB0
fi

# navegate back to barc package
roscd barc;
