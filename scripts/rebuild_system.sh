# rebuild ROS workspace
cd ~/barc/workspace
rm -rf devel/ build/
catkin_make clean
catkin_make

# rebuild arduino libraries
cd ~/sketchbook/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
cd

# reflash arduino
if ! hash ano 2>/dev/null; then
    cd ~/barc/scripts/
    source install_apps.sh
fi    
cd ~/barc/arduino/.arduino_nano328_node
ano clean
ano build -m nano328
ano upload -m nano328 -p /dev/ttyUSB0
roscd barc;
