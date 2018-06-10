#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/ugo/GitHub/barc/workspace/devel/setup.bash
export DISPLAY=:0.0
export ROS_IP=192.168.10.157
export ROS_MASTER_URI=http://192.168.10.147:11311
exec "$@"
