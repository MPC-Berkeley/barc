#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/max/barc/workspace/devel/setup.bash
export PATH=$PATH:/home/max/Downloads/julia-2e358ce975/bin
export ROS_IP=192.168.100.72
export ROS_MASTER_URI=http://192.168.100.72:11311
exec "$@"