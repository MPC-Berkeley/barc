#!/bin/bash
source ~/barc/scripts/startup.sh
source /opt/ros/indigo/setup.bash
source ~/barc/workspace/devel/setup.bash
export PATH=$PATH:/home/odroid/julia
export ROS_IP=192.168.100.100
export ROS_MASTER_URI=http://192.168.100.152:11311
exec "$@"