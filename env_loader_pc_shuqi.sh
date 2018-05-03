#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/shuqi/barc/workspace/devel/setup.bash
#export PATH=$PATH:/home/ugo/julia/bin/julia
export ROS_IP=192.168.10.144
export ROS_MASTER_URI=http://192.168.10.144:11311
exec "$@"
